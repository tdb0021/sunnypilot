import unittest

from opendbc.can.parser import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, CarControllerParams, ChryslerSafetyFlags

LongCtrlState = structs.CarControl.Actuators.LongControlState


def _params(candidate, alpha_long):
  fingerprint = gen_empty_fingerprint()
  CP = CarInterface.get_params(candidate, fingerprint, [], alpha_long=alpha_long, is_release=False, docs=False)
  CP_SP = CarInterface.get_params_sp(CP, candidate, fingerprint, [], alpha_long=alpha_long, is_release_sp=False, docs=False)
  return CP, CP_SP


class TestChryslerLongitudinal(unittest.TestCase):
  def test_off_by_default(self):
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=False)
        self.assertTrue(CP.alphaLongitudinalAvailable)
        self.assertFalse(CP.openpilotLongitudinalControl)
        self.assertTrue(CP.pcmCruise)
        self.assertFalse(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def test_toggle_enables_ram_only(self):
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=True)
        self.assertTrue(CP.openpilotLongitudinalControl)
        self.assertFalse(CP.pcmCruise)
        self.assertEqual(CP.longitudinalActuatorDelay, 0.5)
        self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)
        self.assertEqual(CP.minEnableSpeed, -1.0)

    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.JEEP_GRAND_CHEROKEE):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=True)
        self.assertFalse(CP.alphaLongitudinalAvailable)
        self.assertFalse(CP.openpilotLongitudinalControl)
        self.assertFalse(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def test_ram_hd_safety_flag_includes_platform(self):
    CP, _ = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.RAM_HD)
    self.assertTrue(CP.safetyConfigs[0].safetyParam & ChryslerSafetyFlags.LONG)

  def _das3(self, CI, CC):
    # The first frame's counter does not match the parser's expected counter, so keep two.
    das3 = []
    for _ in range(4):
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
      das3.extend(msg for msg in sends if msg[0] == 0x1F4)
      if len(das3) >= 2:
        break
    self.assertGreaterEqual(len(das3), 2)
    parser = CANParser("chrysler_ram_hd_generated", [("DAS_3", 50)], 0)
    parser.update([(0, das3[-2:])])
    return parser.vl["DAS_3"]

  def test_controller_torque_and_brake(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])

    CI.CS.out.vEgo = 25.0
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.0
    CC.actuators.longControlState = LongCtrlState.pid
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 208.0 + 133.0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 1)
    self.assertEqual(sig["DISABLE_FUEL_SHUTOFF"], 1)
    self.assertEqual(sig["GR_MAX_REQ"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=3)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertLessEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE_CMD)

    # The blended line at 2.0 m/s^2 is 549 Nm. The command cap holds it at 480.
    CC.actuators.accel = 5.0
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE_CMD)
    self.assertLess(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE)

    CC.actuators.accel = -2.5
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.BRAKE_TORQUE)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 1)
    self.assertAlmostEqual(sig["ACC_DECEL"], -2.5, places=2)
    self.assertEqual(sig["ACC_DECEL_REQ"], 1)
    self.assertEqual(sig["GR_MAX_REQ"], 4)

    CC.actuators.accel = -1.0
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.BRAKE_TORQUE)
    self.assertEqual(sig["GR_MAX_REQ"], 0)
    self.assertEqual(sig["ACC_DECEL_REQ"], 1)

    # Above the friction-brake threshold the command stays on the torque line.
    CC.actuators.accel = -0.1
    sig = self._das3(CI, CC)
    self.assertAlmostEqual(sig["ENGINE_TORQUE_REQUEST"], 208.0 * -0.1 + 133.0, delta=0.25)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertEqual(sig["GR_MAX_REQ"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=3)

  def test_gas_override_keeps_acc_at_coast(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CI.CS.out.vEgo = 25.0
    CI.CS.out.gasPressed = True

    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = False
    CC.actuators.accel = 1.5
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ACC_ACTIVE"], 1)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.GAS_OVERRIDE_TORQUE)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 0)
    self.assertEqual(sig["DISABLE_FUEL_SHUTOFF"], 0)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertEqual(sig["GR_MAX_REQ"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=3)

  def test_brake_pedal_and_low_speed_handback(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CI.CS.out.vEgo = 25.0
    CI.CS.out.brakePressed = True

    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.5
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ACC_ACTIVE"], 0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 0)

    CI.CS.out.brakePressed = False
    CI.CS.out.vEgo = 0.5
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ACC_ACTIVE"], 0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 0)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=3)

  def test_das5_matches_stock_idle(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CI.CS.out.vEgo = 24.6  # ~55 mph
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 0.0
    _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
    das5 = [msg for msg in sends if msg[0] == 0x271]
    self.assertEqual(len(das5), 1)
    dat = das5[0][1]
    self.assertEqual(dat[0], 0x1C)   # FCW_STATE=1, FCW_DISTANCE=3
    self.assertEqual(dat[4:6], b"\x00\x00")  # wheel torque raw 0, not 0 Nm
    self.assertEqual(dat[6], 0)
    self.assertEqual(dat[7], 0xFF)

  def test_stock_long_does_not_send_das3(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=False)
    CI = CarInterface(CP, CP_SP)
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    CC.actuators.accel = 1.0
    _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
    self.assertFalse(any(msg[0] == 0x1F4 for msg in sends))
