import unittest

from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.chrysler.interface import CarInterface
from opendbc.car.chrysler.values import CAR, DBC, CarControllerParams, ChryslerSafetyFlags

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

  def test_long_tuning_only_when_openpilot_long(self):
    p = CarControllerParams
    pid_accel_max = (p.MAX_TORQUE_CMD - p.TORQUE_OFFSET) / p.TORQUE_FROM_ACCEL
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      with self.subTest(candidate=candidate, alpha_long=False):
        CP, CP_SP = _params(candidate, alpha_long=False)
        self.assertEqual(list(CP.longitudinalTuning.kiV), [0.])
        self.assertEqual(CarInterface.get_pid_accel_limits(CP, CP_SP, 20., 25.), (p.ACCEL_MIN, p.ACCEL_MAX))
      with self.subTest(candidate=candidate, alpha_long=True):
        CP, CP_SP = _params(candidate, alpha_long=True)
        self.assertEqual(list(CP.longitudinalTuning.kiBP), [0.])
        self.assertEqual(len(CP.longitudinalTuning.kiV), 1)
        self.assertAlmostEqual(CP.longitudinalTuning.kiV[0], 0.3)
        accel_min, accel_max = CarInterface.get_pid_accel_limits(CP, CP_SP, 20., 25.)
        self.assertEqual(accel_min, p.ACCEL_MIN)
        self.assertAlmostEqual(accel_max, pid_accel_max)
        self.assertLess(accel_max, p.ACCEL_MAX)

    for candidate in (CAR.CHRYSLER_PACIFICA_2018, CAR.JEEP_GRAND_CHEROKEE):
      with self.subTest(candidate=candidate):
        CP, _ = _params(candidate, alpha_long=True)
        self.assertEqual(list(CP.longitudinalTuning.kiV), [0.])

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

  def test_op_long_ignores_stock_acc_active(self):
    # Stock ACC on the camera bus can show ACC_ACTIVE because buttons are still
    # forwarded. openpilot longitudinal must not report that as cruise enabled.
    for candidate in (CAR.RAM_1500_5TH_GEN, CAR.RAM_HD_5TH_GEN):
      packer = CANPacker(DBC[candidate][Bus.pt])
      frames = []
      for counter in (0, 1):
        frames.append(packer.make_can_msg("DAS_3", 2, {
          "ACC_ACTIVE": 1,
          "ACC_AVAILABLE": 1,
          "COUNTER": counter,
        }))
      for alpha_long in (False, True):
        with self.subTest(candidate=candidate, alpha_long=alpha_long):
          CP, CP_SP = _params(candidate, alpha_long=alpha_long)
          CI = CarInterface(CP, CP_SP)
          CI.update([])
          CS, _ = CI.update([(0, frames)])
          self.assertTrue(CS.cruiseState.available)
          self.assertEqual(CS.cruiseState.enabled, not alpha_long)
          self.assertTrue(CI.CS.stock_acc_active)

  def _cruise_step(self, CI, packer, state):
    state["ctr"] = (state["ctr"] + 1) & 0xF
    das = packer.make_can_msg("DAS_3", 2, {
      "ACC_AVAILABLE": int(state["available"]),
      "ACC_ACTIVE": int(state["active"]),
      "COUNTER": state["ctr"],
    })
    esp = packer.make_can_msg("ESP_1", 0, {
      "ACC_Enabled": int(state["enabled"]),
      "COUNTER": state["ctr"],
    })
    CS, _ = CI.update([(0, [das, esp])])
    return CS

  def test_available_debounce_only_when_openpilot_long(self):
    packer = CANPacker(DBC[CAR.RAM_HD_5TH_GEN][Bus.pt])

    # Stock long reports ACC_AVAILABLE on the frame it changes.
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=False)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    state = {"ctr": 0, "available": 1, "active": 1, "enabled": 1}
    CS = self._cruise_step(CI, packer, state)
    self.assertTrue(CS.cruiseState.available)
    state["available"] = 0
    CS = self._cruise_step(CI, packer, state)
    self.assertFalse(CS.cruiseState.available)
    self.assertTrue(CS.cruiseState.enabled)

    # openpilot long holds available through a stock main-off until it has lasted
    # more than 0.5 s and ESP_1 ACC_Enabled is also 0.
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    state = {"ctr": 0, "available": 1, "active": 0, "enabled": 1}
    CS = self._cruise_step(CI, packer, state)
    self.assertTrue(CS.cruiseState.available)
    self.assertFalse(CI.CS.stock_acc_active)
    state["available"] = 0
    state["enabled"] = 0
    for _ in range(50):
      CS = self._cruise_step(CI, packer, state)
      self.assertTrue(CS.cruiseState.available)
    CS = self._cruise_step(CI, packer, state)
    self.assertFalse(CS.cruiseState.available)

    # ESP still reporting ACC enabled keeps available up past 0.5 s.
    state = {"ctr": 0, "available": 1, "active": 1, "enabled": 1}
    CS = self._cruise_step(CI, packer, state)
    self.assertTrue(CS.cruiseState.available)
    self.assertTrue(CI.CS.stock_acc_active)
    state["available"] = 0
    for _ in range(60):
      CS = self._cruise_step(CI, packer, state)
      self.assertTrue(CS.cruiseState.available)

  def test_stock_acc_cancel_rate_limit(self):
    # 0x23A present selects the standard HD buttons. An empty fingerprint uses the alt address.
    fingerprint = gen_empty_fingerprint()
    fingerprint[0][0x23A] = 8
    CP = CarInterface.get_params(CAR.RAM_HD_5TH_GEN, fingerprint, [], alpha_long=True, is_release=False, docs=False)
    CP_SP = CarInterface.get_params_sp(CP, CAR.RAM_HD_5TH_GEN, fingerprint, [], alpha_long=True, is_release_sp=False, docs=False)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    self.assertEqual(CI.CS.cruise_btns, "CRUISE_BUTTONS")
    CI.CS.stock_acc_active = True
    CC = structs.CarControl()
    sent_at = []
    for i in range(20):
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
      buttons = [msg for msg in sends if msg[0] == 0x23A]
      if not buttons:
        continue
      self.assertEqual(len(buttons), 1)
      addr, dat, bus = buttons[0]
      self.assertEqual(addr, 0x23A)
      self.assertEqual(bus, 2)
      self.assertEqual(dat[0] & 0x1, 1)
      sent_at.append(i)
    # Shared with the existing button path: (frame - last) * 0.01 > 0.05, so every 6 frames.
    self.assertEqual(sent_at, [6, 12, 18])

    CI.CS.stock_acc_active = False
    for _ in range(20):
      _, sends = CI.apply(CC.as_reader(), structs.CarControlSP(), 0)
      self.assertFalse(any(msg[0] == 0x23A for msg in sends))

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

  def test_acc_available_follows_cruise_main(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])
    CI.CS.out.vEgo = 25.0
    CC = structs.CarControl()
    CC.enabled = True
    CC.actuators.accel = 1.0
    CC.actuators.longControlState = LongCtrlState.pid

    for main_on, long_active in ((True, True), (False, True), (True, False), (False, False)):
      with self.subTest(main_on=main_on, long_active=long_active):
        CI.CS.out.cruiseState.available = main_on
        CC.longActive = long_active
        sig = self._das3(CI, CC)
        self.assertEqual(sig["ACC_AVAILABLE"], int(main_on))

  def test_controller_torque_and_brake(self):
    CP, CP_SP = _params(CAR.RAM_HD_5TH_GEN, alpha_long=True)
    CI = CarInterface(CP, CP_SP)
    CI.update([])

    CI.CS.out.vEgo = 25.0
    CC = structs.CarControl()
    CC.enabled = True
    CC.longActive = True
    # 0.2 m/s^2 is under the planner cap, so the request is the line, not the clip.
    CC.actuators.accel = 0.2
    CC.actuators.longControlState = LongCtrlState.pid
    sig = self._das3(CI, CC)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST"], 550.0 * 0.2 + 200.0)
    self.assertEqual(sig["ENGINE_TORQUE_REQUEST_MAX"], 1)
    self.assertEqual(sig["DISABLE_FUEL_SHUTOFF"], 1)
    self.assertEqual(sig["GR_MAX_REQ"], 0)
    self.assertAlmostEqual(sig["ACC_DECEL"], CarControllerParams.INACTIVE_DECEL, places=3)
    self.assertEqual(sig["ACC_DECEL_REQ"], 0)
    self.assertLessEqual(sig["ENGINE_TORQUE_REQUEST"], CarControllerParams.MAX_TORQUE_CMD)

    # At the 2.0 m/s^2 controller clip the line is 1300 Nm. The command cap holds it at 720.
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
    self.assertAlmostEqual(sig["ENGINE_TORQUE_REQUEST"], 550.0 * -0.1 + 200.0, delta=0.25)
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
