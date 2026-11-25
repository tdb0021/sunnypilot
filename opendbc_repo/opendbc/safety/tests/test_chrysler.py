#!/usr/bin/env python3
import unittest

from opendbc.car.chrysler.values import ChryslerSafetyFlags
from opendbc.sunnypilot.car.chrysler.values_ext import ChryslerSafetyFlagsSP
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
class TestChryslerSafety(common.CarSafetyTest, common.MotorTorqueSteeringSafetyTest):
  DBC = "chrysler_pacifica_2017_hybrid_generated"
  SAFETY_MODEL = CarParams.SafetyModel.chrysler

  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x2D9, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6, 0x2D9)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6, 0x2D9]}

  MAX_RATE_UP = 3
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [261]
  MAX_RT_DELTA = 112
  MAX_TORQUE_ERROR = 80

  LKAS_ACTIVE_VALUE = 1

  DAS_BUS = 0

  def _button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.DAS_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ACTIVE": enable}
    return self.packer.make_can_msg_safety("DAS_3", self.DAS_BUS, values)

  def _speed_msg(self, speed):
    values = {"SPEED_LEFT": speed, "SPEED_RIGHT": speed}
    return self.packer.make_can_msg_safety("SPEED_1", 0, values)

  def _user_gas_msg(self, gas):
    values = {"Accelerator_Position": gas}
    return self.packer.make_can_msg_safety("ECM_5", 0, values)

  def _user_brake_msg(self, brake):
    values = {"Brake_Pedal_State": 1 if brake else 0}
    return self.packer.make_can_msg_safety("ESP_1", 0, values)

  def _torque_meas_msg(self, torque):
    values = {"EPS_TORQUE_MOTOR": torque}
    return self.packer.make_can_msg_safety("EPS_2", 0, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"STEERING_TORQUE": torque, "LKAS_CONTROL_BIT": self.LKAS_ACTIVE_VALUE if steer_req else 0}
    return self.packer.make_can_msg_safety("LKAS_COMMAND", 0, values)

  def test_buttons(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)

      # resume/accel/decel only while controls allowed
      self.assertEqual(controls_allowed, self._tx(self._button_msg(resume=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(accel=True)))
      self.assertEqual(controls_allowed, self._tx(self._button_msg(decel=True)))

      # can always cancel
      self.assertTrue(self._tx(self._button_msg(cancel=True)))

      # invalid: more than one button pressed
      combos = [
        # 2 buttons
        {"cancel": True, "resume": True},
        {"cancel": True, "accel": True},
        {"cancel": True, "decel": True},
        {"resume": True, "accel": True},
        {"resume": True, "decel": True},
        {"accel": True, "decel": True},

        # 3 buttons
        {"cancel": True, "resume": True, "accel": True},
        {"cancel": True, "resume": True, "decel": True},
        {"cancel": True, "accel": True, "decel": True},
        {"resume": True, "accel": True, "decel": True},

        # all 4 buttons
        {"cancel": True, "resume": True, "accel": True, "decel": True},
      ]

      for combo in combos:
        with self.subTest(combo=combo):
          self.assertFalse(self._tx(self._button_msg(**combo)))

  def _lkas_button_msg(self, enabled):
    values = {"TOGGLE_LKAS": enabled}
    return self.packer.make_can_msg_safety("TRACTION_BUTTON", 0, values)

  def _ignore_frequency_check_msgs(self):
    return [self._lkas_button_msg(False)]


class TestChryslerRamDTSafety(TestChryslerSafety):
  DBC = "chrysler_ram_dt_generated"

  SAFETY_PARAM = ChryslerSafetyFlags.RAM_DT

  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA]}

  MAX_RATE_UP = 6
  MAX_RATE_DOWN = 6
  MAX_TORQUE_LOOKUP = [0], [350]

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_safety("Center_Stack_2", 0, values)
class TestChryslerRamHDSafety(TestChryslerSafety):
  DBC = "chrysler_ram_hd_generated"

  SAFETY_PARAM = ChryslerSafetyFlags.RAM_HD

  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276]}

  MAX_TORQUE_LOOKUP = [0], [361]
  MAX_RATE_UP = 14
  MAX_RATE_DOWN = 14
  MAX_RT_DELTA = 182

  DAS_BUS = 2

  LKAS_ACTIVE_VALUE = 2

  def _speed_msg(self, speed):
    values = {"Vehicle_Speed": speed}
    return self.packer.make_can_msg_safety("ESP_8", 0, values)

  def _lkas_button_msg(self, enabled):
    values = {"LKAS_Button": enabled}
    return self.packer.make_can_msg_safety("Center_Stack_2", 0, values)


class TestChryslerRamHDAltButtonSafety(TestChryslerRamHDSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23B, 2]]

  def setUp(self):
    libsafety_py.libsafety.set_current_safety_param_sp(ChryslerSafetyFlagsSP.RAM_HD_ALT_BUTTONS)
    super().setUp()

  def _button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.DAS_BUS, values)


class ChryslerLongitudinalSafetyBase:
  """openpilot longitudinal: engine torque for accel, ACC_DECEL (m/s^2) for brake.

  Stock ACC does not engage. Resume, accel and decel do, on the falling edge.
  Cancel, and the brake pedal, disengage. Gas is an override: controls_allowed stays
  set so lateral can continue. The torque request must then be inactive, or the
  stock coast placeholder (about -90 Nm, ACC still active, max and fuel-shutoff bits off).
  Positive torque is not combined with a friction-brake request. Negative torque is.
  """

  # -120 Nm is the safety floor. Deepest stock request was -98 Nm.
  MIN_ENGINE_TORQUE = -120.0
  MAX_ENGINE_TORQUE = 500.0
  INACTIVE_ENGINE_TORQUE = 0.0
  MIN_POSSIBLE_ENGINE_TORQUE = -500.0
  MAX_POSSIBLE_ENGINE_TORQUE = 1547.5

  MIN_DECEL = -3.5
  MAX_DECEL = 0.0
  # Raw 4093. 4.0 packs to 4094, which this truck does not send.
  INACTIVE_DECEL = 3.9943
  MIN_POSSIBLE_DECEL = -16.0
  MAX_POSSIBLE_DECEL = 4.01

  BUTTON_MSG = "CRUISE_BUTTONS"
  LONG_BUS = 0

  def test_disable_control_allowed_from_cruise(self):
    pass

  def test_enable_control_allowed_from_cruise(self):
    pass

  def test_cruise_engaged_prev(self):
    pass

  def _rx_button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_safety(self.BUTTON_MSG, 0, values)

  def _torque_msg(self, torque):
    commanding = torque != self.INACTIVE_ENGINE_TORQUE
    values = {
      "ENGINE_TORQUE_REQUEST": torque,
      "ENGINE_TORQUE_REQUEST_MAX": 1 if commanding else 0,
      "ACC_ACTIVE": 1 if commanding else 0,
      "ACC_DECEL": self.INACTIVE_DECEL,
      "ACC_DECEL_REQ": 0,
      "DISABLE_FUEL_SHUTOFF": 1 if commanding else 0,
    }
    return self.packer.make_can_msg_safety("DAS_3", self.LONG_BUS, values)

  def _decel_msg(self, decel):
    braking = decel != self.INACTIVE_DECEL
    values = {
      "ENGINE_TORQUE_REQUEST": self.INACTIVE_ENGINE_TORQUE,
      "ENGINE_TORQUE_REQUEST_MAX": 1 if braking else 0,
      "ACC_ACTIVE": 1 if braking else 0,
      "ACC_DECEL": decel,
      "ACC_DECEL_REQ": 1 if braking else 0,
      "DISABLE_FUEL_SHUTOFF": 1 if braking else 0,
    }
    return self.packer.make_can_msg_safety("DAS_3", self.LONG_BUS, values)

  def _das5_msg(self, wheel_torque=-7767, wheel_active=0, set_speed=0, fcw_state=1, fcw_distance=3, stock_tail=True):
    values = {
      "WHEEL_TORQUE_REQUEST": wheel_torque,
      "WHEEL_TORQUE_REQUEST_ACTIVE": wheel_active,
      "FCW_STATE": fcw_state,
      "FCW_DISTANCE": fcw_distance,
      "SET_SPEED_KPH": set_speed,
      "COUNTER": 0,
    }
    addr, dat, _ = self.packer.make_can_msg("DAS_5", self.LONG_BUS, values)
    dat = bytearray(dat)
    if stock_tail:
      dat[6] = 0
      dat[7] = 0xFF
    return libsafety_py.make_CANPacket(addr, self.LONG_BUS, bytes(dat))

  def _das4_msg(self, fcw_brake=0, acc_state=4, set_speed_kph=0, fcw_disabled=1):
    values = {
      "FCW_BRAKE_ENABLED": fcw_brake,
      "FCW_BRAKE_DISABLED": fcw_disabled,
      "ACC_STATE": acc_state,
      "ACC_SET_SPEED_KPH": set_speed_kph,
    }
    return self.packer.make_can_msg_safety("DAS_4", self.LONG_BUS, values)

  def test_torque_limits(self):
    self._generic_limit_safety_check(self._torque_msg, self.MIN_ENGINE_TORQUE, self.MAX_ENGINE_TORQUE,
                                     self.MIN_POSSIBLE_ENGINE_TORQUE, self.MAX_POSSIBLE_ENGINE_TORQUE,
                                     test_delta=1, inactive_value=self.INACTIVE_ENGINE_TORQUE)

  def test_decel_limits(self):
    self._generic_limit_safety_check(self._decel_msg, self.MIN_DECEL, self.MAX_DECEL,
                                     self.MIN_POSSIBLE_DECEL, self.MAX_POSSIBLE_DECEL,
                                     test_delta=0.1, inactive_value=self.INACTIVE_DECEL)

  def _das3(self, **kwargs):
    values = {
      "ENGINE_TORQUE_REQUEST": 0,
      "ENGINE_TORQUE_REQUEST_MAX": 0,
      "ACC_ACTIVE": 0,
      "ACC_DECEL": self.INACTIVE_DECEL,
      "ACC_DECEL_REQ": 0,
      "DISABLE_FUEL_SHUTOFF": 0,
      "GR_MAX_REQ": 0,
    }
    values.update(kwargs)
    return self.packer.make_can_msg_safety("DAS_3", self.LONG_BUS, values)

  def test_no_positive_torque_with_brake(self):
    # Positive torque stacked on ACC_DECEL can accelerate into a stop. Reject it.
    # Stock does send about -80 Nm together with the friction brake. Allow that.
    self.safety.set_controls_allowed(True)
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=100, ENGINE_TORQUE_REQUEST_MAX=1,
      ACC_DECEL=-1.0, ACC_DECEL_REQ=1, DISABLE_FUEL_SHUTOFF=1,
    )))
    self.assertTrue(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-80, ENGINE_TORQUE_REQUEST_MAX=1,
      ACC_DECEL=-2.5, ACC_DECEL_REQ=1, DISABLE_FUEL_SHUTOFF=1, GR_MAX_REQ=4,
    )))
    self.assertTrue(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-80, ENGINE_TORQUE_REQUEST_MAX=1,
      ACC_DECEL=-0.4, ACC_DECEL_REQ=1, DISABLE_FUEL_SHUTOFF=1, GR_MAX_REQ=0,
    )))
    # GR_MAX_REQ was 4 only while the hard brake was active, and never any other value.
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=100, ENGINE_TORQUE_REQUEST_MAX=1,
      DISABLE_FUEL_SHUTOFF=1, GR_MAX_REQ=4,
    )))
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-80, ENGINE_TORQUE_REQUEST_MAX=1,
      ACC_DECEL=-2.5, ACC_DECEL_REQ=1, DISABLE_FUEL_SHUTOFF=1, GR_MAX_REQ=2,
    )))

  def test_stock_decel_sentinel(self):
    # Stock no-brake value is 3.9943 m/s^2 (raw 4093). 4.0 packs to a different raw.
    for allowed in (True, False):
      self.safety.set_controls_allowed(allowed)
      self.assertTrue(self._tx(self._decel_msg(3.9943)))
      self.assertFalse(self._tx(self._decel_msg(4.0)))

  def test_stock_coast_torque_allowed(self):
    # Deepest stock request was -98 Nm. -120 is the floor. 937 Nm was a real kick
    # and stays outside the 500 Nm ceiling, as does the 532.5 Nm cancel blip.
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_msg(-98)))
    self.assertTrue(self._tx(self._torque_msg(-120)))
    self.assertFalse(self._tx(self._torque_msg(-121)))
    self.assertFalse(self._tx(self._torque_msg(532.5)))
    self.assertFalse(self._tx(self._torque_msg(937)))
    self.assertTrue(self._tx(self._torque_msg(480)))
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._torque_msg(-98)))

  def test_fuel_shutoff_bit_only_while_allowed(self):
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._torque_msg(152)))
    self.safety.set_controls_allowed(False)
    values = {
      "ENGINE_TORQUE_REQUEST": 0,
      "ACC_DECEL": self.INACTIVE_DECEL,
      "DISABLE_FUEL_SHUTOFF": 1,
    }
    self.assertFalse(self._tx(self.packer.make_can_msg_safety("DAS_3", self.LONG_BUS, values)))

  def test_gas_override_blocks_longitudinal(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(1))
    # Lateral engagement stays. The planner command does not: either a fully
    # inactive frame, or the stock override (ACC on, coast torque, max bit off).
    self.assertTrue(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_longitudinal_allowed())
    self.assertFalse(self._tx(self._torque_msg(100)))
    self.assertFalse(self._tx(self._decel_msg(-1.0)))
    self.assertTrue(self._tx(self._torque_msg(self.INACTIVE_ENGINE_TORQUE)))
    self.assertTrue(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-90, ENGINE_TORQUE_REQUEST_MAX=0,
      DISABLE_FUEL_SHUTOFF=0,
    )))
    self.assertTrue(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-50, ENGINE_TORQUE_REQUEST_MAX=0,
    )))
    self.assertTrue(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-120, ENGINE_TORQUE_REQUEST_MAX=0,
    )))
    # The max bit, fuel-shutoff disable, positive torque, and a friction brake
    # all drop during the override. The ECM follows the pedal, not this request.
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-90, ENGINE_TORQUE_REQUEST_MAX=1,
    )))
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-90, DISABLE_FUEL_SHUTOFF=1,
    )))
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=-49, ENGINE_TORQUE_REQUEST_MAX=0,
    )))
    self.assertFalse(self._tx(self._das3(
      ACC_ACTIVE=1, ENGINE_TORQUE_REQUEST=200, ENGINE_TORQUE_REQUEST_MAX=0,
    )))
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self.safety.get_longitudinal_allowed())
    self.assertTrue(self._tx(self._torque_msg(100)))

  def test_cancel_disengages(self):
    self.safety.set_controls_allowed(True)
    self._rx(self._rx_button_msg(cancel=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_enable_on_falling_edge(self):
    for kwargs in ({"resume": True}, {"accel": True}, {"decel": True}):
      with self.subTest(kwargs=kwargs):
        self.safety.set_controls_allowed(False)
        self._rx(self._rx_button_msg())
        self._rx(self._rx_button_msg(**kwargs))
        self.assertFalse(self.safety.get_controls_allowed())
        self._rx(self._rx_button_msg())
        self.assertTrue(self.safety.get_controls_allowed())

  def test_rising_edge_does_not_enable(self):
    self.safety.set_controls_allowed(False)
    self._rx(self._rx_button_msg())
    self._rx(self._rx_button_msg(resume=True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_stock_acc_does_not_enable(self):
    self._rx(self._pcm_status_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertFalse(self.safety.get_controls_allowed())

  def test_wheel_torque_inactive_only(self):
    self.safety.set_controls_allowed(True)
    # Stock inactive is raw 0 (-7767 Nm), not the 0 Nm encoding.
    self.assertTrue(self._tx(self._das5_msg(-7767, 0, set_speed=88)))
    self.assertFalse(self._tx(self._das5_msg(0, 0)))
    self.assertFalse(self._tx(self._das5_msg(100, 1)))
    self.assertFalse(self._tx(self._das5_msg(-7767, 1)))
    self.assertFalse(self._tx(self._das5_msg(-7767, 0, stock_tail=False)))
    self.assertFalse(self._tx(self._das5_msg(-7767, 0, fcw_state=0)))

  def test_fcw_brake_rejected(self):
    self.safety.set_controls_allowed(True)
    for state in (0, 3, 4):
      self.assertTrue(self._tx(self._das4_msg(0, acc_state=state, set_speed_kph=89)))
    self.assertFalse(self._tx(self._das4_msg(0, acc_state=2, set_speed_kph=89)))
    self.assertFalse(self._tx(self._das4_msg(1, acc_state=4, set_speed_kph=89)))
    self.assertFalse(self._tx(self._das4_msg(0, acc_state=4, fcw_disabled=0)))

  def test_unvalidated_das3_bits_rejected(self):
    self.safety.set_controls_allowed(True)
    for extra in ({"ACC_BRK_PREP": 1}, {"COLLISION_BRK_PREP": 1}, {"GR_MAX_REQ": 1},
                  {"ACC_STANDSTILL": 1}, {"ACC_GO": 1}):
      with self.subTest(extra=extra):
        values = {
          "ENGINE_TORQUE_REQUEST": 0,
          "ACC_DECEL": self.INACTIVE_DECEL,
          **extra,
        }
        self.assertFalse(self._tx(self.packer.make_can_msg_safety("DAS_3", self.LONG_BUS, values)))


class TestChryslerLongitudinalSafety(ChryslerLongitudinalSafetyBase, TestChryslerSafety):
  TX_MSGS = [[0x23B, 0], [0x292, 0], [0x2A6, 0], [0x2D9, 0], [0x1F4, 0], [0x1F5, 0], [0x271, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x292, 0x2A6, 0x2D9, 0x1F4, 0x1F5, 0x271)}
  FWD_BLACKLISTED_ADDRS = {2: [0x292, 0x2A6, 0x2D9, 0x1F4, 0x1F5, 0x271]}
  SAFETY_PARAM = ChryslerSafetyFlags.LONG


class TestChryslerRamDTLongitudinalSafety(ChryslerLongitudinalSafetyBase, TestChryslerRamDTSafety):
  TX_MSGS = [[0xB1, 2], [0xA6, 0], [0xFA, 0], [0x99, 0], [0xE8, 0], [0xA3, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0xA6, 0xFA, 0x99, 0xE8, 0xA3)}
  FWD_BLACKLISTED_ADDRS = {2: [0xA6, 0xFA, 0x99, 0xE8, 0xA3]}
  SAFETY_PARAM = ChryslerSafetyFlags.RAM_DT | ChryslerSafetyFlags.LONG


class TestChryslerRamHDLongitudinalSafety(ChryslerLongitudinalSafetyBase, TestChryslerRamHDSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23A, 2], [0x1F4, 0], [0x1F5, 0], [0x271, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x276, 0x275, 0x1F4, 0x1F5, 0x271)}
  FWD_BLACKLISTED_ADDRS = {2: [0x275, 0x276, 0x1F4, 0x1F5, 0x271]}
  SAFETY_PARAM = ChryslerSafetyFlags.RAM_HD | ChryslerSafetyFlags.LONG


class TestChryslerRamHDAltButtonLongitudinalSafety(TestChryslerRamHDLongitudinalSafety):
  TX_MSGS = [[0x275, 0], [0x276, 0], [0x23B, 2], [0x1F4, 0], [0x1F5, 0], [0x271, 0]]
  BUTTON_MSG = "CRUISE_BUTTONS_ALT"

  def setUp(self):
    libsafety_py.libsafety.set_current_safety_param_sp(ChryslerSafetyFlagsSP.RAM_HD_ALT_BUTTONS)
    super().setUp()

  def _button_msg(self, cancel=False, resume=False, accel=False, decel=False):
    values = {"ACC_Cancel": cancel, "ACC_Resume": resume, "ACC_Accel": accel, "ACC_Decel": decel}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.DAS_BUS, values)


if __name__ == "__main__":
  unittest.main()
