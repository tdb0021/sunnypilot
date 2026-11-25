from opendbc.car import structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.crc import CRC8J1850
from opendbc.car.chrysler.values import CUSW_CARS, RAM_CARS, CarControllerParams


def _display_speed(set_speed, conversion):
  return int(max(0, min(250, round(set_speed * conversion))))

VisualAlert = structs.CarControl.HUDControl.VisualAlert


def create_lkas_hud(packer, CP, lkas_active, hud_alert, hud_count, car_model, auto_high_beam, mads):
  # LKAS_HUD - Controls what lane-keeping icon is displayed

  # == Color ==
  # 0 hidden?
  # 1 white
  # 2 green
  # 3 ldw

  # == Lines ==
  # 03 white Lines
  # 04 grey lines
  # 09 left lane close
  # 0A right lane close
  # 0B left Lane very close
  # 0C right Lane very close
  # 0D left lane cross
  # 0E right lane cross

  # == Alerts ==
  # 7 Normal
  # 6 lane departure place hands on wheel

  if mads.enable_mads:
    color = 2 if lkas_active else 1 if mads.paused else 0
  else:
    color = 2 if lkas_active else 1
  lines = 3 if lkas_active else 0
  alerts = 7 if lkas_active else 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1

  if hud_alert in (VisualAlert.ldw, VisualAlert.steerRequired):
    color = 4
    lines = 0
    alerts = 6

  values = {
    "LKAS_ICON_COLOR": color,
    "CAR_MODEL": car_model,
    "LKAS_LANE_LINES": lines,
    "LKAS_ALERTS": alerts if CP.carFingerprint not in CUSW_CARS else 0,  # TODO: HUD behavior not yet understood for CUSW
  }

  if CP.carFingerprint in RAM_CARS:
    values['AUTO_HIGH_BEAM_ON'] = auto_high_beam

  return packer.make_can_msg("DAS_6", 0, values)


def create_lkas_command(packer, CP, apply_torque, lkas_control_bit):
  # LKAS_COMMAND Lane-keeping signal to turn the wheel
  enabled_val = 2 if CP.carFingerprint in RAM_CARS else 1
  values = {
    "STEERING_TORQUE": apply_torque,
    "LKAS_CONTROL_BIT": enabled_val if lkas_control_bit else 0,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_das_3(packer, acc_active, torque, decel, braking, torque_max=None, fuel_shutoff=None, gr_max_req=0, acc_available=False):
  # ENGINE_TORQUE_REQUEST is absolute Nm. It is negative while coasting and while braking.
  # ACC_DECEL uses 3.9943 m/s^2 (raw 4093) as the no-brake value.
  # torque_max and fuel_shutoff follow ACC_ACTIVE while we are commanding, and both drop
  # on a gas override (ACC stays active, the request sits near -90 Nm). Stop/go stays 0.
  # ACC_AVAILABLE tracks cruise main. The ECM ignores torque and decel when it is 0.
  if torque_max is None:
    torque_max = bool(acc_active)
  if fuel_shutoff is None:
    fuel_shutoff = bool(acc_active)
  values = {
    "ACC_ACTIVE": 1 if acc_active else 0,
    "ACC_AVAILABLE": 1 if acc_available else 0,
    "ENGINE_TORQUE_REQUEST": torque,
    "ENGINE_TORQUE_REQUEST_MAX": 1 if torque_max else 0,
    "ACC_DECEL": decel,
    "ACC_DECEL_REQ": 1 if braking else 0,
    "DISABLE_FUEL_SHUTOFF": 1 if fuel_shutoff else 0,
    "GR_MAX_REQ": gr_max_req,
  }
  return packer.make_can_msg("DAS_3", 0, values)


def create_das_4(packer, acc_state, set_speed):
  # Cluster set speed lives here. On the 2020 HD, ACC_STATE is 0 off, 3 available, 4 engaged.
  # FCW_BRAKE_DISABLED is 1 in every logged frame. FCW_BRAKE_ENABLED stays 0.
  values = {
    "ACC_STATE": acc_state,
    "ACC_SET_SPEED_KPH": _display_speed(set_speed, CV.MS_TO_KPH),
    "ACC_SET_SPEED_MPH": _display_speed(set_speed, CV.MS_TO_MPH),
    "FCW_BRAKE_ENABLED": 0,
    "FCW_BRAKE_DISABLED": 1,
  }
  return packer.make_can_msg("DAS_4", 0, values)


def create_das_5(packer, vehicle_speed):
  # Wheel torque is not an actuation path. Stock sends raw 0 (bytes 4-5 = 00 00).
  # SET_SPEED_KPH follows vehicle speed on this truck; the cruise set point is DAS_4.
  # FCW_STATE is 1 and FCW_DISTANCE is 3 in every logged frame. There is no Chrysler
  # CRC: the counter stays 0 and the last byte stays 0xFF.
  values = {
    "WHEEL_TORQUE_REQUEST": CarControllerParams.WHEEL_TORQUE_INACTIVE,
    "WHEEL_TORQUE_REQUEST_ACTIVE": 0,
    "FCW_STATE": 1,
    "FCW_DISTANCE": 3,
    "SET_SPEED_KPH": _display_speed(vehicle_speed, CV.MS_TO_KPH),
    "COUNTER": 0,
  }
  addr, dat, bus = packer.make_can_msg("DAS_5", 0, values)
  dat = bytearray(dat)
  dat[6] = 0
  dat[7] = 0xFF
  return addr, bytes(dat), bus


def create_cruise_buttons(packer, cruise_msg, frame, bus, cancel=False, resume=False, accel=False, decel=False):
  values = {
    "ACC_Cancel": cancel,
    "ACC_Resume": resume,
    "ACC_Accel": accel,
    "ACC_Decel": decel,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg(cruise_msg, bus, values)


def chrysler_checksum(address: int, sig, d: bytearray) -> int:
  # standard CRC-8 SAE J1850 (poly 0x1D, init 0xFF, final XOR 0xFF)
  crc = 0xFF
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  return crc ^ 0xFF


def fca_giorgio_checksum(address: int, sig, d: bytearray) -> int:
  crc = 0
  for i in range(len(d) - 1):
    crc ^= d[i]
    crc = CRC8J1850[crc]
  if address == 0xDE:
    return crc ^ 0x10
  elif address == 0x106:
    return crc ^ 0xF6
  elif address == 0x122:
    return crc ^ 0xF1
  else:
    return crc ^ 0x0A
