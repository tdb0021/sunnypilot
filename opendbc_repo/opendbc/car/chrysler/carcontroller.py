from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.chrysler import chryslercan
from opendbc.car.chrysler.values import RAM_DT, RAM_CARS, CUSW_CARS, CarControllerParams, ChryslerFlags
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_meas_steer_torque_limits
from opendbc.sunnypilot.car.chrysler.carcontroller_ext import CarControllerExt
from opendbc.sunnypilot.car.chrysler.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.chrysler.mads import MadsCarController
from opendbc.sunnypilot.car.chrysler.values_ext import ChryslerFlagsSP

LongCtrlState = structs.CarControl.Actuators.LongControlState


class CarController(CarControllerBase, MadsCarController, CarControllerExt, IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    MadsCarController.__init__(self)
    CarControllerExt.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.apply_torque_last = 0

    self.hud_count = 0
    self.last_lkas_falling_edge = 0
    self.lkas_control_bit_prev = False
    self.last_button_frame = 0

    self.packer = CANPacker(dbc_names[Bus.pt])
    self.params = CarControllerParams(CP)
    self.accel = 0.0

  def update(self, CC, CC_SP, CS, now_nanos):
    MadsCarController.update(self, CC, CC_SP, CS)
    can_sends = []

    lkas_active = CC.latActive and self.lkas_control_bit_prev

    # cruise buttons
    if (self.frame - self.last_button_frame) * DT_CTRL > 0.05:
      das_bus = 2 if self.CP.carFingerprint in RAM_CARS else 0

      # ACC cancellation. Stock ACC on the DASM bus engages from forwarded buttons;
      # while it is active, cancel it on that same bus. The limiter is the existing
      # one: more than 50 ms since the last button, so this cannot outrun it.
      stock_cancel = self.CP.openpilotLongitudinalControl and CS.stock_acc_active
      if CC.cruiseControl.cancel or stock_cancel:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.cruise_btns, CS.button_counter + 1, das_bus, cancel=True))

      # ACC resume from standstill
      elif CC.cruiseControl.resume:
        self.last_button_frame = self.frame
        can_sends.append(chryslercan.create_cruise_buttons(self.packer, CS.cruise_btns, CS.button_counter + 1, das_bus, resume=True))

    # HUD alerts
    if self.frame % 25 == 0:
      if CS.lkas_car_model != -1:
        can_sends.append(chryslercan.create_lkas_hud(self.packer, self.CP, lkas_active, CC.hudControl.visualAlert,
                                                     self.hud_count, CS.lkas_car_model, CS.auto_high_beam, self.mads))
        self.hud_count += 1

    # steering
    if self.frame % self.params.STEER_STEP == 0:

      # TODO: can we make this more sane? why is it different for all the cars?
      lkas_control_bit = self.lkas_control_bit_prev
      if self.CP_SP.flags & ChryslerFlagsSP.NO_MIN_STEERING_SPEED or self.CP.carFingerprint in RAM_DT:
        lkas_control_bit = CarControllerExt.get_lkas_control_bit(self, CS, CC, lkas_control_bit)
      elif CS.out.vEgo > self.CP.minSteerSpeed:
        lkas_control_bit = True
      elif self.CP.flags & ChryslerFlags.HIGHER_MIN_STEERING_SPEED:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 3.0):
          lkas_control_bit = False
      elif self.CP.carFingerprint in RAM_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 0.5):
          lkas_control_bit = False
      elif self.CP.carFingerprint in CUSW_CARS:
        if CS.out.vEgo < (self.CP.minSteerSpeed - 2.0):
          lkas_control_bit = False

      # EPS faults if LKAS re-enables too quickly
      lkas_control_bit = lkas_control_bit and (self.frame - self.last_lkas_falling_edge > 200)

      if not lkas_control_bit and self.lkas_control_bit_prev:
        self.last_lkas_falling_edge = self.frame
      self.lkas_control_bit_prev = lkas_control_bit

      # steer torque
      new_torque = int(round(CC.actuators.torque * self.params.STEER_MAX))
      apply_torque = apply_meas_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorqueEps, self.params)
      if not lkas_active or not lkas_control_bit:
        apply_torque = 0
      self.apply_torque_last = apply_torque

      can_sends.append(chryslercan.create_lkas_command(self.packer, self.CP, int(apply_torque), lkas_control_bit))

    if self.frame % 10 == 0 and self.CP.carFingerprint not in (RAM_CARS | CUSW_CARS):
      can_sends.append(MadsCarController.create_lkas_heartbit(self.packer, CS.lkas_heartbit, self.mads))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame))

    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self._long_control_messages(CC, CS))

    self.frame += 1

    new_actuators = CC.actuators.as_builder()
    new_actuators.torque = self.apply_torque_last / self.params.STEER_MAX
    new_actuators.torqueOutputCan = self.apply_torque_last
    new_actuators.accel = self.accel

    return new_actuators, can_sends

  def _long_control_messages(self, CC, CS):
    p = CarControllerParams
    gas_override = CS.out.gasPressed and CC.enabled and not CS.out.brakePressed
    # No stock ACC stop was observed. Below ~3.6 kph, drop the command instead of
    # creeping or holding with the untested standstill bits.
    handback = CS.out.vEgo < p.LOW_SPEED_HANDBACK
    if gas_override:
      # ACC stays engaged. The request parks near -90 Nm and the max / fuel-shutoff
      # bits drop, so the ECM follows the pedal instead of our torque number.
      self.accel = 0.0
      active = True
      braking = False
      torque = p.GAS_OVERRIDE_TORQUE
      decel = p.INACTIVE_DECEL
      torque_max = False
      fuel_shutoff = False
      gr_max = 0
    elif (not CC.longActive) or CS.out.brakePressed or handback:
      self.accel = 0.0
      active = False
      braking = False
      torque = 0.0
      decel = p.INACTIVE_DECEL
      torque_max = False
      fuel_shutoff = False
      gr_max = 0
    else:
      accel = min(max(float(CC.actuators.accel), p.ACCEL_MIN), p.ACCEL_MAX)
      if CC.actuators.longControlState == LongCtrlState.stopping:
        accel = min(accel, p.BRAKE_ENTER_ACCEL)
      braking = accel <= p.BRAKE_ENTER_ACCEL
      self.accel = accel
      active = True
      torque_max = True
      fuel_shutoff = True
      if braking:
        # Stock keeps about -80 Nm of engine request while ACC_DECEL is active.
        torque = p.BRAKE_TORQUE
        decel = self.accel
        gr_max = 4 if self.accel <= p.HARD_BRAKE_DECEL else 0
      else:
        torque = p.TORQUE_FROM_ACCEL * accel + p.TORQUE_OFFSET
        torque = min(max(torque, p.MIN_TORQUE_CMD), p.MAX_TORQUE_CMD)
        decel = p.INACTIVE_DECEL
        gr_max = 0

    sends = []
    # DAS_3 and DAS_5 are 50 Hz on this truck. DAS_3's counter is the real one.
    if self.frame % 2 == 0:
      sends.append(chryslercan.create_das_3(self.packer, active, torque, decel, braking,
                                            torque_max, fuel_shutoff, gr_max, CS.stock_acc_available))
      sends.append(chryslercan.create_das_5(self.packer, CS.out.vEgo))
    # DAS_4 was ~17 Hz in the log. 10 Hz is enough for the set-speed display.
    if self.frame % 10 == 0:
      # 2020 HD ACC_STATE: 0 off, 3 available, 4 engaged.
      if active:
        acc_state = 4
      elif CC.enabled:
        acc_state = 3
      else:
        acc_state = 0
      sends.append(chryslercan.create_das_4(self.packer, acc_state, CC.hudControl.setSpeed))
    return sends
