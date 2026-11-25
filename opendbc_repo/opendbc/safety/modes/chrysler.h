#pragma once

#include "opendbc/safety/declarations.h"
#include "opendbc/safety/modes/chrysler_common.h"

// Chrysler Pacifica/Jeep addresses
#define CHRYSLER_EPS_2            0x220  // EPS driver input torque
#define CHRYSLER_ESP_1            0x140  // Brake pedal and vehicle speed
#define CHRYSLER_ESP_8            0x11C  // Brake pedal and vehicle speed
#define CHRYSLER_ECM_5            0x22F  // Throttle position sensor
#define CHRYSLER_DAS_3            0x1F4  // ACC state, engine-torque and decel from DASM
#define CHRYSLER_DAS_4            0x1F5  // ACC / FCW display from DASM
#define CHRYSLER_DAS_5            0x271  // ACC display and wheel-torque request from DASM
#define CHRYSLER_DAS_6            0x2A6  // LKAS HUD and auto headlight control from DASM
#define CHRYSLER_LKAS_COMMAND     0x292  // LKAS controls from DASM
#define CHRYSLER_CRUISE_BUTTONS   0x23B  // Cruise control buttons
#define CHRYSLER_CRUISE_BUTTONS_ALT  0x000  // Placeholder, does not exist
#define CHRYSLER_LKAS_HEARTBIT    0x2D9  // LKAS HEARTBIT from DASM
#define CHRYSLER_TRACTION_BUTTON  0x330  // Traction control button
#define CHRYSLER_Center_Stack_2   0x000  // Placeholder, does not exist

// RAM DT addresses
#define CHRYSLER_RAM_DT_EPS_2            0x31
#define CHRYSLER_RAM_DT_ESP_1            0x83
#define CHRYSLER_RAM_DT_ESP_8            0x79
#define CHRYSLER_RAM_DT_ECM_5            0x9D
#define CHRYSLER_RAM_DT_DAS_3            0x99
#define CHRYSLER_RAM_DT_DAS_4            0xE8
#define CHRYSLER_RAM_DT_DAS_5            0xA3
#define CHRYSLER_RAM_DT_DAS_6            0xFA
#define CHRYSLER_RAM_DT_LKAS_COMMAND     0xA6
#define CHRYSLER_RAM_DT_CRUISE_BUTTONS   0xB1
#define CHRYSLER_RAM_DT_CRUISE_BUTTONS_ALT  0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_DT_LKAS_HEARTBIT    0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_DT_TRACTION_BUTTON  0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_DT_Center_Stack_2   0x28A

// RAM HD addresses
#define CHRYSLER_RAM_HD_EPS_2            0x220
#define CHRYSLER_RAM_HD_ESP_1            0x140
#define CHRYSLER_RAM_HD_ESP_8            0x11C
#define CHRYSLER_RAM_HD_ECM_5            0x22F
#define CHRYSLER_RAM_HD_DAS_3            0x1F4
#define CHRYSLER_RAM_HD_DAS_4            0x1F5
#define CHRYSLER_RAM_HD_DAS_5            0x271
#define CHRYSLER_RAM_HD_DAS_6            0x275
#define CHRYSLER_RAM_HD_LKAS_COMMAND     0x276
#define CHRYSLER_RAM_HD_CRUISE_BUTTONS   0x23A
#define CHRYSLER_RAM_HD_CRUISE_BUTTONS_ALT  0x23B  // Cruise control buttons for Ram HD alt buttons
#define CHRYSLER_RAM_HD_LKAS_HEARTBIT    0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_HD_TRACTION_BUTTON  0x00  // Placeholder, does not exist
#define CHRYSLER_RAM_HD_Center_Stack_2   0x28A

#ifdef ALLOW_DEBUG
#define CHRYSLER_RAM_HD_COMMON_TX_MSGS                       \
  {CHRYSLER_RAM_HD_LKAS_COMMAND, 0, 8, .check_relay = true}, \
  {CHRYSLER_RAM_HD_DAS_6, 0, 8, .check_relay = true},
#endif

typedef enum {
  CHRYSLER_RAM_DT,
  CHRYSLER_RAM_HD,
  CHRYSLER_PACIFICA,  // plus Jeep
} ChryslerPlatform;
static ChryslerPlatform chrysler_platform;
static bool chrysler_longitudinal = false;
// Driver cruise buttons are on bus 0. Ram HD trucks use either 0x23A or 0x23B.
static uint32_t chrysler_cruise_button_addr = 0U;

#define CHRYSLER_ADDR(name) ((uint32_t)((chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_##name : \
                                        ((chrysler_platform == CHRYSLER_RAM_HD) ? CHRYSLER_RAM_HD_##name : CHRYSLER_##name)))

// Cruise-button bits in byte 0. Matches the DBC (ACC_Cancel/Accel/Decel/Resume).
#define CHRYSLER_BTN_CANCEL  1U
#define CHRYSLER_BTN_ACCEL   4U
#define CHRYSLER_BTN_DECEL   8U
#define CHRYSLER_BTN_RESUME  16U
#define CHRYSLER_BTN_ENABLE  (CHRYSLER_BTN_ACCEL | CHRYSLER_BTN_DECEL | CHRYSLER_BTN_RESUME)

// DAS_3 ENGINE_TORQUE_REQUEST: 13-bit Motorola at bit 4, 0.25 Nm, offset -500.
// 2000 = 0 Nm, sent when ACC is not engaged.
// 1520 = -120 Nm. Deepest stock request was -98 Nm; -120 leaves a little room.
// 1800 = -50 Nm, top of the gas-override coast band (stock sits near -90).
// 4000 = 500 Nm. Ram HD long uses 4960 = 740 Nm: (740 + 500) / 0.25.
// 937 Nm stock kicks stay outside both caps. The negative floor is unchanged.
#define CHRYSLER_TORQUE_INACTIVE      2000
#define CHRYSLER_TORQUE_MIN           1520
#define CHRYSLER_TORQUE_OVERRIDE_MAX  1800
#define CHRYSLER_TORQUE_MAX           4000
#define CHRYSLER_TORQUE_MAX_RAM_HD    4960
// DAS_3 ACC_DECEL: 12-bit Motorola at bit 19, 0.004885 m/s^2, offset -16.
// Packer: -3.5 -> 2559, 0.0 -> 3275.
// Stock "no brake" is raw 4093 = 3.9943 m/s^2, not the 4.0 that packs to 4094.
#define CHRYSLER_DECEL_MIN        2559
#define CHRYSLER_DECEL_MAX        3275
#define CHRYSLER_DECEL_INACTIVE   4093
// DAS_5 on this 2020 HD: wheel torque inactive is raw 0 (bytes 4-5 = 00 00),
// which decodes to -7767 Nm. FCW idle is FCW_STATE=1, FCW_DISTANCE=3 (byte 0 = 0x1C).
// Counter stays 0 and the last byte stays 0xFF. There is no Chrysler CRC on DAS_5.
#define CHRYSLER_DAS5_FCW_IDLE    0x1CU
#define CHRYSLER_DAS5_CHECK_BYTE  0xFFU

static uint8_t chrysler_get_counter(const CANPacket_t *msg) {
  // High nibble of the byte before the checksum. 8-byte msgs: byte 6. 3-byte cruise buttons: byte 1.
  int counter_byte = GET_LEN(msg) - 2;
  return (uint8_t)(msg->data[counter_byte] >> 4);
}

static void chrysler_rx_hook(const CANPacket_t *msg) {
  // Measured EPS torque
  if (msg_matches(msg, CHRYSLER_ADDR(EPS_2), 0U)) {
    int torque_meas_new = ((msg->data[4] & 0x7U) << 8) + msg->data[5] - 1024U;
    update_sample(&torque_meas, torque_meas_new);
  }

  // ACC main (DAS_3 bit 20, ACC_AVAILABLE) in both modes. MADS enables lateral on
  // the rising edge, so LONG mode has to see it too. Stock ACC_ACTIVE is only
  // trusted when openpilot is not the longitudinal controller.
  const unsigned int das_3_bus = (chrysler_platform == CHRYSLER_PACIFICA) ? 0U : 2U;
  if (msg_matches(msg, CHRYSLER_ADDR(DAS_3), das_3_bus)) {
    acc_main_on = GET_BIT(msg, 20U);
    if (!chrysler_longitudinal) {
      bool cruise_engaged = GET_BIT(msg, 21U);
      pcm_cruise_check(cruise_engaged);
    }
  }

  if (chrysler_longitudinal) {
    // Falling edge of resume/accel/decel enters, and only while cruise main is on,
    // matching openpilot. Cancel exits. Gas does not clear controls_allowed;
    // get_longitudinal_allowed() blocks actuation. Brake exits in generic_rx_checks.
    if (msg_matches(msg, chrysler_cruise_button_addr, 0U)) {
      const uint8_t raw_btn = msg->data[0];
      const bool cancel = (raw_btn & CHRYSLER_BTN_CANCEL) != 0U;
      const uint8_t cruise_button = cancel ? (uint8_t)CHRYSLER_BTN_CANCEL : (uint8_t)(raw_btn & CHRYSLER_BTN_ENABLE);

      if (cancel) {
        controls_allowed = false;
      }

      const bool falling_enable = !cancel &&
                                  (cruise_button != cruise_button_prev) &&
                                  ((cruise_button_prev == CHRYSLER_BTN_ACCEL) ||
                                   (cruise_button_prev == CHRYSLER_BTN_DECEL) ||
                                   (cruise_button_prev == CHRYSLER_BTN_RESUME));
      if (falling_enable && acc_main_on) {
        controls_allowed = true;
      }
      cruise_button_prev = cruise_button;
    }
  }

  // TODO: use the same message for both
  // update vehicle moving
  if ((chrysler_platform != CHRYSLER_PACIFICA) && msg_matches(msg, CHRYSLER_ADDR(ESP_8), 0U)) {
    vehicle_moving = ((msg->data[4] << 8) + msg->data[5]) != 0U;
  }
  if ((chrysler_platform == CHRYSLER_PACIFICA) && msg_matches(msg, 514U, 0U)) {
    int speed_l = (msg->data[0] << 4) + (msg->data[1] >> 4);
    int speed_r = (msg->data[2] << 4) + (msg->data[3] >> 4);
    vehicle_moving = (speed_l | speed_r) != 0;
  }

  // exit controls on rising edge of gas press
  if (msg_matches(msg, CHRYSLER_ADDR(ECM_5), 0U)) {
    gas_pressed = msg->data[0U] != 0U;
  }

  // exit controls on rising edge of brake press
  if (msg_matches(msg, CHRYSLER_ADDR(ESP_1), 0U)) {
    brake_pressed = ((msg->data[0U] & 0xFU) >> 2U) == 1U;
  }

  if ((chrysler_platform == CHRYSLER_PACIFICA) && (msg->bus == 0U) && (msg->addr == CHRYSLER_ADDR(TRACTION_BUTTON))) {
    mads_button_press = GET_BIT(msg, 53U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
  }

  if ((chrysler_platform != CHRYSLER_PACIFICA) && (msg->bus == 0U)) {
    if (msg->addr == CHRYSLER_ADDR(Center_Stack_2)) {
      mads_button_press = GET_BIT(msg, 57U) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
}

static bool chrysler_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits CHRYSLER_STEERING_LIMITS = {
    .max_torque = 261,
    .max_rt_delta = 112,
    .max_rate_up = 3,
    .max_rate_down = 3,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_DT_STEERING_LIMITS = {
    .max_torque = 350,
    .max_rt_delta = 112,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  const TorqueSteeringLimits CHRYSLER_RAM_HD_STEERING_LIMITS = {
    .max_torque = 361,
    .max_rt_delta = 182,
    .max_rate_up = 14,
    .max_rate_down = 14,
    .max_torque_error = 80,
    .type = TorqueMotorLimited,
  };

  bool tx = true;

  // STEERING
  if (msg->addr == CHRYSLER_ADDR(LKAS_COMMAND)) {
    int start_byte = (chrysler_platform == CHRYSLER_PACIFICA) ? 0 : 1;
    int desired_torque = ((msg->data[start_byte] & 0x7U) << 8) | msg->data[start_byte + 1];
    desired_torque -= 1024;

    const TorqueSteeringLimits limits = (chrysler_platform == CHRYSLER_PACIFICA) ? CHRYSLER_STEERING_LIMITS :
                                        (chrysler_platform == CHRYSLER_RAM_DT) ? CHRYSLER_RAM_DT_STEERING_LIMITS : CHRYSLER_RAM_HD_STEERING_LIMITS;

    bool steer_req = (chrysler_platform == CHRYSLER_PACIFICA) ? GET_BIT(msg, 4U) : (msg->data[3] & 0x7U) == 2U;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = false;
    }
  }

  // Longitudinal actuation on DAS_3. Two 2020 HD stock-ACC logs
  // (00000a3e--51571a2b07 and 00000a3f--f3c4bf6816):
  // coast and braking both use a negative torque request, friction braking is
  // ACC_DECEL down to about -2.75 m/s^2, and a gas pedal keeps ACC engaged while
  // the request sits near -90 Nm. Ram HD may command up to 740 Nm. Other
  // platforms stay at 500 Nm. A 937 Nm stock kick is still not allowed.
  if (msg->addr == CHRYSLER_ADDR(DAS_3)) {
    const int torque_max = (chrysler_platform == CHRYSLER_RAM_HD) ? CHRYSLER_TORQUE_MAX_RAM_HD : CHRYSLER_TORQUE_MAX;
    const LongitudinalLimits CHRYSLER_LONG_LIMITS = {
      .max_gas = torque_max,
      .min_gas = CHRYSLER_TORQUE_MIN,
      .inactive_gas = CHRYSLER_TORQUE_INACTIVE,
      .max_accel = CHRYSLER_DECEL_MAX,
      .min_accel = CHRYSLER_DECEL_MIN,
      .inactive_accel = CHRYSLER_DECEL_INACTIVE,
    };

    const int torque = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    const int decel = ((msg->data[2] & 0x0FU) << 8) | msg->data[3];
    const bool torque_req = GET_BIT(msg, 7U);
    const bool acc_active = GET_BIT(msg, 21U);
    const bool fuel_shutoff_disable = (msg->data[2] & 0x80U) != 0U;
    const uint8_t decel_req = (uint8_t)((msg->data[4] >> 4) & 0x7U);
    const uint8_t gr_max = msg->data[4] & 0x0FU;
    const bool torque_inactive = torque == CHRYSLER_LONG_LIMITS.inactive_gas;
    const bool decel_inactive = decel == CHRYSLER_LONG_LIMITS.inactive_accel;
    const bool torque_positive = torque > CHRYSLER_LONG_LIMITS.inactive_gas;
    // Pedal down while cruise is still engaged. Longitudinal commands are blocked,
    // but stock does not drop ACC: it parks the request in a coast band.
    const bool gas_override = controls_allowed && gas_pressed && !get_longitudinal_allowed();

    bool violation = false;
    violation |= (msg->data[2] & 0x40U) != 0U;   // unused bit 22
    violation |= GET_BIT(msg, 5U);               // ACC_STANDSTILL, never observed
    violation |= GET_BIT(msg, 6U);               // ACC_GO, never observed
    violation |= (msg->data[4] & 0x80U) != 0U;   // unused
    violation |= msg->data[5] != 0U;             // ACC_FAULTED
    violation |= (msg->data[6] & 0x0FU) != 0U;   // COLLISION_BRK_PREP, ACC_BRK_PREP
    violation |= decel_req > 1U;
    // Stock gear limit during the hard brake was 4, and 0 the rest of the time.
    violation |= (gr_max != 0U) && (gr_max != 4U);
    violation |= (gr_max == 4U) && decel_inactive;

    if (gas_override) {
      const bool inactive_frame = torque_inactive && decel_inactive && !acc_active && !torque_req &&
                                  (decel_req == 0U) && !fuel_shutoff_disable && (gr_max == 0U);
      const bool override_frame = decel_inactive && !torque_req && acc_active && (decel_req == 0U) &&
                                  !fuel_shutoff_disable && (gr_max == 0U) &&
                                  (torque >= CHRYSLER_TORQUE_MIN) && (torque <= CHRYSLER_TORQUE_OVERRIDE_MAX);
      violation |= !(inactive_frame || override_frame);
    } else {
      violation |= longitudinal_gas_checks(torque, CHRYSLER_LONG_LIMITS);
      violation |= longitudinal_accel_checks(decel, CHRYSLER_LONG_LIMITS);
      // Positive torque stacked on the friction brake can accelerate into a stop.
      // Negative torque with ACC_DECEL is how this truck brakes (about -80 Nm).
      violation |= torque_positive && !decel_inactive;
      violation |= !decel_inactive && !acc_active;
      violation |= (decel_req != 0U) != !decel_inactive;
      // While ACC is actually commanding, the max bit and the fuel-shutoff bit match it.
      // Gas override is the exception, handled above: ACC stays on and both bits drop.
      violation |= torque_req != acc_active;
      if (!get_longitudinal_allowed()) {
        violation |= fuel_shutoff_disable;
        violation |= acc_active;
      }
    }

    if (violation) {
      tx = false;
    }
  }

  // DAS_5 wheel torque is unused. Stock inactive is raw 0 (bytes 4-5 = 00 00), not 0 Nm.
  // Byte 0 is the logged FCW idle pattern. Byte 3 echoes vehicle speed. No CRC: the
  // counter nibble is 0 and the last byte is 0xFF. Any other wheel-torque encoding is rejected.
  if (msg->addr == CHRYSLER_ADDR(DAS_5)) {
    bool violation = false;
    violation |= msg->data[0] != CHRYSLER_DAS5_FCW_IDLE;
    violation |= msg->data[1] != 0U;
    violation |= msg->data[2] != 0U;
    violation |= msg->data[4] != 0U;
    violation |= msg->data[5] != 0U;
    violation |= msg->data[6] != 0U;
    violation |= msg->data[7] != CHRYSLER_DAS5_CHECK_BYTE;
    if (violation) {
      tx = false;
    }
  }

  // DAS_4 is the cluster. Set speed is bytes 1-2. ACC_STATE on this truck is 0, 3, or 4.
  // FCW_BRAKE_DISABLED is 1 in every logged frame. FCW_BRAKE_ENABLED (byte 3) stays 0.
  if (msg->addr == CHRYSLER_ADDR(DAS_4)) {
    bool violation = false;
    violation |= msg->data[0] != 0U;
    violation |= msg->data[3] != 0U;
    violation |= (msg->data[4] != 0x00U) && (msg->data[4] != 0x30U) && (msg->data[4] != 0x40U);
    violation |= msg->data[5] != 0x80U;
    violation |= msg->data[6] != 0U;
    if (violation) {
      tx = false;
    }
  }

  // FORCE CANCEL: only the cancel button press is allowed
  if ((msg->addr == CHRYSLER_ADDR(CRUISE_BUTTONS)) || (msg->addr == CHRYSLER_ADDR(CRUISE_BUTTONS_ALT))) {
    const bool is_cancel = msg->data[0] == 1U;
    const bool is_accel = msg->data[0] == 0x04U;
    const bool is_decel = msg->data[0] == 0x08U;
    const bool is_resume = msg->data[0] == 0x10U;
    const bool allowed = is_cancel || ((is_resume || is_accel || is_decel) && controls_allowed);
    if (!allowed) {
      tx = false;
    }
  }

  return tx;
}

static safety_config chrysler_init(uint16_t param) {
  const uint32_t CHRYSLER_PARAM_RAM_DT = 1U;  // set for Ram DT platform

  // Longitudinal is unvalidated. Same gate as other experimental long modes:
  // only a non-release panda (ALLOW_DEBUG) honors the flag. Stock behavior is unchanged.
#ifdef ALLOW_DEBUG
  const uint32_t CHRYSLER_PARAM_LONGITUDINAL = 4U;
  chrysler_longitudinal = GET_FLAG(param, CHRYSLER_PARAM_LONGITUDINAL);
#else
  chrysler_longitudinal = false;
#endif

  static RxCheck chrysler_ram_dt_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_DT_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_rx_checks[] = {
    {.msg = {{CHRYSLER_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{514, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_DAS_3, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_TRACTION_BUTTON, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_TX_MSGS[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_LKAS_HEARTBIT, 0, 5, .check_relay = true},
  };

  static const CanMsg CHRYSLER_RAM_DT_TX_MSGS[] = {
    {CHRYSLER_RAM_DT_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_DT_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_6, 0, 8, .check_relay = true},
  };

#ifdef ALLOW_DEBUG
  // Buttons engage longitudinal. DAS_3 is still received so ACC main (bit 20) can
  // enable MADS lateral. Stock ACC_ACTIVE on that message is not trusted.
  static RxCheck chrysler_long_rx_checks[] = {
    {.msg = {{CHRYSLER_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{514, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_DAS_3, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_CRUISE_BUTTONS, 0, 3, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_TRACTION_BUTTON, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_ram_dt_long_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_DT_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_CRUISE_BUTTONS, 0, 3, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_DT_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_ram_hd_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_HD_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_ram_hd_long_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_HD_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_CRUISE_BUTTONS, 0, 3, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static RxCheck chrysler_ram_hd_alt_long_rx_checks[] = {
    {.msg = {{CHRYSLER_RAM_HD_EPS_2, 0, 8, 100U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_1, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ESP_8, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_ECM_5, 0, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_DAS_3, 2, 8, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_CRUISE_BUTTONS_ALT, 0, 3, 50U, .max_counter = 15U, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{CHRYSLER_RAM_HD_Center_Stack_2, 0, 8, 1U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true, .ignore_frequency_check = true}, { 0 }, { 0 }}},
  };

  static const CanMsg CHRYSLER_LONG_TX_MSGS[] = {
    {CHRYSLER_CRUISE_BUTTONS, 0, 3, .check_relay = false},
    {CHRYSLER_LKAS_COMMAND, 0, 6, .check_relay = true},
    {CHRYSLER_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_LKAS_HEARTBIT, 0, 5, .check_relay = true},
    {CHRYSLER_DAS_3, 0, 8, .check_relay = true},
    {CHRYSLER_DAS_4, 0, 8, .check_relay = true},
    {CHRYSLER_DAS_5, 0, 8, .check_relay = true},
  };

  static const CanMsg CHRYSLER_RAM_DT_LONG_TX_MSGS[] = {
    {CHRYSLER_RAM_DT_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    {CHRYSLER_RAM_DT_LKAS_COMMAND, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_6, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_3, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_4, 0, 8, .check_relay = true},
    {CHRYSLER_RAM_DT_DAS_5, 0, 8, .check_relay = true},
  };

  const uint32_t CHRYSLER_PARAM_RAM_HD = 2U;  // set for Ram HD platform
  const uint16_t CHRYSLER_PARAM_SP_RAM_HD_ALT_BUTTONS = 1U;
  bool enable_ram_hd = GET_FLAG(param, CHRYSLER_PARAM_RAM_HD);
  bool ram_hd_alt_buttons = GET_FLAG(current_safety_param_sp, CHRYSLER_PARAM_SP_RAM_HD_ALT_BUTTONS);
#endif

  safety_config ret;

  bool enable_ram_dt = GET_FLAG(param, CHRYSLER_PARAM_RAM_DT);

  if (enable_ram_dt) {
    chrysler_platform = CHRYSLER_RAM_DT;
    chrysler_cruise_button_addr = CHRYSLER_RAM_DT_CRUISE_BUTTONS;
#ifdef ALLOW_DEBUG
    if (chrysler_longitudinal) {
      ret = BUILD_SAFETY_CFG(chrysler_ram_dt_long_rx_checks, CHRYSLER_RAM_DT_LONG_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(chrysler_ram_dt_rx_checks, CHRYSLER_RAM_DT_TX_MSGS);
    }
#else
    ret = BUILD_SAFETY_CFG(chrysler_ram_dt_rx_checks, CHRYSLER_RAM_DT_TX_MSGS);
#endif
#ifdef ALLOW_DEBUG
  } else if (enable_ram_hd) {
    static const CanMsg CHRYSLER_RAM_HD_TX_MSGS[] = {
      CHRYSLER_RAM_HD_COMMON_TX_MSGS
      {CHRYSLER_RAM_HD_CRUISE_BUTTONS, 2, 3, .check_relay = false},
    };

    static const CanMsg CHRYSLER_RAM_HD_ALT_BUTTONS_TX_MSGS[] = {
      CHRYSLER_RAM_HD_COMMON_TX_MSGS
      {CHRYSLER_RAM_HD_CRUISE_BUTTONS_ALT, 2, 3, .check_relay = false},
    };

    static const CanMsg CHRYSLER_RAM_HD_LONG_TX_MSGS[] = {
      CHRYSLER_RAM_HD_COMMON_TX_MSGS
      {CHRYSLER_RAM_HD_CRUISE_BUTTONS, 2, 3, .check_relay = false},
      {CHRYSLER_RAM_HD_DAS_3, 0, 8, .check_relay = true},
      {CHRYSLER_RAM_HD_DAS_4, 0, 8, .check_relay = true},
      {CHRYSLER_RAM_HD_DAS_5, 0, 8, .check_relay = true},
    };

    static const CanMsg CHRYSLER_RAM_HD_ALT_BUTTONS_LONG_TX_MSGS[] = {
      CHRYSLER_RAM_HD_COMMON_TX_MSGS
      {CHRYSLER_RAM_HD_CRUISE_BUTTONS_ALT, 2, 3, .check_relay = false},
      {CHRYSLER_RAM_HD_DAS_3, 0, 8, .check_relay = true},
      {CHRYSLER_RAM_HD_DAS_4, 0, 8, .check_relay = true},
      {CHRYSLER_RAM_HD_DAS_5, 0, 8, .check_relay = true},
    };

    chrysler_platform = CHRYSLER_RAM_HD;
    chrysler_cruise_button_addr = ram_hd_alt_buttons ? CHRYSLER_RAM_HD_CRUISE_BUTTONS_ALT : CHRYSLER_RAM_HD_CRUISE_BUTTONS;
    if (chrysler_longitudinal) {
      if (ram_hd_alt_buttons) {
        SET_RX_CHECKS(chrysler_ram_hd_alt_long_rx_checks, ret);
        SET_TX_MSGS(CHRYSLER_RAM_HD_ALT_BUTTONS_LONG_TX_MSGS, ret);
      } else {
        SET_RX_CHECKS(chrysler_ram_hd_long_rx_checks, ret);
        SET_TX_MSGS(CHRYSLER_RAM_HD_LONG_TX_MSGS, ret);
      }
    } else {
      SET_RX_CHECKS(chrysler_ram_hd_rx_checks, ret);
      if (ram_hd_alt_buttons) {
        SET_TX_MSGS(CHRYSLER_RAM_HD_ALT_BUTTONS_TX_MSGS, ret);
      } else {
        SET_TX_MSGS(CHRYSLER_RAM_HD_TX_MSGS, ret);
      }
    }
#endif
  } else {
    chrysler_platform = CHRYSLER_PACIFICA;
    chrysler_cruise_button_addr = CHRYSLER_CRUISE_BUTTONS;
#ifdef ALLOW_DEBUG
    if (chrysler_longitudinal) {
      ret = BUILD_SAFETY_CFG(chrysler_long_rx_checks, CHRYSLER_LONG_TX_MSGS);
    } else {
      ret = BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_TX_MSGS);
    }
#else
    ret = BUILD_SAFETY_CFG(chrysler_rx_checks, CHRYSLER_TX_MSGS);
#endif
  }
  return ret;
}

const safety_hooks chrysler_hooks = {
  .init = chrysler_init,
  .rx = chrysler_rx_hook,
  .tx = chrysler_tx_hook,
  .get_counter = chrysler_get_counter,
  .get_checksum = chrysler_get_checksum,
  .compute_checksum = chrysler_compute_checksum,
};
