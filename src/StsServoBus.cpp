#include "StsServoBus.h"
#include <Arduino.h>
#include <SMS_STS.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "Core2PinMap.h"
#include "PlotterConfig.h"

namespace {
// Dedicated UART1. UART2 is reserved for the TMC2209 bus.
// The external adapter provides the required half-duplex TTL direction control.
HardwareSerial servo_uart(1);
SMS_STS servo;

constexpr uint8_t REG_LOCK_LOCKED = 1;
constexpr int MODE_POSITION = 0;
constexpr int SINGLE_TURN_MAX_LIMIT_COUNTS = 4095;

int wrapDeltaCounts(int delta_counts) {
  if (delta_counts > STS_COUNTS_PER_TURN / 2) delta_counts -= STS_COUNTS_PER_TURN;
  if (delta_counts < -STS_COUNTS_PER_TURN / 2) delta_counts += STS_COUNTS_PER_TURN;
  return delta_counts;
}

int floorModTurn(int32_t counts) {
  const int32_t mod = counts % STS_COUNTS_PER_TURN;
  return static_cast<int>(mod < 0 ? mod + STS_COUNTS_PER_TURN : mod);
}

bool readWordOk(uint8_t id, uint8_t address, int& value) {
  value = servo.readWord(id, address);
  return servo.getLastError() == 0 && value >= 0;
}

bool readByteOk(uint8_t id, uint8_t address, int& value) {
  value = servo.readByte(id, address);
  return servo.getLastError() == 0 && value >= 0;
}

// Goal is a sign-magnitude multi-turn value.
bool readGoalCounts(uint8_t id, int32_t& goal_counts) {
  int raw = 0;
  if (!readWordOk(id, SMS_STS_GOAL_POSITION_L, raw)) return false;
  goal_counts = (raw & 0x8000) ? -(raw & 0x7FFF) : raw;
  return true;
}
}  // namespace

const char* stsMoveResultName(StsMoveResult result) {
  switch (result) {
    case StsMoveResult::PASS: return "PASS";
    case StsMoveResult::TIMEOUT: return "TIMEOUT";
    case StsMoveResult::ABORTED: return "ABORTED";
    case StsMoveResult::REJECTED: return "REJECTED";
    case StsMoveResult::FAILED: return "FAILED";
  }
  return "UNKNOWN";
}

bool StsServoBus::begin() {
  if (initialized_) return true;
#if !SIMULATION_MODE
  servo_uart.begin(PEN_SERVO_UART_BAUD, SERIAL_8N1, PEN_SERVO_UART_RX_PIN,
                   PEN_SERVO_UART_TX_PIN);
  servo.pSerial = &servo_uart;
  servo.IOTimeOut = 10;
#endif
  // No position or torque command at boot.
  initialized_ = true;
  return true;
}

bool StsServoBus::validId(uint8_t id) const {
  return id >= STS_SERVO_ID_MIN && id <= STS_SERVO_ID_MAX &&
         id < sizeof(tracks_) / sizeof(tracks_[0]);
}

StsServoBus::Track& StsServoBus::track(uint8_t id) { return tracks_[id]; }

bool StsServoBus::isTracking(uint8_t id) const {
  return validId(id) && tracks_[id].valid;
}

bool StsServoBus::trackedAbsCounts(uint8_t id, int32_t& abs_counts) const {
  if (!isTracking(id)) return false;
  abs_counts = tracks_[id].abs_counts;
  return true;
}

bool StsServoBus::ping(uint8_t id) {
  if (!initialized_) return false;
#if SIMULATION_MODE
  (void)id;
  return true;
#else
  return servo.Ping(id) == id;
#endif
}

bool StsServoBus::readRawPosition(uint8_t id, int& raw_counts) {
  if (!initialized_) return false;
#if SIMULATION_MODE
  raw_counts = validId(id) ? floorModTurn(tracks_[id].abs_counts) : 0;
  return true;
#else
  const int value = servo.ReadPos(id);
  if (servo.getLastError() != 0 || value < 0 || value > SINGLE_TURN_MAX_LIMIT_COUNTS) {
    return false;
  }
  raw_counts = value;
  return true;
#endif
}

bool StsServoBus::readStatus(uint8_t id, StsServoStatus& status) {
  status = StsServoStatus{};
  if (validId(id)) {
    status.tracking = tracks_[id].valid;
    status.abs_counts = tracks_[id].abs_counts;
  }
#if SIMULATION_MODE
  status.link = initialized_;
  status.mode = MODE_POSITION;
  status.torque = 0;
  status.raw_counts = validId(id) ? floorModTurn(tracks_[id].abs_counts) : 0;
  status.min_limit_counts = 0;
  status.max_limit_counts = status.tracking ? 0 : SINGLE_TURN_MAX_LIMIT_COUNTS;
  status.moving = 0;
  return status.link;
#else
  if (!ping(id)) return false;
  status.link = true;
  status.error_flags = servo.getState();
  readByteOk(id, SMS_STS_MODE, status.mode);
  readByteOk(id, SMS_STS_TORQUE_ENABLE, status.torque);
  readRawPosition(id, status.raw_counts);
  int32_t goal_counts = 0;
  if (readGoalCounts(id, goal_counts)) status.goal_counts = goal_counts;
  readWordOk(id, SMS_STS_MIN_ANGLE_LIMIT_L, status.min_limit_counts);
  readWordOk(id, SMS_STS_MAX_ANGLE_LIMIT_L, status.max_limit_counts);
  readByteOk(id, SMS_STS_PRESENT_VOLTAGE, status.voltage_dv);
  readByteOk(id, SMS_STS_PRESENT_TEMPERATURE, status.temperature_c);
  readByteOk(id, SMS_STS_MOVING, status.moving);
  return true;
#endif
}

bool StsServoBus::torqueOff(uint8_t id) {
  if (!initialized_) return false;
#if SIMULATION_MODE
  (void)id;
  return true;
#else
  return servo.EnableTorque(id, 0) != 0;
#endif
}

bool StsServoBus::moveSingleTurn(uint8_t id, uint16_t position_counts,
                                 uint16_t speed, uint8_t acc,
                                 uint32_t timeout_ms, uint16_t tolerance_counts) {
  if (!initialized_ || position_counts > SINGLE_TURN_MAX_LIMIT_COUNTS) return false;
#if SIMULATION_MODE
  (void)id;
  (void)speed;
  (void)acc;
  (void)timeout_ms;
  (void)tolerance_counts;
  return true;
#else
  if (!ping(id)) return false;
  // In multi-turn mode a single-turn goal could spin the servo by whole turns.
  int min_limit = -1;
  int max_limit = -1;
  if (isTracking(id) || !readWordOk(id, SMS_STS_MIN_ANGLE_LIMIT_L, min_limit) ||
      !readWordOk(id, SMS_STS_MAX_ANGLE_LIMIT_L, max_limit) || min_limit != 0 ||
      max_limit != SINGLE_TURN_MAX_LIMIT_COUNTS) {
    return false;
  }
  if (!servo.EnableTorque(id, 1)) return false;
  if (!servo.WritePosEx(id, position_counts, speed, acc)) return false;
  const uint32_t started_ms = millis();
  do {
    int actual = 0;
    if (readRawPosition(id, actual) &&
        abs(actual - static_cast<int>(position_counts)) <= tolerance_counts) {
      return true;
    }
    delay(STS_TEST_POLL_INTERVAL_MS);
  } while (millis() - started_ms < timeout_ms);
  return false;
#endif
}

bool StsServoBus::checkReady(uint8_t id, char* detail, size_t detail_size) {
#if SIMULATION_MODE
  (void)id;
  (void)detail;
  (void)detail_size;
  return true;
#else
  if (!ping(id)) {
    snprintf(detail, detail_size, "reason=no_reply");
    return false;
  }
  const uint8_t flags = servo.getState();
  if (flags != 0) {
    snprintf(detail, detail_size, "reason=servo_error flags=0x%02X", flags);
    return false;
  }
  int mode = -1;
  if (!readByteOk(id, SMS_STS_MODE, mode) || mode != MODE_POSITION) {
    snprintf(detail, detail_size, "reason=not_position_mode mode=%d", mode);
    return false;
  }
  return true;
#endif
}

void StsServoBus::updateTracking(Track& state, int raw_counts) {
  state.abs_counts += wrapDeltaCounts(raw_counts - state.last_raw_counts);
  state.last_raw_counts = raw_counts;
}

// Enables multi-turn control and starts tracking. Refuses whenever the hidden
// multi-turn position cannot be shown to match the tracked one.
bool StsServoBus::enableTracking(uint8_t id, char* detail, size_t detail_size) {
  Track& state = track(id);
#if SIMULATION_MODE
  state.valid = true;
  state.abs_counts = 0;
  state.last_raw_counts = 0;
  (void)detail;
  (void)detail_size;
  return true;
#else
  int lock = -1;
  if (!readByteOk(id, SMS_STS_LOCK, lock) || lock != REG_LOCK_LOCKED) {
    // Unlocked EEPROM would persist the limit change.
    snprintf(detail, detail_size, "reason=eeprom_unlocked lock=%d", lock);
    return false;
  }
  int min_limit = -1;
  int max_limit = -1;
  int torque = -1;
  int moving = -1;
  int raw_counts = 0;
  int32_t goal_counts = 0;
  if (!readWordOk(id, SMS_STS_MIN_ANGLE_LIMIT_L, min_limit) ||
      !readWordOk(id, SMS_STS_MAX_ANGLE_LIMIT_L, max_limit) ||
      !readByteOk(id, SMS_STS_TORQUE_ENABLE, torque) ||
      !readByteOk(id, SMS_STS_MOVING, moving) ||
      !readGoalCounts(id, goal_counts) || !readRawPosition(id, raw_counts)) {
    snprintf(detail, detail_size, "reason=read_failed");
    return false;
  }

  if (min_limit == 0 && max_limit == SINGLE_TURN_MAX_LIMIT_COUNTS) {
    // Goal resets to 0 at servo power-up (measured). Torque off + goal 0 is
    // taken as power-up state; the hold check on the move verifies it.
    const bool power_up_state = goal_counts == 0 && torque == 0;
    if (!power_up_state && abs(goal_counts - raw_counts) > STS_TEST_TOLERANCE_COUNTS) {
      snprintf(detail, detail_size,
               "reason=goal_mismatch goal=%ld raw=%d (power-cycle servo)",
               static_cast<long>(goal_counts), raw_counts);
      return false;
    }
    if (!servo.EnableTorque(id, 0) ||
        !servo.writeWord(id, SMS_STS_MAX_ANGLE_LIMIT_L, 0)) {
      snprintf(detail, detail_size, "reason=limit_write_failed");
      return false;
    }
    state.abs_counts = raw_counts;
  } else if (min_limit == 0 && max_limit == 0) {
    // Already multi-turn (e.g. after an ESP32 reset, possibly with torque
    // off). Recover from a servo at rest whose position still matches its
    // last goal within the turn. A whole-turn mismatch (shaft turned a full
    // turn by hand while torque was off) is caught by the hold check before
    // any goal moves; it stops the servo within ~200 ms.
    const int offset = wrapDeltaCounts(raw_counts - floorModTurn(goal_counts));
    if (moving != 0 || abs(offset) > STS_TEST_TOLERANCE_COUNTS) {
      snprintf(detail, detail_size,
               "reason=unknown_turn_count torque=%d moving=%d goal=%ld raw=%d "
               "(power-cycle servo)",
               torque, moving, static_cast<long>(goal_counts), raw_counts);
      return false;
    }
    state.abs_counts = goal_counts + offset;
  } else {
    snprintf(detail, detail_size, "reason=unexpected_limits min=%d max=%d",
             min_limit, max_limit);
    return false;
  }
  state.last_raw_counts = raw_counts;
  state.valid = true;
  return true;
#endif
}

bool StsServoBus::holdAndVerify(uint8_t id, int32_t hold_counts,
                                int start_raw_counts, char* detail,
                                size_t detail_size) {
#if SIMULATION_MODE
  (void)id;
  (void)hold_counts;
  (void)start_raw_counts;
  (void)detail;
  (void)detail_size;
  return true;
#else
  if (!servo.WritePosEx(id, static_cast<int16_t>(hold_counts),
                        STS_TEST_SPEED_COUNTS_PER_S, STS_TEST_ACC) ||
      !servo.EnableTorque(id, 1)) {
    snprintf(detail, detail_size, "reason=hold_write_failed");
    return false;
  }
  const uint32_t started_ms = millis();
  while (millis() - started_ms < STS_TEST_HOLD_CHECK_MS) {
    delay(STS_TEST_POLL_INTERVAL_MS);
    int raw_counts = 0;
    if (!readRawPosition(id, raw_counts)) continue;
    const int drift = wrapDeltaCounts(raw_counts - start_raw_counts);
    if (abs(drift) > STS_TEST_HOLD_CHECK_TOLERANCE_COUNTS) {
      servo.EnableTorque(id, 0);
      snprintf(detail, detail_size,
               "reason=hold_check_moved drift=%+d torque=OFF (power-cycle servo)", drift);
      return false;
    }
  }
  return true;
#endif
}

StsMoveResult StsServoBus::moveRelative(uint8_t id, int32_t delta_counts,
                                        bool (*abort_requested)(), char* detail,
                                        size_t detail_size) {
  detail[0] = '\0';
  if (!initialized_ || !validId(id)) {
    snprintf(detail, detail_size, "reason=invalid_id");
    return StsMoveResult::REJECTED;
  }
  const int32_t max_delta_counts =
      static_cast<int32_t>(STS_TEST_MAX_TURNS * STS_COUNTS_PER_TURN);
  if (delta_counts == 0 || labs(delta_counts) > max_delta_counts) {
    snprintf(detail, detail_size, "reason=delta_out_of_range max=%ld",
             static_cast<long>(max_delta_counts));
    return StsMoveResult::REJECTED;
  }
  if (!checkReady(id, detail, detail_size)) return StsMoveResult::REJECTED;
  Track& state = track(id);
  if (!state.valid && !enableTracking(id, detail, detail_size)) {
    return StsMoveResult::REJECTED;
  }

#if SIMULATION_MODE
  state.abs_counts += delta_counts;
  state.last_raw_counts = floorModTurn(state.abs_counts);
  snprintf(detail, detail_size, "target=%ld actual=%ld sim=YES",
           static_cast<long>(state.abs_counts), static_cast<long>(state.abs_counts));
  return StsMoveResult::PASS;
#else
  int min_limit = -1;
  int max_limit = -1;
  if (!readWordOk(id, SMS_STS_MIN_ANGLE_LIMIT_L, min_limit) ||
      !readWordOk(id, SMS_STS_MAX_ANGLE_LIMIT_L, max_limit) ||
      min_limit != 0 || max_limit != 0) {
    // Limits restored: the servo was power-cycled and its turn count reset.
    state.valid = false;
    snprintf(detail, detail_size, "reason=multi_turn_lost (retry)");
    return StsMoveResult::REJECTED;
  }
  int raw_counts = 0;
  if (!readRawPosition(id, raw_counts)) {
    snprintf(detail, detail_size, "reason=read_failed");
    return StsMoveResult::REJECTED;
  }
  // Idle drift must stay small; a large jump means the turn count is unknown.
  const int drift = wrapDeltaCounts(raw_counts - state.last_raw_counts);
  if (abs(drift) > STS_TEST_TOLERANCE_COUNTS) {
    state.valid = false;
    snprintf(detail, detail_size,
             "reason=idle_drift drift=%+d tracking=INVALID (power-cycle servo)", drift);
    return StsMoveResult::REJECTED;
  }
  updateTracking(state, raw_counts);
  const int32_t start_counts = state.abs_counts;
  const int32_t target_counts = start_counts + delta_counts;
  if (labs(target_counts) > STS_TEST_ABS_LIMIT_COUNTS) {
    snprintf(detail, detail_size, "reason=abs_out_of_range target=%ld limit=%ld",
             static_cast<long>(target_counts),
             static_cast<long>(STS_TEST_ABS_LIMIT_COUNTS));
    return StsMoveResult::REJECTED;
  }
  if (!holdAndVerify(id, start_counts, raw_counts, detail, detail_size)) {
    state.valid = false;
    return StsMoveResult::FAILED;
  }
  if (!servo.WritePosEx(id, static_cast<int16_t>(target_counts),
                        STS_TEST_SPEED_COUNTS_PER_S, STS_TEST_ACC)) {
    snprintf(detail, detail_size, "reason=goal_write_failed");
    return StsMoveResult::FAILED;
  }

  const uint32_t timeout_ms =
      static_cast<uint32_t>(labs(delta_counts)) * 1000U / STS_TEST_SPEED_COUNTS_PER_S +
      STS_TEST_TIMEOUT_MARGIN_MS;
  const uint32_t started_ms = millis();
  uint32_t last_good_read_ms = started_ms;
  while (millis() - started_ms < timeout_ms) {
    delay(STS_TEST_POLL_INTERVAL_MS);
    if (abort_requested != nullptr && abort_requested()) {
      // Hold where it is instead of dropping torque (a loaded Z drum would fall).
      servo.WritePosEx(id, static_cast<int16_t>(state.abs_counts),
                       STS_TEST_SPEED_COUNTS_PER_S, STS_TEST_ACC);
      snprintf(detail, detail_size, "target=%ld actual=%ld reason=abort hold=YES",
               static_cast<long>(target_counts), static_cast<long>(state.abs_counts));
      return StsMoveResult::ABORTED;
    }
    int sample = 0;
    if (!readRawPosition(id, sample)) {
      if (millis() - last_good_read_ms > STS_TEST_MAX_READ_GAP_MS) {
        state.valid = false;
        snprintf(detail, detail_size,
                 "target=%ld reason=feedback_lost tracking=INVALID",
                 static_cast<long>(target_counts));
        return StsMoveResult::FAILED;
      }
      continue;
    }
    last_good_read_ms = millis();
    updateTracking(state, sample);
    const int32_t moved = state.abs_counts - start_counts;
    const int32_t progress = delta_counts > 0 ? moved : -moved;
    if (progress < -STS_TEST_RUNAWAY_MARGIN_COUNTS ||
        progress > labs(delta_counts) + STS_TEST_RUNAWAY_MARGIN_COUNTS) {
      servo.EnableTorque(id, 0);
      state.valid = false;
      snprintf(detail, detail_size,
               "target=%ld actual=%ld reason=runaway torque=OFF tracking=INVALID",
               static_cast<long>(target_counts), static_cast<long>(state.abs_counts));
      return StsMoveResult::FAILED;
    }
    if (labs(state.abs_counts - target_counts) <= STS_TEST_TOLERANCE_COUNTS &&
        servo.ReadMove(id) == 0) {
      snprintf(detail, detail_size, "target=%ld actual=%ld error=%+ld ms=%lu",
               static_cast<long>(target_counts), static_cast<long>(state.abs_counts),
               static_cast<long>(state.abs_counts - target_counts),
               static_cast<unsigned long>(millis() - started_ms));
      return StsMoveResult::PASS;
    }
  }
  snprintf(detail, detail_size, "target=%ld actual=%ld ms=%lu torque=ON",
           static_cast<long>(target_counts), static_cast<long>(state.abs_counts),
           static_cast<unsigned long>(timeout_ms));
  return StsMoveResult::TIMEOUT;
#endif
}

StsMoveResult StsServoBus::moveToAngle(uint8_t id, float angle_deg,
                                       bool (*abort_requested)(), char* detail,
                                       size_t detail_size) {
  detail[0] = '\0';
  if (!(angle_deg >= 0.0f && angle_deg < 360.0f)) {
    snprintf(detail, detail_size, "reason=angle_out_of_range range=[0,360)");
    return StsMoveResult::REJECTED;
  }
  const int target_raw_counts =
      static_cast<int>(lroundf(angle_deg * STS_COUNTS_PER_TURN / 360.0f)) %
      STS_COUNTS_PER_TURN;
  int raw_counts = 0;
  if (!readRawPosition(id, raw_counts)) {
    snprintf(detail, detail_size, "reason=read_failed");
    return StsMoveResult::REJECTED;
  }
  const int delta_counts = wrapDeltaCounts(target_raw_counts - raw_counts);
  if (abs(delta_counts) <= STS_TEST_TOLERANCE_COUNTS) {
    snprintf(detail, detail_size, "raw=%d already_there=YES", raw_counts);
    return StsMoveResult::PASS;
  }
  return moveRelative(id, delta_counts, abort_requested, detail, detail_size);
}

bool StsServoBus::beginJog(uint8_t id, char* detail, size_t detail_size) {
  detail[0] = '\0';
  if (!initialized_ || !validId(id)) {
    snprintf(detail, detail_size, "reason=invalid_id");
    return false;
  }
  if (!checkReady(id, detail, detail_size)) return false;
  Track& state = track(id);
  if (!state.valid && !enableTracking(id, detail, detail_size)) return false;
#if SIMULATION_MODE
  state.jogging = true;
  state.jog_goal_counts = state.abs_counts;
  return true;
#else
  int raw_counts = 0;
  if (!readRawPosition(id, raw_counts)) {
    snprintf(detail, detail_size, "reason=read_failed");
    return false;
  }
  const int drift = wrapDeltaCounts(raw_counts - state.last_raw_counts);
  if (abs(drift) > STS_TEST_TOLERANCE_COUNTS) {
    state.valid = false;
    snprintf(detail, detail_size,
             "reason=idle_drift drift=%+d tracking=INVALID (power-cycle servo)", drift);
    return false;
  }
  updateTracking(state, raw_counts);
  if (!holdAndVerify(id, state.abs_counts, raw_counts, detail, detail_size)) {
    state.valid = false;
    return false;
  }
  state.jogging = true;
  state.jog_goal_counts = state.abs_counts;
  state.jog_last_good_read_ms = millis();
  return true;
#endif
}

bool StsServoBus::updateJog(uint8_t id, int32_t counts_s, char* detail,
                            size_t detail_size) {
  if (!validId(id)) return false;
  Track& state = track(id);
  if (!state.jogging || !state.valid) return false;
#if SIMULATION_MODE
  state.abs_counts += counts_s * static_cast<int32_t>(JOY_SESSION_PERIOD_MS) / 1000;
  state.last_raw_counts = floorModTurn(state.abs_counts);
  (void)detail;
  (void)detail_size;
  return true;
#else
  int raw_counts = 0;
  if (readRawPosition(id, raw_counts)) {
    updateTracking(state, raw_counts);
    state.jog_last_good_read_ms = millis();
  } else if (millis() - state.jog_last_good_read_ms > STS_TEST_MAX_READ_GAP_MS) {
    servo.WritePosEx(id, static_cast<int16_t>(state.jog_goal_counts),
                     STS_TEST_SPEED_COUNTS_PER_S, STS_TEST_ACC);
    state.jogging = false;
    state.valid = false;
    snprintf(detail, detail_size, "reason=feedback_lost tracking=INVALID");
    return false;
  }
  // Runaway: the servo must stay near the goal we keep placing ahead of it.
  const int32_t lead_counts =
      static_cast<int32_t>(labs(counts_s)) * static_cast<int32_t>(JOY_SERVO_LEAD_MS) / 1000;
  if (labs(state.abs_counts - state.jog_goal_counts) >
      lead_counts + STS_TEST_RUNAWAY_MARGIN_COUNTS + STS_TEST_SPEED_COUNTS_PER_S / 2) {
    servo.EnableTorque(id, 0);
    state.jogging = false;
    state.valid = false;
    snprintf(detail, detail_size,
             "reason=runaway abs=%ld goal=%ld torque=OFF tracking=INVALID",
             static_cast<long>(state.abs_counts), static_cast<long>(state.jog_goal_counts));
    return false;
  }
  int32_t goal_counts = state.abs_counts;
  if (counts_s > 0) goal_counts += lead_counts;
  if (counts_s < 0) goal_counts -= lead_counts;
  if (goal_counts > STS_TEST_ABS_LIMIT_COUNTS) goal_counts = STS_TEST_ABS_LIMIT_COUNTS;
  if (goal_counts < -STS_TEST_ABS_LIMIT_COUNTS) goal_counts = -STS_TEST_ABS_LIMIT_COUNTS;
  if (goal_counts != state.jog_goal_counts) {
    const uint16_t speed = static_cast<uint16_t>(
        counts_s == 0 ? STS_TEST_SPEED_COUNTS_PER_S : labs(counts_s));
    if (!servo.WritePosEx(id, static_cast<int16_t>(goal_counts), speed, STS_TEST_ACC)) {
      snprintf(detail, detail_size, "reason=goal_write_failed");
      return true;  // transient; keep tracking and retry next period
    }
    state.jog_goal_counts = goal_counts;
  }
  return true;
#endif
}

void StsServoBus::endJog(uint8_t id) {
  if (!validId(id)) return;
  Track& state = track(id);
  if (!state.jogging) return;
  state.jogging = false;
#if !SIMULATION_MODE
  int raw_counts = 0;
  if (readRawPosition(id, raw_counts)) updateTracking(state, raw_counts);
  // Hold where it is instead of dropping torque (a loaded Z drum would fall).
  servo.WritePosEx(id, static_cast<int16_t>(state.abs_counts), STS_TEST_SPEED_COUNTS_PER_S,
                   STS_TEST_ACC);
#endif
}
