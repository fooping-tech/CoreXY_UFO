// FEETECH STS3020 standalone test firmware entry point.
// Built only by the `m5stack-core2-sts3020-test` PlatformIO env.
//
// Uses only Core2 power/LCD/touch (M5Unified), USB Serial and UART1.
// No CoreXY, TMC2209, NeoPixel or task-based UI is started. Serial commands and
// the touch UI both run in Arduino loop(), so the servo bus is never accessed
// concurrently.
//
// Safety rules:
// - Boot performs UART init and PING only. The servo never moves at boot.
// - Motion happens only on an explicit JOG command (Serial or one touch tap).
// - Touch and Serial share the same checked JOG path.
// - JOG never moves when any read/check fails.
// - OFF disables torque. It is not an emergency stop.

#include <Arduino.h>
#include <M5Unified.h>
#include <SMS_STS.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "sts3020test/Sts3020TestConfig.h"

namespace {

HardwareSerial servo_serial(STS_TEST_UART_NUM);
SMS_STS servo;

constexpr size_t LINE_BUFFER_SIZE = 64;
char line_buffer[LINE_BUFFER_SIZE];
size_t line_length = 0;

// Active servo ID for all commands. Changed with ID <n>.
uint8_t servo_id = STS_TEST_DEFAULT_SERVO_ID;

// Short result of the last Serial/touch command, shown on the LCD.
char last_result[40] = "-";

// Multi-turn position tracked by the firmware. Present Position wraps per
// turn, so the absolute (multi-turn) position is unwrapped from feedback.
// Valid only between MT_ON and MT_OFF while torque stays on.
bool mt_tracking_valid = false;
int mt_abs_counts = 0;
int mt_last_raw_counts = 0;

void mtStartTracking(int raw_counts) {
  mt_abs_counts = raw_counts;
  mt_last_raw_counts = raw_counts;
  mt_tracking_valid = true;
}

// Feed a wrapped 0..4095 sample. Assumes < half a turn since the last one.
void mtUpdateTracking(int raw_counts) {
  int delta = raw_counts - mt_last_raw_counts;
  if (delta > STS_TEST_COUNTS_PER_TURN / 2) delta -= STS_TEST_COUNTS_PER_TURN;
  if (delta < -STS_TEST_COUNTS_PER_TURN / 2) delta += STS_TEST_COUNTS_PER_TURN;
  mt_abs_counts += delta;
  mt_last_raw_counts = raw_counts;
}

const char* errorName(uint8_t error) {
  switch (error) {
    case 0: return "none";
    case ERR_NO_REPLY: return "no reply (timeout)";
    case ERR_CRC_CMP: return "checksum mismatch";
    case ERR_SLAVE_ID: return "reply from unexpected ID";
    case ERR_BUFF_LEN: return "unexpected reply length";
    default: return "unknown";
  }
}

void printLastError() {
  const uint8_t error = servo.getLastError();
  Serial.printf("  Error: %u (%s)\n", error, errorName(error));
}

// STS status byte returned in every reply packet.
void printServoState(uint8_t state) {
  Serial.printf("  Servo error flags: 0x%02X", state);
  if (state == 0) {
    Serial.println(" (none)");
    return;
  }
  if (state & 0x01) Serial.print(" VOLTAGE");
  if (state & 0x02) Serial.print(" ENCODER");
  if (state & 0x04) Serial.print(" OVERHEAT");
  if (state & 0x08) Serial.print(" OVERCURRENT");
  if (state & 0x20) Serial.print(" OVERLOAD");
  Serial.println();
}

void printPingFailureHints() {
  Serial.println("  Possible causes:");
  Serial.printf("  - Servo ID is not %u (see broadcast result above)\n",
                servo_id);
  Serial.printf("  - Servo baud is not %lu\n",
                static_cast<unsigned long>(STS_TEST_UART_BAUD));
  Serial.printf("  - UART pins: TX=GPIO%u RX=GPIO%u (Core2 Port A) swapped or loose\n",
                STS_TEST_UART_TX_PIN, STS_TEST_UART_RX_PIN);
  Serial.println("  - Bus servo adapter missing/unpowered (half-duplex required)");
  Serial.println("  - Servo external supply off, or GND not shared with Core2");
}

bool readPosition(int& position_counts) {
  const int value = servo.ReadPos(servo_id);
  if (value < STS_TEST_POSITION_MIN_COUNTS ||
      value > STS_TEST_POSITION_MAX_COUNTS) {
    return false;
  }
  position_counts = value;
  return true;
}

// Multi-turn safe read: accepts signed positions. ReadPos() returns -1 both on
// failure and for a valid -1, so success is judged by getLastError().
bool readPositionAny(int& position_counts) {
  const int value = servo.ReadPos(servo_id);
  if (servo.getLastError() != 0) return false;
  position_counts = value;
  return true;
}

bool readAngleLimits(int& min_counts, int& max_counts) {
  min_counts = servo.readWord(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L);
  if (servo.getLastError() != 0 || min_counts < 0) return false;
  max_counts = servo.readWord(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (servo.getLastError() != 0 || max_counts < 0) return false;
  return true;
}

// Angle limits 0/0 select multi-turn position control.
bool isMultiTurn(bool& multi_turn) {
  int min_counts = 0;
  int max_counts = 0;
  if (!readAngleLimits(min_counts, max_counts)) return false;
  multi_turn = min_counts == 0 && max_counts == 0;
  return true;
}

bool ping(bool verbose) {
  const int id = servo.Ping(servo_id);
  if (id == servo_id) {
    if (verbose) {
      Serial.printf("Servo detected: ID=%d\n", id);
      printServoState(servo.getState());
    }
    return true;
  }
  if (verbose) {
    Serial.printf("PING FAILED: ID=%u\n", servo_id);
    printLastError();
    // Only one servo is on the bus, so a broadcast ping is safe and reveals
    // an unexpected ID.
    const int any_id = servo.Ping(STS_TEST_BROADCAST_ID);
    if (any_id >= 0) {
      Serial.printf("  Broadcast PING replied from ID=%d (ID mismatch)\n",
                    any_id);
    } else {
      Serial.println("  Broadcast PING: no reply");
    }
    printPingFailureHints();
  }
  return false;
}

void cmdHelp() {
  Serial.println("Commands:");
  Serial.println("  HELP           Show this list");
  Serial.println("  PING           Check communication with the servo");
  Serial.println("  READ           Read current position (0..4095 counts)");
  Serial.printf("  JOG <+/-n>     Move n counts from current (|n|<=%d)\n",
                STS_TEST_JOG_MAX_DELTA_COUNTS);
  Serial.println("  OFF            Disable torque (NOT an emergency stop)");
  Serial.println("  STATUS         Read ID, position, voltage, temperature etc.");
  Serial.println("Servo ID (multiple servos on one bus need unique IDs):");
  Serial.println("  SCAN           PING IDs 0..20 and list responders");
  Serial.println("  ID <n>         Select the servo ID used by all commands");
  Serial.println("  SET_ID <n>     Change the selected servo's ID (EEPROM, persistent)");
  Serial.println("                 Connect ONLY that servo to the bus first.");
  Serial.println("Multi-turn bench test (no load):");
  Serial.println("  MT_ON          Angle limits 0/0 (temporary, EEPROM stays locked)");
  Serial.printf("  MT_JOG <+/-n>  Multi-turn move n counts (|n|<=%d, 4096=1 turn)\n",
                STS_TEST_MT_JOG_MAX_DELTA_COUNTS);
  Serial.println("  MT_OFF         Restore angle limits 0/4095 (single-turn)");
}

void cmdPing() {
  Serial.println("PING...");
  ping(true);
}

void cmdRead() {
  Serial.println("READ...");
  int position_counts = 0;
  if (!readPositionAny(position_counts)) {
    Serial.println("READ FAILED");
    printLastError();
    return;
  }
  Serial.printf("Position: %d\n", position_counts);
}

void printReadResult(const char* label, int value, const char* unit_fmt,
                     float scale) {
  if (value < 0) {
    Serial.printf("  %-12s: read failed\n", label);
    return;
  }
  Serial.printf("  %-12s: ", label);
  Serial.printf(unit_fmt, value * scale);
  Serial.println();
}

void cmdStatus() {
  Serial.println("STATUS...");
  if (!ping(true)) return;
  Serial.printf("  %-12s: %u\n", "ID", servo_id);
  const int model = servo.readWord(servo_id, SMS_STS_MODEL_L);
  printReadResult("Model", model, "%.0f", 1.0f);
  const int mode = servo.readByte(servo_id, SMS_STS_MODE);
  if (mode < 0) {
    Serial.printf("  %-12s: read failed\n", "Mode");
  } else {
    Serial.printf("  %-12s: %d%s\n", "Mode", mode,
                  mode == STS_TEST_MODE_POSITION ? " (position)" : " (NOT position)");
  }
  const int torque = servo.readByte(servo_id, SMS_STS_TORQUE_ENABLE);
  printReadResult("Torque", torque, "%.0f", 1.0f);
  int status_position_counts = 0;
  if (readPositionAny(status_position_counts)) {
    Serial.printf("  %-12s: %d counts\n", "Position", status_position_counts);
  } else {
    Serial.printf("  %-12s: read failed\n", "Position");
  }
  const int goal = servo.readWord(servo_id, SMS_STS_GOAL_POSITION_L);
  printReadResult("Goal", goal, "%.0f counts", 1.0f);
  const int min_limit = servo.readWord(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L);
  const int max_limit = servo.readWord(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L);
  printReadResult("Min limit", min_limit, "%.0f counts", 1.0f);
  printReadResult("Max limit", max_limit, "%.0f counts", 1.0f);
  if (min_limit >= 0 && max_limit >= 0) {
    Serial.printf("  %-12s: %s\n", "Multi-turn",
                  (min_limit == 0 && max_limit == 0) ? "ON (limits 0/0)" : "OFF");
  }
  if (mt_tracking_valid) {
    Serial.printf("  %-12s: %d counts (%.2f turn)\n", "MT abs", mt_abs_counts,
                  static_cast<float>(mt_abs_counts) / STS_TEST_COUNTS_PER_TURN);
  }
  const int lock = servo.readByte(servo_id, SMS_STS_LOCK);
  if (lock < 0) {
    Serial.printf("  %-12s: read failed\n", "EEPROM lock");
  } else {
    Serial.printf("  %-12s: %d%s\n", "EEPROM lock", lock,
                  lock == STS_TEST_REG_LOCK_LOCKED ? " (locked)" : " (UNLOCKED)");
  }
  // ReadSpeed/ReadLoad return sign-magnitude decoded values; -1 may be valid
  // but is indistinguishable from failure in the library API.
  Serial.printf("  %-12s: %d counts/s\n", "Speed", servo.ReadSpeed(servo_id));
  Serial.printf("  %-12s: %d (0.1%%)\n", "Load", servo.ReadLoad(servo_id));
  printReadResult("Voltage", servo.ReadVoltage(servo_id), "%.1f V", 0.1f);
  // EEPROM protection limits. The VOLTAGE flag is set outside this window.
  printReadResult("Max voltage", servo.readByte(servo_id, STS_TEST_REG_MAX_INPUT_VOLTAGE),
                  "%.1f V", 0.1f);
  printReadResult("Min voltage", servo.readByte(servo_id, STS_TEST_REG_MIN_INPUT_VOLTAGE),
                  "%.1f V", 0.1f);
  printReadResult("Max temp", servo.readByte(servo_id, STS_TEST_REG_MAX_TEMPERATURE),
                  "%.0f C", 1.0f);
  printReadResult("Temperature", servo.ReadTemper(servo_id), "%.0f C", 1.0f);
  printReadResult("Current", servo.ReadCurrent(servo_id), "%.0f (raw)", 1.0f);
  printReadResult("Moving", servo.ReadMove(servo_id), "%.0f", 1.0f);
}

void cmdOff() {
  Serial.println("OFF");
  const bool ok = servo.EnableTorque(servo_id, 0);
  if (ok) {
    Serial.println("Torque disabled");
  } else {
    Serial.println("Torque disable FAILED");
    printLastError();
  }
  Serial.println("  Note: OFF is not an emergency stop. Cut servo power if needed.");
  snprintf(last_result, sizeof(last_result), "OFF %s", ok ? "OK" : "FAILED");
  if (mt_tracking_valid) {
    // Shaft may now be turned by hand; the turn count can no longer be trusted.
    mt_tracking_valid = false;
    Serial.println("  Multi-turn tracking invalidated (run MT_OFF, then MT_ON)");
  }
}

bool parseDelta(const char* text, int& delta_counts) {
  if (text == nullptr || *text == '\0') return false;
  char* end = nullptr;
  const long value = strtol(text, &end, 10);
  if (end == text || *end != '\0') return false;
  delta_counts = static_cast<int>(value);
  return true;
}

int wrapDeltaCounts(int delta_counts) {
  if (delta_counts > STS_TEST_COUNTS_PER_TURN / 2) delta_counts -= STS_TEST_COUNTS_PER_TURN;
  if (delta_counts < -STS_TEST_COUNTS_PER_TURN / 2) delta_counts += STS_TEST_COUNTS_PER_TURN;
  return delta_counts;
}

// Hold the present position, enable torque and confirm the servo stays put.
// A hidden multi-turn offset shows up here as motion; torque is cut at once.
bool holdAndVerify(const char* tag, int hold_goal_counts, uint16_t speed, uint8_t acc,
                   int start_raw_counts) {
  if (!servo.WritePosEx(servo_id, hold_goal_counts, speed, acc)) {
    Serial.printf("%s ABORTED: hold-position write failed\n", tag);
    printLastError();
    return false;
  }
  if (!servo.EnableTorque(servo_id, 1)) {
    Serial.printf("%s ABORTED: torque enable failed\n", tag);
    printLastError();
    return false;
  }
  const uint32_t started_ms = millis();
  while (millis() - started_ms < STS_TEST_HOLD_CHECK_MS) {
    delay(STS_TEST_JOG_POLL_INTERVAL_MS);
    int raw_counts = 0;
    if (!readPosition(raw_counts)) continue;
    const int drift = wrapDeltaCounts(raw_counts - start_raw_counts);
    if (abs(drift) > STS_TEST_HOLD_CHECK_TOLERANCE_COUNTS) {
      servo.EnableTorque(servo_id, 0);
      Serial.printf("%s ABORTED: moved %+d counts while holding; torque disabled\n", tag,
                    drift);
      Serial.println("  Servo internal position differs from Present Position (hidden turns).");
      Serial.println("  Power-cycle the servo supply before retrying.");
      return false;
    }
  }
  Serial.println("Hold check: OK");
  return true;
}

// Cuts torque and returns true if motion left the commanded path.
bool runawayDetected(const char* tag, int start_counts, int now_counts, int delta_counts) {
  const int moved = now_counts - start_counts;
  const int progress = delta_counts > 0 ? moved : -moved;
  if (progress < -STS_TEST_RUNAWAY_MARGIN_COUNTS ||
      progress > abs(delta_counts) + STS_TEST_RUNAWAY_MARGIN_COUNTS) {
    servo.EnableTorque(servo_id, 0);
    Serial.printf("%s RUNAWAY: moved %+d counts for a %+d command; torque disabled\n", tag,
                  moved, delta_counts);
    return true;
  }
  return false;
}

// Shared JOG path for Serial and touch. Returns a short result for the LCD.
const char* jog(int delta_counts) {
  Serial.printf("JOG %+d\n", delta_counts);
  if (delta_counts == 0 ||
      abs(delta_counts) > STS_TEST_JOG_MAX_DELTA_COUNTS) {
    Serial.printf("JOG REJECTED: delta must be 1..%d counts\n",
                  STS_TEST_JOG_MAX_DELTA_COUNTS);
    return "REJECT: delta";
  }

  // Communication and servo condition checks. Any failure: do not move.
  if (!ping(false)) {
    Serial.println("JOG ABORTED: PING failed (no motion)");
    printLastError();
    return "ABORT: no PING";
  }
  const uint8_t state = servo.getState();
  if (state != 0) {
    Serial.println("JOG ABORTED: servo reports error flags (no motion)");
    printServoState(state);
    return "ABORT: servo error";
  }
  const int mode = servo.readByte(servo_id, SMS_STS_MODE);
  if (mode != STS_TEST_MODE_POSITION) {
    Serial.printf("JOG ABORTED: mode=%d is not position mode (no motion)\n", mode);
    return "ABORT: mode";
  }
  // In multi-turn the servo's internal position can differ from the wrapped
  // Present Position by whole turns, so a raw-based hold write could turn it.
  bool multi_turn = false;
  if (!isMultiTurn(multi_turn)) {
    Serial.println("JOG ABORTED: angle limit read failed (no motion)");
    return "ABORT: limit read";
  }
  if (multi_turn) {
    Serial.println("JOG REJECTED: multi-turn is ON (use MT_JOG or MT_OFF)");
    return "REJECT: MT on";
  }

  // 1-2. Read current position; abort on failure.
  int current_counts = 0;
  if (!readPosition(current_counts)) {
    Serial.println("JOG ABORTED: position read failed (no motion)");
    printLastError();
    return "ABORT: read";
  }
  Serial.printf("Current: %d\n", current_counts);

  // 3-4. Compute and range-check target.
  const int target_counts = current_counts + delta_counts;
  Serial.printf("Target: %d\n", target_counts);
  if (target_counts < STS_TEST_JOG_TARGET_MIN_COUNTS ||
      target_counts > STS_TEST_JOG_TARGET_MAX_COUNTS) {
    Serial.printf("JOG REJECTED: target outside %d..%d (no motion)\n",
                  STS_TEST_JOG_TARGET_MIN_COUNTS, STS_TEST_JOG_TARGET_MAX_COUNTS);
    return "REJECT: range";
  }
  const int min_limit = servo.readWord(servo_id, SMS_STS_MIN_ANGLE_LIMIT_L);
  const int max_limit = servo.readWord(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (min_limit < 0 || max_limit < 0) {
    Serial.println("JOG ABORTED: angle limit read failed (no motion)");
    return "ABORT: limit read";
  }
  // 0/0 means limits disabled (multi-turn); otherwise respect them.
  if (!(min_limit == 0 && max_limit == 0) &&
      (target_counts < min_limit || target_counts > max_limit)) {
    Serial.printf("JOG REJECTED: target outside servo limits %d..%d (no motion)\n",
                  min_limit, max_limit);
    return "REJECT: limits";
  }

  // 5. Hold the present position as goal before torque is enabled so that a
  // stale goal register can never cause a jump, then command the target.
  if (!holdAndVerify("JOG", current_counts, STS_TEST_JOG_SPEED_COUNTS_PER_S,
                     STS_TEST_JOG_ACC, current_counts)) {
    return "ABORT: hold check";
  }
  if (!servo.WritePosEx(servo_id, target_counts,
                        STS_TEST_JOG_SPEED_COUNTS_PER_S, STS_TEST_JOG_ACC)) {
    Serial.println("Write command: FAILED");
    printLastError();
    return "ABORT: write";
  }
  Serial.println("Write command: OK");

  // 6. Poll feedback until arrival or timeout.
  const uint32_t started_ms = millis();
  int actual_counts = -1;
  bool arrived = false;
  while (millis() - started_ms < STS_TEST_JOG_TIMEOUT_MS) {
    delay(STS_TEST_JOG_POLL_INTERVAL_MS);
    int position_counts = 0;
    if (!readPosition(position_counts)) continue;
    actual_counts = position_counts;
    if (runawayDetected("JOG", current_counts, actual_counts, delta_counts)) {
      return "ABORT: runaway";
    }
    if (abs(actual_counts - target_counts) <= STS_TEST_JOG_TOLERANCE_COUNTS &&
        servo.ReadMove(servo_id) == 0) {
      arrived = true;
      break;
    }
  }
  const uint32_t elapsed_ms = millis() - started_ms;

  // 7. Report commanded vs actual.
  if (actual_counts < 0) {
    Serial.println("Actual: read failed");
  } else {
    Serial.printf("Actual: %d (error %+d, %lu ms)\n", actual_counts,
                  actual_counts - target_counts,
                  static_cast<unsigned long>(elapsed_ms));
  }
  if (arrived) {
    Serial.println("Result: PASS");
    return "PASS";
  }
  Serial.printf("Result: TIMEOUT (not within %d counts in %lu ms)\n",
                STS_TEST_JOG_TOLERANCE_COUNTS,
                static_cast<unsigned long>(STS_TEST_JOG_TIMEOUT_MS));
  Serial.println("  Torque is still enabled. Use OFF if needed.");
  return "TIMEOUT";
}

void runJog(int delta_counts) {
  const char* result = jog(delta_counts);
  snprintf(last_result, sizeof(last_result), "JOG %+d %s", delta_counts, result);
}

void cmdJog(const char* arg) {
  int delta_counts = 0;
  if (!parseDelta(arg, delta_counts)) {
    Serial.println("JOG REJECTED: usage JOG +30 / JOG -30");
    return;
  }
  runJog(delta_counts);
}

// Common pre-motion/pre-write checks. Prints the reason and returns false on
// any failure.
bool checkServoReady(const char* tag) {
  if (!ping(false)) {
    Serial.printf("%s ABORTED: PING failed\n", tag);
    printLastError();
    return false;
  }
  const uint8_t state = servo.getState();
  if (state != 0) {
    Serial.printf("%s ABORTED: servo reports error flags\n", tag);
    printServoState(state);
    return false;
  }
  const int mode = servo.readByte(servo_id, SMS_STS_MODE);
  if (mode != STS_TEST_MODE_POSITION) {
    Serial.printf("%s ABORTED: mode=%d is not position mode\n", tag, mode);
    return false;
  }
  return true;
}

bool writeMaxAngleLimit(int max_counts) {
  // Torque off first so a limit change can never trigger motion.
  if (!servo.EnableTorque(servo_id, 0)) {
    Serial.println("  Torque disable FAILED");
    printLastError();
    return false;
  }
  Serial.println("  Torque disabled before limit change");
  if (!servo.writeWord(servo_id, SMS_STS_MAX_ANGLE_LIMIT_L,
                       static_cast<uint16_t>(max_counts))) {
    Serial.println("  Max limit write FAILED");
    printLastError();
    return false;
  }
  return true;
}

void printLimitsAfterWrite() {
  int min_counts = 0;
  int max_counts = 0;
  if (readAngleLimits(min_counts, max_counts)) {
    Serial.printf("  Limits now: %d..%d (%s)\n", min_counts, max_counts,
                  (min_counts == 0 && max_counts == 0) ? "multi-turn" : "single-turn");
  } else {
    Serial.println("  Limits read-back FAILED");
  }
}

void cmdMtOn() {
  Serial.println("MT_ON");
  if (!checkServoReady("MT_ON")) {
    snprintf(last_result, sizeof(last_result), "MT_ON ABORT");
    return;
  }
  // Keep the change temporary: refuse if the EEPROM lock is off.
  const int lock = servo.readByte(servo_id, SMS_STS_LOCK);
  if (lock != STS_TEST_REG_LOCK_LOCKED) {
    Serial.printf("MT_ON REFUSED: EEPROM lock=%d (would persist)\n", lock);
    snprintf(last_result, sizeof(last_result), "MT_ON REFUSED lock");
    return;
  }
  int min_counts = 0;
  int max_counts = 0;
  if (!readAngleLimits(min_counts, max_counts)) {
    Serial.println("MT_ON ABORTED: limit read failed");
    snprintf(last_result, sizeof(last_result), "MT_ON ABORT");
    return;
  }
  if (min_counts == 0 && max_counts == 0) {
    Serial.println("Multi-turn already ON (run MT_OFF first to re-zero tracking)");
    snprintf(last_result, sizeof(last_result), "MT already ON");
    return;
  }
  // Only change a known baseline so MT_OFF restores exactly what was there.
  if (min_counts != STS_TEST_SINGLE_TURN_MIN_LIMIT_COUNTS ||
      max_counts != STS_TEST_SINGLE_TURN_MAX_LIMIT_COUNTS) {
    Serial.printf("MT_ON REFUSED: limits %d..%d are not the expected %d..%d\n",
                  min_counts, max_counts, STS_TEST_SINGLE_TURN_MIN_LIMIT_COUNTS,
                  STS_TEST_SINGLE_TURN_MAX_LIMIT_COUNTS);
    snprintf(last_result, sizeof(last_result), "MT_ON REFUSED limits");
    return;
  }
  // Measured 2026-10-18: the servo keeps a hidden multi-turn position across
  // MT_OFF/MT_ON that Present Position does not expose. If the goal register
  // does not match the present position, that hidden count may be off by whole
  // turns, and the next goal write would spin the servo. Require a servo power
  // cycle (assumed to reset the hidden count) in that case.
  int raw_before = 0;
  if (!readPosition(raw_before)) {
    Serial.println("MT_ON ABORTED: position read failed");
    snprintf(last_result, sizeof(last_result), "MT_ON ABORT");
    return;
  }
  const int goal_register = servo.readWord(servo_id, SMS_STS_GOAL_POSITION_L);
  const bool goal_ok = servo.getLastError() == 0;
  const int torque = servo.readByte(servo_id, SMS_STS_TORQUE_ENABLE);
  // Goal resets to 0 at servo power-up (measured). Torque off + goal 0 is
  // taken as power-up state; the hold check on the first MT_JOG verifies it.
  const bool power_up_state = goal_ok && goal_register == 0 && torque == 0;
  if (power_up_state) {
    Serial.println("  Power-up state (goal=0, torque off): hold check will verify");
  }
  if (!goal_ok ||
      (!power_up_state && abs(goal_register - raw_before) > STS_TEST_JOG_TOLERANCE_COUNTS)) {
    Serial.printf("MT_ON REFUSED: goal register %d != position %d\n", goal_register,
                  raw_before);
    Serial.println("  Hidden multi-turn count may be off by whole turns.");
    Serial.println("  Power-cycle the servo supply, then retry MT_ON.");
    snprintf(last_result, sizeof(last_result), "MT_ON REFUSED goal");
    return;
  }
  if (!writeMaxAngleLimit(0)) {
    snprintf(last_result, sizeof(last_result), "MT_ON FAILED");
    return;
  }
  printLimitsAfterWrite();
  int raw_counts = 0;
  if (!readPosition(raw_counts)) {
    Serial.println("  Position read failed: tracking not started (run MT_OFF)");
    snprintf(last_result, sizeof(last_result), "MT_ON no tracking");
    return;
  }
  mtStartTracking(raw_counts);
  Serial.printf("  Tracking zero: abs=%d\n", mt_abs_counts);
  Serial.println("Multi-turn: ON (EEPROM locked; expected NOT saved - verify by power cycle)");
  Serial.println("  Restore with MT_OFF.");
  snprintf(last_result, sizeof(last_result), "MT_ON OK");
}

void cmdMtOff() {
  Serial.println("MT_OFF");
  if (!ping(false)) {
    Serial.println("MT_OFF ABORTED: PING failed");
    printLastError();
    snprintf(last_result, sizeof(last_result), "MT_OFF ABORT");
    return;
  }
  if (!writeMaxAngleLimit(STS_TEST_SINGLE_TURN_MAX_LIMIT_COUNTS)) {
    snprintf(last_result, sizeof(last_result), "MT_OFF FAILED");
    return;
  }
  printLimitsAfterWrite();
  int position_counts = 0;
  if (readPositionAny(position_counts)) {
    Serial.printf("  Position: %d\n", position_counts);
  }
  mt_tracking_valid = false;
  Serial.println("Multi-turn: OFF");
  snprintf(last_result, sizeof(last_result), "MT_OFF OK");
}

const char* jogMultiTurn(int delta_counts) {
  Serial.printf("MT_JOG %+d\n", delta_counts);
  if (delta_counts == 0 || abs(delta_counts) > STS_TEST_MT_JOG_MAX_DELTA_COUNTS) {
    Serial.printf("MT_JOG REJECTED: delta must be 1..%d counts\n",
                  STS_TEST_MT_JOG_MAX_DELTA_COUNTS);
    return "REJECT: delta";
  }
  if (!checkServoReady("MT_JOG")) return "ABORT: not ready";
  bool multi_turn = false;
  if (!isMultiTurn(multi_turn)) {
    Serial.println("MT_JOG ABORTED: limit read failed (no motion)");
    return "ABORT: limit read";
  }
  if (!multi_turn) {
    Serial.println("MT_JOG REJECTED: multi-turn is OFF (run MT_ON first)");
    return "REJECT: MT off";
  }
  if (!mt_tracking_valid) {
    Serial.println("MT_JOG REJECTED: tracking invalid (run MT_OFF, then MT_ON)");
    return "REJECT: no tracking";
  }
  int raw_counts = 0;
  if (!readPosition(raw_counts)) {
    Serial.println("MT_JOG ABORTED: position read failed (no motion)");
    printLastError();
    return "ABORT: read";
  }
  // Idle drift must stay small; a large jump means the turn count is unknown.
  const int drift = wrapDeltaCounts(raw_counts - mt_last_raw_counts);
  if (abs(drift) > STS_TEST_JOG_TOLERANCE_COUNTS) {
    Serial.printf("MT_JOG ABORTED: moved %+d counts while idle; tracking invalidated\n",
                  drift);
    mt_tracking_valid = false;
    return "ABORT: drift";
  }
  mtUpdateTracking(raw_counts);
  const int current_counts = mt_abs_counts;
  const int target_counts = current_counts + delta_counts;
  const int goal_register = servo.readWord(servo_id, SMS_STS_GOAL_POSITION_L);
  Serial.printf("Current: %d (raw %d, goal register %d)\n", current_counts, raw_counts,
                goal_register);
  Serial.printf("Target: %d (%+.2f turn)\n", target_counts,
                static_cast<float>(delta_counts) / STS_TEST_COUNTS_PER_TURN);
  if (abs(target_counts) > STS_TEST_MT_POSITION_LIMIT_COUNTS) {
    Serial.printf("MT_JOG REJECTED: |target| > %d (no motion)\n",
                  STS_TEST_MT_POSITION_LIMIT_COUNTS);
    return "REJECT: range";
  }
  if (!holdAndVerify("MT_JOG", current_counts, STS_TEST_MT_SPEED_COUNTS_PER_S,
                     STS_TEST_MT_ACC, raw_counts)) {
    mt_tracking_valid = false;
    return "ABORT: hold check";
  }
  if (!servo.WritePosEx(servo_id, target_counts,
                        STS_TEST_MT_SPEED_COUNTS_PER_S, STS_TEST_MT_ACC)) {
    Serial.println("Write command: FAILED");
    printLastError();
    return "ABORT: write";
  }
  Serial.println("Write command: OK");

  const uint32_t timeout_ms =
      static_cast<uint32_t>(abs(delta_counts)) * 1000U / STS_TEST_MT_SPEED_COUNTS_PER_S +
      STS_TEST_MT_TIMEOUT_MARGIN_MS;
  const uint32_t started_ms = millis();
  uint32_t last_trace_ms = 0;
  uint32_t last_good_read_ms = started_ms;
  int actual_counts = current_counts;
  bool have_actual = false;
  bool arrived = false;
  while (millis() - started_ms < timeout_ms) {
    delay(STS_TEST_JOG_POLL_INTERVAL_MS);
    int position_counts = 0;
    if (!readPosition(position_counts)) {
      if (millis() - last_good_read_ms > STS_TEST_MT_MAX_READ_GAP_MS) {
        mt_tracking_valid = false;
        Serial.println("MT_JOG: feedback lost > read gap; tracking invalidated");
        Serial.println("  Servo continues to its goal. Use OFF if needed.");
        return "ABORT: feedback lost";
      }
      continue;
    }
    last_good_read_ms = millis();
    mtUpdateTracking(position_counts);
    actual_counts = mt_abs_counts;
    have_actual = true;
    if (runawayDetected("MT_JOG", current_counts, actual_counts, delta_counts)) {
      mt_tracking_valid = false;
      return "ABORT: runaway";
    }
    const uint32_t elapsed_ms = millis() - started_ms;
    if (elapsed_ms - last_trace_ms >= STS_TEST_MT_TRACE_INTERVAL_MS) {
      Serial.printf("  t=%5lu ms abs=%6d raw=%4d\n",
                    static_cast<unsigned long>(elapsed_ms), actual_counts, position_counts);
      last_trace_ms = elapsed_ms;
    }
    if (abs(actual_counts - target_counts) <= STS_TEST_JOG_TOLERANCE_COUNTS &&
        servo.ReadMove(servo_id) == 0) {
      arrived = true;
      break;
    }
  }
  const uint32_t elapsed_ms = millis() - started_ms;
  if (have_actual) {
    Serial.printf("Actual: %d (error %+d, %lu ms)\n", actual_counts,
                  actual_counts - target_counts, static_cast<unsigned long>(elapsed_ms));
  } else {
    Serial.println("Actual: read failed");
  }
  if (arrived) {
    Serial.println("Result: PASS");
    return "PASS";
  }
  Serial.printf("Result: TIMEOUT (not within %d counts in %lu ms)\n",
                STS_TEST_JOG_TOLERANCE_COUNTS, static_cast<unsigned long>(timeout_ms));
  Serial.println("  Torque is still enabled. Use OFF if needed.");
  return "TIMEOUT";
}

void cmdMtJog(const char* arg) {
  int delta_counts = 0;
  if (!parseDelta(arg, delta_counts)) {
    Serial.println("MT_JOG REJECTED: usage MT_JOG +1024 / MT_JOG -1024");
    return;
  }
  const char* result = jogMultiTurn(delta_counts);
  snprintf(last_result, sizeof(last_result), "MT %+d %s", delta_counts, result);
}

bool parseId(const char* text, uint8_t& id) {
  int value = 0;
  if (!parseDelta(text, value) || value < STS_TEST_SERVO_ID_MIN ||
      value > STS_TEST_SERVO_ID_MAX) {
    return false;
  }
  id = static_cast<uint8_t>(value);
  return true;
}

void cmdScan() {
  Serial.printf("SCAN IDs %u..%u\n", STS_TEST_SERVO_ID_MIN, STS_TEST_SCAN_ID_LAST);
  int found = 0;
  for (int id = STS_TEST_SERVO_ID_MIN; id <= STS_TEST_SCAN_ID_LAST; ++id) {
    const int reply = servo.Ping(static_cast<uint8_t>(id));
    if (reply == id) {
      Serial.printf("  ID=%d: found\n", id);
      ++found;
    } else if (servo.getLastError() != ERR_NO_REPLY) {
      // Corrupted replies usually mean two servos share this ID.
      Serial.printf("  ID=%d: bad reply (%s) - possible duplicate ID\n", id,
                    errorName(servo.getLastError()));
    }
  }
  Serial.printf("SCAN done: %d servo(s) found\n", found);
  Serial.println("  Duplicate IDs collide; SCAN cannot count them reliably.");
  snprintf(last_result, sizeof(last_result), "SCAN %d found", found);
}

void cmdId(const char* arg) {
  uint8_t id = 0;
  if (!parseId(arg, id)) {
    Serial.printf("ID: selected %u (usage ID <%u..%u>)\n", servo_id,
                  STS_TEST_SERVO_ID_MIN, STS_TEST_SERVO_ID_MAX);
    return;
  }
  if (id != servo_id && mt_tracking_valid) {
    mt_tracking_valid = false;
    Serial.println("  Multi-turn tracking invalidated (servo changed)");
  }
  servo_id = id;
  Serial.printf("Selected servo ID=%u\n", servo_id);
  ping(true);
  snprintf(last_result, sizeof(last_result), "ID %u", servo_id);
}

void cmdSetId(const char* arg) {
  uint8_t new_id = 0;
  if (!parseId(arg, new_id)) {
    Serial.printf("SET_ID REJECTED: usage SET_ID <%u..%u>\n", STS_TEST_SERVO_ID_MIN,
                  STS_TEST_SERVO_ID_MAX);
    return;
  }
  Serial.printf("SET_ID %u -> %u\n", servo_id, new_id);
  if (new_id == servo_id) {
    Serial.println("SET_ID REJECTED: same ID");
    return;
  }
  // Only one servo may answer: repeated clean PINGs and a matching broadcast.
  for (int i = 0; i < STS_TEST_SET_ID_PING_REPEAT; ++i) {
    if (servo.Ping(servo_id) != servo_id) {
      Serial.printf("SET_ID REFUSED: PING %d/%d failed (%s)\n", i + 1,
                    STS_TEST_SET_ID_PING_REPEAT, errorName(servo.getLastError()));
      Serial.println("  Disconnect all other servos from the bus and retry.");
      snprintf(last_result, sizeof(last_result), "SET_ID REFUSED");
      return;
    }
  }
  if (servo.Ping(STS_TEST_BROADCAST_ID) != servo_id) {
    Serial.println("SET_ID REFUSED: broadcast PING did not cleanly return this ID");
    Serial.println("  Disconnect all other servos from the bus and retry.");
    snprintf(last_result, sizeof(last_result), "SET_ID REFUSED");
    return;
  }
  if (servo.Ping(new_id) >= 0 || servo.getLastError() != ERR_NO_REPLY) {
    Serial.printf("SET_ID REFUSED: something already answers ID=%u\n", new_id);
    snprintf(last_result, sizeof(last_result), "SET_ID REFUSED");
    return;
  }
  if (!servo.EnableTorque(servo_id, 0)) {
    Serial.println("SET_ID ABORTED: torque disable failed");
    printLastError();
    return;
  }
  if (!servo.unLockEprom(servo_id)) {
    Serial.println("SET_ID ABORTED: EEPROM unlock failed");
    printLastError();
    return;
  }
  // The reply to this write comes from the new ID, so the ACK may not match.
  servo.writeByte(servo_id, SMS_STS_ID, new_id);
  delay(20);
  const bool new_ok = servo.Ping(new_id) == new_id;
  const bool locked = new_ok ? servo.LockEprom(new_id) != 0
                             : servo.LockEprom(servo_id) != 0;
  if (!new_ok) {
    Serial.printf("SET_ID FAILED: no reply from new ID=%u (EEPROM relock %s)\n", new_id,
                  locked ? "OK" : "FAILED");
    snprintf(last_result, sizeof(last_result), "SET_ID FAILED");
    return;
  }
  Serial.printf("SET_ID OK: servo now answers ID=%u (EEPROM relock %s)\n", new_id,
                locked ? "OK" : "FAILED");
  mt_tracking_valid = false;
  servo_id = new_id;
  Serial.printf("Selected servo ID=%u\n", servo_id);
  snprintf(last_result, sizeof(last_result), "SET_ID %u OK", new_id);
}

void handleLine(char* line) {
  // Uppercase command word; split off the first argument.
  char* arg = strchr(line, ' ');
  if (arg != nullptr) {
    *arg++ = '\0';
    while (*arg == ' ') ++arg;
    char* tail = arg + strlen(arg);
    while (tail > arg && tail[-1] == ' ') *--tail = '\0';
  }
  for (char* p = line; *p; ++p) *p = static_cast<char>(toupper(*p));

  if (strcmp(line, "HELP") == 0) cmdHelp();
  else if (strcmp(line, "PING") == 0) cmdPing();
  else if (strcmp(line, "READ") == 0) cmdRead();
  else if (strcmp(line, "JOG") == 0) cmdJog(arg);
  else if (strcmp(line, "OFF") == 0) cmdOff();
  else if (strcmp(line, "STATUS") == 0) cmdStatus();
  else if (strcmp(line, "SCAN") == 0) cmdScan();
  else if (strcmp(line, "ID") == 0) cmdId(arg);
  else if (strcmp(line, "SET_ID") == 0) cmdSetId(arg);
  else if (strcmp(line, "MT_ON") == 0) cmdMtOn();
  else if (strcmp(line, "MT_OFF") == 0) cmdMtOff();
  else if (strcmp(line, "MT_JOG") == 0) cmdMtJog(arg);
  else Serial.printf("ERROR: unknown command '%s' (try HELP)\n", line);
  Serial.println();
}

void drawInfo();

void pollUsbSerial() {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (line_length == 0) continue;
      line_buffer[line_length] = '\0';
      line_length = 0;
      handleLine(line_buffer);
      drawInfo();
    } else if (line_length + 1 < LINE_BUFFER_SIZE) {
      line_buffer[line_length++] = c;
    }
  }
}

// ---------------------------------------------------------------------------
// Touch UI
// ---------------------------------------------------------------------------
// One tap = one command. Taps made while a command runs are discarded.

enum class UiAction : uint8_t { Jog, Read, Off };

struct UiButton {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
  UiAction action;
  int delta_counts;
  const char* label;
};

constexpr int16_t UI_INFO_Y = 24;
constexpr int16_t UI_INFO_LINE_H = 20;
constexpr int16_t UI_INFO_H = UI_INFO_LINE_H * 4;

const UiButton UI_BUTTONS[] = {
    {4, 110, 74, 58, UiAction::Jog, -STS_TEST_TOUCH_LARGE_STEP_COUNTS, "-100"},
    {84, 110, 74, 58, UiAction::Jog, -STS_TEST_TOUCH_SMALL_STEP_COUNTS, "-30"},
    {164, 110, 74, 58, UiAction::Jog, STS_TEST_TOUCH_SMALL_STEP_COUNTS, "+30"},
    {244, 110, 72, 58, UiAction::Jog, STS_TEST_TOUCH_LARGE_STEP_COUNTS, "+100"},
    {4, 176, 152, 58, UiAction::Read, 0, "READ"},
    {164, 176, 152, 58, UiAction::Off, 0, "OFF"},
};
static_assert(STS_TEST_TOUCH_SMALL_STEP_COUNTS == 30 &&
                  STS_TEST_TOUCH_LARGE_STEP_COUNTS == 100,
              "Update UI_BUTTONS labels when touch step sizes change");
static_assert(STS_TEST_TOUCH_LARGE_STEP_COUNTS <= STS_TEST_JOG_MAX_DELTA_COUNTS,
              "Touch step must stay within the JOG delta limit");

// Core2 touch zones below the LCD (BtnA/B/C).
const UiButton& UI_BTN_A = UI_BUTTONS[1];  // -30
const UiButton& UI_BTN_B = UI_BUTTONS[5];  // OFF
const UiButton& UI_BTN_C = UI_BUTTONS[2];  // +30

uint32_t ui_last_refresh_ms = 0;

void drawButton(const UiButton& button, bool busy) {
  uint16_t color = TFT_DARKGREY;
  if (button.action == UiAction::Off) color = TFT_RED;
  if (button.action == UiAction::Read) color = TFT_NAVY;
  if (busy) color = TFT_ORANGE;
  M5.Display.fillRoundRect(button.x, button.y, button.w, button.h, 8, color);
  M5.Display.setTextColor(TFT_WHITE, color);
  M5.Display.setTextDatum(middle_center);
  M5.Display.setTextSize(3);
  M5.Display.drawString(button.label, button.x + button.w / 2,
                        button.y + button.h / 2);
  M5.Display.setTextDatum(top_left);
}

void drawInfo() {
  int position_counts = -1;
  const bool link = ping(false);
  const uint8_t flags = link ? servo.getState() : 0;
  bool position_ok = link && readPositionAny(position_counts);
  bool multi_turn = false;
  const bool multi_turn_ok = link && isMultiTurn(multi_turn);
  // Show the unwrapped multi-turn position while it is being tracked.
  if (mt_tracking_valid) {
    position_counts = mt_abs_counts;
    position_ok = true;
  }
  const int voltage_dv = link ? servo.ReadVoltage(servo_id) : -1;
  const int torque = link ? servo.readByte(servo_id, SMS_STS_TORQUE_ENABLE) : -1;

  M5.Display.fillRect(0, UI_INFO_Y, M5.Display.width(), UI_INFO_H, TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_WHITE, TFT_BLACK);
  int16_t y = UI_INFO_Y;
  M5.Display.setCursor(4, y);
  if (position_ok) {
    M5.Display.printf("Pos:%6d %s Torque:%s", position_counts,
                      (multi_turn_ok && multi_turn) ? "MT" : "  ",
                      torque == 1 ? "ON" : (torque == 0 ? "OFF" : "?"));
  } else {
    M5.Display.print("Pos: ----  Torque: ?");
  }
  y += UI_INFO_LINE_H;
  M5.Display.setCursor(4, y);
  M5.Display.setTextColor(link ? TFT_GREEN : TFT_RED, TFT_BLACK);
  if (voltage_dv >= 0) {
    M5.Display.printf("ID%u Link:%s Volt:%.1fV", servo_id, link ? "OK" : "NG",
                      voltage_dv * 0.1f);
  } else {
    M5.Display.printf("ID%u Link:%s Volt:--", servo_id, link ? "OK" : "NG");
  }
  y += UI_INFO_LINE_H;
  M5.Display.setCursor(4, y);
  M5.Display.setTextColor(flags == 0 ? TFT_WHITE : TFT_RED, TFT_BLACK);
  if (flags == 0) {
    M5.Display.print("Flags: none");
  } else {
    M5.Display.printf("Flags: 0x%02X%s", flags, (flags & 0x01) ? " VOLTAGE" : "");
  }
  y += UI_INFO_LINE_H;
  M5.Display.setCursor(4, y);
  M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Display.printf("Last: %s", last_result);
  ui_last_refresh_ms = millis();
}

void drawScreen() {
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextSize(2);
  M5.Display.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Display.setCursor(4, 2);
  M5.Display.print("STS3020 Touch Test");
  for (const UiButton& button : UI_BUTTONS) drawButton(button, false);
  drawInfo();
}

void runUiButton(const UiButton& button) {
  Serial.printf("[TOUCH] %s\n", button.label);
  drawButton(button, true);
  switch (button.action) {
    case UiAction::Jog: runJog(button.delta_counts); break;
    case UiAction::Read: cmdRead(); snprintf(last_result, sizeof(last_result), "READ"); break;
    case UiAction::Off: cmdOff(); break;
  }
  Serial.println();
  drawButton(button, false);
  drawInfo();
  // Consume touch state accumulated during the blocking command so a tap made
  // mid-move does not queue another move.
  M5.update();
}

const UiButton* hitTest(int16_t x, int16_t y) {
  for (const UiButton& button : UI_BUTTONS) {
    if (x >= button.x && x < button.x + button.w &&
        y >= button.y && y < button.y + button.h) {
      return &button;
    }
  }
  return nullptr;
}

void pollTouch() {
  const UiButton* pressed = nullptr;
  if (M5.Touch.getCount() > 0) {
    const auto detail = M5.Touch.getDetail();
    if (detail.wasPressed()) pressed = hitTest(detail.x, detail.y);
  }
  if (pressed == nullptr) {
    if (M5.BtnA.wasPressed()) pressed = &UI_BTN_A;
    else if (M5.BtnB.wasPressed()) pressed = &UI_BTN_B;
    else if (M5.BtnC.wasPressed()) pressed = &UI_BTN_C;
  }
  if (pressed != nullptr) {
    runUiButton(*pressed);
  } else if (millis() - ui_last_refresh_ms >= STS_TEST_UI_REFRESH_INTERVAL_MS) {
    drawInfo();
  }
}

}  // namespace

void setup() {
  auto cfg = M5.config();
  cfg.serial_baudrate = STS_TEST_USB_SERIAL_BAUD;
  M5.begin(cfg);

  delay(500);
  Serial.println();
  Serial.println("=== STS3020 Motor Test ===");
  Serial.println();
  Serial.println("Board: M5Stack Core2");
  Serial.printf("UART: Serial%u\n", STS_TEST_UART_NUM);
  Serial.printf("TX: GPIO%u\n", STS_TEST_UART_TX_PIN);
  Serial.printf("RX: GPIO%u\n", STS_TEST_UART_RX_PIN);
  Serial.printf("Baud: %lu\n", static_cast<unsigned long>(STS_TEST_UART_BAUD));
  Serial.printf("Servo ID: %u\n", servo_id);
  Serial.println();

  servo_serial.begin(STS_TEST_UART_BAUD, SERIAL_8N1, STS_TEST_UART_RX_PIN,
                     STS_TEST_UART_TX_PIN);
  servo.pSerial = &servo_serial;
  servo.IOTimeOut = STS_TEST_UART_IO_TIMEOUT_MS;

  // Boot: communication check only. No position or torque command is sent.
  cmdPing();
  Serial.println();
  Serial.println("Servo is not moved at boot. Type HELP for commands.");
  Serial.println("Touch UI: -100 / -30 / +30 / +100 / READ / OFF");
  Serial.println("  BtnA=-30  BtnB=OFF  BtnC=+30");
  Serial.println();
  drawScreen();
}

void loop() {
  M5.update();
  pollUsbSerial();
  pollTouch();
  delay(5);
}
