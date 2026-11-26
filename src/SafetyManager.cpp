#include "SafetyManager.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "AppContext.h"
#include "Core2PinMap.h"
#include "PlotterConfig.h"

void SafetyManager::begin() {
  pinMode(X_LIMIT_PIN, INPUT);
  pinMode(Y_LIMIT_PIN, INPUT);
  x_last_raw_ = readLimitPin(X_LIMIT_PIN);
  y_last_raw_ = readLimitPin(Y_LIMIT_PIN);
  x_debounced_ = x_last_raw_;
  y_debounced_ = y_last_raw_;
  x_last_change_ms_ = millis();
  y_last_change_ms_ = millis();
}

bool SafetyManager::validateMove(float target_x_mm, float target_y_mm,
                                 float& feed_mm_min) const {
  if (alarmed_) {
    logMessage("REJECT: machine is alarmed reason=%s", alarm_reason_);
    return false;
  }
  const bool has_homed_motion_reference =
      machine_state.homed || job_controller.hasHomedJobMotionGrant();
  if (HOMING_REQUIRE_HOMED_FOR_XY_MOVE && !has_homed_motion_reference) {
    logMessage("REJECT: machine is not homed");
    return false;
  }
  const float dx_mm = target_x_mm - machine_state.x_mm;
  const float dy_mm = target_y_mm - machine_state.y_mm;
  if (x_debounced_ && dx_mm * static_cast<float>(HOMING_X_DIR) > 0.0f) {
    logMessage("REJECT: X limit active and move pushes toward limit");
    return false;
  }
  if (y_debounced_ && dy_mm * static_cast<float>(HOMING_Y_DIR) > 0.0f) {
    logMessage("REJECT: Y limit active and move pushes toward limit");
    return false;
  }
  if (target_x_mm < X_MIN_MM || target_x_mm > X_MAX_MM) {
    logMessage("REJECT: X %.3f outside [%.3f, %.3f]", target_x_mm, X_MIN_MM,
               X_MAX_MM);
    return false;
  }
  if (target_y_mm < Y_MIN_MM || target_y_mm > Y_MAX_MM) {
    logMessage("REJECT: Y %.3f outside [%.3f, %.3f]", target_y_mm, Y_MIN_MM,
               Y_MAX_MM);
    return false;
  }
  if (feed_mm_min <= 0.0f) {
    logMessage("REJECT: feed must be > 0");
    return false;
  }
  if (feed_mm_min > MAX_FEED_MM_MIN) {
    logMessage("CLAMP: feed %.3f -> %.3f", feed_mm_min, MAX_FEED_MM_MIN);
    feed_mm_min = MAX_FEED_MM_MIN;
  }
  return true;
}

bool SafetyManager::validateHomingStart() const {
  if (!HOMING_ENABLED) {
    logMessage("REJECT: homing disabled by config");
    return false;
  }
  if (alarmed_) {
    logMessage("REJECT: machine is alarmed reason=%s", alarm_reason_);
    return false;
  }
  return true;
}

bool SafetyManager::readLimitPin(int pin) const {
  const int level = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? level == LOW : level == HIGH;
}

bool SafetyManager::xLimitRawActive() const { return readLimitPin(X_LIMIT_PIN); }
bool SafetyManager::yLimitRawActive() const { return readLimitPin(Y_LIMIT_PIN); }
bool SafetyManager::xLimitActive() const { return x_debounced_; }
bool SafetyManager::yLimitActive() const { return y_debounced_; }
bool SafetyManager::isAlarmed() const { return alarmed_; }
void SafetyManager::setAlarm(bool alarmed) {
  alarmed_ = alarmed;
  if (!alarmed_) {
    strncpy(alarm_reason_, "none", sizeof(alarm_reason_) - 1);
    alarm_reason_[sizeof(alarm_reason_) - 1] = '\0';
  }
}

void SafetyManager::setAlarm(const char* reason) {
  alarmed_ = true;
  strncpy(alarm_reason_, reason, sizeof(alarm_reason_) - 1);
  alarm_reason_[sizeof(alarm_reason_) - 1] = '\0';
}

void SafetyManager::clearAlarm() { setAlarm(false); }
const char* SafetyManager::alarmReason() const { return alarm_reason_; }
void SafetyManager::setHomingActive(bool active) { homing_active_ = active; }
void SafetyManager::beginNormalMoveLimitReleaseAllowance(bool allow_x,
                                                         bool allow_y,
                                                         float start_x_mm,
                                                         float start_y_mm) {
  x_release_allowed_ = allow_x;
  y_release_allowed_ = allow_y;
  x_release_start_mm_ = start_x_mm;
  y_release_start_mm_ = start_y_mm;
}

void SafetyManager::clearNormalMoveLimitReleaseAllowance() {
  x_release_allowed_ = false;
  y_release_allowed_ = false;
}

bool SafetyManager::updateDebounced(bool raw_active, bool& last_raw,
                                    bool& debounced,
                                    uint32_t& last_change_ms) {
  const uint32_t now_ms = millis();
  if (raw_active != last_raw) {
    last_raw = raw_active;
    last_change_ms = now_ms;
  }
  if (now_ms - last_change_ms >= HOMING_LIMIT_DEBOUNCE_MS) {
    debounced = raw_active;
  }
  return debounced;
}

void SafetyManager::poll() {
  updateDebounced(xLimitRawActive(), x_last_raw_, x_debounced_,
                  x_last_change_ms_);
  updateDebounced(yLimitRawActive(), y_last_raw_, y_debounced_,
                  y_last_change_ms_);
  if (x_release_allowed_ && !x_debounced_ && !xLimitRawActive()) {
    x_release_allowed_ = false;
    x_unexpected_since_ms_ = 0;
  }
  if (y_release_allowed_ && !y_debounced_ && !yLimitRawActive()) {
    y_release_allowed_ = false;
    y_unexpected_since_ms_ = 0;
  }
  if (!homing_active_ && machine_state.homed) {
    const uint32_t now_ms = millis();
    if (x_release_allowed_ && x_debounced_ &&
        fabsf(machine_state.x_mm - x_release_start_mm_) >=
            NORMAL_MOVE_LIMIT_RELEASE_MM) {
      setAlarm("X home limit did not release");
      return;
    }
    if (y_release_allowed_ && y_debounced_ &&
        fabsf(machine_state.y_mm - y_release_start_mm_) >=
            NORMAL_MOVE_LIMIT_RELEASE_MM) {
      setAlarm("Y home limit did not release");
      return;
    }
    // #32: a home switch ON near the home edge is the end of the stage, not
    // an alarm; the motion paths stop the move toward it.
    if (!pollHomeEdge(x_edge_, 'X', x_debounced_, xLimitRawActive(),
                      machine_state.x_mm, HOMING_SET_X_MM, HOMING_X_DIR) ||
        !pollHomeEdge(y_edge_, 'Y', y_debounced_, yLimitRawActive(),
                      machine_state.y_mm, HOMING_SET_Y_MM, HOMING_Y_DIR)) {
      return;
    }
    const bool x_unexpected =
        !x_release_allowed_ &&
        x_edge_.unexpected(x_debounced_, machine_state.x_mm, HOMING_SET_X_MM,
                           HOMING_X_DIR, HOME_EDGE_LIMIT_ZONE_MM);
    const bool y_unexpected =
        !y_release_allowed_ &&
        y_edge_.unexpected(y_debounced_, machine_state.y_mm, HOMING_SET_Y_MM,
                           HOMING_Y_DIR, HOME_EDGE_LIMIT_ZONE_MM);
    if (x_unexpected && x_unexpected_since_ms_ == 0) {
      x_unexpected_since_ms_ = now_ms;
    } else if (!x_unexpected) {
      x_unexpected_since_ms_ = 0;
    }
    if (y_unexpected && y_unexpected_since_ms_ == 0) {
      y_unexpected_since_ms_ = now_ms;
    } else if (!y_unexpected) {
      y_unexpected_since_ms_ = 0;
    }
    const bool x_unexpected_persistent =
        x_unexpected_since_ms_ != 0 &&
        now_ms - x_unexpected_since_ms_ >= HARD_LIMIT_UNEXPECTED_ALARM_MS;
    const bool y_unexpected_persistent =
        y_unexpected_since_ms_ != 0 &&
        now_ms - y_unexpected_since_ms_ >= HARD_LIMIT_UNEXPECTED_ALARM_MS;
    if (x_unexpected_persistent || y_unexpected_persistent) {
      char reason[96];
      snprintf(reason, sizeof(reason),
               "hard limit active away from home x=%s y=%s x_raw=%s y_raw=%s",
               x_debounced_ ? "ON" : "OFF", y_debounced_ ? "ON" : "OFF",
               xLimitRawActive() ? "ON" : "OFF",
               yLimitRawActive() ? "ON" : "OFF");
      setAlarm(reason);
    }
  } else {
    x_unexpected_since_ms_ = 0;
    y_unexpected_since_ms_ = 0;
    x_edge_.reset();
    y_edge_.reset();
  }
}

// Returns false when it raised the alarm.
bool SafetyManager::pollHomeEdge(HomeEdgeLatch& latch, char axis, bool limit_on,
                                 bool limit_raw_on, float pos_mm, float home_mm,
                                 int8_t home_dir) {
  const float touch_mm = latch.touchMm();
  switch (latch.update(limit_on, limit_raw_on, pos_mm, home_mm, home_dir,
                       HOME_EDGE_LIMIT_ZONE_MM, NORMAL_MOVE_LIMIT_RELEASE_MM)) {
    case HomeEdgeEvent::TOUCHED:
      logMessage("LIMIT edge axis=%c %c=%.3f (no alarm)", axis, axis, pos_mm);
      return true;
    case HomeEdgeEvent::RELEASED:
      logMessage("LIMIT edge released axis=%c %c=%.3f moved=%.3f", axis, axis,
                 pos_mm, fabsf(pos_mm - touch_mm));
      return true;
    case HomeEdgeEvent::NOT_RELEASED:
      setAlarm(axis == 'X' ? "X home limit did not release"
                           : "Y home limit did not release");
      return false;
    case HomeEdgeEvent::NONE:
    default:
      return true;
  }
}
