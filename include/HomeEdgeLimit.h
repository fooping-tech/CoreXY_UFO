#pragma once

#include <math.h>
#include <stdint.h>

// Home-side limit switch near the home edge (#32).
//
// A home switch that turns ON within edge_zone_mm of the home position of its
// axis means the carriage touched the end of the stage (e.g. a fast jog that
// overshot the 0 point by a little): the motion toward the switch stops, there
// is no alarm, HOMED stays and the position is not re-zeroed. Farther from
// home it is a lost position / collision and SafetyManager raises the alarm.
//
// Pure logic, no Arduino dependency: built by the native tests.

// Distance from the home position toward the inside of the stage [mm].
// home_dir is HOMING_X_DIR / HOMING_Y_DIR (-1: the switch is at the minimum).
// 0 at home, positive on the stage, negative past the home position.
inline float homeEdgeDistanceMm(float pos_mm, float home_mm, int8_t home_dir) {
  return (pos_mm - home_mm) * static_cast<float>(-home_dir);
}

// True when a home switch that is ON at pos_mm counts as touching the edge.
inline bool homeEdgeWithinZone(float pos_mm, float home_mm, int8_t home_dir,
                               float edge_zone_mm) {
  return homeEdgeDistanceMm(pos_mm, home_mm, home_dir) <= edge_zone_mm;
}

enum class HomeEdgeEvent : uint8_t {
  NONE,
  TOUCHED,       // switch came ON within the zone: edge, no alarm
  RELEASED,      // the switch went OFF again
  NOT_RELEASED,  // still ON after moving release_mm from the touch point: alarm
};

// Per-axis state of an edge touch. update() runs on every SafetyManager poll
// while the machine is homed and not homing; reset() otherwise.
class HomeEdgeLatch {
 public:
  void reset() { latched_ = false; }
  bool latched() const { return latched_; }
  float touchMm() const { return touch_mm_; }

  // limit_on: debounced switch state; limit_raw_on: the raw pin.
  HomeEdgeEvent update(bool limit_on, bool limit_raw_on, float pos_mm,
                       float home_mm, int8_t home_dir, float edge_zone_mm,
                       float release_mm) {
    if (latched_) {
      if (!limit_on && !limit_raw_on) {
        latched_ = false;
        return HomeEdgeEvent::RELEASED;
      }
      if (limit_on && fabsf(pos_mm - touch_mm_) >= release_mm) {
        return HomeEdgeEvent::NOT_RELEASED;
      }
      return HomeEdgeEvent::NONE;
    }
    if (limit_on && homeEdgeWithinZone(pos_mm, home_mm, home_dir, edge_zone_mm)) {
      latched_ = true;
      touch_mm_ = pos_mm;
      return HomeEdgeEvent::TOUCHED;
    }
    return HomeEdgeEvent::NONE;
  }

  // The switch is ON away from home (outside the zone, no edge touch):
  // SafetyManager alarms when this lasts HARD_LIMIT_UNEXPECTED_ALARM_MS.
  bool unexpected(bool limit_on, float pos_mm, float home_mm, int8_t home_dir,
                  float edge_zone_mm) const {
    return limit_on && !latched_ &&
           !homeEdgeWithinZone(pos_mm, home_mm, home_dir, edge_zone_mm);
  }

 private:
  bool latched_ = false;
  float touch_mm_ = 0.0f;
};

// A latch event waiting to be logged. SafetyManager::poll() also runs in
// safetyTask, whose 2 KB stack overflows on logMessage()/float formatting
// (2026-11-27: Double exception in _dtoa_r when the X switch was touched).
// poll() only posts here; motionTask (8 KB) takes and logs it.
struct HomeEdgeReport {
  HomeEdgeEvent event = HomeEdgeEvent::NONE;
  int32_t pos_um = 0;
  int32_t moved_um = 0;
};

class HomeEdgeReportSlot {
 public:
  // No formatting, no allocation: safe from any task. A newer event replaces
  // one that was not taken yet.
  void post(HomeEdgeEvent event, float pos_mm, float moved_mm) {
    pending_ = false;
    report_.event = event;
    report_.pos_um = homeEdgeMicrometres(pos_mm);
    report_.moved_um = homeEdgeMicrometres(moved_mm);
    pending_ = true;
  }
  bool take(HomeEdgeReport& out) {
    if (!pending_) return false;
    out = report_;
    pending_ = false;
    return true;
  }
  static int32_t homeEdgeMicrometres(float mm) {
    if (!(mm == mm)) return 0;  // NaN
    const float um = mm * 1000.0f;
    if (um > 2.0e9f) return 2000000000;
    if (um < -2.0e9f) return -2000000000;
    return static_cast<int32_t>(um < 0.0f ? um - 0.5f : um + 0.5f);
  }

 private:
  volatile bool pending_ = false;
  HomeEdgeReport report_;
};

// "-1.234" from micrometres without float formatting.
struct HomeEdgeMm3 {
  const char* sign;
  long whole;
  long frac;
};
inline HomeEdgeMm3 homeEdgeMm3(int32_t um) {
  const long magnitude = um < 0 ? -static_cast<long>(um) : static_cast<long>(um);
  return HomeEdgeMm3{um < 0 ? "-" : "", magnitude / 1000, magnitude % 1000};
}
