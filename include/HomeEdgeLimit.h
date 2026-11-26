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
