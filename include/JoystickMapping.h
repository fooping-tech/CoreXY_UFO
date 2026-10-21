#pragma once

#include <stdint.h>

// Pure joystick-to-velocity mapping (no hardware). Native-tested.

// Velocity request from joystickTask (Core 0) to motionTask (Core 1) through
// the 1-deep JogQueue. Zero velocity on every axis means "stop".
struct JogRequest {
  float vx_mm_s = 0.0f;
  float vy_mm_s = 0.0f;
  int32_t z_counts_s = 0;    // servo ID 1
  int32_t arm_counts_s = 0;  // servo ID 2
  uint32_t time_ms = 0;      // when joystickTask produced it
  bool link = false;         // false: no valid packet within the timeout
};

struct JoyMapConfig {
  float deadzone = 0.08f;          // fraction of full scale
  // Indexed by JoySpeedLevel (SLOW, FAST, VERY_FAST, ULTRA).
  float feed_mm_min[4] = {600.0f, 3000.0f, 4500.0f, 7500.0f};
  int32_t servo_max_counts_s = 800;
  int8_t z_sign = 1;
  int8_t arm_sign = 1;
};

// -1000..1000 -> -1..1 with deadzone removal and a square curve for fine
// control near the center.
float joyShapeAxis(int16_t axis, float deadzone);

JogRequest joyMapAxes(const int16_t axes[4], uint8_t speed_level, const JoyMapConfig& config);

// Zero the XY components that push into an active limit switch.
// home_dir is HOMING_X_DIR / HOMING_Y_DIR (-1: switch at the negative end).
void joyMaskTowardLimits(float& vx_mm_s, float& vy_mm_s, bool x_limit_active,
                         bool y_limit_active, int8_t home_x_dir, int8_t home_y_dir);

inline bool jogRequestIsZero(const JogRequest& request) {
  return request.vx_mm_s == 0.0f && request.vy_mm_s == 0.0f &&
         request.z_counts_s == 0 && request.arm_counts_s == 0;
}
