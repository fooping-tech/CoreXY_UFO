#pragma once

#include <stdint.h>

// Pure joystick-to-velocity mapping (no hardware). Native-tested.

// Velocity request to motionTask (Core 1) through a 1-deep overwrite queue:
// JogQueue from joystickTask, SerialJogQueue from commandTask (JOG command).
// Zero velocity on every axis means "stop".
struct JogRequest {
  float vx_mm_s = 0.0f;
  float vy_mm_s = 0.0f;
  int32_t z_counts_s = 0;    // servo ID 1
  int32_t arm_counts_s = 0;  // servo ID 2
  uint32_t time_ms = 0;      // when joystickTask produced it
  bool link = false;         // false: no valid packet within the timeout
};

struct JoyMapConfig {
  float deadzone = 0.08f;          // fraction of full scale (XY)
  // Servo axes start only on a clear push: a slightly off-center right stick
  // must not start a servo jog (its 200 ms hold check delays XY).
  float servo_deadzone = 0.25f;
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

// Scale (vx, vy) down so the combined speed stays within max_speed_mm_s.
void jogClampSpeed(float& vx_mm_s, float& vy_mm_s, float max_speed_mm_s);

// Soft-limit braking for one axis of a homed jog. Returns the requested
// velocity limited so the axis can stop before [min_mm, max_mm]:
// |v_out| <= min(sqrt(2 * brake_accel * d_eff), d_eff / lookahead_s), d_eff = distance to the boundary
// in the direction of travel minus |previous_mm_s| * lookahead_s. Within
// stop_tolerance_mm of the boundary the outward velocity is 0; inward
// velocity is never limited.
float jogSoftLimitVelocity(float requested_mm_s, float previous_mm_s, float position_mm,
                           float min_mm, float max_mm, float brake_accel_mm_s2,
                           float lookahead_s, float stop_tolerance_mm);

inline bool jogRequestIsZero(const JogRequest& request) {
  return request.vx_mm_s == 0.0f && request.vy_mm_s == 0.0f &&
         request.z_counts_s == 0 && request.arm_counts_s == 0;
}
