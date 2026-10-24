#include "JoystickMapping.h"
#include <math.h>
#include "JoystickProtocol.h"

float joyShapeAxis(int16_t axis, float deadzone) {
  float value = static_cast<float>(axis) / JOY_AXIS_FULL_SCALE;
  if (value > 1.0f) value = 1.0f;
  if (value < -1.0f) value = -1.0f;
  const float magnitude = fabsf(value);
  if (magnitude <= deadzone || deadzone >= 1.0f) return 0.0f;
  const float scaled = (magnitude - deadzone) / (1.0f - deadzone);
  const float shaped = scaled * scaled;
  return value < 0.0f ? -shaped : shaped;
}

JogRequest joyMapAxes(const int16_t axes[4], uint8_t speed_level,
                      const JoyMapConfig& config) {
  JogRequest request;
  const float feed_mm_s = config.feed_mm_min[speed_level & JOY_FLAG_SPEED_MASK] / 60.0f;
  request.vx_mm_s = joyShapeAxis(axes[JOY_AXIS_LEFT_X], config.deadzone) * feed_mm_s;
  request.vy_mm_s = joyShapeAxis(axes[JOY_AXIS_LEFT_Y], config.deadzone) * feed_mm_s;
  // Keep the combined XY speed within the feed limit on diagonals.
  const float speed_mm_s = sqrtf(request.vx_mm_s * request.vx_mm_s +
                                 request.vy_mm_s * request.vy_mm_s);
  if (speed_mm_s > feed_mm_s && speed_mm_s > 0.0f) {
    request.vx_mm_s *= feed_mm_s / speed_mm_s;
    request.vy_mm_s *= feed_mm_s / speed_mm_s;
  }
  request.z_counts_s = static_cast<int32_t>(lroundf(
      joyShapeAxis(axes[JOY_AXIS_RIGHT_Y], config.servo_deadzone) *
      config.servo_max_counts_s * config.z_sign));
  request.arm_counts_s = static_cast<int32_t>(lroundf(
      joyShapeAxis(axes[JOY_AXIS_RIGHT_X], config.servo_deadzone) *
      config.servo_max_counts_s * config.arm_sign));
  request.link = true;
  return request;
}

void joyMaskTowardLimits(float& vx_mm_s, float& vy_mm_s, bool x_limit_active,
                         bool y_limit_active, int8_t home_x_dir, int8_t home_y_dir) {
  if (x_limit_active && vx_mm_s * static_cast<float>(home_x_dir) > 0.0f) vx_mm_s = 0.0f;
  if (y_limit_active && vy_mm_s * static_cast<float>(home_y_dir) > 0.0f) vy_mm_s = 0.0f;
}

void jogClampSpeed(float& vx_mm_s, float& vy_mm_s, float max_speed_mm_s) {
  const float speed_mm_s = sqrtf(vx_mm_s * vx_mm_s + vy_mm_s * vy_mm_s);
  if (speed_mm_s > max_speed_mm_s && speed_mm_s > 0.0f) {
    vx_mm_s *= max_speed_mm_s / speed_mm_s;
    vy_mm_s *= max_speed_mm_s / speed_mm_s;
  }
}

float jogSoftLimitVelocity(float requested_mm_s, float previous_mm_s, float position_mm,
                           float min_mm, float max_mm, float brake_accel_mm_s2,
                           float lookahead_s, float stop_tolerance_mm) {
  if (requested_mm_s == 0.0f) return 0.0f;
  const float distance_mm =
      requested_mm_s > 0.0f ? max_mm - position_mm : position_mm - min_mm;
  const float effective_mm = distance_mm - fabsf(previous_mm_s) * lookahead_s;
  if (distance_mm <= stop_tolerance_mm || effective_mm <= 0.0f) return 0.0f;
  // The second bound keeps the last slow approach from crossing the boundary
  // before the next update.
  const float max_speed_mm_s = fminf(sqrtf(2.0f * brake_accel_mm_s2 * effective_mm),
                                     lookahead_s > 0.0f ? effective_mm / lookahead_s : 1e9f);
  if (fabsf(requested_mm_s) <= max_speed_mm_s) return requested_mm_s;
  return requested_mm_s > 0.0f ? max_speed_mm_s : -max_speed_mm_s;
}
