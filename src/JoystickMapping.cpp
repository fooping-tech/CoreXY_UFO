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
  const float feed_mm_s = config.feed_mm_min[joySpeedLevel(speed_level)] / 60.0f;
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

float jogSoftLimitVelocity(float requested_mm_s, float position_mm, float min_mm,
                           float max_mm, float brake_accel_mm_s2, float latency_s,
                           float stop_margin_mm) {
  if (requested_mm_s == 0.0f) return 0.0f;
  const float distance_mm =
      (requested_mm_s > 0.0f ? max_mm - position_mm : position_mm - min_mm) - stop_margin_mm;
  if (distance_mm <= 0.0f) return 0.0f;
  // Positive root of v^2 / (2a) + v * T - d = 0.
  const float a = brake_accel_mm_s2;
  const float t = latency_s;
  const float max_speed_mm_s = a * (sqrtf(t * t + 2.0f * distance_mm / a) - t);
  if (fabsf(requested_mm_s) <= max_speed_mm_s) return requested_mm_s;
  return requested_mm_s > 0.0f ? max_speed_mm_s : -max_speed_mm_s;
}
