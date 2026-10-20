#include "CoreXYKinematics.h"
#include <math.h>

CoreXYPositionSteps CoreXYKinematics::xyPositionToABSteps(
    float x_mm, float y_mm, float steps_per_mm) {
  CoreXYPositionSteps position{};
  position.a_steps = lroundf((x_mm + y_mm) * steps_per_mm);
  position.b_steps = lroundf((x_mm - y_mm) * steps_per_mm);
  return position;
}

CoreXYDelta CoreXYKinematics::xyDeltaToABSteps(float dx_mm, float dy_mm,
                                                float steps_per_mm) {
  CoreXYDelta delta{};
  delta.dx_mm = dx_mm;
  delta.dy_mm = dy_mm;
  delta.a_steps = lroundf((dx_mm + dy_mm) * steps_per_mm);
  delta.b_steps = lroundf((dx_mm - dy_mm) * steps_per_mm);
  return delta;
}

CoreXYDelta CoreXYKinematics::xyMoveToABSteps(float current_x_mm,
                                               float current_y_mm,
                                               float target_x_mm,
                                               float target_y_mm,
                                               float steps_per_mm) {
  return xyDeltaToABSteps(target_x_mm - current_x_mm,
                          target_y_mm - current_y_mm, steps_per_mm);
}

CoreXYDelta CoreXYKinematics::abStepsToXYDelta(int32_t a_steps, int32_t b_steps,
                                                float steps_per_mm) {
  CoreXYDelta delta{};
  delta.a_steps = a_steps;
  delta.b_steps = b_steps;
  const float da_mm = static_cast<float>(a_steps) / steps_per_mm;
  const float db_mm = static_cast<float>(b_steps) / steps_per_mm;
  delta.dx_mm = (da_mm + db_mm) * 0.5f;
  delta.dy_mm = (da_mm - db_mm) * 0.5f;
  return delta;
}
