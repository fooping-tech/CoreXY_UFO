#pragma once

#include <stddef.h>
#include <stdint.h>
#include "MotionBlock.h"

class SegmentQueue;

struct MotionSegment {
  int32_t a_steps = 0;
  int32_t b_steps = 0;
  uint32_t duration_us = 0;
};

class SegmentGenerator {
 public:
  bool generate(const MotionBlock& block, MotionSegment& segment) const;
  bool generate(const MotionBlock& block, SegmentQueue& queue) const;
  // Planned speed [mm/s] at time_s from the block start.
  float speedAtTime(const MotionBlock& block, float time_s) const;
  // STOP: replace the rest of a block with a deceleration from speed_mm_s to
  // 0 at accel_mm_s2 along the remaining A/B steps. The ramp never goes past
  // the remaining steps; if it does not fit, it ends there at the residual
  // speed (as the block itself would end at its exit speed). An empty queue
  // means "already stopped".
  bool generateStop(int32_t remaining_a_steps, int32_t remaining_b_steps,
                    float speed_mm_s, float accel_mm_s2, SegmentQueue& queue) const;

 private:
  float distanceAtTime(const MotionBlock& block, float time_s) const;
};
