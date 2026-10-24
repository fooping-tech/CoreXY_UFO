#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <random>

#include "CoreXYKinematics.h"
#include "MotionBlock.h"
#include "PlotterConfig.h"
#include "SegmentGenerator.h"
#include "SegmentQueue.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << "\n";
    std::exit(1);
  }
}

void testAbsoluteStepClosedLoop() {
  constexpr size_t kTriangles = 3334;
  std::mt19937 rng(0xC0E2C0DE);
  std::uniform_real_distribution<float> coordinate_dist(-0.22f, 0.22f);

  int32_t planned_a_steps = 0;
  int32_t planned_b_steps = 0;
  int32_t accumulated_a_steps = 0;
  int32_t accumulated_b_steps = 0;
  const auto length = [](float x0, float y0, float x1, float y1) {
    const float dx = x1 - x0;
    const float dy = y1 - y0;
    return std::sqrt(dx * dx + dy * dy);
  };
  const auto add_target = [&](float x_mm, float y_mm) {
    const CoreXYPositionSteps target =
        CoreXYKinematics::xyPositionToABSteps(x_mm, y_mm, STEPS_PER_MM);
    const int32_t block_a_steps = target.a_steps - planned_a_steps;
    const int32_t block_b_steps = target.b_steps - planned_b_steps;
    accumulated_a_steps += block_a_steps;
    accumulated_b_steps += block_b_steps;
    planned_a_steps = target.a_steps;
    planned_b_steps = target.b_steps;
  };

  for (size_t triangle = 0; triangle < kTriangles; ++triangle) {
    float x1_mm = 0.0f;
    float y1_mm = 0.0f;
    float x2_mm = 0.0f;
    float y2_mm = 0.0f;
    for (;;) {
      x1_mm = coordinate_dist(rng);
      y1_mm = coordinate_dist(rng);
      x2_mm = coordinate_dist(rng);
      y2_mm = coordinate_dist(rng);
      const float l1 = length(0.0f, 0.0f, x1_mm, y1_mm);
      const float l2 = length(x1_mm, y1_mm, x2_mm, y2_mm);
      const float l3 = length(x2_mm, y2_mm, 0.0f, 0.0f);
      if (l1 >= 0.01f && l1 <= 0.5f && l2 >= 0.01f && l2 <= 0.5f &&
          l3 >= 0.01f && l3 <= 0.5f) {
        break;
      }
    }
    add_target(x1_mm, y1_mm);
    add_target(x2_mm, y2_mm);
    add_target(0.0f, 0.0f);
  }

  require(accumulated_a_steps == 0,
          "absolute-step closed loop accumulated nonzero A steps");
  require(accumulated_b_steps == 0,
          "absolute-step closed loop accumulated nonzero B steps");
  require(planned_a_steps == 0 && planned_b_steps == 0,
          "absolute-step closed loop final target is not origin");
}

void testSegmentGeneratorSumsToBlockSteps() {
  SegmentGenerator generator;
  SegmentQueue queue;
  for (int32_t a_steps = -320; a_steps <= 320; a_steps += 37) {
    for (int32_t b_steps = -300; b_steps <= 300; b_steps += 41) {
      if (a_steps == 0 && b_steps == 0) continue;
      MotionBlock block{};
      block.a_steps = a_steps;
      block.b_steps = b_steps;
      block.length_mm =
          std::max(std::abs(a_steps), std::abs(b_steps)) / STEPS_PER_MM;
      block.trapezoid_planned = true;
      block.nominal_speed_mm_min = 600.0f;
      block.acceleration_mm_s2 = 1000.0f;
      block.peak_speed_mm_s = 10.0f;
      block.cruise_distance_mm = block.length_mm;
      block.cruise_time_s = std::max(0.05f, block.length_mm / 10.0f);

      require(generator.generate(block, queue),
              "SegmentGenerator rejected test block");
      int32_t sum_a_steps = 0;
      int32_t sum_b_steps = 0;
      MotionSegment segment{};
      while (queue.dequeue(segment)) {
        sum_a_steps += segment.a_steps;
        sum_b_steps += segment.b_steps;
      }
      require(sum_a_steps == block.a_steps,
              "SegmentGenerator A segment sum mismatch");
      require(sum_b_steps == block.b_steps,
              "SegmentGenerator B segment sum mismatch");
    }
  }
}

// TEST_JOG/TEST_A/TEST_B limit direction checks rely on the inverse.
void testABStepsToXYDeltaInvertsForwardDelta() {
  const float cases_mm[][2] = {{10.0f, 0.0f}, {0.0f, 10.0f}, {-5.0f, 2.5f},
                               {3.0f, -7.0f}, {0.0f, 0.0f}};
  for (const auto& xy : cases_mm) {
    const CoreXYDelta forward =
        CoreXYKinematics::xyDeltaToABSteps(xy[0], xy[1], STEPS_PER_MM);
    const CoreXYDelta inverse = CoreXYKinematics::abStepsToXYDelta(
        forward.a_steps, forward.b_steps, STEPS_PER_MM);
    require(std::fabs(inverse.dx_mm - xy[0]) < 0.02f, "inverse dx mismatch");
    require(std::fabs(inverse.dy_mm - xy[1]) < 0.02f, "inverse dy mismatch");
  }
  // Single motor: A+ is +X +Y, B+ is +X -Y.
  const CoreXYDelta a_only = CoreXYKinematics::abStepsToXYDelta(400, 0, STEPS_PER_MM);
  require(a_only.dx_mm > 0.0f && a_only.dy_mm > 0.0f, "A+ must be +X +Y");
  const CoreXYDelta b_only = CoreXYKinematics::abStepsToXYDelta(0, 400, STEPS_PER_MM);
  require(b_only.dx_mm > 0.0f && b_only.dy_mm < 0.0f, "B+ must be +X -Y");
}

}  // namespace

void testStopRampDeceleratesWithinRemainingSteps() {
  SegmentGenerator generator;
  static SegmentQueue queue;
  // 100 mm/s, 1000 mm/s^2 -> 5 mm stop distance, 100 mm left along +X.
  const CoreXYDelta remaining = CoreXYKinematics::xyDeltaToABSteps(100.0f, 0.0f, STEPS_PER_MM);
  require(generator.generateStop(remaining.a_steps, remaining.b_steps, 100.0f, 1000.0f, queue),
          "stop ramp generated");
  int32_t sum_a = 0;
  int32_t sum_b = 0;
  uint32_t total_us = 0;
  float previous_speed = 1e9f;
  MotionSegment segment{};
  while (queue.dequeue(segment)) {
    sum_a += segment.a_steps;
    sum_b += segment.b_steps;
    total_us += segment.duration_us;
    const float speed = std::fabs(static_cast<float>(segment.a_steps)) /
                        (segment.duration_us / 1e6f);
    require(speed <= previous_speed + 1.0f, "stop ramp speed never increases");
    previous_speed = speed;
  }
  const CoreXYDelta moved = CoreXYKinematics::abStepsToXYDelta(sum_a, sum_b, STEPS_PER_MM);
  require(std::fabs(moved.dx_mm - 5.0f) < 0.05f && std::fabs(moved.dy_mm) < 0.02f,
          "stop ramp travels v^2/2a along the block direction");
  require(std::abs(static_cast<int>(total_us) - 100000) < 2000, "stop ramp lasts v/a");

  // Only 2 mm left: the ramp must end exactly at the block end.
  const CoreXYDelta short_left = CoreXYKinematics::xyDeltaToABSteps(2.0f, 1.0f, STEPS_PER_MM);
  require(generator.generateStop(short_left.a_steps, short_left.b_steps, 100.0f, 1000.0f, queue),
          "short stop ramp generated");
  sum_a = 0;
  sum_b = 0;
  while (queue.dequeue(segment)) {
    sum_a += segment.a_steps;
    sum_b += segment.b_steps;
  }
  require(sum_a == short_left.a_steps && sum_b == short_left.b_steps,
          "stop ramp never passes the remaining steps");

  require(generator.generateStop(remaining.a_steps, remaining.b_steps, 0.0f, 1000.0f, queue) &&
              queue.isEmpty(),
          "zero speed means already stopped");
}

int main() {
  testAbsoluteStepClosedLoop();
  testSegmentGeneratorSumsToBlockSteps();
  testABStepsToXYDeltaInvertsForwardDelta();
  testStopRampDeceleratesWithinRemainingSteps();
  std::cout << "native motion drift tests passed\n";
  return 0;
}
