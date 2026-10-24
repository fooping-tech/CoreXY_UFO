#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "JoystickMapping.h"
#include "JoystickProtocol.h"

namespace {

void require(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAILED: " << message << "\n";
    std::exit(1);
  }
}

void testShapeAxis() {
  require(joyShapeAxis(0, 0.08f) == 0.0f, "center is zero");
  require(joyShapeAxis(70, 0.08f) == 0.0f, "inside deadzone is zero");
  require(joyShapeAxis(-70, 0.08f) == 0.0f, "inside negative deadzone is zero");
  require(std::fabs(joyShapeAxis(1000, 0.08f) - 1.0f) < 1e-5f, "full scale is 1");
  require(std::fabs(joyShapeAxis(-1000, 0.08f) + 1.0f) < 1e-5f, "negative full scale is -1");
  require(std::fabs(joyShapeAxis(2000, 0.08f) - 1.0f) < 1e-5f, "over range is clamped");
  const float half = joyShapeAxis(540, 0.08f);
  require(half > 0.0f && half < 0.5f, "square curve is finer near center");
}

void testMapAxes() {
  JoyMapConfig config;
  const int16_t right[4] = {1000, 0, 0, 0};
  JogRequest fast = joyMapAxes(right, JOY_SPEED_FAST, config);
  require(std::fabs(fast.vx_mm_s - 50.0f) < 1e-3f, "fast +X = 3000 mm/min");
  require(fast.vy_mm_s == 0.0f && fast.z_counts_s == 0 && fast.arm_counts_s == 0,
          "only X moves");
  require(fast.link, "mapped request marks link");
  JogRequest slow = joyMapAxes(right, JOY_SPEED_SLOW, config);
  require(std::fabs(slow.vx_mm_s - 10.0f) < 1e-3f, "slow +X = 600 mm/min");
  JogRequest very_fast = joyMapAxes(right, JOY_SPEED_VERY_FAST, config);
  require(std::fabs(very_fast.vx_mm_s - 75.0f) < 1e-3f, "very fast +X = 4500 mm/min");
  JogRequest ultra = joyMapAxes(right, JOY_SPEED_ULTRA, config);
  require(std::fabs(ultra.vx_mm_s - 125.0f) < 1e-3f, "ultra +X = 7500 mm/min");
  JogRequest masked = joyMapAxes(right, 0xFF, config);
  require(std::fabs(masked.vx_mm_s - 125.0f) < 1e-3f, "speed level uses only the low 2 bits");

  const int16_t diagonal[4] = {1000, 1000, 0, 0};
  JogRequest diag = joyMapAxes(diagonal, JOY_SPEED_FAST, config);
  const float speed = std::sqrt(diag.vx_mm_s * diag.vx_mm_s + diag.vy_mm_s * diag.vy_mm_s);
  require(std::fabs(speed - 50.0f) < 1e-3f, "diagonal speed is capped to feed");

  const int16_t servos[4] = {0, 0, -1000, 1000};
  config.arm_sign = -1;
  JogRequest servo = joyMapAxes(servos, JOY_SPEED_FAST, config);
  require(servo.z_counts_s == 800, "right stick up drives Z at max counts/s");
  require(servo.arm_counts_s == 800, "arm sign inverts right stick X");

  const int16_t right_stick_drift[4] = {0, 0, 200, -200};
  JogRequest drift = joyMapAxes(right_stick_drift, JOY_SPEED_FAST, config);
  require(drift.z_counts_s == 0 && drift.arm_counts_s == 0,
          "small right stick offsets do not start servos");

  const int16_t centered[4] = {0, 0, 0, 0};
  require(jogRequestIsZero(joyMapAxes(centered, JOY_SPEED_FAST, config)), "centered request is zero");
}

void testMaskTowardLimits() {
  float vx = -10.0f;
  float vy = -10.0f;
  joyMaskTowardLimits(vx, vy, true, false, -1, -1);
  require(vx == 0.0f && vy == -10.0f, "active X limit blocks -X only");
  vx = 10.0f;
  joyMaskTowardLimits(vx, vy, true, true, -1, -1);
  require(vx == 10.0f && vy == 0.0f, "moving away from X limit is allowed; -Y blocked");
}

void testClampSpeed() {
  float vx = 30.0f;
  float vy = 40.0f;
  jogClampSpeed(vx, vy, 25.0f);
  require(std::fabs(vx - 15.0f) < 1e-4f && std::fabs(vy - 20.0f) < 1e-4f,
          "combined speed is clamped keeping direction");
  vx = 3.0f;
  vy = 4.0f;
  jogClampSpeed(vx, vy, 25.0f);
  require(vx == 3.0f && vy == 4.0f, "slow request unchanged");
}

void testSoftLimitVelocity() {
  constexpr float A = 300.0f;
  require(jogSoftLimitVelocity(100.0f, 0.0f, 100.0f, 0.0f, 390.0f, A, 0.04f, 0.05f) == 100.0f,
          "far from the boundary the request passes");
  require(jogSoftLimitVelocity(100.0f, 0.0f, 390.0f, 0.0f, 390.0f, A, 0.04f, 0.05f) == 0.0f,
          "outward velocity is 0 at the max boundary");
  require(jogSoftLimitVelocity(-100.0f, 0.0f, 390.0f, 0.0f, 390.0f, A, 0.04f, 0.05f) == -100.0f,
          "inward velocity is allowed at the boundary");
  require(jogSoftLimitVelocity(-100.0f, 0.0f, 0.0f, 0.0f, 390.0f, A, 0.04f, 0.05f) == 0.0f,
          "outward velocity is 0 at the min boundary");
  require(jogSoftLimitVelocity(100.0f, 0.0f, 395.0f, 0.0f, 390.0f, A, 0.04f, 0.05f) == 0.0f,
          "beyond the boundary only inward motion is allowed");
  const float limited = jogSoftLimitVelocity(100.0f, 0.0f, 389.0f, 0.0f, 390.0f, A, 0.04f, 0.05f);
  require(std::fabs(limited - std::sqrt(2.0f * A * 1.0f)) < 1e-3f, "near the boundary v <= sqrt(2ad)");

  // Closed loop: commands every 20 ms, the axis follows with only 1/sqrt(2)
  // of the nominal accel (CoreXY worst case). It must never pass X=390.
  constexpr float NOMINAL_ACCEL = 600.0f;
  const float follow_accel = NOMINAL_ACCEL * 0.7071f;
  float x = 300.0f;
  float v = 0.0f;
  float command = 0.0f;
  float max_x = x;
  for (int ms = 0; ms < 5000; ++ms) {
    if (ms % 20 == 0) {
      command = jogSoftLimitVelocity(125.0f, command, x, 0.0f, 390.0f, NOMINAL_ACCEL * 0.5f,
                                     0.04f, 0.05f);
    }
    const float dv = command - v;
    const float step = follow_accel * 0.001f;
    v += dv > step ? step : (dv < -step ? -step : dv);
    x += v * 0.001f;
    if (x > max_x) max_x = x;
  }
  require(max_x <= 390.0f, "closed-loop jog never passes the soft limit");
  require(x > 389.0f, "closed-loop jog gets close to the soft limit");
}

void testPacketCrc() {
  JoyCommandPacket packet{};
  packet.seq = 42;
  packet.axes[JOY_AXIS_LEFT_X] = 500;
  joyFinalizePacket(packet);
  JoyCommandPacket parsed{};
  require(joyPacketValid(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet), parsed),
          "valid packet accepted");
  require(parsed.seq == 42 && parsed.axes[JOY_AXIS_LEFT_X] == 500, "fields survive");

  JoyCommandPacket corrupt = packet;
  corrupt.axes[JOY_AXIS_LEFT_X] = 501;
  require(!joyPacketValid(reinterpret_cast<const uint8_t*>(&corrupt), sizeof(corrupt), parsed),
          "corrupted packet rejected");
  require(!joyPacketValid(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 1, parsed),
          "short packet rejected");
  JoyStatusPacket status{};
  require(!joyPacketValid(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet), status),
          "command packet is not a status packet");
}

}  // namespace

int main() {
  testShapeAxis();
  testMapAxes();
  testMaskTowardLimits();
  testClampSpeed();
  testSoftLimitVelocity();
  testPacketCrc();
  std::cout << "native joystick tests passed\n";
  return 0;
}
