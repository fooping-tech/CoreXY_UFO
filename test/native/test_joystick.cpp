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
  testPacketCrc();
  std::cout << "native joystick tests passed\n";
  return 0;
}
