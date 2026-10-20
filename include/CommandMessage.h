#pragma once

#include <stdint.h>
#include "CommandTypes.h"
#include "LedTypes.h"
#include "ParsedGcode.h"

struct CommandMessage {
  CommandType type = CommandType::INVALID;
  char name[16] = {};
  bool from_gcode = false;
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float feed_mm_min = 0.0f;
  int32_t steps = 0;
  int32_t a_steps = 0;
  int32_t b_steps = 0;
  uint32_t duration_us = 0;
  uint32_t dwell_ms = 0;
  bool enable = false;      // TEST_MODE ON/OFF
  uint8_t servo_id = 0;     // SERVO_*
  float servo_value = 0.0f; // SERVO_ANGLE [deg] / SERVO_TURN [turn]
  LedCommand led;
  ParsedGcode gcode;
  char error[96] = {};
};
