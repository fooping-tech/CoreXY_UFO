#pragma once

#include <stdint.h>

// STS3020 standalone test firmware configuration.
// Built only by the `m5stack-core2-sts3020-test` PlatformIO env.
//
// Wiring (must be verified before JOG):
//   Core2 Port A GPIO32 (UART1 TX) -> bus servo adapter TX/RXD input
//   Core2 Port A GPIO33 (UART1 RX) <- bus servo adapter RX/TXD output
//   Adapter handles half-duplex direction control to the STS3020 signal line.
//   STS3020 power comes from an external supply via the adapter, GND shared
//   with Core2. Never connect ESP32 TX/RX directly to the servo signal pin.
// Pin assignment matches PEN_SERVO_UART_TX/RX_PIN on the STS3020 pen branch.

constexpr uint32_t STS_TEST_USB_SERIAL_BAUD = 115200;

constexpr uint8_t STS_TEST_UART_NUM = 1;
constexpr uint8_t STS_TEST_UART_TX_PIN = 32;
constexpr uint8_t STS_TEST_UART_RX_PIN = 33;
constexpr uint32_t STS_TEST_UART_BAUD = 1000000;
// Reply wait per transaction [ms]. A 1 Mbps reply arrives well under 1 ms.
constexpr uint32_t STS_TEST_UART_IO_TIMEOUT_MS = 20;

// Factory default ID is normally 1. Selected at runtime with the ID command.
constexpr uint8_t STS_TEST_DEFAULT_SERVO_ID = 1;
// Valid unicast ID range for ID / SET_ID / SCAN.
constexpr uint8_t STS_TEST_SERVO_ID_MIN = 0;
constexpr uint8_t STS_TEST_SERVO_ID_MAX = 253;
// SCAN range (kept short: each missing ID costs one IO timeout).
constexpr uint8_t STS_TEST_SCAN_ID_LAST = 20;
// SET_ID requires this many consecutive clean PINGs (bus collision check).
constexpr int STS_TEST_SET_ID_PING_REPEAT = 10;
constexpr uint8_t STS_TEST_BROADCAST_ID = 0xFE;

// Position mode raw counts: 4096 counts = 360 deg.
constexpr int STS_TEST_POSITION_MIN_COUNTS = 0;
constexpr int STS_TEST_POSITION_MAX_COUNTS = 4095;
// JOG targets are kept away from the 0/4095 wrap point.
constexpr int STS_TEST_JOG_TARGET_MIN_COUNTS = 100;
constexpr int STS_TEST_JOG_TARGET_MAX_COUNTS = 3995;
// Largest accepted |delta| per JOG. 100 counts ~= 8.8 deg.
constexpr int STS_TEST_JOG_MAX_DELTA_COUNTS = 100;

// Low speed/acceleration for bring-up.
// Speed unit: counts/s (200 ~= 17.6 deg/s). ACC unit: 100 counts/s^2.
constexpr uint16_t STS_TEST_JOG_SPEED_COUNTS_PER_S = 200;
constexpr uint8_t STS_TEST_JOG_ACC = 10;

// Arrival check after a JOG command.
constexpr uint32_t STS_TEST_JOG_TIMEOUT_MS = 2000;
constexpr uint32_t STS_TEST_JOG_POLL_INTERVAL_MS = 20;
constexpr int STS_TEST_JOG_TOLERANCE_COUNTS = 5;

// STS series operating mode register value for position (servo) mode.
constexpr int STS_TEST_MODE_POSITION = 0;

// STS EEPROM registers not defined by FTServo SMS_STS.h (STS memory table).
constexpr uint8_t STS_TEST_REG_MAX_TEMPERATURE = 13;    // [C]
constexpr uint8_t STS_TEST_REG_MAX_INPUT_VOLTAGE = 14;  // [0.1 V]
constexpr uint8_t STS_TEST_REG_MIN_INPUT_VOLTAGE = 15;  // [0.1 V]

// Touch UI: one tap = one JOG of this many counts (30 ~= 2.6 deg).
constexpr int STS_TEST_TOUCH_SMALL_STEP_COUNTS = 30;
constexpr int STS_TEST_TOUCH_LARGE_STEP_COUNTS = 100;
// LCD status (position/voltage/flags) poll interval while idle [ms].
constexpr uint32_t STS_TEST_UI_REFRESH_INTERVAL_MS = 1000;

// ---------------------------------------------------------------------------
// Multi-turn bench test (MT_ON / MT_JOG / MT_OFF)
// ---------------------------------------------------------------------------
// MT_ON sets angle limits 0/0 (multi-turn position control) with the EEPROM
// lock left ON, so the change is expected to be lost on servo power cycle.
// Persistence behaviour must be verified on the real STS3020.
constexpr int STS_TEST_SINGLE_TURN_MIN_LIMIT_COUNTS = 0;
constexpr int STS_TEST_SINGLE_TURN_MAX_LIMIT_COUNTS = 4095;
constexpr uint8_t STS_TEST_REG_LOCK_LOCKED = 1;
constexpr int STS_TEST_COUNTS_PER_TURN = 4096;
// Largest accepted |delta| per MT_JOG: three turns.
constexpr int STS_TEST_MT_JOG_MAX_DELTA_COUNTS = 3 * STS_TEST_COUNTS_PER_TURN;
// Firmware guard inside the documented +/-30719 multi-turn goal range.
constexpr int STS_TEST_MT_POSITION_LIMIT_COUNTS = 28000;
// 800 counts/s ~= 70 deg/s (~0.2 rev/s). ACC unit: 100 counts/s^2.
constexpr uint16_t STS_TEST_MT_SPEED_COUNTS_PER_S = 800;
constexpr uint8_t STS_TEST_MT_ACC = 20;
// Timeout = travel time at MT speed + this margin.
constexpr uint32_t STS_TEST_MT_TIMEOUT_MARGIN_MS = 2000;
// Measured 2026-10-18: goal is multi-turn, but Present Position wraps 0..4095
// per turn. Firmware unwraps feedback while polling. Unwrapping is valid only
// while each poll moves < half a turn, so tracking is invalidated when the gap
// between good reads exceeds this (800 counts/s * 0.5 s = 400 counts).
constexpr uint32_t STS_TEST_MT_MAX_READ_GAP_MS = 500;
// Position trace interval printed during MT_JOG.
constexpr uint32_t STS_TEST_MT_TRACE_INTERVAL_MS = 100;

// Hold check: after holding the present position and enabling torque, the
// servo must stay within this many counts for this long. Motion here means the
// servo's internal (hidden multi-turn) position differs from Present Position.
constexpr uint32_t STS_TEST_HOLD_CHECK_MS = 200;
constexpr int STS_TEST_HOLD_CHECK_TOLERANCE_COUNTS = 10;
// Runaway guard: torque is cut if motion goes this far backwards or this far
// beyond the commanded delta.
constexpr int STS_TEST_RUNAWAY_MARGIN_COUNTS = 50;
