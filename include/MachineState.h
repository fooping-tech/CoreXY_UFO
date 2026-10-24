#pragma once

#include <stdint.h>

struct MachineState {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  int32_t a_steps = 0;
  int32_t b_steps = 0;
  float feed_mm_min = 0.0f;
  bool homed = false;
  bool x_homed = false;
  bool y_homed = false;
  bool pen_down = false;
  bool alarmed = false;
  bool tmc_ready = false;
  bool homing_active = false;
  bool motion_active = false;
  bool job_active = false;
  // Serial/WebUI test mode: allows un-homed TEST_JOG and SERVO_* moves.
  bool test_mode_active = false;
  // Atom JoyStick (test input). Written by motionTask only.
  bool joystick_link = false;
  // Jog session active (joystick or Serial JOG).
  bool joystick_jog_active = false;
  char joystick_last_event[32] = "";
  char homing_state[16] = "Idle";
};

// TMC2209 1台分の低頻度診断結果。Core 1で更新し、UI表示用にStatusMessageで渡す。
struct TmcDriverSummary {
  bool checked = false;
  uint8_t connection = 0xFF;  // test_connection(): 0=OK, 1=no reply, 2=zero
  uint8_t version = 0;        // TMC2209 expects 0x21
  bool over_temp = false;     // DRV_STATUS ot
  bool over_temp_warn = false;  // DRV_STATUS otpw
  bool short_detected = false;  // DRV_STATUS s2ga/s2gb/s2vsa/s2vsb
  bool driver_error = false;    // GSTAT drv_err
  bool undervoltage = false;    // GSTAT uv_cp
};

// テストモード等の診断結果。Core 1 (motionTask)だけが書き込む。
struct DiagnosticsState {
  int8_t selftest_result = -1;  // -1=not run, 0=FAIL, 1=PASS
  TmcDriverSummary tmc_a;
  TmcDriverSummary tmc_b;
  char last_motor_test[32] = "";
};

struct StatusMessage {
  MachineState machine;
  bool x_limit_active = false;
  bool y_limit_active = false;
  bool x_limit_raw_active = false;
  bool y_limit_raw_active = false;
  DiagnosticsState diagnostics;

  StatusMessage() = default;
  StatusMessage(const MachineState& machine_state, bool x_limit, bool y_limit,
                bool x_limit_raw, bool y_limit_raw,
                const DiagnosticsState& diagnostics_state)
      : machine(machine_state),
        x_limit_active(x_limit),
        y_limit_active(y_limit),
        x_limit_raw_active(x_limit_raw),
        y_limit_raw_active(y_limit_raw),
        diagnostics(diagnostics_state) {}
};

struct LogMessage {
  // POS with LIMIT_*_RAW and TEST_MODE is ~280 chars; shorter buffers cut it.
  char text[320];
};
