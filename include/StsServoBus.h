#pragma once

#include <stddef.h>
#include <stdint.h>

// Core 1 owned FEETECH STS3020 bus on UART1 (Port A, half-duplex adapter).
// The only module that touches the servo UART. PenController and the test
// mode servo commands go through it.
//
// Multi-turn control (measured on STS3020, see SPEC.md 2026-10-18):
// - Angle limits 0/0 select multi-turn position control. Written with the
//   EEPROM lock on, so it is lost on servo power cycle.
// - Goal is multi-turn, but Present Position wraps 0..4095 per turn, so the
//   absolute position is unwrapped here by polling during every move.
// - The servo keeps a hidden multi-turn position. A goal written from the
//   wrapped position can turn the servo by whole turns, so every move starts
//   with a hold check and is watched by a runaway guard.

struct StsServoStatus {
  bool link = false;
  uint8_t error_flags = 0;
  int mode = -1;
  int torque = -1;
  int raw_counts = -1;
  int goal_counts = 0;
  int min_limit_counts = -1;
  int max_limit_counts = -1;
  int voltage_dv = -1;
  int temperature_c = -1;
  int moving = -1;
  bool tracking = false;
  int32_t abs_counts = 0;
};

enum class StsMoveResult {
  PASS,
  TIMEOUT,
  ABORTED,
  REJECTED,
  FAILED,
};

const char* stsMoveResultName(StsMoveResult result);

class StsServoBus {
 public:
  bool begin();
  bool ping(uint8_t id);
  bool readRawPosition(uint8_t id, int& raw_counts);
  bool readStatus(uint8_t id, StsServoStatus& status);
  bool torqueOff(uint8_t id);

  // Single-turn absolute move (0..4095) with arrival check. Legacy pen path.
  bool moveSingleTurn(uint8_t id, uint16_t position_counts, uint16_t speed,
                      uint8_t acc, uint32_t timeout_ms, uint16_t tolerance_counts);

  // Test mode: relative multi-turn move. Enables multi-turn and tracking on
  // first use. detail receives a one-line summary for the result log.
  // abort_requested is polled during the move; on true the servo holds.
  StsMoveResult moveRelative(uint8_t id, int32_t delta_counts,
                             bool (*abort_requested)(), char* detail,
                             size_t detail_size);
  // Test mode: move to an absolute angle within the turn by the shortest path.
  StsMoveResult moveToAngle(uint8_t id, float angle_deg,
                            bool (*abort_requested)(), char* detail,
                            size_t detail_size);

  bool isTracking(uint8_t id) const;
  // Tracked multi-turn position [counts]; false when tracking is not valid.
  bool trackedAbsCounts(uint8_t id, int32_t& abs_counts) const;

  // Jog (joystick / Serial JOG). beginJog() enables tracking and runs the hold
  // check once; updateJog() keeps the goal "lead" ahead of the tracked
  // position (0 holds); endJog() holds at the current position.
  // false from beginJog/updateJog means the servo must not be jogged further
  // in this session; detail has the reason.
  bool beginJog(uint8_t id, char* detail, size_t detail_size);
  bool updateJog(uint8_t id, int32_t counts_s, char* detail, size_t detail_size);
  void endJog(uint8_t id);

 private:
  struct Track {
    bool valid = false;
    int32_t abs_counts = 0;
    int last_raw_counts = 0;
    // Joystick jog state.
    bool jogging = false;
    int32_t jog_goal_counts = 0;
    uint32_t jog_last_good_read_ms = 0;
  };

  bool validId(uint8_t id) const;
  Track& track(uint8_t id);
  bool enableTracking(uint8_t id, char* detail, size_t detail_size);
  bool checkReady(uint8_t id, char* detail, size_t detail_size);
  void updateTracking(Track& track, int raw_counts);
  bool holdAndVerify(uint8_t id, int32_t hold_counts, int start_raw_counts,
                     char* detail, size_t detail_size);

  bool initialized_ = false;
  Track tracks_[3];
};
