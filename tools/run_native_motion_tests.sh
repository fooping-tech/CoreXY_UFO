#!/bin/sh
set -eu

c++ -std=c++17 -DSIMULATION_MODE=1 \
  -Iinclude -Itest/native \
  test/native/test_motion_drift.cpp \
  src/CoreXYKinematics.cpp src/SegmentGenerator.cpp src/SegmentQueue.cpp \
  -o /tmp/corexy_plotter_native_motion_tests

/tmp/corexy_plotter_native_motion_tests

c++ -std=c++17 -Iinclude -Itest/native \
  test/native/test_joystick.cpp src/JoystickMapping.cpp \
  -o /tmp/corexy_plotter_native_joystick_tests

/tmp/corexy_plotter_native_joystick_tests

c++ -std=c++17 -DSIMULATION_MODE=1 \
  -Iinclude -Itest/native \
  test/native/test_timed_segment_feeder.cpp \
  src/CoreXYKinematics.cpp src/SegmentGenerator.cpp src/SegmentQueue.cpp \
  -o /tmp/corexy_plotter_native_timed_segment_tests

/tmp/corexy_plotter_native_timed_segment_tests

c++ -std=c++17 -Iinclude -Itest/native \
  test/native/test_sts_turn_recovery.cpp \
  -o /tmp/corexy_plotter_native_sts_turn_recovery_tests

/tmp/corexy_plotter_native_sts_turn_recovery_tests

c++ -std=c++17 -Iinclude -Itest/native \
  test/native/test_home_edge_limit.cpp \
  -o /tmp/corexy_plotter_native_home_edge_limit_tests

/tmp/corexy_plotter_native_home_edge_limit_tests

# #32: SafetyManager::poll() also runs in safetyTask (2 KB stack). logMessage()
# or float formatting there overflowed it (Double exception in _dtoa_r).
awk '/^void SafetyManager::poll\(\)/{f=1} f{print} f&&/^}/{exit}' src/SafetyManager.cpp \
  > /tmp/corexy_safety_poll_body.txt
awk '/^bool SafetyManager::pollHomeEdge\(/{f=1} f{print} f&&/^}/{exit}' src/SafetyManager.cpp \
  >> /tmp/corexy_safety_poll_body.txt
if [ ! -s /tmp/corexy_safety_poll_body.txt ] || \
   grep -nE 'logMessage|%[-+ 0-9.]*[fFeEgG]' /tmp/corexy_safety_poll_body.txt; then
  echo "FAILED: SafetyManager::poll()/pollHomeEdge() must not log or format floats"
  exit 1
fi
echo "safety poll logging check passed"
