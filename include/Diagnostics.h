#pragma once

#include "MachineState.h"

class Diagnostics {
 public:
  static void printHelp();
  static void printConfig();
  static void printPosition(const StatusMessage& status);
  static void printLimitStatus(const StatusMessage& status);
  static void printHomingStatus(const StatusMessage& status);
  static bool runSelfTest();
  // Low-frequency TMC2209 UART diagnostics. Do not call while moving.
  static bool runTmcDiagnostics();
};
