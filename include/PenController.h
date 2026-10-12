#pragma once

#include <stdint.h>

// Core 1 owned FEETECH STS3020 position controller.
// Commands wait for encoder feedback within a bounded timeout.
// Encoder arrival does not establish that the external pen mechanism is clear.
class PenController {
 public:
  bool begin();
  bool ping();
  bool readPosition(int& position);
  bool penUp();
  bool penDown();
  bool isPenDown() const;
  // Diagnostic: move to an absolute angle without updating the up/down state.
  bool setAngleDeg(unsigned angle_deg);

 private:
  bool writeAngle(unsigned angle_deg);
  bool initialized_ = false;
  bool pen_down_ = false;
};
