#pragma once

#include <stdint.h>

class StsServoBus;

// Core 1 owned pen interface on the STS3020 bus (StsServoBus owns the UART).
// Commands wait for encoder feedback within a bounded timeout.
// Encoder arrival does not establish that the external pen mechanism is clear.
// With PEN_SERVO_ENABLED == false no servo is assigned to the pen: penUp() and
// penDown() only update the logical state and never move a servo.
class PenController {
 public:
  explicit PenController(StsServoBus& bus) : bus_(bus) {}
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
  StsServoBus& bus_;
  bool pen_down_ = false;
};
