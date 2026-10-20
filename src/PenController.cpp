#include "PenController.h"
#include "PlotterConfig.h"
#include "StsServoBus.h"

bool PenController::begin() {
  bus_.begin();
  // Do not actuate at boot: the horn may be attached at an unknown position.
  return ping();
}

bool PenController::ping() { return bus_.ping(PEN_SERVO_ID); }

bool PenController::readPosition(int& position) {
  return bus_.readRawPosition(PEN_SERVO_ID, position);
}

bool PenController::writeAngle(unsigned angle_deg) {
  // Legacy SERVO command operates over 0..180 degrees.
  if (angle_deg > 180U) return false;
  const uint16_t position = static_cast<uint16_t>(
      (angle_deg * PEN_SERVO_MAX_POSITION + PEN_SERVO_FULL_TURN_DEG / 2) /
      PEN_SERVO_FULL_TURN_DEG);
  return bus_.moveSingleTurn(PEN_SERVO_ID, position, PEN_SERVO_SPEED,
                             PEN_SERVO_ACCELERATION, PEN_SERVO_MOVE_TIMEOUT_MS,
                             PEN_SERVO_POSITION_TOLERANCE);
}

bool PenController::penUp() {
  if (PEN_SERVO_ENABLED && !writeAngle(PEN_UP_ANGLE_DEG)) return false;
  pen_down_ = false;
  return true;
}

bool PenController::penDown() {
  if (PEN_SERVO_ENABLED && !writeAngle(PEN_DOWN_ANGLE_DEG)) return false;
  pen_down_ = true;
  return true;
}

bool PenController::isPenDown() const { return pen_down_; }

bool PenController::setAngleDeg(unsigned angle_deg) {
  return writeAngle(angle_deg);
}
