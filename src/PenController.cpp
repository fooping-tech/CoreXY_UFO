#include "PenController.h"
#include <Arduino.h>
#include <SMS_STS.h>
#include "Core2PinMap.h"
#include "PlotterConfig.h"

namespace {
// Dedicated UART1. UART2 is reserved for the TMC2209 bus.
// The external adapter provides the required half-duplex TTL direction control.
HardwareSerial pen_uart(1);
SMS_STS pen_servo;
}

bool PenController::begin() {
  if (initialized_) return ping();
#if SIMULATION_MODE
  initialized_ = true;
  return true;
#else
  pen_uart.begin(PEN_SERVO_UART_BAUD, SERIAL_8N1,
                 PEN_SERVO_UART_RX_PIN, PEN_SERVO_UART_TX_PIN);
  pen_servo.pSerial = &pen_uart;
  pen_servo.IOTimeOut = 10;
  initialized_ = true;
  // Do not actuate at boot: the horn may be attached at an unknown position.
  return ping();
#endif
}

bool PenController::ping() {
#if SIMULATION_MODE
  return true;
#else
  return initialized_ && pen_servo.Ping(PEN_SERVO_ID) == PEN_SERVO_ID;
#endif
}

bool PenController::readPosition(int& position) {
  if (!initialized_) return false;
#if SIMULATION_MODE
  position = ((pen_down_ ? PEN_DOWN_ANGLE_DEG : PEN_UP_ANGLE_DEG) *
                  PEN_SERVO_MAX_POSITION + PEN_SERVO_FULL_TURN_DEG / 2) /
             PEN_SERVO_FULL_TURN_DEG;
  return true;
#else
  const int current = pen_servo.ReadPos(PEN_SERVO_ID);
  if (current < 0 || current > PEN_SERVO_MAX_POSITION) return false;
  position = current;
  return true;
#endif
}

bool PenController::writeAngle(unsigned angle_deg) {
  // Legacy SERVO command operates over 0..180 degrees.
  if (!initialized_ || angle_deg > 180U) return false;
#if SIMULATION_MODE
  return true;
#else
  if (!ping()) return false;
  const uint16_t position = static_cast<uint16_t>(
      (angle_deg * PEN_SERVO_MAX_POSITION + PEN_SERVO_FULL_TURN_DEG / 2) /
      PEN_SERVO_FULL_TURN_DEG);
  // Default library write return level does not ACK physical motion.
  if (!pen_servo.EnableTorque(PEN_SERVO_ID, 1)) return false;
  if (!pen_servo.WritePosEx(PEN_SERVO_ID, position,
                             PEN_SERVO_SPEED, PEN_SERVO_ACCELERATION)) {
    return false;
  }
  const uint32_t started_ms = millis();
  do {
    const int actual = pen_servo.ReadPos(PEN_SERVO_ID);
    if (actual >= 0 && actual <= PEN_SERVO_MAX_POSITION) {
      const int error = actual > position ? actual - position : position - actual;
      if (error <= PEN_SERVO_POSITION_TOLERANCE) return true;
    }
    delay(20);  // Give SafetyTask/other Core 1 tasks CPU time.
  } while (millis() - started_ms < PEN_SERVO_MOVE_TIMEOUT_MS);
  return false;
#endif
}

bool PenController::penUp() {
  if (!writeAngle(PEN_UP_ANGLE_DEG)) return false;
  pen_down_ = false;
  return true;
}

bool PenController::penDown() {
  if (!writeAngle(PEN_DOWN_ANGLE_DEG)) return false;
  pen_down_ = true;
  return true;
}

bool PenController::isPenDown() const { return pen_down_; }

bool PenController::setAngleDeg(unsigned angle_deg) {
  return writeAngle(angle_deg);
}
