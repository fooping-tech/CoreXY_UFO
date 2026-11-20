#pragma once

#include "StepperBackend.h"
#include "SegmentGenerator.h"
#include "TimedSegmentFeeder.h"

class FastAccelStepper;

class StepperBackendFastAccel : public StepperBackend {
 public:
  bool begin() override;
  bool isReady() const override;
  bool moveASteps(int32_t steps) override;
  bool moveBSteps(int32_t steps) override;
  bool moveABSteps(int32_t a_steps, int32_t b_steps,
                   float feed_mm_min) override;
  TimedSegmentResult queueTimedSegment(const MotionSegment& segment,
                                       bool start);
  bool startTimedSegments();
  // Why the last queueTimedSegment() ERROR happened (valid=false: none yet).
  const timed_segment::Failure& lastTimedSegmentFailure() const;
  // Counters since the A/B queues were last idle.
  const timed_segment::RunStats& timedSegmentStats() const;
  bool beginDiagnosticTone() override;
  DiagnosticPulseResult queueDiagnosticPulse(uint32_t frequency_hz) override;
  void endDiagnosticTone() override;
  bool beginDiagnosticChord();
  DiagnosticPulseResult queueDiagnosticPulseA(uint32_t frequency_hz);
  DiagnosticPulseResult queueDiagnosticPulseB(uint32_t frequency_hz);
  void endDiagnosticChord();
  // Joystick jog only: run continuously at signed step rates [steps/s].
  // 0 decelerates that motor to a stop. Call configureDefaultSpeed() after
  // the jog so later relative moves use the normal speed again.
  bool setVelocityAB(int32_t a_steps_s, int32_t b_steps_s, uint32_t accel_steps_s2);
  // Decelerate both motors to a stop (limit hits use stop() instead).
  void stopVelocity();
  void configureDefaultSpeed();
  void stop() override;
  bool isRunning() const override;
  void waitUntilIdle() override;
  int32_t currentASteps() const;
  int32_t currentBSteps() const;
  bool isMotorARunning() const;
  bool isMotorBRunning() const;
  uint16_t motorAQueueEntries() const;
  uint16_t motorBQueueEntries() const;

 private:
  void configureSpeed(float feed_mm_min);
  FastAccelStepper* motor_a_ = nullptr;
  FastAccelStepper* motor_b_ = nullptr;
  timed_segment::Feeder timed_segment_feeder_;
  bool ready_ = false;
  bool diagnostic_direction_positive_ = true;
  bool diagnostic_direction_positive_b_ = true;
};
