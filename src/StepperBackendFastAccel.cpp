#include "StepperBackendFastAccel.h"
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <math.h>
#include "Core2PinMap.h"
#include "PlotterConfig.h"

namespace {
FastAccelStepperEngine engine;
constexpr uint8_t TIMED_SEGMENT_DIRECTION_ENTRY_MARGIN = 2;
constexpr uint32_t TIMED_SEGMENT_PARTIAL_RETRY_TIMEOUT_MS = 1000;
// FastAccelStepper says a direction change needs at most three pauses.
constexpr uint8_t TIMED_SEGMENT_MAX_PAUSE_INJECTIONS = 4;
// A stopped queue refuses commands until its timer has run out (one queue
// entry, 4.1 ms at most). Starting is retried for this long.
constexpr uint32_t TIMED_SEGMENT_START_TIMEOUT_MS = 20;

// TimedSegmentFeeder.h mirrors these so that it builds without the library.
static_assert(static_cast<int8_t>(MOVE_TIMED_OK) == timed_segment::RESULT_OK,
              "MOVE_TIMED_OK changed");
static_assert(static_cast<int8_t>(MOVE_TIMED_EMPTY) ==
                  timed_segment::RESULT_MOVE_EMPTY,
              "MOVE_TIMED_EMPTY changed");
static_assert(static_cast<int8_t>(MoveTimedResultCode::DirPin2msPauseAdded) ==
                  timed_segment::RESULT_DIR_PIN_2MS_PAUSE_ADDED,
              "DirPin2msPauseAdded changed");
static_assert(static_cast<int8_t>(MoveTimedResultCode::DirChangePauseInjected) ==
                  timed_segment::RESULT_DIR_CHANGE_PAUSE_INJECTED,
              "DirChangePauseInjected changed");

timed_segment::Config timedSegmentConfig() {
  timed_segment::Config config;
  config.queue_len = QUEUE_LEN;
  config.direction_entry_margin = TIMED_SEGMENT_DIRECTION_ENTRY_MARGIN;
  config.partial_retry_timeout_ms = TIMED_SEGMENT_PARTIAL_RETRY_TIMEOUT_MS;
  config.max_pause_injections = TIMED_SEGMENT_MAX_PAUSE_INJECTIONS;
  return config;
}

#if !SIMULATION_MODE
struct FasTimedAxis {
  FastAccelStepper* motor;
  int8_t moveTimed(int16_t steps, uint32_t ticks, uint32_t* actual_ticks,
                   bool start) {
    return static_cast<int8_t>(motor->moveTimed(steps, ticks, actual_ticks, start));
  }
  uint16_t queueEntries() const { return motor->queueEntries(); }
  // The driver state. FastAccelStepper::isRunning() is also true while
  // entries wait in a queue that has not been started.
  bool isRunning() const { return motor->isQueueRunning(); }
  bool isSuspended() const { return motor->areCommandsSuspended(); }
};

struct RtosRetryClock {
  uint32_t nowMs() { return millis(); }
  void waitMs() { vTaskDelay(pdMS_TO_TICKS(1)); }
};
#endif

// 論理A/B(CoreXY A=X+Y, B=X-Y)を出力する物理STEP/DIRピン。
// MOTOR_AB_SWAPPEDで配線の入れ替えをここだけで吸収する。
constexpr uint8_t LOGICAL_A_STEP_PIN =
    MOTOR_AB_SWAPPED ? MOTOR_B_STEP_PIN : MOTOR_A_STEP_PIN;
constexpr uint8_t LOGICAL_A_DIR_PIN =
    MOTOR_AB_SWAPPED ? MOTOR_B_DIR_PIN : MOTOR_A_DIR_PIN;
constexpr uint8_t LOGICAL_B_STEP_PIN =
    MOTOR_AB_SWAPPED ? MOTOR_A_STEP_PIN : MOTOR_B_STEP_PIN;
constexpr uint8_t LOGICAL_B_DIR_PIN =
    MOTOR_AB_SWAPPED ? MOTOR_A_DIR_PIN : MOTOR_B_DIR_PIN;

StepperBackend::DiagnosticPulseResult queueDiagnosticPulseOnMotor(
    FastAccelStepper* motor, bool& direction_positive, uint32_t frequency_hz) {
  if (motor == nullptr || frequency_hz == 0 ||
      frequency_hz > MAX_MOTOR_SPEED_STEPS_S) {
    return StepperBackend::DiagnosticPulseResult::ERROR;
  }
  const uint32_t ticks = TICKS_PER_S / frequency_hz;
  if (ticks == 0 || ticks > UINT16_MAX) {
    return StepperBackend::DiagnosticPulseResult::ERROR;
  }
  const stepper_command_s command = {
      .ticks = static_cast<uint16_t>(ticks),
      .steps = 1,
      .count_up = direction_positive,
  };
  const AqeResultCode result = motor->addQueueEntry(&command);
  if (result == AQE_OK) {
    direction_positive = !direction_positive;
    return StepperBackend::DiagnosticPulseResult::QUEUED;
  }
  return aqeRetry(result) ? StepperBackend::DiagnosticPulseResult::RETRY
                          : StepperBackend::DiagnosticPulseResult::ERROR;
}
}

bool StepperBackendFastAccel::begin() {
  timed_segment_feeder_ = timed_segment::Feeder(timedSegmentConfig());
#if !SIMULATION_MODE
  engine.init();
  motor_a_ = engine.stepperConnectToPin(LOGICAL_A_STEP_PIN);
  motor_b_ = engine.stepperConnectToPin(LOGICAL_B_STEP_PIN);
  if (motor_a_ == nullptr || motor_b_ == nullptr) return false;
  motor_a_->setDirectionPin(LOGICAL_A_DIR_PIN, MOTOR_A_DIRECTION_INVERTED,
                            DIR_CHANGE_DELAY_US);
  motor_b_->setDirectionPin(LOGICAL_B_DIR_PIN, MOTOR_B_DIRECTION_INVERTED,
                            DIR_CHANGE_DELAY_US);
  configureSpeed(DEFAULT_FEED_MM_MIN);
#endif
  ready_ = true;
  return true;
}

bool StepperBackendFastAccel::isReady() const { return ready_; }

void StepperBackendFastAccel::configureSpeed(float feed_mm_min) {
#if !SIMULATION_MODE
  uint32_t speed_steps_s = lroundf(feed_mm_min * STEPS_PER_MM / 60.0f);
  speed_steps_s = constrain(speed_steps_s, 1U, MAX_MOTOR_SPEED_STEPS_S);
  motor_a_->setSpeedInHz(speed_steps_s);
  motor_b_->setSpeedInHz(speed_steps_s);
  motor_a_->setAcceleration(DEFAULT_MOTOR_ACCEL_STEPS_S2);
  motor_b_->setAcceleration(DEFAULT_MOTOR_ACCEL_STEPS_S2);
#endif
}

#if !SIMULATION_MODE
namespace {
// Relative ramp move from where the queue ends. FastAccelStepper's move()
// adds to the ramp generator's last target, which moveTimed() (the timed
// A/B segments of G0/G1) never updates: after a G-code move, move(n) would
// go to <the target before the G-code move> + n (#20: the HOMED fast approach
// ran into the X switch). While a ramp is running, move() is still right.
void moveRelative(FastAccelStepper* motor, int32_t steps) {
  if (motor->isRunning()) {
    motor->move(steps);
  } else {
    motor->moveTo(motor->getPositionAfterCommandsCompleted() + steps);
  }
}
}  // namespace
#endif

bool StepperBackendFastAccel::moveASteps(int32_t steps) {
#if SIMULATION_MODE
  (void)steps;
  return true;
#else
  if (motor_a_ == nullptr) return false;
  moveRelative(motor_a_, steps);
  return true;
#endif
}

bool StepperBackendFastAccel::moveBSteps(int32_t steps) {
#if SIMULATION_MODE
  (void)steps;
  return true;
#else
  if (motor_b_ == nullptr) return false;
  moveRelative(motor_b_, steps);
  return true;
#endif
}

bool StepperBackendFastAccel::moveABSteps(int32_t a_steps, int32_t b_steps,
                                          float feed_mm_min) {
  // Bring-up only:
  // Independent A/B move() calls do not guarantee strict XY interpolation.
  // Future implementation must replace this path with MotionBlock,
  // PlannerQueue, SegmentGenerator, and timed A/B segment execution.
#if SIMULATION_MODE
  (void)a_steps;
  (void)b_steps;
  (void)feed_mm_min;
  return true;
#else
  if (motor_a_ == nullptr || motor_b_ == nullptr) return false;
  configureSpeed(feed_mm_min);
  moveRelative(motor_a_, a_steps);
  moveRelative(motor_b_, b_steps);
  return true;
#endif
}

StepperBackend::TimedSegmentResult StepperBackendFastAccel::queueTimedSegment(
    const MotionSegment& segment, bool start) {
#if SIMULATION_MODE
  (void)segment;
  (void)start;
  return TimedSegmentResult::QUEUED;
#else
  if (motor_a_ == nullptr || motor_b_ == nullptr) {
    return TimedSegmentResult::ERROR;
  }
  // Real segments are about 20 ms. Longer than the feeder accepts (also what
  // would overflow the tick count) goes in as 0 and is refused there, so that
  // the refusal is recorded like any other.
  const uint32_t duration_ticks =
      segment.duration_us >
              timed_segment::MAX_DURATION_TICKS / (TICKS_PER_S / 1000000UL)
          ? 0
          : segment.duration_us * (TICKS_PER_S / 1000000UL);
  FasTimedAxis axis_a{motor_a_};
  FasTimedAxis axis_b{motor_b_};
  RtosRetryClock clock;
  switch (timed_segment_feeder_.feed(axis_a, axis_b, clock, segment.a_steps,
                                     segment.b_steps, duration_ticks, start)) {
    case timed_segment::FeedResult::QUEUED:
      return TimedSegmentResult::QUEUED;
    case timed_segment::FeedResult::RETRY:
      return TimedSegmentResult::RETRY;
    case timed_segment::FeedResult::ERROR:
      break;
  }
  stop();
  return TimedSegmentResult::ERROR;
#endif
}

bool StepperBackendFastAccel::startTimedSegments() {
#if SIMULATION_MODE
  return true;
#else
  if (motor_a_ == nullptr || motor_b_ == nullptr) return false;
  // moveTimed(0, 0, nullptr, true) only starts the queue and reports OK or
  // EMPTY even when the queue refused to start, so check the driver state.
  const uint32_t start_ms = millis();
  for (;;) {
    motor_a_->moveTimed(0, 0, nullptr, true);
    motor_b_->moveTimed(0, 0, nullptr, true);
    const bool started_a = motor_a_->isQueueRunning() || motor_a_->queueEntries() == 0;
    const bool started_b = motor_b_->isQueueRunning() || motor_b_->queueEntries() == 0;
    if (started_a && started_b) return true;
    if (millis() - start_ms >= TIMED_SEGMENT_START_TIMEOUT_MS) {
      const FasTimedAxis axis_a{motor_a_};
      const FasTimedAxis axis_b{motor_b_};
      timed_segment_feeder_.recordStartFailure(axis_a, axis_b);
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
#endif
}

const timed_segment::Failure& StepperBackendFastAccel::lastTimedSegmentFailure()
    const {
  return timed_segment_feeder_.failure();
}

const timed_segment::RunStats& StepperBackendFastAccel::timedSegmentStats() const {
  return timed_segment_feeder_.stats();
}

bool StepperBackendFastAccel::beginDiagnosticTone() {
#if SIMULATION_MODE
  return true;
#else
  if (!ready_ || motor_a_ == nullptr || motor_a_->isRunning()) return false;
  diagnostic_direction_positive_ = true;
  motor_a_->setDirectionPin(LOGICAL_A_DIR_PIN, MOTOR_A_DIRECTION_INVERTED, 0);
  return true;
#endif
}

StepperBackend::DiagnosticPulseResult
StepperBackendFastAccel::queueDiagnosticPulse(uint32_t frequency_hz) {
#if SIMULATION_MODE
  (void)frequency_hz;
  return DiagnosticPulseResult::QUEUED;
#else
  if (!ready_) return DiagnosticPulseResult::ERROR;
  return queueDiagnosticPulseOnMotor(motor_a_, diagnostic_direction_positive_,
                                     frequency_hz);
#endif
}

void StepperBackendFastAccel::endDiagnosticTone() {
#if !SIMULATION_MODE
  if (motor_a_ != nullptr) {
    motor_a_->setDirectionPin(LOGICAL_A_DIR_PIN, MOTOR_A_DIRECTION_INVERTED,
                              DIR_CHANGE_DELAY_US);
  }
#endif
}

bool StepperBackendFastAccel::beginDiagnosticChord() {
#if SIMULATION_MODE
  return true;
#else
  if (!ready_ || motor_a_ == nullptr || motor_b_ == nullptr ||
      motor_a_->isRunning() || motor_b_->isRunning()) {
    return false;
  }
  diagnostic_direction_positive_ = true;
  diagnostic_direction_positive_b_ = true;
  motor_a_->setDirectionPin(LOGICAL_A_DIR_PIN, MOTOR_A_DIRECTION_INVERTED, 0);
  motor_b_->setDirectionPin(LOGICAL_B_DIR_PIN, MOTOR_B_DIRECTION_INVERTED, 0);
  return true;
#endif
}

StepperBackend::DiagnosticPulseResult
StepperBackendFastAccel::queueDiagnosticPulseA(uint32_t frequency_hz) {
#if SIMULATION_MODE
  (void)frequency_hz;
  return DiagnosticPulseResult::QUEUED;
#else
  if (!ready_) return DiagnosticPulseResult::ERROR;
  return queueDiagnosticPulseOnMotor(motor_a_, diagnostic_direction_positive_,
                                     frequency_hz);
#endif
}

StepperBackend::DiagnosticPulseResult
StepperBackendFastAccel::queueDiagnosticPulseB(uint32_t frequency_hz) {
#if SIMULATION_MODE
  (void)frequency_hz;
  return DiagnosticPulseResult::QUEUED;
#else
  if (!ready_) return DiagnosticPulseResult::ERROR;
  return queueDiagnosticPulseOnMotor(motor_b_, diagnostic_direction_positive_b_,
                                     frequency_hz);
#endif
}

void StepperBackendFastAccel::endDiagnosticChord() {
#if !SIMULATION_MODE
  if (motor_a_ != nullptr) {
    motor_a_->setDirectionPin(LOGICAL_A_DIR_PIN, MOTOR_A_DIRECTION_INVERTED,
                              DIR_CHANGE_DELAY_US);
  }
  if (motor_b_ != nullptr) {
    motor_b_->setDirectionPin(LOGICAL_B_DIR_PIN, MOTOR_B_DIRECTION_INVERTED,
                              DIR_CHANGE_DELAY_US);
  }
#endif
}

namespace {
#if !SIMULATION_MODE
void runMotorAtVelocity(FastAccelStepper* motor, int32_t steps_s,
                        uint32_t accel_steps_s2) {
  if (motor == nullptr) return;
  if (steps_s == 0) {
    if (motor->isRunning()) motor->stopMove();
    return;
  }
  uint32_t speed_steps_s = static_cast<uint32_t>(steps_s < 0 ? -steps_s : steps_s);
  if (speed_steps_s > MAX_MOTOR_SPEED_STEPS_S) speed_steps_s = MAX_MOTOR_SPEED_STEPS_S;
  motor->setSpeedInHz(speed_steps_s);
  motor->setAcceleration(static_cast<int32_t>(accel_steps_s2));
  // runForward/runBackward update a running ramp and reverse if needed.
  if (steps_s > 0) motor->runForward();
  else motor->runBackward();
}
#endif
}  // namespace

bool StepperBackendFastAccel::setVelocityAB(int32_t a_steps_s, int32_t b_steps_s,
                                            uint32_t accel_steps_s2) {
#if SIMULATION_MODE
  (void)a_steps_s;
  (void)b_steps_s;
  (void)accel_steps_s2;
  return true;
#else
  if (!ready_ || accel_steps_s2 == 0) return false;
  runMotorAtVelocity(motor_a_, a_steps_s, accel_steps_s2);
  runMotorAtVelocity(motor_b_, b_steps_s, accel_steps_s2);
  return true;
#endif
}

void StepperBackendFastAccel::stopVelocity() {
#if !SIMULATION_MODE
  if (motor_a_ != nullptr && motor_a_->isRunning()) motor_a_->stopMove();
  if (motor_b_ != nullptr && motor_b_->isRunning()) motor_b_->stopMove();
#endif
}

void StepperBackendFastAccel::configureDefaultSpeed() {
  configureSpeed(DEFAULT_FEED_MM_MIN);
}

void StepperBackendFastAccel::stop() {
#if !SIMULATION_MODE
  if (motor_a_ != nullptr) motor_a_->forceStop();
  if (motor_b_ != nullptr) motor_b_->forceStop();
#endif
}

bool StepperBackendFastAccel::isRunning() const {
#if SIMULATION_MODE
  return false;
#else
  return (motor_a_ != nullptr && motor_a_->isRunning()) ||
         (motor_b_ != nullptr && motor_b_->isRunning());
#endif
}

void StepperBackendFastAccel::waitUntilIdle() {
  while (isRunning()) {
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

int32_t StepperBackendFastAccel::currentASteps() const {
#if SIMULATION_MODE
  return 0;
#else
  return motor_a_ == nullptr ? 0 : motor_a_->getCurrentPosition();
#endif
}

int32_t StepperBackendFastAccel::currentBSteps() const {
#if SIMULATION_MODE
  return 0;
#else
  return motor_b_ == nullptr ? 0 : motor_b_->getCurrentPosition();
#endif
}

bool StepperBackendFastAccel::isMotorARunning() const {
#if SIMULATION_MODE
  return false;
#else
  return motor_a_ != nullptr && motor_a_->isRunning();
#endif
}

bool StepperBackendFastAccel::isMotorBRunning() const {
#if SIMULATION_MODE
  return false;
#else
  return motor_b_ != nullptr && motor_b_->isRunning();
#endif
}

uint16_t StepperBackendFastAccel::motorAQueueEntries() const {
#if SIMULATION_MODE
  return 0;
#else
  return motor_a_ == nullptr ? 0 : motor_a_->queueEntries();
#endif
}

uint16_t StepperBackendFastAccel::motorBQueueEntries() const {
#if SIMULATION_MODE
  return 0;
#else
  return motor_b_ == nullptr ? 0 : motor_b_->queueEntries();
#endif
}
