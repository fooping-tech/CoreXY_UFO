#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "AppContext.h"
#include "Core2PinMap.h"
#include "CoreXYKinematics.h"
#include "Diagnostics.h"
#include "GcodeInterpreter.h"
#include "JoystickMapping.h"
#include "JunctionPlanner.h"
#include "PlotterConfig.h"
#include "PlannerQueue.h"
#include "SegmentGenerator.h"
#include "SegmentQueue.h"
#include "TrapezoidPlanner.h"

namespace {
TrapezoidPlanner trapezoid_planner;
JunctionPlanner junction_planner;
GcodeInterpreter gcode_interpreter;
SegmentGenerator segment_generator;
SegmentQueue segment_queue;
PlannerQueue planner_queue;
CommandMessage pending_command;
bool has_pending_command = false;

struct MotionSyncReference {
  int32_t backend_a_steps = 0;
  int32_t backend_b_steps = 0;
  int32_t machine_a_steps = 0;
  int32_t machine_b_steps = 0;
  float machine_x_mm = 0.0f;
  float machine_y_mm = 0.0f;
};

int32_t drift_backend_origin_a_steps = 0;
int32_t drift_backend_origin_b_steps = 0;
int32_t drift_machine_origin_a_steps = 0;
int32_t drift_machine_origin_b_steps = 0;

void syncJobActiveFlag() {
  machine_state.job_active = job_controller.isActive() ||
                             job_controller.isRunning();
}

void postLedStatus(LedStatus status) {
  if (led_command_queue == nullptr) return;
  LedCommand command{};
  command.type = LedCommandType::SET_STATUS;
  command.status = status;
  if (xQueueSend(led_command_queue, &command, 0) != pdTRUE) {
    logMessage("WARN: LedCommandQueue full status dropped");
  }
}

void setMotionActive(bool active) {
  if (machine_state.motion_active == active) return;
  machine_state.motion_active = active;
  syncJobActiveFlag();
  publishStatus();
}

const char* timedSegmentResultName(StepperBackend::TimedSegmentResult result) {
  switch (result) {
    case StepperBackend::TimedSegmentResult::QUEUED:
      return "QUEUED";
    case StepperBackend::TimedSegmentResult::RETRY:
      return "RETRY";
    case StepperBackend::TimedSegmentResult::ERROR:
      return "ERROR";
  }
  return "UNKNOWN";
}

StatusMessage currentStatus() {
  return StatusMessage{machine_state, safety_manager.xLimitActive(),
                       safety_manager.yLimitActive(),
                       safety_manager.xLimitRawActive(),
                       safety_manager.yLimitRawActive(),
                       diagnostics_state};
}

void invalidateHomed(const char* reason) {
  machine_state.homed = false;
  machine_state.x_homed = false;
  machine_state.y_homed = false;
  logMessage("HOMED invalidated: %s", reason);
}

MotionSyncReference captureMotionSyncReference() {
  MotionSyncReference reference{};
  reference.backend_a_steps = stepper_backend.currentASteps();
  reference.backend_b_steps = stepper_backend.currentBSteps();
  reference.machine_a_steps = machine_state.a_steps;
  reference.machine_b_steps = machine_state.b_steps;
  reference.machine_x_mm = machine_state.x_mm;
  reference.machine_y_mm = machine_state.y_mm;
  return reference;
}

void updateMachinePositionEstimateFromBackend(
    const MotionSyncReference& reference) {
  const int32_t delta_a_steps =
      stepper_backend.currentASteps() - reference.backend_a_steps;
  const int32_t delta_b_steps =
      stepper_backend.currentBSteps() - reference.backend_b_steps;
  machine_state.a_steps = reference.machine_a_steps + delta_a_steps;
  machine_state.b_steps = reference.machine_b_steps + delta_b_steps;
  const float delta_a_mm = static_cast<float>(delta_a_steps) / STEPS_PER_MM;
  const float delta_b_mm = static_cast<float>(delta_b_steps) / STEPS_PER_MM;
  machine_state.x_mm = reference.machine_x_mm + (delta_a_mm + delta_b_mm) * 0.5f;
  machine_state.y_mm = reference.machine_y_mm + (delta_a_mm - delta_b_mm) * 0.5f;
}

void resetDriftReference(const char* reason) {
#if !SIMULATION_MODE
  drift_backend_origin_a_steps = stepper_backend.currentASteps();
  drift_backend_origin_b_steps = stepper_backend.currentBSteps();
#else
  drift_backend_origin_a_steps = machine_state.a_steps;
  drift_backend_origin_b_steps = machine_state.b_steps;
#endif
  drift_machine_origin_a_steps = machine_state.a_steps;
  drift_machine_origin_b_steps = machine_state.b_steps;
  logMessage("DRIFT reference reset reason=%s A=%ld B=%ld",
             reason, machine_state.a_steps, machine_state.b_steps);
}

void warnIfDriftDetected() {
#if !SIMULATION_MODE
  const int32_t machine_delta_a =
      machine_state.a_steps - drift_machine_origin_a_steps;
  const int32_t machine_delta_b =
      machine_state.b_steps - drift_machine_origin_b_steps;
  const int32_t backend_delta_a =
      stepper_backend.currentASteps() - drift_backend_origin_a_steps;
  const int32_t backend_delta_b =
      stepper_backend.currentBSteps() - drift_backend_origin_b_steps;
  const int32_t drift_a = machine_delta_a - backend_delta_a;
  const int32_t drift_b = machine_delta_b - backend_delta_b;
  if (drift_a != 0 || drift_b != 0) {
    logMessage("WARN: DRIFT a=%ld b=%ld machine=(%ld,%ld) backend=(%ld,%ld)",
               drift_a, drift_b, machine_delta_a, machine_delta_b,
               backend_delta_a, backend_delta_b);
  }
#endif
}

bool stopForAbort(const char* context) {
  if (!isMotionAbortRequested()) return false;
  clearMotionAbort();
  setMotionActive(false);
  stepper_backend.stop();
  safety_manager.setAlarm("abort requested");
  machine_state.alarmed = true;
  invalidateHomed("abort requested");
  if (job_controller.isActive() || job_controller.isRunning()) {
    job_controller.markAborted("abort requested");
  }
  postLedStatus(LedStatus::ERROR);
  logMessage("%s: abort requested", context);
  return true;
}

bool waitForMotionOrLimit(const MotionSyncReference& reference) {
  while (stepper_backend.isRunning()) {
    if (stopForAbort("Motion stopped")) {
      return false;
    }
    updateMachinePositionEstimateFromBackend(reference);
    safety_manager.poll();
    if (safety_manager.isAlarmed()) {
      stepper_backend.stop();
      postLedStatus(LedStatus::ERROR);
      logMessage("Motion stopped: alarm reason=%s", safety_manager.alarmReason());
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  updateMachinePositionEstimateFromBackend(reference);
  return true;
}

bool waitForMotionOrLimit() {
  const MotionSyncReference reference = captureMotionSyncReference();
  return waitForMotionOrLimit(reference);
}

void handleTimedSegmentQueueError(const MotionSyncReference& reference,
                                  const char* context) {
  stepper_backend.stop();
  updateMachinePositionEstimateFromBackend(reference);
  safety_manager.setAlarm("timed segment queue lost position confidence");
  machine_state.alarmed = true;
  invalidateHomed("timed segment queue error");
  postLedStatus(LedStatus::ERROR);
  logMessage("%s: ERROR position confidence lost; resynced from backend X=%.3f Y=%.3f A=%ld B=%ld",
             context, machine_state.x_mm, machine_state.y_mm,
             machine_state.a_steps, machine_state.b_steps);
}

bool queueTimedSegmentWithRetry(const MotionSegment& segment, bool start,
                                const MotionSyncReference& reference) {
  for (;;) {
    if (stopForAbort("Motion stopped while queueing segment")) {
      return false;
    }
    const StepperBackend::TimedSegmentResult result =
        stepper_backend.queueTimedSegment(segment, start);
    if (result == StepperBackend::TimedSegmentResult::QUEUED) return true;
    if (result == StepperBackend::TimedSegmentResult::ERROR) {
      handleTimedSegmentQueueError(reference,
                                   "Motion stopped while queueing segment");
      return false;
    }
    safety_manager.poll();
    if (safety_manager.isAlarmed()) {
      stepper_backend.stop();
      postLedStatus(LedStatus::ERROR);
      logMessage("Motion stopped while queueing segment: alarm reason=%s",
                 safety_manager.alarmReason());
      return false;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

bool executeTimedSegments(SegmentQueue& queue) {
  if (stopForAbort("Motion stopped before timed segment start")) {
    return false;
  }
  const MotionSyncReference reference = captureMotionSyncReference();
  MotionSegment segment{};
  if (!queue.dequeue(segment)) return false;
  if (!queueTimedSegmentWithRetry(segment, false, reference)) return false;
  if (!stepper_backend.startTimedSegments()) return false;
  setMotionActive(true);

  while (queue.dequeue(segment)) {
    if (stopForAbort("Motion stopped during timed segment queueing")) {
      setMotionActive(false);
      return false;
    }
    if (!queueTimedSegmentWithRetry(segment, true, reference)) {
      setMotionActive(false);
      return false;
    }
  }
  const bool completed = waitForMotionOrLimit(reference);
  setMotionActive(false);
  return completed;
}

void stashPendingCommand(const CommandMessage& command) {
  pending_command = command;
  has_pending_command = true;
}

bool receiveNextCommand(CommandMessage& command, TickType_t ticks_to_wait) {
  if (has_pending_command) {
    command = pending_command;
    has_pending_command = false;
    return true;
  }
  return xQueueReceive(command_queue, &command, ticks_to_wait) == pdTRUE;
}

bool commandQueueEmpty() {
  return command_queue == nullptr || uxQueueMessagesWaiting(command_queue) == 0;
}

void clearMotionQueues(const char* reason, bool clear_pending = true) {
  planner_queue.clear();
  segment_queue.clear();
  if (clear_pending) {
    has_pending_command = false;
  }
  if (reason != nullptr) {
    logMessage("MOTION_QUEUES cleared reason=%s", reason);
  }
}

JobPreflight currentJobPreflight() {
  JobPreflight preflight{};
  preflight.pending_empty = !has_pending_command;
  preflight.planner_empty = planner_queue.isEmpty();
  preflight.segment_empty = segment_queue.isEmpty();
  preflight.command_queue_empty = commandQueueEmpty();
  preflight.backend_idle = !stepper_backend.isRunning();
  return preflight;
}

bool jobPreflightIdle(const JobPreflight& preflight) {
  return preflight.pending_empty && preflight.planner_empty &&
         preflight.segment_empty && preflight.command_queue_empty &&
         preflight.backend_idle;
}

bool prepareJobBeginAutoHome() {
  if (!JOB_BEGIN_AUTO_HOME || machine_state.homed) {
    return true;
  }
  job_controller.recoverToIdleIfSafe(safety_manager, machine_state);
  if (job_controller.state() != JobState::IDLE) {
    return true;
  }

  const JobPreflight preflight = currentJobPreflight();
  if (!jobPreflightIdle(preflight)) {
    return true;
  }

  safety_manager.poll();
  machine_state.alarmed = safety_manager.isAlarmed();
  if (machine_state.alarmed) {
    return true;
  }

  if (!machine_state.tmc_ready || !tmc_manager.isReady()) {
    logMessage("JOB_BEGIN TMC_INIT auto");
    machine_state.tmc_ready = tmc_manager.begin();
  }
  if (!machine_state.tmc_ready) {
    job_controller.markFailed("tmc_not_ready");
    logMessage("JOB_BEGIN rejected reason=tmc_not_ready");
    return false;
  }

  logMessage("JOB_BEGIN AUTO_HOME start");
  postLedStatus(LedStatus::HOMING);
  if (!homing_controller.runHome(stepper_backend, safety_manager,
                                 machine_state)) {
    machine_state.alarmed = safety_manager.isAlarmed();
    job_controller.markFailed("auto_home_failed");
    postLedStatus(LedStatus::ERROR);
    logMessage("JOB_BEGIN rejected reason=auto_home_failed");
    return false;
  }
  logMessage("JOB_BEGIN AUTO_HOME OK");
  postLedStatus(LedStatus::COMPLETED);
  return true;
}

bool rejectDisallowedJobCommand(const CommandMessage& command) {
  if (job_controller.allowCommand(command.type, command.from_gcode)) {
    return false;
  }
  if (command.type == CommandType::JOB_ABORT) {
    clearMotionAbort();
    clearMotionQueues("JOB_ABORT no active job");
    logMessage("JOB_ABORT rejected reason=no_active_job");
  } else {
    logMessage("REJECT: command %s not allowed while job_state=%s source=%s",
               command.name, job_controller.stateName(),
               command.from_gcode ? "GCODE" : "SERIAL");
  }
  return true;
}

uint32_t last_tmc_check_ms = 0;
TMC2209Health last_tmc_health = TMC2209Health::NO_REPLY;

// Applies a health check result. A driver reset or lost driver means the
// motors may have lost holding torque, so homing can no longer be trusted.
void checkTmcHealth(const char* context) {
  last_tmc_check_ms = millis();
  const bool was_ready = machine_state.tmc_ready;
  const TMC2209Health health = tmc_manager.checkAndRecover();
  machine_state.tmc_ready = health != TMC2209Health::NO_REPLY;
  if (health == TMC2209Health::REAPPLIED) {
    if (was_ready) invalidateHomed("TMC driver reset");
    logMessage("%s TMC config re-applied after driver reset", context);
  } else if (health == TMC2209Health::NO_REPLY &&
             last_tmc_health != TMC2209Health::NO_REPLY) {
    if (was_ready) invalidateHomed("TMC driver not responding");
    logMessage("WARN: %s TMC driver not responding (VM off?)", context);
  } else if (health == TMC2209Health::OK && last_tmc_health == TMC2209Health::NO_REPLY) {
    logMessage("%s TMC driver responding", context);
  }
  last_tmc_health = health;
  publishStatus();
}

bool ensureTmcReadyForMotion(const char* context) {
  if (machine_state.tmc_ready && tmc_manager.isReady() &&
      !stepper_backend.isRunning() &&
      millis() - last_tmc_check_ms >= TMC_HEALTH_CHECK_INTERVAL_MS) {
    checkTmcHealth(context);
  }
  if (machine_state.tmc_ready && tmc_manager.isReady()) {
    return true;
  }
  logMessage("%s TMC_INIT auto", context);
  machine_state.tmc_ready = tmc_manager.begin();
  if (!machine_state.tmc_ready) {
    logMessage("REJECT: %s reason=tmc_not_ready", context);
    return false;
  }
  return true;
}

GcodeInterpreterResult translateGcodeCommand(const CommandMessage& command,
                                             const MachineState& reference,
                                             CommandMessage& translated) {
  char log[128] = {};
  const GcodeInterpreterResult result = gcode_interpreter.interpret(
      command.gcode, reference, translated, log, sizeof(log));
  if (result == GcodeInterpreterResult::MODAL_UPDATE) {
    logMessage("%s", log);
  } else if (result == GcodeInterpreterResult::ERROR) {
    logMessage("ERROR: %s", log);
  } else if (translated.type == CommandType::XY) {
    logMessage("GCODE %s -> XY X=%.3f Y=%.3f F=%.3f mode=%s units=%s",
               command.name, translated.x_mm, translated.y_mm,
               translated.feed_mm_min,
               gcode_interpreter.absoluteMode() ? "ABS" : "REL",
               gcode_interpreter.unitsInches() ? "INCH" : "MM");
  } else if (translated.type == CommandType::DWELL) {
    logMessage("GCODE %s -> DWELL P=%lums", command.name,
               static_cast<unsigned long>(translated.dwell_ms));
  } else {
    logMessage("GCODE %s -> command %s", command.name, translated.name);
  }
  return result;
}

void resetGcodeModalForJob() {
  gcode_interpreter.resetModalState();
  machine_state.feed_mm_min = DEFAULT_FEED_MM_MIN;
  logMessage("JOB modal reset units=MM distance=ABSOLUTE feed=%.3f",
             DEFAULT_FEED_MM_MIN);
}

bool buildXYBlock(const CommandMessage& command, float start_x_mm,
                  float start_y_mm, int32_t start_a_steps,
                  int32_t start_b_steps, MotionBlock& block) {
  if (stopForAbort("XY rejected before planning")) {
    logMessage("NACK_XY target=(%.3f,%.3f) reason=abort",
               command.x_mm, command.y_mm);
    return false;
  }
  safety_manager.poll();
  float feed_mm_min = command.feed_mm_min;
  if (!safety_manager.validateMove(command.x_mm, command.y_mm, feed_mm_min)) {
    logMessage("NACK_XY target=(%.3f,%.3f) reason=rejected",
               command.x_mm, command.y_mm);
    return false;
  }
  const float dx_mm = command.x_mm - start_x_mm;
  const float dy_mm = command.y_mm - start_y_mm;
  const CoreXYPositionSteps target_steps =
      CoreXYKinematics::xyPositionToABSteps(command.x_mm, command.y_mm,
                                            STEPS_PER_MM);
  block = MotionBlock{};
  block.start_x_mm = start_x_mm;
  block.start_y_mm = start_y_mm;
  block.target_x_mm = command.x_mm;
  block.target_y_mm = command.y_mm;
  block.dx_mm = dx_mm;
  block.dy_mm = dy_mm;
  block.length_mm = sqrtf(dx_mm * dx_mm + dy_mm * dy_mm);
  block.nominal_speed_mm_min = feed_mm_min;
  block.entry_speed_mm_min = 0.0f;
  block.exit_speed_mm_min = 0.0f;
  block.a_steps = target_steps.a_steps - start_a_steps;
  block.b_steps = target_steps.b_steps - start_b_steps;
  block.target_a_steps = target_steps.a_steps;
  block.target_b_steps = target_steps.b_steps;
  block.pen_down = machine_state.pen_down;
  return true;
}

bool isNoOpXYBlock(const MotionBlock& block) {
  return block.a_steps == 0 && block.b_steps == 0;
}

void acknowledgeNoOpXY(const MotionBlock& block, bool update_machine_state) {
  if (update_machine_state) {
    machine_state.x_mm = block.target_x_mm;
    machine_state.y_mm = block.target_y_mm;
    machine_state.a_steps = block.target_a_steps;
    machine_state.b_steps = block.target_b_steps;
    machine_state.feed_mm_min = block.nominal_speed_mm_min;
  }
  logMessage("XY no-op current=(%.3f,%.3f) target=(%.3f,%.3f) F=%.3f",
             block.start_x_mm, block.start_y_mm, block.target_x_mm,
             block.target_y_mm, block.nominal_speed_mm_min);
  logMessage("ACK_XY target=(%.3f,%.3f) A=%ld B=%ld F=%.3f",
             block.target_x_mm, block.target_y_mm, block.a_steps,
             block.b_steps, block.nominal_speed_mm_min);
}

bool planQueuedBlocks() {
  if (!junction_planner.plan(planner_queue)) {
    logMessage("ERROR: junction planner rejected XY batch");
    return false;
  }
  for (size_t index = 0; index < planner_queue.count(); ++index) {
    MotionBlock* block = planner_queue.at(index);
    if (block == nullptr || !trapezoid_planner.plan(*block)) {
      logMessage("ERROR: trapezoid planner rejected XY batch index=%u",
                 static_cast<unsigned>(index));
      return false;
    }
  }
  return true;
}

bool executePlannedBlock(MotionBlock& block, size_t index, size_t count) {
  logMessage("XY batch=%u/%u current=(%.3f,%.3f) target=(%.3f,%.3f) dx=%.3f dy=%.3f A=%ld B=%ld F=%.3f entry=%.3f exit=%.3f",
             static_cast<unsigned>(index + 1), static_cast<unsigned>(count),
             block.start_x_mm, block.start_y_mm, block.target_x_mm,
             block.target_y_mm, block.dx_mm, block.dy_mm, block.a_steps,
             block.b_steps,
             block.nominal_speed_mm_min, block.entry_speed_mm_min,
             block.exit_speed_mm_min);
  logMessage("TRAPEZOID profile=%s length=%.3f accel=%.3f peak=%.3f accel_d=%.3f cruise_d=%.3f decel_d=%.3f t=%.3f",
             block.triangular_profile ? "TRIANGULAR" : "TRAPEZOID",
             block.length_mm, block.acceleration_mm_s2, block.peak_speed_mm_s,
             block.acceleration_distance_mm, block.cruise_distance_mm,
             block.deceleration_distance_mm,
             block.acceleration_time_s + block.cruise_time_s +
                 block.deceleration_time_s);
  if (!segment_generator.generate(block, segment_queue)) {
    logMessage("ERROR: segment generator rejected XY move");
    logMessage("NACK_XY target=(%.3f,%.3f) reason=segment",
               block.target_x_mm, block.target_y_mm);
    return false;
  }
  logMessage("SEGMENTS count=%u duration=%.3f dda=YES",
             static_cast<unsigned>(segment_queue.count()),
             block.acceleration_time_s + block.cruise_time_s +
                 block.deceleration_time_s);
#if SIMULATION_MODE
  logMessage("SIMULATION_MODE: no motor output");
  logMessage("ACK_XY target=(%.3f,%.3f) A=%ld B=%ld F=%.3f",
             block.target_x_mm, block.target_y_mm, block.a_steps,
             block.b_steps, block.nominal_speed_mm_min);
#else
  safety_manager.poll();
  const bool allow_x_limit_release =
      safety_manager.xLimitActive() &&
      block.dx_mm * static_cast<float>(HOMING_X_DIR) < 0.0f;
  const bool allow_y_limit_release =
      safety_manager.yLimitActive() &&
      block.dy_mm * static_cast<float>(HOMING_Y_DIR) < 0.0f;
  if (allow_x_limit_release || allow_y_limit_release) {
    logMessage("LIMIT_RELEASE_ALLOW X=%s Y=%s max=%.3f",
               allow_x_limit_release ? "YES" : "NO",
               allow_y_limit_release ? "YES" : "NO",
               NORMAL_MOVE_LIMIT_RELEASE_MM);
  }
  safety_manager.beginNormalMoveLimitReleaseAllowance(
      allow_x_limit_release, allow_y_limit_release, block.start_x_mm,
      block.start_y_mm);
  const bool executed = executeTimedSegments(segment_queue);
  safety_manager.clearNormalMoveLimitReleaseAllowance();
  if (!executed) {
    const char* alarm_reason = safety_manager.alarmReason();
    if (safety_manager.isAlarmed()) {
      const bool abort_alarm =
          strstr(alarm_reason, "abort requested") != nullptr;
      logMessage("ERROR: timed XY move stopped: alarm reason=%s",
                 alarm_reason);
      logMessage("NACK_XY target=(%.3f,%.3f) reason=%s",
                 block.target_x_mm, block.target_y_mm,
                 abort_alarm ? "abort" : "alarm");
    } else {
      logMessage("ERROR: backend rejected timed XY move");
      logMessage("NACK_XY target=(%.3f,%.3f) reason=backend",
                 block.target_x_mm, block.target_y_mm);
    }
    return false;
  }
  logMessage("ACK_XY target=(%.3f,%.3f) A=%ld B=%ld F=%.3f",
             block.target_x_mm, block.target_y_mm, block.a_steps,
             block.b_steps, block.nominal_speed_mm_min);
#endif
  machine_state.x_mm = block.target_x_mm;
  machine_state.y_mm = block.target_y_mm;
  machine_state.a_steps = block.target_a_steps;
  machine_state.b_steps = block.target_b_steps;
  machine_state.feed_mm_min = block.nominal_speed_mm_min;
  return true;
}

bool handleXYBatch(const CommandMessage& first_command) {
  planner_queue.clear();
  float planned_x_mm = machine_state.x_mm;
  float planned_y_mm = machine_state.y_mm;
  int32_t planned_a_steps = machine_state.a_steps;
  int32_t planned_b_steps = machine_state.b_steps;
  float planned_feed_mm_min = machine_state.feed_mm_min;

  MotionBlock block{};
  if (!buildXYBlock(first_command, planned_x_mm, planned_y_mm,
                    planned_a_steps, planned_b_steps, block)) {
    clearMotionQueues("XY build first failed");
    return false;
  }
  if (isNoOpXYBlock(block)) {
    acknowledgeNoOpXY(block, true);
  } else {
    if (!planner_queue.enqueue(block)) {
      logMessage("NACK_XY target=(%.3f,%.3f) reason=planner_queue_full",
                 first_command.x_mm, first_command.y_mm);
      return false;
    }
  }
  planned_x_mm = first_command.x_mm;
  planned_y_mm = first_command.y_mm;
  planned_a_steps = block.target_a_steps;
  planned_b_steps = block.target_b_steps;
  planned_feed_mm_min = first_command.feed_mm_min;

  CommandMessage next_command;
  while (!planner_queue.isFull() &&
         receiveNextCommand(next_command,
                            pdMS_TO_TICKS(LOOKAHEAD_BATCH_COLLECT_MS))) {
    if (rejectDisallowedJobCommand(next_command)) {
      continue;
    }
    if (next_command.type == CommandType::GCODE) {
      MachineState planned_state = machine_state;
      planned_state.x_mm = planned_x_mm;
      planned_state.y_mm = planned_y_mm;
      planned_state.a_steps = planned_a_steps;
      planned_state.b_steps = planned_b_steps;
      planned_state.feed_mm_min = planned_feed_mm_min;
      CommandMessage translated{};
      const GcodeInterpreterResult result =
          translateGcodeCommand(next_command, planned_state, translated);
      if (result == GcodeInterpreterResult::ERROR) {
        clearMotionQueues("GCODE translate failed");
        return false;
      }
      if (result == GcodeInterpreterResult::MODAL_UPDATE) {
        continue;
      }
      if (translated.type != CommandType::XY) {
        stashPendingCommand(translated);
        break;
      }
      next_command = translated;
    }
    if (rejectDisallowedJobCommand(next_command)) {
      continue;
    }
    if (next_command.type != CommandType::XY) {
      stashPendingCommand(next_command);
      break;
    }
    MotionBlock next_block{};
    if (!buildXYBlock(next_command, planned_x_mm, planned_y_mm,
                      planned_a_steps, planned_b_steps, next_block)) {
      clearMotionQueues("XY build batch failed");
      return false;
    }
    if (isNoOpXYBlock(next_block)) {
      acknowledgeNoOpXY(next_block, planner_queue.isEmpty());
    } else {
      if (!planner_queue.enqueue(next_block)) {
        logMessage("NACK_XY target=(%.3f,%.3f) reason=planner_queue_full",
                   next_command.x_mm, next_command.y_mm);
        return false;
      }
    }
    planned_x_mm = next_command.x_mm;
    planned_y_mm = next_command.y_mm;
    planned_a_steps = next_block.target_a_steps;
    planned_b_steps = next_block.target_b_steps;
    planned_feed_mm_min = next_command.feed_mm_min;
  }

  if (planner_queue.isEmpty()) {
    return true;
  }

  if (!planQueuedBlocks()) {
    const MotionBlock* failed = planner_queue.peekNext();
    logMessage("NACK_XY target=(%.3f,%.3f) reason=planner",
               failed != nullptr ? failed->target_x_mm : first_command.x_mm,
               failed != nullptr ? failed->target_y_mm : first_command.y_mm);
    clearMotionQueues("XY planning failed");
    return false;
  }

  logMessage("LOOKAHEAD blocks=%u junction_deviation=%.3f classic_jerk=%.3f",
             static_cast<unsigned>(planner_queue.count()),
             JUNCTION_DEVIATION_MM, CLASSIC_JERK_LIMIT_MM_S);

  postLedStatus(machine_state.pen_down ? LedStatus::DRAWING_PEN_DOWN
                                       : LedStatus::DRAWING_PEN_UP);
  const size_t planned_count = planner_queue.count();
  for (size_t index = 0; index < planned_count; ++index) {
    MotionBlock* planned_block = planner_queue.at(index);
    if (planned_block == nullptr ||
        !executePlannedBlock(*planned_block, index, planned_count)) {
      postLedStatus(safety_manager.isAlarmed() ? LedStatus::ERROR
                                               : LedStatus::WARNING);
      clearMotionQueues("XY execution failed");
      return false;
    }
  }
  machine_state.x_mm = planned_x_mm;
  machine_state.y_mm = planned_y_mm;
  machine_state.a_steps = planned_a_steps;
  machine_state.b_steps = planned_b_steps;
  machine_state.feed_mm_min = planned_feed_mm_min;
  warnIfDriftDetected();
  clearMotionQueues("XY complete", false);
  if (!job_controller.isRunning()) {
    postLedStatus(LedStatus::IDLE);
  }
  return true;
}

bool moveToJobEndPark() {
  if (!JOB_END_PARK_ENABLED) {
    logMessage("JOB_END park skipped: disabled by config");
    return true;
  }
  if (fabsf(machine_state.x_mm - JOB_END_PARK_X_MM) < 0.01f &&
      fabsf(machine_state.y_mm - JOB_END_PARK_Y_MM) < 0.01f) {
    logMessage("JOB_END park skipped: already at X=%.3f Y=%.3f",
               JOB_END_PARK_X_MM, JOB_END_PARK_Y_MM);
    return true;
  }
  CommandMessage park{};
  park.type = CommandType::XY;
  park.from_gcode = true;
  snprintf(park.name, sizeof(park.name), "JOB_PARK");
  park.x_mm = JOB_END_PARK_X_MM;
  park.y_mm = JOB_END_PARK_Y_MM;
  park.feed_mm_min = JOB_END_PARK_FEED_MM_MIN;
  logMessage("JOB_END park target=(%.3f,%.3f) F=%.3f",
             park.x_mm, park.y_mm, park.feed_mm_min);
  return handleXYBatch(park);
}

void handleJobEnd() {
  if (!job_controller.isRunning()) {
    job_controller.endJob(currentJobPreflight(), safety_manager, machine_state,
                          pen_controller);
    return;
  }

  if (!jobPreflightIdle(currentJobPreflight())) {
    job_controller.markFailed("job_end_queue_not_empty");
    logMessage("JOB_END failed reason=queue_not_empty");
    return;
  }

  if (!pen_controller.penUp()) {
    job_controller.markFailed("pen_servo_no_response");
    safety_manager.setAlarm("pen servo no response");
    machine_state.alarmed = true;
    invalidateHomed("pen servo no response");
    clearMotionQueues("pen servo fault");
    logMessage("ERROR: JOB_END pen up failed; park refused");
    postLedStatus(LedStatus::ERROR);
    return;
  }
  machine_state.pen_down = false;
  postLedStatus(LedStatus::DRAWING_PEN_UP);
  logMessage("JOB_END pen up before park");

  if (!moveToJobEndPark()) {
    job_controller.markFailed("job_end_park_failed");
    postLedStatus(LedStatus::ERROR);
    logMessage("JOB_END failed reason=park_failed");
    return;
  }
  if (!motor_melody_controller.playJobEndJingle(stepper_backend, tmc_manager,
                                                safety_manager)) {
    job_controller.markFailed("job_end_jingle_failed");
    postLedStatus(LedStatus::ERROR);
    logMessage("JOB_END failed reason=jingle_failed");
    return;
  }
  job_controller.endJob(currentJobPreflight(), safety_manager, machine_state,
                        pen_controller);
  postLedStatus(LedStatus::COMPLETED);
}

void handleABTimed(const CommandMessage& command) {
  logMessage("AB_TIMED command a_steps=%ld b_steps=%ld duration_us=%lu",
             command.a_steps, command.b_steps, command.duration_us);
  if (stopForAbort("AB_TIMED rejected before queue")) {
    logMessage("NACK_AB_TIMED reason=abort");
    return;
  }
  if (safety_manager.isAlarmed()) {
    logMessage("NACK_AB_TIMED reason=alarm");
    return;
  }
  if (!stepper_backend.isReady()) {
    logMessage("NACK_AB_TIMED reason=backend_not_ready");
    return;
  }
  if (command.a_steps == 0 && command.b_steps == 0) {
    logMessage("NACK_AB_TIMED reason=no_steps");
    return;
  }
  if (command.duration_us < AB_TIMED_MIN_DURATION_US) {
    logMessage("NACK_AB_TIMED reason=duration_too_short min_us=%lu",
               AB_TIMED_MIN_DURATION_US);
    return;
  }
  if (command.a_steps < INT16_MIN || command.a_steps > INT16_MAX ||
      command.b_steps < INT16_MIN || command.b_steps > INT16_MAX) {
    logMessage("NACK_AB_TIMED reason=steps_out_of_range int16_required=YES");
    return;
  }

  MotionSegment segment{};
  segment.a_steps = command.a_steps;
  segment.b_steps = command.b_steps;
  segment.duration_us = command.duration_us;
  const uint32_t micros_before_queue = micros();
  const MotionSyncReference reference = captureMotionSyncReference();
  logMessage("AB_TIMED before_queue micros=%lu queueEntries A=%u B=%u running A=%u B=%u",
             micros_before_queue, stepper_backend.motorAQueueEntries(),
             stepper_backend.motorBQueueEntries(),
             stepper_backend.isMotorARunning(),
             stepper_backend.isMotorBRunning());
  const StepperBackend::TimedSegmentResult queue_result =
      stepper_backend.queueTimedSegment(segment, false);
  const uint32_t micros_after_queue = micros();
  logMessage("AB_TIMED after_queue micros=%lu result=%s queueEntries A=%u B=%u running A=%u B=%u",
             micros_after_queue, timedSegmentResultName(queue_result),
             stepper_backend.motorAQueueEntries(),
             stepper_backend.motorBQueueEntries(),
             stepper_backend.isMotorARunning(),
             stepper_backend.isMotorBRunning());
  if (queue_result != StepperBackend::TimedSegmentResult::QUEUED) {
    if (queue_result == StepperBackend::TimedSegmentResult::ERROR) {
      handleTimedSegmentQueueError(reference, "AB_TIMED queue");
    }
    logMessage("NACK_AB_TIMED reason=queue_%s",
               timedSegmentResultName(queue_result));
    return;
  }
  const bool started = stepper_backend.startTimedSegments();
  logMessage("AB_TIMED start result=%s queueEntries A=%u B=%u running A=%u B=%u",
             started ? "OK" : "ERROR",
             stepper_backend.motorAQueueEntries(),
             stepper_backend.motorBQueueEntries(),
             stepper_backend.isMotorARunning(),
             stepper_backend.isMotorBRunning());
  if (!started) {
    handleTimedSegmentQueueError(reference, "AB_TIMED start");
    logMessage("NACK_AB_TIMED reason=start_error");
    return;
  }
  setMotionActive(true);
  if (!waitForMotionOrLimit(reference)) {
    setMotionActive(false);
    logMessage("AB_TIMED result=STOPPED queueEntries A=%u B=%u running A=%u B=%u",
               stepper_backend.motorAQueueEntries(),
               stepper_backend.motorBQueueEntries(),
               stepper_backend.isMotorARunning(),
               stepper_backend.isMotorBRunning());
    logMessage("NACK_AB_TIMED reason=stopped");
    return;
  }
  setMotionActive(false);
  logMessage("AB_TIMED result=OK queueEntries A=%u B=%u running A=%u B=%u",
             stepper_backend.motorAQueueEntries(),
             stepper_backend.motorBQueueEntries(),
             stepper_backend.isMotorARunning(),
             stepper_backend.isMotorBRunning());
  logMessage("ACK_AB_TIMED a_steps=%ld b_steps=%ld duration_us=%lu",
             command.a_steps, command.b_steps, command.duration_us);
}

void recordMotorTestResult(bool motor_a, int32_t steps, const char* result) {
  snprintf(diagnostics_state.last_motor_test,
           sizeof(diagnostics_state.last_motor_test), "TEST_%c %+ld %s",
           motor_a ? 'A' : 'B', static_cast<long>(steps), result);
  publishStatus();
}

// Un-homed test motion (TEST_A/TEST_B/TEST_JOG). Homing does not protect
// these moves, so a limit switch that is open at the start stops the move as
// soon as it turns ON (raw or debounced), and moves pushing into an already
// active switch are refused. feed_mm_min > 0 runs both motors at that feed;
// otherwise a single motor runs at the backend default speed (legacy Serial
// TEST_A/TEST_B). Returns the result token for the "<cmd> result=" log.
const char* runTestMotion(int32_t a_steps, int32_t b_steps, float feed_mm_min,
                          char* detail, size_t detail_size) {
  detail[0] = '\0';
  if (a_steps == 0 && b_steps == 0) {
    snprintf(detail, detail_size, "reason=no_steps");
    return "REJECTED";
  }
  safety_manager.poll();
  machine_state.alarmed = safety_manager.isAlarmed();
  if (machine_state.alarmed) {
    snprintf(detail, detail_size, "reason=alarm (ALARM_CLEAR)");
    return "REJECTED";
  }
  const bool x_active_at_start =
      safety_manager.xLimitActive() || safety_manager.xLimitRawActive();
  const bool y_active_at_start =
      safety_manager.yLimitActive() || safety_manager.yLimitRawActive();
  const CoreXYDelta delta =
      CoreXYKinematics::abStepsToXYDelta(a_steps, b_steps, STEPS_PER_MM);
  if (x_active_at_start && delta.dx_mm * static_cast<float>(HOMING_X_DIR) > 0.0f) {
    snprintf(detail, detail_size, "reason=x_limit_active_toward_limit");
    return "REJECTED";
  }
  if (y_active_at_start && delta.dy_mm * static_cast<float>(HOMING_Y_DIR) > 0.0f) {
    snprintf(detail, detail_size, "reason=y_limit_active_toward_limit");
    return "REJECTED";
  }
  invalidateHomed("test motion without homing");
#if SIMULATION_MODE
  machine_state.x_mm += delta.dx_mm;
  machine_state.y_mm += delta.dy_mm;
  machine_state.a_steps += a_steps;
  machine_state.b_steps += b_steps;
  snprintf(detail, detail_size, "sim=YES");
  return "DONE";
#else
  const MotionSyncReference reference = captureMotionSyncReference();
  bool accepted = false;
  if (feed_mm_min > 0.0f) {
    accepted = stepper_backend.moveABSteps(a_steps, b_steps, feed_mm_min);
  } else if (b_steps == 0) {
    accepted = stepper_backend.moveASteps(a_steps);
  } else if (a_steps == 0) {
    accepted = stepper_backend.moveBSteps(b_steps);
  }
  if (!accepted) {
    snprintf(detail, detail_size, "reason=backend_rejected");
    return "REJECTED";
  }
  setMotionActive(true);
  const char* result = "DONE";
  while (stepper_backend.isRunning()) {
    if (stopForAbort("Test motion stopped")) {
      snprintf(detail, detail_size, "reason=abort");
      result = "STOPPED";
      break;
    }
    updateMachinePositionEstimateFromBackend(reference);
    safety_manager.poll();
    const bool x_hit = !x_active_at_start && (safety_manager.xLimitActive() ||
                                              safety_manager.xLimitRawActive());
    const bool y_hit = !y_active_at_start && (safety_manager.yLimitActive() ||
                                              safety_manager.yLimitRawActive());
    if (x_hit || y_hit) {
      stepper_backend.stop();
      snprintf(detail, detail_size, "limit=%s%s", x_hit ? "X" : "", y_hit ? "Y" : "");
      result = "STOPPED_LIMIT";
      break;
    }
    if (safety_manager.isAlarmed()) {
      stepper_backend.stop();
      snprintf(detail, detail_size, "reason=alarm");
      result = "STOPPED";
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  updateMachinePositionEstimateFromBackend(reference);
  setMotionActive(false);
  return result;
#endif
}

// feed_mm_min > 0: 指定feed相当の速度で単独モータを動かす(UI/WebUIテストモード用)。
// feed_mm_min <= 0: 従来通りbackendの現在速度設定で動かす(Serial TEST_A/TEST_B)。
void handleSingleMotor(bool motor_a, int32_t steps, float feed_mm_min) {
  char detail[64];
  const char* result = runTestMotion(motor_a ? steps : 0, motor_a ? 0 : steps,
                                     feed_mm_min, detail, sizeof(detail));
  recordMotorTestResult(motor_a, steps, result);
  logMessage("TEST_%c result=%s steps=%+ld %s", motor_a ? 'A' : 'B', result,
             static_cast<long>(steps), detail);
}

void handleTestMode(const CommandMessage& command) {
  if (!command.enable) {
    machine_state.test_mode_active = false;
    logMessage("TEST_MODE result=OFF homed=%s%s", machine_state.homed ? "YES" : "NO",
               machine_state.homed ? "" : " (HOME before jobs)");
    return;
  }
  if (job_controller.isActive() || job_controller.isRunning()) {
    logMessage("TEST_MODE result=REJECTED reason=job_active");
    return;
  }
  if (machine_state.homing_active || stepper_backend.isRunning()) {
    logMessage("TEST_MODE result=REJECTED reason=busy");
    return;
  }
  machine_state.test_mode_active = true;
  logMessage("TEST_MODE result=ON (moves clear HOMED; limits stop TEST_JOG/TEST_A/TEST_B)");
}

void handleTestJog(const CommandMessage& command) {
  const char* result = "REJECTED";
  char detail[64] = "";
  if (!machine_state.test_mode_active) {
    snprintf(detail, sizeof(detail), "reason=test_mode_off");
  } else if (fabsf(command.x_mm) > TEST_JOG_MAX_DISTANCE_MM ||
             fabsf(command.y_mm) > TEST_JOG_MAX_DISTANCE_MM) {
    snprintf(detail, sizeof(detail), "reason=distance_over max=%.1fmm",
             TEST_JOG_MAX_DISTANCE_MM);
  } else if (!(command.feed_mm_min > 0.0f) ||
             command.feed_mm_min > TEST_JOG_MAX_FEED_MM_MIN) {
    snprintf(detail, sizeof(detail), "reason=feed_out_of_range max=%.0f",
             TEST_JOG_MAX_FEED_MM_MIN);
  } else if (ensureTmcReadyForMotion("TEST_JOG")) {
    const CoreXYDelta delta =
        CoreXYKinematics::xyDeltaToABSteps(command.x_mm, command.y_mm, STEPS_PER_MM);
    result = runTestMotion(delta.a_steps, delta.b_steps, command.feed_mm_min, detail,
                           sizeof(detail));
  } else {
    snprintf(detail, sizeof(detail), "reason=tmc_not_ready");
  }
  logMessage("TEST_JOG result=%s dx=%.3f dy=%.3f F=%.0f X=%.3f Y=%.3f %s", result,
             command.x_mm, command.y_mm, command.feed_mm_min, machine_state.x_mm,
             machine_state.y_mm, detail);
}

void handleServoStatus(uint8_t id) {
  StsServoStatus status;
  if (!sts_servo_bus.readStatus(id, status)) {
    logMessage("SERVO_STATUS id=%u result=NO_REPLY tracking=%s", id,
               status.tracking ? "YES" : "NO");
    return;
  }
  logMessage("SERVO_STATUS id=%u result=OK flags=0x%02X mode=%d torque=%d raw=%d "
             "goal=%d limits=%d/%d tracking=%s abs=%ld turns=%.2f voltage=%.1f "
             "temp=%d moving=%d",
             id, status.error_flags, status.mode, status.torque, status.raw_counts,
             status.goal_counts, status.min_limit_counts, status.max_limit_counts,
             status.tracking ? "YES" : "NO", static_cast<long>(status.abs_counts),
             static_cast<float>(status.abs_counts) / STS_COUNTS_PER_TURN,
             status.voltage_dv * 0.1f, status.temperature_c, status.moving);
}

void handleServoMove(const CommandMessage& command) {
  const bool angle = command.type == CommandType::SERVO_ANGLE;
  const char* name = angle ? "SERVO_ANGLE" : "SERVO_TURN";
  char detail[112] = "";
  StsMoveResult result = StsMoveResult::REJECTED;
  if (!machine_state.test_mode_active) {
    snprintf(detail, sizeof(detail), "reason=test_mode_off");
  } else if (angle) {
    setMotionActive(true);
    result = sts_servo_bus.moveToAngle(command.servo_id, command.servo_value,
                                       isMotionAbortRequested, detail, sizeof(detail));
    setMotionActive(false);
  } else if (fabsf(command.servo_value) > STS_TEST_MAX_TURNS) {
    snprintf(detail, sizeof(detail), "reason=turns_over max=%.1f", STS_TEST_MAX_TURNS);
  } else {
    const int32_t delta_counts =
        static_cast<int32_t>(lroundf(command.servo_value * STS_COUNTS_PER_TURN));
    setMotionActive(true);
    result = sts_servo_bus.moveRelative(command.servo_id, delta_counts,
                                        isMotionAbortRequested, detail, sizeof(detail));
    setMotionActive(false);
  }
  logMessage("%s id=%u value=%.2f result=%s %s", name, command.servo_id,
             command.servo_value, stsMoveResultName(result), detail);
}

// ---------------------------------------------------------------------------
// Atom JoyStick jog session (test mode only)
// ---------------------------------------------------------------------------
// JogRequests arrive through the 1-deep JogQueue from joystickTask (Core 0).
// XY runs in backend velocity mode; servos ID1 (Z) / ID2 (arm) use the
// StsServoBus jog. Any Serial command ends the session so the PC stays in
// charge.

bool peekJogRequest(JogRequest& request) {
  return jog_queue != nullptr && xQueuePeek(jog_queue, &request, 0) == pdTRUE;
}

bool jogRequestFresh(const JogRequest& request) {
  return request.link && millis() - request.time_ms <= JOY_LINK_TIMEOUT_MS;
}

void setJoystickEvent(const char* format, const char* reason) {
  snprintf(machine_state.joystick_last_event, sizeof(machine_state.joystick_last_event),
           format, reason);
}

void runJogSession(const JogRequest& first) {
  constexpr uint8_t SERVO_IDS[2] = {1, 2};
  const uint32_t accel_steps_s2 = static_cast<uint32_t>(
      JOY_ACCEL_MM_S2 * STEPS_PER_MM * COREXY_MAX_MOTOR_GAIN + 0.5f);
  bool xy_ready = false;
  bool xy_failed = false;
  bool servo_begun[2] = {false, false};
  bool servo_failed[2] = {false, false};
  bool moving_toward_x = false;
  bool moving_toward_y = false;
  bool x_was_active = safety_manager.xLimitActive() || safety_manager.xLimitRawActive();
  bool y_was_active = safety_manager.yLimitActive() || safety_manager.yLimitRawActive();
  const char* reason = "idle";
  uint32_t zero_since_ms = 0;
  JogRequest request = first;
  const MotionSyncReference reference = captureMotionSyncReference();

  const uint32_t session_start_ms = millis();
  // Latency diagnostics: request age at start, TMC check, servo hold checks,
  // and the time until XY gets its first velocity command.
  const uint32_t request_age_ms = session_start_ms - first.time_ms;
  uint32_t tmc_check_ms = 0;
  bool xy_start_logged = false;
  machine_state.joystick_jog_active = true;
  setMotionActive(true);
  logMessage("JOY_JOG start vx=%.1f vy=%.1f z=%ld arm=%ld", first.vx_mm_s, first.vy_mm_s,
             static_cast<long>(first.z_counts_s), static_cast<long>(first.arm_counts_s));

  for (;;) {
    if (stopForAbort("JOY_JOG stopped")) {
      reason = "abort";
      break;
    }
    if (has_pending_command || !commandQueueEmpty()) {
      // Serial/WebUI commands and joystick buttons (TEST_MODE, ABORT, ...).
      reason = "command";
      break;
    }
    if (!machine_state.test_mode_active) {
      reason = "test_mode_off";
      break;
    }
    peekJogRequest(request);
    if (!jogRequestFresh(request)) {
      reason = "link_lost";
      break;
    }

    // XY
    safety_manager.poll();
    machine_state.alarmed = safety_manager.isAlarmed();
    if (machine_state.alarmed) {
      stepper_backend.stop();
      reason = "alarm";
      break;
    }
    const bool x_active = safety_manager.xLimitActive() || safety_manager.xLimitRawActive();
    const bool y_active = safety_manager.yLimitActive() || safety_manager.yLimitRawActive();
    if ((x_active && !x_was_active && moving_toward_x) ||
        (y_active && !y_was_active && moving_toward_y)) {
      stepper_backend.stop();
      reason = x_active && !x_was_active ? "limit_x" : "limit_y";
      break;
    }
    x_was_active = x_active;
    y_was_active = y_active;
    float vx_mm_s = request.vx_mm_s;
    float vy_mm_s = request.vy_mm_s;
    joyMaskTowardLimits(vx_mm_s, vy_mm_s, x_active, y_active, HOMING_X_DIR, HOMING_Y_DIR);
    const bool xy_requested = vx_mm_s != 0.0f || vy_mm_s != 0.0f;
    if (xy_requested && !xy_ready && !xy_failed) {
      const uint32_t tmc_started_ms = millis();
      xy_failed = !ensureTmcReadyForMotion("JOY_JOG");
      tmc_check_ms = millis() - tmc_started_ms;
      if (!xy_failed) {
        invalidateHomed("joystick jog without homing");
        xy_ready = true;
      }
    }
    if (xy_ready && xy_requested) {
      // xyDeltaToABSteps of a 1 s delta = A/B step rates.
      const CoreXYDelta rate =
          CoreXYKinematics::xyDeltaToABSteps(vx_mm_s, vy_mm_s, STEPS_PER_MM);
      stepper_backend.setVelocityAB(rate.a_steps, rate.b_steps, accel_steps_s2);
      if (!xy_start_logged) {
        xy_start_logged = true;
        logMessage("JOY_JOG xy_start after_session_ms=%lu request_age_ms=%lu tmc_check_ms=%lu",
                   static_cast<unsigned long>(millis() - session_start_ms),
                   static_cast<unsigned long>(request_age_ms),
                   static_cast<unsigned long>(tmc_check_ms));
      }
      moving_toward_x = vx_mm_s * static_cast<float>(HOMING_X_DIR) > 0.0f;
      moving_toward_y = vy_mm_s * static_cast<float>(HOMING_Y_DIR) > 0.0f;
    } else if (xy_ready) {
      stepper_backend.stopVelocity();
    }
    updateMachinePositionEstimateFromBackend(reference);

    // Servos. beginJog() holds for the 200 ms hold check, during which limits
    // are not polled, so only start a servo while XY is stopped.
    const int32_t servo_rates[2] = {request.z_counts_s, request.arm_counts_s};
    for (int i = 0; i < 2; ++i) {
      if (servo_failed[i]) continue;
      char detail[96] = "";
      if (!servo_begun[i]) {
        if (servo_rates[i] == 0 || stepper_backend.isRunning()) continue;
        const uint32_t begin_started_ms = millis();
        if (!sts_servo_bus.beginJog(SERVO_IDS[i], detail, sizeof(detail))) {
          servo_failed[i] = true;
          logMessage("JOY_JOG servo id=%u disabled %s", SERVO_IDS[i], detail);
          continue;
        }
        servo_begun[i] = true;
        logMessage("JOY_JOG servo id=%u begin_ms=%lu (hold check; XY waits)", SERVO_IDS[i],
                   static_cast<unsigned long>(millis() - begin_started_ms));
      }
      if (!sts_servo_bus.updateJog(SERVO_IDS[i], servo_rates[i], detail, sizeof(detail))) {
        servo_failed[i] = true;
        logMessage("JOY_JOG servo id=%u stopped %s", SERVO_IDS[i], detail);
      }
    }

    if (jogRequestIsZero(request) && !stepper_backend.isRunning()) {
      if (zero_since_ms == 0) zero_since_ms = millis();
      if (millis() - zero_since_ms >= JOY_SESSION_IDLE_EXIT_MS) {
        reason = "idle";
        break;
      }
    } else {
      zero_since_ms = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(JOY_SESSION_PERIOD_MS));
  }

  // Decelerate XY, still stopping at once on a new limit hit.
  stepper_backend.stopVelocity();
  while (stepper_backend.isRunning()) {
    safety_manager.poll();
    const bool x_active = safety_manager.xLimitActive() || safety_manager.xLimitRawActive();
    const bool y_active = safety_manager.yLimitActive() || safety_manager.yLimitRawActive();
    if ((x_active && !x_was_active && moving_toward_x) ||
        (y_active && !y_was_active && moving_toward_y) || isMotionAbortRequested()) {
      stepper_backend.stop();
    }
    updateMachinePositionEstimateFromBackend(reference);
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  updateMachinePositionEstimateFromBackend(reference);
  stepper_backend.configureDefaultSpeed();
  for (int i = 0; i < 2; ++i) {
    if (servo_begun[i]) sts_servo_bus.endJog(SERVO_IDS[i]);
  }
  setMotionActive(false);
  machine_state.joystick_jog_active = false;
  setJoystickEvent("JOG end %s", reason);
  logMessage("JOY_JOG end reason=%s X=%.3f Y=%.3f%s", reason, machine_state.x_mm,
             machine_state.y_mm, xy_failed ? " xy=tmc_not_ready" : "");
  publishStatus();
}

// Called while motionTask waits for commands.
void pollJoystick() {
  JogRequest request;
  if (!peekJogRequest(request)) return;
  const bool linked = jogRequestFresh(request);
  if (linked != machine_state.joystick_link) {
    machine_state.joystick_link = linked;
    publishStatus();
  }
  static bool ignored_logged = false;
  if (!linked || jogRequestIsZero(request)) {
    ignored_logged = false;
    return;
  }
  const char* blocked = nullptr;
  if (!machine_state.test_mode_active) blocked = "test_mode_off";
  else if (safety_manager.isAlarmed()) blocked = "alarm";
  else if (job_controller.isActive() || job_controller.isRunning()) blocked = "job_active";
  else if (machine_state.homing_active) blocked = "homing";
  if (blocked != nullptr) {
    if (!ignored_logged) {
      logMessage("JOY_JOG ignored reason=%s", blocked);
      setJoystickEvent("ignored %s", blocked);
      publishStatus();
      ignored_logged = true;
    }
    return;
  }
  ignored_logged = false;
  runJogSession(request);
}
}

void motionTask(void*) {
  pen_controller.begin();
  for (uint8_t id = STS_SERVO_ID_MIN; id <= STS_SERVO_ID_MAX; ++id) {
    logMessage("STS3020 UART1 TX=%u RX=%u baud=%lu ID=%u ping=%s (no startup motion)",
               PEN_SERVO_UART_TX_PIN, PEN_SERVO_UART_RX_PIN,
               static_cast<unsigned long>(PEN_SERVO_UART_BAUD), id,
               sts_servo_bus.ping(id) ? "OK" : "NO_REPLY");
  }
  logMessage("PEN servo %s", PEN_SERVO_ENABLED ? "enabled" : "disabled (no servo assigned)");
  // Send the TMC2209 configuration once at boot. If VM is off this fails and
  // the pre-motion check applies it when the drivers come up (GSTAT.reset).
  machine_state.tmc_ready = tmc_manager.begin();
  last_tmc_health = machine_state.tmc_ready ? TMC2209Health::OK : TMC2209Health::NO_REPLY;
  last_tmc_check_ms = millis();
  publishStatus();
  CommandMessage command;
  for (;;) {
    const TickType_t wait = JOYSTICK_ENABLED ? pdMS_TO_TICKS(JOY_SESSION_PERIOD_MS)
                                             : portMAX_DELAY;
    if (!receiveNextCommand(command, wait)) {
      if (JOYSTICK_ENABLED) pollJoystick();
      continue;
    }
    syncJobActiveFlag();
    if (rejectDisallowedJobCommand(command)) {
      publishStatus();
      continue;
    }
    switch (command.type) {
      case CommandType::HELP:
        Diagnostics::printHelp();
        break;
      case CommandType::CONFIG:
        Diagnostics::printConfig();
        break;
      case CommandType::POS: {
        Diagnostics::printPosition(currentStatus());
        break;
      }
      case CommandType::ZERO:
        machine_state.x_mm = 0;
        machine_state.y_mm = 0;
        machine_state.a_steps = 0;
        machine_state.b_steps = 0;
        resetDriftReference("ZERO");
        invalidateHomed("ZERO logical origin reset");
        logMessage("ZERO logical origin reset; this is not homing");
        break;
      case CommandType::TEST_A:
      case CommandType::TEST_B: {
        const bool motor_a = command.type == CommandType::TEST_A;
        if (ensureTmcReadyForMotion(motor_a ? "TEST_A" : "TEST_B")) {
          handleSingleMotor(motor_a, command.steps, command.feed_mm_min);
        } else {
          recordMotorTestResult(motor_a, command.steps, "REJECT");
          logMessage("TEST_%c result=REJECTED steps=%+ld reason=tmc_not_ready",
                     motor_a ? 'A' : 'B', static_cast<long>(command.steps));
        }
        break;
      }
      case CommandType::TEST_MODE:
        handleTestMode(command);
        break;
      case CommandType::TEST_JOG:
        handleTestJog(command);
        break;
      case CommandType::SERVO_STATUS:
        handleServoStatus(command.servo_id);
        break;
      case CommandType::SERVO_OFF:
        logMessage("SERVO_OFF id=%u result=%s", command.servo_id,
                   sts_servo_bus.torqueOff(command.servo_id) ? "OK" : "FAILED");
        break;
      case CommandType::SERVO_ANGLE:
      case CommandType::SERVO_TURN:
        handleServoMove(command);
        break;
      case CommandType::AB_TIMED:
        if (ensureTmcReadyForMotion("AB_TIMED")) {
          handleABTimed(command);
        }
        break;
      case CommandType::XY:
        if (ensureTmcReadyForMotion("XY")) {
          handleXYBatch(command);
        }
        break;
      case CommandType::DWELL:
        logMessage("DWELL P=%lums", static_cast<unsigned long>(command.dwell_ms));
        vTaskDelay(pdMS_TO_TICKS(command.dwell_ms));
        break;
      case CommandType::PEN_UP:
        if (!PEN_SERVO_ENABLED) {
          if (command.from_gcode) {
            pen_controller.penUp();
            machine_state.pen_down = false;
            logMessage("PEN UP skipped: no pen servo assigned");
          } else {
            logMessage("ERROR: PENUP disabled: no pen servo assigned (PEN_SERVO_ENABLED=false)");
          }
          break;
        }
        if (pen_controller.penUp()) {
          machine_state.pen_down = false;
          postLedStatus(LedStatus::DRAWING_PEN_UP);
          logMessage("PEN UP");
        } else {
          logMessage("ERROR: PEN UP STS3020 no response");
          postLedStatus(LedStatus::ERROR);
          if (job_controller.isActive()) {
            job_controller.markFailed("pen_servo_no_response");
            safety_manager.setAlarm("pen servo no response");
            machine_state.alarmed = true;
            invalidateHomed("pen servo no response");
            clearMotionQueues("pen servo fault");
          }
        }
        break;
      case CommandType::PEN_DOWN:
        if (!PEN_SERVO_ENABLED) {
          if (command.from_gcode) {
            pen_controller.penDown();
            machine_state.pen_down = true;
            logMessage("PEN DOWN skipped: no pen servo assigned");
          } else {
            logMessage("ERROR: PENDOWN disabled: no pen servo assigned (PEN_SERVO_ENABLED=false)");
          }
          break;
        }
        if (safety_manager.isAlarmed()) {
          logMessage("ERROR: PEN DOWN blocked; machine alarmed");
          break;
        }
        if (pen_controller.penDown()) {
          machine_state.pen_down = true;
          postLedStatus(LedStatus::DRAWING_PEN_DOWN);
          logMessage("PEN DOWN");
        } else {
          logMessage("ERROR: PEN DOWN STS3020 no response");
          postLedStatus(LedStatus::ERROR);
          if (job_controller.isActive()) {
            job_controller.markFailed("pen_servo_no_response");
            safety_manager.setAlarm("pen servo no response");
            machine_state.alarmed = true;
            invalidateHomed("pen servo no response");
            clearMotionQueues("pen servo fault");
          }
        }
        break;
      case CommandType::PEN_STATUS: {
        if (!PEN_SERVO_ENABLED) {
          logMessage("PEN_STATUS disabled: no pen servo assigned (use SERVO_STATUS <id>)");
          break;
        }
        int position = -1;
        if (pen_controller.ping() && pen_controller.readPosition(position)) {
          logMessage("PEN_STATUS OK id=%u position=%d (0..%u) commanded=%s",
                     PEN_SERVO_ID, position, PEN_SERVO_MAX_POSITION,
                     pen_controller.isPenDown() ? "DOWN" : "UP");
        } else {
          logMessage("ERROR: PEN_STATUS STS3020 no response");
        }
        break;
      }
      case CommandType::SELFTEST:
        Diagnostics::runSelfTest();
        break;
      case CommandType::TMC_INIT:
        machine_state.tmc_ready = tmc_manager.begin();
        last_tmc_health = machine_state.tmc_ready ? TMC2209Health::OK
                                                  : TMC2209Health::NO_REPLY;
        last_tmc_check_ms = millis();
        break;
      case CommandType::TMC_STATUS:
        tmc_manager.printStatus();
        Diagnostics::runTmcDiagnostics();
        break;
      case CommandType::HOME:
        if (ensureTmcReadyForMotion("HOME")) {
          clearMotionQueues("HOME start");
          postLedStatus(LedStatus::HOMING);
          if (homing_controller.runHome(stepper_backend, safety_manager,
                                        machine_state)) {
            resetDriftReference("HOME");
            postLedStatus(LedStatus::COMPLETED);
          } else {
            postLedStatus(LedStatus::ERROR);
          }
          clearMotionQueues("HOME end");
        }
        break;
      case CommandType::HOME_X:
        if (ensureTmcReadyForMotion("HOME_X")) {
          clearMotionQueues("HOME_X start");
          postLedStatus(LedStatus::HOMING);
          if (homing_controller.runHomeX(stepper_backend, safety_manager,
                                         machine_state)) {
            resetDriftReference("HOME_X");
            postLedStatus(LedStatus::COMPLETED);
          } else {
            postLedStatus(LedStatus::ERROR);
          }
          clearMotionQueues("HOME_X end");
        }
        break;
      case CommandType::HOME_Y:
        if (ensureTmcReadyForMotion("HOME_Y")) {
          clearMotionQueues("HOME_Y start");
          postLedStatus(LedStatus::HOMING);
          if (homing_controller.runHomeY(stepper_backend, safety_manager,
                                         machine_state)) {
            resetDriftReference("HOME_Y");
            postLedStatus(LedStatus::COMPLETED);
          } else {
            postLedStatus(LedStatus::ERROR);
          }
          clearMotionQueues("HOME_Y end");
        }
        break;
      case CommandType::HOME_STATUS:
        Diagnostics::printHomingStatus(currentStatus());
        break;
      case CommandType::LIMIT_STATUS:
        safety_manager.poll();
        Diagnostics::printLimitStatus(currentStatus());
        break;
      case CommandType::ALARM_CLEAR:
        safety_manager.poll();
        if (safety_manager.xLimitRawActive() || safety_manager.yLimitRawActive() ||
            safety_manager.xLimitActive() || safety_manager.yLimitActive()) {
          invalidateHomed("ALARM_CLEAR with limit active");
        }
        safety_manager.clearAlarm();
        machine_state.alarmed = false;
        postLedStatus(LedStatus::IDLE);
        logMessage("ALARM_CLEAR complete");
        break;
      case CommandType::ABORT:
        clearMotionAbort();
        stepper_backend.stop();
        clearMotionQueues("ABORT");
        safety_manager.setAlarm("abort requested");
        machine_state.alarmed = true;
        invalidateHomed("abort requested");
        if (job_controller.isActive() || job_controller.isRunning()) {
          job_controller.markAborted("abort requested");
        }
        postLedStatus(LedStatus::ERROR);
        logMessage("ABORT complete");
        break;
      case CommandType::JOB_BEGIN:
        if (machine_state.test_mode_active) {
          logMessage("REJECT: JOB_BEGIN test_mode_active (send TEST_MODE OFF, then HOME)");
          break;
        }
        if (prepareJobBeginAutoHome() &&
            job_controller.beginJob(currentJobPreflight(), safety_manager,
                                    machine_state, pen_controller,
                                    tmc_manager)) {
          resetGcodeModalForJob();
          resetDriftReference("JOB_BEGIN");
          postLedStatus(machine_state.pen_down ? LedStatus::DRAWING_PEN_DOWN
                                               : LedStatus::DRAWING_PEN_UP);
        } else if (safety_manager.isAlarmed()) {
          postLedStatus(LedStatus::ERROR);
        }
        break;
      case CommandType::JOB_END:
        handleJobEnd();
        break;
      case CommandType::JOB_ABORT:
        clearMotionQueues("JOB_ABORT");
        if (job_controller.abortJob("job abort requested")) {
          clearMotionAbort();
          stepper_backend.stop();
          safety_manager.setAlarm("job abort requested");
          machine_state.alarmed = true;
          invalidateHomed("job abort requested");
          postLedStatus(LedStatus::WARNING);
          logMessage("JOB_ABORT complete");
        }
        break;
      case CommandType::JOB_STATUS:
        job_controller.printStatus();
        break;
      case CommandType::MELODY:
        motor_melody_controller.play(stepper_backend, tmc_manager,
                                     safety_manager);
        break;
      case CommandType::GCODE: {
        CommandMessage translated{};
        const GcodeInterpreterResult result =
            translateGcodeCommand(command, machine_state, translated);
        if (result == GcodeInterpreterResult::ERROR ||
            result == GcodeInterpreterResult::MODAL_UPDATE) {
          break;
        }
        if (rejectDisallowedJobCommand(translated)) {
          break;
        }
        if (translated.type == CommandType::XY) {
          if (ensureTmcReadyForMotion("GCODE_XY")) {
            handleXYBatch(translated);
          }
        } else {
          stashPendingCommand(translated);
        }
        break;
      }
      case CommandType::LED:
      case CommandType::LED_PIXEL:
      case CommandType::LED_OFF:
      case CommandType::LED_PATTERN:
      case CommandType::LED_BRIGHTNESS:
      case CommandType::LED_PARAM:
      case CommandType::LED_AUTO:
      case CommandType::LED_STATUS_SET:
      case CommandType::LED_STATUS:
      case CommandType::INVALID:
        break;
    }
    machine_state.alarmed = safety_manager.isAlarmed();
    if (machine_state.alarmed && (job_controller.isActive() ||
                                  job_controller.isRunning())) {
      job_controller.markFailed(safety_manager.alarmReason());
      postLedStatus(LedStatus::ERROR);
    }
    syncJobActiveFlag();
    publishStatus();
  }
}
