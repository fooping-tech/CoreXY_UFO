#pragma once

#include <Arduino.h>
#include "CommandMessage.h"
#include "MachineState.h"
#include "HomingController.h"
#include "JobController.h"
#include "JoystickLink.h"
#include "LedPatternEngine.h"
#include "NeoPixelController.h"
#include "MotorMelodyController.h"
#include "PenController.h"
#include "SafetyManager.h"
#include "StepperBackendFastAccel.h"
#include "StsServoBus.h"
#include "TMC2209Manager.h"

extern QueueHandle_t command_queue;
extern QueueHandle_t status_queue;
extern QueueHandle_t log_queue;
extern QueueHandle_t led_command_queue;
// Atom JoyStick: latest JogRequest (Core 0 -> Core 1, 1-deep, overwrite) and a
// status copy for the joystick task (UI consumes status_queue).
extern QueueHandle_t jog_queue;
extern QueueHandle_t joystick_status_queue;
// Serial JOG: latest JogRequest (Core 0 -> Core 1, 1-deep, overwrite). Kept out
// of CommandQueue so 20 Hz JOG refreshes neither fill it nor flood the log.
extern QueueHandle_t serial_jog_queue;
extern MachineState machine_state;
extern DiagnosticsState diagnostics_state;
extern SafetyManager safety_manager;
extern StepperBackendFastAccel stepper_backend;
extern TMC2209Manager tmc_manager;
extern StsServoBus sts_servo_bus;
extern JoystickLink joystick_link;
extern PenController pen_controller;
extern NeoPixelController neopixel_controller;
extern LedPatternEngine led_pattern_engine;
extern MotorMelodyController motor_melody_controller;
extern HomingController homing_controller;
extern JobController job_controller;

void requestMotionAbort();
bool isMotionAbortRequested();
void clearMotionAbort();
// STOP: decelerate the running XY move / jog without an alarm.
void requestMotionStop();
bool isMotionStopRequested();
void clearMotionStop();
// commandTask could not queue the STOP command (CommandQueue full); motionTask
// then completes the STOP from the flag alone.
void markStopCommandLost();
bool takeStopCommandLost();
void logMessage(const char* format, ...);
void publishStatus();
void uiTask(void*);
void commandTask(void*);
void motionTask(void*);
void stepperFeedTask(void*);
void tmcTask(void*);
void safetyTask(void*);
void logTask(void*);
void joystickTask(void*);
