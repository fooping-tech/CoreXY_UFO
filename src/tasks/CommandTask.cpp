#include <Arduino.h>
#include "AppContext.h"
#include "CommandDispatcher.h"
#include "Core2PinMap.h"
#include "JoystickMapping.h"
#include <string.h>

namespace {
bool isLedCommand(CommandType type) {
  return type == CommandType::LED || type == CommandType::LED_PIXEL ||
         type == CommandType::LED_OFF || type == CommandType::LED_PATTERN ||
         type == CommandType::LED_BRIGHTNESS || type == CommandType::LED_PARAM ||
         type == CommandType::LED_AUTO ||
         type == CommandType::LED_STATUS_SET ||
         type == CommandType::LED_STATUS;
}

bool isReliableCommand(CommandType type) {
  return type == CommandType::GCODE || type == CommandType::XY ||
         type == CommandType::AB_TIMED || type == CommandType::JOB_END;
}

bool enqueueReliableCommand(const CommandMessage& command) {
  bool logged_wait = false;
  for (;;) {
    if (xQueueSend(command_queue, &command, pdMS_TO_TICKS(50)) == pdTRUE) {
      return true;
    }
    if (!logged_wait) {
      logMessage("CommandQueue full; waiting to queue %s", command.name);
      logged_wait = true;
    }
    (void)isMotionAbortRequested();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
}

void commandTask(void*) {
  char line[128] = {};
  size_t length = 0;
  bool discard_until_newline = false;
  for (;;) {
    while (Serial.available() > 0) {
      const char c = static_cast<char>(Serial.read());
      if (c == '\r' || c == '\n') {
        if (discard_until_newline) {
          discard_until_newline = false;
          length = 0;
          continue;
        }
        line[length] = '\0';
        if (length > 0) {
          CommandMessage command = CommandDispatcher::parse(line);
          if (command.type == CommandType::INVALID) {
            if (strcmp(command.name, "JOG") == 0) {
              logMessage("NACK_JOG reason=bad_args (%s)", command.error);
            } else {
              logMessage("ERROR: %s", command.error);
            }
          } else if (command.type == CommandType::JOG) {
            // No ACK: 20 Hz refreshes would flood the 24-line LogQueue.
            // motionTask reports JOG start/end and NACK_JOG.
            JogRequest request;
            request.vx_mm_s = command.jog_vx_mm_s;
            request.vy_mm_s = command.jog_vy_mm_s;
            request.z_counts_s = command.jog_z_counts_s;
            request.arm_counts_s = command.jog_arm_counts_s;
            request.time_ms = millis();
            request.link = true;
            xQueueOverwrite(serial_jog_queue, &request);
          } else if (command.type == CommandType::STOP) {
            // Takes effect at once like ABORT; the front slot makes motionTask
            // handle it before commands queued earlier (which it discards).
            requestMotionStop();
            if (xQueueSendToFront(command_queue, &command, 0) != pdTRUE) {
              markStopCommandLost();
              logMessage("ACK STOP requested");
            } else {
              logMessage("ACK QUEUED STOP");
            }
          } else if (command.type == CommandType::ABORT ||
                     command.type == CommandType::JOB_ABORT) {
            requestMotionAbort();
            if (xQueueSend(command_queue, &command, 0) != pdTRUE) {
              logMessage("ACK %s requested", command.name);
            } else {
              logMessage("ACK QUEUED %s", command.name);
            }
          } else if (isLedCommand(command.type) && !NEOPIXEL_ENABLED) {
            logMessage("ERROR: NeoPixel disabled; GPIO33 reserved for STS3020 RX");
          } else if (isLedCommand(command.type) &&
                     xQueueSend(led_command_queue, &command.led, 0) != pdTRUE) {
            logMessage("ERROR: LedCommandQueue full");
          } else if (!isLedCommand(command.type) &&
                     isReliableCommand(command.type)) {
            enqueueReliableCommand(command);
            logMessage("ACK QUEUED %s", command.name);
          } else if (!isLedCommand(command.type) &&
                     xQueueSend(command_queue, &command, pdMS_TO_TICKS(50)) !=
                     pdTRUE) {
            logMessage("ERROR: CommandQueue full");
          } else {
            logMessage("ACK QUEUED %s", command.name);
          }
        }
        length = 0;
      } else if (length + 1 < sizeof(line)) {
        line[length++] = c;
      } else {
        length = 0;
        discard_until_newline = true;
        logMessage("ERROR: input line too long");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
