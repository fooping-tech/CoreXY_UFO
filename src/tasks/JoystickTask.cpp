#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include "AppContext.h"
#include "CommandDispatcher.h"
#include "JoystickMapping.h"
#include "PlotterConfig.h"

// Core 0: Atom JoyStick input. Converts the latest ESP-NOW packet into a
// JogRequest for motionTask (JogQueue) and button presses into CommandQueue
// commands. Never touches motors, servos or the TMC UART.

namespace {
constexpr uint32_t JOY_TASK_PERIOD_MS = 20;
constexpr uint32_t JOY_STATUS_PERIOD_MS = 100;

JoyMapConfig mapConfig() {
  JoyMapConfig config;
  config.deadzone = JOY_DEADZONE;
  config.servo_deadzone = JOY_SERVO_DEADZONE;
  config.feed_mm_min[JOY_SPEED_SLOW] = JOY_SLOW_FEED_MM_MIN;
  config.feed_mm_min[JOY_SPEED_FAST] = JOY_FAST_FEED_MM_MIN;
  config.feed_mm_min[JOY_SPEED_VERY_FAST] = JOY_VERY_FAST_FEED_MM_MIN;
  config.feed_mm_min[JOY_SPEED_ULTRA] = JOY_ULTRA_FEED_MM_MIN;
  config.servo_max_counts_s = JOY_SERVO_MAX_COUNTS_S;
  config.z_sign = JOY_Z_SIGN;
  config.arm_sign = JOY_ARM_SIGN;
  return config;
}

void queueButtonCommand(const char* line) {
  CommandMessage command = CommandDispatcher::parse(line);
  if (command.type == CommandType::INVALID) return;
  if (command.type == CommandType::ABORT) requestMotionAbort();
  if (xQueueSend(command_queue, &command, 0) == pdTRUE) {
    logMessage("JOY button: %s", line);
  } else {
    logMessage("ERROR: JOY button %s dropped (CommandQueue full)", line);
  }
}

void sendStatus(const StatusMessage& status, uint16_t ack_seq) {
  JoyStatusPacket packet{};
  const MachineState& machine = status.machine;
  packet.ack_seq = ack_seq;
  packet.x_mm = machine.x_mm;
  packet.y_mm = machine.y_mm;
  if (machine.test_mode_active) packet.flags |= JOY_STATUS_TEST_MODE;
  if (machine.alarmed) packet.flags |= JOY_STATUS_ALARM;
  if (machine.homed) packet.flags |= JOY_STATUS_HOMED;
  if (machine.joystick_jog_active) packet.flags |= JOY_STATUS_JOG_ACTIVE;
  if (machine.tmc_ready) packet.flags |= JOY_STATUS_TMC_READY;
  snprintf(packet.last_result, sizeof(packet.last_result), "%s",
           machine.joystick_last_event);
  joystick_link.sendStatus(packet);
}
}  // namespace

void joystickTask(void*) {
  if (!joystick_link.begin()) {
    logMessage("ERROR: JOY ESP-NOW init failed; joystick disabled");
    vTaskDelete(nullptr);
    return;
  }
  uint8_t mac[6];
  char mac_text[18];
  JoystickLink::localMac(mac);
  formatMac(mac, mac_text);
  logMessage("JOY ESP-NOW ready channel=%u core2_mac=%s",
             JOY_ESPNOW_CHANNEL, mac_text);

  const JoyMapConfig config = mapConfig();
  StatusMessage status{};
  bool was_linked = false;
  bool have_baseline = false;
  uint8_t last_estop = 0;
  uint8_t last_toggle = 0;
  uint8_t last_alarm_clear = 0;
  uint32_t last_status_ms = 0;
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(JOY_TASK_PERIOD_MS));
    if (joystick_status_queue != nullptr) {
      xQueuePeek(joystick_status_queue, &status, 0);
    }

    JoyCommandPacket packet{};
    uint32_t age_ms = 0;
    const bool linked =
        joystick_link.latest(packet, age_ms) && age_ms <= JOY_LINK_TIMEOUT_MS;
    JogRequest request;
    if (linked) {
      request = joyMapAxes(packet.axes, packet.flags & JOY_FLAG_SPEED_MASK, config);
      // Counters seen first after (re)link are the baseline, not presses:
      // an Atom reboot resets them and must not trigger an action.
      if (have_baseline) {
        if (packet.estop_count != last_estop) queueButtonCommand("ABORT");
        if (packet.test_mode_toggle_count != last_toggle) {
          queueButtonCommand(status.machine.test_mode_active ? "TEST_MODE OFF"
                                                             : "TEST_MODE ON");
        }
        if (packet.alarm_clear_count != last_alarm_clear) queueButtonCommand("ALARM_CLEAR");
      }
      have_baseline = true;
      last_estop = packet.estop_count;
      last_toggle = packet.test_mode_toggle_count;
      last_alarm_clear = packet.alarm_clear_count;
    }
    request.link = linked;
    request.time_ms = millis();
    if (linked != was_linked) {
      if (linked) {
        uint8_t peer[6];
        char peer_text[18];
        joystick_link.pairedMac(peer);
        formatMac(peer, peer_text);
        logMessage("JOY link UP atom_mac=%s", peer_text);
      } else {
        logMessage("JOY link LOST (no packet for %lums)",
                   static_cast<unsigned long>(JOY_LINK_TIMEOUT_MS));
        have_baseline = false;
      }
      was_linked = linked;
    }
    xQueueOverwrite(jog_queue, &request);

    if (linked && millis() - last_status_ms >= JOY_STATUS_PERIOD_MS) {
      last_status_ms = millis();
      sendStatus(status, packet.seq);
    }
  }
}
