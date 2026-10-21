// Atom JoyStick (v1.0, AtomS3) remote for the CoreXY UFO test mode.
// Built only by the `atoms3-joystick` PlatformIO env.
//
// Reads both sticks and buttons from the on-board STM32 (I2C 0x59) and sends
// a JoyCommandPacket over ESP-NOW at 50 Hz. The Core2 answers with
// JoyStatusPacket, shown on the AtomS3 LCD. The Core2 decides everything
// about safety; this firmware only reports what the hands do.
//
// Controls:
//   Left stick          XY jog
//   Right stick up/down Z drum (servo ID 1)
//   Right stick L/R     arm (servo ID 2)
//   Right button        emergency stop (ABORT)
//   Left button 1 s     toggle test mode
//   Screen button 1 s   ALARM_CLEAR
//   Left stick press    speed SLOW -> FAST -> VERY FAST -> ULTRA -> SLOW
// Keep the sticks released while powering on (center calibration).

#include <Arduino.h>
#include <AtomJoyStick.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>
#include "JoystickProtocol.h"

namespace {
// +1 when pushing right / up raises the ADC value. Check the LCD readout and
// flip a sign if an axis is reversed. Y is inverted on the real stick
// (measured 2026-10-22: left Y moved the machine the wrong way); the right
// stick uses the same part, so its Y is inverted too.
constexpr int8_t AXIS_SIGN[JOY_AXIS_COUNT] = {1, -1, 1, -1};
// ADC counts from center to full deflection (12-bit ADC, center ~2048).
constexpr int ADC_FULL_DEFLECTION = 1700;
// A center reading this far off means a stick was held at power-on.
constexpr int ADC_CENTER_SUSPECT = 400;
constexpr uint32_t SEND_PERIOD_MS = 20;
constexpr uint32_t DISPLAY_PERIOD_MS = 200;
constexpr uint32_t STATUS_TIMEOUT_MS = 500;
constexpr uint32_t LONG_PRESS_MS = 1000;
constexpr uint8_t SDA_PIN = 38;
constexpr uint8_t SCL_PIN = 39;
const uint8_t BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

AtomJoyStick joystick;
M5Canvas canvas(&M5.Display);
uint16_t center[JOY_AXIS_COUNT] = {2048, 2048, 2048, 2048};
bool center_suspect = false;
JoyCommandPacket packet{};
uint8_t speed_level = JOY_SPEED_FAST;

portMUX_TYPE status_mux = portMUX_INITIALIZER_UNLOCKED;
JoyStatusPacket status{};
uint32_t status_ms = 0;
bool have_status = false;
uint8_t core2_mac[6] = {};
bool unicast_peer = false;

struct Button {
  bool pressed = false;
  uint32_t since_ms = 0;
  bool long_fired = false;
};
Button left_button;
Button screen_button;
bool right_button_was_pressed = false;
bool left_stick_was_pressed = false;

void onReceive(const uint8_t* mac, const uint8_t* data, int length) {
  JoyStatusPacket incoming{};
  if (mac == nullptr || length < 0 ||
      !joyPacketValid(data, static_cast<size_t>(length), incoming)) {
    return;
  }
  portENTER_CRITICAL(&status_mux);
  status = incoming;
  status_ms = millis();
  have_status = true;
  memcpy(core2_mac, mac, 6);
  portEXIT_CRITICAL(&status_mux);
}

uint16_t readAxisRaw(int axis) {
  switch (axis) {
    case JOY_AXIS_LEFT_X: return joystick.getJoy1ADCValueX(_12bit);
    case JOY_AXIS_LEFT_Y: return joystick.getJoy1ADCValueY(_12bit);
    case JOY_AXIS_RIGHT_X: return joystick.getJoy2ADCValueX(_12bit);
    default: return joystick.getJoy2ADCValueY(_12bit);
  }
}

// Buttons read 0 while pressed (BUTTON_1/2 = stick press, BUTTON_A/B = L/R).
bool buttonPressed(button_t button) { return joystick.getButtonValue(button) == 0; }

void calibrateCenter() {
  for (int axis = 0; axis < JOY_AXIS_COUNT; ++axis) {
    uint32_t sum = 0;
    for (int i = 0; i < 16; ++i) {
      sum += readAxisRaw(axis);
      delay(5);
    }
    const uint16_t value = static_cast<uint16_t>(sum / 16);
    if (abs(static_cast<int>(value) - 2048) > ADC_CENTER_SUSPECT) {
      center_suspect = true;
      center[axis] = 2048;
    } else {
      center[axis] = value;
    }
  }
}

int16_t readAxis(int axis) {
  const int delta = (static_cast<int>(readAxisRaw(axis)) - center[axis]) * AXIS_SIGN[axis];
  long value = static_cast<long>(delta) * JOY_AXIS_FULL_SCALE / ADC_FULL_DEFLECTION;
  if (value > JOY_AXIS_FULL_SCALE) value = JOY_AXIS_FULL_SCALE;
  if (value < -JOY_AXIS_FULL_SCALE) value = -JOY_AXIS_FULL_SCALE;
  return static_cast<int16_t>(value);
}

// Counts one press after the button is held for LONG_PRESS_MS.
bool longPress(Button& button, bool pressed) {
  const uint32_t now = millis();
  if (pressed && !button.pressed) {
    button.since_ms = now;
    button.long_fired = false;
  }
  button.pressed = pressed;
  if (pressed && !button.long_fired && now - button.since_ms >= LONG_PRESS_MS) {
    button.long_fired = true;
    return true;
  }
  return false;
}

void addPeer(const uint8_t mac[6]) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = JOY_ESPNOW_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  if (!esp_now_is_peer_exist(mac)) esp_now_add_peer(&peer);
}

void sendPacket() {
  for (int axis = 0; axis < JOY_AXIS_COUNT; ++axis) packet.axes[axis] = readAxis(axis);

  if (buttonPressed(BUTTON_B) && !right_button_was_pressed) ++packet.estop_count;
  right_button_was_pressed = buttonPressed(BUTTON_B);
  if (longPress(left_button, buttonPressed(BUTTON_A))) ++packet.test_mode_toggle_count;
  M5.update();
  if (longPress(screen_button, M5.BtnA.isPressed())) ++packet.alarm_clear_count;
  const bool left_stick = buttonPressed(BUTTON_1);
  if (left_stick && !left_stick_was_pressed) {
    speed_level = static_cast<uint8_t>((speed_level + 1) % JOY_SPEED_LEVEL_COUNT);
  }
  left_stick_was_pressed = left_stick;

  packet.flags = speed_level & JOY_FLAG_SPEED_MASK;
  packet.battery_mv = static_cast<uint16_t>(joystick.getBattery1Voltage() * 1000.0f);
  ++packet.seq;
  joyFinalizePacket(packet);

  uint8_t target[6];
  bool linked = false;
  portENTER_CRITICAL(&status_mux);
  linked = have_status && millis() - status_ms <= STATUS_TIMEOUT_MS;
  memcpy(target, core2_mac, 6);
  portEXIT_CRITICAL(&status_mux);
  if (linked) {
    if (!unicast_peer) {
      addPeer(target);
      unicast_peer = true;
    }
  } else {
    // Broadcast until a Core2 answers (it pairs with the first Atom it hears).
    memcpy(target, BROADCAST_MAC, 6);
  }
  esp_now_send(target, reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void drawStatus() {
  JoyStatusPacket shown{};
  bool linked = false;
  portENTER_CRITICAL(&status_mux);
  shown = status;
  linked = have_status && millis() - status_ms <= STATUS_TIMEOUT_MS;
  portEXIT_CRITICAL(&status_mux);

  canvas.fillSprite(TFT_BLACK);
  canvas.setTextSize(1);
  canvas.setCursor(0, 0);
  canvas.setTextColor(TFT_CYAN);
  canvas.println("UFO JOY");
  canvas.setTextColor(linked ? TFT_GREEN : TFT_RED);
  canvas.printf("Link: %s\n", linked ? "OK" : "NO");
  canvas.setTextColor((shown.flags & JOY_STATUS_TEST_MODE) && linked ? TFT_YELLOW
                                                                     : TFT_WHITE);
  canvas.printf("TEST: %s\n", (shown.flags & JOY_STATUS_TEST_MODE) && linked ? "ON" : "OFF");
  canvas.setTextColor((shown.flags & JOY_STATUS_ALARM) && linked ? TFT_RED : TFT_WHITE);
  canvas.printf("ALARM: %s\n", (shown.flags & JOY_STATUS_ALARM) && linked ? "YES" : "no");
  canvas.setTextColor(TFT_WHITE);
  canvas.printf("JOG: %s\n", (shown.flags & JOY_STATUS_JOG_ACTIVE) ? "RUN" : "-");
  canvas.setTextColor(speed_level >= JOY_SPEED_VERY_FAST ? TFT_ORANGE : TFT_WHITE);
  canvas.printf("SPD: %s\n", joySpeedLevelName(speed_level));
  canvas.setTextColor(TFT_WHITE);
  canvas.printf("BAT: %.2fV\n", packet.battery_mv / 1000.0f);
  canvas.printf("L %5d %5d\n", packet.axes[JOY_AXIS_LEFT_X], packet.axes[JOY_AXIS_LEFT_Y]);
  canvas.printf("R %5d %5d\n", packet.axes[JOY_AXIS_RIGHT_X], packet.axes[JOY_AXIS_RIGHT_Y]);
  if (center_suspect) {
    canvas.setTextColor(TFT_RED);
    canvas.println("CENTER? reboot");
    canvas.println("sticks released");
  }
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.println(linked ? shown.last_result : "");
  canvas.pushSprite(0, 0);
}
}  // namespace

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setRotation(0);
  canvas.createSprite(M5.Display.width(), M5.Display.height());
  canvas.fillSprite(TFT_BLACK);
  canvas.setCursor(0, 0);
  canvas.println("UFO JOY");
  canvas.println("I2C...");
  canvas.pushSprite(0, 0);

  while (!joystick.begin(&Wire, ATOM_JOYSTICK_ADDR, SDA_PIN, SCL_PIN, 400000U)) {
    Serial.println("Atom JoyStick not found");
    delay(500);
  }
  calibrateCenter();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(JOY_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
  }
  esp_now_register_recv_cb(onReceive);
  addPeer(BROADCAST_MAC);
  Serial.printf("UFO JOY atom_mac=%s channel=%u center=%u,%u,%u,%u%s\n",
                WiFi.macAddress().c_str(), JOY_ESPNOW_CHANNEL, center[0], center[1],
                center[2], center[3], center_suspect ? " (stick held at boot?)" : "");
}

void loop() {
  static uint32_t last_send_ms = 0;
  static uint32_t last_draw_ms = 0;
  const uint32_t now = millis();
  if (now - last_send_ms >= SEND_PERIOD_MS) {
    last_send_ms = now;
    sendPacket();
  }
  if (now - last_draw_ms >= DISPLAY_PERIOD_MS) {
    last_draw_ms = now;
    drawStatus();
  }
  delay(1);
}
