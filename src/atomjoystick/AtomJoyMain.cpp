// Atom JoyStick (v1.0, AtomS3) remote for the CoreXY UFO.
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
//   Left button 1 s     HOME
//   Screen button 1 s   ALARM_CLEAR
//   Left stick press    speed SLOW -> FAST -> VERY FAST -> ULTRA -> MAX -> SLOW
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
constexpr uint32_t DISPLAY_PERIOD_MS = 100;
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
  if (longPress(left_button, buttonPressed(BUTTON_A))) ++packet.home_count;
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

// ---------------------------------------------------------------------------
// LCD (128 x 128)
// ---------------------------------------------------------------------------
// Status chip + battery / X Y / speed bar / both sticks / bottom line
// (long-press progress, last event, or a control hint).

constexpr int SCREEN_W = 128;

void drawStatusChip(const JoyStatusPacket& shown, bool linked) {
  const char* text = "READY";
  uint16_t color = TFT_GREEN;
  if (!linked) {
    text = "NO LINK";
    color = TFT_DARKGREY;
  } else if (shown.flags & JOY_STATUS_ALARM) {
    text = "ALARM";
    color = TFT_RED;
  } else if (shown.flags & JOY_STATUS_HOMING) {
    text = "HOMING";
    color = TFT_YELLOW;
  } else if (shown.flags & JOY_STATUS_UNHOMED_JOG) {
    text = "SLOW JOG";
    color = TFT_ORANGE;
  } else if (shown.flags & JOY_STATUS_JOG_ACTIVE) {
    text = "JOG";
    color = TFT_CYAN;
  } else if (!(shown.flags & JOY_STATUS_HOMED)) {
    text = "NEED HOME";
    color = TFT_BLUE;
  }
  canvas.fillRoundRect(0, 0, 94, 18, 5, color);
  canvas.setFont(&fonts::Font2);
  canvas.setTextColor(color == TFT_YELLOW || color == TFT_CYAN || color == TFT_GREEN ||
                              color == TFT_ORANGE
                          ? TFT_BLACK
                          : TFT_WHITE);
  canvas.setTextDatum(middle_center);
  canvas.drawString(text, 47, 9);
}

void drawBattery(uint16_t battery_mv) {
  // 1S LiPo: 3.4 V empty .. 4.1 V full.
  int percent = (static_cast<int>(battery_mv) - 3400) * 100 / 700;
  if (percent < 0) percent = 0;
  if (percent > 100) percent = 100;
  const uint16_t color = percent > 40 ? TFT_GREEN : percent > 15 ? TFT_YELLOW : TFT_RED;
  canvas.drawRect(98, 3, 26, 12, TFT_LIGHTGREY);
  canvas.fillRect(124, 6, 3, 6, TFT_LIGHTGREY);
  canvas.fillRect(100, 5, 22 * percent / 100, 8, color);
}

void drawPosition(const JoyStatusPacket& shown, bool linked) {
  const bool valid = linked && (shown.flags & JOY_STATUS_HOMED);
  canvas.setFont(&fonts::Font0);
  canvas.setTextSize(2);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(valid ? TFT_WHITE : TFT_DARKGREY);
  char text[16];
  snprintf(text, sizeof(text), "X%6.1f", linked ? shown.x_mm : 0.0f);
  canvas.drawString(text, 2, 23);
  snprintf(text, sizeof(text), "Y%6.1f", linked ? shown.y_mm : 0.0f);
  canvas.drawString(text, 2, 41);
  canvas.setTextSize(1);
}

void drawSpeed(const JoyStatusPacket& shown, bool linked) {
  static const uint16_t LEVEL_COLORS[JOY_SPEED_LEVEL_COUNT] = {
      TFT_CYAN, TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_RED};
  const uint8_t level = joySpeedLevel(speed_level);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(top_left);
  canvas.setTextColor(LEVEL_COLORS[level]);
  canvas.drawString(joySpeedLevelName(level), 2, 61);
  // Without HOME the Core2 caps XY at SLOW whatever the level.
  if (linked && !(shown.flags & JOY_STATUS_HOMED)) {
    canvas.setTextColor(TFT_ORANGE);
    canvas.setTextDatum(top_right);
    canvas.drawString("NO HOME:SLOW", SCREEN_W - 1, 61);
  }
  constexpr int SEG_W = 24;
  for (int i = 0; i < JOY_SPEED_LEVEL_COUNT; ++i) {
    const int x = 1 + i * (SEG_W + 1);
    if (i <= level) canvas.fillRect(x, 71, SEG_W, 6, LEVEL_COLORS[level]);
    else canvas.drawRect(x, 71, SEG_W, 6, TFT_DARKGREY);
  }
}

void drawStick(int cx, int cy, int16_t x, int16_t y, const char* label) {
  constexpr int R = 13;
  canvas.drawCircle(cx, cy, R, TFT_DARKGREY);
  canvas.drawFastHLine(cx - 3, cy, 7, TFT_DARKGREY);
  canvas.drawFastVLine(cx, cy - 3, 7, TFT_DARKGREY);
  const bool active = x != 0 || y != 0;
  const int dx = static_cast<int>(x) * (R - 3) / JOY_AXIS_FULL_SCALE;
  const int dy = -static_cast<int>(y) * (R - 3) / JOY_AXIS_FULL_SCALE;
  canvas.fillCircle(cx + dx, cy + dy, 3, active ? TFT_WHITE : TFT_DARKGREY);
  canvas.setFont(&fonts::Font0);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setTextDatum(middle_left);
  canvas.drawString(label, cx + R + 3, cy);
}

// Fraction of LONG_PRESS_MS a button has been held (0 when not held).
float longPressProgress(const Button& button) {
  if (!button.pressed || button.long_fired) return 0.0f;
  const float held = static_cast<float>(millis() - button.since_ms) / LONG_PRESS_MS;
  return held > 1.0f ? 1.0f : held;
}

void drawBottomLine(const JoyStatusPacket& shown, bool linked) {
  const float home = longPressProgress(left_button);
  const float clear = longPressProgress(screen_button);
  canvas.setFont(&fonts::Font0);
  canvas.setTextDatum(middle_center);
  if (home > 0.0f || clear > 0.0f) {
    const float progress = home > 0.0f ? home : clear;
    const uint16_t color = home > 0.0f ? TFT_YELLOW : TFT_SKYBLUE;
    canvas.drawRect(0, 114, SCREEN_W, 14, color);
    canvas.fillRect(1, 115, static_cast<int>((SCREEN_W - 2) * progress), 12, color);
    canvas.setTextColor(TFT_BLACK);
    canvas.drawString(home > 0.0f ? "HOME..." : "ALARM CLEAR...", SCREEN_W / 2, 121);
    return;
  }
  if (linked && shown.last_result[0] != '\0') {
    canvas.setTextColor(TFT_LIGHTGREY);
    canvas.drawString(shown.last_result, SCREEN_W / 2, 121);
  } else {
    canvas.setTextColor(TFT_DARKGREY);
    canvas.drawString("L:HOME  R:STOP", SCREEN_W / 2, 121);
  }
}

void drawStatus() {
  JoyStatusPacket shown{};
  bool linked = false;
  portENTER_CRITICAL(&status_mux);
  shown = status;
  linked = have_status && millis() - status_ms <= STATUS_TIMEOUT_MS;
  portEXIT_CRITICAL(&status_mux);

  canvas.fillSprite(TFT_BLACK);
  drawStatusChip(shown, linked);
  drawBattery(packet.battery_mv);
  drawPosition(shown, linked);
  drawSpeed(shown, linked);
  drawStick(18, 95, packet.axes[JOY_AXIS_LEFT_X], packet.axes[JOY_AXIS_LEFT_Y], "XY");
  drawStick(78, 95, packet.axes[JOY_AXIS_RIGHT_X], packet.axes[JOY_AXIS_RIGHT_Y], "Z/A");
  drawBottomLine(shown, linked);
  if (center_suspect) {
    canvas.fillRoundRect(4, 40, SCREEN_W - 8, 40, 6, TFT_RED);
    canvas.setFont(&fonts::Font0);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(middle_center);
    canvas.drawString("RELEASE STICKS", SCREEN_W / 2, 53);
    canvas.drawString("AND REBOOT", SCREEN_W / 2, 67);
  }
  canvas.setTextDatum(top_left);
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
