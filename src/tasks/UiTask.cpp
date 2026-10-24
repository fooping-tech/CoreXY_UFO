#include <Arduino.h>
#include <M5Unified.h>
#include "AppContext.h"
#include "Core2PinMap.h"
#include "PlotterConfig.h"
#include <math.h>
#include <string.h>

namespace {
enum class UiPage : uint8_t {
  STATUS,
  CONTROL,
  DETAIL,
};

// BtnB長押しで入る専用テストモードのページ。
enum class TestPage : uint8_t {
  MOTOR,
  DIAG,
};

struct Rect {
  int16_t x;
  int16_t y;
  int16_t w;
  int16_t h;
};

constexpr uint16_t COLOR_BG = 0x0841;
constexpr uint16_t COLOR_PANEL = 0x10A2;
constexpr uint16_t COLOR_PANEL_2 = 0x18E3;
constexpr uint16_t COLOR_TEXT = TFT_WHITE;
constexpr uint16_t COLOR_MUTED = 0x9CF3;
constexpr uint16_t COLOR_ACCENT = 0x05FF;
constexpr uint16_t COLOR_GREEN = 0x37E6;
constexpr uint16_t COLOR_YELLOW = 0xFEA0;
constexpr uint16_t COLOR_RED = 0xF946;
constexpr uint16_t COLOR_BLUE = 0x3D7F;
constexpr uint16_t COLOR_DISABLED = 0x4A69;
constexpr uint16_t COLOR_ORANGE = 0xFC60;

constexpr float UI_JOG_STEP_MM = 1.0f;
constexpr float UI_JOG_FEED_MM_MIN = 900.0f;

UiPage current_page = UiPage::STATUS;
bool force_redraw = true;
uint32_t last_clock_redraw_ms = 0;
char ui_notice[48] = "Ready";
uint32_t ui_notice_until_ms = 0;
M5Canvas ui_canvas(&M5.Display);
LovyanGFX* ui_gfx = &M5.Display;
float ui_jog_base_x_mm = 0.0f;
float ui_jog_base_y_mm = 0.0f;
bool ui_have_jog_base = false;
bool test_mode = false;
TestPage test_page = TestPage::MOTOR;
uint8_t test_step_index = TEST_MODE_MOTOR_STEP_DEFAULT_INDEX;
constexpr uint8_t kTestStepOptionCount =
    sizeof(TEST_MODE_MOTOR_STEP_OPTIONS) /
    sizeof(TEST_MODE_MOTOR_STEP_OPTIONS[0]);

// Test mode layout.
constexpr Rect kTestLimitStrip{10, 42, 300, 24};
constexpr Rect kTestAPlus{10, 72, 145, 38};
constexpr Rect kTestAMinus{165, 72, 145, 38};
constexpr Rect kTestBPlus{10, 116, 145, 38};
constexpr Rect kTestBMinus{165, 116, 145, 38};
constexpr Rect kTestStep{10, 160, 145, 28};
constexpr Rect kTestStop{165, 160, 145, 28};
constexpr Rect kDiagLimitX{10, 42, 145, 40};
constexpr Rect kDiagLimitY{165, 42, 145, 40};
constexpr Rect kDiagSelftest{10, 88, 94, 28};
constexpr Rect kDiagTmcInit{113, 88, 94, 28};
constexpr Rect kDiagTmcStatus{216, 88, 94, 28};
constexpr Rect kDiagSummary{10, 122, 300, 22};
constexpr Rect kDiagTmcA{10, 150, 145, 58};
constexpr Rect kDiagTmcB{165, 150, 145, 58};

LovyanGFX& gfx() {
  return *ui_gfx;
}

const char* pageName(UiPage page) {
  switch (page) {
    case UiPage::STATUS:
      return "Status";
    case UiPage::CONTROL:
      return "Control";
    case UiPage::DETAIL:
      return "Detail";
  }
  return "";
}

uint16_t safetyColor(const MachineState& state) {
  if (state.alarmed) return COLOR_RED;
  if (state.jog_unhomed) return COLOR_ORANGE;
  if (state.homing_active) return COLOR_YELLOW;
  if (state.motion_active) return COLOR_YELLOW;
  if (state.job_active) return COLOR_BLUE;
  if (!state.homed) return COLOR_BLUE;
  return COLOR_GREEN;
}

const char* safetyText(const MachineState& state) {
  if (state.alarmed) return "ALARM";
  if (state.jog_unhomed) return "SLOW JOG";
  if (state.homing_active) return "HOMING";
  if (state.motion_active) return "MOVING";
  if (state.job_active) return "RUNNING";
  if (!state.homed) return "NEED HOME";
  return "READY";
}

bool contains(const Rect& rect, int16_t x, int16_t y) {
  return x >= rect.x && x < rect.x + rect.w && y >= rect.y &&
         y < rect.y + rect.h;
}

void setNotice(const char* text) {
  snprintf(ui_notice, sizeof(ui_notice), "%s", text);
  ui_notice_until_ms = millis() + 1800;
  force_redraw = true;
}

void drawTextInRect(const char* text, const Rect& rect, uint16_t color,
                    uint8_t size = 1) {
  gfx().setTextSize(size);
  gfx().setTextColor(color, COLOR_PANEL);
  gfx().setTextDatum(middle_center);
  gfx().drawString(text, rect.x + rect.w / 2, rect.y + rect.h / 2);
  gfx().setTextDatum(top_left);
}

void drawRoundPanel(const Rect& rect, uint16_t color = COLOR_PANEL) {
  gfx().fillRoundRect(rect.x, rect.y, rect.w, rect.h, 8, color);
}

void drawButton(const Rect& rect, const char* label, uint16_t color,
                bool enabled = true, uint8_t size = 1) {
  const uint16_t fill = enabled ? color : COLOR_DISABLED;
  const uint16_t text = enabled ? COLOR_TEXT : COLOR_MUTED;
  gfx().fillRoundRect(rect.x, rect.y, rect.w, rect.h, 7, fill);
  gfx().drawRoundRect(rect.x, rect.y, rect.w, rect.h, 7,
                           enabled ? TFT_WHITE : COLOR_PANEL_2);
  drawTextInRect(label, rect, text, size);
}

void drawHeader(const StatusMessage& status) {
  const MachineState& state = status.machine;
  gfx().fillRect(0, 0, 320, 34, COLOR_BG);
  gfx().setTextDatum(top_left);
  gfx().setTextSize(2);
  gfx().setTextColor(COLOR_TEXT, COLOR_BG);
  gfx().drawString("CoreXY", 10, 8);

  gfx().setTextSize(1);
  if (test_mode) {
    gfx().fillRoundRect(80, 9, 110, 18, 9, COLOR_ORANGE);
    gfx().setTextColor(COLOR_BG, COLOR_ORANGE);
    gfx().setTextDatum(middle_center);
    gfx().drawString(test_page == TestPage::MOTOR ? "TEST: MOTOR"
                                                  : "TEST: DIAG",
                     135, 18);
    gfx().setTextDatum(top_left);
  } else {
    gfx().setTextColor(COLOR_MUTED, COLOR_BG);
    gfx().drawString(pageName(current_page), 82, 14);
  }

  const Rect pill{218, 6, 92, 22};
  gfx().fillRoundRect(pill.x, pill.y, pill.w, pill.h, 11,
                           safetyColor(state));
  gfx().setTextColor(COLOR_BG, safetyColor(state));
  gfx().setTextDatum(middle_center);
  gfx().drawString(safetyText(state), pill.x + pill.w / 2,
                        pill.y + pill.h / 2);
  gfx().setTextDatum(top_left);
}

void drawFooter() {
  const int16_t y = 214;
  gfx().fillRect(0, y, 320, 26, COLOR_BG);
  const Rect tabs[] = {{12, y + 3, 86, 20}, {117, y + 3, 86, 20},
                       {222, y + 3, 86, 20}};
  const UiPage pages[] = {UiPage::STATUS, UiPage::CONTROL, UiPage::DETAIL};
  const char* names[] = {"STAT", "CTRL", "INFO"};
  const char* test_names[] = {"MOTOR", "DIAG", "EXIT"};
  for (uint8_t i = 0; i < 3; ++i) {
    if (test_mode) {
      const bool active = (i == 0 && test_page == TestPage::MOTOR) ||
                          (i == 1 && test_page == TestPage::DIAG);
      const uint16_t fill =
          active ? COLOR_ORANGE : (i == 2 ? COLOR_RED : COLOR_PANEL_2);
      gfx().fillRoundRect(tabs[i].x, tabs[i].y, tabs[i].w, tabs[i].h, 10,
                          fill);
      gfx().setTextColor(active ? COLOR_BG : COLOR_TEXT, fill);
      gfx().setTextDatum(middle_center);
      gfx().drawString(test_names[i], tabs[i].x + tabs[i].w / 2,
                       tabs[i].y + tabs[i].h / 2);
      continue;
    }
    const bool active = current_page == pages[i];
    gfx().fillRoundRect(tabs[i].x, tabs[i].y, tabs[i].w, tabs[i].h, 10,
                             active ? COLOR_ACCENT : COLOR_PANEL_2);
    gfx().setTextColor(active ? COLOR_BG : COLOR_MUTED,
                            active ? COLOR_ACCENT : COLOR_PANEL_2);
    gfx().setTextDatum(middle_center);
    gfx().drawString(names[i], tabs[i].x + tabs[i].w / 2,
                          tabs[i].y + tabs[i].h / 2);
  }
  gfx().setTextDatum(top_left);
}

void drawNotice() {
  if (millis() > ui_notice_until_ms) return;
  const Rect rect{72, 188, 176, 18};
  gfx().fillRoundRect(rect.x, rect.y, rect.w, rect.h, 8, COLOR_PANEL_2);
  gfx().setTextColor(COLOR_TEXT, COLOR_PANEL_2);
  gfx().setTextDatum(middle_center);
  gfx().drawString(ui_notice, rect.x + rect.w / 2, rect.y + rect.h / 2);
  gfx().setTextDatum(top_left);
}

// Joystick jog without HOME: slow XY and no soft limit. Drawn over the
// notice area so it cannot be missed.
void drawUnhomedJogBanner(const MachineState& state) {
  if (!state.jog_unhomed) return;
  const Rect rect{8, 184, 304, 26};
  gfx().fillRoundRect(rect.x, rect.y, rect.w, rect.h, 8, COLOR_ORANGE);
  gfx().setTextColor(COLOR_BG, COLOR_ORANGE);
  gfx().setTextDatum(middle_center);
  gfx().drawString("NO HOME: SLOW XY, NO SOFT LIMIT", rect.x + rect.w / 2,
                   rect.y + rect.h / 2);
  gfx().setTextDatum(top_left);
}

// Speaker warning for the un-homed joystick jog: two rising tones when it
// starts, then a short beep every UNHOMED_JOG_BEEP_INTERVAL_MS while XY moves.
void updateUnhomedJogSound(const MachineState& state) {
  static bool was_unhomed = false;
  static uint32_t last_beep_ms = 0;
  static uint32_t second_tone_ms = 0;  // 0: none pending
  if (!UI_SOUND_ENABLED) return;
  const uint32_t now_ms = millis();
  if (second_tone_ms != 0 && static_cast<int32_t>(now_ms - second_tone_ms) >= 0) {
    second_tone_ms = 0;
    if (state.jog_unhomed) M5.Speaker.tone(UNHOMED_JOG_BEEP_HZ, 160);
  }
  if (state.jog_unhomed && !was_unhomed) {
    M5.Speaker.tone(1500, 120);
    second_tone_ms = now_ms + 150;
    last_beep_ms = now_ms + 150;
  } else if (state.jog_unhomed_moving && second_tone_ms == 0 &&
             now_ms - last_beep_ms >= UNHOMED_JOG_BEEP_INTERVAL_MS) {
    M5.Speaker.tone(UNHOMED_JOG_BEEP_HZ, UNHOMED_JOG_BEEP_MS);
    last_beep_ms = now_ms;
  }
  was_unhomed = state.jog_unhomed;
}

void drawMetric(const Rect& rect, const char* label, const char* value,
                uint16_t value_color = COLOR_TEXT) {
  drawRoundPanel(rect);
  if (rect.h < 40) {
    gfx().setTextSize(1);
    gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
    gfx().drawString(label, rect.x + 10, rect.y + 9);
    gfx().setTextColor(value_color, COLOR_PANEL);
    gfx().setTextDatum(middle_right);
    gfx().drawString(value, rect.x + rect.w - 10, rect.y + rect.h / 2);
    gfx().setTextDatum(top_left);
    return;
  }
  gfx().setTextSize(1);
  gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
  gfx().drawString(label, rect.x + 10, rect.y + 8);
  gfx().setTextSize(2);
  gfx().setTextColor(value_color, COLOR_PANEL);
  gfx().drawString(value, rect.x + 10, rect.y + 26);
}

void drawStatusPage(const StatusMessage& status) {
  const MachineState& state = status.machine;
  char value[32] = {};
  const Rect hero{10, 42, 300, 64};
  gfx().fillRoundRect(hero.x, hero.y, hero.w, hero.h, 10,
                           safetyColor(state));
  gfx().setTextColor(COLOR_BG, safetyColor(state));
  gfx().setTextDatum(top_left);
  gfx().setTextSize(1);
  gfx().drawString(SIMULATION_MODE ? "SIMULATION MODE" : "MACHINE STATE",
                        hero.x + 14, hero.y + 12);
  gfx().setTextSize(3);
  gfx().drawString(safetyText(state), hero.x + 14, hero.y + 30);

  snprintf(value, sizeof(value), "X %.1f", state.x_mm);
  drawMetric({10, 116, 94, 58}, "Position", value, COLOR_ACCENT);
  snprintf(value, sizeof(value), "Y %.1f", state.y_mm);
  drawMetric({113, 116, 94, 58}, "Position", value, COLOR_ACCENT);
  drawMetric({216, 116, 94, 58}, "Pen", state.pen_down ? "DOWN" : "UP",
             state.pen_down ? COLOR_YELLOW : COLOR_GREEN);

  drawMetric({10, 182, 145, 26}, "Home",
             state.homed ? "OK" : state.homing_active ? "RUN" : "WAIT",
             state.homed ? COLOR_GREEN : COLOR_YELLOW);
  drawMetric({165, 182, 145, 26}, "TMC",
             state.tmc_ready ? "READY" : "OFF",
             state.tmc_ready ? COLOR_GREEN : COLOR_MUTED);
}

bool canManualMove(const MachineState& state) {
  return state.homed && !state.alarmed && !state.homing_active &&
         !state.motion_active && !state.job_active;
}

void drawControlPage(const StatusMessage& status) {
  const MachineState& state = status.machine;
  const bool enabled = canManualMove(state);
  const bool home_enabled =
      !state.homing_active && !state.motion_active && !state.job_active;
  drawButton({10, 42, 145, 36}, "HOME", COLOR_BLUE, home_enabled, 2);
  drawButton({165, 42, 145, 36}, "CLEAR ALARM", COLOR_RED, state.alarmed, 1);

  drawButton({119, 86, 82, 34}, "UP", COLOR_PANEL_2, enabled, 1);
  drawButton({119, 162, 82, 34}, "DOWN", COLOR_PANEL_2, enabled, 1);
  drawButton({32, 124, 82, 34}, "LEFT", COLOR_PANEL_2, enabled, 1);
  drawButton({206, 124, 82, 34}, "RIGHT", COLOR_PANEL_2, enabled, 1);
  drawButton({119, 124, 82, 34}, "STOP", COLOR_RED, true, 1);

  drawButton({10, 178, 94, 30}, "PEN UP", COLOR_GREEN, enabled, 1);
  drawButton({216, 178, 94, 30}, "PEN DOWN", COLOR_YELLOW, enabled, 1);

  if (!enabled) {
    gfx().setTextColor(COLOR_MUTED, COLOR_BG);
    gfx().setTextDatum(middle_center);
    gfx().drawString(state.homed ? "Manual locked" : "Home required",
                          160, 105);
    gfx().setTextDatum(top_left);
  }
}

void drawDetailPage(const StatusMessage& status) {
  const MachineState& state = status.machine;
  char value[40] = {};
  drawRoundPanel({10, 42, 300, 36});
  gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
  gfx().setTextSize(1);
  gfx().drawString("Homing", 22, 51);
  gfx().setTextColor(COLOR_TEXT, COLOR_PANEL);
  snprintf(value, sizeof(value), "%s X%s Y%s", state.homing_state,
           state.x_homed ? "+" : "-", state.y_homed ? "+" : "-");
  gfx().drawString(value, 112, 51);

  snprintf(value, sizeof(value), "%.0f mm/min", state.feed_mm_min);
  drawMetric({10, 88, 145, 46}, "Feed", value, COLOR_ACCENT);
  snprintf(value, sizeof(value), "A %ld", static_cast<long>(state.a_steps));
  drawMetric({165, 88, 145, 46}, "Motor", value, COLOR_TEXT);
  snprintf(value, sizeof(value), "B %ld", static_cast<long>(state.b_steps));
  drawMetric({165, 142, 145, 46}, "Motor", value, COLOR_TEXT);

  drawMetric({10, 142, 145, 46}, "Limit",
             status.x_limit_active || status.y_limit_active ? "ACTIVE" : "OPEN",
             status.x_limit_active || status.y_limit_active ? COLOR_RED
                                                            : COLOR_GREEN);
  gfx().setTextColor(COLOR_MUTED, COLOR_BG);
  gfx().setTextDatum(middle_center);
  snprintf(value, sizeof(value), "X:%s raw:%s  Y:%s raw:%s",
           status.x_limit_active ? "ON" : "OFF",
           status.x_limit_raw_active ? "ON" : "OFF",
           status.y_limit_active ? "ON" : "OFF",
           status.y_limit_raw_active ? "ON" : "OFF");
  gfx().drawString(value, 160, 196);
  gfx().setTextColor(COLOR_ORANGE, COLOR_BG);
  gfx().drawString("Hold center button: TEST MODE", 160, 208);
  gfx().setTextDatum(top_left);
}

// テストモード中のコマンド実行可否。モーション/homing/ジョブ中は新規テストを出さない。
bool canRunTest(const MachineState& state) {
  return !state.motion_active && !state.homing_active && !state.job_active;
}

bool canRunMotorTest(const MachineState& state) {
  return canRunTest(state) && !state.alarmed;
}

void drawLimitIndicator(const Rect& rect, const char* label, uint8_t gpio,
                        bool active, bool raw_active) {
  const uint16_t color = active ? COLOR_RED : COLOR_GREEN;
  drawRoundPanel(rect);
  char text[32] = {};
  gfx().setTextSize(1);
  gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
  snprintf(text, sizeof(text), "%s GPIO%u", label, gpio);
  gfx().drawString(text, rect.x + 8, rect.y + 6);
  snprintf(text, sizeof(text), "raw %s", raw_active ? "ON" : "OFF");
  gfx().setTextDatum(top_right);
  gfx().drawString(text, rect.x + rect.w - 8, rect.y + 6);
  gfx().setTextDatum(top_left);
  gfx().setTextSize(2);
  gfx().setTextColor(color, COLOR_PANEL);
  gfx().drawString(active ? "ON" : "OFF", rect.x + 8, rect.y + 20);
  gfx().setTextSize(1);
}

void drawTestMotorPage(const StatusMessage& status) {
  const MachineState& state = status.machine;
  const bool enabled = canRunMotorTest(state);
  char text[48] = {};

  const bool any_limit = status.x_limit_active || status.y_limit_active;
  drawRoundPanel(kTestLimitStrip, any_limit ? COLOR_RED : COLOR_PANEL);
  snprintf(text, sizeof(text), "LIMIT X:%s  Y:%s   HOME:%s",
           status.x_limit_active ? "ON" : "OFF",
           status.y_limit_active ? "ON" : "OFF", state.homed ? "OK" : "--");
  drawTextInRect(text, kTestLimitStrip, COLOR_TEXT, 1);

  // A=X+Y, B=X-Y。単独モータ移動はXY対角へ動く。
  drawButton(kTestAPlus, "A+  (+X +Y)", COLOR_BLUE, enabled, 1);
  drawButton(kTestAMinus, "A-  (-X -Y)", COLOR_BLUE, enabled, 1);
  drawButton(kTestBPlus, "B+  (+X -Y)", COLOR_ACCENT, enabled, 1);
  drawButton(kTestBMinus, "B-  (-X +Y)", COLOR_ACCENT, enabled, 1);

  snprintf(text, sizeof(text), "STEP %ld",
           static_cast<long>(TEST_MODE_MOTOR_STEP_OPTIONS[test_step_index]));
  drawButton(kTestStep, text, COLOR_PANEL_2, true, 1);
  drawButton(kTestStop, "STOP", COLOR_RED, true, 1);

  gfx().setTextColor(COLOR_MUTED, COLOR_BG);
  gfx().setTextDatum(middle_center);
  if (state.alarmed) {
    snprintf(text, sizeof(text), "ALARM: clear on CTRL page");
  } else if (status.diagnostics.last_motor_test[0] == '\0') {
    snprintf(text, sizeof(text), "Motor test clears HOME");
  } else {
    snprintf(text, sizeof(text), "Last: %s",
             status.diagnostics.last_motor_test);
  }
  gfx().drawString(text, 160, 200);
  gfx().setTextDatum(top_left);
}

void drawTmcPanel(const Rect& rect, const char* name, uint8_t address,
                  const TmcDriverSummary& tmc) {
  drawRoundPanel(rect);
  char text[40] = {};
  gfx().setTextSize(1);
  gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
  snprintf(text, sizeof(text), "TMC %s  addr %u", name, address);
  gfx().drawString(text, rect.x + 8, rect.y + 6);

  if (!tmc.checked) {
    gfx().setTextColor(COLOR_MUTED, COLOR_PANEL);
    gfx().drawString("not checked", rect.x + 8, rect.y + 24);
    return;
  }
  if (tmc.connection != 0) {
    gfx().setTextColor(COLOR_RED, COLOR_PANEL);
    snprintf(text, sizeof(text), "UART NG (code %u)", tmc.connection);
    gfx().drawString(text, rect.x + 8, rect.y + 24);
    return;
  }
  gfx().setTextColor(tmc.version == 0x21 ? COLOR_GREEN : COLOR_YELLOW,
                     COLOR_PANEL);
  snprintf(text, sizeof(text), "UART OK  ver 0x%02X", tmc.version);
  gfx().drawString(text, rect.x + 8, rect.y + 24);

  const bool fatal = tmc.over_temp || tmc.short_detected || tmc.driver_error;
  const bool warn = tmc.over_temp_warn || tmc.undervoltage;
  if (!fatal && !warn) {
    gfx().setTextColor(COLOR_GREEN, COLOR_PANEL);
    gfx().drawString("DRV OK", rect.x + 8, rect.y + 40);
    return;
  }
  snprintf(text, sizeof(text), "%s%s%s%s%s", tmc.over_temp ? "OT " : "",
           tmc.over_temp_warn ? "OTPW " : "",
           tmc.short_detected ? "SHORT " : "",
           tmc.driver_error ? "DRV_ERR " : "",
           tmc.undervoltage ? "UV" : "");
  gfx().setTextColor(fatal ? COLOR_RED : COLOR_YELLOW, COLOR_PANEL);
  gfx().drawString(text, rect.x + 8, rect.y + 40);
}

void drawTestDiagPage(const StatusMessage& status) {
  const MachineState& state = status.machine;
  const bool enabled = canRunTest(state);
  drawLimitIndicator(kDiagLimitX, "LIMIT X", X_LIMIT_PIN,
                     status.x_limit_active, status.x_limit_raw_active);
  drawLimitIndicator(kDiagLimitY, "LIMIT Y", Y_LIMIT_PIN,
                     status.y_limit_active, status.y_limit_raw_active);

  drawButton(kDiagSelftest, "SELFTEST", COLOR_BLUE, enabled, 1);
  drawButton(kDiagTmcInit, "TMC INIT", COLOR_BLUE, enabled, 1);
  drawButton(kDiagTmcStatus, "TMC STATUS", COLOR_BLUE, enabled, 1);

  const int8_t selftest = status.diagnostics.selftest_result;
  char text[48] = {};
  snprintf(text, sizeof(text), "SELFTEST: %s     TMC profile: %s",
           selftest < 0 ? "--" : selftest > 0 ? "PASS" : "FAIL",
           state.tmc_ready ? "READY" : "OFF");
  drawRoundPanel(kDiagSummary);
  drawTextInRect(text, kDiagSummary,
                 selftest == 0 ? COLOR_RED : COLOR_TEXT, 1);

  drawTmcPanel(kDiagTmcA, "A", TMC_A_UART_ADDRESS, status.diagnostics.tmc_a);
  drawTmcPanel(kDiagTmcB, "B", TMC_B_UART_ADDRESS, status.diagnostics.tmc_b);
}

void drawUi(const StatusMessage& status) {
  gfx().startWrite();
  gfx().fillScreen(COLOR_BG);
  drawHeader(status);
  if (test_mode) {
    if (test_page == TestPage::MOTOR) {
      drawTestMotorPage(status);
    } else {
      drawTestDiagPage(status);
    }
  } else {
    switch (current_page) {
      case UiPage::STATUS:
        drawStatusPage(status);
        break;
      case UiPage::CONTROL:
        drawControlPage(status);
        break;
      case UiPage::DETAIL:
        drawDetailPage(status);
        break;
    }
  }
  drawNotice();
  drawUnhomedJogBanner(status.machine);
  drawFooter();
  gfx().endWrite();
  if (ui_gfx == &ui_canvas) {
    ui_canvas.pushSprite(&M5.Display, 0, 0);
  }
}

void nextPage(int8_t delta) {
  int8_t page = static_cast<int8_t>(current_page) + delta;
  if (page < 0) page = 2;
  if (page > 2) page = 0;
  current_page = static_cast<UiPage>(page);
  force_redraw = true;
}

bool queueCommand(const CommandMessage& command) {
  if (xQueueSend(command_queue, &command, pdMS_TO_TICKS(10)) == pdTRUE) {
    return true;
  }
  setNotice("Queue full");
  return false;
}

void queueSimpleCommand(CommandType type, const char* name) {
  CommandMessage command{};
  command.type = type;
  snprintf(command.name, sizeof(command.name), "%s", name);
  if (queueCommand(command)) setNotice(name);
}

float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

void queueJog(const StatusMessage& status, float dx_mm, float dy_mm) {
  const MachineState& state = status.machine;
  if (!canManualMove(state)) {
    setNotice(state.homed ? "Manual locked" : "Home first");
    return;
  }
  if (!ui_have_jog_base ||
      fabsf(state.x_mm - ui_jog_base_x_mm) > UI_JOG_STEP_MM * 2.0f ||
      fabsf(state.y_mm - ui_jog_base_y_mm) > UI_JOG_STEP_MM * 2.0f) {
    ui_jog_base_x_mm = state.x_mm;
    ui_jog_base_y_mm = state.y_mm;
    ui_have_jog_base = true;
  }
  CommandMessage command{};
  command.type = CommandType::XY;
  snprintf(command.name, sizeof(command.name), "UI_JOG");
  command.x_mm = clampFloat(ui_jog_base_x_mm + dx_mm, X_MIN_MM, X_MAX_MM);
  command.y_mm = clampFloat(ui_jog_base_y_mm + dy_mm, Y_MIN_MM, Y_MAX_MM);
  command.feed_mm_min = UI_JOG_FEED_MM_MIN;
  if (fabsf(command.x_mm - ui_jog_base_x_mm) < 0.001f &&
      fabsf(command.y_mm - ui_jog_base_y_mm) < 0.001f) {
    setNotice("Soft limit");
    return;
  }
  if (queueCommand(command)) {
    ui_jog_base_x_mm = command.x_mm;
    ui_jog_base_y_mm = command.y_mm;
    setNotice("Jog queued");
  }
}

// Serial ABORTと同じく、実行中motionを止めるためフラグを先に立ててからキューへ入れる。
void queueAbort() {
  requestMotionAbort();
  queueSimpleCommand(CommandType::ABORT, "ABORT");
}

void enterTestMode(const MachineState& state) {
  if (!canRunTest(state)) {
    setNotice("Busy: cannot enter TEST");
    return;
  }
  test_mode = true;
  test_page = TestPage::MOTOR;
  force_redraw = true;
  logMessage("UI TEST MODE enter");
  setNotice("TEST MODE");
}

void exitTestMode(const char* reason) {
  test_mode = false;
  force_redraw = true;
  logMessage("UI TEST MODE exit reason=%s", reason);
  setNotice("TEST MODE exit");
}

void queueMotorTest(const MachineState& state, bool motor_a, int8_t sign) {
  if (!canRunMotorTest(state)) {
    setNotice(state.alarmed ? "Alarm active" : "Busy");
    return;
  }
  CommandMessage command{};
  command.type = motor_a ? CommandType::TEST_A : CommandType::TEST_B;
  snprintf(command.name, sizeof(command.name), "UI_TEST_%c",
           motor_a ? 'A' : 'B');
  command.steps = sign * TEST_MODE_MOTOR_STEP_OPTIONS[test_step_index];
  command.feed_mm_min = TEST_MODE_MOTOR_FEED_MM_MIN;
  if (queueCommand(command)) {
    char text[32] = {};
    snprintf(text, sizeof(text), "TEST_%c %+ld", motor_a ? 'A' : 'B',
             static_cast<long>(command.steps));
    setNotice(text);
  }
}

void handleTestMotorTouch(const StatusMessage& status, int16_t x, int16_t y) {
  const MachineState& state = status.machine;
  if (contains(kTestAPlus, x, y)) {
    queueMotorTest(state, true, 1);
  } else if (contains(kTestAMinus, x, y)) {
    queueMotorTest(state, true, -1);
  } else if (contains(kTestBPlus, x, y)) {
    queueMotorTest(state, false, 1);
  } else if (contains(kTestBMinus, x, y)) {
    queueMotorTest(state, false, -1);
  } else if (contains(kTestStep, x, y)) {
    test_step_index = (test_step_index + 1) % kTestStepOptionCount;
    force_redraw = true;
  } else if (contains(kTestStop, x, y)) {
    queueAbort();
  }
}

void handleTestDiagTouch(const StatusMessage& status, int16_t x, int16_t y) {
  const MachineState& state = status.machine;
  const bool selftest = contains(kDiagSelftest, x, y);
  const bool tmc_init = contains(kDiagTmcInit, x, y);
  const bool tmc_status = contains(kDiagTmcStatus, x, y);
  if (!selftest && !tmc_init && !tmc_status) return;
  if (!canRunTest(state)) {
    setNotice("Busy");
    return;
  }
  if (selftest) {
    queueSimpleCommand(CommandType::SELFTEST, "SELFTEST");
  } else if (tmc_init) {
    queueSimpleCommand(CommandType::TMC_INIT, "TMC_INIT");
  } else {
    queueSimpleCommand(CommandType::TMC_STATUS, "TMC_STATUS");
  }
}

void handleTestModeTouch(const StatusMessage& status) {
  if (M5.BtnA.wasClicked()) {
    test_page = TestPage::MOTOR;
    force_redraw = true;
  }
  if (M5.BtnC.wasClicked()) {
    test_page = TestPage::DIAG;
    force_redraw = true;
  }

  if (M5.Touch.getCount() == 0) return;
  const auto touch = M5.Touch.getDetail();
  if (touch.wasFlicked() && abs(touch.distanceX()) > 55 &&
      abs(touch.distanceX()) > abs(touch.distanceY())) {
    test_page = touch.distanceX() < 0 ? TestPage::DIAG : TestPage::MOTOR;
    force_redraw = true;
    return;
  }
  if (!touch.wasClicked()) return;

  const int16_t x = touch.x;
  const int16_t y = touch.y;
  if (y >= 214) {
    if (x < 107) {
      test_page = TestPage::MOTOR;
    } else if (x < 214) {
      test_page = TestPage::DIAG;
    } else {
      exitTestMode("exit tab");
      return;
    }
    force_redraw = true;
    return;
  }
  if (test_page == TestPage::MOTOR) {
    handleTestMotorTouch(status, x, y);
  } else {
    handleTestDiagTouch(status, x, y);
  }
}

void handleControlTouch(const StatusMessage& status, int16_t x, int16_t y) {
  const MachineState& state = status.machine;
  if (contains({10, 42, 145, 36}, x, y)) {
    if (state.homing_active) {
      setNotice("Homing now");
    } else if (state.motion_active || state.job_active) {
      setNotice("Busy");
    } else {
      queueSimpleCommand(CommandType::HOME, "HOME");
    }
  } else if (contains({165, 42, 145, 36}, x, y)) {
    if (state.alarmed) {
      queueSimpleCommand(CommandType::ALARM_CLEAR, "ALARM_CLEAR");
    } else {
      setNotice("No alarm");
    }
  } else if (contains({119, 86, 82, 34}, x, y)) {
    queueJog(status, 0.0f, UI_JOG_STEP_MM);
  } else if (contains({119, 162, 82, 34}, x, y)) {
    queueJog(status, 0.0f, -UI_JOG_STEP_MM);
  } else if (contains({32, 124, 82, 34}, x, y)) {
    queueJog(status, -UI_JOG_STEP_MM, 0.0f);
  } else if (contains({206, 124, 82, 34}, x, y)) {
    queueJog(status, UI_JOG_STEP_MM, 0.0f);
  } else if (contains({119, 124, 82, 34}, x, y)) {
    queueAbort();
  } else if (contains({10, 178, 94, 30}, x, y)) {
    if (canManualMove(state)) {
      queueSimpleCommand(CommandType::PEN_UP, "PENUP");
    } else {
      setNotice(state.homed ? "Manual locked" : "Home first");
    }
  } else if (contains({216, 178, 94, 30}, x, y)) {
    if (canManualMove(state)) {
      queueSimpleCommand(CommandType::PEN_DOWN, "PENDOWN");
    } else {
      setNotice(state.homed ? "Manual locked" : "Home first");
    }
  }
}

void handleTouch(const StatusMessage& status) {
  const MachineState& state = status.machine;
  if (M5.BtnB.wasHold()) {
    if (test_mode) {
      exitTestMode("hold");
    } else {
      enterTestMode(state);
    }
    return;
  }
  if (test_mode && state.job_active) {
    exitTestMode("job active");
    return;
  }
  if (test_mode) {
    handleTestModeTouch(status);
    return;
  }

  if (M5.BtnA.wasClicked()) nextPage(-1);
  if (M5.BtnC.wasClicked()) nextPage(1);

  if (M5.Touch.getCount() == 0) return;
  const auto touch = M5.Touch.getDetail();
  if (touch.wasFlicked() && abs(touch.distanceX()) > 55 &&
      abs(touch.distanceX()) > abs(touch.distanceY())) {
    nextPage(touch.distanceX() < 0 ? 1 : -1);
    return;
  }
  if (!touch.wasClicked()) return;

  const int16_t x = touch.x;
  const int16_t y = touch.y;
  if (y >= 214) {
    if (x < 107) current_page = UiPage::STATUS;
    else if (x < 214) current_page = UiPage::CONTROL;
    else current_page = UiPage::DETAIL;
    force_redraw = true;
    return;
  }
  if (x < 24 && y > 38 && y < 210) {
    nextPage(-1);
    return;
  }
  if (x > 296 && y > 38 && y < 210) {
    nextPage(1);
    return;
  }
  if (current_page == UiPage::CONTROL) {
    handleControlTouch(status, x, y);
  }
}
}

void uiTask(void*) {
  bool lcd_ready = false;
  if (M5_UI_ENABLED) {
    auto m5_config = M5.config();
    m5_config.internal_spk = UI_SOUND_ENABLED;
    m5_config.internal_mic = false;
    M5.begin(m5_config);
    lcd_ready = M5.Display.begin();
    M5.Display.setRotation(1);
    M5.Display.setTextFont(1);
    M5.Display.setTextSize(1);
    M5.Display.setTextDatum(top_left);
    M5.BtnB.setHoldThresh(TEST_MODE_HOLD_MS);
    if (UI_SOUND_ENABLED) M5.Speaker.setVolume(UI_SOUND_VOLUME);
    if (lcd_ready) {
      ui_canvas.setColorDepth(16);
      if (ui_canvas.createSprite(M5.Display.width(), M5.Display.height())) {
        ui_canvas.setTextFont(1);
        ui_canvas.setTextSize(1);
        ui_canvas.setTextDatum(top_left);
        ui_gfx = &ui_canvas;
        logMessage("UI canvas enabled %dx%d",
                   static_cast<int>(M5.Display.width()),
                   static_cast<int>(M5.Display.height()));
      } else {
        logMessage("WARN: UI canvas allocation failed; direct LCD drawing");
      }
    }
  } else {
    logMessage("ERROR: M5 UI disabled by pin configuration");
  }
  if (NEOPIXEL_ENABLED) {
    neopixel_controller.begin();
    led_pattern_engine.begin(neopixel_controller);
    led_pattern_engine.setStatus(LedStatus::IDLE);
  } else {
    logMessage("NeoPixel disabled: GPIO33 is STS3020 UART RX");
  }

  StatusMessage status;
  StatusMessage displayed_status;
  bool have_status = false;
  bool have_displayed_status = false;
  for (;;) {
    if (M5_UI_ENABLED) M5.update();
    LedCommand led_command;
    while (xQueueReceive(led_command_queue, &led_command, 0) == pdTRUE) {
      if (NEOPIXEL_ENABLED) led_pattern_engine.applyCommand(led_command);
    }
    if (NEOPIXEL_ENABLED) led_pattern_engine.tick(millis());

    if (xQueueReceive(status_queue, &status, pdMS_TO_TICKS(20)) == pdTRUE) {
      have_status = true;
    }

    if (M5_UI_ENABLED && have_status) updateUnhomedJogSound(status.machine);
    if (lcd_ready && have_status) {
      handleTouch(status);
      const bool changed =
          !have_displayed_status ||
          memcmp(&status, &displayed_status, sizeof(status)) != 0;
      const bool notice_expired =
          ui_notice_until_ms != 0 && millis() > ui_notice_until_ms;
      const bool clock_tick = millis() - last_clock_redraw_ms > 1000;
      if (changed || force_redraw || notice_expired || clock_tick) {
        drawUi(status);
        displayed_status = status;
        have_displayed_status = true;
        force_redraw = false;
        if (notice_expired) ui_notice_until_ms = 0;
        last_clock_redraw_ms = millis();
      }
    }
  }
}
