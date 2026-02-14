/**
 * @file    menu.c
 * @brief   Menu/app state machine — LCD rendering, navigation, settings
 *
 * Screen flow:
 *   Main Menu → Run CCD / Old Measurements / Settings → Laser PWM
 *
 * Settings are persisted to STM32H7 Backup SRAM (battery-backed).
 */
#include "menu.h"
#include <stdio.h>
#include <string.h>

// External ADC3 handle (defined in main.c, used for pot reading)
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim15;

// ============================================================
// Backup SRAM persistence (battery-backed, no Flash wear)
// ============================================================
// STM32H743 Backup SRAM: 0x38800000, 4KB
#define BKPSRAM_BASE 0x38800000UL

void Settings_Load(DeviceSettings *s) {
  // Enable backup SRAM clock
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  // Read from backup SRAM
  memcpy(s, (void *)BKPSRAM_BASE, sizeof(DeviceSettings));

  // Validate
  if (s->magic != SETTINGS_MAGIC) {
    // First boot or corrupted — init defaults
    s->magic = SETTINGS_MAGIC;
    s->laser1_pwm = 0; // Lasers off by default
    s->laser2_pwm = 0;
    Settings_Save(s);
  }
}

void Settings_Save(const DeviceSettings *s) {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  memcpy((void *)BKPSRAM_BASE, s, sizeof(DeviceSettings));
}

// ============================================================
// Flash persistence for measurements
// ============================================================
// Store measurements after settings in backup SRAM (plenty of room in 4KB)
#define MEAS_SRAM_BASE (BKPSRAM_BASE + 64) // Offset past settings
#define MEAS_HEADER_MAGIC 0xBEEF

typedef struct {
  uint16_t magic;
  uint8_t count;
  uint16_t next_index;
  MeasurementRecord records[MAX_MEASUREMENTS];
} MeasStorageHeader;

static void Measurements_Load(MenuContext *ctx) {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  MeasStorageHeader hdr;
  memcpy(&hdr, (void *)MEAS_SRAM_BASE, sizeof(MeasStorageHeader));

  if (hdr.magic == MEAS_HEADER_MAGIC && hdr.count <= MAX_MEASUREMENTS) {
    ctx->meas_count = hdr.count;
    ctx->meas_next_index = hdr.next_index;
    memcpy(ctx->measurements, hdr.records, sizeof(ctx->measurements));
  } else {
    ctx->meas_count = 0;
    ctx->meas_next_index = 1;
    memset(ctx->measurements, 0, sizeof(ctx->measurements));
  }
}

static void Measurements_Save(MenuContext *ctx) {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  MeasStorageHeader hdr;
  hdr.magic = MEAS_HEADER_MAGIC;
  hdr.count = ctx->meas_count;
  hdr.next_index = ctx->meas_next_index;
  memcpy(hdr.records, ctx->measurements, sizeof(ctx->measurements));
  memcpy((void *)MEAS_SRAM_BASE, &hdr, sizeof(MeasStorageHeader));
}

// ============================================================
// Pot reading helper (ADC3 scan mode, 2 channels)
// ============================================================
uint8_t Menu_ReadPotPercent(uint8_t pot_id) {
  HAL_ADC_Start(&hadc3);

  // Read both channels in scan order
  HAL_ADC_PollForConversion(&hadc3, 10);
  uint32_t ch1 = HAL_ADC_GetValue(&hadc3);

  HAL_ADC_PollForConversion(&hadc3, 10);
  uint32_t ch2 = HAL_ADC_GetValue(&hadc3);

  HAL_ADC_Stop(&hadc3);

  uint32_t raw = (pot_id == 0) ? ch1 : ch2;
  return (uint8_t)((raw * 100) / 4095);
}

// Map percentage (0-100) to PWM duty (0-2399)
static uint16_t PctToPwm(uint8_t pct) {
  return (uint16_t)(((uint32_t)pct * 2399) / 100);
}

// ============================================================
// LCD Rendering Helpers
// ============================================================
static const char *menu_items[] = {"Run CCD", "Old Measure.", "Settings"};
#define MENU_ITEM_COUNT 3

static void LCD_ClearLine(LCD_HandleTypeDef *lcd, uint8_t row) {
  LCD_SetCursor(lcd, 0, row);
  LCD_Print(lcd, "                "); // 16 spaces
}

// Draw a progress bar: [████░░░░░░] (10 chars wide)
static void LCD_DrawProgressBar(LCD_HandleTypeDef *lcd, uint8_t col,
                                uint8_t row, uint8_t pct) {
  LCD_SetCursor(lcd, col, row);
  uint8_t filled = (pct + 5) / 10; // 0-10 blocks
  char bar[12];
  bar[0] = '[';
  for (int i = 0; i < 10; i++) {
    bar[i + 1] = (i < filled) ? '\xFF' : '\xDB'; // Full block or light shade
  }
  bar[11] = '\0';
  LCD_Print(lcd, bar);
  // Print percentage after bar
  char buf[5];
  snprintf(buf, sizeof(buf), "]%2d", pct);
  LCD_Print(lcd, buf);
}

// ============================================================
// Screen Rendering
// ============================================================
static void Render_MainMenu(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  // Show 2 items at a time with cursor
  uint8_t top = (ctx->menu_sel > 0) ? ctx->menu_sel - 1 : 0;
  if (ctx->menu_sel == 0)
    top = 0;

  for (uint8_t row = 0; row < 2; row++) {
    uint8_t idx = top + row;
    LCD_ClearLine(lcd, row);
    LCD_SetCursor(lcd, 0, row);
    if (idx < MENU_ITEM_COUNT) {
      LCD_PrintChar(lcd, (idx == ctx->menu_sel) ? '>' : ' ');
      LCD_Print(lcd, menu_items[idx]);
    }
  }
}

static void Render_RunCCD(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  LCD_ClearLine(lcd, 0);
  LCD_SetCursor(lcd, 0, 0);
  LCD_Print(lcd, "CCD Running...");

  LCD_ClearLine(lcd, 1);
  LCD_SetCursor(lcd, 0, 1);
  char buf[17];
  snprintf(buf, sizeof(buf), "Frame: %lu", (unsigned long)ctx->ccd_frame_count);
  LCD_Print(lcd, buf);
}

static void Render_OldMeasurements(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  if (ctx->meas_count == 0) {
    LCD_ClearLine(lcd, 0);
    LCD_SetCursor(lcd, 0, 0);
    LCD_Print(lcd, "No measurements");
    LCD_ClearLine(lcd, 1);
    LCD_SetCursor(lcd, 0, 1);
    LCD_Print(lcd, "BACK to return");
    return;
  }

  // Show selected measurement
  uint8_t idx = ctx->meas_sel;
  if (idx >= ctx->meas_count)
    idx = ctx->meas_count - 1;
  MeasurementRecord *m = &ctx->measurements[idx];

  char buf[17];
  LCD_ClearLine(lcd, 0);
  LCD_SetCursor(lcd, 0, 0);
  snprintf(buf, sizeof(buf), "#%03d Chl-a:%.2f", m->index, (double)m->chl_a);
  LCD_Print(lcd, buf);

  LCD_ClearLine(lcd, 1);
  LCD_SetCursor(lcd, 0, 1);
  snprintf(buf, sizeof(buf), "     Chl-b:%.2f", (double)m->chl_b);
  LCD_Print(lcd, buf);
}

static void Render_Settings(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  LCD_ClearLine(lcd, 0);
  LCD_SetCursor(lcd, 0, 0);
  LCD_Print(lcd, ">Laser PWM");

  LCD_ClearLine(lcd, 1);
  LCD_SetCursor(lcd, 0, 1);
  char buf[17];
  uint8_t pct1 = (uint8_t)((ctx->settings.laser1_pwm * 100UL) / 2399);
  uint8_t pct2 = (uint8_t)((ctx->settings.laser2_pwm * 100UL) / 2399);
  snprintf(buf, sizeof(buf), "404:%3d%% 450:%3d%%", pct1, pct2);
  LCD_Print(lcd, buf);
}

static void Render_LaserSettings(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  // Line 1: Laser name + pot reading as progress bar
  uint8_t pot_pct = Menu_ReadPotPercent(ctx->laser_sel);

  LCD_ClearLine(lcd, 0);
  LCD_SetCursor(lcd, 0, 0);
  char label[6];
  snprintf(label, sizeof(label), "%s",
           (ctx->laser_sel == 0) ? "404nm" : "450nm");
  LCD_Print(lcd, label);

  // Progress bar at col 5 (takes 14 chars: [██████████]99)
  LCD_DrawProgressBar(lcd, 5, 0, pot_pct);

  // Line 2: Instructions
  LCD_ClearLine(lcd, 1);
  LCD_SetCursor(lcd, 0, 1);
  LCD_Print(lcd, "OK=Save  LR=Sel");
}

// ============================================================
// Menu Logic
// ============================================================
void Menu_Init(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  memset(ctx, 0, sizeof(MenuContext));
  ctx->screen = SCREEN_MAIN_MENU;
  ctx->need_redraw = 1;

  // Load saved settings from backup SRAM
  Settings_Load(&ctx->settings);

  // Load saved measurements
  Measurements_Load(ctx);

  // Apply saved laser PWM values (lasers start at saved power)
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
}

uint8_t Menu_Update(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  ButtonEvent ev_up = Buttons_GetEvent(BTN_ID_UP);
  ButtonEvent ev_down = Buttons_GetEvent(BTN_ID_DOWN);
  ButtonEvent ev_ok = Buttons_GetEvent(BTN_ID_OK);
  ButtonEvent ev_left = Buttons_GetEvent(BTN_ID_LEFT);
  ButtonEvent ev_right = Buttons_GetEvent(BTN_ID_RIGHT);

  uint8_t pressed_up = (ev_up == BTN_EVENT_PRESSED || ev_up == BTN_EVENT_HELD);
  uint8_t pressed_down =
      (ev_down == BTN_EVENT_PRESSED || ev_down == BTN_EVENT_HELD);
  uint8_t pressed_ok = (ev_ok == BTN_EVENT_PRESSED);
  uint8_t pressed_left = (ev_left == BTN_EVENT_PRESSED);
  uint8_t pressed_right = (ev_right == BTN_EVENT_PRESSED);

  switch (ctx->screen) {

  // ---- MAIN MENU ----
  case SCREEN_MAIN_MENU:
    if (pressed_up && ctx->menu_sel > 0) {
      ctx->menu_sel--;
      ctx->need_redraw = 1;
    }
    if (pressed_down && ctx->menu_sel < MENU_ITEM_COUNT - 1) {
      ctx->menu_sel++;
      ctx->need_redraw = 1;
    }
    if (pressed_ok) {
      switch (ctx->menu_sel) {
      case 0:
        ctx->screen = SCREEN_RUN_CCD;
        ctx->ccd_running = 1;
        ctx->ccd_frame_count = 0;
        break;
      case 1:
        ctx->screen = SCREEN_OLD_MEASUREMENTS;
        ctx->meas_sel = 0;
        break;
      case 2:
        ctx->screen = SCREEN_SETTINGS;
        break;
      }
      ctx->need_redraw = 1;
    }
    if (ctx->need_redraw) {
      Render_MainMenu(ctx, lcd);
      ctx->need_redraw = 0;
    }
    return 0; // No CCD running

  // ---- RUN CCD ----
  case SCREEN_RUN_CCD:
    if (pressed_left) {
      // BACK — stop CCD, return to menu
      ctx->ccd_running = 0;

      // Run dummy ML prediction and save
      Menu_SaveMeasurement(ctx, 1.56f, 0.83f);

      ctx->screen = SCREEN_MAIN_MENU;
      ctx->need_redraw = 1;
      return 0;
    }
    if (ctx->need_redraw) {
      Render_RunCCD(ctx, lcd);
      ctx->need_redraw = 0;
    }
    return 1; // CCD should run

  // ---- OLD MEASUREMENTS ----
  case SCREEN_OLD_MEASUREMENTS:
    if (pressed_left) {
      ctx->screen = SCREEN_MAIN_MENU;
      ctx->need_redraw = 1;
    }
    if (pressed_up && ctx->meas_sel > 0) {
      ctx->meas_sel--;
      ctx->need_redraw = 1;
    }
    if (pressed_down && ctx->meas_count > 0 &&
        ctx->meas_sel < ctx->meas_count - 1) {
      ctx->meas_sel++;
      ctx->need_redraw = 1;
    }
    if (ctx->need_redraw) {
      Render_OldMeasurements(ctx, lcd);
      ctx->need_redraw = 0;
    }
    return 0;

  // ---- SETTINGS ----
  case SCREEN_SETTINGS:
    if (pressed_left) {
      ctx->screen = SCREEN_MAIN_MENU;
      ctx->need_redraw = 1;
    }
    if (pressed_ok) {
      ctx->screen = SCREEN_SETTINGS_LASER;
      ctx->laser_sel = 0; // Start with 404nm
      ctx->need_redraw = 1;
    }
    if (ctx->need_redraw) {
      Render_Settings(ctx, lcd);
      ctx->need_redraw = 0;
    }
    return 0;

  // ---- LASER PWM SETTINGS ----
  case SCREEN_SETTINGS_LASER:
    // LEFT/RIGHT to switch laser
    if (pressed_left && ctx->laser_sel > 0) {
      ctx->laser_sel = 0;
      ctx->need_redraw = 1;
    }
    if (pressed_right && ctx->laser_sel < 1) {
      ctx->laser_sel = 1;
      ctx->need_redraw = 1;
    }
    // OK = save current pot reading as the PWM value
    if (pressed_ok) {
      uint8_t pot_pct = Menu_ReadPotPercent(ctx->laser_sel);
      uint16_t pwm_val = PctToPwm(pot_pct);

      if (ctx->laser_sel == 0) {
        ctx->settings.laser1_pwm = pwm_val;
      } else {
        ctx->settings.laser2_pwm = pwm_val;
      }

      // Apply immediately
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);

      // Persist to backup SRAM
      Settings_Save(&ctx->settings);

      // Briefly show "Saved!" then return to settings
      LCD_ClearLine(lcd, 1);
      LCD_SetCursor(lcd, 0, 1);
      LCD_Print(lcd, "  ** Saved! **  ");
      HAL_Delay(500);

      ctx->screen = SCREEN_SETTINGS;
      ctx->need_redraw = 1;
    }
    // BACK = discard and go back
    if (ev_left == BTN_EVENT_PRESSED && !pressed_ok) {
      // Already handled by LEFT above for laser_sel=0
      // If laser_sel is already 0, LEFT goes back to settings
      if (ctx->laser_sel == 0) {
        ctx->screen = SCREEN_SETTINGS;
        ctx->need_redraw = 1;
      }
    }

    // Always redraw laser settings (pot is live)
    Render_LaserSettings(ctx, lcd);
    return 0;
  }

  return 0;
}

void Menu_CCD_FrameUpdate(MenuContext *ctx, LCD_HandleTypeDef *lcd,
                          uint32_t frame_num) {
  ctx->ccd_frame_count = frame_num;
  // Only update frame counter line (avoid full redraw flicker)
  LCD_SetCursor(lcd, 7, 1); // After "Frame: "
  char buf[9];
  snprintf(buf, sizeof(buf), "%lu", (unsigned long)frame_num);
  LCD_Print(lcd, buf);
  LCD_Print(lcd, "    "); // Clear trailing chars
}

void Menu_SaveMeasurement(MenuContext *ctx, float chl_a, float chl_b) {
  // Circular buffer: overwrite oldest if full
  uint8_t slot = ctx->meas_count;
  if (slot >= MAX_MEASUREMENTS) {
    // Shift all measurements down by one (remove oldest)
    for (int i = 0; i < MAX_MEASUREMENTS - 1; i++) {
      ctx->measurements[i] = ctx->measurements[i + 1];
    }
    slot = MAX_MEASUREMENTS - 1;
  } else {
    ctx->meas_count++;
  }

  ctx->measurements[slot].valid = 1;
  ctx->measurements[slot].index = ctx->meas_next_index++;
  ctx->measurements[slot].chl_a = chl_a;
  ctx->measurements[slot].chl_b = chl_b;

  // Persist
  Measurements_Save(ctx);
}
