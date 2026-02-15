/**
 * @file    menu.c
 * @brief   Menu/app state machine — LCD rendering, navigation, settings
 *
 * Screen flow:
 *   Main Menu → Run CCD / Auto Measure / Old Measurements / Settings
 *              Settings → Laser PWM
 *
 * Settings are persisted to STM32H7 Backup SRAM (battery-backed).
 */
#include "menu.h"
#include "buttons.h"
#include "sd_storage.h"
#include "servo.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim15;
extern volatile uint32_t integration_time_ms;

// ============================================================
// Backup SRAM persistence (battery-backed, no Flash wear)
// ============================================================
// STM32H743 Backup SRAM: 0x38800000, 4KB
#define BKPSRAM_BASE 0x38800000UL

void Settings_Load(DeviceSettings *s) {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  memcpy(s, (void *)BKPSRAM_BASE, sizeof(DeviceSettings));

  if (s->magic != SETTINGS_MAGIC) {
    s->magic = SETTINGS_MAGIC;
    s->laser1_pwm = 2399;      // 100% default brightness
    s->laser2_pwm = 2399;      // 100% default brightness
    s->integration_time = 300; // Default integration time
    Settings_Save(s);
  }
  // Sync global
  integration_time_ms = s->integration_time;
}

void Settings_Save(const DeviceSettings *s) {
  __HAL_RCC_BKPRAM_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  memcpy((void *)BKPSRAM_BASE, s, sizeof(DeviceSettings));
}

// ============================================================
// Measurement persistence (backup SRAM)
// ============================================================
#define MEAS_SRAM_BASE (BKPSRAM_BASE + 64)
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

  HAL_ADC_PollForConversion(&hadc3, 10);
  uint32_t ch1 = HAL_ADC_GetValue(&hadc3);
  HAL_ADC_PollForConversion(&hadc3, 10);
  uint32_t ch2 = HAL_ADC_GetValue(&hadc3);
  HAL_ADC_Stop(&hadc3);

  uint32_t raw = (pot_id == 0) ? ch1 : ch2;
  return (uint8_t)((raw * 100) / 4095);
}

static uint16_t PctToPwm(uint8_t pct) {
  return (uint16_t)(((uint32_t)pct * 2399) / 100);
}

// ============================================================
// LCD Rendering Helpers
// ============================================================
static const char *menu_items[] = {"Run CCD", "Auto Measure", "Old Measure",
                                   "Settings"};

static void LCD_PrintLine(LCD_HandleTypeDef *lcd, uint8_t row,
                          const char *text) {
  LCD_SetCursor(lcd, 0, row);
  char padded[17];
  int i;
  for (i = 0; i < 16 && text[i] != '\0'; i++)
    padded[i] = text[i];
  for (; i < 16; i++)
    padded[i] = ' ';
  padded[16] = '\0';
  LCD_Print(lcd, padded);
}

static void LCD_DrawProgressBar(LCD_HandleTypeDef *lcd, uint8_t col,
                                uint8_t row, uint8_t pct) {
  LCD_SetCursor(lcd, col, row);
  uint8_t filled = (pct + 5) / 10;
  char bar[15];
  bar[0] = '[';
  for (int i = 0; i < 10; i++)
    bar[i + 1] = (i < filled) ? '\xFF' : ' ';
  snprintf(&bar[11], 4, "]%2d", pct);
  LCD_Print(lcd, bar);
}

// ============================================================
// Screen Rendering
// ============================================================
static void Render_SplashScreen(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  LCD_PrintLine(lcd, 0, "CHL Conc. METER");
  LCD_PrintLine(lcd, 1, "    V2.0.0");
}

static void Render_MainMenu(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  uint8_t top = (ctx->menu_sel > 0) ? ctx->menu_sel - 1 : 0;
  if (ctx->menu_sel == 0)
    top = 0;

  for (uint8_t row = 0; row < 2; row++) {
    uint8_t idx = top + row;
    char line[17];
    if (idx < MENU_ITEM_COUNT) {
      snprintf(line, sizeof(line), "%c%s", (idx == ctx->menu_sel) ? '>' : ' ',
               menu_items[idx]);
    } else {
      line[0] = '\0';
    }
    LCD_PrintLine(lcd, row, line);
  }
}

#define DBLCLICK_MS 400 // Max gap between two RIGHT presses for double-click

static void Render_RunCCD(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  // Clear LCD first to flush any corrupted I2C state from USB interrupts
  LCD_Clear(lcd);
  if (ctx->ccd_paused) {
    LCD_PrintLine(lcd, 0, "CCD PAUSED");
  } else {
    char line0[17];
    snprintf(line0, sizeof(line0), "CCD  %s",
             (ctx->active_laser == 0) ? "405nm" : "450nm");
    LCD_PrintLine(lcd, 0, line0);
  }
  char buf[17];
  snprintf(buf, sizeof(buf), "Frame: %lu", (unsigned long)ctx->ccd_frame_count);
  LCD_PrintLine(lcd, 1, buf);
}

static void Render_AutoMeasure(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  char line1[17], line2[17];

  switch (ctx->auto_state) {
  case AUTO_IDLE:
    LCD_PrintLine(lcd, 0, "Auto Measure");
    LCD_PrintLine(lcd, 1, "OK=Start LEFT=Bk");
    return;
  case AUTO_MOVE_LASER1:
  case AUTO_SETTLE_LASER1:
    snprintf(line1, sizeof(line1), "Moving: 405nm");
    snprintf(line2, sizeof(line2), "Please wait...");
    break;
  case AUTO_CAPTURE_LASER1:
    snprintf(line1, sizeof(line1), "405nm Capture");
    snprintf(line2, sizeof(line2), "Frame %d/%d", ctx->auto_frame_count,
             AUTO_FRAMES_PER_LASER);
    break;
  case AUTO_CAPTURE_DARK:
    snprintf(line1, sizeof(line1), "Capturing Dark");
    snprintf(line2, sizeof(line2), "Frame %d/%d", ctx->auto_frame_count,
             AUTO_FRAMES_PER_LASER);
    break;
  case AUTO_MOVE_LASER2:
  case AUTO_SETTLE_LASER2:
    snprintf(line1, sizeof(line1), "Moving: 450nm");
    snprintf(line2, sizeof(line2), "Please wait...");
    break;
  case AUTO_CAPTURE_LASER2:
    snprintf(line1, sizeof(line1), "450nm Capture");
    snprintf(line2, sizeof(line2), "Frame %d/%d", ctx->auto_frame_count,
             AUTO_FRAMES_PER_LASER);
    break;
  case AUTO_SAVING:
    snprintf(line1, sizeof(line1), "Saving to SD...");
    snprintf(line2, sizeof(line2), "Please wait");
    break;
  case AUTO_COMPLETE: {
    if (ctx->result_chl_a <= -4.0f || ctx->result_chl_b <= -4.0f) {
      snprintf(line1, sizeof(line1), "Low Signal!");
    } else {
      int a_int = (int)ctx->result_chl_a;
      int a_frac = (int)(fabsf(ctx->result_chl_a - a_int) * 100);
      int b_int = (int)ctx->result_chl_b;
      int b_frac = (int)(fabsf(ctx->result_chl_b - b_int) * 100);

      snprintf(line1, sizeof(line1), "A:%s%d.%02d B:%s%d.%02d",
               (ctx->result_chl_a < 0 ? "-" : ""), abs(a_int), a_frac,
               (ctx->result_chl_b < 0 ? "-" : ""), abs(b_int), b_frac);
    }
  }
    snprintf(line2, sizeof(line2), "OK=Done");
    break;
  default:
    return;
  }

  LCD_PrintLine(lcd, 0, line1);
  LCD_PrintLine(lcd, 1, line2);
}

static void Render_OldMeasurements(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  // Check SD card first
  if (SD_IsPresent()) {
    if (ctx->old_meas_total == 0) {
      LCD_PrintLine(lcd, 0, "SD: No projects");
      LCD_PrintLine(lcd, 1, "LEFT=Back");
      return;
    }

    // Show current selection from SD
    uint8_t sel_in_page = ctx->meas_sel;
    if (sel_in_page >= ctx->old_meas_page_count)
      sel_in_page = ctx->old_meas_page_count - 1;

    uint16_t proj_idx = ctx->old_meas_indices[sel_in_page];
    char buf[17];
    snprintf(buf, sizeof(buf), ">MEAS_%03d  %d/%d", proj_idx,
             ctx->old_meas_page * 4 + sel_in_page + 1, ctx->old_meas_total);
    LCD_PrintLine(lcd, 0, buf);
    LCD_PrintLine(lcd, 1, "OK=View LR=Page");
    return;
  }

  // Fallback: backup SRAM measurements
  if (ctx->meas_count == 0) {
    LCD_PrintLine(lcd, 0, "No measurements");
    LCD_PrintLine(lcd, 1, "LEFT to return");
    return;
  }

  uint8_t idx = ctx->meas_sel;
  if (idx >= ctx->meas_count)
    idx = ctx->meas_count - 1;
  MeasurementRecord *m = &ctx->measurements[idx];

  char buf[20];
  int a_int = (int)m->chl_a;
  int a_frac = (int)(fabsf(m->chl_a - a_int) * 100);
  int b_int = (int)m->chl_b;
  int b_frac = (int)(fabsf(m->chl_b - b_int) * 100);

  snprintf(buf, sizeof(buf), "#%03d A:%s%d.%02d", m->index,
           (m->chl_a < 0 ? "-" : ""), abs(a_int), a_frac);
  LCD_PrintLine(lcd, 0, buf);

  snprintf(buf, sizeof(buf), "     B:%s%d.%02d", (m->chl_b < 0 ? "-" : ""),
           abs(b_int), b_frac);
  LCD_PrintLine(lcd, 1, buf);
}

static void Render_Settings(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  if (ctx->settings_sel == 0) {
    LCD_PrintLine(lcd, 0, ">Laser PWM");
    LCD_PrintLine(lcd, 1, " Integ Time");
  } else {
    LCD_PrintLine(lcd, 0, " Laser PWM");
    LCD_PrintLine(lcd, 1, ">Integ Time");
  }
}

static uint32_t laser_lcd_last_update = 0;
static uint8_t laser_line2_written = 0;

static void Render_LaserSettings(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  uint32_t now = HAL_GetTick();
  if ((now - laser_lcd_last_update) < 100)
    return;
  laser_lcd_last_update = now;

  uint8_t pot_pct = Menu_ReadPotPercent(ctx->laser_sel);

  // LIVE PREVIEW: Activate only the selected laser at pot brightness
  uint16_t live_pwm = PctToPwm(pot_pct);
  if (ctx->laser_sel == 0) {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, live_pwm); // 405nm ON
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);        // 450nm OFF
  } else {
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);        // 405nm OFF
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, live_pwm); // 450nm ON
  }

  LCD_SetCursor(lcd, 0, 0);
  LCD_Print(lcd, (ctx->laser_sel == 0) ? "405nm" : "450nm");
  LCD_DrawProgressBar(lcd, 5, 0, pot_pct);

  laser_line2_written = 1;
}

static void Render_IntegSettings(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  LCD_PrintLine(lcd, 0, ">Integ Time");
  char buf[17];
  snprintf(buf, sizeof(buf), "%d ms", ctx->settings.integration_time);
  LCD_PrintLine(lcd, 1, buf);
}

static void Render_OldMeasView(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  char line1[17];
  char line2[17];

  // Title: Measurement ID
  if (SD_IsPresent()) {
    // We are viewing the selected item from the page
    // Reuse old_meas_indices if valid? Yes, we didn't clear them.
    if (ctx->meas_sel < ctx->old_meas_page_count) {
      uint16_t idx = ctx->old_meas_indices[ctx->meas_sel];
      snprintf(line1, sizeof(line1), "MEAS_%03d (SD)", idx);
    } else {
      snprintf(line1, sizeof(line1), "Invalid Select");
    }
  } else {
    // SRAM
    if (ctx->meas_sel < ctx->meas_count) {
      uint16_t idx = ctx->measurements[ctx->meas_sel].index;
      snprintf(line1, sizeof(line1), "MEAS_%03d (RAM)", idx);
      // Load data for display (SRAM already has it)
      ctx->result_chl_a = ctx->measurements[ctx->meas_sel].chl_a;
      ctx->result_chl_b = ctx->measurements[ctx->meas_sel].chl_b;
    } else {
      snprintf(line1, sizeof(line1), "Invalid Select");
    }
  }

  // Value
  // For SD, we loaded it in Menu_Update
  if (ctx->result_chl_a < -8.0f) { // e.g. -99.0 init value? using < -8 for
                                   // safety against -1,-2,-3 codes
    snprintf(line2, sizeof(line2), "No Data Found");
  } else {
    int a_int = (int)ctx->result_chl_a;
    int a_frac = (int)(fabsf(ctx->result_chl_a - a_int) * 100);
    int b_int = (int)ctx->result_chl_b;
    int b_frac = (int)(fabsf(ctx->result_chl_b - b_int) * 100);

    snprintf(line2, sizeof(line2), "A:%s%d.%02d B:%s%d.%02d",
             (ctx->result_chl_a < 0 ? "-" : ""), abs(a_int), a_frac,
             (ctx->result_chl_b < 0 ? "-" : ""), abs(b_int), b_frac);
  }

  LCD_Clear(lcd);
  LCD_SetCursor(lcd, 0, 0);
  LCD_Print(lcd, line1);
  LCD_SetCursor(lcd, 0, 1);
  LCD_Print(lcd, line2);
}

// ============================================================
// Helper: refresh Old Measurements page from SD
// ============================================================
static void OldMeas_RefreshPage(MenuContext *ctx) {
  ctx->old_meas_total = SD_CountProjects();
  if (ctx->old_meas_total > 0) {
    SD_ListProjects(ctx->old_meas_page * 4, ctx->old_meas_indices, 4,
                    &ctx->old_meas_page_count);
  } else {
    ctx->old_meas_page_count = 0;
  }
  ctx->meas_sel = 0;
}

// ============================================================
// Auto-Measurement State Machine (runs in Menu_Update)
// ============================================================
static void AutoMeas_Tick(MenuContext *ctx) {
  uint32_t now = HAL_GetTick();

  switch (ctx->auto_state) {
  case AUTO_MOVE_LASER1:
    // Move servo to laser 1 position, turn on laser 1
    Servo_MoveTo(0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    ctx->auto_timer = now;
    ctx->auto_state = AUTO_SETTLE_LASER1;
    ctx->need_redraw = 1;
    break;

  case AUTO_SETTLE_LASER1:
    if ((now - ctx->auto_timer) >= SERVO_SETTLE_MS) {
      ctx->auto_frame_count = 0;
      ctx->auto_state = AUTO_CAPTURE_LASER1;
      ctx->ccd_running = 1; // Signal main loop to start CCD
      ctx->need_redraw = 1;
    }
    break;

  case AUTO_CAPTURE_LASER1:
    // Frames are fed via Menu_AutoMeas_OnFrame callback
    // Transition happens in that callback when enough frames collected
    break;

  case AUTO_MOVE_LASER2:
    Servo_MoveTo(1);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
    ctx->auto_timer = now;
    ctx->auto_state = AUTO_SETTLE_LASER2;
    ctx->need_redraw = 1;
    break;

  case AUTO_SETTLE_LASER2:
    if ((now - ctx->auto_timer) >= SERVO_SETTLE_MS) {
      ctx->auto_frame_count = 0;
      ctx->auto_state = AUTO_CAPTURE_LASER2;
      ctx->ccd_running = 1;
      ctx->need_redraw = 1;
    }
    break;

  case AUTO_CAPTURE_LASER2:
    // Frames are fed via Menu_AutoMeas_OnFrame callback
    break;

  case AUTO_SAVING:
    // Saving is done in Menu_AutoMeas_OnFrame after last frame
    // Turn off lasers
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
    Servo_SetPulse(SERVO_POS_NEUTRAL);
    ctx->auto_state = AUTO_COMPLETE;
    ctx->need_redraw = 1;
    break;

  case AUTO_COMPLETE:
  case AUTO_IDLE:
    break;
  }
}

// ============================================================
// Menu Logic
// ============================================================
static uint32_t splash_start_tick = 0;

void Menu_Init(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  memset(ctx, 0, sizeof(MenuContext));
  ctx->screen = SCREEN_SPLASH;
  splash_start_tick = HAL_GetTick();
  ctx->need_redraw = 1;

  Settings_Load(&ctx->settings);
  Measurements_Load(ctx);

  // Lasers OFF on boot — they are turned on when Run CCD or Auto Measure starts
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
}

uint8_t Menu_Update(MenuContext *ctx, LCD_HandleTypeDef *lcd) {
  ButtonID id;
  ButtonEvent event;

  // Run auto-measurement state machine if active
  if (ctx->screen == SCREEN_AUTO_MEASURE && ctx->auto_state != AUTO_IDLE) {
    AutoMeas_Tick(ctx);
  }

  // Process all pending events in the queue
  while (Buttons_GetNextEvent(&id, &event)) {
    if (event == BTN_EVENT_RELEASED)
      continue;

    switch (ctx->screen) {

    // ---- SPLASH SCREEN ----
    case SCREEN_SPLASH:
      // Allow skipping with OK
      if (id == BTN_ID_OK) {
        ctx->screen = SCREEN_MAIN_MENU;
        ctx->need_redraw = 1;
        Buttons_Reset();
      }
      break;

    // ---- MAIN MENU ----
    case SCREEN_MAIN_MENU:
      if (id == BTN_ID_UP && ctx->menu_sel > 0) {
        ctx->menu_sel--;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_DOWN && ctx->menu_sel < MENU_ITEM_COUNT - 1) {
        ctx->menu_sel++;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_OK) {
        switch (ctx->menu_sel) {
        case 0: // Run CCD
          Buttons_Reset();
          ctx->screen = SCREEN_RUN_CCD;
          ctx->ccd_running = 1;
          ctx->ccd_paused = 0;
          ctx->ccd_frame_count = 0;
          // Start with 405nm laser active
          ctx->active_laser = 0;
          Servo_MoveTo(0);
          __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,
                                ctx->settings.laser1_pwm);
          __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
          break;
        case 1: // Auto Measure
          Buttons_Reset();
          ctx->screen = SCREEN_AUTO_MEASURE;
          ctx->auto_state = AUTO_IDLE;
          break;
        case 2: // Old Measurements
          Buttons_Reset();
          ctx->screen = SCREEN_OLD_MEASUREMENTS;
          ctx->meas_sel = 0;
          ctx->old_meas_page = 0;
          if (SD_IsPresent())
            OldMeas_RefreshPage(ctx);
          break;
        case 3: // Settings
          Buttons_Reset();
          ctx->screen = SCREEN_SETTINGS;
          break;
        }
        ctx->need_redraw = 1;
      }
      break;

    // ---- RUN CCD ----
    case SCREEN_RUN_CCD:
      if (id == BTN_ID_LEFT) {
        ctx->ccd_running = 0;
        ctx->ccd_paused = 0;
        // Turn off lasers when leaving Run CCD
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
        Buttons_Reset();
        ctx->screen = SCREEN_MAIN_MENU;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_UP) {
        // Switch to 405nm laser
        ctx->active_laser = 0;
        Servo_MoveTo(0);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_DOWN) {
        // Switch to 450nm laser
        ctx->active_laser = 1;
        Servo_MoveTo(1);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_RIGHT) {
        uint32_t now = HAL_GetTick();
        if ((now - ctx->right_last_tick) < DBLCLICK_MS) {
          // Double-click RIGHT: reset frame counter
          ctx->ccd_frame_count = 0;
          ctx->right_last_tick = 0; // Consume the double-click
        } else {
          // Single-click RIGHT: toggle pause/resume
          ctx->ccd_paused = !ctx->ccd_paused;
          if (ctx->ccd_paused) {
            // Paused: turn off active laser
            __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
            __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
          } else {
            // Resumed: turn on only the active laser
            if (ctx->active_laser == 0) {
              __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1,
                                    ctx->settings.laser1_pwm);
              __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
            } else {
              __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
              __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2,
                                    ctx->settings.laser2_pwm);
            }
          }
          ctx->right_last_tick = now;
        }
        ctx->need_redraw = 1;
      }
      break;

    // ---- AUTO MEASURE ----
    case SCREEN_AUTO_MEASURE:
      if (ctx->auto_state == AUTO_IDLE) {
        if (id == BTN_ID_LEFT) {
          Buttons_Reset();
          ctx->screen = SCREEN_MAIN_MENU;
          ctx->need_redraw = 1;
        }
        if (id == BTN_ID_OK) {
          // Create SD project
          ctx->auto_proj_index = ctx->meas_next_index;
          if (SD_IsPresent()) {
            ProjectConfig pcfg;
            pcfg.meas_index = ctx->auto_proj_index;
            pcfg.integration_time_ms = integration_time_ms;
            pcfg.pwm_405 = ctx->settings.laser1_pwm;
            pcfg.pwm_450 = ctx->settings.laser2_pwm;
            pcfg.frames_per_laser = AUTO_FRAMES_PER_LASER;
            pcfg.timestamp = HAL_GetTick();
            SD_CreateProject(ctx->auto_proj_index, &pcfg);
          }
          ctx->auto_state = AUTO_CAPTURE_DARK;
          ctx->auto_frame_count = 0;
          ctx->ccd_running = 1; // Start CCD if not already
          ctx->need_redraw = 1;
          Buttons_Reset();
        }
      } else if (ctx->auto_state == AUTO_COMPLETE) {
        if (id == BTN_ID_OK || id == BTN_ID_LEFT) {
          // meas_next_index already incremented by Menu_SaveMeasurement()
          Buttons_Reset();
          ctx->screen = SCREEN_MAIN_MENU;
          ctx->auto_state = AUTO_IDLE;
          ctx->need_redraw = 1;
        }
      }
      // During capture, ignore button presses
      break;

    // ---- OLD MEASUREMENTS ----
    case SCREEN_OLD_MEASUREMENTS:
      if (id == BTN_ID_LEFT) {
        Buttons_Reset();
        ctx->screen = SCREEN_MAIN_MENU;
        ctx->need_redraw = 1;
      }
      if (SD_IsPresent()) {
        // SD card pagination: UP/DOWN scroll within page, LEFT/RIGHT change
        // page
        if (id == BTN_ID_UP && ctx->meas_sel > 0) {
          ctx->meas_sel--;
          ctx->need_redraw = 1;
        }
        if (id == BTN_ID_DOWN && ctx->meas_sel < ctx->old_meas_page_count - 1) {
          ctx->meas_sel++;
          ctx->need_redraw = 1;
        }
        if (id == BTN_ID_RIGHT) {
          uint16_t max_page = (ctx->old_meas_total + 3) / 4 - 1;
          if (ctx->old_meas_page < max_page) {
            ctx->old_meas_page++;
            OldMeas_RefreshPage(ctx);
            ctx->need_redraw = 1;
          }
        }
        // LEFT for page back (but LEFT=0 handled above as "go back")
        // So allow re-entering with another press or use dedicated key
      } else {
        // Backup SRAM fallback
        if (id == BTN_ID_UP && ctx->meas_sel > 0) {
          ctx->meas_sel--;
          ctx->need_redraw = 1;
        }
        if (id == BTN_ID_DOWN && ctx->meas_count > 0 &&
            ctx->meas_sel < ctx->meas_count - 1) {
          ctx->meas_sel++;
          ctx->need_redraw = 1;
        }
      }
      // Handle OK button to view result
      if (id == BTN_ID_OK) {
        if (SD_IsPresent()) {
          if (ctx->meas_sel < ctx->old_meas_page_count) {
            uint16_t idx = ctx->old_meas_indices[ctx->meas_sel];
            // reuse result_chl_a/b for viewing
            SD_LoadMeasResult(idx, &ctx->result_chl_a, &ctx->result_chl_b);
            ctx->screen = SCREEN_OLD_MEAS_VIEW;
            ctx->need_redraw = 1;
            Buttons_Reset();
          }
        } else {
          // SRAM view (data is already in ctx->measurements)
          // We can just switch to view, Render_OldMeasView will handle it
          ctx->screen = SCREEN_OLD_MEAS_VIEW;
          ctx->need_redraw = 1;
          Buttons_Reset();
        }
      }
      break;

    // ---- OLD MEAS VIEW ----
    case SCREEN_OLD_MEAS_VIEW:
      if (id == BTN_ID_OK || id == BTN_ID_LEFT) {
        ctx->screen = SCREEN_OLD_MEASUREMENTS;
        ctx->need_redraw = 1;
        Buttons_Reset();
      }
      break;

    // ---- SETTINGS ----
    case SCREEN_SETTINGS:
      if (id == BTN_ID_LEFT) {
        // Turn off lasers when leaving Settings
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
        Buttons_Reset();
        ctx->screen = SCREEN_MAIN_MENU;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_UP && ctx->settings_sel > 0) {
        ctx->settings_sel--;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_DOWN && ctx->settings_sel < 1) {
        ctx->settings_sel++;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_OK) {
        Buttons_Reset();
        if (ctx->settings_sel == 0) {
          ctx->screen = SCREEN_SETTINGS_LASER;
          ctx->laser_sel = 0;
        } else {
          ctx->screen = SCREEN_SETTINGS_INTEG;
        }
        ctx->need_redraw = 1;
      }
      break;

    // ---- INTEGRATION TIME SETTINGS ----
    case SCREEN_SETTINGS_INTEG:
      if (id == BTN_ID_LEFT) {
        // Cancel / Go Back
        ctx->screen = SCREEN_SETTINGS;
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_UP) {
        // Increase
        if (ctx->settings.integration_time < 10000) {
          if (ctx->settings.integration_time < 100)
            ctx->settings.integration_time += 10;
          else
            ctx->settings.integration_time += 50;
        }
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_DOWN) {
        // Decrease
        if (ctx->settings.integration_time > 10) {
          if (ctx->settings.integration_time <= 100)
            ctx->settings.integration_time -= 10;
          else
            ctx->settings.integration_time -= 50;
        }
        ctx->need_redraw = 1;
      }
      if (id == BTN_ID_OK) {
        // Save and Exit
        Settings_Save(&ctx->settings);
        integration_time_ms = ctx->settings.integration_time;
        ctx->screen = SCREEN_SETTINGS;
        ctx->need_redraw = 1;
        Buttons_Reset();
      }
      break;

    // ---- LASER PWM SETTINGS ----
    case SCREEN_SETTINGS_LASER:
      if (id == BTN_ID_LEFT && ctx->laser_sel > 0) {
        ctx->laser_sel = 0;
        ctx->need_redraw = 1;
      } else if (id == BTN_ID_LEFT && ctx->laser_sel == 0) {
        // Restore saved laser PWM values (turn off live preview)
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
        laser_line2_written = 0;
        laser_lcd_last_update = 0;
        Buttons_Reset();
        ctx->screen = SCREEN_SETTINGS;
        ctx->need_redraw = 1;
      }

      if (id == BTN_ID_RIGHT && ctx->laser_sel < 1) {
        ctx->laser_sel = 1;
        ctx->need_redraw = 1;
      }

      if (id == BTN_ID_OK) {
        uint8_t pot_pct = Menu_ReadPotPercent(ctx->laser_sel);
        uint16_t pwm_val = PctToPwm(pot_pct);

        if (ctx->laser_sel == 0) {
          ctx->settings.laser1_pwm = pwm_val;
        } else {
          ctx->settings.laser2_pwm = pwm_val;
        }

        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
        Settings_Save(&ctx->settings);

        LCD_PrintLine(lcd, 1, "  ** Saved! **");
        HAL_Delay(300);

        // Restore saved laser PWM values after save
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
        laser_line2_written = 0;
        laser_lcd_last_update = 0;
        Buttons_Reset();
        ctx->screen = SCREEN_SETTINGS;
        ctx->need_redraw = 1;
      }
      break;
    }
  }

  // Auto-transition for splash screen
  if (ctx->screen == SCREEN_SPLASH) {
    if ((HAL_GetTick() - splash_start_tick) > 2000) {
      ctx->screen = SCREEN_MAIN_MENU;
      ctx->need_redraw = 1;
    }
  }

  // --- Auto-Measure State Machine (Timed/Active Transitions) ---
  if (ctx->screen == SCREEN_AUTO_MEASURE) {
    switch (ctx->auto_state) {
    case AUTO_MOVE_LASER1:
      Servo_MoveTo(0);
      ctx->auto_timer = HAL_GetTick();
      ctx->auto_state = AUTO_SETTLE_LASER1;
      ctx->need_redraw = 1;
      break;

    case AUTO_SETTLE_LASER1:
      if ((HAL_GetTick() - ctx->auto_timer) > SERVO_SETTLE_MS) {
        // Turn ON Laser 1
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, ctx->settings.laser1_pwm);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
        ctx->auto_frame_count = 0;
        ctx->auto_state = AUTO_CAPTURE_LASER1;
        ctx->ccd_running = 1; // Start capture
        ctx->need_redraw = 1;
      }
      break;

    case AUTO_MOVE_LASER2:
      // Turn OFF Laser 1 while moving
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
      Servo_MoveTo(1);
      ctx->auto_timer = HAL_GetTick();
      ctx->auto_state = AUTO_SETTLE_LASER2;
      ctx->need_redraw = 1;
      break;

    case AUTO_SETTLE_LASER2:
      if ((HAL_GetTick() - ctx->auto_timer) > SERVO_SETTLE_MS) {
        // Turn ON Laser 2
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, ctx->settings.laser2_pwm);
        ctx->auto_frame_count = 0;
        ctx->auto_state = AUTO_CAPTURE_LASER2;
        ctx->ccd_running = 1; // Start capture
        ctx->need_redraw = 1;
      }
      break;

    case AUTO_SAVING:
      // Turn OFF lasers after finish
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 0);
      ctx->auto_state = AUTO_COMPLETE;
      ctx->need_redraw = 1;
      break;

    default:
      break;
    }
  }

  // --- Rendering phase ---
  if (ctx->need_redraw) {
    switch (ctx->screen) {
    case SCREEN_SPLASH:
      Render_SplashScreen(ctx, lcd);
      break;
    case SCREEN_MAIN_MENU:
      Render_MainMenu(ctx, lcd);
      break;
    case SCREEN_RUN_CCD:
      Render_RunCCD(ctx, lcd);
      break;
    case SCREEN_AUTO_MEASURE:
      Render_AutoMeasure(ctx, lcd);
      break;
    case SCREEN_OLD_MEASUREMENTS:
      Render_OldMeasurements(ctx, lcd);
      break;
    case SCREEN_OLD_MEAS_VIEW:
      Render_OldMeasView(ctx, lcd);
      break;
    case SCREEN_SETTINGS:
      Render_Settings(ctx, lcd);
      break;
    case SCREEN_SETTINGS_INTEG:
      Render_IntegSettings(ctx, lcd);
      break;
    case SCREEN_SETTINGS_LASER:
      break; // Handled below (throttled)
    }
    ctx->need_redraw = 0;
  }

  // Laser Settings needs continuous update for pot values
  if (ctx->screen == SCREEN_SETTINGS_LASER) {
    Render_LaserSettings(ctx, lcd);
  }

  // Return code:
  // 0 = idle
  // 1 = run CCD (normal mode)
  // 2 = run CCD (auto-measurement mode)
  if (ctx->screen == SCREEN_RUN_CCD && ctx->ccd_running && !ctx->ccd_paused)
    return 1;
  if (ctx->screen == SCREEN_AUTO_MEASURE && ctx->ccd_running)
    return 2;
  return 0;
}

void Menu_CCD_FrameUpdate(MenuContext *ctx, LCD_HandleTypeDef *lcd,
                          uint32_t frame_num) {
  ctx->ccd_frame_count = frame_num;
  char buf[17];
  snprintf(buf, sizeof(buf), "Frame: %lu", (unsigned long)frame_num);
  LCD_PrintLine(lcd, 1, buf);
}

void Menu_SaveMeasurement(MenuContext *ctx, float chl_a, float chl_b) {
  uint8_t slot = ctx->meas_count;
  if (slot >= MAX_MEASUREMENTS) {
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
  // Save to persistent storage (Backup SRAM)
  Measurements_Save(ctx);

  // If SD is present, also save the result to the project folder
  if (SD_IsPresent()) {
    SD_SaveMeasResult(ctx->auto_proj_index, chl_a, chl_b);
  }
}

// ============================================================
// Auto-Measurement Frame Callback
// Called from main loop when a CCD frame is ready during auto-measure
// ============================================================
// External reference to integration time (from main.c)
extern volatile uint32_t integration_time_ms;

void Menu_AutoMeas_OnFrame(MenuContext *ctx, const uint16_t *pixels,
                           uint16_t pixel_count) {

  // 1. Accumulate Frame Data
  // ------------------------
  int len = (pixel_count < FULL_SPECTRUM_LEN) ? pixel_count : FULL_SPECTRUM_LEN;

  if (ctx->auto_frame_count == 0) {
    // First frame: overwrite accumulator
    for (int i = 0; i < len; i++) {
      ctx->auto_accum[i] = (float)pixels[i];
    }
  } else {
    // Subsequent frames: accumulate
    for (int i = 0; i < len; i++) {
      ctx->auto_accum[i] += (float)pixels[i];
    }
  }

  // 2. Process Based on State
  // -------------------------
  if (ctx->auto_state == AUTO_CAPTURE_DARK) {
    ctx->auto_frame_count++;
    ctx->need_redraw = 1;

    // Last frame for Dark Reference?
    if (ctx->auto_frame_count >= AUTO_FRAMES_PER_LASER) {
      // Average for Dark baseline
      for (int i = 0; i < len; i++) {
        ctx->dark_accum[i] = ctx->auto_accum[i] / (float)AUTO_FRAMES_PER_LASER;
      }
      ctx->auto_frame_count = 0;
      ctx->auto_state = AUTO_MOVE_LASER1;
    }

  } else if (ctx->auto_state == AUTO_CAPTURE_LASER1) {
    // Save raw frame to SD (optional, but good for debugging/data collection)
    if (SD_IsPresent()) {
      SD_SaveFrame(ctx->auto_proj_index, 0, pixels, pixel_count,
                   ctx->auto_frame_count);
    }

    ctx->auto_frame_count++;
    ctx->need_redraw = 1;

    // Last frame for Laser 1? Run Prediction
    if (ctx->auto_frame_count >= AUTO_FRAMES_PER_LASER) {
      // Average the accumulator
      for (int i = 0; i < len; i++) {
        ctx->auto_accum[i] /= (float)AUTO_FRAMES_PER_LASER;
      }

      // Predict Chl-a
      float int_time = (float)integration_time_ms;
      if (int_time < 0.1f)
        int_time = 1.0f; // Safety

      chl_status_t status =
          chl_predict_chla(&ctx->predictor, ctx->auto_accum, ctx->dark_accum,
                           int_time, &ctx->result_chl_a);
      if (status != CHL_OK) {
        ctx->result_chl_a = -1.0f; // Error flag (e.g. invalid time)
        if (status == CHL_ERR_INVALID_TIME)
          ctx->result_chl_a = -2.0f; // Specific error
        if (status == CHL_ERR_LOW_SIGNAL)
          ctx->result_chl_a = -4.0f; // Low Signal
      } else if (ctx->result_chl_a == 0.0f) {
        ctx->result_chl_a = -3.0f; // Suspicious exact zero
      }

      ctx->ccd_running = 0;
      ctx->auto_state = AUTO_MOVE_LASER2;
    }

  } else if (ctx->auto_state == AUTO_CAPTURE_LASER2) {
    if (SD_IsPresent()) {
      SD_SaveFrame(ctx->auto_proj_index, 1, pixels, pixel_count,
                   ctx->auto_frame_count);
    }

    ctx->auto_frame_count++;
    ctx->need_redraw = 1;

    // Last frame for Laser 2? Run Prediction
    if (ctx->auto_frame_count >= AUTO_FRAMES_PER_LASER) {
      // Average
      for (int i = 0; i < len; i++) {
        ctx->auto_accum[i] /= (float)AUTO_FRAMES_PER_LASER;
      }

      // Predict Chl-b
      float int_time = (float)integration_time_ms;
      if (int_time < 0.1f)
        int_time = 1.0f;

      chl_status_t status =
          chl_predict_chlb(&ctx->predictor, ctx->auto_accum, ctx->dark_accum,
                           int_time, &ctx->result_chl_b);
      if (status != CHL_OK) {
        ctx->result_chl_b = -1.0f;
        if (status == CHL_ERR_INVALID_TIME)
          ctx->result_chl_b = -2.0f;
        if (status == CHL_ERR_LOW_SIGNAL)
          ctx->result_chl_b = -4.0f;
      } else if (ctx->result_chl_b == 0.0f) {
        ctx->result_chl_b = -3.0f;
      }

      ctx->ccd_running = 0;
      // Save measurement record + result to SD
      Menu_SaveMeasurement(ctx, ctx->result_chl_a, ctx->result_chl_b);
      ctx->auto_state = AUTO_SAVING;
    }
  }
}
