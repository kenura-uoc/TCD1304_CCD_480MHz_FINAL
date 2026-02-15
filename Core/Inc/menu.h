/**
 * @file    menu.h
 * @brief   Menu/app state machine for LCD-driven CCD spectrometer
 *
 * Apps: Run CCD, Auto Measure, Old Measurements, Settings (Laser PWM)
 */
#ifndef MENU_H
#define MENU_H

#include "buttons.h"
#include "chl_predictor.h"
#include "liquidcrystal_i2c.h"
#include "main.h"
#include "sd_storage.h"

// --- App/Screen States ---
typedef enum {
  SCREEN_MAIN_MENU = 0,
  SCREEN_RUN_CCD,
  SCREEN_AUTO_MEASURE, // Automated dual-laser measurement
  SCREEN_OLD_MEASUREMENTS,
  SCREEN_SETTINGS,
  SCREEN_SETTINGS_LASER, // Sub-screen: adjust laser PWM
} ScreenState;

// Number of main menu items
#define MENU_ITEM_COUNT 4

// --- Stored measurement record ---
#define MAX_MEASUREMENTS 10

typedef struct {
  uint8_t valid;  // 1 if this slot has data
  uint16_t index; // Measurement number (1-based)
  float chl_a;    // Chlorophyll-a (mg/L) — dummy for now
  float chl_b;    // Chlorophyll-b (mg/L) — dummy for now
} MeasurementRecord;

// --- Persistent settings (saved to backup SRAM) ---
#define SETTINGS_MAGIC 0xCB00 // Bumped — servo fields removed

typedef struct {
  uint16_t magic;      // Identifies valid settings
  uint16_t laser1_pwm; // 405nm duty (0-2399)
  uint16_t laser2_pwm; // 450nm duty (0-2399)
} DeviceSettings;

// --- Auto-measurement sub-states ---
typedef enum {
  AUTO_IDLE = 0,
  AUTO_MOVE_LASER1,    // Moving servo to laser 1 position
  AUTO_SETTLE_LASER1,  // Waiting for servo to settle
  AUTO_CAPTURE_LASER1, // Capturing CCD frames for laser 1
  AUTO_MOVE_LASER2,    // Moving servo to laser 2 position
  AUTO_SETTLE_LASER2,  // Waiting for servo to settle
  AUTO_CAPTURE_LASER2, // Capturing CCD frames for laser 2
  AUTO_SAVING,         // Saving to SD card
  AUTO_COMPLETE,       // Done, showing results
} AutoMeasState;

#define AUTO_FRAMES_PER_LASER 5 // Number of CCD frames to collect per laser
#define SERVO_SETTLE_MS 500     // Time to wait for servo to reach position

// --- Menu context (global state) ---
typedef struct {
  ScreenState screen;
  uint8_t menu_sel;         // Main menu cursor position (0-3)
  uint8_t meas_sel;         // Old measurements scroll index
  uint8_t laser_sel;        // 0=405nm, 1=450nm in settings
  uint8_t ccd_running;      // 1 when CCD acquisition is active
  uint8_t ccd_paused;       // 1 when CCD streaming is paused (Run CCD)
  uint8_t active_laser;     // 0=405nm, 1=450nm — active laser in Run CCD
  uint8_t need_redraw;      // Force LCD redraw
  uint32_t ccd_frame_count; // Frame counter for display
  uint32_t right_last_tick; // Tick of last RIGHT press (for double-click)
  DeviceSettings settings;  // Active saved settings
  MeasurementRecord measurements[MAX_MEASUREMENTS];
  uint8_t meas_count;       // Number of stored measurements
  uint16_t meas_next_index; // Next measurement index

  // Auto-measurement state
  AutoMeasState auto_state;
  uint32_t auto_timer;       // Tick for timing sub-states
  uint16_t auto_frame_count; // Frames captured so far for current laser
  uint16_t auto_proj_index;  // Current project index on SD card

  // ML Inference State
  float auto_accum[FULL_SPECTRUM_LEN]; // Accumulator for averaging frames
  chl_predictor_t predictor;           // ML predictor working memory
  float result_chl_a;                  // Last predicted Chl-a
  float result_chl_b;                  // Last predicted Chl-b

  // Old measurements pagination (SD card)
  uint16_t old_meas_page; // Current page (0-based)
  uint16_t
      old_meas_indices[4];     // Project indices on current page (4 per screen)
  uint8_t old_meas_page_count; // Number of items on current page
  uint16_t old_meas_total;     // Total projects on SD
} MenuContext;

// Initialize menu system (call once after LCD init)
void Menu_Init(MenuContext *ctx, LCD_HandleTypeDef *lcd);

// Process button events and update LCD (call every main loop iteration)
// Returns: 0 = idle, 1 = run CCD normally, 2 = run CCD for auto-measurement
uint8_t Menu_Update(MenuContext *ctx, LCD_HandleTypeDef *lcd);

// Called from CCD app to update frame counter on LCD
void Menu_CCD_FrameUpdate(MenuContext *ctx, LCD_HandleTypeDef *lcd,
                          uint32_t frame_num);

// Save current measurement (called after CCD capture + ML inference)
void Menu_SaveMeasurement(MenuContext *ctx, float chl_a, float chl_b);

// Called from main loop with completed CCD frame data during auto-measurement
void Menu_AutoMeas_OnFrame(MenuContext *ctx, const uint16_t *pixels,
                           uint16_t pixel_count);

// Load/save settings to backup SRAM
void Settings_Load(DeviceSettings *s);
void Settings_Save(const DeviceSettings *s);

// Get current pot reading as percentage (for settings screen)
// pot_id: 0 = pot1 (PC0), 1 = pot2 (PC1)
uint8_t Menu_ReadPotPercent(uint8_t pot_id);

#endif // MENU_H
