/**
 * @file    menu.h
 * @brief   Menu/app state machine for LCD-driven CCD spectrometer
 *
 * Apps: Run CCD, Old Measurements, Settings (Laser PWM)
 */
#ifndef MENU_H
#define MENU_H

#include "buttons.h"
#include "liquidcrystal_i2c.h"
#include "main.h"


// --- App/Screen States ---
typedef enum {
  SCREEN_MAIN_MENU = 0,
  SCREEN_RUN_CCD,
  SCREEN_OLD_MEASUREMENTS,
  SCREEN_SETTINGS,
  SCREEN_SETTINGS_LASER, // Sub-screen: adjust laser PWM
} ScreenState;

// --- Stored measurement record ---
#define MAX_MEASUREMENTS 10

typedef struct {
  uint8_t valid;  // 1 if this slot has data
  uint16_t index; // Measurement number (1-based)
  float chl_a;    // Chlorophyll-a (mg/L) — dummy for now
  float chl_b;    // Chlorophyll-b (mg/L) — dummy for now
} MeasurementRecord;

// --- Persistent settings (saved to backup SRAM) ---
#define SETTINGS_MAGIC 0xCAFE

typedef struct {
  uint16_t magic;      // Identifies valid settings
  uint16_t laser1_pwm; // 404nm duty (0-2399)
  uint16_t laser2_pwm; // 450nm duty (0-2399)
} DeviceSettings;

// --- Menu context (global state) ---
typedef struct {
  ScreenState screen;
  uint8_t menu_sel;         // Main menu cursor position (0-2)
  uint8_t meas_sel;         // Old measurements scroll index
  uint8_t laser_sel;        // 0=404nm, 1=450nm in settings
  uint8_t ccd_running;      // 1 when CCD acquisition is active
  uint8_t need_redraw;      // Force LCD redraw
  uint32_t ccd_frame_count; // Frame counter for display
  DeviceSettings settings;  // Active saved settings
  MeasurementRecord measurements[MAX_MEASUREMENTS];
  uint8_t meas_count;       // Number of stored measurements
  uint16_t meas_next_index; // Next measurement index
} MenuContext;

// Initialize menu system (call once after LCD init)
void Menu_Init(MenuContext *ctx, LCD_HandleTypeDef *lcd);

// Process button events and update LCD (call every main loop iteration)
// Returns 1 if CCD acquisition should run this frame, 0 otherwise
uint8_t Menu_Update(MenuContext *ctx, LCD_HandleTypeDef *lcd);

// Called from CCD app to update frame counter on LCD
void Menu_CCD_FrameUpdate(MenuContext *ctx, LCD_HandleTypeDef *lcd,
                          uint32_t frame_num);

// Save current measurement (called after CCD capture + ML inference)
void Menu_SaveMeasurement(MenuContext *ctx, float chl_a, float chl_b);

// Load/save settings to backup SRAM
void Settings_Load(DeviceSettings *s);
void Settings_Save(const DeviceSettings *s);

// Get current pot reading as percentage (for settings screen)
// pot_id: 0 = pot1 (PC0), 1 = pot2 (PC1)
uint8_t Menu_ReadPotPercent(uint8_t pot_id);

#endif // MENU_H
