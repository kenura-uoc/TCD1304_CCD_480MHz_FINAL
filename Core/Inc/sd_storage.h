#ifndef SD_STORAGE_H
#define SD_STORAGE_H

#include "main.h"

/**
 * @brief SD card storage for CCD measurement projects
 *
 * Folder structure on SD card:
 *   /measurements/
 *     /MEAS_001/
 *       config.txt      (settings: integration_time, pwm_405, pwm_450,
 * timestamp) laser1_data.bin  (raw CCD frames from 405nm) laser2_data.bin  (raw
 * CCD frames from 450nm) /MEAS_002/
 *       ...
 */

#define SD_MAX_PROJECT_NAME 32
#define SD_MAX_PROJECTS 255 // Effectively unlimited (paginated)

// Project configuration saved alongside data
typedef struct {
  uint16_t integration_time_ms;
  uint16_t pwm_405;
  uint16_t pwm_450;
  uint16_t frames_per_laser;
  uint32_t timestamp;  // HAL_GetTick at creation
  uint16_t meas_index; // Sequential measurement number
} ProjectConfig;

// === SD Card status ===
uint8_t SD_Init(void);      // Mount filesystem, returns 1 on success
uint8_t SD_IsPresent(void); // Check if SD card is mounted
void SD_Deinit(void);       // Unmount

// === Project management ===
// Create a new project folder, returns 1 on success
uint8_t SD_CreateProject(uint16_t index, const ProjectConfig *cfg);

// Save raw CCD frame data for a specific laser
// laser_id: 0=405nm, 1=450nm
// pixels: array of CCD pixel values
// pixel_count: number of pixels (3694)
// frame_num: which frame in the sequence (0-based)
uint8_t SD_SaveFrame(uint16_t proj_index, uint8_t laser_id,
                     const uint16_t *pixels, uint16_t pixel_count,
                     uint16_t frame_num);

// Save project config to config.txt
uint8_t SD_SaveConfig(uint16_t proj_index, const ProjectConfig *cfg);

// === Project browsing ===
// Count how many measurement folders exist
uint16_t SD_CountProjects(void);

// Get the measurement index of the Nth project (for pagination)
// page_start: skip this many projects, then return up to max_count
uint8_t SD_ListProjects(uint16_t page_start, uint16_t *indices,
                        uint8_t max_count, uint8_t *out_count);

// Load config from a specific project
uint8_t SD_LoadConfig(uint16_t proj_index, ProjectConfig *cfg);

#endif // SD_STORAGE_H
