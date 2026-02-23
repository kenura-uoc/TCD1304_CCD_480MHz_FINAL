/**
 * @file    sd_storage.c
 * @brief   SD card storage using FatFS via SDMMC1
 *
 * Stores CCD measurement projects in /measurements/MEAS_XXX/ folders.
 * Each project has a config.txt and binary frame data files.
 */
#include "sd_storage.h"
#include "fatfs.h"
#include "main.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// FatFS objects — SDFatFS is provided by fatfs.c (CubeMX)
static uint8_t sd_mounted = 0;

// Deferred init functions from main.c / fatfs.c
extern void MX_FATFS_Init(void);
extern SD_HandleTypeDef hsd1;

// ============================================================
// SD Card lifecycle
// ============================================================
uint8_t SD_Init(void) {
  // Initialize SDMMC1 hardware
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 2;
  if (HAL_SD_Init(&hsd1) != HAL_OK) {
    sd_mounted = 0;
    return 0; // No SD card — that's OK
  }

  // Link FatFS driver
  MX_FATFS_Init();

  // Try to mount
  FRESULT res = f_mount(&SDFatFS, "0:", 1);
  if (res == FR_OK) {
    sd_mounted = 1;
    f_mkdir("0:/measurements");
    return 1;
  }
  sd_mounted = 0;
  return 0;
}

uint8_t SD_IsPresent(void) { return sd_mounted; }

void SD_Deinit(void) {
  if (sd_mounted) {
    f_mount(NULL, "0:", 0);
    sd_mounted = 0;
  }
}

// ============================================================
// Helper: build path string
// ============================================================
static void BuildProjectPath(char *buf, size_t bufsize, uint16_t index) {
  snprintf(buf, bufsize, "0:/measurements/MEAS_%03d", index);
}

// ============================================================
// Project creation and data saving
// ============================================================
uint8_t SD_CreateProject(uint16_t index, const ProjectConfig *cfg) {
  if (!sd_mounted)
    return 0;

  char path[48];
  BuildProjectPath(path, sizeof(path), index);

  FRESULT res = f_mkdir(path);
  if (res != FR_OK && res != FR_EXIST)
    return 0;

  // Save config immediately
  return SD_SaveConfig(index, cfg);
}

uint8_t SD_SaveFrame(uint16_t proj_index, uint8_t laser_id,
                     const uint16_t *pixels, uint16_t pixel_count,
                     uint16_t frame_num) {
  if (!sd_mounted)
    return 0;

  char path[128];
  char proj_path[48];
  BuildProjectPath(proj_path, sizeof(proj_path), proj_index);
  snprintf(path, sizeof(path), "%s/laser%d_data.bin", proj_path, laser_id + 1);

  FIL fil;
  FRESULT res;
  UINT bw;

  if (frame_num == 0) {
    // First frame: create new file
    res = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
  } else {
    // Append subsequent frames
    res = f_open(&fil, path, FA_WRITE | FA_OPEN_APPEND);
  }

  if (res != FR_OK)
    return 0;

  // Write raw pixel data
  res = f_write(&fil, pixels, pixel_count * sizeof(uint16_t), &bw);
  f_close(&fil);

  return (res == FR_OK && bw == pixel_count * sizeof(uint16_t));
}

uint8_t SD_SaveConfig(uint16_t proj_index, const ProjectConfig *cfg) {
  if (!sd_mounted)
    return 0;

  char path[128];
  char proj_path[48];
  BuildProjectPath(proj_path, sizeof(proj_path), proj_index);
  snprintf(path, sizeof(path), "%s/config.txt", proj_path);

  FIL fil;
  FRESULT res = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK)
    return 0;

  char buf[128];
  int len;

  len = snprintf(buf, sizeof(buf), "meas_index=%d\n", cfg->meas_index);
  f_write(&fil, buf, len, NULL);

  len = snprintf(buf, sizeof(buf), "integration_time_ms=%d\n",
                 cfg->integration_time_ms);
  f_write(&fil, buf, len, NULL);

  len = snprintf(buf, sizeof(buf), "pwm_405=%d\n", cfg->pwm_405);
  f_write(&fil, buf, len, NULL);

  len = snprintf(buf, sizeof(buf), "pwm_450=%d\n", cfg->pwm_450);
  f_write(&fil, buf, len, NULL);

  len = snprintf(buf, sizeof(buf), "frames_per_laser=%d\n",
                 cfg->frames_per_laser);
  f_write(&fil, buf, len, NULL);

  len = snprintf(buf, sizeof(buf), "timestamp=%lu\n",
                 (unsigned long)cfg->timestamp);
  f_write(&fil, buf, len, NULL);

  f_close(&fil);
  return 1;
}

// ============================================================
// Project browsing
// ============================================================
uint16_t SD_CountProjects(void) {
  if (!sd_mounted)
    return 0;

  DIR dir;
  FILINFO fno;
  uint16_t count = 0;

  if (f_opendir(&dir, "0:/measurements") != FR_OK)
    return 0;

  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
    if ((fno.fattrib & AM_DIR) && strncmp(fno.fname, "MEAS_", 5) == 0) {
      count++;
    }
  }

  f_closedir(&dir);
  return count;
}

uint8_t SD_ListProjects(uint16_t page_start, uint16_t *indices,
                        uint8_t max_count, uint8_t *out_count) {
  if (!sd_mounted) {
    *out_count = 0;
    return 0;
  }

  DIR dir;
  FILINFO fno;
  uint16_t skip = 0;
  uint8_t found = 0;

  if (f_opendir(&dir, "0:/measurements") != FR_OK) {
    *out_count = 0;
    return 0;
  }

  while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
    if ((fno.fattrib & AM_DIR) && strncmp(fno.fname, "MEAS_", 5) == 0) {
      if (skip < page_start) {
        skip++;
        continue;
      }
      if (found >= max_count)
        break;

      // Extract index from "MEAS_XXX"
      int idx = 0;
      sscanf(&fno.fname[5], "%d", &idx);
      indices[found++] = (uint16_t)idx;
    }
  }

  f_closedir(&dir);
  *out_count = found;
  return 1;
}

uint8_t SD_LoadConfig(uint16_t proj_index, ProjectConfig *cfg) {
  if (!sd_mounted)
    return 0;

  char path[64];
  char proj_path[48];
  BuildProjectPath(proj_path, sizeof(proj_path), proj_index);
  snprintf(path, sizeof(path), "%s/config.txt", proj_path);

  FIL fil;
  if (f_open(&fil, path, FA_READ) != FR_OK)
    return 0;

  char line[64];
  memset(cfg, 0, sizeof(ProjectConfig));

  while (f_gets(line, sizeof(line), &fil)) {
    int val;
    unsigned long uval;
    if (sscanf(line, "meas_index=%d", &val) == 1)
      cfg->meas_index = val;
    else if (sscanf(line, "integration_time_ms=%d", &val) == 1)
      cfg->integration_time_ms = val;
    else if (sscanf(line, "pwm_405=%d", &val) == 1)
      cfg->pwm_405 = val;
    else if (sscanf(line, "pwm_450=%d", &val) == 1)
      cfg->pwm_450 = val;
    else if (sscanf(line, "frames_per_laser=%d", &val) == 1)
      cfg->frames_per_laser = val;
    else if (sscanf(line, "timestamp=%lu", &uval) == 1)
      cfg->timestamp = (uint32_t)uval;
  }

  f_close(&fil);
  return 1;
}

uint8_t SD_SaveMeasResult(uint16_t proj_index, float chl_a, float chl_b) {
  if (!sd_mounted)
    return 0;

  char path[64];
  char proj_path[48];
  BuildProjectPath(proj_path, sizeof(proj_path), proj_index);
  snprintf(path, sizeof(path), "%s/results.txt", proj_path);

  FIL fil;
  FRESULT res = f_open(&fil, path, FA_WRITE | FA_CREATE_ALWAYS);
  if (res != FR_OK)
    return 0;

  char buf[64];
  int len;

  // Manual float formatting
  int a_int = (int)chl_a;
  int a_frac = (int)(fabsf(chl_a - a_int) * 10000); // 4 decimals
  len = snprintf(buf, sizeof(buf), "chl_a=%s%d.%04d\n", (chl_a < 0 ? "-" : ""),
                 abs(a_int), a_frac);
  f_write(&fil, buf, len, NULL);

  int b_int = (int)chl_b;
  int b_frac = (int)(fabsf(chl_b - b_int) * 10000);
  len = snprintf(buf, sizeof(buf), "chl_b=%s%d.%04d\n", (chl_b < 0 ? "-" : ""),
                 abs(b_int), b_frac);
  f_write(&fil, buf, len, NULL);

  f_close(&fil);
  return 1;
}

uint8_t SD_LoadMeasResult(uint16_t proj_index, float *chl_a, float *chl_b) {
  if (!sd_mounted)
    return 0;

  char path[64];
  char proj_path[48];
  BuildProjectPath(proj_path, sizeof(proj_path), proj_index);
  snprintf(path, sizeof(path), "%s/results.txt", proj_path);

  FIL fil;
  if (f_open(&fil, path, FA_READ) != FR_OK)
    return 0;

  char line[64];
  // Defaults
  *chl_a = -1.0f;
  *chl_b = -1.0f;

  while (f_gets(line, sizeof(line), &fil)) {
    // Manual parsing: find "="
    char *eq = strchr(line, '=');
    if (eq) {
      eq++; // Skip '='
      float val =
          (float)atof(eq); // atof usually works even if scanf %f doesn't
      // If atof fails, we might need manual parsing too.
      // Let's assume atof works or try a simple manual fallback if val is 0 but
      // string isn't "0"

      if (strncmp(line, "chl_a=", 6) == 0) {
        *chl_a = val;
      } else if (strncmp(line, "chl_b=", 6) == 0) {
        *chl_b = val;
      }
    }
  }

  f_close(&fil);
  return 1;
}
