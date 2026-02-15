/**
 * @file    liquidcrystal_i2c.h
 * @brief   STM32 HAL port of Arduino LiquidCrystal_I2C for PCF8574 backpack
 */
#ifndef LIQUIDCRYSTAL_I2C_H
#define LIQUIDCRYSTAL_I2C_H

#include "stm32h7xx_hal.h"

// PCF8574 I2C LCD backpack bit assignments
#define LCD_RS (1 << 0)
#define LCD_RW (1 << 1)
#define LCD_EN (1 << 2)
#define LCD_BL (1 << 3) // Backlight

// HD44780 commands
#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME 0x02
#define LCD_CMD_ENTRY_MODE 0x04
#define LCD_CMD_DISPLAY_CTRL 0x08
#define LCD_CMD_SHIFT 0x10
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_CGRAM 0x40
#define LCD_CMD_SET_DDRAM 0x80

// Entry mode flags
#define LCD_ENTRY_LEFT 0x02
#define LCD_ENTRY_SHIFT_INC 0x01

// Display control flags
#define LCD_DISPLAY_ON 0x04
#define LCD_CURSOR_ON 0x02
#define LCD_BLINK_ON 0x01

// Function set flags
#define LCD_4BIT_MODE 0x00
#define LCD_2LINE 0x08
#define LCD_5x8DOTS 0x00

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t addr; // 7-bit I2C address (e.g. 0x27)
  uint8_t cols;
  uint8_t rows;
  uint8_t backlight; // LCD_BL or 0
} LCD_HandleTypeDef;

// Core API
void LCD_Init(LCD_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr,
              uint8_t cols, uint8_t rows);
void LCD_Clear(LCD_HandleTypeDef *lcd);
void LCD_Home(LCD_HandleTypeDef *lcd);
void LCD_SetCursor(LCD_HandleTypeDef *lcd, uint8_t col, uint8_t row);
void LCD_Print(LCD_HandleTypeDef *lcd, const char *str);
void LCD_PrintChar(LCD_HandleTypeDef *lcd, char c);
void LCD_Backlight(LCD_HandleTypeDef *lcd, uint8_t on);
void LCD_SendCommand(LCD_HandleTypeDef *lcd, uint8_t cmd);
void LCD_SendData(LCD_HandleTypeDef *lcd, uint8_t data);

#endif // LIQUIDCRYSTAL_I2C_H
