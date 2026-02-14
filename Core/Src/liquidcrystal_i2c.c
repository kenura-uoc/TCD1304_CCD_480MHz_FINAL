/**
 * @file    liquidcrystal_i2c.c
 * @brief   STM32 HAL port of Arduino LiquidCrystal_I2C for PCF8574 backpack
 *
 * Protocol: The PCF8574 expander drives the LCD in 4-bit mode.
 * Each byte sent over I2C maps to: [D7 D6 D5 D4 BL EN RW RS]
 * A nibble transfer requires: EN high pulse with data, then EN low.
 */
#include "liquidcrystal_i2c.h"

// ---------------------------------------------------------------------------
// Low-level I2C send
// ---------------------------------------------------------------------------
static void LCD_I2C_Write(LCD_HandleTypeDef *lcd, uint8_t data) {
  HAL_I2C_Master_Transmit(lcd->hi2c, lcd->addr << 1, &data, 1, 10);
}

// Send a nibble (4 bits) with EN pulse
static void LCD_SendNibble(LCD_HandleTypeDef *lcd, uint8_t nibble, uint8_t rs) {
  uint8_t data = (nibble & 0xF0) | lcd->backlight | rs;

  // EN high
  LCD_I2C_Write(lcd, data | LCD_EN);
  HAL_Delay(1);

  // EN low (latch)
  LCD_I2C_Write(lcd, data & ~LCD_EN);
  HAL_Delay(1);
}

// Send a full byte as two nibbles
static void LCD_SendByte(LCD_HandleTypeDef *lcd, uint8_t byte, uint8_t rs) {
  LCD_SendNibble(lcd, byte & 0xF0, rs);        // High nibble
  LCD_SendNibble(lcd, (byte << 4) & 0xF0, rs); // Low nibble
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void LCD_SendCommand(LCD_HandleTypeDef *lcd, uint8_t cmd) {
  LCD_SendByte(lcd, cmd, 0); // RS=0 for command
}

void LCD_SendData(LCD_HandleTypeDef *lcd, uint8_t data) {
  LCD_SendByte(lcd, data, LCD_RS); // RS=1 for data
}

void LCD_Init(LCD_HandleTypeDef *lcd, I2C_HandleTypeDef *hi2c, uint8_t addr,
              uint8_t cols, uint8_t rows) {
  lcd->hi2c = hi2c;
  lcd->addr = addr;
  lcd->cols = cols;
  lcd->rows = rows;
  lcd->backlight = LCD_BL;

  // Wait for LCD power-up (>40ms after Vcc rises to 2.7V)
  HAL_Delay(50);

  // HD44780 initialization sequence for 4-bit mode
  // Must send Function Set 3 times to reliably enter 4-bit mode
  LCD_SendNibble(lcd, 0x30, 0);
  HAL_Delay(5); // Wait >4.1ms
  LCD_SendNibble(lcd, 0x30, 0);
  HAL_Delay(5); // Wait >100us
  LCD_SendNibble(lcd, 0x30, 0);
  HAL_Delay(1);
  LCD_SendNibble(lcd, 0x20, 0);
  HAL_Delay(1); // Set 4-bit mode

  // Now in 4-bit mode, can send full commands
  LCD_SendCommand(lcd, LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE |
                           LCD_5x8DOTS);
  LCD_SendCommand(lcd, LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON);
  LCD_Clear(lcd);
  LCD_SendCommand(lcd, LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT);

  HAL_Delay(2);
}

void LCD_Clear(LCD_HandleTypeDef *lcd) {
  LCD_SendCommand(lcd, LCD_CMD_CLEAR);
  HAL_Delay(2); // Clear takes ~1.52ms
}

void LCD_Home(LCD_HandleTypeDef *lcd) {
  LCD_SendCommand(lcd, LCD_CMD_HOME);
  HAL_Delay(2);
}

void LCD_SetCursor(LCD_HandleTypeDef *lcd, uint8_t col, uint8_t row) {
  // Row offsets for 16x2 and 20x4 LCDs
  static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
  if (row >= lcd->rows)
    row = lcd->rows - 1;
  LCD_SendCommand(lcd, LCD_CMD_SET_DDRAM | (col + row_offsets[row]));
}

void LCD_PrintChar(LCD_HandleTypeDef *lcd, char c) {
  LCD_SendData(lcd, (uint8_t)c);
}

void LCD_Print(LCD_HandleTypeDef *lcd, const char *str) {
  while (*str) {
    LCD_SendData(lcd, (uint8_t)*str++);
  }
}

void LCD_Backlight(LCD_HandleTypeDef *lcd, uint8_t on) {
  lcd->backlight = on ? LCD_BL : 0;
  LCD_I2C_Write(lcd, lcd->backlight); // Update immediately
}
