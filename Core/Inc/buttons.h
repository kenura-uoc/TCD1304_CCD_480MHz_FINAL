/**
 * @file    buttons.h
 * @brief   5-button driver with software debounce for menu navigation
 *          Active LOW (internal pull-ups), 20ms debounce
 */
#ifndef BUTTONS_H
#define BUTTONS_H

#include "main.h"

// Button identifiers
typedef enum {
  BTN_ID_UP = 0,
  BTN_ID_DOWN,
  BTN_ID_OK,
  BTN_ID_LEFT,
  BTN_ID_RIGHT,
  BTN_COUNT
} ButtonID;

// Button events (edge-detected)
typedef enum {
  BTN_EVENT_NONE = 0,
  BTN_EVENT_PRESSED,  // Just pressed (falling edge)
  BTN_EVENT_RELEASED, // Just released (rising edge)
  BTN_EVENT_HELD      // Held down > 500ms (auto-repeat every 150ms)
} ButtonEvent;

// Poll all buttons (call every loop iteration, uses HAL_GetTick for timing)
void Buttons_Update(void);

// Get event for a specific button (consumed on read)
ButtonEvent Buttons_GetEvent(ButtonID id);

// Check if button is currently held down
uint8_t Buttons_IsPressed(ButtonID id);

#endif // BUTTONS_H
