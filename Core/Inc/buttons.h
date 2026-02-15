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
  BTN_EVENT_HELD      // Held down > 300ms (auto-repeat every 100ms)
} ButtonEvent;

// --- ISR Interface ---
// Call this function every 1ms (e.g., from SysTick or TIM interrupt)
void Buttons_Tick(void);

// --- Foreground Interface ---

// Initialize button queue
void Buttons_Init(void);

// Pop the next event from the queue. Returns 1 if event found, 0 if empty.
// If 1, *id and *event are populated.
uint8_t Buttons_GetNextEvent(ButtonID *id, ButtonEvent *event);

// Check current REAL TIME state (for mode switches etc)
uint8_t Buttons_IsPressed(ButtonID id);

// Clear queue
void Buttons_Reset(void);

#endif // BUTTONS_H
