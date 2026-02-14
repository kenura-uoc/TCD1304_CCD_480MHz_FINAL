/**
 * @file    buttons.c
 * @brief   5-button driver — debounce, edge detect, auto-repeat
 *
 * Hardware: PD0-PD4, active LOW with internal pull-ups
 * Debounce: 20ms (rejects bounces shorter than 20ms)
 * Auto-repeat: 500ms initial delay, then 150ms repeat rate
 */
#include "buttons.h"

// Button hardware mapping (must match order of ButtonID enum)
static const uint16_t btn_pins[BTN_COUNT] = {
    BTN_UP_Pin, BTN_DOWN_Pin, BTN_OK_Pin, BTN_LEFT_Pin, BTN_RIGHT_Pin};

// Per-button state
typedef struct {
  uint8_t raw;          // Current raw reading (0=pressed, 1=released)
  uint8_t stable;       // Debounced state
  uint8_t prev_stable;  // Previous stable state (for edge detection)
  uint32_t last_change; // Tick when raw state last changed
  uint32_t hold_next;   // Tick for next auto-repeat event
  ButtonEvent event;    // Pending event (consumed by GetEvent)
} ButtonState;

static ButtonState buttons[BTN_COUNT];

#define DEBOUNCE_MS 20
#define HOLD_DELAY_MS 500
#define REPEAT_MS 150

void Buttons_Update(void) {
  uint32_t now = HAL_GetTick();

  for (int i = 0; i < BTN_COUNT; i++) {
    ButtonState *b = &buttons[i];

    // Read GPIO (active LOW: 0 = pressed)
    uint8_t reading =
        (HAL_GPIO_ReadPin(GPIOD, btn_pins[i]) == GPIO_PIN_RESET) ? 1 : 0;

    // Debounce: only accept change if stable for DEBOUNCE_MS
    if (reading != b->raw) {
      b->raw = reading;
      b->last_change = now;
    }

    if ((now - b->last_change) >= DEBOUNCE_MS) {
      b->prev_stable = b->stable;
      b->stable = b->raw;

      // Edge detection
      if (b->stable && !b->prev_stable) {
        // Rising edge (button just pressed)
        b->event = BTN_EVENT_PRESSED;
        b->hold_next = now + HOLD_DELAY_MS;
      } else if (!b->stable && b->prev_stable) {
        // Falling edge (button just released)
        b->event = BTN_EVENT_RELEASED;
      }

      // Auto-repeat while held
      if (b->stable && (now >= b->hold_next)) {
        b->event = BTN_EVENT_HELD;
        b->hold_next = now + REPEAT_MS;
      }
    }
  }
}

ButtonEvent Buttons_GetEvent(ButtonID id) {
  if (id >= BTN_COUNT)
    return BTN_EVENT_NONE;
  ButtonEvent e = buttons[id].event;
  buttons[id].event = BTN_EVENT_NONE; // Consume
  return e;
}

uint8_t Buttons_IsPressed(ButtonID id) {
  if (id >= BTN_COUNT)
    return 0;
  return buttons[id].stable;
}
