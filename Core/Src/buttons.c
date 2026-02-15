/**
 * @file    buttons.c
 * @brief   5-button driver — debounce, edge detect, auto-repeat
 *
 * Hardware: PD0,PD1,PD3,PD4,PD5 active LOW with internal pull-ups
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
} ButtonState;

static ButtonState buttons[BTN_COUNT];

// FIFO Event Queue
#define EVENT_QUEUE_SIZE 16
typedef struct {
  ButtonID id;
  ButtonEvent event;
} ButtonQueueItem;

static ButtonQueueItem event_queue[EVENT_QUEUE_SIZE];
static volatile uint8_t q_head = 0;
static volatile uint8_t q_tail = 0;

// Push event to queue (called from ISR)
static void Queue_Push(ButtonID id, ButtonEvent event) {
  uint8_t next_head = (q_head + 1) % EVENT_QUEUE_SIZE;
  if (next_head != q_tail) { // strict ring buffer full check
    event_queue[q_head].id = id;
    event_queue[q_head].event = event;
    q_head = next_head;
  }
}

#define DEBOUNCE_MS 15
#define HOLD_DELAY_MS 300
#define REPEAT_MS 100

// --- ISR Interface ---
// Call this every 1ms
void Buttons_Tick(void) {
  uint32_t now = HAL_GetTick(); // GetTick is ISR safe (just reads variable)

  for (int i = 0; i < BTN_COUNT; i++) {
    ButtonState *b = &buttons[i];

    // Read GPIO (active LOW: 0 = pressed)
    uint8_t reading =
        (HAL_GPIO_ReadPin(GPIOD, btn_pins[i]) == GPIO_PIN_RESET) ? 1 : 0;

    // Debounce
    if (reading != b->raw) {
      b->raw = reading;
      b->last_change = now;
    }

    if ((now - b->last_change) >= DEBOUNCE_MS) {
      b->prev_stable = b->stable;
      b->stable = b->raw;

      // Edge detection
      if (b->stable && !b->prev_stable) {
        // Pressed
        Queue_Push((ButtonID)i, BTN_EVENT_PRESSED);
        b->hold_next = now + HOLD_DELAY_MS;
      } else if (!b->stable && b->prev_stable) {
        // Released
        Queue_Push((ButtonID)i, BTN_EVENT_RELEASED);
      }

      // Auto-repeat
      if (b->stable && (now >= b->hold_next)) {
        Queue_Push((ButtonID)i, BTN_EVENT_HELD);
        b->hold_next = now + REPEAT_MS;
      }
    }
  }
}

// --- Foreground Interface ---

void Buttons_Init(void) { Buttons_Reset(); }

uint8_t Buttons_GetNextEvent(ButtonID *id, ButtonEvent *event) {
  if (q_head == q_tail) {
    return 0; // Empty
  }

  *id = event_queue[q_tail].id;
  *event = event_queue[q_tail].event;

  q_tail = (q_tail + 1) % EVENT_QUEUE_SIZE;
  return 1;
}

uint8_t Buttons_IsPressed(ButtonID id) {
  if (id >= BTN_COUNT)
    return 0;
  return buttons[id].stable;
}

void Buttons_Reset(void) {
  // Atomic clear (simple way: just reset indices)
  q_head = 0;
  q_tail = 0;
}
