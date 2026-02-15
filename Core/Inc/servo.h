#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/**
 * @brief Servo driver for MG90 on TIM1_CH1 (PA8)
 *
 * Linear actuator positions:
 *   Position 0 = Laser 1 (405nm) over sample
 *   Position 1 = Laser 2 (450nm) over sample
 *
 * Pulse widths (at 50Hz / 20ms period):
 *   1ms (1000 counts) = 0°
 *   1.5ms (1500 counts) = 90° (neutral)
 *   2ms (2000 counts) = 180°
 */

// Calibration: servo pulse widths for each laser position
// Adjust these to match your linear actuator travel
#define SERVO_POS_LASER1 900   // a bit more right, laser 1 (405nm)
#define SERVO_POS_LASER2 1500  // ~90°  — center, laser 2 (450nm) over sample
#define SERVO_POS_NEUTRAL 1500 // ~90°  — center / rest

void Servo_Init(void);
void Servo_SetPulse(uint16_t pulse);    // Raw pulse (1000-2000)
void Servo_SetAngle(uint8_t angle_deg); // 0-180°
void Servo_MoveTo(uint8_t position);    // 0=laser1, 1=laser2
void Servo_Disable(void);               // Stop PWM (servo relaxes)

#endif // SERVO_H
