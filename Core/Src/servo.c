/**
 * @file    servo.c
 * @brief   MG90 servo driver using TIM1_CH1 (PA8)
 *
 * TIM1 config: PSC=239 (1MHz tick), ARR=19999 (50Hz = 20ms period)
 * Pulse: 1000 = 1ms (0°), 1500 = 1.5ms (90°), 2000 = 2ms (180°)
 */
#include "servo.h"

extern TIM_HandleTypeDef htim1;

void Servo_Init(void) {
  // Start PWM on TIM1 CH1
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  // Start at neutral position
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_POS_NEUTRAL);
}

void Servo_SetPulse(uint16_t pulse) {
  // Clamp to safe range
  if (pulse < 500)
    pulse = 500;
  if (pulse > 2500)
    pulse = 2500;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pulse);
}

void Servo_SetAngle(uint8_t angle_deg) {
  if (angle_deg > 180)
    angle_deg = 180;
  // Map 0-180° to 1000-2000 pulse width
  uint16_t pulse = 1000 + ((uint32_t)angle_deg * 1000) / 180;
  Servo_SetPulse(pulse);
}

void Servo_MoveTo(uint8_t position) {
  if (position == 0) {
    Servo_SetPulse(SERVO_POS_LASER1);
  } else {
    Servo_SetPulse(SERVO_POS_LASER2);
  }
}

void Servo_Disable(void) { HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1); }
