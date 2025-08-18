#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
}
#endif

/* Handles dos timers (definidos no main.cpp) */
extern TIM_HandleTypeDef htim3;   // ENB
extern TIM_HandleTypeDef htim4;   // ENA

/* PWM base: 20 kHz com TIMCLK=84 MHz (APB1 timers)  -> PSC=0, ARR=4199 */
#define MOTOR_PWM_MAX  (4199U)

/* ======================= Motor A ======================= */
/* ENA -> timer/canal e pino AF */
#define MOTOR_A_TIMER_HANDLE   (&htim4)
#define MOTOR_A_CHANNEL        TIM_CHANNEL_4
#define MOTOR_A_EN_PORT        GPIOB
#define MOTOR_A_EN_PIN         GPIO_PIN_9
#define MOTOR_A_EN_AF          GPIO_AF2_TIM4

/* IN1/IN2 (GPIOs) */
#define MOTOR_A_IN1_PORT       GPIOB
#define MOTOR_A_IN1_PIN        GPIO_PIN_8
#define MOTOR_A_IN2_PORT       GPIOB
#define MOTOR_A_IN2_PIN        GPIO_PIN_7

/* Inverter sentido lógico (0 normal, 1 inverte) */
#define MOTOR_A_INVERT         0

/* ======================= Motor B ======================= */
/* ENB -> timer/canal e pino AF */
#define MOTOR_B_TIMER_HANDLE   (&htim3)
#define MOTOR_B_CHANNEL        TIM_CHANNEL_1
#define MOTOR_B_EN_PORT        GPIOB
#define MOTOR_B_EN_PIN         GPIO_PIN_4
#define MOTOR_B_EN_AF          GPIO_AF2_TIM3

/* IN3/IN4 (GPIOs) */
#define MOTOR_B_IN1_PORT       GPIOB   /* IN3 */
#define MOTOR_B_IN1_PIN        GPIO_PIN_6
#define MOTOR_B_IN2_PORT       GPIOB   /* IN4 */
#define MOTOR_B_IN2_PIN        GPIO_PIN_5

#define MOTOR_B_INVERT         0
