#include "main.h"
#include "motor_config.hpp"
#include "motor.hpp"

/* Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);   // usa só macros do motor_config.hpp
static void MX_TIM3_Init(void);   // ENB (APB1)
static void MX_TIM4_Init(void);   // ENA (APB1)
void Error_Handler(void);

/* Handles (visíveis no motor_config.hpp) */
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* Objetos Motor com pinos da config */
Motor motorA(MOTOR_A_TIMER_HANDLE, MOTOR_A_CHANNEL,
             MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
             MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_A_INVERT);

Motor motorB(MOTOR_B_TIMER_HANDLE, MOTOR_B_CHANNEL,
             MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
             MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_B_INVERT);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  motorA.begin();
  motorB.begin();

  /* ---- Loop de teste parametrizado (5 s cada fase) ---- */
  const uint8_t DUTY_A_FWD = 90, DUTY_A_REV = 90;
  const uint8_t DUTY_B_FWD = 90, DUTY_B_REV = 90;

  while (1) {
    // 1) Ambos frente
    motorA.start(Motor::Direction::Forward,  DUTY_A_FWD);
    motorB.start(Motor::Direction::Forward,  DUTY_B_FWD);
    HAL_Delay(5000);

    // 2) Ambos ré
    motorA.start(Motor::Direction::Backward, DUTY_A_REV);
    motorB.start(Motor::Direction::Backward, DUTY_B_REV);
    HAL_Delay(5000);

    // 3) A frente, B ré
    motorA.start(Motor::Direction::Forward,  DUTY_A_FWD);
    motorB.start(Motor::Direction::Backward, DUTY_B_REV);
    HAL_Delay(5000);

    // 4) A ré, B frente
    motorA.start(Motor::Direction::Backward, DUTY_A_REV);
    motorB.start(Motor::Direction::Forward,  DUTY_B_FWD);
    HAL_Delay(5000);
  }
}

/* ===== Clock (HSI->PLL @84 MHz) ===== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // timers APB1 = 84 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ===== GPIO via config (sem literais) ===== */
static void MX_GPIO_Init(void)
{
  // Clocks das portas usadas (B e C no seu setup; ajuste se trocar porta)
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  // INs todos como saída e em 0 (evita tranco no boot)
  HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);

  g.Pin   = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);  // (se trocar de porta, separe por PORT)

  // ENA como AF (ex.: PB9/AF2/TIM4_CH4)
  g.Pin       = MOTOR_A_EN_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = MOTOR_A_EN_AF;
  HAL_GPIO_Init(MOTOR_A_EN_PORT, &g);

  // ENB como AF (ex.: PB4/AF2/TIM3_CH1)
  g.Pin       = MOTOR_B_EN_PIN;
  g.Alternate = MOTOR_B_EN_AF;
  HAL_GPIO_Init(MOTOR_B_EN_PORT, &g);
}

/* ===== TIM3: PWM CH1 @20 kHz (Motor B) ===== */
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler         = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.Period            = MOTOR_PWM_MAX;
  htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, MOTOR_B_CHANNEL) != HAL_OK) { Error_Handler(); }
}

/* ===== TIM4: PWM CH4 @20 kHz (Motor A) ===== */
static void MX_TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler         = 0;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.Period            = MOTOR_PWM_MAX;
  htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode     = TIM_OCMODE_PWM1;
  oc.Pulse      = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, MOTOR_A_CHANNEL) != HAL_OK) { Error_Handler(); }
}

/* ===== Error handler ===== */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
