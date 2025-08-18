#include "main.h"
#include <cstdio> // std::snprintf

extern "C"
{
#include "ssd1306.h"
#include "ssd1306_fonts.h"
  // Handles com linkage C (a lib SSD1306 é C)
  I2C_HandleTypeDef hi2c2;
  TIM_HandleTypeDef htim3; // ENB (PB4, TIM3_CH1)
  TIM_HandleTypeDef htim4; // ENA (PB9, TIM4_CH4)
}

#include "motor.hpp" // classe Motor (construtor com 8 parâmetros)

// ===== Prototypes =====
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void Error_Handler(void);

// ===== PWM base 20 kHz (TIMCLK=84 MHz) =====
#define PWM_ARR (4199U) // 84e6 / (1*(4199+1)) = 20 kHz

// ===== Ponteiros globais p/ facilitar chamadas no loop =====
static Motor *gMotorA = nullptr;
static Motor *gMotorB = nullptr;

// ===== OLED status (opcional) =====
static void OLED_UpdateStatus(uint8_t dutyA, uint8_t dutyB, Motor::Direction dirA, Motor::Direction dirB)
{
  static uint32_t last = 0;
  static bool beat = false;
  uint32_t now = HAL_GetTick();
  if (now - last < 250)
    return; // 4 Hz
  last = now;
  beat = !beat;

  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"BlackPill F411", Font_7x10, White);
  ssd1306_SetCursor(0, 12);
  ssd1306_WriteString((char *)"Motors + OLED", Font_7x10, White);

  char line[22];
  ssd1306_SetCursor(0, 26);
  std::snprintf(line, sizeof(line), "A:%s %3u%%", (dirA == Motor::Direction::Forward ? "FWD" : "REV"), dutyA);
  ssd1306_WriteString((char *)line, Font_7x10, White);

  ssd1306_SetCursor(0, 38);
  std::snprintf(line, sizeof(line), "B:%s %3u%%", (dirB == Motor::Direction::Forward ? "FWD" : "REV"), dutyB);
  ssd1306_WriteString((char *)line, Font_7x10, White);

  ssd1306_SetCursor(118, 0);
  ssd1306_WriteString((char *)(beat ? "*" : " "), Font_7x10, White);

  ssd1306_UpdateScreen();
}

// ===== Padrão de teste dos motores (não-bloqueante) =====
// 4 fases de 5 s: 0) A/B FWD 90%, 1) A/B REV 90%, 2) A FWD & B REV 90%, 3) A REV & B FWD 90%
static void Motors_TestPattern(void)
{
  static int phase = 0;
  static uint32_t t0 = 0;
  const uint32_t PHASE_MS = 5000;

  uint32_t now = HAL_GetTick();
  if (now - t0 >= PHASE_MS)
  {
    t0 = now;
    phase = (phase + 1) & 3;
  }

  Motor::Direction dirA = Motor::Direction::Forward;
  Motor::Direction dirB = Motor::Direction::Forward;
  uint8_t dutyA = 90, dutyB = 90;

  switch (phase)
  {
  case 0:
    dirA = Motor::Direction::Forward;
    dirB = Motor::Direction::Forward;
    break;
  case 1:
    dirA = Motor::Direction::Backward;
    dirB = Motor::Direction::Backward;
    break;
  case 2:
    dirA = Motor::Direction::Forward;
    dirB = Motor::Direction::Backward;
    break;
  default:
    dirA = Motor::Direction::Backward;
    dirB = Motor::Direction::Forward;
    break;
  }

  gMotorA->start(dirA, dutyA); // setDirection + setSpeed
  gMotorB->start(dirB, dutyB);

  OLED_UpdateStatus(dutyA, dutyB, dirA, dirB); // comente se não quiser OLED agora
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init(); // PB8..PB5 como GPIO Output (IN1..IN4 em 0)
  MX_I2C2_Init(); // PB10/PB3
  MX_TIM3_Init(); // TIM3_CH1 @ PB4
  MX_TIM4_Init(); // TIM4_CH4 @ PB9

  // ===== OLED Hello =====
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"Init...", Font_7x10, White);
  ssd1306_UpdateScreen();

  // ===== Instancia os motores =====
  // Motor A: ENA = TIM4_CH4 (PB9), IN1=PB8, IN2=PB7
  static Motor motorA(&htim4, TIM_CHANNEL_4,
                      GPIOB, GPIO_PIN_8,
                      GPIOB, GPIO_PIN_7,
                      PWM_ARR, false); // invert=false
  // Motor B: ENB = TIM3_CH1 (PB4), IN3=PB6, IN4=PB5
  static Motor motorB(&htim3, TIM_CHANNEL_1,
                      GPIOB, GPIO_PIN_6,
                      GPIOB, GPIO_PIN_5,
                      PWM_ARR, false);

  gMotorA = &motorA;
  gMotorB = &motorB;

  // Inicia PWM (CCR=0) e coloca INs conforme direção default (Forward)
  gMotorA->begin();
  gMotorB->begin();
  gMotorA->coast();
  gMotorB->coast();

  while (1)
  {
    // Comente/ative o que quiser testar:
    Motors_TestPattern(); // padrão automático (frente/ré/cruzado)

    // Exemplo de comando direto (comente o padrão acima para testar manual):
    // gMotorA->start(Motor::Direction::Forward, 90);
    // gMotorB->start(Motor::Direction::Forward, 90);

    HAL_Delay(5);
  }
}

/* ================= Clock: HSI->PLL @84MHz ================= */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2; // PCLK1=42; timers APB1=84
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2; // PCLK2=42
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ================= I2C2 (PB10 SCL AF4, PB3 SDA AF9) ================= */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000; // 100kHz (pode 400kHz)
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ================= TIM3: PWM CH1 @20kHz (ENB PB4) ================= */
static void MX_TIM3_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_ARR;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
}

/* ================= TIM4: PWM CH4 @20kHz (ENA PB9) ================= */
static void MX_TIM4_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = PWM_ARR;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

/* ================= GPIO: IN1..IN4 (PB8..PB5) ================= */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_8 | GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
}

/* ================= Error handler ================= */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
