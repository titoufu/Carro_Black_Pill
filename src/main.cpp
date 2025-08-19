#include "main.h"
#include "motor_config.hpp"
#include "motor.hpp"
#include <cstdio>
#include <cstdlib>
#include <cmath>

extern "C"
{
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "gy88.h"
#include "gy88_utils.h"
}

/* Prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void); // ENB (APB1)
static void MX_TIM4_Init(void); // ENA (APB1)
void Error_Handler(void);

/* Handles */
extern "C"
{
  I2C_HandleTypeDef hi2c2;
}

/* IMU (uma única definição!) */
static gy88_t gIMU;
static bool gIMU_OK = false;
static gy88u_cfg_t gCFG;
// Handles dos timers (expostos pelo seu Cube)
TIM_HandleTypeDef htim3; // ENB -> TIM3 (ex.: CH1)
TIM_HandleTypeDef htim4; // ENA -> TIM4 (ex.: CH4)

// Motores usando os macros do motor_config.hpp
Motor motorA(MOTOR_A_TIMER_HANDLE, MOTOR_A_CHANNEL,
             MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
             MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_A_INVERT);

Motor motorB(MOTOR_B_TIMER_HANDLE, MOTOR_B_CHANNEL,
             MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
             MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_B_INVERT);

/* ================== TESTES NO OLED ================== */
// ---- formata float sem %f (fixo, com sinal) ----
static inline void fmt_signed_fp(char *buf, size_t n, float v, int decs = 2)
{
  char sign = (v < 0.0f) ? '-' : '+';
  if (v < 0.0f)
    v = -v;

  int pow10 = 1;
  for (int i = 0; i < decs; ++i)
    pow10 *= 10;

  int scaled = (int)(v * pow10 + 0.5f);
  int whole = scaled / pow10;
  int frac = scaled % pow10;

  if (decs <= 0)
  {
    std::snprintf(buf, n, "%c%d", sign, whole);
  }
  else
  {
    // largura dinâmica no fractional: %0*d evita o aviso do compilador
    std::snprintf(buf, n, "%c%d.%0*d", sign, whole, decs, frac);
  }
}

static void OLED_Test_Accel(void)
{
  if (!gIMU_OK)
    return;
  gy88u_data_t d;
  if (gy88u_read_and_convert(&gIMU, &gCFG, &d) != 0)
    return;

  char ln[24], v[24];
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"ACC (g)", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  fmt_signed_fp(v, sizeof(v), d.ax_g, 2);
  std::snprintf(ln, sizeof(ln), "Ax:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  fmt_signed_fp(v, sizeof(v), d.ay_g, 2);
  std::snprintf(ln, sizeof(ln), "Ay:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  fmt_signed_fp(v, sizeof(v), d.az_g, 2);
  std::snprintf(ln, sizeof(ln), "Az:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 48);
  float mod = std::sqrt(d.ax_g * d.ax_g + d.ay_g * d.ay_g + d.az_g * d.az_g);
  fmt_signed_fp(v, sizeof(v), mod, 2);
  std::snprintf(ln, sizeof(ln), "|A|:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}
static void OLED_Test_Gyro(void)
{
  if (!gIMU_OK)
    return;
  gy88u_data_t d;
  if (gy88u_read_and_convert(&gIMU, &gCFG, &d) != 0)
    return;

  char ln[24], v[24];
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"GYRO (dps)", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  fmt_signed_fp(v, sizeof(v), d.gx_dps, 1);
  std::snprintf(ln, sizeof(ln), "Gx:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  fmt_signed_fp(v, sizeof(v), d.gy_dps, 1);
  std::snprintf(ln, sizeof(ln), "Gy:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  fmt_signed_fp(v, sizeof(v), d.gz_dps, 1);
  std::snprintf(ln, sizeof(ln), "Gz:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 48);
  fmt_signed_fp(v, sizeof(v), d.roll_deg, 1);
  std::snprintf(ln, sizeof(ln), "r:%s ", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);
  fmt_signed_fp(v, sizeof(v), d.pitch_deg, 1);
  std::snprintf(ln, sizeof(ln), "p:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}
static void OLED_Test_Mag(void)
{
  if (!gIMU_OK)
    return;
  gy88u_data_t d;
  if (gy88u_read_and_convert(&gIMU, &gCFG, &d) != 0)
    return;

  char ln[24], v[24];
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"MAG (uT)", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  fmt_signed_fp(v, sizeof(v), d.mx_uT, 2);
  std::snprintf(ln, sizeof(ln), "Mx:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  fmt_signed_fp(v, sizeof(v), d.my_uT, 2);
  std::snprintf(ln, sizeof(ln), "My:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  fmt_signed_fp(v, sizeof(v), d.mz_uT, 2);
  std::snprintf(ln, sizeof(ln), "Mz:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 48);
  fmt_signed_fp(v, sizeof(v), d.heading_deg, 0); // inteiro
  std::snprintf(ln, sizeof(ln), "HDG:%s", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}
static void OLED_Test_Baro(void)
{
  if (!gIMU_OK)
    return;
  gy88u_data_t d;
  if (gy88u_read_and_convert(&gIMU, &gCFG, &d) != 0)
    return;

  char ln[24], v[24];
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"BARO", Font_7x10, White);

  ssd1306_SetCursor(0, 12);
  fmt_signed_fp(v, sizeof(v), d.temperature_c, 1);
  std::snprintf(ln, sizeof(ln), "T:%sC", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 24);
  int p_int = (int)(d.pressure_pa + 0.5f); // inteiro em Pa
  std::snprintf(ln, sizeof(ln), "P:%7dPa", p_int);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_SetCursor(0, 36);
  fmt_signed_fp(v, sizeof(v), d.altitude_m, 1);
  std::snprintf(ln, sizeof(ln), "ALT:%sm", v);
  ssd1306_WriteString((char *)ln, Font_7x10, White);

  ssd1306_UpdateScreen();
}
static void Motors_Pattern_Update(uint8_t dutyA, uint8_t dutyB, uint32_t fase_ms)
{
  using Dir = Motor::Direction;
  static uint32_t t0 = 0;
  static uint8_t fase = 0; // 0=a, 1=b, 2=c, 3=d

  uint32_t now = HAL_GetTick();
  if ((now - t0) >= fase_ms)
  {
    t0 = now;
    fase = (uint8_t)((fase + 1) & 0x03);

    switch (fase)
    {
    case 0: // (a) ambos para frente
      motorA.start(Dir::Forward, dutyA);
      motorB.start(Dir::Forward, dutyB);
      break;
    case 1: // (b) ambos em ré
      motorA.start(Dir::Backward, dutyA);
      motorB.start(Dir::Backward, dutyB);
      break;
    case 2: // (c) A frente, B ré
      motorA.start(Dir::Forward, dutyA);
      motorB.start(Dir::Backward, dutyB);
      break;
    default: // (d) A ré, B frente
      motorA.start(Dir::Backward, dutyA);
      motorB.start(Dir::Forward, dutyB);
      break;
    }
  }
}

/* ================== MAIN ================== */

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();

  // MOTORES
  MX_TIM3_Init();
  MX_TIM4_Init();

  HAL_TIM_PWM_Start(&htim4, MOTOR_A_CHANNEL); // ENA PB9
  HAL_TIM_PWM_Start(&htim3, MOTOR_B_CHANNEL); // ENB PB4

  motorA.begin();
  motorB.begin();

  // OLED
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  HAL_Delay(50);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString((char *)"Init OLED", Font_7x10, White);
  ssd1306_UpdateScreen();
  HAL_Delay(300);

  // IMU
  if (gy88_init(&gIMU) == 0)
  {
    gIMU_OK = true;
    gy88u_default_cfg(&gCFG, gIMU.mag_type); // escalas e µT/LSB default
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString((char *)"GY-88 OK", Font_7x10, White);
    const char *magtxt =
        (gIMU.mag_type == GY88_MAG_HMC5883L) ? "MAG:HMC5883L" : (gIMU.mag_type == GY88_MAG_QMC5883L) ? "MAG:QMC5883L"
                                                                                                     : "MAG:none";
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString((char *)magtxt, Font_7x10, White);
    ssd1306_UpdateScreen();
  }
  else
  {
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString((char *)"GY-88 FAIL", Font_7x10, White);
    ssd1306_UpdateScreen();
  }

  while (1)
  {
    // Mostra uma das telas (ex.: acelerômetro); pode alternar como preferir
    OLED_Test_Accel(); // ou OLED_Test_Gyro / OLED_Test_Mag / OLED_Test_Baro

    // Atualiza o padrão dos motores (ciclo a,b,c,d)
    Motors_Pattern_Update(90, 90, 5000);

    HAL_Delay(100); // curto, mantém IMU/OLED responsivos e motor fluindo
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ===== GPIO (I2C2 PB10/PB3 OD AF) ===== */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // --- INs em saída push-pull e zerados ---
  HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);

  GPIO_InitTypeDef g = {0};
  g.Pin = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN; // PB8,7,6,5
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g); // (tudo na porta B)

  // --- ENA (PB9) como AF2 -> TIM4_CH4 ---
  g.Pin = MOTOR_A_EN_PIN; // PB9
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = MOTOR_A_EN_AF; // AF2
  HAL_GPIO_Init(MOTOR_A_EN_PORT, &g);

  // --- ENB (PB4) como AF2 -> TIM3_CH1 ---
  g.Pin = MOTOR_B_EN_PIN;      // PB4
  g.Alternate = MOTOR_B_EN_AF; // AF2
  HAL_GPIO_Init(MOTOR_B_EN_PORT, &g);

  // --- I2C2 (PB10/PB3) como você já tinha ---
  GPIO_InitTypeDef gi2c = {0};
  gi2c.Mode = GPIO_MODE_AF_OD;
  gi2c.Pull = GPIO_NOPULL; // pull-ups no módulo
  gi2c.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gi2c.Pin = GPIO_PIN_10;
  gi2c.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &gi2c);
  gi2c.Pin = GPIO_PIN_3;
  gi2c.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &gi2c);
}

/* ===== I2C2 (PB10/PB3) ===== */
static void MX_I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
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
/* ===== TIM3: PWM CH1 @20 kHz (Motor B) ===== */
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = MOTOR_PWM_MAX;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, MOTOR_B_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ===== TIM4: PWM CH4 @20 kHz (Motor A) ===== */
static void MX_TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MOTOR_PWM_MAX;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, MOTOR_A_CHANNEL) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
