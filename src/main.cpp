/**
 * Projeto: BlackPill F411 + L298N + OLED SSD1306 + IMU GY-88
 * Arquivo: src/main.cpp
 *
 * Propósito:
 *  - Orquestrar inicialização de clock, GPIO, I2C2, TIM3/TIM4 (PWM)
 *  - Inicializar e exibir telas no OLED (via wrapper Display/UI)
 *  - Ler amostras do GY-88 (via wrapper IMU)
 *  - Acionar motores A/B com padrão cíclico (a→b→c→d) sem bloquear o loop
 *
 * Observação:
 *  - Toda a “lógica de controle” estão nas libs (Display/UI/IMU/Motor);
 *    aqui apenas coordenamos as tarefas para manter o main limpo.
 */

#include "main.h"
#include "project_config.hpp"   // flags/feature toggles (USE_OLED/USE_IMU/USE_MOTORS)

#include <cstdio>
#include <cstdlib>
#include <cmath>

#if USE_OLED
  #include "display.hpp"        // wrapper C++ fino p/ drivers C do SSD1306
  #include "status_screens.hpp" // UI padronizada (linhas/formatos prontos)
#endif

#if USE_IMU
  #include "imu.hpp"            // wrapper C++ p/ GY-88 (mpu/hmc/qmc/bmp)
#endif

#if USE_MOTORS
  #include "motor.hpp"          // a classe Motor (habilita begin/start/…)
  #include "motor_config.hpp"   // mapeamento de pinos/AF/canais (derivado do Cube)
#endif

// ===== Prototypes gerados/adaptados do Cube =====


void SystemClock_Config(void);
static void MX_GPIO_Init(void);  // INs PB8..PB5 (push-pull), ENA/ENB em AF, I2C2 PB10/PB3
static void MX_I2C2_Init(void);  // I2C2 a 100kHz, 7-bit
static void MX_TIM3_Init(void);  // ENB -> TIM3 (ex.: CH1)
static void MX_TIM4_Init(void);  // ENA -> TIM4 (ex.: CH4)
void Error_Handler(void);

// ===== Handles globais =====
// Mantêm a compatibilidade com drivers C (SSD1306/GY-88) e com HAL.
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3, htim4;

#if USE_MOTORS
// ===== Objetos Motor =====
// Construtores usam os macros do motor_config.hpp (pinos/AF/timer/channel).
Motor motorA(MOTOR_A_TIMER_HANDLE, MOTOR_A_CHANNEL,
             MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
             MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
             MOTOR_PWM_MAX,   MOTOR_A_INVERT);

Motor motorB(MOTOR_B_TIMER_HANDLE, MOTOR_B_CHANNEL,
             MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
             MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
             MOTOR_PWM_MAX,   MOTOR_B_INVERT);

/**
 * @brief Atualiza padrão de direção dos motores A/B (a→b→c→d) sem bloquear.
 * @param dutyA   Duty do motor A (0..100)
 * @param dutyB   Duty do motor B (0..100)
 * @param fase_ms Duração de cada fase (ms)
 *
 * Fases:
 *   0) A:FWD, B:FWD
 *   1) A:REV, B:REV
 *   2) A:FWD, B:REV
 *   3) A:REV, B:FWD
 *
 * Implementação usa millis (HAL_GetTick) e estado estático local.
 */
static void Motors_Pattern_Update(uint8_t dutyA, uint8_t dutyB, uint32_t fase_ms)
{
  using Dir = Motor::Direction;

  static uint32_t t0   = 0;  // instante da última troca
  static uint8_t  fase = 0;  // 0..3

  const uint32_t now = HAL_GetTick();
  if ((now - t0) >= fase_ms) {
    t0   = now;
    fase = (uint8_t)((fase + 1) & 0x03);

    switch (fase) {
      case 0: motorA.start(Dir::Forward,  dutyA); motorB.start(Dir::Forward,  dutyB); break;
      case 1: motorA.start(Dir::Backward, dutyA); motorB.start(Dir::Backward, dutyB); break;
      case 2: motorA.start(Dir::Forward,  dutyA); motorB.start(Dir::Backward, dutyB); break;
      default:motorA.start(Dir::Backward, dutyA); motorB.start(Dir::Forward,  dutyB); break;
    }
  }
}
#endif // USE_MOTORS

int main(void)
{
  // ===== Sequência padrão de boot HAL =====
  HAL_Init();             // reset periféricos, Systick etc.
  SystemClock_Config();   // HSI->PLL @84 MHz (latência 2)
  MX_GPIO_Init();         // INs/ENs/I2C2 nos modos corretos
  MX_I2C2_Init();         // I2C2 p/ OLED e GY-88
  MX_TIM3_Init();         // PWM p/ ENB (Motor B)
  MX_TIM4_Init();         // PWM p/ ENA (Motor A)

#if USE_OLED
  // ===== OLED =====
  display::init();                       // ssd1306_Init + limpar tela
  display::splash("BlackPill F411",      // splash inicial
                  "OLED/IMU/Motors");
#endif

#if USE_IMU
  // ===== IMU (GY-88) =====
  if (imu::init()) {
  #if USE_OLED
    display::splash("GY-88", "OK");      // sinaliza IMU ok
  #endif
  } else {
  #if USE_OLED
    display::splash("GY-88", "FAIL");    // sinaliza falha de init
  #endif
  }
#endif

#if USE_MOTORS
  // ===== Motores =====
  // Importante: start do PWM antes de usar a classe Motor
  HAL_TIM_PWM_Start(&htim4, MOTOR_A_CHANNEL); // ENA PB9
  HAL_TIM_PWM_Start(&htim3, MOTOR_B_CHANNEL); // ENB PB4

  motorA.begin();  // zera duty/canais, prepara INs
  motorB.begin();
  motorA.coast();  // começa solto (sem torque)
  motorB.coast();
#endif

  // ===== Loop principal =====
  // Mantém delays curtos p/ responsividade (OLED/IMU) e padrão dos motores.
  while (1)
  {
  #if USE_IMU && USE_OLED
    // Leitura única da IMU e exibição de uma tela (ex.: ACC).
    // Altere para ui::showGyro/ showMag / showBaro conforme desejado.
    const auto sample = imu::read();
    ui::showAccel(sample);
  #endif

  #if USE_MOTORS
    // Padrão cíclico a→b→c→d com 5s por fase (@90% aqui p/ validação)
    Motors_Pattern_Update(/*dutyA=*/90, /*dutyB=*/90, /*fase_ms=*/5000);
  #endif

    HAL_Delay(100); // intervalo curto: deixa I2C/PWM/loop responsivos
  }
}

/* ===== Clock (HSI->PLL @84 MHz) =====
 * Mantém a mesma configuração testada por você.
 * APB1 em /2 garante timers de APB1 a 84 MHz (duplicação interna).
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM            = 16;
  RCC_OscInitStruct.PLL.PLLN            = 168;
  RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType =
    RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    // timers APB1 a 84 MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* ===== GPIO =====
 * - INs do L298N: PB8..PB5 em saída push-pull (iniciam em 0 p/ evitar tranco)
 * - ENA (PB9)  AF2 TIM4_CH4
 * - ENB (PB4)  AF2 TIM3_CH1
 * - I2C2 (PB10/PB3) em AF open-drain (pull-ups no módulo)
 */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // INs em 0 (estado seguro no boot)
  HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);

  GPIO_InitTypeDef g = {0};
  g.Pin   = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN; // PB8,7,6,5
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  // ENA (PB9) -> TIM4_CH4 AF2
  g.Pin       = MOTOR_A_EN_PIN;
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = MOTOR_A_EN_AF;
  HAL_GPIO_Init(MOTOR_A_EN_PORT, &g);

  // ENB (PB4) -> TIM3_CH1 AF2
  g.Pin       = MOTOR_B_EN_PIN;
  g.Alternate = MOTOR_B_EN_AF;
  HAL_GPIO_Init(MOTOR_B_EN_PORT, &g);

  // I2C2 PB10 (SCL AF4) / PB3 (SDA AF9) – open-drain, pull-ups no módulo OLED/IMU
  GPIO_InitTypeDef gi2c = {0};
  gi2c.Mode  = GPIO_MODE_AF_OD;
  gi2c.Pull  = GPIO_NOPULL; // (mantemos OFF; se precisar, habilite no Cube ou HW)
  gi2c.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gi2c.Pin = GPIO_PIN_10; gi2c.Alternate = GPIO_AF4_I2C2; HAL_GPIO_Init(GPIOB, &gi2c);
  gi2c.Pin = GPIO_PIN_3;  gi2c.Alternate = GPIO_AF9_I2C2; HAL_GPIO_Init(GPIOB, &gi2c);
}

/* ===== I2C2 @100kHz =====
 * Usado pelo SSD1306 e pelo GY-88 (MPU6050/HMC/QMC/BMP085)
 */
static void MX_I2C2_Init(void)
{
  __HAL_RCC_I2C2_CLK_ENABLE();

  hi2c2.Instance             = I2C2;
  hi2c2.Init.ClockSpeed      = 100000;
  hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1     = 0;
  hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2     = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

/* ===== TIM3: PWM p/ ENB (Motor B) =====
 * Period = MOTOR_PWM_MAX (deriva de motor_config.hpp)
 * Modo: PWM1, polaridade HIGH, prescaler 0 (base @84 MHz APB1*2)
 */
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance               = TIM3;
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

/* ===== TIM4: PWM p/ ENA (Motor A) ===== */
static void MX_TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim4.Instance               = TIM4;
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

// ===== Error handler simples =====
void Error_Handler(void)
{
  __disable_irq();
  while (1) { /* travado p/ diagnóstico */ }
}
