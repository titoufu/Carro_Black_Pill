/**
 * Projeto Carro Rx_Mtor — BlackPill F411 (STM32F411CE)
 * Subsistemas: Motores (L298N), OLED SSD1306 (I2C2), IMU GY-88 (I2C2), Rádio nRF24 (SPI1), UART1 logs
 *
 * Este arquivo faz o bring-up e o loop principal.
 * A lógica de cada subsistema mora em lib/ (Motor, Display, IMU, Rádio).
 *
 * Flags de build (project_config.hpp):
 *   USE_OLED, USE_IMU, USE_MOTORS, USE_RADIO, USE_RADIO_TEST
 *   - USE_RADIO_TEST=1 mantém o loop focado só no rádio (diagnóstico/integração).
 */

#include "main.h"
#include "project_config.hpp"

// === C headers (printf / setvbuf / stdout) ===
#include <stdio.h>
#include <stdint.h>
#include <math.h>

// === HAL/Cube drivers ===
#include "usart.h"   // MX_USART1_UART_Init, huart1
#include "spi.h"     // MX_SPI1_Init, hspi1, spi1_safe_setup

// I2C/Timers: forward-declared e implementados neste arquivo
// (se você usa as versões geradas pelo Cube em outros .c, remova as impls daqui)
void        SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
void        Error_Handler(void);

// === Drivers/Libs opcionais ===
#if USE_OLED
  #include "display.hpp"
  #include "status_screens.hpp"
#endif
#if USE_IMU
  #include "imu.hpp"
#endif
#if USE_MOTORS
  #include "motor.hpp"
  #include "motor_config.hpp"
#endif
#if USE_RADIO
extern "C" {
  #include "radio_cfg.h"
  #include "radio.h"
  #include "nrf24.h"  // NRF_REG_* e nrf_read_register()/nrf_has_rx()
}
#endif

// ========= Retarget do printf (UART1) =========
extern UART_HandleTypeDef huart1;
extern "C" int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(&ch), 1, HAL_MAX_DELAY);
  return ch;
}
extern "C" int _write(int, char *ptr, int len) {
  HAL_UART_Transmit(&huart1, reinterpret_cast<uint8_t*>(ptr), len, HAL_MAX_DELAY);
  return len;
}

// ========= Handles globais (I2C2 / TIMs) =========
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim3, htim4;

// ========= Instâncias de motor =========
#if USE_MOTORS
Motor motorA(MOTOR_A_TIMER_HANDLE, MOTOR_A_CHANNEL,
             MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
             MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_A_INVERT);

Motor motorB(MOTOR_B_TIMER_HANDLE, MOTOR_B_CHANNEL,
             MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
             MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
             MOTOR_PWM_MAX, MOTOR_B_INVERT);
#endif

// ========= Utilitário: padrão placeholder de movimento =========
#if USE_MOTORS
static void Motors_Pattern_Update(uint8_t dutyA, uint8_t dutyB, uint32_t fase_ms)
{
  using Dir = Motor::Direction;
  static uint32_t t0 = 0;
  static uint8_t fase = 0;
  const uint32_t now = HAL_GetTick();

  if ((now - t0) >= fase_ms) {
    t0 = now;
    fase = (uint8_t)((fase + 1) & 0x03);
    switch (fase) {
      case 0: motorA.start(Dir::Forward, dutyA);  motorB.start(Dir::Forward, dutyB); break;
      case 1: motorA.start(Dir::Backward,dutyA);  motorB.start(Dir::Backward,dutyB); break;
      case 2: motorA.start(Dir::Forward, dutyA);  motorB.start(Dir::Backward,dutyB); break;
      default:motorA.start(Dir::Backward,dutyA);  motorB.start(Dir::Forward, dutyB); break;
    }
  }
}
#endif

// ========= Helpers do rádio (poll + telemetria) =========
#if USE_RADIO
static void Radio_PollOnce(void)
{
  joy_pkt_t pkt;
  if (nrf_check_rx(&pkt)) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    printf("RX seq=%lu  x=%u y=%u sw=%u  CRC=OK\r\n",
           (unsigned long)pkt.seq, pkt.x, pkt.y, pkt.sw);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

    // TODO (próximo passo): mapear pkt.x/pkt.y -> motores
  }
}

#ifndef NRF_REG_RPD
#define NRF_REG_RPD          0x09
#endif
#ifndef NRF_REG_FIFO_STATUS
#define NRF_REG_FIFO_STATUS  0x17
#endif
static void Radio_DebugTick(void)
{
  static uint32_t t0 = 0;
  uint32_t now = HAL_GetTick();
  if (now - t0 < 500) return;
  t0 = now;

  uint8_t st   = nrf_read_register(NRF_REG_STATUS);
  uint8_t fifo = nrf_read_register(NRF_REG_FIFO_STATUS);
  uint8_t rpd  = nrf_read_register(NRF_REG_RPD) & 1;
  int has      = nrf_has_rx();
  int ce       = (HAL_GPIO_ReadPin(NRF_CE_GPIO_Port,  NRF_CE_Pin ) == GPIO_PIN_SET);
  int csn      = (HAL_GPIO_ReadPin(NRF_CSN_GPIO_Port, NRF_CSN_Pin) == GPIO_PIN_SET);

  printf("ST=0x%02X  FIFO=0x%02X  RPD=%d  has_rx=%d  CE=%d CSN=%d\r\n",
         st, fifo, rpd, has, ce, csn);
}
#endif // USE_RADIO

// ============================================================================
//                                   main()
// Bring-up ordenado (por dependência de clock/barramento) + init modular.
// ============================================================================
int main(void)
{
  // 0) HAL + Clock raiz (84 MHz)
  HAL_Init();
  SystemClock_Config();

  // 1) GPIO base (níveis seguros): LED=OFF (PC13=HIGH), CE=LOW, CSN=HIGH, IN1..IN4=LOW
  MX_GPIO_Init();

  // 2) UART1 para logs e printf sem buffer
  MX_USART1_UART_Init();
  setvbuf(stdout, NULL, _IONBF, 0);

  // 3) SPI1 (nRF24) — precisa vir antes do rádio; ajusta prescaler/pulls
  MX_SPI1_Init();
  spi1_safe_setup();

  // 4) I2C2 (OLED/IMU) e Timers (PWM motores)
  MX_I2C2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  // 5) OLED — tela de apresentação
#if USE_OLED
  display::init();
  display::splash("BlackPill F411", "OLED/IMU/Motors");
#endif

  // 6) IMU (GY-88) — com feedback no OLED
#if USE_IMU
  if (imu::init()) {
    #if USE_OLED
      display::splash("GY-88", "OK");
    #endif
  } else {
    #if USE_OLED
      display::splash("GY-88", "FAIL");
    #endif
  }
#endif

  // 7) Motores — inicia PWM e coloca ambas as pontes em “coast”
#if USE_MOTORS
  HAL_TIM_PWM_Start(&htim4, MOTOR_A_CHANNEL); // ENA PB9 (TIM4_CH4)
  HAL_TIM_PWM_Start(&htim3, MOTOR_B_CHANNEL); // ENB PB4 (TIM3_CH1)
  motorA.begin(); motorB.begin();
  motorA.coast(); motorB.coast();
#endif

  // 8) Rádio — init consolidado + resumo (sanity check)
#if USE_RADIO
  printf("\r\n=== RX Joystick NRF24L01 ===\r\n");
  printf("HCLK=%lu Hz | UART1=115200\r\n", HAL_RCC_GetHCLKFreq());
  radio_init_rx();            // aplica radio_cfg.h e sobe CE
  radio_debug_summary();      // imprime CFG/CH/RF_SETUP/... e endereços
#endif

  // 9) Loop principal
  while (1)
  {
#if USE_RADIO && USE_RADIO_TEST
    // MODO DE TESTE DO RÁDIO: foca só no RX (diagnóstico/integração)
    Radio_PollOnce();         // imprime "RX seq=.. x=.. y=.. sw=.."
    Radio_DebugTick();        // STATUS/FIFO/RPD/has_rx/CE/CSN
    HAL_Delay(10);
    continue;                 // evita rodar demais tarefas neste modo
#endif

#if USE_IMU && USE_OLED
    const auto sample = imu::read();
    ui::showAccel(sample);    // exemplo de tela
#endif

#if USE_MOTORS
    Motors_Pattern_Update(90, 90, 5000); // placeholder até mapear joystick
#endif

    HAL_Delay(100);
  }
}

// ============================== Implementações Cube ==============================
// Ajuste conforme seu hardware/projeto. Estas versões funcionam com:
// - ENA PB9 (TIM4_CH4 AF2), ENB PB4 (TIM3_CH1 AF2)
// - IN1..IN4 em PB8..PB5 (GPIO)
// - I2C2 em PB10 (SCL AF4) / PB3 (SDA AF9)
// - LED PC13 (ativo em LOW)
// - nRF24: CE PB0, CSN PB1
// -----------------------------------------------------------------------------

/* Clock: HSI->PLL @84 MHz (latência 2WS) */
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* GPIO:
 * - INs L298N: PB8..PB5 -> saída LOW
 * - ENA PB9 (AF2 TIM4_CH4), ENB PB4 (AF2 TIM3_CH1)
 * - LED PC13 -> saída HIGH (apagado, já que é ativo em LOW)
 * - nRF24: CE PB0 (LOW), CSN PB1 (HIGH)
 * - I2C2: PB10 (SCL AF4) / PB3 (SDA AF9) em open-drain
 */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

#if USE_MOTORS
  // Níveis seguros para IN1..IN4 (pontes em coast)
  HAL_GPIO_WritePin(MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN, GPIO_PIN_RESET);

  GPIO_InitTypeDef g = {0};
  g.Pin   = MOTOR_A_IN1_PIN | MOTOR_A_IN2_PIN | MOTOR_B_IN1_PIN | MOTOR_B_IN2_PIN; // PB8..PB5
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);

  // ENA (PB9) -> TIM4_CH4 AF2
  g.Pin = MOTOR_A_EN_PIN;
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Alternate = MOTOR_A_EN_AF;
  HAL_GPIO_Init(MOTOR_A_EN_PORT, &g);

  // ENB (PB4) -> TIM3_CH1 AF2
  g.Pin = MOTOR_B_EN_PIN;
  g.Alternate = MOTOR_B_EN_AF;
  HAL_GPIO_Init(MOTOR_B_EN_PORT, &g);
#endif

  // LED PC13 (apagado no boot)
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  GPIO_InitTypeDef led = {0};
  led.Pin = LED_Pin;
  led.Mode = GPIO_MODE_OUTPUT_PP;
  led.Pull = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &led);

  // nRF24: CE PB0 (LOW), CSN PB1 (HIGH)
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
  GPIO_InitTypeDef rf = {0};
  rf.Pin = NRF_CE_Pin | NRF_CSN_Pin;
  rf.Mode = GPIO_MODE_OUTPUT_PP;
  rf.Pull = GPIO_NOPULL;
  rf.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &rf);

  // I2C2 pins são configurados no MX_I2C2_Init()
}

/* I2C2 @100kHz (PB10 SCL AF4 / PB3 SDA AF9) */
static void MX_I2C2_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  GPIO_InitTypeDef gi2c = {0};
  gi2c.Mode = GPIO_MODE_AF_OD;
  gi2c.Pull = GPIO_NOPULL;
  gi2c.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  gi2c.Pin = GPIO_PIN_10; gi2c.Alternate = GPIO_AF4_I2C2; HAL_GPIO_Init(GPIOB, &gi2c); // SCL
  gi2c.Pin = GPIO_PIN_3;  gi2c.Alternate = GPIO_AF9_I2C2; HAL_GPIO_Init(GPIOB, &gi2c); // SDA

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) { Error_Handler(); }
}

/* TIM3: PWM p/ ENB (PB4) */
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = MOTOR_PWM_MAX;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) { Error_Handler(); }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH; oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, MOTOR_B_CHANNEL) != HAL_OK) { Error_Handler(); }
}

/* TIM4: PWM p/ ENA (PB9) */
static void MX_TIM4_Init(void)
{
  __HAL_RCC_TIM4_CLK_ENABLE();
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MOTOR_PWM_MAX;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) { Error_Handler(); }

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode = TIM_OCMODE_PWM1; oc.Pulse = 0;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH; oc.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &oc, MOTOR_A_CHANNEL) != HAL_OK) { Error_Handler(); }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) { /* trap */ }
}
