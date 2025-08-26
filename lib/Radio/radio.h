#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Traz HAL; usa main.h se existir (labels CE/CSN), senão cai no HAL direto
#ifdef __has_include
  #if __has_include("main.h")
    #include "main.h"
  #else
    #include "stm32f4xx_hal.h"
  #endif
#else
  #include "stm32f4xx_hal.h"
#endif

// ---- Payload do joystick (12 bytes) ----
typedef struct __attribute__((packed)){
  uint16_t x;
  uint16_t y;
  uint8_t  sw;
  uint8_t  reserved;
  uint32_t seq;
  uint16_t crc;
} joy_pkt_t;

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
  _Static_assert(sizeof(joy_pkt_t) == 12, "joy_pkt_t deve ter 12 bytes");
#endif

// ---- API do módulo de rádio ----
void  radio_init_rx(void);               // aplica radio_cfg.h e sobe CE
void  radio_debug_summary(void);         // imprime registradores úteis
bool  nrf_check_rx(joy_pkt_t *out);      // leitura do FIFO + verificação CRC (app)

void  nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len);
void  nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len);

#ifdef __cplusplus
}
#endif
