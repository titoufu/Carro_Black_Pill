#ifndef NRF24_H
#define NRF24_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

// ======= SPI handle (gerado pelo Cube) =======
extern SPI_HandleTypeDef hspi1;

// ======= Endereços (5 bytes) =======
extern const uint8_t NRF_ADDR_TX[5];   // definido em nrf24.c

// ======= REGISTERS =======
#define NRF_REG_CONFIG       0x00
#define NRF_REG_EN_AA        0x01
#define NRF_REG_EN_RXADDR    0x02
#define NRF_REG_SETUP_AW     0x03
#define NRF_REG_SETUP_RETR   0x04
#define NRF_REG_RF_CH        0x05
#define NRF_REG_RF_SETUP     0x06
#define NRF_REG_STATUS       0x07
#define NRF_REG_RX_ADDR_P0   0x0A
#define NRF_REG_TX_ADDR      0x10
#define NRF_REG_RX_PW_P0     0x11
#define NRF_REG_FIFO_STATUS  0x17
#define NRF_REG_DYNPD        0x1C
#define NRF_REG_FEATURE      0x1D

// ======= COMMANDS =======
#define NRF_CMD_R_REGISTER     0x00
#define NRF_CMD_W_REGISTER     0x20
#define NRF_CMD_R_RX_PAYLOAD   0x61
#define NRF_CMD_W_TX_PAYLOAD   0xA0
#define NRF_CMD_FLUSH_TX       0xE1
#define NRF_CMD_FLUSH_RX       0xE2
#define NRF_CMD_REUSE_TX_PL    0xE3
#define NRF_CMD_ACTIVATE       0x50
#define NRF_CMD_W_ACK_PAYLOAD  0xA8  // | pipe#
#define NRF_CMD_NOP            0xFF

// ======= CONFIG bits =======
#define CONFIG_MASK_RX_DR  (1<<6)
#define CONFIG_MASK_TX_DS  (1<<5)
#define CONFIG_MASK_MAX_RT (1<<4)
#define CONFIG_EN_CRC      (1<<3)
#define CONFIG_CRCO        (1<<2)
#define CONFIG_PWR_UP      (1<<1)
#define CONFIG_PRIM_RX     (1<<0)

// ======= API =======
#ifdef __cplusplus
extern "C" {
#endif

void    nrf_init_tx(uint8_t channel, bool rate250kbps, uint8_t paLevel);
void    nrf_init_rx(uint8_t channel, bool rate250kbps, uint8_t paLevel);

bool    nrf_send(const void* data, uint8_t len, uint32_t timeoutMs);
bool    nrf_has_rx(void);
uint8_t nrf_read_rx(void* buf, uint8_t maxlen);

void    nrf_set_ack_payload(const void* data, uint8_t len);
void    nrf_powerdown(void);
uint8_t nrf_read_register(uint8_t reg);
void    nrf_write_register(uint8_t reg, uint8_t val);


#ifdef __cplusplus
}
#endif

#endif // NRF24_H
