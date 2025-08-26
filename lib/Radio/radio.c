#include "radio.h"
#include "radio_cfg.h"
#include "nrf24.h"
#include "spi.h"
#include <string.h>

// ---------- util ----------
static uint16_t crc16_xor(const void* data, uint16_t len){
  const uint8_t* p = (const uint8_t*)data;
  uint16_t c = 0;
  for (uint16_t i = 0; i < len; i++) c ^= p[i];
  return c;
}

static void nrf_cmd(uint8_t cmd){
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

// ---------- wrappers de burst ----------
void nrf_write_buf(uint8_t reg, const uint8_t *buf, uint8_t len)
{
  uint8_t cmd = NRF_CMD_W_REGISTER | (reg & 0x1F);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

void nrf_read_buf(uint8_t reg, uint8_t *buf, uint8_t len)
{
  uint8_t cmd = NRF_CMD_R_REGISTER | (reg & 0x1F);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, buf, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

// ---------- RX ----------
void radio_init_rx(void)
{
  // CE=LOW durante setup; CSN=HIGH em repouso
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
  HAL_Delay(2);

  // Base (canal, rate, PA)
  nrf_init_rx(RF_CHANNEL, RF_RATE_250KBPS, RF_PA_LEVEL);

  // Setup conhecido
  nrf_write_register(NRF_REG_STATUS,      0x70); // limpa IRQs
  nrf_write_register(NRF_REG_SETUP_AW,    0x03); // 5 bytes
  nrf_write_register(NRF_REG_EN_AA,       0x01); // AA pipe0
  nrf_write_register(NRF_REG_EN_RXADDR,   0x01); // habilita pipe0
  nrf_write_register(NRF_REG_RF_CH,       RF_CHANNEL);

  // 250 kbps (DR_LOW=1, DR_HIGH=0) — mantém PA da lib
  uint8_t rfsetup = nrf_read_register(NRF_REG_RF_SETUP);
  rfsetup &= ~(1u<<3); // DR_HIGH=0
  rfsetup |=  (1u<<5); // DR_LOW=1
  nrf_write_register(NRF_REG_RF_SETUP, rfsetup);

  // Payload fixo e sem DPL
  nrf_write_register(NRF_REG_FEATURE, 0x00);
  nrf_write_register(NRF_REG_DYNPD,   0x00);
  nrf_write_register(NRF_REG_RX_PW_P0, sizeof(joy_pkt_t));

  // Endereços (ACK volta ao mesmo address)
  nrf_write_buf(NRF_REG_RX_ADDR_P0, (uint8_t*)RF_ADDR, 5);
  nrf_write_buf(NRF_REG_TX_ADDR,    (uint8_t*)RF_ADDR, 5);

  // Garante FIFO RX limpo, zera flags e entra em PRIM_RX
  nrf_cmd(0xE2); // FLUSH_RX
  nrf_write_register(NRF_REG_STATUS, 0x70);

  nrf_write_register(NRF_REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP | CONFIG_PRIM_RX);
  HAL_Delay(3); // >= 2 ms até Standby-I
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

void radio_debug_summary(void)
{
  uint8_t cfg  = nrf_read_register(NRF_REG_CONFIG);
  uint8_t ch   = nrf_read_register(NRF_REG_RF_CH);
  uint8_t retr = nrf_read_register(NRF_REG_SETUP_RETR);
  uint8_t enaa = nrf_read_register(NRF_REG_EN_AA);
  uint8_t enrx = nrf_read_register(NRF_REG_EN_RXADDR);
  uint8_t aw   = nrf_read_register(NRF_REG_SETUP_AW);
  uint8_t st   = nrf_read_register(NRF_REG_STATUS);
  uint8_t tx[5], rx0[5];

  nrf_read_buf(NRF_REG_TX_ADDR,    tx,  5);
  nrf_read_buf(NRF_REG_RX_ADDR_P0, rx0, 5);

  printf("NRF -> CFG=0x%02X CH=0x%02X RETR=0x%02X EN_AA=0x%02X EN_RX=0x%02X AW=0x%02X ST=0x%02X\r\n",
         cfg, ch, retr, enaa, enrx, aw, st);
  printf("ADDR TX=%02X %02X %02X %02X %02X  RX0=%02X %02X %02X %02X %02X\r\n",
         tx[0],tx[1],tx[2],tx[3],tx[4], rx0[0],rx0[1],rx0[2],rx0[3],rx0[4]);
}

bool nrf_check_rx(joy_pkt_t *out)
{
  if (!nrf_has_rx()) return false;

  joy_pkt_t pkt;
  (void)nrf_read_rx(&pkt, sizeof(pkt));

  const uint16_t expect = crc16_xor(&pkt, sizeof(pkt) - sizeof(pkt.crc));
  if (expect != pkt.crc) return false;

  if (out) *out = pkt;
  return true;
}
