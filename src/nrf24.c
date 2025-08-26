#include "nrf24.h"

// Use os nomes que o Cube gera:
static inline void CE_H(void){ HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_SET); }
static inline void CE_L(void){ HAL_GPIO_WritePin(NRF_CE_GPIO_Port,  NRF_CE_Pin,  GPIO_PIN_RESET); }
static inline void CSN_H(void){ HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET); }
static inline void CSN_L(void){ HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET); }

const uint8_t NRF_ADDR_TX[5] = {'R','C','0','0','1'};

static uint8_t spi_txrx(uint8_t v){
  uint8_t r=0;
  HAL_SPI_TransmitReceive(&hspi1, &v, &r, 1, HAL_MAX_DELAY);
  return r;
}

static uint8_t read_reg(uint8_t reg){
  CSN_L();
  spi_txrx(NRF_CMD_R_REGISTER | (reg & 0x1F));
  uint8_t v = spi_txrx(0xFF);
  CSN_H();
  return v;
}

static void write_reg(uint8_t reg, uint8_t val){
  CSN_L();
  spi_txrx(NRF_CMD_W_REGISTER | (reg & 0x1F));
  spi_txrx(val);
  CSN_H();
}

static void write_buf(uint8_t reg, const uint8_t* buf, uint8_t len){
  CSN_L();
  spi_txrx(NRF_CMD_W_REGISTER | (reg & 0x1F));
  HAL_SPI_Transmit(&hspi1, (uint8_t*)buf, len, HAL_MAX_DELAY);
  CSN_H();
}

static void cmd(uint8_t c){
  CSN_L(); spi_txrx(c); CSN_H();
}

static void feature_enable_dynpayload(void){
  // Alguns chips precisam do ACTIVATE 0x73
  CSN_L(); spi_txrx(NRF_CMD_ACTIVATE); spi_txrx(0x73); CSN_H();
  write_reg(NRF_REG_FEATURE, 0x06); // EN_DPL | EN_ACK_PAY
  write_reg(NRF_REG_DYNPD,   0x01); // DPL_P0
}

static void rf_setup(bool lowRate250kbps, uint8_t paLevel){
  // RF_SETUP: [7]CONT_WAVE [6]RF_DR_LOW [5]PLL_LOCK [3]RF_DR_HIGH [2:1]RF_PWR [0]LNA_HCURR
  uint8_t v = 0x01; // LNA_HCURR=1
  if(lowRate250kbps){ v |= (1<<5); } // RF_DR_LOW=1
  else { v |= (1<<3); }              // 1Mbps: RF_DR_HIGH=1? (p/ 1Mbps, DR_HIGH=0 e DR_LOW=0; se setar HIGH=1 dá 2Mbps) -> use 250kbps para robustez
  // PA level (00=-18dBm, 01=-12, 10=-6, 11=0)
  if(paLevel>3) paLevel=3;
  v |= (paLevel<<1);
  write_reg(NRF_REG_RF_SETUP, v & ~(1<<3)); // força 1Mbps/250kbps
}

static void common_setup(uint8_t channel, bool lowRate250kbps, uint8_t paLevel){
  CE_L();
  HAL_Delay(5);
  write_reg(NRF_REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO); // CRC 2B, PWR_UP=0
  write_reg(NRF_REG_EN_AA, 0x01);       // Auto-ACK pipe0
  write_reg(NRF_REG_EN_RXADDR, 0x01);   // Enable pipe0
  write_reg(NRF_REG_SETUP_AW, 0x03);    // 5-byte addr
  write_reg(NRF_REG_SETUP_RETR, (3<<4)|5); // 3*250us delay, 5 retries
  write_reg(NRF_REG_RF_CH, channel);    // 0..125
  rf_setup(lowRate250kbps, paLevel);

  write_buf(NRF_REG_TX_ADDR, NRF_ADDR_TX, 5);
  write_buf(NRF_REG_RX_ADDR_P0, NRF_ADDR_TX, 5);
  feature_enable_dynpayload();

  cmd(NRF_CMD_FLUSH_RX);
  cmd(NRF_CMD_FLUSH_TX);
  write_reg(NRF_REG_STATUS, 0x70); // limpa RX_DR, TX_DS, MAX_RT
}

void nrf_init_tx(uint8_t channel, bool lowRate250kbps, uint8_t paLevel){
  common_setup(channel, lowRate250kbps, paLevel);
  // Power up TX
  write_reg(NRF_REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP); // PRIM_RX=0
  HAL_Delay(5);
  CE_L();
}

void nrf_init_rx(uint8_t channel, bool lowRate250kbps, uint8_t paLevel){
  common_setup(channel, lowRate250kbps, paLevel);
  // payload dinâmico no pipe0 já habilitado
  write_reg(NRF_REG_CONFIG, CONFIG_EN_CRC | CONFIG_CRCO | CONFIG_PWR_UP | CONFIG_PRIM_RX);
  HAL_Delay(5);
  CE_H();
}

bool nrf_send(const void* data, uint8_t len, uint32_t timeoutMs){
  if(len==0) return false;
  if(len>32) len=32;

  // Vai garantir modo TX (CE low)
  CE_L();
  cmd(NRF_CMD_FLUSH_TX);
  write_reg(NRF_REG_STATUS, 0x70);

  // Carrega payload
  CSN_L();
  spi_txrx(NRF_CMD_W_TX_PAYLOAD);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);
  CSN_H();

  // Pulso CE >=10us
  CE_H(); HAL_Delay(1); CE_L();

  uint32_t t0 = HAL_GetTick();
  while((HAL_GetTick()-t0) < timeoutMs){
    uint8_t st = read_reg(NRF_REG_STATUS);
    if(st & (1<<5)){ // TX_DS
      write_reg(NRF_REG_STATUS, (1<<5));
      return true;
    }
    if(st & (1<<4)){ // MAX_RT
      write_reg(NRF_REG_STATUS, (1<<4));
      cmd(NRF_CMD_FLUSH_TX);
      return false;
    }
  }
  return false;
}

bool nrf_has_rx(void){
  uint8_t st = read_reg(NRF_REG_STATUS);
  return (st & (1<<6)) != 0; // RX_DR
}

uint8_t nrf_read_rx(void* buf, uint8_t maxlen){
  if(maxlen<1) return 0;
  CSN_L();
  spi_txrx(NRF_CMD_R_RX_PAYLOAD);
  HAL_SPI_Receive(&hspi1, (uint8_t*)buf, maxlen, HAL_MAX_DELAY);
  CSN_H();
  write_reg(NRF_REG_STATUS, (1<<6)); // limpa RX_DR
  return maxlen; // com DPL não sabemos tamanho real sem R_RX_PL_WID; simples: use tamanho fixo no teste
}

void nrf_set_ack_payload(const void* data, uint8_t len){
  if(len>32) len=32;
  CSN_L();
  spi_txrx(NRF_CMD_W_ACK_PAYLOAD | 0x00); // pipe 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)data, len, HAL_MAX_DELAY);
  CSN_H();
}

void nrf_powerdown(void){
  uint8_t c = read_reg(NRF_REG_CONFIG);
  c &= ~CONFIG_PWR_UP;
  write_reg(NRF_REG_CONFIG, c);
  CE_L();
}
uint8_t nrf_read_register(uint8_t reg) {
    uint8_t cmd = NRF_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t val = 0xFF;
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &val, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
    return val;
}
void nrf_write_register(uint8_t reg, uint8_t val){
    uint8_t cmd = NRF_CMD_W_REGISTER | (reg & 0x1F);
    CSN_L();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    HAL_SPI_Transmit(&hspi1, &val, 1, HAL_MAX_DELAY);
    CSN_H();
}
