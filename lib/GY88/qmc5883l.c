#include "qmc5883l.h"
#include "i2c_utils.h"

int qmc5883l_init(qmc5883l_t* qmc, I2C_HandleTypeDef* i2c, uint16_t addr) {
  qmc->i2c=i2c; qmc->addr=addr;
  // Set/Reset period and continuous mode
  if (i2c_mem_write8(i2c, addr, 0x0B, 0x01)!=HAL_OK) return -1; // Set/Reset period
  // OSR=512 (00), RNG=2G (01), ODR=50Hz (10), MODE=cont(01) -> 0x1D
  if (i2c_mem_write8(i2c, addr, 0x09, 0x1D)!=HAL_OK) return -2;
  HAL_Delay(10);
  return 0;
}

int qmc5883l_read_raw(qmc5883l_t* qmc, int16_t* mx, int16_t* my, int16_t* mz) {
  uint8_t b[6];
  if (i2c_mem_read(qmc->i2c, qmc->addr, 0x00, b, 6)!=HAL_OK) return -1;
  // QMC: X L, X H, Y L, Y H, Z L, Z H
  *mx = (int16_t)((b[1]<<8)|b[0]);
  *my = (int16_t)((b[3]<<8)|b[2]);
  *mz = (int16_t)((b[5]<<8)|b[4]);
  return 0;
}