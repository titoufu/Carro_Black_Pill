#include "hmc5883l.h"
#include "i2c_utils.h"

int hmc5883l_detect(hmc5883l_t* hmc, I2C_HandleTypeDef* i2c, uint16_t addr) {
  hmc->i2c=i2c; hmc->addr=addr;
  uint8_t a=0,b=0,c=0;
  if (i2c_mem_read8(i2c, addr, 0x0A, &a)!=HAL_OK) return -1;
  if (i2c_mem_read8(i2c, addr, 0x0B, &b)!=HAL_OK) return -1;
  if (i2c_mem_read8(i2c, addr, 0x0C, &c)!=HAL_OK) return -1;
  return (a=='H' && b=='4' && c=='3') ? 0 : -2;
}

int hmc5883l_init(hmc5883l_t* hmc) {
  // CRA=0x70 (8 samples avg, 15Hz), CRB=0xA0 (gain), MODE=0x00 (continuous)
  if (i2c_mem_write8(hmc->i2c, hmc->addr, 0x00, 0x70)!=HAL_OK) return -1;
  if (i2c_mem_write8(hmc->i2c, hmc->addr, 0x01, 0xA0)!=HAL_OK) return -2;
  if (i2c_mem_write8(hmc->i2c, hmc->addr, 0x02, 0x00)!=HAL_OK) return -3;
  HAL_Delay(10);
  return 0;
}

int hmc5883l_read_raw(hmc5883l_t* hmc, int16_t* mx, int16_t* my, int16_t* mz) {
  uint8_t b[6];
  if (i2c_mem_read(hmc->i2c, hmc->addr, 0x03, b, 6)!=HAL_OK) return -1;
  // Order HMC: X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
  *mx = (int16_t)((b[0]<<8)|b[1]);
  *mz = (int16_t)((b[2]<<8)|b[3]);
  *my = (int16_t)((b[4]<<8)|b[5]);
  return 0;
}