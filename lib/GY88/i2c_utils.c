#include "i2c_utils.h"

HAL_StatusTypeDef i2c_mem_read8(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t *val) {
  return HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, val, 1, 100);
}
HAL_StatusTypeDef i2c_mem_write8(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t val) {
  return HAL_I2C_Mem_Write(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 100);
}
HAL_StatusTypeDef i2c_mem_read(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}