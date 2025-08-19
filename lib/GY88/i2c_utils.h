#ifndef I2C_UTILS_H
#define I2C_UTILS_H
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

HAL_StatusTypeDef i2c_mem_read8 (I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t *val);
HAL_StatusTypeDef i2c_mem_write8(I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t  val);
HAL_StatusTypeDef i2c_mem_read  (I2C_HandleTypeDef* hi2c, uint16_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif