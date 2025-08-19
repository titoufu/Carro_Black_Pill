#ifndef GY88_CONF_H
#define GY88_CONF_H

#include "main.h"
/* Torna o handle visível para C e C++ */
#ifdef __cplusplus
extern "C" {
#endif
extern I2C_HandleTypeDef hi2c2;
#ifdef __cplusplus
}
#endif

// Use the same I2C used in your project (OLED): hi2c2 (PB10/PB3)
#define GY88_I2C                (&hi2c2)

// 7-bit addresses shifted for HAL (<<1)
#define MPU6050_I2C_ADDR        (0x68 << 1)   // or (0x69<<1) if AD0 pulled high
#define HMC5883L_I2C_ADDR       (0x1E << 1)
#define QMC5883L_I2C_ADDR       (0x0D << 1)
#define BMP085_I2C_ADDR         (0x77 << 1)

// Try to auto-detect magnetometer: HMC5883L vs QMC5883L
#define GY88_AUTO_DETECT_MAG    1

#endif