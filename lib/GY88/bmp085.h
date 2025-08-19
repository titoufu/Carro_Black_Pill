#ifndef BMP085_H
#define BMP085_H
#include "main.h"
#include "gy88_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  I2C_HandleTypeDef* i2c;
  uint16_t addr;
  int16_t AC1, AC2, AC3, B1, B2, MB, MC, MD;
  uint16_t AC4, AC5, AC6;
  uint8_t oss; // oversampling (0..3)
} bmp085_t;

typedef struct {
  float temperature_c;
  int32_t pressure_pa;
} bmp085_data_t;

int  bmp085_init(bmp085_t* bmp, I2C_HandleTypeDef* i2c, uint16_t addr, uint8_t oss);
int  bmp085_read(bmp085_t* bmp, bmp085_data_t* out); // blocking read (temp + pressure)

#ifdef __cplusplus
}
#endif
#endif