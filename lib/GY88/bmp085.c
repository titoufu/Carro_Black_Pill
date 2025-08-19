#include "bmp085.h"
#include "i2c_utils.h"

static int read_coeffs(bmp085_t* b) {
  uint8_t buf[22];
  if (HAL_OK != HAL_I2C_Mem_Read(b->i2c, b->addr, 0xAA, I2C_MEMADD_SIZE_8BIT, buf, 22, 100)) return -1;
  b->AC1 = (int16_t)((buf[0]<<8)|buf[1]);
  b->AC2 = (int16_t)((buf[2]<<8)|buf[3]);
  b->AC3 = (int16_t)((buf[4]<<8)|buf[5]);
  b->AC4 = (uint16_t)((buf[6]<<8)|buf[7]);
  b->AC5 = (uint16_t)((buf[8]<<8)|buf[9]);
  b->AC6 = (uint16_t)((buf[10]<<8)|buf[11]);
  b->B1  = (int16_t)((buf[12]<<8)|buf[13]);
  b->B2  = (int16_t)((buf[14]<<8)|buf[15]);
  b->MB  = (int16_t)((buf[16]<<8)|buf[17]);
  b->MC  = (int16_t)((buf[18]<<8)|buf[19]);
  b->MD  = (int16_t)((buf[20]<<8)|buf[21]);
  return 0;
}

int bmp085_init(bmp085_t* bmp, I2C_HandleTypeDef* i2c, uint16_t addr, uint8_t oss) {
  bmp->i2c = i2c; bmp->addr = addr; bmp->oss = (oss>3)?3:oss;
  if (read_coeffs(bmp) != 0) return -1;
  return 0;
}

int bmp085_read(bmp085_t* b, bmp085_data_t* out) {
  // Start temperature conversion
  if (i2c_mem_write8(b->i2c, b->addr, 0xF4, 0x2E) != HAL_OK) return -1;
  HAL_Delay(5); // 4.5ms typ
  uint8_t tbuf[2];
  if (HAL_OK != HAL_I2C_Mem_Read(b->i2c, b->addr, 0xF6, I2C_MEMADD_SIZE_8BIT, tbuf, 2, 100)) return -2;
  int32_t UT = (int32_t)((tbuf[0]<<8) | tbuf[1]);

  // Start pressure conversion
  uint8_t cmd = 0x34 + (b->oss<<6);
  if (i2c_mem_write8(b->i2c, b->addr, 0xF4, cmd) != HAL_OK) return -3;
  // wait according to OSS
  switch (b->oss) {
    case 0: HAL_Delay(5);  break; // 4.5ms
    case 1: HAL_Delay(8);  break; // 7.5ms
    case 2: HAL_Delay(14); break; // 13.5ms
    case 3: HAL_Delay(26); break; // 25.5ms
  }
  uint8_t pbuf[3] = {0};
  if (HAL_OK != HAL_I2C_Mem_Read(b->i2c, b->addr, 0xF6, I2C_MEMADD_SIZE_8BIT, pbuf, 3, 100)) return -4;
  int32_t UP = ((int32_t)pbuf[0] << 16 | (int32_t)pbuf[1] << 8 | (int32_t)pbuf[2]) >> (8 - b->oss);

  // Compensation (datasheet)
  int32_t X1 = ((UT - (int32_t)b->AC6) * (int32_t)b->AC5) >> 15;
  int32_t X2 = ((int32_t)b->MC << 11) / (X1 + b->MD);
  int32_t B5 = X1 + X2;
  int32_t T = (B5 + 8) >> 4; // 0.1°C
  out->temperature_c = T / 10.0f;

  int32_t B6 = B5 - 4000;
  X1 = ((int32_t)b->B2 * ((B6 * B6) >> 12)) >> 11;
  X2 = ((int32_t)b->AC2 * B6) >> 11;
  int32_t X3 = X1 + X2;
  int32_t B3 = (((((int32_t)b->AC1) * 4 + X3) << b->oss) + 2) >> 2;

  X1 = ((int32_t)b->AC3 * B6) >> 13;
  X2 = ((int32_t)b->B1 * ((B6 * B6) >> 12)) >> 16;
  X3 = ((X1 + X2) + 2) >> 2;
  uint32_t B4 = ((uint32_t)b->AC4 * (uint32_t)(X3 + 32768)) >> 15;
  uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (uint32_t)(50000 >> b->oss);

  int32_t p;
  if (B7 < 0x80000000) p = (int32_t)((B7 << 1) / B4);
  else                 p = (int32_t)((B7 / B4) << 1);

  X1 = (p >> 8) * (p >> 8);
  X1 = (X1 * 3038) >> 16;
  X2 = (-7357 * p) >> 16;
  p = p + ((X1 + X2 + 3791) >> 4);
  out->pressure_pa = p;

  return 0;
}