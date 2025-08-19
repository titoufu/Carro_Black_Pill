#ifndef GY88_H
#define GY88_H
#include "main.h"
#include "gy88_conf.h"
#include "mpu6050.h"
#include "hmc5883l.h"
#include "qmc5883l.h"
#include "bmp085.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  GY88_MAG_NONE = 0,
  GY88_MAG_HMC5883L,
  GY88_MAG_QMC5883L
} gy88_mag_type_t;

typedef struct {
  mpu6050_t    mpu;
  hmc5883l_t   hmc;
  qmc5883l_t   qmc;
  bmp085_t     bmp;
  gy88_mag_type_t mag_type;
} gy88_t;

typedef struct {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t mx, my, mz;
  int16_t temp_raw;     // from MPU6050
  float   temperature_c; // from BMP085
  int32_t pressure_pa;   // from BMP085
} gy88_raw_t;

int gy88_init(gy88_t* g);
int gy88_read_all(gy88_t* g, gy88_raw_t* out);

#ifdef __cplusplus
}
#endif
#endif