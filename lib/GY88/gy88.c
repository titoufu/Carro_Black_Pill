#include "gy88.h"
#include "gy88_conf.h"

int gy88_init(gy88_t* g) {
  // MPU6050 (0x68 by default)
  if (mpu6050_init(&g->mpu, GY88_I2C, MPU6050_I2C_ADDR) != 0) return -1;

  // Magnetometer
  g->mag_type = GY88_MAG_NONE;
#if GY88_AUTO_DETECT_MAG
  if (hmc5883l_detect(&g->hmc, GY88_I2C, HMC5883L_I2C_ADDR) == 0) {
    if (hmc5883l_init(&g->hmc) != 0) return -2;
    g->mag_type = GY88_MAG_HMC5883L;
  } else {
    // try QMC5883L at 0x0D
    if (qmc5883l_init(&g->qmc, GY88_I2C, QMC5883L_I2C_ADDR) == 0) {
      g->mag_type = GY88_MAG_QMC5883L;
    } else {
      g->mag_type = GY88_MAG_NONE;
    }
  }
#else
  // Force HMC first; if fail, try QMC
  if (hmc5883l_init(&g->hmc) == 0) g->mag_type = GY88_MAG_HMC5883L;
  else if (qmc5883l_init(&g->qmc, GY88_I2C, QMC5883L_I2C_ADDR) == 0) g->mag_type = GY88_MAG_QMC5883L;
#endif

  // BMP085 (oss=0)
  if (bmp085_init(&g->bmp, GY88_I2C, BMP085_I2C_ADDR, 0) != 0) return -3;

  return 0;
}

int gy88_read_all(gy88_t* g, gy88_raw_t* out) {
  if (mpu6050_read_raw(&g->mpu, &out->ax, &out->ay, &out->az,
                                  &out->gx, &out->gy, &out->gz,
                                  &out->temp_raw) != 0) return -1;

  switch (g->mag_type) {
    case GY88_MAG_HMC5883L:
      if (hmc5883l_read_raw(&g->hmc, &out->mx, &out->my, &out->mz) != 0) return -2;
      break;
    case GY88_MAG_QMC5883L:
      if (qmc5883l_read_raw(&g->qmc, &out->mx, &out->my, &out->mz) != 0) return -3;
      break;
    default:
      out->mx=out->my=out->mz=0;
      break;
  }

  bmp085_data_t bmd;
  if (bmp085_read(&g->bmp, &bmd) != 0) return -4;
  out->temperature_c = bmd.temperature_c;
  out->pressure_pa   = bmd.pressure_pa;

  return 0;
}