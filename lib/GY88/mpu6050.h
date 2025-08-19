#ifndef MPU6050_H
#define MPU6050_H
#include "main.h"
#include "gy88_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_REG_WHOAMI        0x75
#define MPU6050_REG_PWR_MGMT1     0x6B
#define MPU6050_REG_ACCEL_XOUT_H  0x3B  // lê 14 bytes
#define MPU6050_REG_INT_PIN_CFG   0x37  // aqui fica o I2C_BYPASS_EN

typedef struct {
  I2C_HandleTypeDef* i2c;
  uint16_t addr;
  uint8_t  whoami;
} mpu6050_t;

int mpu6050_init(mpu6050_t* mpu, I2C_HandleTypeDef* i2c, uint16_t addr);
int mpu6050_read_raw(mpu6050_t* mpu,
                     int16_t* ax, int16_t* ay, int16_t* az,
                     int16_t* gx, int16_t* gy, int16_t* gz,
                     int16_t* temp);

/* opcional: caso queira chamar à parte */
int mpu6050_enable_bypass(mpu6050_t* mpu, uint8_t enable);

#ifdef __cplusplus
}
#endif
#endif
