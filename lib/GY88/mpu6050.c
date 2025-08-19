#include "mpu6050.h"
#include "i2c_utils.h"

static int _enable_bypass(mpu6050_t *mpu, uint8_t enable)
{
  uint8_t v = 0;
  if (i2c_mem_read8(mpu->i2c, mpu->addr, MPU6050_REG_INT_PIN_CFG, &v) != HAL_OK)
    return -1;
  if (enable)
    v |= 0x02; // bit1 = I2C_BYPASS_EN
  else
    v &= (uint8_t)~0x02;
  if (i2c_mem_write8(mpu->i2c, mpu->addr, MPU6050_REG_INT_PIN_CFG, v) != HAL_OK)
    return -2;
  HAL_Delay(2);
  return 0;
}

int mpu6050_enable_bypass(mpu6050_t *mpu, uint8_t enable)
{
  return _enable_bypass(mpu, enable);
}

int mpu6050_init(mpu6050_t *mpu, I2C_HandleTypeDef *i2c, uint16_t addr)
{
  mpu->i2c = i2c;
  mpu->addr = addr;
  uint8_t id = 0;
  if (i2c_mem_read8(i2c, addr, MPU6050_REG_WHOAMI, &id) != HAL_OK)
    return -1;
  mpu->whoami = id; // esperado 0x68 (0x69 se AD0=1)

  // Wake-up
  if (i2c_mem_write8(i2c, addr, MPU6050_REG_PWR_MGMT1, 0x00) != HAL_OK)
    return -2;
  HAL_Delay(10);
  
  // ACCEL_CONFIG (0x1C): ±2g => 0x00
  i2c_mem_write8(i2c, addr, 0x1C, 0x00);
  // GYRO_CONFIG  (0x1B): ±250 dps => 0x00
  i2c_mem_write8(i2c, addr, 0x1B, 0x00);
  HAL_Delay(2);

  // *** habilita o BYPASS para expor o magnetômetro no barramento principal ***
  if (_enable_bypass(mpu, 1) != 0)
    return -3;

  return 0;
}

int mpu6050_read_raw(mpu6050_t *mpu,
                     int16_t *ax, int16_t *ay, int16_t *az,
                     int16_t *gx, int16_t *gy, int16_t *gz,
                     int16_t *temp)
{
  uint8_t buf[14];
  if (i2c_mem_read(mpu->i2c, mpu->addr, MPU6050_REG_ACCEL_XOUT_H, buf, 14) != HAL_OK)
    return -1;

  *ax = (int16_t)((buf[0] << 8) | buf[1]);
  *ay = (int16_t)((buf[2] << 8) | buf[3]);
  *az = (int16_t)((buf[4] << 8) | buf[5]);
  *temp = (int16_t)((buf[6] << 8) | buf[7]);
  *gx = (int16_t)((buf[8] << 8) | buf[9]);
  *gy = (int16_t)((buf[10] << 8) | buf[11]);
  *gz = (int16_t)((buf[12] << 8) | buf[13]);
  return 0;
}
