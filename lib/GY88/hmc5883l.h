#ifndef HMC5883L_H
#define HMC5883L_H
#include "main.h"
#include "gy88_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { I2C_HandleTypeDef* i2c; uint16_t addr; } hmc5883l_t;

int hmc5883l_detect(hmc5883l_t* hmc, I2C_HandleTypeDef* i2c, uint16_t addr);
int hmc5883l_init(hmc5883l_t* hmc);
int hmc5883l_read_raw(hmc5883l_t* hmc, int16_t* mx, int16_t* my, int16_t* mz);

#ifdef __cplusplus
}
#endif
#endif