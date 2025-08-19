#ifndef QMC5883L_H
#define QMC5883L_H
#include "main.h"
#include "gy88_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { I2C_HandleTypeDef* i2c; uint16_t addr; } qmc5883l_t;

int qmc5883l_init(qmc5883l_t* qmc, I2C_HandleTypeDef* i2c, uint16_t addr);
int qmc5883l_read_raw(qmc5883l_t* qmc, int16_t* mx, int16_t* my, int16_t* mz);

#ifdef __cplusplus
}
#endif
#endif