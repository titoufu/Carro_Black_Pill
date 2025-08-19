#ifndef GY88_UTILS_H
#define GY88_UTILS_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Dependências (sua lib base do módulo + math)
#include "gy88.h"
#include <math.h>

// -------------------- Config e calibração --------------------
typedef struct {
  // Escalas do MPU6050 (ajuste se mudar o range no ACCEL_CONFIG / GYRO_CONFIG)
  // Padrão: ±2g (16384 LSB/g) e ±250 dps (131 LSB/(°/s))
  float acc_g_per_lsb;     // 1.0f / 16384.0f
  float gyro_dps_per_lsb;  // 1.0f / 131.0f

  // Calibração de magnetômetro (offset e escala por eixo)
  // Valores iniciais "neutros": offset=0, scale=1
  float mag_offset_x, mag_offset_y, mag_offset_z;
  float mag_scale_x,  mag_scale_y,  mag_scale_z;

  // Conversão de LSB para microTesla (µT/LSB)
  // HMC5883L @ gain 0xA0 ≈ 100 µT / 1090 LSB ≈ 0.09174f µT/LSB
  // QMC5883L varia por config; comece com 0.10f e ajuste após calibração
  float mag_uT_per_lsb;

  // Pressão de referência ao nível do mar (Pa) p/ altitude
  // Padrão: 101325 Pa
  float sea_level_pa;
} gy88u_cfg_t;

// Dados já convertidos
typedef struct {
  // Aceleração em g
  float ax_g, ay_g, az_g;

  // Velocidade angular em °/s
  float gx_dps, gy_dps, gz_dps;

  // Campo magnético em µT (após offset/scale)
  float mx_uT, my_uT, mz_uT;

  // Temperatura / Pressão / Altitude
  float temperature_c;
  float pressure_pa;
  float altitude_m;

  // Atitudes
  float roll_deg;   // a partir do acelerômetro
  float pitch_deg;  // a partir do acelerômetro
  float heading_deg; // bússola compensada por inclinação
} gy88u_data_t;

// -------------------- API --------------------

// Popular cfg com valores padrão sensatos conforme o tipo de magnetômetro detectado
void gy88u_default_cfg(gy88u_cfg_t* cfg, gy88_mag_type_t mag_type);

// Converte "raw" (gy88_read_all) -> unidades físicas + atitudes
void gy88u_from_raw(const gy88_t* dev, const gy88_raw_t* r,
                    const gy88u_cfg_t* cfg, gy88u_data_t* out);

// Le e já converte (chama gy88_read_all internamente)
int gy88u_read_and_convert(gy88_t* dev, const gy88u_cfg_t* cfg, gy88u_data_t* out);

// Helpers de atitudes
void gy88u_compute_rp_from_accel(float ax_g, float ay_g, float az_g,
                                 float* roll_deg, float* pitch_deg);

float gy88u_heading_deg_tiltcomp(float mx, float my, float mz,
                                 float roll_deg, float pitch_deg);

// Altitude padrão (barométrica)
float gy88u_altitude_from_pressure(float pressure_pa, float sea_level_pa);

// Calibração simples do magnetômetro (aplicar offset e escala)
void gy88u_apply_mag_calib(const gy88u_cfg_t* cfg,
                           int16_t mx_counts, int16_t my_counts, int16_t mz_counts,
                           float* mx_uT, float* my_uT, float* mz_uT);

#ifdef __cplusplus
}
#endif
#endif
