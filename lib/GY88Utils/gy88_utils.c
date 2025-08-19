#include "gy88_utils.h"

// -------------------- Internos --------------------
static inline float _deg(float rad)      { return rad * 57.2957795f; }   // 180/pi
static inline float _rad(float deg)      { return deg * 0.0174532925f; } // pi/180
static inline float _safe_sqrt(float v)  { return (v > 0.0f) ? sqrtf(v) : 0.0f; }

// -------------------- API --------------------
void gy88u_default_cfg(gy88u_cfg_t* cfg, gy88_mag_type_t mag_type)
{
  cfg->acc_g_per_lsb    = 1.0f / 16384.0f; // ±2g
  cfg->gyro_dps_per_lsb = 1.0f / 131.0f;   // ±250 dps

  cfg->mag_offset_x = cfg->mag_offset_y = cfg->mag_offset_z = 0.0f;
  cfg->mag_scale_x  = cfg->mag_scale_y  = cfg->mag_scale_z  = 1.0f;

  // Conversão µT/LSB
  if (mag_type == GY88_MAG_HMC5883L) {
    cfg->mag_uT_per_lsb = 100.0f / 1090.0f; // ≈ 0.09174 µT/LSB @ gain 0xA0
  } else {
    // QMC5883L: depende do setup. Comece em 0.1 e ajuste após calibração
    cfg->mag_uT_per_lsb = 0.10f;
  }

  cfg->sea_level_pa = 101325.0f;
}

void gy88u_apply_mag_calib(const gy88u_cfg_t* cfg,
                           int16_t mx, int16_t my, int16_t mz,
                           float* mx_uT, float* my_uT, float* mz_uT)
{
  float x = ((float)mx) * cfg->mag_uT_per_lsb;
  float y = ((float)my) * cfg->mag_uT_per_lsb;
  float z = ((float)mz) * cfg->mag_uT_per_lsb;

  // offset & scale
  x = (x - cfg->mag_offset_x) * cfg->mag_scale_x;
  y = (y - cfg->mag_offset_y) * cfg->mag_scale_y;
  z = (z - cfg->mag_offset_z) * cfg->mag_scale_z;

  *mx_uT = x; *my_uT = y; *mz_uT = z;
}

void gy88u_compute_rp_from_accel(float ax_g, float ay_g, float az_g,
                                 float* roll_deg, float* pitch_deg)
{
  // Convenção comum:
  // roll  = atan2( Ay, Az )
  // pitch = atan2( -Ax, sqrt(Ay^2 + Az^2) )
  float roll  = atan2f(ay_g, az_g);
  float pitch = atan2f(-ax_g, _safe_sqrt(ay_g*ay_g + az_g*az_g));
  *roll_deg  = _deg(roll);
  *pitch_deg = _deg(pitch);
}

float gy88u_heading_deg_tiltcomp(float mx, float my, float mz,
                                 float roll_deg, float pitch_deg)
{
  // Compensação de inclinação para heading:
  float roll  = _rad(roll_deg);
  float pitch = _rad(pitch_deg);

  // Rotaciona medidas de M para o plano horizontal
  // Ref típica:
  // Xh = Mx*cos(pitch) + Mz*sin(pitch)
  // Yh = Mx*sin(roll)*sin(pitch) + My*cos(roll) - Mz*sin(roll)*cos(pitch)
  float Xh = mx * cosf(pitch) + mz * sinf(pitch);
  float Yh = mx * sinf(roll)  * sinf(pitch) + my * cosf(roll) - mz * sinf(roll) * cosf(pitch);

  float heading = atan2f(Yh, Xh); // [-pi, pi]
  float hdg_deg = _deg(heading);
  if (hdg_deg < 0.0f) hdg_deg += 360.0f;
  return hdg_deg;
}

float gy88u_altitude_from_pressure(float pressure_pa, float sea_level_pa)
{
  // Fórmula barométrica padrão (ISA)
  // h = 44330 * (1 - (P/Po)^(1/5.255))
  float ratio = pressure_pa / sea_level_pa;
  return 44330.0f * (1.0f - powf(ratio, 0.190294957f)); // 1/5.255 = 0.19029...
}

void gy88u_from_raw(const gy88_t* dev, const gy88_raw_t* r,
                    const gy88u_cfg_t* cfg, gy88u_data_t* out)
{
  (void)dev; // não precisamos de info extra aqui por enquanto

  // Acel em g
  out->ax_g = r->ax * cfg->acc_g_per_lsb;
  out->ay_g = r->ay * cfg->acc_g_per_lsb;
  out->az_g = r->az * cfg->acc_g_per_lsb;

  // Giro em °/s
  out->gx_dps = r->gx * cfg->gyro_dps_per_lsb;
  out->gy_dps = r->gy * cfg->gyro_dps_per_lsb;
  out->gz_dps = r->gz * cfg->gyro_dps_per_lsb;

  // Mag em µT (com offset/escala)
  gy88u_apply_mag_calib(cfg, r->mx, r->my, r->mz, &out->mx_uT, &out->my_uT, &out->mz_uT);

  // T/P e altitude
  out->temperature_c = r->temperature_c;
  out->pressure_pa   = (float)r->pressure_pa;
  out->altitude_m    = gy88u_altitude_from_pressure(out->pressure_pa, cfg->sea_level_pa);

  // Atitudes (roll/pitch do accel; heading do mag compensado)
  gy88u_compute_rp_from_accel(out->ax_g, out->ay_g, out->az_g, &out->roll_deg, &out->pitch_deg);
  out->heading_deg = gy88u_heading_deg_tiltcomp(out->mx_uT, out->my_uT, out->mz_uT,
                                                out->roll_deg, out->pitch_deg);
}

int gy88u_read_and_convert(gy88_t* dev, const gy88u_cfg_t* cfg, gy88u_data_t* out)
{
  gy88_raw_t r;
  if (gy88_read_all(dev, &r) != 0) return -1;
  gy88u_from_raw(dev, &r, cfg, out);
  return 0;
}
