#include "imu.hpp"

namespace imu {
static gy88_t     gIMU;
static gy88u_cfg_t gCFG;
static bool       gOK = false;

bool init() {
  if (gy88_init(&gIMU) != 0) { gOK = false; return false; }
  gy88u_default_cfg(&gCFG, gIMU.mag_type);
  gOK = true;
  return true;
}

bool ok() { return gOK; }

Sample read() {
  Sample s{}; if (!gOK) return s;
  gy88u_data_t d;
  if (gy88u_read_and_convert(&gIMU, &gCFG, &d) != 0) return s;

  s.ax_g = d.ax_g; s.ay_g = d.ay_g; s.az_g = d.az_g;
  s.gx_dps = d.gx_dps; s.gy_dps = d.gy_dps; s.gz_dps = d.gz_dps;
  s.roll_deg = d.roll_deg; s.pitch_deg = d.pitch_deg;
  s.mx_uT = d.mx_uT; s.my_uT = d.my_uT; s.mz_uT = d.mz_uT; s.heading_deg = d.heading_deg;
  s.temperature_c = d.temperature_c; s.pressure_pa = d.pressure_pa; s.altitude_m = d.altitude_m;
  return s;
}

} // namespace imu
