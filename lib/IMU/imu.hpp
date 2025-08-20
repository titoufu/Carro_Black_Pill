#pragma once
extern "C" {
  #include "gy88.h"
  #include "gy88_utils.h"
}

namespace imu {

struct Sample {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  float roll_deg, pitch_deg;
  float mx_uT, my_uT, mz_uT, heading_deg;
  float temperature_c, pressure_pa, altitude_m;
};

bool init();         // gy88_init + cfg padrão
bool ok();           // estado de inicialização
Sample read();       // gy88u_read_and_convert

} // namespace imu
