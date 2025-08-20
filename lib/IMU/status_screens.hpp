#pragma once
#include "imu.hpp"

namespace ui {
  void showAccel(const imu::Sample& s);
  void showGyro (const imu::Sample& s);
  void showMag  (const imu::Sample& s);
  void showBaro (const imu::Sample& s);
}
