# GY-88 (MPU6050 + HMC/QMC5883 + BMP085) for STM32 HAL (PlatformIO)

**Wiring (BlackPill F411, I2C2):**
- PB10 = I2C2_SCL (AF4)
- PB3  = I2C2_SDA (AF9)
- 3V3, GND in common with the GY-88 module

**Add to your project:**
- Copy the `lib/GY88/` folder into your project (PlatformIO).
- Make sure your `stm32f4xx_hal_msp.c` configures I2C2 pins as AF_OD (no internal pull-ups).
- In `ssd1306_conf.h` you already set `SSD1306_I2C_PORT hi2c2`. This library uses the same I2C handle via `gy88_conf.h` (`#define GY88_I2C (&hi2c2)`).

**Usage snippet (C++ main.cpp):**
```cpp
extern "C" {
  #include "gy88.h"
}
gy88_t imu;
if (gy88_init(&imu) == 0) {
  gy88_raw_t d;
  if (gy88_read_all(&imu, &d) == 0) {
    // d.ax..gz (raw), d.mx..mz (raw), d.temperature_c, d.pressure_pa
  }
}
```

**Notes:**
- This is a minimal, blocking implementation. For higher rate sampling, place BMP085 reads less frequently or run in a separate task.
- If your MPU6050 uses address 0x69 (AD0=1), change `MPU6050_I2C_ADDR` in `gy88_conf.h`.
- Magnetometer auto-detection tries HMC5883L (0x1E) first, then QMC5883L (0x0D).
```