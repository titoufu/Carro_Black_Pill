#include "status_screens.hpp"
#include "display.hpp"
#include "text_fmt.hpp"
#include <cstdio>
#include <cmath>

namespace ui
{

    void showAccel(const imu::Sample &s)
    {
        char ln[24], v[16];
        display::clear();
        display::line(0, "ACC (g)");

        fmt_signed_fp(v, sizeof(v), s.ax_g, 2);
        std::snprintf(ln, sizeof(ln), "Ax:%s", v);
        display::line(12, ln);

        fmt_signed_fp(v, sizeof(v), s.ay_g, 2);
        std::snprintf(ln, sizeof(ln), "Ay:%s", v);
        display::line(24, ln);

        fmt_signed_fp(v, sizeof(v), s.az_g, 2);
        std::snprintf(ln, sizeof(ln), "Az:%s", v);
        display::line(36, ln);

        float mod = std::sqrt(s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
        fmt_signed_fp(v, sizeof(v), mod, 2);
        std::snprintf(ln, sizeof(ln), "|A|:%s", v);
        display::line(48, ln);

        display::update();
    }

    void showGyro(const imu::Sample &s)
    {
        char ln[24], v[16], v2[16];
        display::clear();
        display::line(0, "GYRO (dps)");

        fmt_signed_fp(v, sizeof(v), s.gx_dps, 1);
        std::snprintf(ln, sizeof(ln), "Gx:%s", v);
        display::line(12, ln);

        fmt_signed_fp(v, sizeof(v), s.gy_dps, 1);
        std::snprintf(ln, sizeof(ln), "Gy:%s", v);
        display::line(24, ln);

        fmt_signed_fp(v, sizeof(v), s.gz_dps, 1);
        std::snprintf(ln, sizeof(ln), "Gz:%s", v);
        display::line(36, ln);

        fmt_signed_fp(v, sizeof(v), s.roll_deg, 1);
        fmt_signed_fp(v2, sizeof(v2), s.pitch_deg, 1);
        std::snprintf(ln, sizeof(ln), "r:%s p:%s", v, v2);
        display::line(48, ln);

        display::update();
    }

    void showMag(const imu::Sample &s)
    {
        char ln[24], v[16];
        display::clear();
        display::line(0, "MAG (uT)");

        fmt_signed_fp(v, sizeof(v), s.mx_uT, 2);
        std::snprintf(ln, sizeof(ln), "Mx:%s", v);
        display::line(12, ln);
        fmt_signed_fp(v, sizeof(v), s.my_uT, 2);
        std::snprintf(ln, sizeof(ln), "My:%s", v);
        display::line(24, ln);
        fmt_signed_fp(v, sizeof(v), s.mz_uT, 2);
        std::snprintf(ln, sizeof(ln), "Mz:%s", v);
        display::line(36, ln);

        fmt_signed_fp(v, sizeof(v), s.heading_deg, 0);
        std::snprintf(ln, sizeof(ln), "HDG:%s", v);
        display::line(48, ln);

        display::update();
    }

    void showBaro(const imu::Sample &s)
    {
        char ln[24], v[16];
        display::clear();
        display::line(0, "BARO");

        fmt_signed_fp(v, sizeof(v), s.temperature_c, 1);
        std::snprintf(ln, sizeof(ln), "T:%sC", v);
        display::line(12, ln);

        std::snprintf(ln, sizeof(ln), "P:%7dPa", (int)(s.pressure_pa + 0.5f));
        display::line(24, ln);

        fmt_signed_fp(v, sizeof(v), s.altitude_m, 1);
        std::snprintf(ln, sizeof(ln), "ALT:%sm", v);
        display::line(36, ln);

        display::update();
    }

} // namespace ui
