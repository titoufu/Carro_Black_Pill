# TROUBLESHOOTING — Rápido

- **Motores imóveis**: PWM startado? ENA/ENB em AF2 corretos? INs PB8..PB5 são saídas? GND comum?
- **OLED só com labels**: evite `%f`; use `fmt_signed_fp`. Endereço 0x3C ok? I²C2 PB10/PB3 AFs corretos?
- **IMU FAIL**: `gy88_conf.h` aponta p/ `&hi2c2`? Alimentação e cabos ok?
- **Conflito PB4/SYS**: desabilite debug Serial Wire no Cube se necessário.
