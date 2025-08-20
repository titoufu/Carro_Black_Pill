# WIRING — Ligações e Jumpers

## L298N (MCU → Driver)
| L298N | MCU | Obs. |
|---|---|---|
| ENA  | **PB9**  | PWM (TIM4_CH4 / AF2) |
| IN1  | **PB8**  | saída digital |
| IN2  | **PB7**  | saída digital |
| IN3  | **PB6**  | saída digital |
| IN4  | **PB5**  | saída digital |
| ENB  | **PB4**  | PWM (TIM3_CH1 / AF2) |
| 12V  | Fonte motores | **Nunca** 12V no 5V! |
| GND  | Comum | **Unir GND** de fonte dos motores, L298N e STM32 |

## I²C2 (OLED + GY-88)
| Função | MCU | AF |
|---|---|---|
| SCL | **PB10** | AF4 |
| SDA | **PB3**  | AF9 |

- OLED 0x3C, IMU: MPU6050 0x68, QMC/HMC 0x0D/0x1E, BMP085 0x77.
