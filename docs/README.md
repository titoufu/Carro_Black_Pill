# Carrinho STM32 (BlackPill F411) + L298N + OLED SSD1306 + IMU GY-88

Projeto de robótica embarcada usando:
- **WeAct BlackPill F411CE** (STM32F411, 100 MHz)
- **Driver de motores L298N** (2× DC)
- **OLED SSD1306 128×64 I²C** (0x3C)
- **IMU GY-88** (MPU6050 + HMC/QMC5883 + BMP085) via I²C2

## Principais recursos
- PWM por hardware para **ENA (PB9/TIM4_CH4)** e **ENB (PB4/TIM3_CH1)**
- Classe `Motor` (C++) para **start/brake/coast** e direção com duty 0–100%
- OLED com **UI padronizada** (telas de Accel/Gyro/Mag/Barômetro)
- IMU com **conversões já prontas** (g, °/s, µT, heading, °C, Pa, m)
- `main.cpp` **enxuto/orquestrador**, resto em **libs** (`lib/`)

## Estrutura do projeto

.
├─ src/
│ └─ main.cpp
├─ include/
│ ├─ project_config.hpp
│ └─ text_fmt.hpp
├─ lib/
│ ├─ Motor/ (C++)
│ ├─ Display/ (C++ wrapper para SSD1306 em C)
│ ├─ IMU/ (C++ wrapper para GY-88 em C)
│ ├─ UI/ (telas do OLED)
│ └─ Drivers/
│ ├─ SSD1306_C/ (C “vendor”: ssd1306.c/.h, fonts, conf)
│ └─ GY88_C/ (C “vendor”: mpu6050, qmc/hmc, bmp085, gy88, conf)
└─ docs/

## Pinos (resumo)
| Função | MCU Pin | Observação |
|---|---|---|
| ENA (PWM A) | **PB9** | TIM4_CH4 (AF2) |
| ENB (PWM B) | **PB4** | TIM3_CH1 (AF2) |
| IN1 / IN2   | **PB8 / PB7** | Saídas digitais |
| IN3 / IN4   | **PB6 / PB5** | Saídas digitais |
| I²C2 SCL    | **PB10** | AF4 |
| I²C2 SDA    | **PB3**  | AF9 |

> **L298N**: Deixe **5V_EN** **FECHADO** apenas se **NÃO** for usar 5 V do módulo para alimentar outras coisas. Motores **devem** ter fonte externa adequada. GND comum em todo o sistema.

## Requisitos
- PlatformIO (`board = blackpill_f411ce`, `framework = stm32cube`)
- Gravação via ST-Link (`upload_protocol = stlink`)
- Pull-ups I²C no módulo OLED/IMU (normalmente já presentes)

## Como compilar e gravar
1. Abra a pasta no VS Code (PlatformIO).
2. Se necessário, ajuste `project_config.hpp` (ativar/desativar OLED/IMU/Motor).
3. `PlatformIO: Build` e depois `PlatformIO: Upload`.

## Referências rápidas
- `docs/WIRING.md`: Ligações de hardware e jumpers do L298N  
- `docs/BUILD.md`: Dicas de build e flags úteis  
- `docs/ARCHITECTURE.md`: Camadas e responsabilidades  
- `docs/TROUBLESHOOTING.md`: Problemas comuns e soluções

