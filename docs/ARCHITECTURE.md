# ARCHITECTURE — Camadas

- **lib/SSD306/** (C): driver SSD1306 (vendor)
- **lib/Display/** (C++): wrapper leve p/ driver SSD1306
- **lib/GY88/** e **lib/GY88Utils/** (C): drivers vendor e utilitários do GY-88
- **lib/IMU/** (C++): wrapper da IMU **+** telas (`status_screens.*`)
- **lib/motor/** (C++): classe `Motor` (L298N)
- **src/main.cpp**: inicialização de HAL e orquestração de tarefas


