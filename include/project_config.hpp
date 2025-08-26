#pragma once
// Liga/desliga subsistemas (útil p/ testes ou builds pequenos)
#define USE_OLED    1
#define USE_IMU     1
#define USE_MOTORS  1
// --- Rádio nRF24 ---
#ifndef USE_RADIO
#define USE_RADIO 1
#endif

#ifndef USE_RADIO_TEST
#define USE_RADIO_TEST 1
#endif

// Impressão de float: 0 = helper sem %f ; 1 = ativa printf float (maior binário)
#define ENABLE_PRINTF_FLOAT 0
