#pragma once
#include <stdint.h>

// Parâmetros compartilhados TX/RX
#define RF_CHANNEL        108        // 0x6C
#define RF_RATE_250KBPS   1          // DR_LOW=1, DR_HIGH=0
#define RF_PA_LEVEL       1          // ~-12 dBm

// Endereço 5B "RC001"
static const uint8_t RF_ADDR[5] = { 'R','C','0','0','1' };
