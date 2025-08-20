# BUILD — Como compilar e gravar

## PlatformIO
`platformio.ini` (trecho relevante):
```ini
[env:blackpill_f411ce]
platform = ststm32
board = blackpill_f411ce
framework = stm32cube
upload_protocol = stlink
debug_tool = stlink

; Flags opcionais
build_flags =
  -DUSE_HAL_DRIVER
  -DSTM32F411xE
  -DUSE_OLED=1
  -DUSE_IMU=1
  -DUSE_MOTORS=1

  Fluxo

Conecte o ST-Link ao SWD da BlackPill.

PlatformIO: Build → PlatformIO: Upload.

Float em printf

O projeto não usa %f (mantém binário menor).

Funções de formatação: include/text_fmt.hpp (fmt_signed_fp).

Se quiser %f em printf/snprintf, habilite _printf_float (tamanho do binário aumenta):

Adicione ao platformio.ini:
