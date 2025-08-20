#pragma once
extern "C" {
  #include "ssd1306.h"
  #include "ssd1306_fonts.h"
}

namespace display {
  inline void init() {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
  }

  inline void clear() {
    ssd1306_Fill(Black);
  }

  inline void splash(const char* l1, const char* l2 = nullptr) {
    clear();
    ssd1306_SetCursor(0, 0);  ssd1306_WriteString((char*)l1, Font_7x10, White);
    if (l2) { ssd1306_SetCursor(0, 12); ssd1306_WriteString((char*)l2, Font_7x10, White); }
    ssd1306_UpdateScreen();
  }

  inline void line(int row_px, const char* txt) {
    ssd1306_SetCursor(0, row_px);
    ssd1306_WriteString((char*)txt, Font_7x10, White);
  }

  inline void update() { ssd1306_UpdateScreen(); }
}
