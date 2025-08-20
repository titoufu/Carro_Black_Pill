#pragma once
#include <cstdio>

static inline void fmt_signed_fp(char* buf, size_t n, float v, int decs = 2) {
  char sign = (v < 0.0f) ? '-' : '+';
  if (v < 0.0f) v = -v;
  int pow10 = 1; for (int i=0;i<decs;++i) pow10 *= 10;
  int scaled = (int)(v * pow10 + 0.5f);
  int whole  = scaled / pow10;
  int frac   = scaled % pow10;
  if (decs <= 0) std::snprintf(buf, n, "%c%d", sign, whole);
  else           std::snprintf(buf, n, "%c%d.%0*d", sign, whole, decs, frac);
}
