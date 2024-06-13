#pragma once

typedef struct {
  uint8_t h, s, v;
} CHSV;

typedef struct {
  uint8_t r, g, b;
} CRGB;

static inline uint8_t scale8(uint8_t n, uint8_t t)
{
  return ((uint16_t)n * (uint16_t)t) >> 8;
}

static inline CHSV CHSV_blend(CHSV a, CHSV b, uint8_t t)
{
  // Ref. FastLED, src/colorutils.cpp
  // Shortest hues
  if (t == 0) return a;
  if (t == 255) return b;

  CHSV c;

  uint8_t h_diff = b.h - a.h;
  if (h_diff < 128) {
    c.h = a.h + scale8(h_diff, t);
  } else {
    c.h = a.h - scale8(-h_diff, t);
  }
  c.s = scale8(a.s, 255 - t) + scale8(b.s, t);
  c.v = scale8(a.v, 255 - t) + scale8(b.v, t);

  return c;
}

static inline CRGB CHSV_to_CRGB(CHSV tint)
{
  uint8_t h = scale8(tint.h, 191);
  uint8_t s = tint.s;
  uint8_t v = tint.v;
  uint8_t m = (v * (255 - s)) / 256;
  uint8_t a = v - m;
  uint8_t sect = h / 64;
  uint8_t offs = h % 64;
  uint8_t c1 = m + (offs * a) / 64;
  uint8_t c2 = m + ((63 - offs) * a) / 64;
  if (sect == 0) {
    return (CRGB){c2, c1, m};
  } else if (sect == 1) {
    return (CRGB){m, c2, c1};
  } else {
    return (CRGB){c1, m, c2};
  }
}

#define LED_N_Lorivox 23

void gradient_Lorivox(int intensity, uint8_t out[][4][3])
{
  // CRGB a = CHSV_to_CRGB((CHSV){40, 180, 80});
  // my_printf("%u %u %u\n", a.r, a.g, a.b);

  for (int i = 0; i < LED_N_Lorivox; i++) {
    float progress = (float)i / (LED_N_Lorivox - 1);
    progress = 1 - (1 - progress) * (1 - progress);
    CHSV yellow = (CHSV){30, 255, 255};
    CHSV green = (CHSV){72, 255, 255};
    CHSV blended = CHSV_blend(green, yellow, progress * 255);
    if (i < 10) blended.v = (int32_t)blended.v * (i * 24) / 255;
    blended.v = (int32_t)blended.v * intensity / 4096 / 4;
    CRGB c = CHSV_to_CRGB(blended);
    out[i][0][0] = (c.r >= 8 || c.r < c.g ? c.r : c.g);
    out[i][0][1] = c.g;
    out[i][0][2] = c.b;
  }
}
