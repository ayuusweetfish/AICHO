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

static inline uint8_t blend8(uint8_t a, uint8_t b, uint8_t t)
{
  return a + ((((uint16_t)b - a) * t) >> 8);
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

static inline CRGB CRGB_blend(CRGB a, CRGB b, uint8_t t)
{
  return (CRGB){
    blend8(a.r, b.r, t),
    blend8(a.g, b.g, t),
    blend8(a.b, b.b, t),
  };
}

#define LED_N_Lorivox 23
void gradient_Lorivox(int intensity, uint8_t out[])
{
  for (int i = 0; i < LED_N_Lorivox; i++) {
    float progress = (float)i / (LED_N_Lorivox - 1);
    progress = 1 - (1 - progress) * (1 - progress);
    CHSV yellow = (CHSV){30, 255, 255};
    CHSV green = (CHSV){72, 255, 255};
    CHSV blended = CHSV_blend(green, yellow, progress * 255);
    if (i < 10) blended.v = (int32_t)blended.v * (i * 24) / 255;
    blended.v = (int32_t)blended.v * intensity / 4096 / 2;
    CRGB c = CHSV_to_CRGB(blended);
    c.r = (c.r >= 8 || c.r < c.g ? c.r : c.g);
    LED_WRITE_I(i, c);
  }
}

#define LED_N_Lumisonic 69
void gradient_Lumisonic(int intensity, int progress, uint8_t out[])
{
  for (int i = 0; i < LED_N_Lumisonic; i++) {
    LED_WRITE_I(i, ((CRGB){0, 0, 0}));
  }

  int waveLength = 10;
  int waveStart = -waveLength + (long)(LED_N_Lumisonic + waveLength) * progress / 8192;

  for (int i = 0; i < waveLength; i++) {
    int pos = waveStart + i;
    if (pos >= LED_N_Lumisonic) break;
    if (pos < 0) continue;
    int brightness = i * intensity * 255 / (waveLength - 1) / 4096 / 4;

    CRGB blended = (CRGB){
      144 * brightness / 255,
      128 * brightness / 255,
      255 * brightness / 255,
    };
    LED_WRITE_I(pos, blended);
  }
}

#define LED_N_Harmonia 30
void gradient_Harmonia(int intensity, uint8_t out[])
{
  for (int i = 0; i < LED_N_Harmonia; i++) {
    CHSV blue = (CHSV){44, 255, 255};
    CHSV white = (CHSV){0, 0, 255};
    CHSV blended = CHSV_blend(blue, white, i * 255 / (LED_N_Harmonia - 1));
    blended.v = (int32_t)blended.v * intensity / 4096 / 4;
    CRGB c = CHSV_to_CRGB(blended);
    if (c.r < 8 && c.r > c.g && c.r > c.b) {
      c.r = (c.g > c.b ? c.g : c.b);
    }
    LED_WRITE_I(i, c);
  }
}

#define LED_N_Titanus 120
void gradient_Titanus(int intensity, uint8_t out[])
{
  for (int i = 0; i < LED_N_Titanus; i++) {
    int saturation = 255 - (255 - intensity / 16) / 2;
    CHSV yellow = (CHSV){50, saturation, 255};
    CHSV blue = (CHSV){180, saturation * 3 / 4, 255};
    CRGB blended = CRGB_blend(
      CHSV_to_CRGB(yellow), CHSV_to_CRGB(blue),
      i * 255 / (LED_N_Titanus - 1));
    blended.r = (uint32_t)blended.r * intensity * intensity / 16777216 / 4;
    blended.g = (uint32_t)blended.g * intensity * intensity / 16777216 / 4;
    blended.b = (uint32_t)blended.b * intensity * intensity / 16777216 / 4;
    LED_WRITE_I(i, blended);
  }
}
