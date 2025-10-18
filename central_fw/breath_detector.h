#pragma once

#define FIXED_POINT 32
#include "kiss_fftr.h"  // kiss_fft-131.1.0

#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#define BREATH_DET_WINDOW_SIZE 4096
struct breath_detector {
  int32_t buf[BREATH_DET_WINDOW_SIZE];
  uint32_t buf_ptr;

  bool is_exhale;
};

static void breath_detector_init(struct breath_detector *d)
{
  d->buf_ptr = 0;
  d->is_exhale = false;
}

#define min(_a, _b) ((_a) < (_b) ? (_a) : (_b))
static int int32_compare_desc(const void *a, const void *b)
{
  return *(const int32_t *)b - *(const int32_t *)a;
}

static void breath_detector_feed(struct breath_detector *restrict d, const int16_t *restrict a, uint32_t n)
{
  static int count = 0; // XXX: Not suitable for multiple instances but anyway

  uint32_t i = 0;
  while (i < n) {
    uint32_t limit = i +
      (BREATH_DET_WINDOW_SIZE / 2 - d->buf_ptr % (BREATH_DET_WINDOW_SIZE / 2));
    if (limit > n) limit = n;
    while (i < limit) d->buf[d->buf_ptr++] = a[i++];

    if (d->buf_ptr % (BREATH_DET_WINDOW_SIZE / 2) == 0) {
      // Analyse
      static kiss_fftr_cfg fft_cfg;
      static kiss_fft_cpx fft_result[BREATH_DET_WINDOW_SIZE / 2 + 1];
      static int16_t hann_window[BREATH_DET_WINDOW_SIZE];
      static bool initialised = false;
      if (!initialised) {
        fft_cfg = kiss_fftr_alloc(BREATH_DET_WINDOW_SIZE, 0, NULL, NULL);
        for (uint32_t i = 0; i < BREATH_DET_WINDOW_SIZE; i++) {
          hann_window[i] = (int16_t)(32767.5f *
            (0.5f * (1 - cosf((float)i / BREATH_DET_WINDOW_SIZE * (2 * (float)M_PI)))));
        }
        initialised = true;
      }

      static int32_t b[BREATH_DET_WINDOW_SIZE];
      for (uint32_t i = 0; i < BREATH_DET_WINDOW_SIZE; i++) {
        uint32_t j = (i + d->buf_ptr) % BREATH_DET_WINDOW_SIZE;
        b[i] = (int32_t)d->buf[j] * hann_window[i] / 32768 * 256;
      }

      kiss_fftr(fft_cfg, b, fft_result);
      // 2048 = Nyquist = 24 kHz
      // 1 = 11.71875 Hz

      #define cplx_norm2(_x) \
        ((uint32_t)((int32_t)(_x).r * (_x).r) + \
         (uint32_t)((int32_t)(_x).i * (_x).i))

      float T = (float)(count++) * (BREATH_DET_WINDOW_SIZE / 2) / 48000;
      printf("(%4d) %6.2f |", count, T);

      uint32_t sum = 0;
      for (int i = 240; i < 640; i++) sum += cplx_norm2(fft_result[i]) / 256;
      printf("%10u |", (unsigned)sum);

      bool stop = 0;
      for (int i = 6; i < 640; i += 4) {
        int32_t e = 0;
        for (int j = 0; j < 1; j++) {
          e += cplx_norm2(fft_result[i + j]) / 256;
        }
        e /= 64;
        e = min(10, e);
        // if (stop) e = 0; else if (e < 5) stop = 1;
        putchar(e == 0 ? ' ' : '0' + e);
      }
      putchar('|');

      bool pred = 1;
      bool confidence = (
        1
      );

      static int count = 0;
      static bool last_out = false;
      if (confidence) {
        if (pred && count < 4) count++;
        else if (!pred && count > 0) count--;
        if (!last_out && count >= 3) last_out = true;
        else if (last_out && count < 2) last_out = false;
      }
      bool out = last_out;

      printf("\n");
      if (d->buf_ptr == BREATH_DET_WINDOW_SIZE) d->buf_ptr = 0;
    }
  }
}

static bool breath_detector_is_exhale(const struct breath_detector *d)
{
  return d->is_exhale;
}
