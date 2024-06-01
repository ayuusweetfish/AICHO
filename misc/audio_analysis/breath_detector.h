#pragma once

#define FIXED_POINT 16
#include "kiss_fftr.h"  // kiss_fft-131.1.0

#include <stdbool.h>
#include <stdint.h>

#define BREATH_DET_WINDOW_SIZE 2048
struct breath_detector {
  int16_t buf[BREATH_DET_WINDOW_SIZE];
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
  uint32_t i = 0;
  while (i < n) {
    uint32_t limit = i + BREATH_DET_WINDOW_SIZE - d->buf_ptr;
    if (limit > n) limit = n;
    while (i < limit) d->buf[d->buf_ptr++] = a[i++];

    if (d->buf_ptr == BREATH_DET_WINDOW_SIZE) {
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

      for (uint32_t i = 0; i < BREATH_DET_WINDOW_SIZE; i++) {
        d->buf[i] = (int32_t)d->buf[i] * hann_window[i] / 32768;
      }

      kiss_fftr(fft_cfg, d->buf, fft_result);
      // 1024 = Nyquist = 24 kHz
      // 1 = 23.4375 Hz

      #define cplx_norm2(_x) ((int32_t)(_x).r * (_x).r + (int32_t)(_x).i * (_x).i)

      static int count = 0; // XXX: Not suitable for multiple instances but anyway
      if (count % 100 == 0) {
        printf("              |");
        for (uint32_t i = 1, bin_size; i <= BREATH_DET_WINDOW_SIZE / 2; i += bin_size) {
          bin_size = (i < 800 ? 8 : 16);
          if ((int)(i * 23.4375 / 2000) != (int)((i + bin_size) * 23.4375 / 2000))
            putchar('0' + (int)((i + bin_size) * 23.4375 / 1000) % 10);
          else putchar(' ');
        }
        putchar('|');
        putchar('\n');
      }
      float T = (float)(count++) * BREATH_DET_WINDOW_SIZE / 48000;
      printf("(%4d) %6.2f |", count, T);
      for (uint32_t i = 1, bin_size; i <= BREATH_DET_WINDOW_SIZE / 2; i += bin_size) {
        bin_size = (i < 800 ? 8 : 16);
        int32_t sum = 0;
        for (uint32_t j = i; j < i + 8; j++) {
          sum += (int32_t)fft_result[j].r * fft_result[j].r + 
                 (int32_t)fft_result[j].i * fft_result[j].i;
        }
        // putchar(sum >= 500 ? '*' : sum >= 100 ? '+' : sum >= 50 ? '.' : ' ');
      }
      for (int i = 280; i < 640; i++) {
        int32_t e = cplx_norm2(fft_result[i]);
        e = min(10, e);
        putchar(e == 0 ? ' ' : '0' + e);
      }
      putchar('|');

      int32_t energy_total = 0;
      for (uint32_t j = 100; j <= BREATH_DET_WINDOW_SIZE / 2; j++)
        energy_total += cplx_norm2(fft_result[j]);

      int32_t sum_lo = 0;
      for (uint32_t j = 1; j <= 32; j++)
        sum_lo +=
          (int32_t)fft_result[j].r * fft_result[j].r + 
          (int32_t)fft_result[j].i * fft_result[j].i;
      int32_t sum_mid = 0;
      for (uint32_t j = 200; j <= 300; j++)
        sum_mid +=
          (int32_t)fft_result[j].r * fft_result[j].r + 
          (int32_t)fft_result[j].i * fft_result[j].i;
      int32_t sum_hi = 0;
      for (uint32_t j = 512; j <= 1024; j++)
        sum_hi += cplx_norm2(fft_result[j]);

      if (false && count == 152) {
        putchar('\n');
        for (int i = 1; i <= 1024; i++) printf("%4d %5d\n", i, cplx_norm2(fft_result[i]));
        printf("total = %d, hi = %d\n", energy_total, sum_hi);
        exit(0);
      }

      static int32_t bins[BREATH_DET_WINDOW_SIZE / 2];
      // From 6.6 kHz = 280-th coefficient
      // To 15 kHz = 640-th coefficient
      for (uint32_t i = 70; i < 160; i++) {
        int32_t sum = 0;
        for (int j = i * 4 + 1; j <= i * 4 + 4; j++)
          sum += cplx_norm2(fft_result[j]);
        bins[i - 70] = sum;
      }
      qsort(bins, 90, sizeof(int32_t), int32_compare_desc);

      // printf(" %4d %4d %4d\n", (int)min(sum_lo / 100, 9999), (int)min(sum_mid / 100, 9999), (int)sum_hi);
      // printf(" %d %d %4d %4d %4d\n", sum_lo >= 5000, sum_hi >= 200, (int)bins[5], (int)bins[10], (int)bins[60]);
      // printf(" %d %d %d %d\n", sum_lo >= 5000, sum_hi >= 200, bins[20] >= 30, bins[40] >= 15); // for non-binned
      printf(" %4d %4d %3d %3d %3d",
        min(sum_mid / 100, 9999), min(sum_hi, 9999),
        min(999, bins[5]), min(999, bins[10]), min(999, bins[20]));

      // printf(" %6d", sum_hi * 10000 / (energy_total == 0 ? 1 : energy_total));
      // printf(" %6d", sum_hi * 10000 / (energy_total == 0 ? 1 : energy_total) >= 500);

      // printf(" %d", sum_hi >= 150);
      printf(" GT=%d\n",
        (T >= 6.4 && T < 7.8) ||
        (T >= 14.9 && T < 16.0) ||
        (T >= 20.64 && T < 21.74) ||
        (T >= 30.0 && T < 30.9) ||
        (T >= 33.84 && T < 34.55) ||
        (T >= 37.21 && T < 37.83) ||
        (T >= 40.93 && T < 42.8) ||
        (T >= 44.93 && T < 45.91) ||
        (T >= 49.98 && T < 51.32) ||
        (T >= 55.15 && T < 56.97)
      );

      d->buf_ptr = 0;
    }
  }
}

static bool breath_detector_is_exhale(const struct breath_detector *d)
{
  return d->is_exhale;
}
