#pragma once

#define FIXED_POINT 32
#include "kiss_fftr.h"  // kiss_fft-131.1.0

#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#define BREATH_DET_WINDOW_SIZE 2048
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
  static int confusion[2][2] = {{ 0 }};
  static int count = 0; // XXX: Not suitable for multiple instances but anyway

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
        d->buf[i] = (int32_t)d->buf[i] * hann_window[i] / 32768 * 256;
      }

      kiss_fftr(fft_cfg, d->buf, fft_result);
      // 1024 = Nyquist = 24 kHz
      // 1 = 23.4375 Hz

      #define cplx_norm2(_x) ((int32_t)(_x).r * (_x).r + (int32_t)(_x).i * (_x).i)

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
        for (uint32_t j = i; j < i + 8; j++)
          sum += cplx_norm2(fft_result[j]);
        // putchar(sum >= 500 ? '*' : sum >= 100 ? '+' : sum >= 50 ? '.' : ' ');
      }
      /* for (int i = 280; i < 640; i++) {
        int32_t e = cplx_norm2(fft_result[i]) / 256;
        e = min(10, e);
        putchar(e == 0 ? ' ' : '0' + e);
      } */
      putchar('|');

      int32_t energy_total = 0;
      for (uint32_t j = 100; j <= BREATH_DET_WINDOW_SIZE / 2; j++)
        energy_total += cplx_norm2(fft_result[j]);

      int64_t sum_lo = 0;
      for (uint32_t j = 1; j <= 32; j++)
        sum_lo += cplx_norm2(fft_result[j]);
      int64_t sum_mid = 0;
      for (uint32_t j = 100; j <= 280; j++)
        sum_mid += cplx_norm2(fft_result[j]);
      int64_t sum_hi = 0;
      for (uint32_t j = 640; j < 1024; j++)
        sum_hi += cplx_norm2(fft_result[j]);

      static int64_t hi_ema = 0;
      hi_ema += (sum_hi - hi_ema) / 8;
      printf(" %4d ", (int)((sum_hi - hi_ema) / 10000));

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

      /* printf(" %4d %4d %4d",
        (int)min(sum_lo / 10000 / 256, 9999),
        (int)min(sum_mid / 100 / 256, 9999),
        (int)min(sum_hi / 360 / 256, 9999)); */
      printf("%5d", (int)(sum_lo * 1 / (sum_mid + 1)));
      /* printf(" %7d %4d",
        min(bins[10], 9999999),
        (int)min((uint64_t)bins[10] * 10000 / (sum_mid + 1), 9999)); */
      printf(" %5d", (bins[10] - bins[20]) * 10 / (bins[20] - bins[50] + 1));

      printf(" [%d %d %d %d %d %d %d] ",
        bins[20] / 256 >= 20,
        (uint64_t)bins[10] * 10000 / (sum_mid + 1) >= 50,
        (uint64_t)bins[20] * 10000 / (sum_mid + 1) >= 25,
        (uint64_t)bins[40] * 10000 / (sum_mid + 1) >= 5,
        (int)(sum_lo / (sum_mid + 1)) >= 10,
        (bins[10] - bins[20]) * 10 / (bins[30] - bins[50] + 1) <= 150,
        llabs(sum_hi - hi_ema) <= 500000
      );

      bool pred = (
        // (bins[10] / 256) >= 100;
        (bins[20] / 256 >= 20) +
        1 +
        1 +
        ((uint64_t)bins[40] * 10000 / (sum_mid + 1) >= 5) +
        1 +
        ((bins[10] - bins[20]) * 10 / (bins[30] - bins[50] + 1) <= 150) +
        (llabs(sum_hi - hi_ema) <= 500000)
      ) >= 7;
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

      // printf(" %d", sum_hi >= 150);
      bool gt =
        (T >= 6.4 && T < 7.8) ||
        (T >= 14.9 && T < 16.0) ||
        (T >= 20.64 && T < 21.74) ||
        (T >= 30.0 && T < 30.9) ||
        (T >= 33.84 && T < 34.55) ||
        (T >= 37.21 && T < 37.83) ||
        (T >= 40.93 && T < 42.8) ||
        (T >= 44.93 && T < 45.91) ||
        (T >= 49.98 && T < 51.32) ||
        (T >= 55.15 && T < 56.97);
      printf(" GT=%d Pred=%d Hy.Out=%d\n", gt, pred, out);
      confusion[(int)gt][(int)out] += 1;

      d->buf_ptr = 0;
    }
  }

  if (count == 1349) {
    printf("GT\\Pred\n");
    printf("  %5d %5d\n  %5d %5d\n",
      confusion[0][0], confusion[0][1],
      confusion[1][0], confusion[1][1]);
  }
}

static bool breath_detector_is_exhale(const struct breath_detector *d)
{
  return d->is_exhale;
}
