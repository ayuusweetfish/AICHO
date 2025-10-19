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

  int ban_counter;
  int out_counter;
  bool last_out;
};

static void breath_detector_init(struct breath_detector *d)
{
  d->buf_ptr = 0;

  d->ban_counter = 0;
  d->out_counter = 0;
  d->last_out = false;
}

#define min(_a, _b) ((_a) < (_b) ? (_a) : (_b))

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
      static uint16_t hann_window[BREATH_DET_WINDOW_SIZE];
      static bool initialised = false;
      if (!initialised) {
        fft_cfg = kiss_fftr_alloc(BREATH_DET_WINDOW_SIZE, 0, NULL, NULL);
        for (uint32_t i = 0; i < BREATH_DET_WINDOW_SIZE; i++) {
          hann_window[i] = (int16_t)(65535.5f *
            (0.5f * (1 - cosf((float)i / BREATH_DET_WINDOW_SIZE * (2 * (float)M_PI)))));
        }
        initialised = true;
      }

      static int32_t b[BREATH_DET_WINDOW_SIZE];
      for (uint32_t i = 0; i < BREATH_DET_WINDOW_SIZE; i++) {
        uint32_t j = (i + d->buf_ptr) % BREATH_DET_WINDOW_SIZE;
        b[i] = (int32_t)d->buf[j] * hann_window[i] / 65536;
      }

      kiss_fftr(fft_cfg, b, fft_result);
      // 2048 = Nyquist = 24 kHz
      // 1 = 11.71875 Hz

      #define cplx_norm2(_x) \
        ((uint32_t)((int32_t)(_x).r * (_x).r) + \
         (uint32_t)((int32_t)(_x).i * (_x).i))

      float T = (float)(count++) * (BREATH_DET_WINDOW_SIZE / 2) / 48000;
      debug("(%4d) %6.2f |", count, T);

      uint64_t sum = 0;
      for (int i = 6; i < 1280; i++)
        sum += cplx_norm2(fft_result[i]) + 1;
      uint64_t unsmooth = 0;
      uint32_t min_e = 0;
      for (int i = 6; i < 1280; i += 5) {
        uint32_t e = 0;
        for (int j = 0; j < 60; j++) {
          e += cplx_norm2(fft_result[i + j]);
        }
        if (e > min_e) unsmooth += (e - min_e) * i;
        else if (e < min_e) min_e -= (min_e - e) / 2;
      }
      debug("%10u |", (unsigned)(unsmooth / sum));
      bool smooth_det = (unsmooth / sum <= 500);

      uint64_t low_sum = 0;
      for (int i = 6; i < 20; i++)
        low_sum += cplx_norm2(fft_result[i]);

      int low_cont, low_tol = 25;
      for (low_cont = 6; low_cont < 17; low_cont++) {
        uint32_t e = cplx_norm2(fft_result[low_cont]);
        if (e < 10) {
          e = 10 - e;
          if ((low_tol -= e) <= 0) break;
        }
      }
      debug("%10u %3d |", (unsigned)(low_sum / 100), low_cont);
      bool low_det_1 = low_sum >= 1000 * 100;
      bool low_det_2 = low_cont >= 15;

      debug(" %c %c %c |", smooth_det ? '#' : ' ', low_det_1 ? '*' : ' ', low_det_2 ? '*' : ' ');
      bool final_det = (smooth_det && (low_det_1 || low_det_2));

      if (!smooth_det) {
        d->ban_counter += 5;
        if (d->ban_counter >= 10) d->ban_counter = 10;
      }

      if (d->ban_counter > 0) d->ban_counter--;
      if (d->ban_counter <= 2) {
        if (final_det && d->out_counter < 4) d->out_counter++;
        else if (!final_det && d->out_counter > 0) d->out_counter--;
        if (!d->last_out && d->out_counter >= 3) d->last_out = true;
        else if (d->last_out && d->out_counter < 2) d->last_out = false;
      } else {
        d->out_counter = 0;
        d->last_out = false;
      }
      bool hyst_det = d->last_out;

      debug(" %c %c %c |", final_det ? '!' : ' ', d->ban_counter <= 2 ? ' ' : 'x', hyst_det ? '!' : ' ');

      for (int i = 6; i < 80; i += 1) {
        uint32_t e = 0;
        for (int j = 0; j < 1; j++) {
          e += cplx_norm2(fft_result[i + j]);
        }
        e = min(10, e / 2);
        debug("%c", e == 0 ? ' ' : '0' + e);
      }
      debug("\n");
      if (d->buf_ptr == BREATH_DET_WINDOW_SIZE) d->buf_ptr = 0;
    }
  }
}
