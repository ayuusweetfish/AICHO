#include "main.h"

// FFT_N = 2048: 20756 bytes
// FFT_N = 4096: 41236 bytes
// XXX: If FFT size is changed, this needs to be updated as well
#define MY_KISS_FFT_BUF_SIZE 20756
#define KISS_FFT_AMALG_IMPL
#include "kiss_fft_amalgamate.h"

#include <math.h>

static kiss_fftr_cfg fft_cfg;
static uint32_t hann_window[FFT_N];

void fft_init()
{
  fft_cfg = kiss_fftr_alloc(FFT_N, 0, NULL, NULL);

  for (int j = 0; j < FFT_N; j++) {
    hann_window[j] = 4294967295.0f * (1 - cosf((float)j / FFT_N * (float)(2 * M_PI)));
  }

/*
  static int32_t in[FFT_N];
  static kiss_fft_cpx fft_result[FFT_N / 2 + 1];

  for (int i = 0; i < FFT_N; i++) in[i] = i + (i % 4 / 2) * 50;

  kiss_fftr(fft_cfg, in, fft_result);
  for (int i = 0; i <= FFT_N / 2; i++)
    my_printf("%4d %5d %5d\n", i, fft_result[i].r, fft_result[i].i);

  kiss_fftri(fft_cfg, fft_result, in);
  for (int i = 0; i < FFT_N; i++)
    my_printf("%4d %5d\n", i, in[i]);
*/
}

void fft(const int32_t *restrict in, int32_t *restrict out)
{
  // Apply window
  static int32_t windowed[FFT_N];
  for (int i = 0; i < FFT_N; i++) {
    windowed[i] = (int64_t)in[i] * hann_window[i] >> 32;
  }
  kiss_fftr(fft_cfg, windowed, (kiss_fft_cpx *)out);
}
