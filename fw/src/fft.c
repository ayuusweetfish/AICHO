#define N 2048
// N = 2048: 20756 bytes
// XXX: If FFT size is changed, this needs to be updated as well
#define MY_KISS_FFT_BUF_SIZE 20756
#define KISS_FFT_AMALG_IMPL
#include "kiss_fft_amalgamate.h"

#include "main.h"

static kiss_fftr_cfg fft_cfg;

void fft_init()
{
  fft_cfg = kiss_fftr_alloc(N, 0, NULL, NULL);

  static int32_t in[N];
  static kiss_fft_cpx fft_result[N / 2 + 1];

  for (int i = 0; i < N; i++) in[i] = i + (i % 4 / 2) * 50;

  kiss_fftr(fft_cfg, in, fft_result);
  for (int i = 0; i <= N / 2; i++)
    my_printf("%4d %5d %5d\n", i, fft_result[i].r, fft_result[i].i);

  kiss_fftri(fft_cfg, fft_result, in);
  for (int i = 0; i < N; i++)
    my_printf("%4d %5d\n", i, in[i]);
}
