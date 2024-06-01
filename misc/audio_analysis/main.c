// gcc -O2 -DMINIAUDIO_IMPLEMENTATION -c -x c miniaudio.h
// make KISSFFT_DATATYPE=int16_t KISSFFT_STATIC=1
// gcc -std=c99 main.c miniaudio.o -Ikissfft-131.1.0 kissfft-131.1.0/libkissfft-int16_t.a

#include "miniaudio.h"  // miniaudio - v0.11.21 (4a5b74b)

#define FIXED_POINT 16
#include "kiss_fftr.h"  // kiss_fft-131.1.0

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

static void breath_detector_feed(struct breath_detector *restrict d, const int16_t *restrict a, uint32_t n)
{
  uint32_t i = 0;
  while (i < n) {
    uint32_t limit = BREATH_DET_WINDOW_SIZE - d->buf_ptr;
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
      for (uint32_t i = 1; i <= 1024; i += 8) {
        int32_t sum = 0;
        for (uint32_t j = i; j < i + 8; j++) {
          sum += (int32_t)fft_result[j].r * fft_result[j].r + 
                 (int32_t)fft_result[j].i * fft_result[j].i;
        }
        putchar(sum >= 500 ? '*' : (sum >= 100 ? '.' : ' '));
      }
      putchar('|');
      putchar('\n');

      d->buf_ptr = 0;
    }
  }
}

static bool breath_detector_is_exhale(const struct breath_detector *d)
{
  return d->is_exhale;
}

static struct breath_detector d;

void input_cb(ma_device* dev, void *_output, const void *_input, ma_uint32 n_frames)
{
  const int16_t *input = (const int16_t *)_input;
  // static int n = 0; if ((n += n_frames) >= 48000) { n -= 48000; puts("!"); }
  breath_detector_feed(&d, input, n_frames);
  (void)_output;
}

int main()
{
  ma_context ctx;
  if (ma_context_init(NULL, 0, NULL, &ctx) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise audio context\n");
    return 1;
  }

  ma_device_info *in_dev_infos;
  ma_uint32 n_in_devs = 0;
  ma_context_get_devices(&ctx, NULL, NULL, &in_dev_infos, &n_in_devs);

  const ma_device_id *sel_id = NULL;
  for (int i = 0; i < n_in_devs; i++) {
    printf("Device: %s\n", in_dev_infos[i].name);
    if (strstr(in_dev_infos[i].name, "MC001"))
      sel_id = &in_dev_infos[i].id;
  }

  ma_device_config dev_conf;
  ma_device dev;

  dev_conf = ma_device_config_init(ma_device_type_capture);
  dev_conf.capture.pDeviceID = sel_id;
  dev_conf.capture.format = ma_format_s16;
  dev_conf.capture.channels = 1;
  dev_conf.sampleRate = 48000;
  dev_conf.dataCallback = input_cb;

  if (ma_device_init(NULL, &dev_conf, &dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise capture device\n");
    return 1;
  }

  char name[256];
  ma_device_get_name(&dev, ma_device_type_capture, name, sizeof name, NULL);
  printf("Selected device: %s\n", name);

  breath_detector_init(&d);

  if (ma_device_start(&dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot start device\n");
    return 1;
  }

  printf("Running. Press Enter to stop\n");
  getchar();

  return 0;
}
