#include "miniaudio.h"

#define debug(...)
#include "breath_detector.h"

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>

static struct breath_detector d;

static void input_cb(ma_device* dev, void *_output, const void *_input, ma_uint32 n_frames)
{
  const int16_t *input = (const int16_t *)_input;
  breath_detector_feed(&d, input, n_frames);
  (void)_output;
}

static ma_context ctx;
static ma_device dev;

void microphone_start(const char *device_name)
{
  if (ma_context_init(NULL, 0, NULL, &ctx) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise audio context\n");
    exit(1);
  }

  ma_device_info *in_dev_infos;
  ma_uint32 n_in_devs = 0;
  ma_context_get_devices(&ctx, NULL, NULL, &in_dev_infos, &n_in_devs);

  const ma_device_id *sel_id = NULL;
  for (int i = 0; i < n_in_devs; i++) {
    printf("Input device: %s\n", in_dev_infos[i].name);
    if (device_name != NULL && strstr(in_dev_infos[i].name, device_name))
      sel_id = &in_dev_infos[i].id;
  }
  if (sel_id == NULL) {
    fprintf(stderr, "No device selected!\n");
    exit(1);
  }

  ma_device_config dev_conf;

  dev_conf = ma_device_config_init(ma_device_type_capture);
  dev_conf.capture.pDeviceID = sel_id;
  dev_conf.capture.format = ma_format_s16;
  dev_conf.capture.channels = 1;
  dev_conf.sampleRate = 48000;
  dev_conf.dataCallback = input_cb;

  if (ma_device_init(NULL, &dev_conf, &dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise capture device\n");
    exit(1);
  }

  char name[256];
  ma_device_get_name(&dev, ma_device_type_capture, name, sizeof name, NULL);
  printf("Selected device: %s\n", name);

  breath_detector_init(&d);

  if (ma_device_start(&dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot start device\n");
    exit(1);
  }
}

bool microphone_breath_state(void)
{
  return d.last_out;
}
