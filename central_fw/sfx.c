#include "miniaudio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static ma_context ctx;
static ma_device dev;

static void output_cb(ma_device* dev, void *_output, const void *_input, ma_uint32 n_frames)
{
  int16_t *output = (int16_t *)_output;
  for (int i = 0; i < n_frames; i++) {
    output[i * 2 + 0] = rand() % 512 - 256;
    output[i * 2 + 1] = rand() % 512 - 256;
  }
  (void)_input;
}

void sfx_start(const char *device_name)
{
  if (ma_context_init(NULL, 0, NULL, &ctx) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise audio context\n");
    exit(1);
  }

  ma_device_info *out_dev_infos;
  ma_uint32 n_out_devs = 0;
  ma_context_get_devices(&ctx, &out_dev_infos, &n_out_devs, NULL, NULL);

  const ma_device_id *sel_id = NULL;
  for (int i = 0; i < n_out_devs; i++) {
    printf("Output device: %s\n", out_dev_infos[i].name);
    if (device_name != NULL && strstr(out_dev_infos[i].name, device_name))
      sel_id = &out_dev_infos[i].id;
  }

  ma_device_config dev_conf;

  dev_conf = ma_device_config_init(ma_device_type_playback);
  dev_conf.playback.pDeviceID = sel_id;
  dev_conf.playback.format = ma_format_s16;
  dev_conf.playback.channels = 2;
  dev_conf.sampleRate = 48000;
  dev_conf.dataCallback = output_cb;

  if (ma_device_init(NULL, &dev_conf, &dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialise playback device\n");
    exit(1);
  }

  char name[256];
  ma_device_get_name(&dev, ma_device_type_playback, name, sizeof name, NULL);
  printf("Selected device: %s\n", name);

  if (ma_device_start(&dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot start device\n");
    exit(1);
  }
}

static int n_sounds = 0;

void sfx_load(const char *path)
{
  printf("Loading sound %s (%d)\n", path, n_sounds);
  n_sounds++;
}

void sfx_play(int index)
{
  printf("Sound play %d\n", index);
}
