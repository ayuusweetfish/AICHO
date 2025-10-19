#include "miniaudio.h"

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static ma_context ctx;
static ma_device dev;

static int n_sounds = 0;
static struct sound_t {
  int16_t *buf;
  int n_frames;
} sounds[16];

static struct channel_t {
  int index;
  int ptr;
} ch[8];
static pthread_mutex_t ch_lock;

static inline int16_t sat_add(int16_t a, int16_t b)
{
  int32_t sum = (int32_t)a + (int32_t)b;
  if (sum > INT16_MAX) sum = INT16_MAX;
  if (sum < INT16_MIN) sum = INT16_MIN;
  return sum;
}

static void output_cb(ma_device* dev, void *_output, const void *_input, ma_uint32 n_frames)
{
  int16_t *output = (int16_t *)_output;

  memset(output, 0, n_frames * 2 * sizeof(int16_t));

  pthread_mutex_lock(&ch_lock);
  for (int c = 0; c < sizeof ch / sizeof ch[0]; c++) if (ch[c].index != -1) {
    int index = ch[c].index;
    int n = n_frames;
    int ptr = ch[c].ptr;
    if (n > sounds[index].n_frames - ptr)
      n = sounds[index].n_frames - ptr;
    const int16_t *buf = sounds[index].buf;
    for (int i = 0; i < n; i++) {
      output[i * 2 + 0] = sat_add(output[i * 2 + 0], buf[ptr + i * 2 + 0]);
      output[i * 2 + 1] = sat_add(output[i * 2 + 1], buf[ptr + i * 2 + 1]);
    }
    if ((ch[c].ptr += n) == sounds[index].n_frames) {
      ch[c].index = -1;
    }
  }
  pthread_mutex_unlock(&ch_lock);

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

  pthread_mutex_init(&ch_lock, NULL);
  for (int i = 0; i < sizeof ch / sizeof ch[0]; i++)
    ch[i].index = -1;

  if (ma_device_start(&dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot start device\n");
    exit(1);
  }
}

void sfx_load(const char *path)
{
  printf("Loading sound %s (%d)\n", path, n_sounds);

  ma_decoder dec;
  ma_decoder_config dec_cfg = ma_decoder_config_init(ma_format_s16, 1, 48000);
  if (ma_decoder_init_file(path, &dec_cfg, &dec) != MA_SUCCESS) {
    fprintf(stderr, "Cannot decode file %s\n", path);
    exit(1);
  }

  ma_uint64 n_frames;
  if (ma_decoder_get_length_in_pcm_frames(&dec, &n_frames) != MA_SUCCESS) {
    fprintf(stderr, "Cannot get audio length of %s\n", path);
    exit(1);
  }

  int16_t *buf = malloc(sizeof(int16_t) * 2 * n_frames);

  ma_uint64 n_frames_read;
  if (ma_decoder_read_pcm_frames(&dec, buf, n_frames, &n_frames_read) != MA_SUCCESS
      || n_frames_read != n_frames) {
    fprintf(stderr, "Error decoding file %s\n", path);
    exit(1);
  }

  printf("- Length: %u frames\n", (unsigned)n_frames);

  // No need to lock here...
  sounds[n_sounds] = (struct sound_t){
    .buf = buf,
    .n_frames = n_frames,
  };
  n_sounds++;
}

void sfx_play(int index)
{
  printf("Sound play %d\n", index);
  pthread_mutex_lock(&ch_lock);
  for (int i = 0; i < sizeof ch / sizeof ch[0]; i++) if (ch[i].index == -1) {
    ch[i].index = index;
    ch[i].ptr = 0;
    break;
  }
  pthread_mutex_unlock(&ch_lock);
}
