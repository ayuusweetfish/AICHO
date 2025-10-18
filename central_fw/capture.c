// gcc -O2 -DMINIAUDIO_IMPLEMENTATION -c -x c miniaudio.h
// make -C kissfft KISSFFT_DATATYPE=int32_t KISSFFT_STATIC=1
// gcc -std=c99 capture.c miniaudio.o -Ikissfft kissfft/libkissfft-int32_t.a -lm

#include "miniaudio.h"

#include "breath_detector.h"

#include <stdio.h>

static struct breath_detector d;

int main_capture(const char *device_name)
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
    if (device_name != NULL && strstr(in_dev_infos[i].name, device_name))
      sel_id = &in_dev_infos[i].id;
  }

  ma_encoder enc;

  void input_cb(ma_device* dev, void *_output, const void *_input, ma_uint32 n_frames)
  {
    const int16_t *input = (const int16_t *)_input;
    breath_detector_feed(&d, input, n_frames);
    (void)_output;

    ma_uint64 n_frames_written;
    if (ma_encoder_write_pcm_frames(&enc, input, n_frames, &n_frames_written) != MA_SUCCESS) {
      fprintf(stderr, "Error encoding\n");
    }
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

  const char *enc_file = "/dev/shm/1.wav";
  ma_encoder_config enc_cfg = ma_encoder_config_init(
    ma_encoding_format_wav, ma_format_s16, 1, 48000
  );
  if (ma_encoder_init_file(enc_file, &enc_cfg, &enc) != MA_SUCCESS) {
    fprintf(stderr, "Cannot initialize encoder\n");
    return 1;
  }

  if (ma_device_start(&dev) != MA_SUCCESS) {
    fprintf(stderr, "Cannot start device\n");
    return 1;
  }

  printf("Writing recorded audio to %s\n", enc_file);
  printf("Running. Press Enter to stop\n");
  getchar();

  ma_encoder_uninit(&enc);

  return 0;
}

int main_file(const char *path)
{
  ma_decoder dec;
  ma_decoder_config dec_cfg = ma_decoder_config_init(ma_format_s16, 1, 48000);

  if (ma_decoder_init_file(path, &dec_cfg, &dec) != MA_SUCCESS) {
    fprintf(stderr, "Cannot decode file %s\n", path);
    return 1;
  }

  static int16_t a[16384];
  static const int a_count = sizeof a / sizeof a[0];
  ma_uint64 n_frames_read;
  do {
    if (ma_decoder_read_pcm_frames(&dec, a, a_count, &n_frames_read) != MA_SUCCESS) {
      fprintf(stderr, "Error during decoding\n");
      return 1;
    }
    breath_detector_feed(&d, a, n_frames_read);
  } while (n_frames_read == a_count);

  return 0;
}

int main(int argc, char *argv[])
{
  if (argc >= 3 && strcmp(argv[1], "-f") == 0) {
    return main_file(argv[2]);
  }
  if (argc >= 2) {
    return main_capture(argv[1]);
  }
  return main_capture(NULL);
}
