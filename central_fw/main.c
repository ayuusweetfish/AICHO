// cc -O2 -DMINIAUDIO_IMPLEMENTATION -c -x c miniaudio.h
// make -C kissfft KISSFFT_DATATYPE=int32_t KISSFFT_STATIC=1

// cc main.c keyboard.c serial.c microphone.c sfx.c miniaudio.o -Ikissfft kissfft/libkissfft-int32_t.a -lm -o /dev/shm/a.out && /dev/shm/a.out

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

void keyboard_start(void);
void keyboard_update(void (*callback)(int));
void serial_start(const char *devices[], int n_devices);
void serial_tx(int index, const uint8_t *buf, int len);
void microphone_start(const char *device_name);
bool microphone_breath_state(void);
void sfx_start(const char *device_name);
void sfx_load(const char *path);
void sfx_play(int index);

typedef struct {
  int PUMP_INFLATE_DUTY;
  int PUMP_DRAIN_DUTY;
  int PRESSURE_LIMIT;
  int PRESSURE_BAIL;
  int INFLATE_TIME_LIMIT;
} args_t;
static const args_t args[4] = {
  {
    .PUMP_INFLATE_DUTY = 60,
    .PUMP_DRAIN_DUTY = 20,
    .PRESSURE_LIMIT = 11500,
    .PRESSURE_BAIL = 12000,
    .INFLATE_TIME_LIMIT = 1800,
  },
  {
    .PUMP_INFLATE_DUTY = 0,
    .PUMP_DRAIN_DUTY = 0,
    .PRESSURE_LIMIT = 0,
    .PRESSURE_BAIL = 0,
    .INFLATE_TIME_LIMIT = 0,
  },
  {
    .PUMP_INFLATE_DUTY = 100,
    .PUMP_DRAIN_DUTY = 120,
    .PRESSURE_LIMIT = 13000,
    .PRESSURE_BAIL = 14000,
    .INFLATE_TIME_LIMIT = 2000,
  },
  {
    .PUMP_INFLATE_DUTY = 150,
    .PUMP_DRAIN_DUTY = 200,
    .PRESSURE_LIMIT = 15000,
    .PRESSURE_BAIL = 16000,
    .INFLATE_TIME_LIMIT = 2000,
  },
};

static void lane_reset(int index)
{
  serial_tx(index, (uint8_t []){0xCF}, 1);
}
static void lane_initiate(int index)
{
  serial_tx(index, (uint8_t []){
    0x10 + index,
    args[index].PUMP_INFLATE_DUTY,
    args[index].PUMP_DRAIN_DUTY,
    args[index].PRESSURE_LIMIT >> 8, args[index].PRESSURE_LIMIT & 0xff,
    args[index].PRESSURE_BAIL >> 8, args[index].PRESSURE_BAIL & 0xff,
    args[index].INFLATE_TIME_LIMIT >> 8, args[index].INFLATE_TIME_LIMIT & 0xff,
  }, 9);
}
static void lane_inflate(int index)
{
  serial_tx(index, (uint8_t []){0xA1}, 1);
}
static void lane_drain(int index)
{
  serial_tx(index, (uint8_t []){0xA2}, 1);
}
static void lane_fade_out(int index)
{
  serial_tx(index, (uint8_t []){0xAF}, 1);
}
static void lane_reappear(int index)
{
  serial_tx(index, (uint8_t []){0xAE}, 1);
}

static const int SOUND_ENSEMBLE = 4;
static const int SOUND_INHALE = 0;
static const int SOUND_EXHALE = 1;
static void load_sounds()
{
  const char *names[] = {
    "Lorivox", "Lumisonic", "Harmonia", "Titanus", "Ensemble",
  };
  const char *directions[] = {
    "Inhale", "Exhale",
  };
  for (int i = 0; i <= 4; i++) {
    for (int j = 0; j <= 1; j++) {
      char path[64];
      snprintf(path, sizeof path, "sfx/%02d_%s_%s.wav", i, names[i], directions[j]);
      sfx_load(path);
    }
  }
}
static void play_sound(int index, int direction)
{
  sfx_play(index * 2 + direction);
}

int main()
{
  serial_start((const char *[4]){
    "/dev/ttyS2", // PI9,10
    "/dev/ttyS1", // PI5,6
    "/dev/ttyS5", // PH2,3
    "/dev/ttyS3", // PI13,14
  }, 4);
  if (0) goto test;

  for (int i = 0; i < 4; i++) lane_reset(i);
  usleep(1500000);
  for (int i = 0; i < 4; i++) lane_initiate(i);

  keyboard_start();
  void clear_keyboard_queue() {
    void cb(int n) { }
    keyboard_update(cb);
  }

  microphone_start("(Unitek Y-247A) Mono");

  sfx_start("(Unitek Y-247A) Analog Stereo");
  load_sounds();

  puts("Entering loop");

  enum {
    STATE_IDLE,
    STATE_BREATHE,
  } state = STATE_IDLE;

  int act_organism = 0;

  int since_last_exhale = 0;  // Milliseconds
  int exhale_total_time = 0;
  int exhale_interval = 0;
  int exhale_count = 0;
  bool ensemble_inhaled = false;

  static const int LOOP_INTERVAL = 10;

  #define min(_a, _b) ((_a) < (_b) ? (_a) : (_b))
  #define max(_a, _b) ((_a) > (_b) ? (_a) : (_b))

  while (1) {
    bool is_exhale = microphone_breath_state();

    if (state == STATE_IDLE) {
      // Get keypresses
      int keypress = -1;
      void keyboard_callback(int n) {
        if (n >= 16 && n <= 19) keypress = n - 16;
      }
      keyboard_update(keyboard_callback);
      if (keypress >= 0) {
        for (int i = 0; i < 4; i++) {
          if (i == keypress) lane_inflate(i);
          else lane_fade_out(i);
        }
        act_organism = keypress;
        since_last_exhale = 0;
        exhale_total_time = 0;
        exhale_interval = 6000;
        exhale_count = 0;
        ensemble_inhaled = false;
        state = STATE_BREATHE;
        play_sound(act_organism, SOUND_INHALE);
        usleep(1000 * 1000);
      }

    } else if (state == STATE_BREATHE) {
      since_last_exhale += LOOP_INTERVAL;
      exhale_total_time += LOOP_INTERVAL;

      // Starting from the 3-rd inhale, ensemble performance is guaranteed
      bool ensemble = (exhale_total_time >= 15000 || exhale_count >= 3);

      if (exhale_count > 0 &&
          since_last_exhale >= exhale_interval / 2 &&
          since_last_exhale - LOOP_INTERVAL < exhale_interval / 2) {
        // Inhale
        if (ensemble) {
          for (int i = 0; i < 4; i++) lane_inflate(i);
          ensemble_inhaled = true;
          play_sound(SOUND_ENSEMBLE, SOUND_INHALE);
        } else {
          lane_inflate(act_organism);
          play_sound(act_organism, SOUND_INHALE);
        }
      }

      if (microphone_breath_state() &&
          (exhale_count == 0 ||
           since_last_exhale > exhale_interval * 9 / 16)) {
        // Exhale
        exhale_interval += (
          min(since_last_exhale, exhale_interval * 2) - exhale_interval) / 2;
        exhale_interval = max(4000, min(10000, exhale_interval));
        if (exhale_count == 0) exhale_total_time = 0;
          // So that total time represents time since start of performance
        exhale_count += 1;
        since_last_exhale = 0;
        if (ensemble && ensemble_inhaled) {
          // First ensemble action should be inhalation
          for (int i = 0; i < 4; i++) lane_drain(i);
          play_sound(SOUND_ENSEMBLE, SOUND_EXHALE);
        } else {
          lane_drain(act_organism);
          play_sound(act_organism, SOUND_EXHALE);
        }

      } else if (since_last_exhale >= 15000) {
        for (int i = 0; i < 4; i++) lane_fade_out(i);
        usleep(4000 * 1000);
        for (int i = 0; i < 4; i++) lane_reappear(i);
        state = STATE_IDLE;
        usleep(4000 * 1000);
        clear_keyboard_queue();
      }

    }

    usleep(LOOP_INTERVAL * 1000);
  }

test:
  while (1) {
    int index = 3;
    static int n = 0;
    n++;
    if (n == 1) {
      lane_reset(index);
      usleep(1500000);
      lane_initiate(index);
    }
    if (n % 2 == 0) {
      getchar(); lane_fade_out(index);
    }
    getchar(); lane_inflate(index);
    getchar(); lane_drain(index);
    getchar(); lane_inflate(index);
    getchar(); lane_drain(index);
    getchar(); lane_inflate(index);
    getchar(); lane_fade_out(index);
    getchar(); lane_reappear(index);
  }

  while (1) usleep(1000000), puts("!");
  return 0;
}
