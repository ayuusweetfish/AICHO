// cc main.c keyboard.c serial.c /dev/shm/a.out && /dev/shm/a.out

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

void keyboard_start(void);
void keyboard_update(void (*callback)(int));
void serial_start(const char *devices[], int n_devices);
void serial_tx(int index, const uint8_t *buf, int len);

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

int main()
{
  keyboard_start();

  serial_start((const char *[4]){
    "/dev/ttyS2",
    "/dev/ttyS1",
    "/dev/ttyS5",
    "/dev/ttyS3",
  }, 4);

  while (0) {
    void keyboard_callback(int n) {
      if (n >= 16 && n <= 19) printf("%d\n", n);
    }
    keyboard_update(keyboard_callback);
    usleep(10000);
  }

  int index = 3;

  while (1) {
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
