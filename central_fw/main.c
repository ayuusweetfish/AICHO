// cc main.c keyboard.c serial.c /dev/shm/a.out && /dev/shm/a.out

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

void keyboard_start(void (*callback)(int));
void serial_start(const char *devices[], int n_devices);
void serial_tx(int index, const uint8_t *buf, int len);

void keyboard_callback(int n)
{
  if (n >= 16 && n <= 19) printf("%d\n", n);
}

int main()
{
  keyboard_start(keyboard_callback);

  serial_start((const char *[4]){
    "/dev/ttyS2",
    "/dev/ttyS1",
    "/dev/ttyS5",
    "/dev/ttyS3",
  }, 4);

  typedef struct {
    int PUMP_INFLATE_DUTY;
    int PUMP_DRAIN_DUTY;
    int PRESSURE_LIMIT;
    int PRESSURE_BAIL;
    int INFLATE_TIME_LIMIT;
  } args_t;
  static const args_t args[4] = {
    {
      .PUMP_INFLATE_DUTY = 40,
      .PUMP_DRAIN_DUTY = 20,
      .PRESSURE_LIMIT = 9000,
      .PRESSURE_BAIL = 9500,
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
      .PUMP_INFLATE_DUTY = 60,
      .PUMP_DRAIN_DUTY = 40,
      .PRESSURE_LIMIT = 6800,
      .PRESSURE_BAIL = 7500,
      .INFLATE_TIME_LIMIT = 2000,
    },
    {
      .PUMP_INFLATE_DUTY = 133,
      .PUMP_DRAIN_DUTY = 200,
      .PRESSURE_LIMIT = 7900,
      .PRESSURE_BAIL = 8300,
      .INFLATE_TIME_LIMIT = 2000,
    },
  };

  int index = 1;

  while (1) {
    static int n = 0;
    n++;
    if (n == 1) {
      serial_tx(index, (uint8_t []){0xCF}, 1); usleep(1000000);
      serial_tx(index, (uint8_t []){
        0x10 + index,
        args[index].PUMP_INFLATE_DUTY,
        args[index].PUMP_DRAIN_DUTY,
        args[index].PRESSURE_LIMIT >> 8, args[index].PRESSURE_LIMIT & 0xff,
        args[index].PRESSURE_BAIL >> 8, args[index].PRESSURE_BAIL & 0xff,
        args[index].INFLATE_TIME_LIMIT >> 8, args[index].INFLATE_TIME_LIMIT & 0xff,
      }, 9);
    }
    usleep(3000000);
    if (n % 2 == 0) {
      serial_tx(index, (uint8_t []){0xAF}, 1); usleep(3000000);
    }
    serial_tx(index, (uint8_t []){0xA1}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xA2}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xA1}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xA2}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xA1}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xAF}, 1); usleep(3000000);
    serial_tx(index, (uint8_t []){0xAE}, 1); usleep(6000000);
  }

  while (1) usleep(1000000), puts("!");
  return 0;
}
