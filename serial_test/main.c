// cc main.c serial.c -O2 && ./a.out /dev/serial/by-id/*
// ./a.out /dev/ttyAMA3

#include <stddef.h>
#include <stdint.h>

int serial_open(const char *device);
int serial_close(int fd);
int serial_read(int fd, uint8_t *buf, size_t buf_size);
int serial_write(int fd, const uint8_t *buf, size_t buf_size);

#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include "crc32.h"

static inline void tx(int fd, const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  for (int i = 0; i < len + 4; i++) {
    uint8_t x = (i < len ? buf[i] : s8[i - len]);
    if (x == 0xAA || x == 0x55) {
      assert(serial_write(fd, (uint8_t []){0x55, x ^ 0xF0}, 2) == 2);
    } else {
      assert(serial_write(fd, (uint8_t []){x}, 1) == 1);
    }
  }
  assert(serial_write(fd, (uint8_t []){0xAA}, 1) == 1);

  printf("> [%2u]", (unsigned)len);
  for (int i = 0; i < len; i++) printf(" %02x", (unsigned)buf[i]);
  printf(" |");
  for (int i = 0; i < 4; i++) printf(" %02x", (unsigned)s8[i]);
  putchar('\n');
}

int main(int argc, char *argv[])
{
  if (argc < 2) {
    fprintf(stderr, "%s <serial device>\n", argv[0]);
    return 1;
  }

  int fd = serial_open(argv[1]);

  while (1) {
    static uint32_t l = 0;
    l = (l + 32) % 4096;
    printf("intensity = %u\n", l);
    if (0) {
      tx(fd, (uint8_t []){0x01, l >> 8, l & 0xFF, 0x00, 0x00}, 5);
    } else {

  // test bladder: 150, 150, 7500, 8700, 2000
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

      int index = 3;

      static int n = 0;
      n++;
      if (n == 1) {
        tx(fd, (uint8_t []){0xCF}, 1); usleep(1000000);
        tx(fd, (uint8_t []){
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
        tx(fd, (uint8_t []){0xAF}, 1); usleep(3000000);
      }
      tx(fd, (uint8_t []){0xA1}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xA2}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xA1}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xA2}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xA1}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xAF}, 1); usleep(3000000);
      tx(fd, (uint8_t []){0xAE}, 1); usleep(6000000);
    }
    usleep(20000);

    uint8_t a[64];
    int n = serial_read(fd, a, sizeof a);
    printf("< [%2d]", n);
    for (int i = 0; i < n; i++) printf(" %02x", (int)a[i]);
    printf("\n");
  }

  serial_close(fd);

  return 0;
}
