// cc main.c serial.c -O2 && ./a.out /dev/serial/by-id/*

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
  uint8_t s8[5] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
    0xAA,
  };

  int n_tx;

  n_tx = serial_write(fd, buf, len);
  assert(n_tx == len);

  n_tx = serial_write(fd, s8, 5);
  assert(n_tx == 5);

  printf("> [%2u]", (unsigned)len);
  for (int i = 0; i < len; i++) printf(" %02x", (unsigned)buf[i]);
  printf(" |");
  for (int i = 0; i < 5; i++) printf(" %02x", (unsigned)s8[i]);
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
    tx(fd, (uint8_t []){0x01, l >> 8, l & 0xFF}, 3);
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
