// cc main.c serial.c -O2 && ./a.out /dev/serial/by-id/*

#include <stddef.h>
#include <stdint.h>

int serial_open(const char *device);
int serial_close(int fd);
int serial_read(int fd, uint8_t *buf, size_t buf_size);
int serial_write(int fd, const uint8_t *buf, size_t buf_size);

#include <stdio.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
  if (argc < 2) {
    fprintf(stderr, "%s <serial device>\n", argv[0]);
    return 1;
  }

  int fd = serial_open(argv[1]);

  while (1) {
    puts("!");
    serial_write(fd, (uint8_t []){0x01, 0x01, 0x00, 0xAA}, 4);
    usleep(1000000);

    uint8_t a[64];
    int n = serial_read(fd, a, sizeof a);
    printf("[%2d]", n);
    for (int i = 0; i < n; i++) printf(" %02x", (int)a[i]);
    printf("\n");
  }

  serial_close(fd);

  return 0;
}
