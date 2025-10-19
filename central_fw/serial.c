#include <stddef.h>
#include <stdint.h>

static int serial_open(const char *device);
static int serial_close(int fd);
static int serial_read(int fd, uint8_t *buf, size_t buf_size);
static int serial_write(int fd, const uint8_t *buf, size_t buf_size);

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_BAUD_RATE B115200
#define SERIAL_MODE (CS8 | CLOCAL | CREAD)  // 8N1, no parity, 1 stop bit

// Open the serial port with the given device path
static int serial_open(const char *device) {
  int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    perror("serial_open");
    return -1;
  }

  struct termios tty;
  if (tcgetattr(fd, &tty) != 0) {
    perror("tcgetattr");
    close(fd);
    return -1;
  }

  cfsetospeed(&tty, SERIAL_BAUD_RATE);
  cfsetispeed(&tty, SERIAL_BAUD_RATE);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8-bit chars
  tty.c_cflag &= ~HUPCL;                      // no hang-up on close
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);     // no software flow control
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // raw input
  tty.c_oflag &= ~OPOST;                      // raw output
  tty.c_cc[VMIN]  = 1;                        // read blocks until at least 1 byte
  tty.c_cc[VTIME] = 0;                        // no timeout

  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    perror("tcsetattr");
    close(fd);
    return -1;
  }

  return fd;
}

// Close the serial port
static int serial_close(int fd) {
  return close(fd);
}

// Read data from the serial port
static int serial_read(int fd, uint8_t *buf, size_t buf_size) {
  int n = read(fd, buf, buf_size);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK)
      return 0;

    perror("serial_read");
    return -1;
  }
  return n;
}

// Write data to the serial port
static int serial_write(int fd, const uint8_t *buf, size_t buf_size) {
  size_t p = 0;
  while (p < buf_size) {
    ssize_t n = write(fd, buf + p, buf_size - p);
    if (n < 0) {
      perror("serial_write");
      return -1;
    }
    p += n;
  }
  return buf_size;
}

#include <assert.h>
#include <stdio.h>
#include <unistd.h>

#include "crc32.h"

static void tx(int fd, const uint8_t *buf, uint8_t len)
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

  printf("> %2u [%2u]", fd, (unsigned)len);
  for (int i = 0; i < len; i++) printf(" %02x", (unsigned)buf[i]);
  printf(" |");
  for (int i = 0; i < 4; i++) printf(" %02x", (unsigned)s8[i]);
  putchar('\n');
}

static int fds[8];

void serial_start(const char *devices[], int n_devices)
{
  for (int i = 0; i < n_devices; i++) {
    fds[i] = serial_open(devices[i]);
    printf("Opened serial %s (%d)\n", devices[i], fds[i]);
  }
}

void serial_tx(int index, const uint8_t *buf, int len)
{
  tx(fds[index], buf, len);
}
