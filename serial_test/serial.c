#include <stddef.h>
#include <stdint.h>

int serial_open(const char *device);
int serial_close(int fd);
int serial_read(int fd, uint8_t *buf, size_t buf_size);
int serial_write(int fd, const uint8_t *buf, size_t buf_size);

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_BAUD_RATE B115200
#define SERIAL_MODE (CS8 | CLOCAL | CREAD)  // 8N1, no parity, 1 stop bit

// Open the serial port with the given device path
int serial_open(const char *device) {
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
int serial_close(int fd) {
  return close(fd);
}

// Read data from the serial port
int serial_read(int fd, uint8_t *buf, size_t buf_size) {
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
int serial_write(int fd, const uint8_t *buf, size_t buf_size) {
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
