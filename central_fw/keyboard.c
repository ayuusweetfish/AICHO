#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static bool is_keyboard(const char *device)
{
  int fd = open(device, O_RDONLY);
  if (fd == -1) return false;

  unsigned long ev;
  ioctl(fd, EVIOCGBIT(0, sizeof(ev)), &ev);

  close(fd);
  return (ev & (1 << EV_KEY));
}

static int fds[64], n_fds = 0;

void keyboard_start(void)
{
  DIR *dir = opendir("/dev/input");
  if (dir == NULL) {
    perror("opendir");
    exit(1);
  }

  struct dirent *ent;

  while ((ent = readdir(dir)) != NULL) {
    if (strncmp(ent->d_name, "event", 5) == 0) {
      char full_path[64] = "/dev/input/";
      strncat(full_path, ent->d_name, (sizeof full_path) - 1);
      if (is_keyboard(full_path)) {
        int fd = open(full_path, O_RDONLY | O_NONBLOCK);
        if (fd == -1) {
          perror("open");
          exit(1);
        }
        printf("Monitoring keyboard %s (%d)\n", full_path, fd);
        if (n_fds < sizeof fds / sizeof(int))
          fds[n_fds++] = fd;
      }
    }
  }

  closedir(dir);
}

void keyboard_update(void (*callback)(int))
{
  struct input_event ev;

  // XXX: We can use poll()
  for (int i = 0; i < n_fds; i++) {
    while (1) {
      ssize_t n = read(fds[i], &ev, sizeof ev);
      if (n == -1) {
        if (errno == EAGAIN) {
          break;
        } else {
          perror("read");
          exit(1);
        }
      }
      if (n == sizeof ev && ev.type == EV_KEY && ev.value == 1) {
        callback(ev.code);
      }
    }
  }
}
