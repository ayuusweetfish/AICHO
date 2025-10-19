#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

bool is_keyboard(const char *device)
{
  int fd = open(device, O_RDONLY);
  if (fd == -1) return false;

  unsigned long ev;
  ioctl(fd, EVIOCGBIT(0, sizeof(ev)), &ev);

  close(fd);
  return (ev & (1 << EV_KEY));
}

void monitor_keyboard(const char *device)
{
  int fd = open(device, O_RDONLY | O_NONBLOCK);
  if (fd == -1) {
    perror("open");
    exit(1);
  }

  printf("Monitoring keyboard %s\n", device);

  struct input_event ev;

  while (1) {
    ssize_t n = read(fd, &ev, sizeof ev);
    if (n == -1) {
      if (errno == EAGAIN) {
        usleep(100000);
        continue;
      } else {
        perror("read");
        exit(1);
      }
    }
    if (n == sizeof ev && ev.type == EV_KEY && ev.value == 1) {
      printf("+%d %s\n", ev.code, device);
    }
  }

  close(fd);
}

int main()
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
      printf("%s %d\n", full_path, (int)is_keyboard(full_path));
      monitor_keyboard("/dev/input/event2");
    }
  }

  closedir(dir);
  return 0;
}
