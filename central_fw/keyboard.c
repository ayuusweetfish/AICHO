#include <dirent.h>
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

int main()
{
  DIR *dir = opendir("/dev/input");
  if (dir == NULL) perror("opendir");

  struct dirent *ent;
  while ((ent = readdir(dir)) != NULL) {
    if (strncmp(ent->d_name, "event", 5) == 0) {
      char full_path[64] = "/dev/input/";
      strncat(full_path, ent->d_name, (sizeof full_path) - 1);
      printf("%s %d\n", full_path, (int)is_keyboard(full_path));
    }
  }

  closedir(dir);
  return 0;
}
