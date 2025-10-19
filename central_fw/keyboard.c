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

struct monitor_args_t {
  const int *fds;
  int n_fds;
  void (*callback)(int);
};

static void *monitor_keyboard_loop(void *_args)
{
  struct monitor_args_t *args = _args;
  const int *fds = args->fds;
  int n_fds = args->n_fds;
  void (*callback)(int) = args->callback;
  free(args);

  struct input_event ev;

  while (1) {
    bool has_read = false;
    // XXX: We can use poll()
    for (int i = 0; i < n_fds; i++) {
      ssize_t n = read(fds[i], &ev, sizeof ev);
      if (n == -1) {
        if (errno == EAGAIN) {
          continue;
        } else {
          perror("read");
          exit(1);
        }
      }
      if (n == sizeof ev && ev.type == EV_KEY && ev.value == 1) {
        has_read = true;
        callback(ev.code);
      }
    }
    if (!has_read) usleep(10000);
  }

  return NULL;
}

static void monitor_keyboard(const int *fds, int n_fds, void (*callback)(int))
{
  pthread_t thr;
  struct monitor_args_t *args = malloc(sizeof(struct monitor_args_t));
  int *fds_clone = malloc(sizeof(int) * n_fds);
  memcpy(fds_clone, fds, sizeof(int) * n_fds);
  *args = (struct monitor_args_t){
    .fds = fds_clone,
    .n_fds = n_fds,
    .callback = callback,
  };
  if (pthread_create(&thr, NULL, monitor_keyboard_loop, args) != 0) {
    perror("pthread_create");
    exit(1);
  }
}

void keyboard_start(void (*callback)(int))
{
  DIR *dir = opendir("/dev/input");
  if (dir == NULL) {
    perror("opendir");
    exit(1);
  }

  struct dirent *ent;
  int fds[64], n_fds = 0;

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

  monitor_keyboard(fds, n_fds, callback);
}
