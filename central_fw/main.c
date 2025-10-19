// cc main.c keyboard.c serial.c /dev/shm/a.out && /dev/shm/a.out

#include <stdio.h>
#include <unistd.h>

void keyboard_start(void (*callback)(int));
void serial_start(const char *devices[], int n_devices);
void serial_tx(int index, const void *buf, int len);

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

  while (1) usleep(1000000), puts("!");
  return 0;
}
