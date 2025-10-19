// cc main.c keyboard.c /dev/shm/a.out && /dev/shm/a.out

#include <stdio.h>
#include <unistd.h>

void keyboard_start(void (*callback)(int));

void keyboard_callback(int n)
{
  if (n >= 16 && n <= 19) printf("%d\n", n);
}

int main()
{
  keyboard_start(keyboard_callback);

  while (1) usleep(1000000), puts("!");
  return 0;
}
