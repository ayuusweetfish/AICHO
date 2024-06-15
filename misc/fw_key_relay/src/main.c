#include "main.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/watchdog.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "tusb.h"

#define PRODUCTION 1

// ============ Debug output ============

static inline void my_putc(uint8_t c)
{
#if !PRODUCTION
  if (c == '\n') uart_putc_raw(uart0, '\r');
  uart_putc_raw(uart0, c);
#endif
}
int my_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) my_putc(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) my_putc('.');
    my_putc('\n');
  }
  return 0;
}

// ============ Entry point ============

int main()
{
  // 132 MHz
  // python pico-sdk/src/rp2_common/hardware_clocks/scripts/vcocalc.py 132
  set_sys_clock_pll(1584000000, 6, 2);

  gpio_init(act_1);
  gpio_set_dir(act_1, GPIO_OUT);
  gpio_init(act_2);
  gpio_set_dir(act_2, GPIO_OUT);
  gpio_put(act_2, 1);

  uart_init(uart0, 115200);
  gpio_set_function(16, GPIO_FUNC_UART);
  gpio_set_function(17, GPIO_FUNC_UART);

  if (watchdog_caused_reboot()) my_printf("Rebooted by watchdog\n");

  my_printf("sys clk %u\n", clock_get_hz(clk_sys));

  tuh_init(BOARD_TUH_RHPORT);

  // watchdog_enable(2000, true);

  while (1) {
    tuh_task();
    // watchdog_update();
    __wfe();
  }
}
