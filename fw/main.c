#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"

#include "i2s.pio.h"

static inline void my_putc(uint8_t c)
{
  if (c == '\n') uart_putc_raw(uart0, '\r');
  uart_putc_raw(uart0, c);
}
void my_printf(const char *restrict fmt, ...)
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
}

int main()
{
  // 132 MHz
  // python pico-sdk/src/rp2_common/hardware_clocks/scripts/vcocalc.py 132
  set_sys_clock_pll(1584000000, 6, 2);

  const uint32_t LED_PIN = 22;
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  gpio_init(23);
  gpio_set_dir(23, GPIO_OUT);
  gpio_put(23, 1);

  uart_init(uart0, 115200);
  gpio_set_function(16, GPIO_FUNC_UART);
  gpio_set_function(17, GPIO_FUNC_UART);

  my_printf("sys clk %u\n", clock_get_hz(clk_sys));

/*
  tuh_init(BOARD_TUH_RHPORT);
  while (1) {
    tuh_task();
  }
  while (1) { }
*/

  uint offset = pio_add_program(pio0, &i2s_program);
  uint sm = pio_claim_unused_sm(pio0, true);
  i2s_program_init(pio0, sm, offset, 2, 3);

  pio_sm_set_enabled(pio0, sm, true);

  uint32_t count = 0;
  gpio_set_dir(9, GPIO_IN);

  uint32_t seed = 1;

  while (1) {
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    uint16_t sample = (seed >> 5) ^ (seed >> 11);
    sample >>= 6;
    pio_sm_put_blocking(pio0, sm, ((uint32_t)sample << 16) | sample);
    if (++count == 48000) {
      static int parity = 0;
      gpio_put(LED_PIN, parity ^= 1);
      gpio_put(23, 0);
      my_printf("second! read = %u\n", gpio_get(9));
      count = 0;
    }
  }

  while (1) {
    gpio_put(LED_PIN, 1); sleep_ms(100);
    gpio_put(LED_PIN, 0); sleep_ms(400);
    static uint32_t i = 0;
    i++;
    my_printf("Hello, UART %u!%c", i, i % 2 == 0 ? '\n' : '\t');
  }
}
