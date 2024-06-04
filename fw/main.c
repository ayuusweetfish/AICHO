#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"

#include "i2s.pio.h"

static inline void my_putc(uint8_t c)
{
  if (c == '\n') uart_putc_raw(uart0, '\r');
  uart_putc_raw(uart0, c);
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

static uint32_t audio_buf[9600];

static inline void refill_buffer(int half);
static void dma_irq0_handler();

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

#define GAIN 0.5  // 16 for 4Ω speaker, 1 for headphone

  for (int i = 0; i < 9600; i++) {
    int16_t sample = (i < 4800 ?
      (int16_t)(0.5f + GAIN * 256 * sin((float)(i % 100) / 100 * (float)M_PI * 2)) :
      (int16_t)(0.5f + GAIN * 256 * sin((float)(i %  50) /  50 * (float)M_PI * 2)));
    audio_buf[i] = ((uint32_t)sample << 16) | (uint16_t)sample;
  }

  const uint32_t half_size = (sizeof audio_buf) / (sizeof audio_buf[0]) / 2;

  dma_channel_config dma_ch0 = dma_channel_get_default_config(0);
  channel_config_set_read_increment(&dma_ch0, true);
  channel_config_set_write_increment(&dma_ch0, false);
  channel_config_set_dreq(&dma_ch0, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch0, 2);
  dma_channel_set_irq0_enabled(0, true);
  dma_channel_configure(0, &dma_ch0, &pio0->txf[sm], audio_buf, half_size, false);

  dma_channel_config dma_ch1 = dma_channel_get_default_config(1);
  channel_config_set_read_increment(&dma_ch1, true);
  channel_config_set_write_increment(&dma_ch1, false);
  channel_config_set_dreq(&dma_ch1, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch1, 3);
  dma_channel_set_irq0_enabled(1, true);
  dma_channel_configure(1, &dma_ch1, &pio0->txf[sm], audio_buf + half_size, half_size, false);

  irq_set_exclusive_handler(11, dma_irq0_handler);
  irq_set_enabled(11, true);

  static uint32_t addr0 = (uint32_t)&audio_buf[0];
  static uint32_t addr1 = (uint32_t)&audio_buf[half_size];

  dma_channel_config dma_ch2 = dma_channel_get_default_config(2);
  channel_config_set_read_increment(&dma_ch2, false);
  channel_config_set_write_increment(&dma_ch2, false);
  dma_channel_configure(2, &dma_ch2, &dma_hw->ch[1].al3_read_addr_trig, &addr1, 1, false);

  dma_channel_config dma_ch3 = dma_channel_get_default_config(3);
  channel_config_set_read_increment(&dma_ch3, false);
  channel_config_set_write_increment(&dma_ch3, false);
  dma_channel_configure(3, &dma_ch3, &dma_hw->ch[0].al3_read_addr_trig, &addr0, 1, false);

  // Start channel 0 (the first half)
  dma_channel_set_config(0, &dma_ch0, true);

  while (1) {
    gpio_put(LED_PIN, 1); sleep_ms(100);
    gpio_put(LED_PIN, 0); sleep_ms(400);
    static uint32_t i = 0;
    i++;
    my_printf("Hello, UART %u!%c", i, i % 2 == 0 ? '\n' : '\t');
  }
}

void fill_buffer(int half)
{
  static uint32_t seed = 1;
  seed = seed * 1103515245 + 12345;
  uint32_t period = ((seed >> 3) ^ (seed >> 9)) % 50 + 50;
  for (int i = 0; i < 4800; i++) {
    int16_t sample =
      (int16_t)(0.5f + GAIN * 256 * sin((float)i / period * (float)M_PI * 2));
    audio_buf[4800 * half + i] = ((uint32_t)sample << 16) | (uint16_t)sample;
  }
}

void dma_irq0_handler()
{
  if (dma_channel_get_irq0_status(0)) {
    dma_channel_acknowledge_irq0(0);
    gpio_put(23, 0);
    fill_buffer(0);
  } else {
    dma_channel_acknowledge_irq0(1);
    gpio_put(23, 1);
    fill_buffer(1);
  }
}
