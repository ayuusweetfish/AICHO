#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"

#include "i2s.pio.h"

#define uQOA_IMPL
#include "uqoa.h"

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

extern uint32_t __etext;
#define FLASH_BASE  0x10000000
#define AUXDAT_ADDR 0x00100000 // 1 MiB
#define AUXDAT(_offs) (void *)(FLASH_BASE + AUXDAT_ADDR + (_offs))

uint8_t flash_test_write_buf[256 * 32];

__attribute__ ((used))
void flash_erase_64k(uint32_t addr)
{
  flash_range_erase(AUXDAT_ADDR + addr, 65536);
}

__attribute__ ((used))
void flash_test_write(uint32_t addr, size_t size)
{
  flash_range_program(AUXDAT_ADDR + addr, flash_test_write_buf, sizeof flash_test_write_buf);
}

static int16_t audio_buf[2400];
static const uint32_t audio_buf_half_size = (sizeof audio_buf) / (sizeof audio_buf[0]) / 2;

static inline void refill_buffer(int half);
static void dma_irq0_handler();

struct sampler {
  uint32_t start, len;
  qoa_lms qoa_state;
  uint32_t ptr;
};

static inline void sampler_decode(struct sampler *s, int16_t out[20])
{
  if (s->ptr % (258 * 8) == 0) {
    uint64_t h = __builtin_bswap64(*(uint64_t *)AUXDAT(s->start + s->ptr));
    uint64_t w = __builtin_bswap64(*(uint64_t *)AUXDAT(s->start + s->ptr + 8));
    for (int i = 0; i < 4; i++)
      s->qoa_state.history[i] = (int16_t)(h >> (16 * (3 - i)));
    for (int i = 0; i < 4; i++)
      s->qoa_state.weights[i] = (int16_t)(w >> (16 * (3 - i)));
    s->ptr += 16;
  /*
    my_printf("reload! %016llx %016llx\n", h, w);
    my_printf("%08x %08x %08x %08x\n",
      (uint32_t)s->qoa_state.weights[0],
      (uint32_t)s->qoa_state.weights[1],
      (uint32_t)s->qoa_state.weights[2],
      (uint32_t)s->qoa_state.weights[3]
    );
  */
  }
  if (s->ptr >= s->len) s->ptr = 0;
  uint64_t slice = __builtin_bswap64(*(uint64_t *)AUXDAT(s->start + s->ptr));
  // my_printf("decode slice %016llx\n", slice);
  qoa_decode_slice(&s->qoa_state, slice, out);
  s->ptr += 8;
}

struct sampler s1;

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

  my_printf("__etext is %08x\n", &__etext);
  assert((uint32_t)&__etext <= FLASH_BASE + AUXDAT_ADDR);

  my_printf("flash_erase_64k()  at %08x\n", &flash_erase_64k);
  my_printf("flash_test_write() at %08x\n", &flash_test_write);

  // my_printf("word of aux data %08x\n", *(uint32_t *)(FLASH_BASE + AUXDAT_ADDR + 104));

#define FILE_ADDR_zq3jTYLUbiEf_128_bin 0
#define FILE_SIZE_zq3jTYLUbiEf_128_bin 1161008
  s1 = (struct sampler){
    .start = FILE_ADDR_zq3jTYLUbiEf_128_bin,
    .len = FILE_SIZE_zq3jTYLUbiEf_128_bin,
    .ptr = 0,
  };

/*
  int16_t samples[20];
  for (int i = 0; i < 259; i++) {
    sampler_decode(&s1, samples);
    if (1) for (int j = 0; j < 20; j++)
      my_printf("%6d%c", (int)samples[j], j == 19 ? '\n' : ' ');
  }
  while (1) { }
*/

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

#define GAIN 4  // 16 for 4Ω speaker, 1 for headphone

  for (int i = 0; i < audio_buf_half_size * 2; i++) {
    int16_t sample = (int16_t)(0.5f + GAIN * 256 * sin((float)(i % 100) / 100 * (float)M_PI * 2));
    audio_buf[i] = sample;
  }

  dma_channel_config dma_ch0 = dma_channel_get_default_config(0);
  channel_config_set_read_increment(&dma_ch0, true);
  channel_config_set_write_increment(&dma_ch0, false);
  channel_config_set_transfer_data_size(&dma_ch0, DMA_SIZE_16);
  channel_config_set_dreq(&dma_ch0, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch0, 2);
  dma_channel_set_irq0_enabled(0, true);
  dma_channel_configure(0, &dma_ch0, &pio0->txf[sm], audio_buf, audio_buf_half_size, false);

  dma_channel_config dma_ch1 = dma_channel_get_default_config(1);
  channel_config_set_read_increment(&dma_ch1, true);
  channel_config_set_write_increment(&dma_ch1, false);
  channel_config_set_transfer_data_size(&dma_ch1, DMA_SIZE_16);
  channel_config_set_dreq(&dma_ch1, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch1, 3);
  dma_channel_set_irq0_enabled(1, true);
  dma_channel_configure(1, &dma_ch1, &pio0->txf[sm], audio_buf + audio_buf_half_size, audio_buf_half_size, false);

  irq_set_exclusive_handler(11, dma_irq0_handler);
  irq_set_enabled(11, true);

  static uint32_t addr0 = (uint32_t)&audio_buf[0];
  static uint32_t addr1 = (uint32_t)&audio_buf[audio_buf_half_size];

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
  /*
    if (i >= 5 && i % 5 == 0) {
      channel_config_set_enable(&dma_ch0, false);
      dma_channel_set_config(0, &dma_ch0, false);
    } else if (i >= 5 && i % 5 == 1) {
      channel_config_set_enable(&dma_ch0, true);
      dma_channel_configure(0, &dma_ch0, &pio0->txf[sm], audio_buf, audio_buf_half_size, true);
    }
  */
  }
}

void fill_buffer(bool half)
{
/*
  static uint32_t count = 0;
  static uint32_t seed = 1;
  if (++count == 4) {
    count = 0;
    seed = seed * 1103515245 + 12345;
  }
  uint32_t period = ((seed >> 3) ^ (seed >> 9)) % 50 + 50;
  for (int i = 0; i < audio_buf_half_size; i++) {
    int16_t sample =
      (int16_t)(0.5f + GAIN * 256 * sin((float)(i + count * audio_buf_half_size) / period * (float)M_PI * 2));
    audio_buf[audio_buf_half_size * (int)half + i] = sample;
  }
*/
  int16_t *buf = audio_buf + (audio_buf_half_size * (int)half);
  assert(audio_buf_half_size % 20 == 0);
  for (int i = 0; i < audio_buf_half_size; i += 20) {
    sampler_decode(&s1, buf + i);
    for (int j = 0; j < 20; j++)
      buf[i + j] >>= 7;
  }
}

void dma_irq0_handler()
{
  if (dma_channel_get_irq0_status(0)) {
    dma_channel_acknowledge_irq0(0);
    fill_buffer(0);
  } else {
    dma_channel_acknowledge_irq0(1);
    fill_buffer(1);
  }
}
