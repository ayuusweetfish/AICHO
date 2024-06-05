#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>

#include "tusb.h"

#include "i2s.pio.h"
#include "ws2812.pio.h"

#define uQOA_IMPL
#include "uqoa.h"

// ============ Debug output ============

#define act_1 22
#define act_2 23

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

// ============ Auxiliary data on flash ============

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

// ============ Audio buffer ============

static int16_t audio_buf[2400];
static const uint32_t audio_buf_half_size = (sizeof audio_buf) / (sizeof audio_buf[0]) / 2;

static inline void refill_buffer(int16_t *buf);
static void dma_irq0_handler();

static dma_channel_config dma_ch0, dma_ch1, dma_ch2, dma_ch3;

void audio_buf_init()
{
  uint offset = pio_add_program(pio0, &i2s_program);
  uint sm = pio_claim_unused_sm(pio0, true);
  i2s_program_init(pio0, sm, offset, 2, 3);

  pio_sm_set_enabled(pio0, sm, true);

  dma_ch0 = dma_channel_get_default_config(0);
  channel_config_set_read_increment(&dma_ch0, true);
  channel_config_set_write_increment(&dma_ch0, false);
  channel_config_set_transfer_data_size(&dma_ch0, DMA_SIZE_16);
  channel_config_set_dreq(&dma_ch0, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch0, 2);
  dma_channel_set_irq0_enabled(0, true);
  dma_channel_configure(0, &dma_ch0, &pio0->txf[sm], audio_buf, audio_buf_half_size, false);

  dma_ch1 = dma_channel_get_default_config(1);
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

  dma_ch2 = dma_channel_get_default_config(2);
  channel_config_set_read_increment(&dma_ch2, false);
  channel_config_set_write_increment(&dma_ch2, false);
  dma_channel_configure(2, &dma_ch2, &dma_hw->ch[1].al3_read_addr_trig, &addr1, 1, false);

  dma_ch3 = dma_channel_get_default_config(3);
  channel_config_set_read_increment(&dma_ch3, false);
  channel_config_set_write_increment(&dma_ch3, false);
  dma_channel_configure(3, &dma_ch3, &dma_hw->ch[0].al3_read_addr_trig, &addr0, 1, false);
}

void audio_buf_suspend()
{
  channel_config_set_enable(&dma_ch0, false);
  dma_channel_set_config(0, &dma_ch0, false);
}

void audio_buf_resume()
{
  channel_config_set_enable(&dma_ch0, true);
  dma_channel_set_read_addr(0, audio_buf, false);
  dma_channel_set_config(0, &dma_ch0, true);
}

void dma_irq0_handler()
{
  if (dma_channel_get_irq0_status(0)) {
    dma_channel_acknowledge_irq0(0);
    refill_buffer(audio_buf);
  } else {
    dma_channel_acknowledge_irq0(1);
    refill_buffer(audio_buf + audio_buf_half_size);
  }
}

// ============ Sampler ============

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
  }
  uint64_t slice = __builtin_bswap64(*(uint64_t *)AUXDAT(s->start + s->ptr));
  qoa_decode_slice(&s->qoa_state, slice, out);
  if ((s->ptr += 8) >= s->len) s->ptr = 0;
}

struct sampler s1;

#define POLYPHONY 8
struct polyphonic_sampler {
  critical_section_t crit;
  struct sampler s[POLYPHONY];
  bool active[POLYPHONY];
};

static inline void polyphonic_init(struct polyphonic_sampler *s)
{
  critical_section_init(&s->crit);
  for (uint8_t i = 0; i < POLYPHONY; i++) s->active[i] = 0;
}

static inline uint8_t polyphonic_trigger(
  struct polyphonic_sampler *s,
  uint32_t start, uint32_t len
) {
  critical_section_enter_blocking(&s->crit);

  // Find a voice
  uint8_t voice_id = 0xff;
  for (uint8_t i = 0; i < POLYPHONY; i++)
    if (!s->active[i]) {
      voice_id = i;
      break;
    }
  if (voice_id == 0xff) {
    // Steal a random voice
    for (uint8_t i = 0; i < POLYPHONY - 1; i++) {
      s->s[i] = s->s[i + 1];
      // `active[i]` is true
    }
    voice_id = POLYPHONY - 1;
  }

  s->s[voice_id] = (struct sampler){
    .start = start,
    .len = len,
    .ptr = 0,
  };
  s->active[voice_id] = true;

  critical_section_exit(&s->crit);
}

static inline void polyphonic_out(struct polyphonic_sampler *s, int16_t out[20])
{
  int32_t mix[20] = { 0 };
  int16_t voice[20];

  critical_section_enter_blocking(&s->crit);
  for (uint8_t i = 0; i < POLYPHONY; i++)
    if (s->active[i]) {
      sampler_decode(&s->s[i], voice);
      for (int j = 0; j < 20; j++) mix[j] += voice[j];
      if (s->s[i].ptr == 0) s->active[i] = false;
    }
  critical_section_exit(&s->crit);

  for (int j = 0; j < 20; j++)
    out[j] = mix[j] >> 7;
}

struct polyphonic_sampler ps1;

// ============ LED strip ============

static dma_channel_config dma_ch4;

void leds_init()
{
  uint offset = pio_add_program(pio1, &ws2812_program);
  uint sm = pio_claim_unused_sm(pio1, true);
  ws2812_program_init(pio1, sm, offset, 11);

  pio_sm_set_enabled(pio1, sm, true);

  dma_ch4 = dma_channel_get_default_config(4);
  channel_config_set_read_increment(&dma_ch4, true);
  channel_config_set_write_increment(&dma_ch4, false);
  dma_channel_set_config(4, &dma_ch4, false);
  dma_channel_set_write_addr(4, &pio1->txf[sm], false);

  // while (1) pio_sm_put_blocking(pio1, sm, 0xffffffff);
}

void leds_blast(int n)
{
  static uint32_t leds_buf[80];
  for (int i = 0; i < n; i++) {
    leds_buf[i] = 0xaa55ccd4;
  }

  dma_channel_set_read_addr(4, leds_buf, false);
  dma_channel_set_trans_count(4, n, true);
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

  my_printf("sys clk %u\n", clock_get_hz(clk_sys));

  my_printf("__etext is %08x\n", &__etext);
  assert((uint32_t)&__etext <= FLASH_BASE + AUXDAT_ADDR);

  my_printf("flash_erase_64k()  at %08x\n", &flash_erase_64k);
  my_printf("flash_test_write() at %08x\n", &flash_test_write);

#define FILE_ADDR_zq3jTYLUbiEf_128_bin 0
#define FILE_SIZE_zq3jTYLUbiEf_128_bin 1161008
  s1 = (struct sampler){
    .start = FILE_ADDR_zq3jTYLUbiEf_128_bin,
    .len = FILE_SIZE_zq3jTYLUbiEf_128_bin,
    .ptr = 0,
  };

  polyphonic_init(&ps1);

/*
  tuh_init(BOARD_TUH_RHPORT);
  while (1) {
    tuh_task();
  }
  while (1) { }
*/

/*
  leds_init();
  while (1) {
    leds_blast(60);
    gpio_put(act_1, 1); sleep_ms(100);
    gpio_put(act_1, 0); sleep_ms(400);
  }
*/

  audio_buf_init();
  audio_buf_resume();

  polyphonic_trigger(&ps1,
    FILE_ADDR_zq3jTYLUbiEf_128_bin,
    FILE_SIZE_zq3jTYLUbiEf_128_bin);

  while (1) {
    gpio_put(act_1, 1); sleep_ms(100);
    gpio_put(act_1, 0); sleep_ms(400);
    static uint32_t i = 0;
    i++;
    my_printf("Hello, UART %u!%c", i, i % 2 == 0 ? '\n' : '\t');
    if (i >= 10 && i % 10 == 0) {
      audio_buf_suspend();
      gpio_put(act_2, 0);
    } else if (i >= 10 && i % 10 == 1) {
      audio_buf_resume();
      gpio_put(act_2, 1);
    }
    if (i % 10 == 3)
      polyphonic_trigger(&ps1,
        FILE_ADDR_zq3jTYLUbiEf_128_bin,
        FILE_SIZE_zq3jTYLUbiEf_128_bin);
  }
}

void refill_buffer(int16_t *buf)
{
  assert(audio_buf_half_size % 20 == 0);
  for (int i = 0; i < audio_buf_half_size; i += 20) {
    // sampler_decode(&s1, buf + i);
    // for (int j = 0; j < 20; j++)
    //   buf[i + j] >>= 7;
    polyphonic_out(&ps1, buf + i);
  }
}
