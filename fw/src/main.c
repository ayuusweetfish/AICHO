// Board revision: 1a

#include "pico/stdlib.h"
#include "pico/multicore.h"
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

static uint32_t audio_buf[2400];
static const uint32_t audio_buf_half_size = (sizeof audio_buf) / (sizeof audio_buf[0]) / 2;

static inline void refill_buffer(uint32_t *buf);
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
  channel_config_set_transfer_data_size(&dma_ch0, DMA_SIZE_32);
  channel_config_set_dreq(&dma_ch0, pio_get_dreq(pio0, sm, /* is_tx */ true));
  channel_config_set_chain_to(&dma_ch0, 2);
  dma_channel_set_irq0_enabled(0, true);
  dma_channel_configure(0, &dma_ch0, &pio0->txf[sm], audio_buf, audio_buf_half_size, false);

  dma_ch1 = dma_channel_get_default_config(1);
  channel_config_set_read_increment(&dma_ch1, true);
  channel_config_set_write_increment(&dma_ch1, false);
  channel_config_set_transfer_data_size(&dma_ch1, DMA_SIZE_32);
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
  if (s->ptr % (256 * 8) == 0) {
    uint64_t h = *(uint64_t *)AUXDAT(s->start + s->ptr);
    uint64_t w = *(uint64_t *)AUXDAT(s->start + s->ptr + 8);
    for (int i = 0; i < 4; i++)
      s->qoa_state.history[i] = (int16_t)(h >> (16 * i));
    for (int i = 0; i < 4; i++)
      s->qoa_state.weights[i] = (int16_t)(w >> (16 * i));
    s->ptr += 16;
  }
  uint64_t slice = *(uint64_t *)AUXDAT(s->start + s->ptr);
  qoa_decode_slice(&s->qoa_state, slice, out);
  if ((s->ptr += 8) >= s->len) s->ptr = 0;
}

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

static inline void polyphonic_out(struct polyphonic_sampler *s, uint32_t out[20])
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

  for (int j = 0; j < 20; j++) {
    int16_t sample = mix[j] >> 7;
    // (R << 16) | L
    out[j] = (((uint32_t)sample << 16) | (uint32_t)sample);
  }
}

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

// Input is `a[pendent index][strip index][RGB]`
void leds_blast(uint8_t a[][4][3], int n)
{
  // Little-endian, PIO shifts right
  static uint8_t leds_buf[80][12];

/*
  static uint32_t seed = 24067;
  for (int i = 0; i < 80 * 4; i++) {
    seed = seed * 1103515245 + 12345;
    leds_buf[i % 80] ^= (seed ^ (seed << 11));
  }
*/
/*
  for (int i = 0; i < n; i++) {
    leds_buf[i] = 0xaa55ccd4;
  }
*/

  for (int i = 0; i < n; i++) {
    // G
    for (int j = 0; j < 4; j++)
      leds_buf[i][j] =
        (((a[i][0][1] >> (0 + j * 2)) & 1) << 0) |
        (((a[i][1][1] >> (0 + j * 2)) & 1) << 1) |
        (((a[i][2][1] >> (0 + j * 2)) & 1) << 2) |
        (((a[i][3][1] >> (0 + j * 2)) & 1) << 3) |
        (((a[i][0][1] >> (1 + j * 2)) & 1) << 4) |
        (((a[i][1][1] >> (1 + j * 2)) & 1) << 5) |
        (((a[i][2][1] >> (1 + j * 2)) & 1) << 6) |
        (((a[i][3][1] >> (1 + j * 2)) & 1) << 7);
    // R
    for (int j = 0; j < 4; j++)
      leds_buf[i][j + 4] =
        (((a[i][0][0] >> (0 + j * 2)) & 1) << 0) |
        (((a[i][1][0] >> (0 + j * 2)) & 1) << 1) |
        (((a[i][2][0] >> (0 + j * 2)) & 1) << 2) |
        (((a[i][3][0] >> (0 + j * 2)) & 1) << 3) |
        (((a[i][0][0] >> (1 + j * 2)) & 1) << 4) |
        (((a[i][1][0] >> (1 + j * 2)) & 1) << 5) |
        (((a[i][2][0] >> (1 + j * 2)) & 1) << 6) |
        (((a[i][3][0] >> (1 + j * 2)) & 1) << 7);
    // B
    for (int j = 0; j < 4; j++)
      leds_buf[i][j + 8] =
        (((a[i][0][2] >> (0 + j * 2)) & 1) << 0) |
        (((a[i][1][2] >> (0 + j * 2)) & 1) << 1) |
        (((a[i][2][2] >> (0 + j * 2)) & 1) << 2) |
        (((a[i][3][2] >> (0 + j * 2)) & 1) << 3) |
        (((a[i][0][2] >> (1 + j * 2)) & 1) << 4) |
        (((a[i][1][2] >> (1 + j * 2)) & 1) << 5) |
        (((a[i][2][2] >> (1 + j * 2)) & 1) << 6) |
        (((a[i][3][2] >> (1 + j * 2)) & 1) << 7);

/*
    for (int j = 0; j < 12; j++)
      my_printf(" %01x %01x", (int)(leds_buf[i][j] & 0xf), (int)(leds_buf[i][j] >> 4));
    my_printf("\n");
    for (int j = 0; j < 3; j++)
      my_printf(" %08x", ((uint32_t *)&leds_buf[i][0])[j]);
    my_printf("\n");
*/
  }
  // my_printf("======\n");

  dma_channel_set_read_addr(4, leds_buf, false);
  dma_channel_set_trans_count(4, n * 3, true);
}

// ============ Auxiliary data ============

#define FILE_ADDR_Lorivox_In_bin   0
#define FILE_SIZE_Lorivox_In_bin   154816
#define FILE_ADDR_Lorivox_Ex_bin   154816
#define FILE_SIZE_Lorivox_Ex_bin   154816
#define FILE_ADDR_Lumisonic_In_bin 309632
#define FILE_SIZE_Lumisonic_In_bin 154816
#define FILE_ADDR_Lumisonic_Ex_bin 464448
#define FILE_SIZE_Lumisonic_Ex_bin 154816
#define FILE_ADDR_Harmonia_In_bin  619264
#define FILE_SIZE_Harmonia_In_bin  154816
#define FILE_ADDR_Harmonia_Ex_bin  774080
#define FILE_SIZE_Harmonia_Ex_bin  154816
#define FILE_ADDR_Titanus_In_bin   928896
#define FILE_SIZE_Titanus_In_bin   154816
#define FILE_ADDR_Titanus_Ex_bin   1083712
#define FILE_SIZE_Titanus_Ex_bin   154816
// [index][inhale/exhale][addr/size]
static const uint32_t organism_sounds[4][2][2] = {
  {{FILE_ADDR_Lorivox_In_bin, FILE_SIZE_Lorivox_In_bin},
   {FILE_ADDR_Lorivox_Ex_bin, FILE_SIZE_Lorivox_Ex_bin}},
  {{FILE_ADDR_Lumisonic_In_bin, FILE_SIZE_Lumisonic_In_bin},
   {FILE_ADDR_Lumisonic_Ex_bin, FILE_SIZE_Lumisonic_Ex_bin}},
  {{FILE_ADDR_Harmonia_In_bin, FILE_SIZE_Harmonia_In_bin},
   {FILE_ADDR_Harmonia_Ex_bin, FILE_SIZE_Harmonia_Ex_bin}},
  {{FILE_ADDR_Titanus_In_bin, FILE_SIZE_Titanus_In_bin},
   {FILE_ADDR_Titanus_Ex_bin, FILE_SIZE_Titanus_Ex_bin}},
};

// ============ Solenoid valves / pumps output ============

static const struct {
  uint8_t solenoid, pump_inflate, pump_drain;
} air_outputs[4] = {
  {.solenoid = 10, .pump_inflate = 11, .pump_drain = 12},
  {.solenoid = 10, .pump_inflate = 11, .pump_drain = 12},
  {.solenoid = 10, .pump_inflate = 11, .pump_drain = 12},
  {.solenoid = 27, .pump_inflate = 28, .pump_drain = 29},
};

static inline void pump_init()
{
  // TODO: Initialise all organisms with the updated board (Rev. 2)
  for (int i = 3; i < 4; i++) {
    uint8_t pin;
    pin = air_outputs[i].solenoid;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
    pin = air_outputs[i].pump_inflate;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
    pin = air_outputs[i].pump_drain;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
  }
}

static inline void pump(int org_index, int dir)
{
  // Solenoid on -> air can get out
  gpio_put(air_outputs[org_index].solenoid, dir != +1);
  sleep_ms(100);
  gpio_put(air_outputs[org_index].pump_inflate, dir == +1);
  gpio_put(air_outputs[org_index].pump_drain, dir == -1);
}

// ============ Entry point ============

void core1_entry();

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
  // while (1) { }

  multicore_launch_core1(core1_entry);

  pump_init();

  while (1) {
    pump(3, -1); gpio_put(act_1, 0); sleep_ms(1800);
    pump(3,  0); gpio_put(act_1, 0); sleep_ms(500);
    pump(3, +1); gpio_put(act_1, 1); sleep_ms(1800);
    pump(3,  0); gpio_put(act_1, 0); sleep_ms(500);
  }

  tuh_init(BOARD_TUH_RHPORT);
  while (1) {
    tuh_task();
  }
  while (1) { }

/*
  leds_init();

  uint8_t a[10][4][3] = {{{ 0 }}};
  while (1) {
    if (a[0][0][1] > 0) {
      if (++a[0][0][1] >= 10) a[0][0][1] = 0;
    } else {
      if (++a[0][0][0] >= 10) { a[0][0][0] = 0; a[0][0][1] = 1; }
    }
    a[1][0][0] = 0x33; a[1][0][1] = 0x01; a[1][0][2] = 0x60;
    a[2][0][0] = 0x33; a[2][0][1] = 0x01; a[2][0][2] = 0x60;
    a[3][0][0] = 0x33; a[3][0][1] = 0x01; a[3][0][2] = 0x60;
    leds_blast(a, 10);
    gpio_put(act_1, 1); sleep_ms(100);
    gpio_put(act_1, 0); sleep_ms(400);
  }
*/

}

struct polyphonic_sampler ps1;

void core1_entry()
{
  polyphonic_init(&ps1);

  audio_buf_init();
  audio_buf_resume();

  polyphonic_trigger(&ps1,
    FILE_ADDR_Lorivox_In_bin,
    FILE_SIZE_Lorivox_In_bin);

  while (1) {
    gpio_put(act_2, 1); sleep_ms(100);
    gpio_put(act_2, 0); sleep_ms(200);
    static uint32_t i = 0;
    i++;
    my_printf("Hello, UART %u!%c", i, i % 2 == 0 ? '\n' : '\t');
  /*
    if (i >= 10 && i % 10 == 0) {
      audio_buf_suspend();
      gpio_put(act_2, 0);
    } else if (i >= 10 && i % 10 == 1) {
      audio_buf_resume();
      gpio_put(act_2, 1);
    }
  */
    if (i % 10 == 3) {
      int org = ((i / 10 + 1) / 2) % 4;
      int dir = (i / 10 + 1) % 2;
      polyphonic_trigger(&ps1,
        organism_sounds[org][dir][0],
        organism_sounds[org][dir][1]);
    }
  }
}

void refill_buffer(uint32_t *buf)
{
  assert(audio_buf_half_size % 20 == 0);
  for (int i = 0; i < audio_buf_half_size; i += 20) {
    polyphonic_out(&ps1, buf + i);
  }
}
