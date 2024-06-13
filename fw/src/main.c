#include "main.h"

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

#include "tusb.h"

#include "i2s_out.pio.h"
#include "ws2812.pio.h"
#include "i2s_in.pio.h"

#define uQOA_IMPL
#include "uqoa.h"

#include "leds.h"

#define TESTRUN 0

#define max(_a, _b) ((_a) > (_b) ? (_a) : (_b))
#define min(_a, _b) ((_a) < (_b) ? (_a) : (_b))

static inline uint32_t sat_add_u32(uint32_t a, uint32_t b) {
  return (a + b < a) ? UINT32_MAX : a + b;
}
static inline uint32_t sat_sub_u32(uint32_t a, uint32_t b) {
  return (a < b ? 0 : a - b);
}

// ============ Debug output ============

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

static uint32_t audio_buf[9600];  // s32 stereo; half buffer 2400 samples = 50 ms
// Count of u32's (words / transfer units)
#define audio_buf_half_size ((sizeof audio_buf) / (sizeof audio_buf[0]) / 2)
// Count of sample frames
#define audio_buf_half_frame (audio_buf_half_size / 2)

static inline void refill_buffer(uint32_t *buf);
static void dma_irq0_handler();

static dma_channel_config dma_ch0, dma_ch1, dma_ch2, dma_ch3;

void audio_buf_init()
{
  uint sm = pio_claim_unused_sm(pio0, true);
#if BOARD_REV == 1
  i2s_out_32b_program_init(pio0, sm, &i2s_out_32b_program, 2, 3);
#elif BOARD_REV == 2
  i2s_out_32b_program_init(pio0, sm, &i2s_out_32b_program, 1, 2);
#endif

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

  irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
  irq_set_enabled(DMA_IRQ_0, true);

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

// ============ Audio input buffer ============

static uint32_t audio_in_buf[FFT_N * 2];
#define audio_in_buf_half_size ((sizeof audio_in_buf) / (sizeof audio_in_buf[0]) / 2)
// audio_in_buf_half_size == FFT_N

static inline void consume_buffer(const int32_t *buf);
static void dma_irq1_handler();

static dma_channel_config dma_ch8, dma_ch9, dma_ch10, dma_ch11;

void audio_in_init()
{
#if BOARD_REV == 2
  uint sm_mck = pio_claim_unused_sm(pio0, true);
  i2s_in_mck_out_program_init(pio0, sm_mck, 7);
  pio_sm_set_enabled(pio0, sm_mck, true);

  gpio_init(8); gpio_set_dir(8, GPIO_OUT);
  gpio_put(8, 1);

  sleep_ms(40); // MAX9814 shutdown disable time (t_OFF)
  uint sm_in = pio_claim_unused_sm(pio0, true);
  i2s_in_program_init(pio0, sm_in, 4);
  pio_sm_set_enabled(pio0, sm_in, true);

  dma_ch8 = dma_channel_get_default_config(8);
  channel_config_set_read_increment(&dma_ch8, false);
  channel_config_set_write_increment(&dma_ch8, true);
  channel_config_set_transfer_data_size(&dma_ch8, DMA_SIZE_32);
  channel_config_set_dreq(&dma_ch8, pio_get_dreq(pio0, sm_in, /* is_tx */ false));
  channel_config_set_chain_to(&dma_ch8, 10);
  dma_channel_set_irq1_enabled(8, true);
  dma_channel_configure(8, &dma_ch8, audio_in_buf, &pio0->rxf[sm_in], audio_in_buf_half_size, false);

  dma_ch9 = dma_channel_get_default_config(9);
  channel_config_set_read_increment(&dma_ch9, false);
  channel_config_set_write_increment(&dma_ch9, true);
  channel_config_set_transfer_data_size(&dma_ch9, DMA_SIZE_32);
  channel_config_set_dreq(&dma_ch9, pio_get_dreq(pio0, sm_in, /* is_tx */ false));
  channel_config_set_chain_to(&dma_ch9, 11);
  dma_channel_set_irq1_enabled(9, true);
  dma_channel_configure(9, &dma_ch9, audio_in_buf + audio_in_buf_half_size, &pio0->rxf[sm_in], audio_in_buf_half_size, false);

  irq_set_exclusive_handler(DMA_IRQ_1, dma_irq1_handler);
  irq_set_enabled(DMA_IRQ_1, true);

  static uint32_t addr0 = (uint32_t)&audio_in_buf[0];
  static uint32_t addr1 = (uint32_t)&audio_in_buf[audio_in_buf_half_size];

  dma_ch10 = dma_channel_get_default_config(10);
  channel_config_set_read_increment(&dma_ch10, false);
  channel_config_set_write_increment(&dma_ch10, false);
  dma_channel_configure(10, &dma_ch10, &dma_hw->ch[9].al2_write_addr_trig, &addr1, 1, false);

  dma_ch11 = dma_channel_get_default_config(11);
  channel_config_set_read_increment(&dma_ch11, false);
  channel_config_set_write_increment(&dma_ch11, false);
  dma_channel_configure(11, &dma_ch11, &dma_hw->ch[8].al2_write_addr_trig, &addr0, 1, false);
#endif
}

void audio_in_suspend()
{
  channel_config_set_enable(&dma_ch8, false);
  dma_channel_set_config(8, &dma_ch8, false);
}

void audio_in_resume()
{
  channel_config_set_enable(&dma_ch8, true);
  dma_channel_set_write_addr(8, audio_in_buf, false);
  dma_channel_set_config(8, &dma_ch8, true);
}

void dma_irq1_handler()
{
  if (dma_channel_get_irq1_status(8)) {
    dma_channel_acknowledge_irq1(8);
    consume_buffer(audio_in_buf);
  } else {
    dma_channel_acknowledge_irq1(9);
    consume_buffer(audio_in_buf + audio_in_buf_half_size);
  }
}

// ============ Sampler ============

struct sampler {
  uint32_t start, len;
  qoa_lms qoa_state;
  uint32_t ptr;
};

#pragma gcc optimize("O3")
static inline void sampler_mix(struct sampler *s, int32_t out[20])
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
  qoa_mix_slice(&s->qoa_state, slice, out);
  if ((s->ptr += 8) >= s->len) s->ptr = 0;
}

#define POLYPHONY 8
struct polyphonic_sampler {
  critical_section_t crit;
  struct sampler s[POLYPHONY];
  bool active[POLYPHONY];

  struct trigger_queue_el {
    uint32_t start;
    uint32_t len;
  } trigger_queue[POLYPHONY];
  uint8_t trigger_queue_len;
};

static inline void polyphonic_init(struct polyphonic_sampler *s)
{
  critical_section_init(&s->crit);
  for (uint8_t i = 0; i < POLYPHONY; i++) s->active[i] = 0;
  s->trigger_queue_len = 0;
}

static inline uint8_t polyphonic_trigger(
  struct polyphonic_sampler *s,
  uint32_t start, uint32_t len
) {
  critical_section_enter_blocking(&s->crit);
  if (s->trigger_queue_len < POLYPHONY) {
    s->trigger_queue[s->trigger_queue_len++] = (struct trigger_queue_el){
      .start = start,
      .len = len,
    };
  }
  critical_section_exit(&s->crit);
}

// Fixed size `audio_buf_half_size` (count of s32's) / `audio_buf_half_frame` (count of sample frames)
// Representation is s32, but pointer type is u32 to unify all data buffers
#pragma gcc optimize("O3")
static inline void polyphonic_out(struct polyphonic_sampler *restrict s, uint32_t *restrict out)
{
  // Process triggers

  critical_section_enter_blocking(&s->crit);

  for (int i = 0; i < s->trigger_queue_len; i++) {
    uint32_t start = s->trigger_queue[i].start;
    uint32_t len = s->trigger_queue[i].len;

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
  }

  s->trigger_queue_len = 0;

  critical_section_exit(&s->crit);

  // Synthesise and mix

  assert(audio_buf_half_frame % 20 == 0);

  static int32_t mix[audio_buf_half_frame];
  memset(mix, 0, sizeof mix);

  for (uint8_t i = 0; i < POLYPHONY; i++)
    if (s->active[i]) {
      for (int j = 0; j < audio_buf_half_frame; j += 20) {
        sampler_mix(&s->s[i], mix + j);
        if (s->s[i].ptr == 0) {
          s->active[i] = false;
          break;
        }
      }
    }

  for (int j = 0; j < audio_buf_half_frame; j++) {
    // 1.5V ~ 2.2V ~ 2.9V (0x4000 ~ 0x6000 ~ 0x7fff)
    // Divide sample by 128 (>> 7), then shift left by 16 bits
    // Note: the scaler should be at least equal to
    //  `POLYPHONY` * 4 (dynamic range, from 0x4000 to 0x7fff) = 32
    // i.e., 0x60000000 + ((0x7fff * POLYPHONY) << 9) <= 0x7fffffff
    int32_t sample = 0x60000000 + (mix[j] << 9);
    out[j * 2] = out[j * 2 + 1] = sample;
  }
}

// ============ LED strip ============

static dma_channel_config dma_ch4;

void leds_init()
{
  uint sm = pio_claim_unused_sm(pio0, true);
#if BOARD_REV == 1
  ws2812_program_init(pio0, sm, 11);
#elif BOARD_REV == 2
  ws2812_program_init(pio0, sm, 18);
#endif

  pio_sm_set_enabled(pio0, sm, true);

  dma_ch4 = dma_channel_get_default_config(4);
  channel_config_set_read_increment(&dma_ch4, true);
  channel_config_set_write_increment(&dma_ch4, false);
  channel_config_set_dreq(&dma_ch4, pio_get_dreq(pio0, sm, /* is_tx */ true));
  dma_channel_set_config(4, &dma_ch4, false);
  dma_channel_set_write_addr(4, &pio0->txf[sm], false);

  // while (1) pio_sm_put_blocking(pio0, sm, 0xffffffff);
}

// Input is `a[pendent index][strip index][RGB]`
void leds_blast(uint8_t a[][4][3], int n)
{
  // Little-endian, PIO shifts right
  static uint8_t leds_buf[96][12];
  assert(n <= sizeof leds_buf / sizeof leds_buf[0]);

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
  }

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
#define FILE_ADDR_Ensemble_In_bin  1238528
#define FILE_SIZE_Ensemble_In_bin  154816
#define FILE_ADDR_Ensemble_Ex_bin  1393344
#define FILE_SIZE_Ensemble_Ex_bin  154816
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
static const uint32_t ensemble_sounds[2][2] = {
  {FILE_ADDR_Ensemble_In_bin, FILE_SIZE_Ensemble_In_bin},
  {FILE_ADDR_Ensemble_Ex_bin, FILE_SIZE_Ensemble_Ex_bin},
};

// ============ Solenoid valves / pumps output ============

static const struct {
  uint8_t solenoid, pump_inflate, pump_drain;
} air_outputs[4] = {
#if BOARD_REV == 1
  {.solenoid =  5, .pump_inflate =  6, .pump_drain =  7},
  {.solenoid =  8, .pump_inflate =  9, .pump_drain = 10},
  {.solenoid = 18, .pump_inflate = 19, .pump_drain = 20},
  {.solenoid = 27, .pump_inflate = 28, .pump_drain = 29},
#elif BOARD_REV == 2
  {.solenoid = 10, .pump_inflate = 11, .pump_drain = 12},
  {.solenoid = 13, .pump_inflate = 14, .pump_drain = 15},
  {.solenoid = 22, .pump_inflate = 23, .pump_drain = 24},
  {.solenoid = 27, .pump_inflate = 28, .pump_drain = 29},
#endif
};

static inline void pump_init()
{
  for (int i = 0; i < 4; i++) {
    uint8_t pin;
    pin = air_outputs[i].solenoid;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
    pin = air_outputs[i].pump_inflate;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
    pin = air_outputs[i].pump_drain;
    gpio_init(pin); gpio_set_dir(pin, GPIO_OUT); gpio_put(pin, 0);
  }
}

// +2: inflate
// +1: hold, solenoid off (standby)
// -1: hold, solenoid on (minor air leakage)
// -2: drain
static int8_t pump_last_op[4] = { 0 };
static inline void pump(int org_index, int dir)
{
  // Solenoid on -> air can get out
  gpio_put(air_outputs[org_index].solenoid, dir < 0);
  gpio_put(air_outputs[org_index].pump_inflate, dir == +2);
  gpio_put(air_outputs[org_index].pump_drain, dir == -2);
}

// ============ Entry point ============

struct polyphonic_sampler ps1;

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

  if (watchdog_caused_reboot()) my_printf("Rebooted by watchdog\n");

  my_printf("sys clk %u\n", clock_get_hz(clk_sys));

  my_printf("__etext is %08x\n", &__etext);
  assert((uint32_t)&__etext <= FLASH_BASE + AUXDAT_ADDR);

  my_printf("flash_erase_64k()  at %08x\n", &flash_erase_64k);
  my_printf("flash_test_write() at %08x\n", &flash_test_write);
  // while (1) { } // Uncomment when uploading the data

if (0) {
  struct polyphonic_sampler ps1;
  polyphonic_init(&ps1);
  for (int dir = 0; dir < 2; dir++)
    for (int org = 0; org < 4; org++)
      polyphonic_trigger(&ps1,
        organism_sounds[org][dir][0],
        organism_sounds[org][dir][1]);
  uint32_t buf[audio_buf_half_size];
  uint32_t t0 = to_ms_since_boot(get_absolute_time());
  for (int i = 0; i < 1000; i++) {
    polyphonic_out(&ps1, buf);
    if ((i + 1) % 50 == 0) {
      uint32_t t1 = to_ms_since_boot(get_absolute_time());
      // 1340 ms for 50 * 2400 = 120 k samples = 5 s of audio
      my_printf("%4d %8u\n", i + 1, t1 - t0);
      t0 = t1;
    }
  }
  while (1) { }
}

  // NOTE: seems to have a pop/glitch in the exhale sound of any organism
  // which is present iff the draining pump is connected and
  // the power supply is debug probe's VBUS
  // -- EMI? Power supply? Unsure for now.

  fft_init();
  pump_init();
  leds_init();
  tuh_init(BOARD_TUH_RHPORT);

  multicore_reset_core1();
  multicore_fifo_pop_blocking();
  multicore_launch_core1(core1_entry);

  // See `pump()` signal values
  int pump_dir_signals[4] = { 0 };

  enum state_t {
    IDLE,
    SINGLE_RUN,
    FOLLOWER_RUN,
  } state = IDLE;
  int8_t org_id;        // For `SINGLE_RUN` and `FOLLOWER_RUN`
  uint32_t state_time;  // Duration into the current state, invalid for `IDLE`

  // Updates `pump_dir_signals` from `org_key` and breath signals (TODO)
  void update_signals(uint32_t t) {
    // Q - inflate
    // W - drain
    // E - leak
    // pump_dir_signals[0] = (org_key[0] ? +2 : org_key[1] ? -2 : org_key[2] ? -1 : +1);

    if (state == SINGLE_RUN) {
      int last_phase = (state_time == 0 ? -1 : state_time / 2000);
      state_time += t;
      int cur_phase = state_time / 2000;
      if (state_time >= 20000) {
        state = IDLE;
        for (int i = 0; i < 4; i++)
          pump_dir_signals[i] = +1;
      } else if (cur_phase != last_phase) {
        if (cur_phase < 4) {
          pump_dir_signals[org_id] = (cur_phase % 2 == 0 ? +2 : -2);
          polyphonic_trigger(&ps1,
            organism_sounds[org_id][cur_phase % 2][0],
            organism_sounds[org_id][cur_phase % 2][1]);
        } else if (cur_phase < 8) {
          for (int i = 0; i < 4; i++)
            pump_dir_signals[i] = (cur_phase % 2 == 0 ? +2 : -2);

          polyphonic_trigger(&ps1,
            ensemble_sounds[cur_phase % 2][0],
            ensemble_sounds[cur_phase % 2][1]);
        } else {
          for (int i = 0; i < 4; i++)
            pump_dir_signals[i] = -1;
        }
      }
    } else if (state == FOLLOWER_RUN) {
      // TODO: Check breath signals
    }

    static int pressed_key = -1;
    static int pressed_dur = 0;

    int cur_pressed_key = -1;
    for (int i = 0; i < 4; i++)
      if (org_key[i]) { cur_pressed_key = i; break; }
    if (cur_pressed_key != -1) {
      // One key pressed
      if (cur_pressed_key == pressed_key) {
        // Held
        pressed_dur += 1;
        if (pressed_dur == 500) {
          if (state == IDLE) {
            my_printf("long %d start\n", pressed_key);
            state = FOLLOWER_RUN;
            org_id = pressed_key;
            state_time = 0;
          }
        }
      } else if (pressed_key == -1) {
        // Newly pressed
        pressed_key = cur_pressed_key;
        pressed_dur = 0;
      } else {
        // Release from a multiple press / quick switch
        // Treat as a release for the current update; leave the press event for next time
        cur_pressed_key = -1;
      }
    }
    if (cur_pressed_key == -1) {
      // No key pressed
      if (pressed_key != -1) {
        // Release
        if (pressed_dur < 500) {
          if (state == IDLE) {
            my_printf("short %d\n", pressed_key);
            state = SINGLE_RUN;
            org_id = pressed_key;
            state_time = 0;
          }
        } else {
          if (state == FOLLOWER_RUN && org_id == pressed_key) {
            my_printf("long %d end\n", pressed_key);
            state = IDLE;
          }
        }
      }
      pressed_key = -1;
    }
  }

  // Task pool
  struct proceed_t {
    struct proceed_t (*fn)(uint32_t);
    uint32_t wait;
  };

  struct proceed_t task_uart_1s() {
    my_printf("[%8u] second!\n", to_ms_since_boot(get_absolute_time()));
    return (struct proceed_t){task_uart_1s, 1000000};
  }

  struct proceed_t task_usb(uint32_t _missed) {
    // uint32_t t0 = to_ms_since_boot(get_absolute_time());
    tuh_task();
    // uint32_t t1 = to_ms_since_boot(get_absolute_time());
    // if (t1 - t0 >= 3) my_printf("tuh_task() takes %u ms\n", t1 - t0);
    return (struct proceed_t){task_usb, 1000};    // 1 ms
  }

  struct proceed_t task_update_signals(uint32_t missed) {
    update_signals(missed + 1);
    return (struct proceed_t){task_update_signals, 1000}; // 1 ms
  }

  static int32_t air[4] = { 0 };
  static int32_t wait[4] = { 0 }; // How long has been waited for at -1 signal
  static const struct {
    uint32_t air_limit, inflate_rate, leakage, drain_rate;
  } rates[4] = {
    {30000, 10, 3, 10},
    {20000, 10, 3, 10},
    {20000, 10, 3, 10},
    {20000, 10, 3, 10},
  };

  struct proceed_t task_pumps(uint32_t missed) {
    static bool startup = true;
    if (startup) {
      // Drain remaining air
      for (int i = 0; i < 4; i++) pump(i, -2);
      startup = false;
      return (struct proceed_t){task_pumps, 2500000};
    }

    if (missed > 0) {
      // my_printf("missed %u\n", missed);
      for (int i = 0; i < 4; i++) switch (pump_last_op[i]) {
        case +2: {
          air[i] += rates[i].inflate_rate * missed;
        } break;
        case -2: {
          air[i] = sat_sub_u32(air[i], rates[i].drain_rate * missed);
        } break;
        case +1: default: {
        } break;
        case -1: {
          air[i] = sat_sub_u32(air[i], rates[i].leakage * missed);
        } break;
      }
    }
    for (int i = 0; i < 4; i++) switch (pump_dir_signals[i]) {
      case +2: {
        if (air[i] + rates[i].inflate_rate < rates[i].air_limit) {
          pump(i, +2);
          air[i] += rates[i].inflate_rate;
        } else {
          pump(i, +1);
        }
        wait[i] = 0;
      } break;
      case -2: {
        if (wait[i] >= 100) {
          pump(i, -2);
          air[i] = sat_sub_u32(air[i], rates[i].drain_rate);
        } else {
          pump(i, -1);
          air[i] = sat_sub_u32(air[i], rates[i].leakage);
          wait[i] += 1;
        }
      } break;
      case +1: {
        // Hold
        pump(i, +1);
        wait[i] = 0;
      } break;
      case -1: default: {
        pump(i, -1);
        air[i] = sat_sub_u32(air[i], rates[i].leakage);
        wait[i] = 0;
      } break;
    }

    return (struct proceed_t){task_pumps, 1000};  // 1 ms
  };

  struct proceed_t task_leds(uint32_t _missed) {
    uint8_t a[96][4][3] = {{{ 0 }}};
    static uint32_t n = 0;
    for (int strip = 1; strip < 1; strip++) {
      for (int i = 0; i < 96; i++) {
        int32_t x = 4 - (int32_t)(i + strip + n / 8) % 8;
        a[i][strip][strip % 3] = x < 0 ? -x : x;
      }
    }
    if (state == SINGLE_RUN) {
      if (state_time < 16000) {
        // Default to fade-out
        int intensity = (state_time >= 500 ? 0 :
          4096 * (500 - state_time) / 500 * (500 - state_time) / 500);
        gradient_Lorivox(intensity, a);
        gradient_Lumisonic(intensity, 3200, a);
        gradient_Harmonia(intensity, a);
        gradient_Titanus(intensity, a);
        // Replace the active organism(s) with their active animations
        if (org_id == 0 || state_time >= 8000)
          gradient_Lorivox(4096 * air[0] / rates[0].air_limit, a);
        if (org_id == 1 || state_time >= 8000) {
          int lum_progress = (state_time % 4000) * 8192 / 4000;
          gradient_Lumisonic(4096, lum_progress, a);
        }
        if (org_id == 2 || state_time >= 8000)
          gradient_Harmonia(4096 * air[2] / rates[2].air_limit, a);
        if (org_id == 3 || state_time >= 8000)
          gradient_Titanus(4096 * air[3] / rates[3].air_limit, a);
      } else {
        int intensity = 4096 * (state_time - 16000) / 4000;
        float x = (float)(state_time - 16000) / 4000;
        int lum_progress = 3200 * (1 - (1 - x) * (1 - x));
        gradient_Lorivox(intensity, a);
        gradient_Lumisonic(intensity, lum_progress, a);
        gradient_Harmonia(intensity, a);
        gradient_Titanus(intensity, a);
      }
    } else {
      gradient_Lorivox(4096, a);
      gradient_Lumisonic(4096, 3200, a);
      gradient_Harmonia(4096, a);
      gradient_Titanus(4096, a);
    }
    n++;
    // 96 * 24 / 800 kHz = 2.88 ms
    leds_blast(a, 96);
    return (struct proceed_t){task_leds, 10000};  // 10 ms = 100 fps
  };

  struct proceed_t task_watchdog(uint32_t _missed) {
    watchdog_update();
    return (struct proceed_t){task_watchdog, 100000}; // 100 ms
  }

  // Due to the long time that a single `tuh_task()` may take (max. 500 ms)
  watchdog_enable(1000, true);

  struct task_t {
    struct proceed_t (*fn)(uint32_t);
    uint64_t next_tick;
  };
  uint64_t cur_tick = time_us_64(); // to_us_since_boot(get_absolute_time());
  struct task_t task_pool[] = {
    {task_uart_1s, cur_tick},
    {task_usb, cur_tick},
    {task_update_signals, cur_tick},
    {task_pumps, cur_tick},
    {task_leds, cur_tick},
    {task_watchdog, cur_tick},
  };
  while (1) {
    uint64_t cur_tick = time_us_64();
    int64_t soonest_diff = INT64_MAX;
    int soonest_index = -1;
    for (int i = 0; i < (sizeof task_pool) / (sizeof task_pool[0]); i++) {
      int64_t diff = (int64_t)(task_pool[i].next_tick - cur_tick);
      if (diff < soonest_diff) {
        soonest_diff = diff;
        soonest_index = i;
      }
    }
    if (soonest_diff <= 0) {
      // Execute task
      // my_printf("[%8u] task %08x\n", to_ms_since_boot(get_absolute_time()), task_pool[soonest_index].fn);
      uint64_t expected_tick = task_pool[soonest_index].next_tick;
      uint32_t missed = 0;
      if (expected_tick < cur_tick) missed = (cur_tick - expected_tick) / 1000;
      struct proceed_t result = task_pool[soonest_index].fn(missed);
      task_pool[soonest_index].fn = result.fn;
      task_pool[soonest_index].next_tick = max(expected_tick + result.wait, cur_tick + 1);
    } else {
      if (soonest_diff > 5) sleep_us(soonest_diff - 5);
    }
  }
}

void core1_entry()
{
  polyphonic_init(&ps1);

  audio_buf_init();
  audio_buf_resume();

  audio_in_init();
  audio_in_resume();

#if TESTRUN
  while (1) {
    static uint32_t i = 0;
    i++;
    // gpio_put(act_2, 1); sleep_ms(100);
    // gpio_put(act_2, 0); sleep_ms(200);
    sleep_ms(300);
    // my_printf("[%8u] Hello, UART %u!\n", to_ms_since_boot(get_absolute_time()), i);
    if (i % 10 == 3) {
      int org = ((i / 10 + 1) / 2) % 6;
      int dir = (i / 10 + 1) % 2;
      if (org < 2) {
        int group = org;
        for (int org = group; org < 4; org++)
          polyphonic_trigger(&ps1,
            organism_sounds[org][dir][0],
            organism_sounds[org][dir][1]);
      } else {
        org -= 2;
        polyphonic_trigger(&ps1,
          organism_sounds[org][dir][0],
          organism_sounds[org][dir][1]);
      }
    }
  }
#endif

  while (1) sleep_ms(1000);
}

void refill_buffer(uint32_t *buf)
{
  polyphonic_out(&ps1, buf);
}

void consume_buffer(const int32_t *buf)
{
  int32_t min = INT32_MAX;
  int32_t max = INT32_MIN;
  for (int i = 0; i < audio_in_buf_half_size; i++) {
    int32_t value = buf[i] & 0xffffff00;
    if (value > max) max = value;
    if (value < min) min = value;
  }
  uint32_t diff = max - min;
  // gpio_put(act_1, diff >= 0xa0000000);

  static int count = 0;

  static int32_t fft_result[audio_in_buf_half_size / 2 + 1][2];
  fft(buf, &fft_result[0][0]);

  // FFT bins = 0 ~ 1024, from 0 to Nyquist freq. 25.78125 kHz
  // each bin corresponds to 25.177 Hz

  static uint32_t ampl[audio_in_buf_half_size / 2 + 1];
  for (int i = 0; i <= audio_in_buf_half_size / 2; i++)
    ampl[i] =
      (uint32_t)((
        (int64_t)fft_result[i][0] * fft_result[i][0] +
        (int64_t)fft_result[i][1] * fft_result[i][1]
      ) >> 32);

  uint64_t enr_total = 0;
  for (uint32_t j = 100; j <= 280; j++) enr_total += ampl[j];

  int64_t sum_hi = 0;
  for (uint32_t j = 640; j <= 1024; j++) sum_hi += ampl[j];
  static int64_t hi_ema = 0;
  hi_ema += (sum_hi - hi_ema) / 8;

  static uint32_t bins[audio_in_buf_half_size / 2];
  // From 7 kHz = 280-th coefficient
  // To 16 kHz = 640-th coefficient
  for (uint32_t i = 70; i < 160; i++) {
    uint32_t sum = 0;
    for (int j = i * 4 + 1; j <= i * 4 + 4; j++) sum = sat_add_u32(sum, ampl[j]);
    sum = (uint64_t)sum * 1000 / (enr_total / 100);
    bins[i - 70] = sum;
  }
  int uint32_compare_desc(const void *a, const void *b) {
    return *(const uint32_t *)b - *(const uint32_t *)a;
  }
  qsort(bins, 90, sizeof(uint32_t), uint32_compare_desc);

  static uint32_t accum = 0;
  static bool signal = false;
  static uint32_t held = 0;

  if (bins[ 5] >= 400)
    accum += (min(50, bins[ 5] - 400) << 14);
  if (bins[40] >= 100)
    accum += (min(50, bins[40] - 100) << 14);
  if (bins[60] >=  50)
    accum += (min(50, bins[60] -  50) << 14);
  if (bins[70] >= 500)
    accum = sat_sub_u32(accum, min(50, bins[70] - 500) << 16);
  if (bins[80] >= 200)
    accum = sat_sub_u32(accum, min(50, bins[80] - 200) << 16);

  if (!signal) {
    if (accum >= 0x200000) signal = true;
  } else {
    if (accum < 0x1a0000 - held * 0x5000) {
      signal = false;
      held = 0;
    } else {
      if (held < 40) held++;
    }
  }
  gpio_put(act_2, signal);
  accum >>= 1;

  if (++count >= 0.2 * 51563 / audio_in_buf_half_size) {
  /*
    my_printf("[%8u] p-p %10u |", to_ms_since_boot(get_absolute_time()), diff);
    for (int i = 512; i <= 1024; i += 8) {
      uint32_t sum = 0;
      for (int j = i; j < i + 8; j++) sum += ampl[j];
      sum /= 8;
      my_printf("%c",
        sum >= 300 ? '#' :
        sum >= 100 ? '*' :
        sum >=  25 ? '.' :
        ' ');
    }
    my_printf("|\n");
  */

  /*
    my_printf("[%8u] | ", to_ms_since_boot(get_absolute_time()));
    my_printf("%8x | %6u %6u |",
      accum,
      (uint32_t)min(999999, enr_total / 1000),
      (uint32_t)llabs(sum_hi - hi_ema)
    );
    uint32_t pivot = bins[30];
    for (int i = 0; i < 90; i += 5)
      my_printf(" %5u", min(99999, bins[i]));
    my_printf("\n");
  */
    count = 0;
  }
}
