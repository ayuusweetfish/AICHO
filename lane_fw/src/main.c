#include "py32f0xx_hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

// #define RELEASE
#include "debug_printf.h"

static void spin_delay(uint32_t cycles)
{
  __asm__ volatile (
    "   cmp %[cycles], #5\n"
    "   ble 2f\n"
    "   sub %[cycles], #5\n"
    "   lsr %[cycles], #2\n"
    "1: sub %[cycles], #1\n"
    "   nop\n"
    "   bne 1b\n"   // 2 cycles if taken
    "2: \n"
    : [cycles] "+l" (cycles)
    : // No output
    : "cc"
  );
}
static inline void delay_us(uint32_t us)
{
  spin_delay(us * 16);
}

#define PACKETS_INSTANCE_NAME downstream
#include "packets.h"

#define PACKETS_INSTANCE_NAME upstream
#include "packets.h"

typedef enum {
  OP_INFLATE = (1 << 0),
  OP_DRAIN = (1 << 1),
  OP_FADE_OUT = (1 << 2),
} op_t;
static volatile op_t op_pending = 0;
static int lane_index = -1;
static struct {
  int PUMP_INFLATE_DUTY;
  int PUMP_DRAIN_DUTY;
  int PRESSURE_LIMIT;
  int PRESSURE_BAIL;
  int INFLATE_TIME_LIMIT;
} args;

#pragma GCC push_options
#pragma GCC optimize("O3")
int main()
{
  HAL_Init();

  // ============ Clocks ============ //
{
  HAL_RCC_OscConfig(&(RCC_OscInitTypeDef){
    .OscillatorType = RCC_OSCILLATORTYPE_HSE,
    .HSEState = RCC_HSE_ON,
    .HSEFreq = RCC_HSE_16_32MHz,
  });
  HAL_RCC_ClockConfig(&(RCC_ClkInitTypeDef){
    .ClockType =
      RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_HCLK |
      RCC_CLOCKTYPE_PCLK1,
    .SYSCLKSource = RCC_SYSCLKSOURCE_HSE, // 16 MHz
    .AHBCLKDivider = RCC_SYSCLK_DIV1,
    .APB1CLKDivider = RCC_HCLK_DIV1,
  }, FLASH_LATENCY_0);
}
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

{
  uint32_t rcc_csr = RCC->CSR;
  RCC->CSR = rcc_csr | RCC_CSR_RMVF;
  if (rcc_csr & RCC_CSR_IWDGRSTF) printf("IWDG reset\n");
  if (rcc_csr & RCC_CSR_SFTRSTF) printf("Soft reset\n");
  if (rcc_csr & RCC_CSR_PWRRSTF) printf("Power reset\n");
  if (rcc_csr & RCC_CSR_PINRSTF) printf("NRST pin reset\n");

  printf("sysclk = %lu Hz\n", HAL_RCC_GetSysClockFreq());
  while (0) {   // Test usage
    delay_us(1000000);
    printf("ticks = %lu\n", HAL_GetTick());
  }
}

  // ============ Activity LED ============ //
{
#define ACT_ON()  GPIOB->BSRR = (1 << 7)
#define ACT_OFF() GPIOB->BSRR = (1 << 23)
  ACT_OFF();
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pin = (1 << 7),
  });
  ACT_ON(); HAL_Delay(100); ACT_OFF(); HAL_Delay(100);
}

  // ============ Pumps ============ //
  // PUMP_1 (Drain)   = PA0 AF13 = TIM1_CH3
  // PUMP_2 (Inflate) = PA1 AF13 = TIM1_CH4
  // PUMP_3 (Valve)   = PA2 AF13 = TIM3_CH1
{
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_AF_PP,
    .Pin = (1 << 0) | (1 << 1) | (1 << 2),
    .Alternate = 13,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
}
{
  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM_HandleTypeDef tim1 = {
    .Instance = TIM1,
    .Init = {
      .Prescaler = 1 - 1,   // 16 MHz
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 200 - 1,    // 80 kHz
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    },
  };
  HAL_TIM_PWM_Init(&tim1);

  HAL_TIM_PWM_ConfigChannel(&tim1, &(TIM_OC_InitTypeDef){
    .OCMode = TIM_OCMODE_PWM1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  }, TIM_CHANNEL_3);
  HAL_TIM_PWM_ConfigChannel(&tim1, &(TIM_OC_InitTypeDef){
    .OCMode = TIM_OCMODE_PWM1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  }, TIM_CHANNEL_4);
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_4);
}
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  TIM_HandleTypeDef tim3 = {
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,   // 16 MHz
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 200 - 1,    // 80 kHz
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    },
  };
  HAL_TIM_PWM_Init(&tim3);

  HAL_TIM_PWM_ConfigChannel(&tim3, &(TIM_OC_InitTypeDef){
    .OCMode = TIM_OCMODE_PWM1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  }, TIM_CHANNEL_1);
  TIM3->CCR1 = 0;
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1);
}

  // ============ Pressure sensor ============ //
  // PA3 = ADC_IN3
  static ADC_HandleTypeDef adc1;
  uint32_t vrefint_value = 0;
{
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_ANALOG,
    .Pin = (1 << 3),
  });

  __HAL_RCC_ADC_CLK_ENABLE();
  adc1 = (ADC_HandleTypeDef){
    .Instance = ADC1,
    .Init = {
      .ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2, // APB/2 = 8 MHz
      .Resolution = ADC_RESOLUTION_12B,
      .DataAlign = ADC_DATAALIGN_RIGHT,
      .ScanConvMode = ADC_SCAN_DIRECTION_FORWARD,
      .EOCSelection = ADC_EOC_SINGLE_CONV,
      .SamplingTimeCommon = ADC_SAMPLETIME_41CYCLES_5,
    },
  };
  HAL_ADC_Init(&adc1);
  HAL_ADC_Calibration_Start(&adc1);
}

  void calibrate_vcc() {
    HAL_ADC_ConfigChannel(&adc1, &(ADC_ChannelConfTypeDef){
      .Channel = ADC_CHANNEL_3,
      .Rank = ADC_RANK_NONE,
    });
    HAL_ADC_ConfigChannel(&adc1, &(ADC_ChannelConfTypeDef){
      .Channel = ADC_CHANNEL_VREFINT,
      .Rank = ADC_RANK_CHANNEL_NUMBER,
    });

    ADC->CCR |= ADC_CCR_VREFEN;
    vrefint_value = 0;
    for (int i = 0; i < 16; i++) {
      delay_us(100);
      HAL_ADC_Start(&adc1);
      HAL_ADC_PollForConversion(&adc1, HAL_MAX_DELAY);
      vrefint_value += HAL_ADC_GetValue(&adc1);
      HAL_ADC_Stop(&adc1);
    }
    vrefint_value = (vrefint_value + 8) / 16;
    if (!vrefint_value) vrefint_value = 1;
    uint32_t vcc_mV = (1200 * 4096 / vrefint_value);
    printf("Vrefint reading = %lu, ", vrefint_value);
    printf("Vcc = %lu mV\n", vcc_mV);
    if (vcc_mV <= 3000 || vcc_mV >= 3600) {
      for (int i = 0; i < 10; i++) {
        ACT_ON(); delay_us(100000);
        ACT_OFF(); delay_us(100000);
      }
      HAL_NVIC_SystemReset();
    }

    HAL_ADC_ConfigChannel(&adc1, &(ADC_ChannelConfTypeDef){
      .Channel = ADC_CHANNEL_VREFINT,
      .Rank = ADC_RANK_NONE,
    });
    HAL_ADC_ConfigChannel(&adc1, &(ADC_ChannelConfTypeDef){
      .Channel = ADC_CHANNEL_3,
      .Rank = ADC_RANK_CHANNEL_NUMBER,
    });
  }
  calibrate_vcc();

  // TODO: Optimize this with DMA/interrupt
  uint32_t read_adc() {
    // Median filter; take central two quartiles
    // In experiments, this achieves +/- 10 mV consistently with the pumps and valves running
    static uint32_t v[128];
    for (int i = 0; i < 128; i++) {
      HAL_ADC_Start(&adc1);
      HAL_ADC_PollForConversion(&adc1, HAL_MAX_DELAY);
      uint32_t adc_value = HAL_ADC_GetValue(&adc1);
      // if (i % 2 == 1) printf("%4d ", (unsigned)adc_value);
      v[i] = adc_value;
      HAL_ADC_Stop(&adc1);
    }
    int uint32_cmp(const void *_a, const void *_b) {
      uint32_t a = *(const uint32_t *)_a;
      uint32_t b = *(const uint32_t *)_b;
      return a < b ? -1 : a == b ? 0 : 1;
    }
    qsort(v, sizeof(v) / sizeof(uint32_t), sizeof(uint32_t), uint32_cmp);
    // for (int i = 0; i < 16; i++)
    //   if (i % 2 == 0) printf("%4d ", (unsigned)v[i]);
    uint32_t sum = 0;
    for (int i = 48; i < 80; i++) sum += v[i];
    // Reading = round(sum / 32 * Vcc / 4096)
    // Vcc = 1.2 V / (vrefint_value / 4096)
    // Reading = round(sum / 32 * 1.2 V / vrefint_value)
    return ((sum * 375 + vrefint_value / 2) / vrefint_value); // Unit: 0.1 mV
  }

  // Test ADC sampling speed
if (0) {
  for (int i = 0; i < 10; i++) {
    HAL_Delay(2);
    uint32_t t = HAL_GetTick();
    int n = 0;
    while (HAL_GetTick() - t < 1000) { read_adc(); n++; }
    printf("ADC %d samples/s\n", n);
  }
}

  while (0) {
    printf("%u\n", (unsigned)read_adc());
    HAL_Delay(1000);
  }

  // ============ RS-485 driver enable signal ============ //
  HAL_GPIO_WritePin(GPIOA, (1 << 6), 0);
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pin = (1 << 6),
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  });

  // ============ RS-485 UART ============ //
  // PA4 AF9 = UART2_TX
  // PA5 AF9 = UART2_RX
{
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_AF_PP,
    .Pin = (1 << 4) | (1 << 5),
    .Pull = GPIO_NOPULL,
    .Alternate = 9,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_USART2_CLK_ENABLE();
  UART_HandleTypeDef uart2 = (UART_HandleTypeDef){
    .Instance = USART2,
    .Init = (UART_InitTypeDef){
      .BaudRate = 115200,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  };
  HAL_UART_Init(&uart2);
  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

  USART2->CR1 |= (USART_CR1_RXNEIE | USART_CR1_UE);
}

  // ============ Half-duplex UART ============ //
  // PA7 AF8 = UART1_TX
{
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_AF_PP,
    .Pin = (1 << 7),
    .Pull = GPIO_PULLUP,
    .Alternate = 8,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });
  __HAL_RCC_USART1_CLK_ENABLE();
  UART_HandleTypeDef uart1 = (UART_HandleTypeDef){
    .Instance = USART1,
    .Init = (UART_InitTypeDef){
      .BaudRate = 115200,
      .WordLength = UART_WORDLENGTH_8B,
      .StopBits = UART_STOPBITS_1,
      .Parity = UART_PARITY_NONE,
      .Mode = UART_MODE_TX_RX,
      .HwFlowCtl = UART_HWCONTROL_NONE,
      .OverSampling = UART_OVERSAMPLING_16,
    },
  };
  HAL_HalfDuplex_Init(&uart1);

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  USART1->CR1 |= (USART_CR1_RXNEIE | USART_CR1_UE);

  USART1->CR1 = (USART1->CR1 & ~(USART_CR1_TE | USART_CR1_RE)) | USART_CR1_RE;
}

  // ============ Watchdog ============ //
if (0)
{
  __HAL_RCC_LSI_ENABLE();
  while (!(RCC->CSR & RCC_CSR_LSIRDY)) { }

  __HAL_DBGMCU_FREEZE_IWDG(); // Freeze IWDG when core stopped in debug mode
  HAL_IWDG_Init(&(IWDG_HandleTypeDef){
    .Instance = IWDG,
    .Init = (IWDG_InitTypeDef){
      .Prescaler = IWDG_PRESCALER_32, // 1 kHz
      .Reload = 200,
    },
  });
}

  ACT_ON();

  void drain(unsigned reading) {
    uint32_t t0 = HAL_GetTick();
    TIM1->CCR3 = 150; TIM3->CCR1 = 160;
    while (read_adc() > reading && HAL_GetTick() - t0 < 5000)
      IWDG->KR = IWDG_KEY_RELOAD;
    TIM1->CCR3 = TIM3->CCR1 = 0;
  }
  drain(500);

  while (lane_index == -1) {
    IWDG->KR = IWDG_KEY_RELOAD;
    __WFI();
  }

  // Clear lights
  downstream_tx((uint8_t []){0x01 + lane_index, 0x00, 0x00, 0x00, 0x00}, 5);

  while (0) {
    ACT_ON();
    for (int i = 0; i < 256; i++) {
      uint16_t l = i * 16;
      downstream_tx((uint8_t []){0x01, l >> 8, l & 0xff, 0x00, 0x00}, 5);
      delay_us(10000);
      IWDG->KR = IWDG_KEY_RELOAD;
    }
  }

  enum {
    STATE_IDLE,
    STATE_INFLATE,
    STATE_DRAIN,
    STATE_FADE_OUT,
  } state = STATE_IDLE;
  int state_time = 0;

  op_t op = 0;

  inline bool try_take_op(unsigned mask) {
    if ((op & mask) == mask) {
      op ^= mask;
      return true;
    } else {
      return false;
    }
  }

  int intensity = 0;
  int progress = 0;

  int state_start_intensity = 0;
  bool inflate_is_first = false;

  const int base_progress = 2048;
  int breath_rate;
  int projected_phase = 0, eased_phase = 0;

  uint32_t tick = HAL_GetTick();
  while (1) {
    uint32_t pressure = read_adc();
    printf("%u\n", (unsigned)pressure);

re_switch:
    #define reset_state(_s) do { \
      state = (_s); state_time = 0; \
      state_start_intensity = intensity; \
      goto re_switch; \
    } while (0)
    #define reset_state_cont(_s) do { \
      state = (_s); \
      goto re_switch; \
    } while (0)

    if (pressure >= args.PRESSURE_BAIL) {
      // Dangerous level. Open valve and wait for 5 seconds
      // Should not happen during normal operation
      TIM1->CCR4 = 0;
      TIM1->CCR3 = 0; TIM3->CCR1 = 160;
      for (int i = 0; i < 500; i++) {
        if (i % 5 == 0) HAL_GPIO_TogglePin(GPIOB, 1 << 7);
        HAL_Delay(10);
        IWDG->KR = IWDG_KEY_RELOAD;
      }
    }

    switch (state) {
    case STATE_IDLE: {
      if (try_take_op(OP_INFLATE)) {
        inflate_is_first = true;
        projected_phase = eased_phase = base_progress;
        breath_rate = 3000;
        reset_state(STATE_INFLATE);
      }
      if (try_take_op(OP_DRAIN)) reset_state(STATE_DRAIN);

      intensity = 3072;
      if (state_time < 3072) intensity = state_time;

      progress = base_progress;
      if (state_time < 2048)
        // Quadratic ease
        progress = (2048 * 2048 -
          (2048 - state_time) * (2048 - state_time)) / 2048
          * base_progress / 2048;

      TIM1->CCR4 = 0;
      TIM1->CCR3 = 0; TIM3->CCR1 = 0;
    } break;

    case STATE_INFLATE: {
      if (try_take_op(OP_DRAIN)) {
        breath_rate = (breath_rate * 8 + state_time * 8 / 2) / 16;
        if (breath_rate < 1000) breath_rate = 1000;
        projected_phase = (projected_phase & ~4095) + 4096;
        reset_state(STATE_DRAIN);
      }
      if (try_take_op(OP_FADE_OUT)) reset_state(STATE_FADE_OUT);

      static bool stopped = false;
      if (state_time == 0) stopped = false;

      static int time_stop;
      if (!stopped && (
          state_time >= args.INFLATE_TIME_LIMIT || pressure >= args.PRESSURE_LIMIT)) {
        time_stop = state_time;
        stopped = true;
      }

      if (inflate_is_first) {
        intensity = 4096;
      } else {
        intensity = state_start_intensity + state_time * 2;
        if (intensity >= 3072) {
          intensity = 3072 + (intensity - 3072) / 2;
          if (intensity >= 4096) intensity = 4096;
        }
      }
      projected_phase = (projected_phase & ~4095) +
        (state_time >= breath_rate ? 4095 : state_time * 4095 / breath_rate);
      if (inflate_is_first) projected_phase = 4095;
      eased_phase = (eased_phase * 63 + projected_phase * 1) / 64;
      if (0 && inflate_is_first && state_time < 1024) {
        progress = (eased_phase * state_time
          + base_progress * (1024 - state_time)) / 1024 % 8192;
      } else if (0 && inflate_is_first && eased_phase < base_progress - 512) {
        progress = base_progress;
      } else if (0 && inflate_is_first && eased_phase < base_progress + 512) {
        progress = base_progress + (eased_phase - base_progress + 512) / 2;
      } else {
        progress = eased_phase % 8192;
      }

      uint32_t inflate_duty = 0;
      if (!stopped) {
        inflate_duty = args.PUMP_INFLATE_DUTY;
      } else {
        if (state_time - time_stop < args.PUMP_INFLATE_DUTY * 5)
          inflate_duty = args.PUMP_INFLATE_DUTY - (state_time - time_stop) / 5;
      }
      TIM1->CCR4 = inflate_duty;
      TIM1->CCR3 = 0; TIM3->CCR1 = 0;
    } break;

    case STATE_DRAIN: {
      if (try_take_op(OP_INFLATE)) {
        inflate_is_first = false;
        projected_phase = (projected_phase & ~4095) + 4096;
        reset_state(STATE_INFLATE);
      }
      if (try_take_op(OP_FADE_OUT)) reset_state(STATE_FADE_OUT);

      static bool stopped = false;
      if (state_time == 0) stopped = false;

      if (!stopped && (state_time >= 2000 || pressure < 1000))
        stopped = true;

      intensity = state_start_intensity - state_time * 3;
      if (intensity < 512) {
        intensity = 512 - (512 - intensity) / 2;
        if (intensity < 0) intensity = 0;
      }
      projected_phase = (projected_phase & ~4095) +
        (state_time >= breath_rate ? 4095 : state_time * 4095 / breath_rate);
      eased_phase = (eased_phase * 63 + projected_phase * 1) / 64;
      progress = eased_phase % 8192;

      uint32_t drain_duty = (!stopped ? args.PUMP_DRAIN_DUTY : 0);
      TIM1->CCR4 = 0;
      TIM1->CCR3 = drain_duty; TIM3->CCR1 = 160;
    } break;

    case STATE_FADE_OUT: {
      if (state_time >= 2000) reset_state(STATE_IDLE);

      intensity = state_start_intensity - state_time * 3;
      if (intensity < 0) intensity = 0;
      eased_phase = (eased_phase * 63 + projected_phase * 1) / 64;
      progress = eased_phase % 8192;

      static int min_pressure = 0;
      if (state_time == 0) min_pressure = pressure;
      else if (pressure < min_pressure) min_pressure = pressure;

      TIM1->CCR4 = 0;
      TIM1->CCR3 = (min_pressure >= 1000 ? args.PUMP_DRAIN_DUTY : 0);
      TIM3->CCR1 = (min_pressure >= 500 ? 160 : 0);
    } break;
    }
    if (state_time < 60000) state_time += 10;

    while (HAL_GetTick() - tick < 10) __WFI();
    tick += 10;
    IWDG->KR = IWDG_KEY_RELOAD;
    op = 0;

    downstream_tx((uint8_t []){
      0x01 + lane_index,
      intensity >> 8, intensity & 0xff,
      progress >> 8, progress & 0xff,
    }, 5);

  if (0) {
    // Operations simulation
    static int tt = 0;
    tt += 10;
    if (tt == 15000) tt = 0;
    if (tt == 4000) op = OP_INFLATE;
    if (tt == 6000) op = OP_DRAIN;
    if (tt == 8000) op = OP_INFLATE;
    if (tt == 10000) op = OP_DRAIN;
    if (tt == 11000) op = OP_INFLATE;
    if (tt == 13000) op = OP_FADE_OUT;
  }

    __disable_irq();
    op = op_pending;
    op_pending = 0;
    __enable_irq();
  }

  while (1) {
  }
}

static inline void downstream_driver_enable(bool enable)
{
  HAL_GPIO_WritePin(GPIOA, (1 << 6), enable);
}
static inline void downstream_tx_byte(uint8_t x)
{
  while (!(USART2->SR & USART_SR_TXE)) { }
  USART2->DR = x;
}
static inline void downstream_tx_finish()
{
  while (!(USART2->SR & USART_SR_TC)) { }
}
static inline uint32_t downstream_get_tick()
{
  return HAL_GetTick();
}

static inline void downstream_rx_process_packet(uint8_t *packet, uint8_t n)
{
  printf("ok %d\n", (int)n);
  HAL_GPIO_TogglePin(GPIOB, 1 << 7);
}

static inline void upstream_driver_enable(bool enable)
{
  USART1->CR1 = (USART1->CR1 & ~(USART_CR1_TE | USART_CR1_RE))
    | (enable ? USART_CR1_TE : USART_CR1_RE);
}
static inline void upstream_tx_byte(uint8_t x)
{
  while (!(USART1->SR & USART_SR_TXE)) { }
  USART1->DR = x;
}
static inline void upstream_tx_finish()
{
  while (!(USART1->SR & USART_SR_TC)) { }
}
static inline uint32_t upstream_get_tick()
{
  return HAL_GetTick();
}

static inline void upstream_rx_process_packet(uint8_t *packet, uint8_t n)
{
  if (n >= 9 && lane_index == -1 && packet[0] >= 0x10 && packet[0] <= 0x13) {
    lane_index = packet[0] - 0x10;
    args.PUMP_INFLATE_DUTY = packet[1];
    args.PUMP_DRAIN_DUTY = packet[2];
    args.PRESSURE_LIMIT = ((int)packet[3] << 8) | packet[4];
    args.PRESSURE_BAIL = ((int)packet[5] << 8) | packet[6];
    args.INFLATE_TIME_LIMIT = ((int)packet[7] << 8) | packet[8];
  }
  if (n >= 1) {
    if (packet[0] == 0xA1) op_pending |= OP_INFLATE;
    if (packet[0] == 0xA2) op_pending |= OP_DRAIN;
    if (packet[0] == 0xAF) op_pending |= OP_FADE_OUT;
  }
  // upstream_tx((uint8_t []){'o', 'k'}, 2);
  HAL_GPIO_TogglePin(GPIOB, 1 << 7);
}

void NMI_Handler() { while (1) { } }
void HardFault_Handler() { while (1) { } }
void SVC_Handler() { while (1) { } }
void PendSV_Handler() { while (1) { } }
void SysTick_Handler()
{
  HAL_IncTick();
}
void WWDG_IRQHandler() { while (1) { } }
void PVD_IRQHandler() { while (1) { } }
void RTC_IRQHandler() { while (1) { } }
void FLASH_IRQHandler() { while (1) { } }
void RCC_IRQHandler() { while (1) { } }
void EXTI0_1_IRQHandler() { while (1) { } }
void EXTI2_3_IRQHandler() { while (1) { } }
void EXTI4_15_IRQHandler() { while (1) { } }
void DMA1_Channel1_IRQHandler() { while (1) { } }
void DMA1_Channel2_3_IRQHandler() { while (1) { } }
void ADC_COMP_IRQHandler() { while (1) { } }
void TIM1_BRK_UP_TRG_COM_IRQHandler() { while (1) { } }
void TIM1_CC_IRQHandler() { while (1) { } }
void TIM3_IRQHandler() { while (1) { } }
void LPTIM1_IRQHandler() { while (1) { } }
void TIM14_IRQHandler() { while (1) { } }
void TIM16_IRQHandler() { while (1) { } }
void TIM17_IRQHandler() { while (1) { } }
void I2C1_IRQHandler() { while (1) { } }
void SPI1_IRQHandler() { while (1) { } }

#pragma GCC push_options
#pragma GCC optimize("O3")
void USART1_IRQHandler()
{
  uint8_t x = USART1->DR;
  upstream_rx_process_byte(x);
}

void USART2_IRQHandler()
{
  uint8_t x = USART2->DR;
  downstream_rx_process_byte(x);
}
#pragma GCC pop_options
