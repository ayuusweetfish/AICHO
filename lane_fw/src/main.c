#include "py32f0xx_hal.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// #define RELEASE
#include "debug_printf.h"

#include "crc32.h"

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

static inline void serial_tx(const uint8_t *buf, uint8_t len);

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

  printf("sysclk = %lu Hz\n", HAL_RCC_GetSysClockFreq());
  while (0) {   // Test usage
    delay_us(1000000);
    printf("ticks = %lu\n", HAL_GetTick());
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

  HAL_ADC_ConfigChannel(&adc1, &(ADC_ChannelConfTypeDef){
    .Channel = ADC_CHANNEL_3,
    .Rank = ADC_RANK_CHANNEL_NUMBER,
    .SamplingTime = ADC_SAMPLETIME_41CYCLES_5,  // Obsolete
  });
}

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
    for (int i = 0; i < 128; i++)
      for (int j = i; j < 128; j++)
        if (v[i] > v[j]) { uint32_t t = v[i]; v[i] = v[j]; v[j] = t; }
    // for (int i = 0; i < 16; i++)
    //   if (i % 2 == 0) printf("%4d ", (unsigned)v[i]);
    uint32_t sum = 0;
    for (int i = 48; i < 80; i++) sum += v[i];
    return (sum * 4125 / 4 /* * 33000 / 32 */ + 2048) / 4096;  // Unit: 0.1 mV
  }

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

  USART2->CR1 |= USART_CR1_RXNEIE;
  USART2->CR1 |= USART_CR1_UE;
}

  ACT_ON();
  while (1) {
    for (int i = 0; i < 256; i++) {
      uint16_t l = i * 16;
      serial_tx((uint8_t []){0x01, l >> 8, l & 0xff}, 3);
      delay_us(10000);
    }
  }

if (0) {
  TIM1->CCR3 = 200; TIM3->CCR1 = 200;
  HAL_Delay(1000);
  TIM1->CCR3 = TIM3->CCR1 = 0;
  while (1) {
    uint32_t v[25][3];
    for (int i = 0; i < 25; i++) {
      TIM1->CCR4 = (i < 10 ? 150 : 0);
      delay_us(100000);
      v[i][0] = read_adc();
      v[i][1] = read_adc();
      v[i][2] = read_adc();
    }
    for (int i = 0; i < 25; i++) {
      printf("%2d %5u %5u %5u\n", i, (unsigned)v[i][0], (unsigned)v[i][1], (unsigned)v[i][2]);
    }
    TIM1->CCR3 = 150; TIM3->CCR1 = 200;
    HAL_Delay(1500);
    TIM1->CCR3 = TIM3->CCR1 = 0;
  }
}

  void inflate(unsigned reading) {
    uint32_t t0 = HAL_GetTick();
    TIM1->CCR4 = 150;
    while (read_adc() < reading && HAL_GetTick() - t0 < 2000) { }
    TIM1->CCR4 = 0;
  }
  void drain(unsigned reading) {
    uint32_t t0 = HAL_GetTick();
    TIM1->CCR3 = 150; TIM3->CCR1 = 200;
    while (read_adc() > reading && HAL_GetTick() - t0 < 5000) { }
    TIM1->CCR3 = TIM3->CCR1 = 0;
  }
  drain(500);
  while (1) {
    ACT_ON(); inflate(8500); ACT_OFF();
    HAL_Delay(500);
    ACT_ON(); drain(500); ACT_OFF(); 
    HAL_Delay(500);
  }
}

static inline void serial_tx_byte(uint8_t x)
{
  while (!(USART2->SR & USART_SR_TXE)) { }
  USART2->DR = x;
}

static inline void serial_tx(const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  HAL_GPIO_WritePin(GPIOA, (1 << 6), 1);
  __DMB();
  for (int i = 0; i < len + 4; i++) {
    uint8_t x = (i < len ? buf[i] : s8[i - len]);
    if (x == 0xAA || x == 0x55) {
      serial_tx_byte(0x55);
      serial_tx_byte(x ^ 0xF0);
    } else {
      serial_tx_byte(x);
    }
  }
  serial_tx_byte(0xAA);
  while (!(USART2->SR & USART_SR_TC)) { }
  __DMB();
  HAL_GPIO_WritePin(GPIOA, (1 << 6), 0);
}

static inline void serial_rx_process_packet(uint8_t *packet, uint8_t n)
{
  if (crc32_bulk(packet, n) != 0x2144DF1C) return;
  printf("ok %d\n", (int)n);
}

static inline void serial_rx_process_byte(uint8_t x)
{
  static bool is_in_escape = false;
  static uint8_t packet[16], ptr = 0;

  static uint32_t last_timestamp = (uint32_t)-100;
  uint32_t t = HAL_GetTick();
  if (t - last_timestamp >= 100) {
    // Reset
    is_in_escape = false;
    ptr = 0;
  }
  last_timestamp = t;

  if (is_in_escape) {
    if (ptr < sizeof packet) packet[ptr++] = x ^ 0xF0;
    is_in_escape = false;
  } else {
    if (x == 0xAA) {
      serial_rx_process_packet(packet, ptr);
      ptr = 0;
    } else if (x == 0x55) {
      is_in_escape = true;
    } else {
      if (ptr < sizeof packet) packet[ptr++] = x;
    }
  }
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
void USART1_IRQHandler() { while (1) { } }

#pragma GCC push_options
#pragma GCC optimize("O3")
void USART2_IRQHandler()
{
  uint8_t x = USART2->DR;
  serial_rx_process_byte(x);
}
#pragma GCC pop_options
