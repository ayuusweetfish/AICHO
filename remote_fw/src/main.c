#include "py32f0xx_hal.h"
#include "py32f0xx_ll_rcc.h"
#include "py32f0xx_ll_tim.h"
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

// #define RELEASE
#include "debug_printf.h"

#define LED_WRITE_I(_i, _c) do { \
  for (int _j = 0; _j < 8; _j++) out[(_i) * 24 +  0 + _j] = ((((_c).g >> (7 - _j)) & 1) ? 13 : 6); \
  for (int _j = 0; _j < 8; _j++) out[(_i) * 24 +  8 + _j] = ((((_c).r >> (7 - _j)) & 1) ? 13 : 6); \
  for (int _j = 0; _j < 8; _j++) out[(_i) * 24 + 16 + _j] = ((((_c).b >> (7 - _j)) & 1) ? 13 : 6); \
} while (0)
#include "leds.h"

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

#define PACKETS_INSTANCE_NAME serial
#include "packets.h"

static volatile uint8_t lights_type = 3;
static volatile uint16_t lights_intensity = 512, lights_progress = 0;

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

  static DMA_HandleTypeDef dma1_ch1;

  // ============ Light strip ============ //
  // PB6 AF1 = TIM1_CH3
{
  HAL_GPIO_Init(GPIOB, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_AF_PP,
    .Pin = (1 << 6),
    .Alternate = 1,
    .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
  });

  __HAL_RCC_TIM1_CLK_ENABLE();
  TIM_HandleTypeDef tim1 = {
    .Instance = TIM1,
    .Init = {
      .Prescaler = 1 - 1,   // 16 MHz
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 20 - 1,     // 800 kHz
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
    },
  };
  HAL_TIM_PWM_Init(&tim1);
  HAL_TIM_PWM_ConfigChannel(&tim1, &(TIM_OC_InitTypeDef){
    .OCMode = TIM_OCMODE_PWM1,
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  }, TIM_CHANNEL_3);

  __HAL_RCC_DMA_CLK_ENABLE();
  // Reference manual speficies that all channels map to all peripherals
  dma1_ch1 = (DMA_HandleTypeDef){
    .Instance = DMA1_Channel1,
    .Init = {
      .Direction = DMA_MEMORY_TO_PERIPH,
      .PeriphInc = DMA_PINC_DISABLE,
      .MemInc = DMA_MINC_ENABLE,
      .PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD,
      .MemDataAlignment = DMA_MDATAALIGN_BYTE,
      .Mode = DMA_NORMAL,
      .Priority = DMA_PRIORITY_HIGH,
    },
  };
  HAL_DMA_Init(&dma1_ch1);
  HAL_DMA_ChannelMap(&dma1_ch1, DMA_CHANNEL_MAP_TIM1_UP);
  __HAL_LINKDMA(&tim1, hdma[TIM_DMA_ID_UPDATE], dma1_ch1);
  __HAL_TIM_ENABLE_OCxPRELOAD(&tim1, TIM_CHANNEL_3);

  LL_TIM_ConfigDMABurst(TIM1, LL_TIM_DMABURST_BASEADDR_CCR3, LL_TIM_DMABURST_LENGTH_1TRANSFER);
  LL_TIM_EnableDMAReq_UPDATE(TIM1);
  __HAL_TIM_ENABLE_DMA(&tim1, TIM_DMA_UPDATE);

  TIM1->CCR3 = 0;
  HAL_TIM_PWM_Start(&tim1, TIM_CHANNEL_3);
}

  void send_lights_raw(int n, uint8_t light_buf[]) {
    light_buf[n * 24] = 0;
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->CNT = TIM1->ARR;
      // 0 holds PWM output high during the time
      // from when DMA starts and timer is re-enabled
    __DMB();  // Data Memory Barrier
    HAL_DMA_Start(&dma1_ch1, (uint32_t)light_buf, (uint32_t)&TIM1->DMAR, n * 24 + 1);
    TIM1->CR1 |= TIM_CR1_CEN;
  }
  void send_lights(int n, const uint32_t a[]) {
    static uint8_t light_buf[24 * 8 + 1];
    if (n >= 8) n = 8;
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < 24; j++)
        light_buf[i * 24 + j] = ((a[i] >> (23 - j)) & 1 ? 13 : 6);
    }
    send_lights_raw(n, light_buf);
  }
  void wait_lights() {
    HAL_DMA_PollForTransfer(&dma1_ch1, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
  }

  // ============ RS-485 driver enable signal ============ //
  HAL_GPIO_WritePin(GPIOA, (1 << 6), 0);
  HAL_GPIO_Init(GPIOA, &(GPIO_InitTypeDef){
    .Mode = GPIO_MODE_OUTPUT_PP,
    .Pin = (1 << 6),
    .Pull = GPIO_NOPULL,
    .Speed = GPIO_SPEED_FREQ_LOW,
  });

  // ============ UART ============ //
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

  static uint8_t a[128 * 24 + 1];
  uint32_t ticks = HAL_GetTick();
  uint32_t count = 0;
  while (1) {
    if (++count == 500) count = 0;
    if (count < 20) ACT_ON(); else ACT_OFF();
    wait_lights();
    __disable_irq();
    uint8_t t = lights_type;
    uint16_t l = lights_intensity;
    uint16_t p = lights_progress;
    __enable_irq();
    switch (t) {
    case 1:
      gradient_Lorivox(l, a);
      send_lights_raw(LED_N_Lorivox, a);
      break;
    case 2:
      gradient_Lumisonic(l, p, a);
      send_lights_raw(LED_N_Lumisonic, a);
      break;
    case 3:
      gradient_Harmonia(l, a);
      send_lights_raw(LED_N_Harmonia, a);
      break;
    case 4:
      gradient_Titanus(l, a);
      send_lights_raw(LED_N_Titanus, a);
      break;
    default:
    }
    while (HAL_GetTick() - ticks < 2) __WFI();
  }
}

static inline void serial_driver_enable(bool enable)
{
  HAL_GPIO_WritePin(GPIOA, (1 << 6), enable);
}
static inline void serial_tx_byte(uint8_t x)
{
  while (!(USART2->SR & USART_SR_TXE)) { }
  USART2->DR = x;
}
static inline void serial_tx_finish()
{
  while (!(USART2->SR & USART_SR_TC)) { }
}
static inline uint32_t serial_get_tick()
{
  return HAL_GetTick();
}

static inline void serial_rx_process_packet(uint8_t *packet, uint8_t n)
{
  if (n >= 5 && (packet[0] >= 0x01 && packet[0] <= 0x04)) {
    lights_type = packet[0];
    lights_intensity = ((uint16_t)packet[1] << 8) | packet[2];
    lights_progress = ((uint16_t)packet[3] << 8) | packet[4];
  }
  serial_tx((uint8_t []){0xAC, lights_type}, 2);
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
