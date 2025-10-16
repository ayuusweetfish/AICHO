#include <stdbool.h>
#include <stdint.h>

#define PACKETS_FN__(_i, _n) _i##_##_n
#define PACKETS_FN_(_i, _n) PACKETS_FN__(_i, _n)
#define PACKETS_FN(_n) PACKETS_FN_(PACKETS_INSTANCE_NAME, _n)

static inline void PACKETS_FN(tx_byte)(uint8_t x);
static inline void PACKETS_FN(tx_finish)(void);
static inline void PACKETS_FN(driver_enable)(bool enable);
static inline uint32_t PACKETS_FN(get_tick)();
static inline void PACKETS_FN(rx_process_packet)(uint8_t *packet, uint8_t n);

static inline void PACKETS_FN(tx)(const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  PACKETS_FN(driver_enable)(1);
  __DMB();
  for (int i = 0; i < len + 4; i++) {
    uint8_t x = (i < len ? buf[i] : s8[i - len]);
    if (x == 0xAA || x == 0x55) {
      PACKETS_FN(tx_byte)(0x55);
      PACKETS_FN(tx_byte)(x ^ 0xF0);
    } else {
      PACKETS_FN(tx_byte)(x);
    }
  }
  PACKETS_FN(tx_byte)(0xAA);
  PACKETS_FN(tx_finish)();
  __DMB();
  PACKETS_FN(driver_enable)(0);
}

static inline void PACKETS_FN(rx_process_byte)(uint8_t x)
{
  static bool is_in_escape = false;
  static uint8_t packet[16], ptr = 0;

  static uint32_t last_timestamp = (uint32_t)-100;
  uint32_t t = PACKETS_FN(get_tick)();
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
      PACKETS_FN(rx_process_packet)(packet, ptr);
      ptr = 0;
    } else if (x == 0x55) {
      is_in_escape = true;
    } else {
      if (ptr < sizeof packet) packet[ptr++] = x;
    }
  }
}
