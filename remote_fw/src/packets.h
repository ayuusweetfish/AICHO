#include <stdbool.h>
#include <stdint.h>

static inline void serial_tx_byte(uint8_t x);
static inline void serial_tx_finish(void);
static inline void serial_driver_enable(bool enable);
static inline uint32_t serial_get_tick();
static inline void serial_rx_process_packet(uint8_t *packet, uint8_t n);

static inline void serial_tx(const uint8_t *buf, uint8_t len)
{
  uint32_t s = crc32_bulk(buf, len);
  uint8_t s8[4] = {
    (uint8_t)(s >>  0),
    (uint8_t)(s >>  8),
    (uint8_t)(s >> 16),
    (uint8_t)(s >> 24),
  };

  serial_driver_enable(1);
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
  serial_tx_finish();
  __DMB();
  serial_driver_enable(0);
}

static inline void serial_rx_process_byte(uint8_t x)
{
  static bool is_in_escape = false;
  static uint8_t packet[16], ptr = 0;

  static uint32_t last_timestamp = (uint32_t)-100;
  uint32_t t = serial_get_tick();
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
