#include "crsf.h"
#include "imu.h"
#include "settings.h"

static uint32_t lastSend = 0;
static bool portOpen = false;
static bool lastInv = false;

static uint16_t usToCrsf(uint16_t us) {
  int v = (int)lroundf(((int)us - 1500) * 1.6f + 992.0f);
  if (v < 172) v = 172;
  if (v > 1811) v = 1811;
  return (uint16_t)v;
}

static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0xD5);
      else crc <<= 1;
    }
  }
  return crc;
}

static void pack11(const uint16_t ch[16], uint8_t out[22]) {
  memset(out, 0, 22);
  uint32_t bit = 0;
  for (int i = 0; i < 16; i++) {
    uint32_t value = ch[i] & 0x7FF;
    uint32_t byte = bit >> 3;
    uint32_t off = bit & 7;
    uint32_t acc = value << off;
    out[byte] |= acc & 0xFF;
    if (byte + 1 < 22) out[byte + 1] |= (acc >> 8) & 0xFF;
    if (byte + 2 < 22) out[byte + 2] |= (acc >> 16) & 0xFF;
    bit += 11;
  }
}

static void openPort() {
  if (portOpen && lastInv == S.crsftxinv) return;
  // RP1 as an internal TX wants a normal (not JR-bay inverted) UART.
  // The checkbox is stored either way. This clone has no hardware inverter.
  Serial1.begin(400000);
  NRF_UARTE_Type *u = NRF_UARTE0;
  // The library has no 400000 step: begin() leaves the PHY at 460800.
  // BAUDRATE is only accepted while the UART is disabled, so a write done
  // after begin() never reaches the RP1. ELRS then never locks, and the
  // ER5C stays without a link. Stop, set 400000, and transmit only.
  u->EVENTS_RXTO = 0;
  u->EVENTS_TXSTOPPED = 0;
  u->TASKS_STOPRX = 1;
  u->TASKS_STOPTX = 1;
  uint32_t t0 = micros();
  while (!(u->EVENTS_TXSTOPPED && u->EVENTS_RXTO)) {
    if ((uint32_t)(micros() - t0) > 3000) break;
  }
  u->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
  u->BAUDRATE = 0x06666666UL;
  u->PSEL.RXD = 0xFFFFFFFFUL;
  u->ERRORSRC = 0x0FUL;
  u->EVENTS_ERROR = 0;
  u->EVENTS_ENDTX = 0;
  u->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
  u->INTENCLR = UARTE_INTENCLR_ENDRX_Msk;
  portOpen = true;
  lastInv = S.crsftxinv;
}

void crsfBegin() { openPort(); }

void crsfUpdate() {
  if (S.uartmode != UART_CRSF_OUT) {
    if (portOpen) {
      Serial1.end();
      portOpen = false;
    }
    return;
  }
  openPort();
  uint32_t period = 1000000UL / (uint32_t)S.crsftxrate;
  uint32_t now = micros();
  if ((uint32_t)(now - lastSend) < period) return;
  lastSend = now;

  uint16_t packed[16];
  for (int i = 0; i < 16; i++) packed[i] = usToCrsf(pose.channelUs[i]);
  uint8_t payload[22];
  pack11(packed, payload);

  uint8_t frame[26];
  frame[0] = 0xC8;
  frame[1] = 0x18;
  frame[2] = 0x16;
  memcpy(frame + 3, payload, 22);
  frame[25] = crc8(frame + 2, 23);
  Serial1.write(frame, sizeof(frame));
}
