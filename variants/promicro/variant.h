#ifndef _VARIANT_PROMICRO_NRF52840_
#define _VARIANT_PROMICRO_NRF52840_

#define VARIANT_MCK (64000000ul)
#define USE_LFXO

#include "WVariant.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PINS_COUNT (19)
#define NUM_DIGITAL_PINS (19)
#define NUM_ANALOG_INPUTS (4)
#define NUM_ANALOG_OUTPUTS (0)

#define PIN_LED1 (18)
#define LED_BUILTIN PIN_LED1
#define LED_STATE_ON 1

#define PIN_A0 (14)
#define PIN_A1 (15)
#define PIN_A2 (16)
#define PIN_A3 (17)
static const uint8_t A0 = PIN_A0;
static const uint8_t A1 = PIN_A1;
static const uint8_t A2 = PIN_A2;
static const uint8_t A3 = PIN_A3;
#define ADC_RESOLUTION 14

// nice!nano / Pro Micro: D0 = P0.08 RX, D1 = P0.06 TX
#define PIN_SERIAL1_RX (0)
#define PIN_SERIAL1_TX (1)

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO (11)
#define PIN_SPI_MOSI (10)
#define PIN_SPI_SCK (12)
static const uint8_t SS = 13;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK = PIN_SPI_SCK;

#define WIRE_INTERFACES_COUNT 1
// Stacked MPU: SDA is silk 002 (P0.02), SCL is silk 029 (P0.29).
#define PIN_WIRE_SDA (16)
#define PIN_WIRE_SCL (15)

#ifdef __cplusplus
}
#endif

#endif
