#include "variant.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "nrf.h"

// Arduino pin index -> nRF52840 GPIO (P1.xx = 32 + xx)
const uint32_t g_ADigitalPinMap[] = {
    8,   // 0  D0  P0.08  RX  -> RP1 TX
    6,   // 1  D1  P0.06  TX  -> RP1 RX
    17,  // 2  D2  P0.17
    20,  // 3  D3  P0.20
    22,  // 4  D4  P0.22
    24,  // 5  D5  P0.24
    32,  // 6  D6  P1.00
    11,  // 7  D7  P0.11
    36,  // 8  D8  P1.04
    38,  // 9  D9  P1.06
    9,   // 10 D10 P0.09
    10,  // 11 D11 P0.10
    43,  // 12 D12 P1.11
    45,  // 13 D13 P1.13
    31,  // 14 A0  P0.31  silk 031, held low: MPU GND
    29,  // 15 A1  P0.29  silk 029, SCL
    2,   // 16 A2  P0.02  silk 002, SDA
    47,  // 17 A3  P1.15  silk 115
    15,  // 18 LED P0.15
};

static void pinLowStrong(uint32_t psel) {
  NRF_GPIO_Type *port = (psel < 32) ? NRF_P0 : NRF_P1;
  uint32_t bit = psel & 31;
  port->OUTCLR = 1UL << bit;
  port->PIN_CNF[bit] =
      (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

static void pinDisconnect(uint32_t psel) {
  NRF_GPIO_Type *port = (psel < 32) ? NRF_P0 : NRF_P1;
  port->PIN_CNF[psel & 31] =
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

void initVariant() {
  pinMode(PIN_LED1, OUTPUT);
  digitalWrite(PIN_LED1, !LED_STATE_ON);
  // Stack, right side, top to bottom from VCC: VCC, GND, SCL, SDA.
  pinLowStrong(31);   // 031 = MPU GND
  pinLowStrong(45);   // 111 = AD0, address stays 0x68 if that leg is soldered
  pinDisconnect(47);  // 115 = XDA
  pinDisconnect(43);  // 113 = XCL
  pinDisconnect(10);  // 010 = INT
}
