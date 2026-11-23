#include <Arduino.h>
#include <nrf.h>
#include "crsf.h"
#include "imu.h"
#include "protocol.h"
#include "settings.h"

static void pollCenterButton() {
  static bool ready = false;
  static int last = HIGH;
  static uint32_t downAt = 0;
  if (!ready) {
    pinMode(2, INPUT_PULLUP);
    ready = true;
    return;
  }
  int v = digitalRead(2);
  uint32_t now = millis();
  if (v == LOW) {
    if (last == HIGH) downAt = now;
  } else if (last == LOW && downAt && (uint32_t)(now - downAt) >= 60 && (uint32_t)(now - downAt) < 1500) {
    imuResetCenter();
    downAt = 0;
  }
  last = v;
}

static void blinkLed() {
  uint32_t now = millis();
  if (imuCenterAt && (uint32_t)(now - imuCenterAt) < 90) {
    digitalWrite(LED_BUILTIN, !LED_STATE_ON);
    return;
  }
  if (!pose.imuOk) {
    // Sensor not connected yet: one short flash each second, then dark.
    digitalWrite(LED_BUILTIN, ((now % 1000) < 40) ? LED_STATE_ON : !LED_STATE_ON);
  } else if (!pose.gyroCal) {
    digitalWrite(LED_BUILTIN, ((now / 400) & 1) ? LED_STATE_ON : !LED_STATE_ON);
  } else {
    digitalWrite(LED_BUILTIN, LED_STATE_ON);
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  // 031 is the stacked MPU ground. Keep sinking after any later pin init.
  NRF_P0->OUTCLR = 1UL << 31;
  NRF_P0->PIN_CNF[31] =
      (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
  settingsBegin();
  protocolBegin();
  for (int i = 0; i < 16; i++) pose.channelUs[i] = PPM_CENTER;
  if (S.ch5arm) pose.channelUs[4] = 2000;
  // UART to the RP1 starts before the sensor, so a slow IMU probe cannot drop the link.
  crsfBegin();
  crsfUpdate();
  imuBegin();
}

void loop() {
  pollCenterButton();
  imuUpdate();
  crsfUpdate();
  protocolPoll();
  blinkLed();
}
