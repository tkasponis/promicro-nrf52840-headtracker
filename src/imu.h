#pragma once

#include <Arduino.h>

struct Pose {
  bool imuOk = false;
  bool gyroCal = false;
  bool calibrated = false;
  float tilt = 0, roll = 0, pan = 0;
  float tiltOff = 0, rollOff = 0, panOff = 0;
  int tiltUs = 1500, rollUs = 1500, panUs = 1500;
  float accx = 0, accy = 0, accz = 0;
  float gyrox = 0, gyroy = 0, gyroz = 0;
  uint16_t channelUs[16];
};

extern Pose pose;
extern bool outputsEnabled;
extern uint32_t imuCenterAt;

void imuBegin();
void imuUpdate();
void imuResetCenter();
void imuOnSettingsChanged();
