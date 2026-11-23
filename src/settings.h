#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>

// Same numeric values as HeadTracker GUI basetrackersettings.h
static const int CH_OFF = -1;
static const int UART_OFF = 0;
static const int UART_SBUS = 1;
static const int UART_CRSF_IN = 2;
static const int UART_CRSF_OUT = 3;
static const int PPM_CENTER = 1500;
static const int PPM_MIN = 988;
static const int PPM_MAX = 2012;
static const uint8_t REV_TILT = 1;
static const uint8_t REV_ROLL = 2;
static const uint8_t REV_PAN = 4;

struct Settings {
  int rll_min = 1050;
  int rll_max = 1950;
  int rll_cnt = PPM_CENTER;
  float rll_gain = 5;
  int tlt_min = 1050;
  int tlt_max = 1950;
  int tlt_cnt = PPM_CENTER;
  float tlt_gain = 5;
  int pan_min = 1050;
  int pan_max = 1950;
  int pan_cnt = PPM_CENTER;
  float pan_gain = 5;
  int tltch = 2;
  int rllch = CH_OFF;
  int panch = 1;
  int alertch = CH_OFF;
  int pwm0 = CH_OFF, pwm1 = CH_OFF, pwm2 = CH_OFF, pwm3 = CH_OFF;
  int an0ch = CH_OFF, an1ch = CH_OFF, an2ch = CH_OFF, an3ch = CH_OFF;
  int aux0ch = CH_OFF, aux1ch = CH_OFF, aux2ch = CH_OFF;
  int rstppm = CH_OFF;
  int aux0func = 0, aux1func = 0, aux2func = 0;
  float an0gain = 310, an1gain = 310, an2gain = 310, an3gain = 310;
  float an0off = 0, an1off = 0, an2off = 0, an3off = 0;
  uint8_t servoreverse = 0;
  float magxoff = 0, magyoff = 0, magzoff = 0;
  float accxoff = 0, accyoff = 0, acczoff = 0;
  float gyrxoff = 0, gyryoff = 0, gyrzoff = 0;
  float so00 = 1, so01 = 0, so02 = 0;
  float so10 = 0, so11 = 1, so12 = 0;
  float so20 = 0, so21 = 0, so22 = 1;
  bool dismag = true;
  float rotx = 0, roty = 0, rotz = 0;
  int uartmode = UART_CRSF_OUT;
  int crsftxrate = 100;
  int sbustxrate = 80;
  bool sbininv = true;
  bool sboutinv = true;
  bool crsftxinv = false;
  bool ch5arm = true;
  int btmode = 0;
  bool rstonwave = false;
  bool butlngps = false;
  bool rstontlt = false;
  bool rstondblttap = true;
  float rstondbltapthres = 80;
  float rstondbltapmin = 100;
  float rstondbltapmax = 400;
  bool dbltapmig = false;
  bool ppmoutinvert = false;
  bool ppmininvert = false;
  int ppmframe = 22500;
  int ppmsync = 350;
  int ppmchcnt = 8;
  String btpairedaddress;
};

extern Settings S;

bool settingsBegin();
bool settingsSave();
void settingsErase();
void settingsToJson(JsonObject obj);
void settingsFromJson(JsonObjectConst obj);
bool axisReversed(uint8_t bit);
