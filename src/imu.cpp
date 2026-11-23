#include "crsf.h"
#include "imu.h"
#include "protocol.h"
#include "settings.h"
#include <math.h>
#include <stdio.h>

Pose pose;
bool outputsEnabled = true;
uint32_t imuCenterAt = 0;

static uint8_t mpuAddr = 0x68;
static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;
static float gxBias = 0, gyBias = 0, gzBias = 0;
static float lastRotX = 0, lastRotY = 0, lastRotZ = 0;
static uint32_t lastUs = 0;
static bool yawInit = false;
static bool seedQuat = true;
static bool snapOut = false;
static float panCont = 0;
static float lastYaw = 0;
static float fTilt = 0, fRoll = 0, fPan = 0;
static float biasStill = 0;
static float stillTime = 0;
static float slowX = 0, slowY = 0, slowZ = 0;
static float fastX = 0, fastY = 0, fastZ = 0;
static bool meanInit = false;
static bool centeredOnce = false;
static bool detent = true;
static bool lockBias = false;
static int badSamples = 0;

static uint8_t waveState = 0;
static uint32_t waveStart = 0;
static uint32_t waveHold = 0;
static bool waveHolding = false;

static void rotX(float &x, float &y, float &z, float deg) {
  float a = deg * (float)DEG_TO_RAD;
  float c = cosf(a), s = sinf(a);
  float y2 = y * c - z * s;
  float z2 = y * s + z * c;
  y = y2;
  z = z2;
}

static void rotY(float &x, float &y, float &z, float deg) {
  float a = deg * (float)DEG_TO_RAD;
  float c = cosf(a), s = sinf(a);
  float x2 = x * c + z * s;
  float z2 = -x * s + z * c;
  x = x2;
  z = z2;
}

static void rotZ(float &x, float &y, float &z, float deg) {
  float a = deg * (float)DEG_TO_RAD;
  float c = cosf(a), s = sinf(a);
  float x2 = x * c - y * s;
  float y2 = x * s + y * c;
  x = x2;
  y = y2;
}

static void rotate(float &x, float &y, float &z) {
  rotX(x, y, z, S.rotx);
  rotY(x, y, z, S.roty);
  rotZ(x, y, z, S.rotz);
}

// Bit-banged open-drain I2C. The nRF TWIM at 400 kHz never got an ACK on this stack,
// and a missed read left both servos parked at 1500. Half-period is 10 us (~50 kHz).
static const uint32_t I2C_HALF_US = 10;

static NRF_GPIO_Type *pinPort(uint32_t arduinoPin, uint32_t *bit) {
  uint32_t psel = g_ADigitalPinMap[arduinoPin];
  *bit = psel & 31;
  return (psel < 32) ? NRF_P0 : NRF_P1;
}

static void lineRelease(uint32_t arduinoPin) {
  uint32_t bit;
  NRF_GPIO_Type *port = pinPort(arduinoPin, &bit);
  port->PIN_CNF[bit] =
      (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Pullup << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_S0D1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

static void lineDriveLow(uint32_t arduinoPin) {
  uint32_t bit;
  NRF_GPIO_Type *port = pinPort(arduinoPin, &bit);
  port->OUTCLR = 1UL << bit;
  port->PIN_CNF[bit] =
      (GPIO_PIN_CNF_DIR_Output << GPIO_PIN_CNF_DIR_Pos) |
      (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) |
      (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) |
      (GPIO_PIN_CNF_DRIVE_H0H1 << GPIO_PIN_CNF_DRIVE_Pos) |
      (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}

static bool lineIsHigh(uint32_t arduinoPin) {
  uint32_t bit;
  NRF_GPIO_Type *port = pinPort(arduinoPin, &bit);
  return (port->IN >> bit) & 1UL;
}

static bool sclHighWait() {
  lineRelease(PIN_WIRE_SCL);
  uint32_t start = micros();
  while (!lineIsHigh(PIN_WIRE_SCL)) {
    if ((uint32_t)(micros() - start) > 1500) return false;
  }
  delayMicroseconds(I2C_HALF_US);
  lineDriveLow(PIN_WIRE_SCL);
  return true;
}

static void i2cStop() {
  lineDriveLow(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  lineRelease(PIN_WIRE_SCL);
  delayMicroseconds(I2C_HALF_US);
  lineRelease(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
}

static void i2cRelease() {
  NRF_TWIM0->ENABLE = TWIM_ENABLE_ENABLE_Disabled << TWIM_ENABLE_ENABLE_Pos;
  NRF_TWIM0->PSEL.SCL = 0xFFFFFFFFUL;
  NRF_TWIM0->PSEL.SDA = 0xFFFFFFFFUL;
  lineRelease(PIN_WIRE_SCL);
  lineRelease(PIN_WIRE_SDA);
  delay(2);
  for (int i = 0; i < 9 && !lineIsHigh(PIN_WIRE_SDA); i++) {
    lineDriveLow(PIN_WIRE_SCL);
    delayMicroseconds(I2C_HALF_US);
    lineRelease(PIN_WIRE_SCL);
    delayMicroseconds(I2C_HALF_US);
  }
  i2cStop();
}

static bool i2cStart() {
  lineRelease(PIN_WIRE_SDA);
  lineRelease(PIN_WIRE_SCL);
  delayMicroseconds(I2C_HALF_US);
  if (!lineIsHigh(PIN_WIRE_SDA) || !lineIsHigh(PIN_WIRE_SCL)) return false;
  lineDriveLow(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  lineDriveLow(PIN_WIRE_SCL);
  return true;
}

static bool i2cRestart() {
  lineRelease(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  lineRelease(PIN_WIRE_SCL);
  delayMicroseconds(I2C_HALF_US);
  if (!lineIsHigh(PIN_WIRE_SCL)) return false;
  lineDriveLow(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  lineDriveLow(PIN_WIRE_SCL);
  return true;
}

static bool i2cWriteByte(uint8_t data) {
  for (int bit = 0; bit < 8; bit++) {
    if (data & 0x80) lineRelease(PIN_WIRE_SDA);
    else lineDriveLow(PIN_WIRE_SDA);
    data <<= 1;
    delayMicroseconds(I2C_HALF_US);
    if (!sclHighWait()) return false;
  }
  lineRelease(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  lineRelease(PIN_WIRE_SCL);
  uint32_t start = micros();
  while (!lineIsHigh(PIN_WIRE_SCL)) {
    if ((uint32_t)(micros() - start) > 1500) return false;
  }
  delayMicroseconds(I2C_HALF_US);
  bool ack = !lineIsHigh(PIN_WIRE_SDA);
  lineDriveLow(PIN_WIRE_SCL);
  return ack;
}

static bool i2cReadByte(uint8_t *data, bool ack) {
  uint8_t value = 0;
  lineRelease(PIN_WIRE_SDA);
  for (int bit = 0; bit < 8; bit++) {
    delayMicroseconds(I2C_HALF_US);
    lineRelease(PIN_WIRE_SCL);
    uint32_t start = micros();
    while (!lineIsHigh(PIN_WIRE_SCL)) {
      if ((uint32_t)(micros() - start) > 1500) return false;
    }
    delayMicroseconds(I2C_HALF_US);
    value = (uint8_t)((value << 1) | (lineIsHigh(PIN_WIRE_SDA) ? 1 : 0));
    lineDriveLow(PIN_WIRE_SCL);
  }
  if (ack) lineDriveLow(PIN_WIRE_SDA);
  else lineRelease(PIN_WIRE_SDA);
  delayMicroseconds(I2C_HALF_US);
  if (!sclHighWait()) return false;
  lineRelease(PIN_WIRE_SDA);
  *data = value;
  return true;
}

static bool mpuRead(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len) {
  if (!i2cStart()) return false;
  if (!i2cWriteByte(addr << 1) || !i2cWriteByte(reg) || !i2cRestart() || !i2cWriteByte((addr << 1) | 1)) {
    i2cStop();
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    if (!i2cReadByte(&data[i], i + 1 < len)) {
      i2cStop();
      return false;
    }
  }
  i2cStop();
  return true;
}

static bool mpuWrite(uint8_t reg, uint8_t val) {
  if (!i2cStart()) return false;
  bool ok = i2cWriteByte(mpuAddr << 1) && i2cWriteByte(reg) && i2cWriteByte(val);
  i2cStop();
  return ok;
}

static float accDiv = 8192.0f;
static float gyrDiv = 16.4f;
static bool accScaled = false;
static int accLow = 0;
static int accHigh = 0;

static bool mpuConfigure() {
  bool ok = mpuWrite(0x6B, 0x01);
  mpuWrite(0x6C, 0x00);
  mpuWrite(0x1A, 0x03);
  mpuWrite(0x1B, 0x18);  // ±2000 °/s. ±250 clipped a fast look and the lost degrees never came back.
  mpuWrite(0x1C, 0x08);  // ±4 g
  mpuWrite(0x19, 0x04);
  uint8_t g = 0, a = 0;
  if (mpuRead(mpuAddr, 0x1B, &g, 1) && mpuRead(mpuAddr, 0x1C, &a, 1)) {
    static const float gs[4] = {131.0f, 65.5f, 32.8f, 16.4f};
    static const float as[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
    gyrDiv = gs[(g >> 3) & 3];
    accDiv = as[(a >> 3) & 3];
  }
  return ok;
}

static void mpuWake() {
  mpuWrite(0x6B, 0x80);
  for (int i = 0; i < 20; i++) {
    delay(5);
    crsfUpdate();
  }
  mpuConfigure();
  for (int i = 0; i < 8; i++) {
    delay(5);
    crsfUpdate();
  }
}

static void madgwick(float gx, float gy, float gz, float dt) {
  // Gyro only. Heading is this integral. Accel is applied afterwards with the
  // same yaw written back, so it can level tilt without walking pan.
  float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
  float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
  float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
  q0 += qDot0 * dt;
  q1 += qDot1 * dt;
  q2 += qDot2 * dt;
  q3 += qDot3 * dt;
  float norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (norm > 0) {
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
  }
}

static float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

static void seedFromAccel(float ax, float ay, float az) {
  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  q0 = cr * cp;
  q1 = sr * cp;
  q2 = cr * sp;
  q3 = -sr * sp;
}

static void setAttitude(float rollDeg, float pitchDeg, float yawDeg) {
  float roll = rollDeg * (float)DEG_TO_RAD;
  float pitch = pitchDeg * (float)DEG_TO_RAD;
  float yaw = yawDeg * (float)DEG_TO_RAD;
  float cr = cosf(roll * 0.5f), sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
  float cy = cosf(yaw * 0.5f), sy = sinf(yaw * 0.5f);
  float w = cr * cp, x = sr * cp, y = cr * sp, z = -sr * sp;
  q0 = cy * w - sy * z;
  q1 = cy * x - sy * y;
  q2 = sy * x + cy * y;
  q3 = sy * w + cy * z;
}

static bool quatOk() {
  if (q0 != q0 || q1 != q1 || q2 != q2 || q3 != q3) return false;
  float n = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  return n > 0.5f && n < 1.5f;
}

static void eulerEarth(float *tilt, float *roll, float *pan) {
  float sinp = 2.0f * (q0 * q2 - q1 * q3);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  *tilt = asinf(sinp) * (float)RAD_TO_DEG;
  *roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * (float)RAD_TO_DEG;
  *pan = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * (float)RAD_TO_DEG;
}

static float follow(float cur, float goal, float dt) {
  float a = dt / (dt + 0.008f);
  float next = cur + (goal - cur) * a;
  float maxStep = 900.0f * dt;
  float d = next - cur;
  if (d > maxStep) next = cur + maxStep;
  if (d < -maxStep) next = cur - maxStep;
  return next;
}

static int clampUs(int us, int lo, int hi) {
  if (us < lo) us = lo;
  if (us > hi) us = hi;
  if (us < PPM_MIN) us = PPM_MIN;
  if (us > PPM_MAX) us = PPM_MAX;
  return us;
}

static void clampPanToServo() {
  float gain = S.pan_gain;
  if (gain < 0.05f) gain = 0.05f;
  float loA = (S.pan_min - S.pan_cnt) / gain;
  float hiA = (S.pan_max - S.pan_cnt) / gain;
  if (axisReversed(REV_PAN)) {
    loA = -loA;
    hiA = -hiA;
  }
  if (loA > hiA) {
    float swap = loA;
    loA = hiA;
    hiA = swap;
  }
  float rel = panCont - pose.panOff;
  if (rel > hiA) panCont = pose.panOff + hiA;
  else if (rel < loA) panCont = pose.panOff + loA;
}

static int axisUs(float angle, float offset, bool reverse, float gain, int center, int lo, int hi) {
  float a = angle - offset;
  if (reverse) a = -a;
  return clampUs((int)lroundf(center + a * gain), lo, hi);
}

static void assignChannel(int channel, int us) {
  if (channel >= 1 && channel <= 16) pose.channelUs[channel - 1] = (uint16_t)us;
}

static void updateWave(uint32_t now) {
  if (!S.rstontlt) {
    waveState = 0;
    waveHolding = false;
    return;
  }
  bool atMin = pose.rollUs <= S.rll_min + 8;
  bool atMax = pose.rollUs >= S.rll_max - 8;
  bool atCenter = abs(pose.rollUs - S.rll_cnt) < 40;
  if (waveState == 0 && (atMin || atMax)) {
    waveState = atMin ? 1 : 2;
    waveStart = now;
  } else if (waveState == 1 && atMax && now - waveStart < 1500) {
    waveState = 3;
  } else if (waveState == 2 && atMin && now - waveStart < 1500) {
    waveState = 3;
  } else if (waveState && waveState < 3 && now - waveStart > 1500) {
    waveState = 0;
  }
  if (waveState == 3 && atCenter) {
    if (!waveHolding) {
      waveHolding = true;
      waveHold = now;
    } else if (now - waveHold > 1000) {
      imuResetCenter();
      waveState = 0;
      waveHolding = false;
    }
  } else {
    waveHolding = false;
  }
}

static uint32_t tapIgnoreUntil = 0;
static uint32_t tapLast = 0;
static float tapFx = 0, tapFy = 0, tapFz = 0;
static float tapPx = 0, tapPy = 0, tapPz = 0;
static float tapGapMin = 1.0e9f;
static bool tapFiltInit = false;
static bool tapHot = false;
static bool tapGap = false;
static uint8_t tapCount = 0;

static float tapWindowMs(float v, float fallback) {
  if (v <= 0) return fallback;
  if (v < 5.0f) return v * 1000.0f;
  return v;
}

static void pollDoubleTap(float ax, float ay, float az, float gmag, float dt) {
  if (!S.rstondblttap) {
    tapCount = 0;
    tapHot = false;
    tapGap = false;
    return;
  }
  uint32_t now = millis();
  if ((int32_t)(tapIgnoreUntil - now) > 0) return;
  if (dt < 0.001f) dt = 0.001f;
  if (!tapFiltInit) {
    tapFx = tapPx = ax;
    tapFy = tapPy = ay;
    tapFz = tapPz = az;
    tapFiltInit = true;
    return;
  }
  float a = dt / (dt + 0.008f);
  tapFx += (ax - tapFx) * a;
  tapFy += (ay - tapFy) * a;
  tapFz += (az - tapFz) * a;
  const float G = 9.80665f;
  float jx = (tapFx - tapPx) * G / dt;
  float jy = (tapFy - tapPy) * G / dt;
  float jz = (tapFz - tapPz) * G / dt;
  tapPx = tapFx;
  tapPy = tapFy;
  tapPz = tapFz;
  float jerk = sqrtf(jx * jx + jy * jy + jz * jz);

  float th = S.rstondbltapthres;
  if (th < 20.0f) th = 80.0f;
  float minMs = tapWindowMs(S.rstondbltapmin, 100);
  float maxMs = tapWindowMs(S.rstondbltapmax, 400);
  if (maxMs < minMs + 30.0f) maxMs = minMs + 300.0f;

  if (tapCount == 1 && tapGap && (now - tapLast) > 30) {
    if (gmag < tapGapMin) tapGapMin = gmag;
  }

  bool hot = jerk > th;
  if (hot && !tapHot) {
    bool second = tapCount == 1 && tapGap && tapGapMin < 80.0f && (now - tapLast) >= (uint32_t)minMs &&
                  (now - tapLast) <= (uint32_t)maxMs;
    tapHot = true;
    if (second) {
      tapCount = 0;
      tapGap = false;
      imuResetCenter();
    } else {
      tapCount = 1;
      tapLast = now;
      tapGap = false;
      tapGapMin = 1.0e9f;
    }
  } else if (!hot && tapHot) {
    tapHot = false;
    if (tapCount == 1) tapGap = true;
  }
  if (tapCount && (now - tapLast) > (uint32_t)maxMs) {
    tapCount = 0;
    tapGap = false;
  }
}

void imuResetCenter() {
  pose.tiltOff = pose.tilt;
  pose.rollOff = pose.roll;
  pose.panOff = pose.pan;
  snapOut = true;
  detent = true;
  imuCenterAt = millis();
  tapCount = 0;
  tapHot = false;
  tapGap = false;
  tapIgnoreUntil = millis() + 700;
}

void imuOnSettingsChanged() {
  if (S.rotx != lastRotX || S.roty != lastRotY || S.rotz != lastRotZ) {
    lastRotX = S.rotx;
    lastRotY = S.roty;
    lastRotZ = S.rotz;
    q0 = 1;
    q1 = q2 = q3 = 0;
    seedQuat = true;
    yawInit = false;
    meanInit = false;
    stillTime = 0;
    panCont = 0;
    centeredOnce = false;
    pose.tiltOff = pose.rollOff = pose.panOff = 0;
    snapOut = true;
  }
}

void imuBegin() {
  i2cRelease();
  crsfUpdate();
  pose.imuOk = false;
  if (!lineIsHigh(PIN_WIRE_SCL) || !lineIsHigh(PIN_WIRE_SDA)) {
    protocolLog(lineIsHigh(PIN_WIRE_SCL) ? "e SDA held low" : "e SCL held low");
    return;
  }

  uint8_t id = 0;
  bool found = false;
  for (int attempt = 0; attempt < 3 && !found; attempt++) {
    crsfUpdate();
    if (mpuRead(0x68, 0x75, &id, 1) && id != 0x00 && id != 0xFF) {
      mpuAddr = 0x68;
      found = true;
    } else if (mpuRead(0x69, 0x75, &id, 1) && id != 0x00 && id != 0xFF) {
      mpuAddr = 0x69;
      found = true;
    }
  }
  if (!found) {
    protocolLog("e MPU6050 no ack on 002/029");
    return;
  }

  mpuWake();
  pose.imuOk = true;
  lastUs = micros();
  char msg[] = "i MPU id 00";
  static const char hex[] = "0123456789ABCDEF";
  msg[9] = hex[id >> 4];
  msg[10] = hex[id & 0x0F];
  protocolLog(msg);
}

void imuUpdate() {
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = 0.005f;

  for (int i = 0; i < 16; i++) pose.channelUs[i] = PPM_CENTER;

  if (pose.imuOk) {
    uint8_t raw[14];
    if (mpuRead(mpuAddr, 0x3B, raw, 14)) {
      int16_t axr = (raw[0] << 8) | raw[1];
      int16_t ayr = (raw[2] << 8) | raw[3];
      int16_t azr = (raw[4] << 8) | raw[5];
      int16_t gxr = (raw[8] << 8) | raw[9];
      int16_t gyr = (raw[10] << 8) | raw[11];
      int16_t gzr = (raw[12] << 8) | raw[13];

      float ax = axr / accDiv;
      float ay = ayr / accDiv;
      float az = azr / accDiv;
      float amag = sqrtf(ax * ax + ay * ay + az * az);
      if (!accScaled && amag > 0.2f) {
        if (amag > 0.85f && amag < 1.25f) accScaled = true;
        else if (amag < 0.68f && ++accLow > 40) {
          accDiv *= 2.0f;
          accScaled = true;
        } else if (amag > 1.55f && amag < 2.3f && ++accHigh > 40) {
          accDiv *= 0.5f;
          accScaled = true;
        }
      }
      // A sleeping or glitched MPU ACKs and returns zeros. Hold the last attitude.
      // Clearing calibration here used to throw the center away mid-drive.
      if (amag < 0.25f) {
        badSamples++;
        if (badSamples == 30 || (badSamples > 30 && (badSamples % 200) == 0)) mpuConfigure();
      } else {
        badSamples = 0;
        bool sat = abs(gxr) > 32000 || abs(gyr) > 32000 || abs(gzr) > 32000;
        float gx = gxr / gyrDiv;
        float gy = gyr / gyrDiv;
        float gz = gzr / gyrDiv;
        rotate(ax, ay, az);
        rotate(gx, gy, gz);

        bool gravity = amag > 0.70f && amag < 1.40f;
        if (!meanInit) {
          slowX = fastX = gx;
          slowY = fastY = gy;
          slowZ = fastZ = gz;
          meanInit = true;
        }
        float kf = dt / (dt + 0.12f);
        fastX += (gx - fastX) * kf;
        fastY += (gy - fastY) * kf;
        fastZ += (gz - fastZ) * kf;
        float sx = fastX - slowX, sy = fastY - slowY, sz = fastZ - slowZ;
        float spread = sqrtf(sx * sx + sy * sy + sz * sz);
        float nx = gx - fastX, ny = gy - fastY, nz = gz - fastZ;
        float noise = sqrtf(nx * nx + ny * ny + nz * nz);
        // Always follow, so a look cannot leave the mean stuck and the
        // stillness test dead for the rest of the drive.
        float ks = dt / (dt + 0.35f);
        slowX += (fastX - slowX) * ks;
        slowY += (fastY - slowY) * ks;
        slowZ += (fastZ - slowZ) * ks;
        bool flat = gravity && spread < 0.8f;
        if (flat) stillTime += dt;
        else stillTime = 0;
        bool calm = flat && noise < 8.0f;
        if (!pose.gyroCal && calm && stillTime > 0.7f) {
          gxBias = slowX;
          gyBias = slowY;
          gzBias = slowZ;
          pose.gyroCal = true;
          pose.calibrated = true;
          lockBias = true;
        }

        float gxc = gx - gxBias;
        float gyc = gy - gyBias;
        float gzc = gz - gzBias;
        // Hold pan only when the gyro itself is still. A wider gate used to
        // swallow the start and the slow return of each look, so the same
        // side accumulated after a few movements.
        float rx = fastX - gxBias, ry = fastY - gyBias, rz = fastZ - gzBias;
        float residual = sqrtf(rx * rx + ry * ry + rz * rz);
        float inst = sqrtf(gxc * gxc + gyc * gyc + gzc * gzc);
        bool resting = flat && residual < 0.45f && inst < 0.8f;
        if (pose.gyroCal && resting) {
          biasStill += dt;
          if (biasStill > 0.4f) {
            float a = dt / (dt + 2.0f);
            gxBias += (slowX - gxBias) * a;
            gyBias += (slowY - gyBias) * a;
            gzBias += (slowZ - gzBias) * a;
            gxc = gx - gxBias;
            gyc = gy - gyBias;
            gzc = gz - gzBias;
          }
        } else {
          biasStill = 0;
        }

        pose.accx = ax;
        pose.accy = ay;
        pose.accz = az;
        pose.gyrox = gxc;
        pose.gyroy = gyc;
        pose.gyroz = gzc;
        pollDoubleTap(ax, ay, az, inst, dt);

        // Freeze only when the whole head is quiet. Chopping each axis on its
        // own left a few degrees behind after a look, so the nose did not return.
        float igx = gxc, igy = gyc, igz = gzc;
        if (pose.gyroCal && resting) igx = igy = igz = 0;
        if (sat) igx = igy = igz = 0;
        bool holdOutput = false;
        if (seedQuat || !quatOk()) {
          bool recover = !seedQuat && centeredOnce;
          seedFromAccel(ax, ay, az);
          seedQuat = false;
          yawInit = false;
          if (recover) holdOutput = true;
        }
        madgwick(igx * (float)DEG_TO_RAD, igy * (float)DEG_TO_RAD, igz * (float)DEG_TO_RAD, dt);
        if (!quatOk()) {
          seedFromAccel(ax, ay, az);
          yawInit = false;
          if (centeredOnce) holdOutput = true;
        }

        float tilt, roll, yaw;
        eulerEarth(&tilt, &roll, &yaw);
        if (amag > 0.85f && amag < 1.15f) {
          float pitchA = atan2f(-ax, sqrtf(ay * ay + az * az)) * (float)RAD_TO_DEG;
          float rollA = atan2f(ay, az) * (float)RAD_TO_DEG;
          float tau = inst > 20.0f ? 2.0f : 0.30f;
          float k = dt / (dt + tau);
          tilt += wrap180(pitchA - tilt) * k;
          roll += wrap180(rollA - roll) * k;
          setAttitude(roll, tilt, yaw);
        }
        if (!yawInit) {
          panCont = yaw;
          lastYaw = yaw;
          yawInit = true;
        } else {
          panCont += wrap180(yaw - lastYaw);
          lastYaw = yaw;
        }
        pose.tilt = tilt;
        pose.roll = roll;
        clampPanToServo();
        pose.pan = panCont;
        {
          static bool wasResting = true;
          static float moveShown0 = 0;
          float shown = pose.pan - pose.panOff;
          if (pose.gyroCal && resting) {
            if (!wasResting && fabsf(shown - moveShown0) > 2.0f) {
              char buf[48];
              snprintf(buf, sizeof(buf), "i look %+.1f bz %+.2f", shown - moveShown0, gzBias);
              protocolLog(buf);
            }
            wasResting = true;
          } else if (wasResting) {
            moveShown0 = shown;
            wasResting = false;
          }
        }
        if (!centeredOnce) {
          imuResetCenter();
          centeredOnce = true;
          lockBias = false;
        } else if (holdOutput) {
          pose.tiltOff = pose.tilt - fTilt;
          pose.rollOff = pose.roll - fRoll;
          pose.panOff = pose.pan - fPan;
          snapOut = true;
        } else if (lockBias) {
          lockBias = false;
          float moved = fabsf(wrap180(pose.tilt - pose.tiltOff));
          float mr = fabsf(wrap180(pose.roll - pose.rollOff));
          float mp = fabsf(pose.pan - pose.panOff);
          if (mr > moved) moved = mr;
          if (mp > moved) moved = mp;
          // Only tidy the nose if the head is still where it was at power-on.
          // A turn of the car must not capture a new front.
          if (moved < 6.0f) imuResetCenter();
        }
      }
    }
  }

  if (outputsEnabled && pose.imuOk && yawInit && badSamples < 30) {
    if (badSamples < 30) {
      float dT = wrap180(pose.tilt - pose.tiltOff);
      float dR = wrap180(pose.roll - pose.rollOff);
      float dP = pose.pan - pose.panOff;
      if (dT != dT || dR != dR || dP != dP) dT = dR = dP = 0;
      float span = fabsf(dT);
      if (fabsf(dR) > span) span = fabsf(dR);
      if (fabsf(dP) > span) span = fabsf(dP);
      if (detent) {
        if (span > 1.6f) detent = false;
        else dT = dR = dP = 0;
      } else if (span < 0.55f) {
        detent = true;
        dT = dR = dP = 0;
      }
      if (snapOut) {
        fTilt = dT;
        fRoll = dR;
        fPan = dP;
        snapOut = false;
      } else {
        fTilt = follow(fTilt, dT, dt);
        fRoll = follow(fRoll, dR, dt);
        fPan = follow(fPan, dP, dt);
      }
    }
    pose.tiltUs = axisUs(fTilt, 0, axisReversed(REV_TILT), S.tlt_gain, S.tlt_cnt, S.tlt_min, S.tlt_max);
    pose.rollUs = axisUs(fRoll, 0, axisReversed(REV_ROLL), S.rll_gain, S.rll_cnt, S.rll_min, S.rll_max);
    pose.panUs = axisUs(fPan, 0, axisReversed(REV_PAN), S.pan_gain, S.pan_cnt, S.pan_min, S.pan_max);
  } else {
    pose.tiltUs = S.tlt_cnt;
    pose.rollUs = S.rll_cnt;
    pose.panUs = S.pan_cnt;
    if (!yawInit) fTilt = fRoll = fPan = 0;
  }

  assignChannel(S.tltch, pose.tiltUs);
  assignChannel(S.rllch, pose.rollUs);
  assignChannel(S.panch, pose.panUs);
  if (S.ch5arm) pose.channelUs[4] = 2000;
  updateWave(millis());
}
