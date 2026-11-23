#include "protocol.h"
#include "imu.h"
#include "settings.h"
#include <ArduinoJson.h>

static char jsonBuf[2800];
static uint16_t jsonLen = 0;
static bool inFrame = false;
static bool want[32];
static const char *kKeys[] = {
    "tilt", "roll", "pan", "tiltoff", "rolloff", "panoff", "tiltout", "rollout", "panout",
    "gyrocal", "iscal", "trpenabled", "accx", "accy", "accz", "gyrox", "gyroy", "gyroz",
    "btcon", "chout"};
static const int kKeyCount = 20;
static uint32_t lastData = 0;

static uint16_t crc16(const char *data, uint16_t length) {
  uint16_t crc = 0xFFFF;
  if (length == 0) return (uint16_t)~crc;
  for (uint16_t n = 0; n < length; n++) {
    uint16_t byte = (uint8_t)data[n];
    for (uint8_t i = 0; i < 8; i++) {
      if ((crc & 1) ^ (byte & 1)) crc = (crc >> 1) ^ 0x8408;
      else crc >>= 1;
      byte >>= 1;
    }
  }
  return (uint16_t)~crc;
}

static uint16_t escapeCrc(uint16_t crc) {
  uint8_t lo = crc & 0xFF;
  uint8_t hi = (crc >> 8) & 0xFF;
  if (lo == 0x00 || lo == 0x02 || lo == 0x03 || lo == 0x06 || lo == 0x15) lo ^= 0xFF;
  if (hi == 0x00 || hi == 0x02 || hi == 0x03 || hi == 0x06 || hi == 0x15) hi ^= 0xFF;
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

static void sendFrame(const char *json) {
  uint16_t n = (uint16_t)strlen(json);
  uint16_t crc = escapeCrc(crc16(json, n));
  Serial.write((uint8_t)0x02);
  Serial.write((const uint8_t *)json, n);
  Serial.write((uint8_t)(crc & 0xFF));
  Serial.write((uint8_t)(crc >> 8));
  Serial.write((uint8_t)0x03);
  Serial.print("\r\n");
}

static void ack() {
  Serial.write((uint8_t)0x06);
  Serial.print("\r\n");
}
static void nak() {
  Serial.write((uint8_t)0x15);
  Serial.print("\r\n");
}

void protocolLog(const char *text) {
  Serial.write((uint8_t)0x01);
  Serial.print(text);
  Serial.write((uint8_t)0x03);
  Serial.print("\r\n");
}

static int keyIndex(const char *name) {
  for (int i = 0; i < kKeyCount; i++) {
    if (!strcmp(kKeys[i], name)) return i;
  }
  return -1;
}

static void b64(const uint8_t *in, int len, char *out) {
  static const char *tbl = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int o = 0;
  for (int i = 0; i < len; i += 3) {
    uint32_t v = in[i] << 16;
    if (i + 1 < len) v |= in[i + 1] << 8;
    if (i + 2 < len) v |= in[i + 2];
    out[o++] = tbl[(v >> 18) & 63];
    out[o++] = tbl[(v >> 12) & 63];
    out[o++] = (i + 1 < len) ? tbl[(v >> 6) & 63] : '=';
    out[o++] = (i + 2 < len) ? tbl[v & 63] : '=';
  }
  out[o] = 0;
}

static void sendData() {
  bool any = false;
  for (int i = 0; i < kKeyCount; i++) any = any || want[i];
  if (!any) return;
  DynamicJsonDocument doc(1024);
  JsonObject obj = doc.to<JsonObject>();
  obj["Cmd"] = "Data";
  if (want[keyIndex("tilt")]) obj["tilt"] = pose.tilt;
  if (want[keyIndex("roll")]) obj["roll"] = pose.roll;
  if (want[keyIndex("pan")]) obj["pan"] = pose.pan;
  if (want[keyIndex("tiltoff")]) obj["tiltoff"] = pose.tilt - pose.tiltOff;
  if (want[keyIndex("rolloff")]) obj["rolloff"] = pose.roll - pose.rollOff;
  if (want[keyIndex("panoff")]) obj["panoff"] = pose.pan - pose.panOff;
  if (want[keyIndex("tiltout")]) obj["tiltout"] = pose.tiltUs;
  if (want[keyIndex("rollout")]) obj["rollout"] = pose.rollUs;
  if (want[keyIndex("panout")]) obj["panout"] = pose.panUs;
  if (want[keyIndex("gyrocal")]) obj["gyrocal"] = pose.gyroCal;
  if (want[keyIndex("iscal")]) obj["iscal"] = pose.calibrated;
  if (want[keyIndex("trpenabled")]) obj["trpenabled"] = outputsEnabled;
  if (want[keyIndex("accx")]) obj["accx"] = pose.accx;
  if (want[keyIndex("accy")]) obj["accy"] = pose.accy;
  if (want[keyIndex("accz")]) obj["accz"] = pose.accz;
  if (want[keyIndex("gyrox")]) obj["gyrox"] = pose.gyrox;
  if (want[keyIndex("gyroy")]) obj["gyroy"] = pose.gyroy;
  if (want[keyIndex("gyroz")]) obj["gyroz"] = pose.gyroz;
  if (want[keyIndex("btcon")]) obj["btcon"] = false;
  if (want[keyIndex("chout")]) {
    uint8_t raw[32];
    for (int i = 0; i < 16; i++) {
      raw[i * 2] = pose.channelUs[i] & 0xFF;
      raw[i * 2 + 1] = pose.channelUs[i] >> 8;
    }
    char enc[64];
    b64(raw, 32, enc);
    obj["6choutu16"] = enc;
  }
  char out[1024];
  serializeJson(doc, out, sizeof(out));
  sendFrame(out);
}

static void handle(char *json) {
  uint16_t n = (uint16_t)strlen(json);
  if (n < 2) {
    nak();
    return;
  }
  uint8_t crcLo = (uint8_t)json[n - 2];
  uint8_t crcHi = (uint8_t)json[n - 1];
  json[n - 2] = 0;
  uint16_t expect = escapeCrc(crc16(json, (uint16_t)strlen(json)));
  if (crcLo != (expect & 0xFF) || crcHi != (expect >> 8)) {
    nak();
    return;
  }
  DynamicJsonDocument doc(3072);
  if (deserializeJson(doc, json)) {
    nak();
    return;
  }
  ack();
  const char *cmd = doc["Cmd"] | "";
  if (!strcmp(cmd, "FW")) {
    sendFrame("{\"Cmd\":\"FW\",\"Vers\":\"2.40\",\"Hard\":\"NANO33BLE\",\"Git\":\"promicro\"}");
  } else if (!strcmp(cmd, "Get")) {
    DynamicJsonDocument reply(3072);
    JsonObject obj = reply.to<JsonObject>();
    obj["Cmd"] = "Set";
    settingsToJson(obj);
    char out[2800];
    serializeJson(reply, out, sizeof(out));
    sendFrame(out);
  } else if (!strcmp(cmd, "FE")) {
    sendFrame("{\"Cmd\":\"FE\",\"FEAT\":[\"IMU\",\"AUXSERIAL\",\"AUXINVERT\",\"LED\"],\"PINS\":{\"TX\":\"D1 P0.06\",\"RX\":\"D0 P0.08\",\"SDA\":\"002 P0.02\",\"SCL\":\"029 P0.29\"}}");
  } else if (!strcmp(cmd, "Set")) {
    settingsFromJson(doc.as<JsonObjectConst>());
    imuOnSettingsChanged();
  } else if (!strcmp(cmd, "Flash")) {
    bool ok = settingsSave();
    protocolLog(ok ? "i settings saved" : "e settings save failed");
  } else if (!strcmp(cmd, "Erase")) {
    settingsErase();
    imuOnSettingsChanged();
    protocolLog("i settings erased");
  } else if (!strcmp(cmd, "Reboot")) {
    delay(40);
    NVIC_SystemReset();
  } else if (!strcmp(cmd, "RstCnt")) {
    imuResetCenter();
  } else if (!strcmp(cmd, "D--")) {
    memset(want, 0, sizeof(want));
  } else if (!strcmp(cmd, "RD")) {
    for (JsonPair kv : doc.as<JsonObject>()) {
      int idx = keyIndex(kv.key().c_str());
      if (idx >= 0) want[idx] = kv.value().as<bool>();
    }
  } else if (!strcmp(cmd, "IH")) {
    // Keepalive. ACK already sent.
  }
}

void protocolBegin() { Serial.begin(115200); }

void protocolPoll() {
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();
    if (!inFrame) {
      if (b == 0x02) {
        inFrame = true;
        jsonLen = 0;
      }
      continue;
    }
    if (b == 0x03) {
      inFrame = false;
      if (jsonLen + 1 < sizeof(jsonBuf)) {
        jsonBuf[jsonLen] = 0;
        handle(jsonBuf);
      }
      jsonLen = 0;
      continue;
    }
    if (b == 0x02) {
      jsonLen = 0;
      continue;
    }
    if (jsonLen + 1 < sizeof(jsonBuf)) jsonBuf[jsonLen++] = (char)b;
  }
  if (millis() - lastData > 50) {
    lastData = millis();
    sendData();
  }
}
