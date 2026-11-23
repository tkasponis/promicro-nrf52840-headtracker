#include "settings.h"
#include <InternalFileSystem.h>

using Adafruit_LittleFS_Namespace::File;
using Adafruit_LittleFS_Namespace::FILE_O_READ;
using Adafruit_LittleFS_Namespace::FILE_O_WRITE;

Settings S;

static const char *kPath = "/ht.json";
static bool fsOk = false;

static int asInt(JsonVariantConst v, int fallback) {
  if (v.is<int>() || v.is<float>() || v.is<bool>()) return v.as<int>();
  if (v.is<const char *>()) return atoi(v.as<const char *>());
  return fallback;
}

static float asFloat(JsonVariantConst v, float fallback) {
  if (v.is<float>() || v.is<int>()) return v.as<float>();
  if (v.is<const char *>()) return atof(v.as<const char *>());
  return fallback;
}

static bool asBool(JsonVariantConst v, bool fallback) {
  if (v.is<bool>()) return v.as<bool>();
  if (v.is<int>()) return v.as<int>() != 0;
  if (v.is<const char *>()) {
    const char *s = v.as<const char *>();
    if (!strcasecmp(s, "true") || !strcmp(s, "1")) return true;
    if (!strcasecmp(s, "false") || !strcmp(s, "0")) return false;
  }
  return fallback;
}

bool settingsBegin() {
  fsOk = InternalFS.begin();
  if (!fsOk) return false;
  File f = InternalFS.open(kPath, FILE_O_READ);
  if (f) {
    DynamicJsonDocument doc(3072);
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    if (!err) settingsFromJson(doc.as<JsonObjectConst>());
  }
  if (!S.dbltapmig) {
    S.rstondblttap = true;
    S.dbltapmig = true;
    settingsSave();
  }
  return true;
}

bool settingsSave() {
  if (!fsOk && !InternalFS.begin()) return false;
  fsOk = true;
  InternalFS.remove(kPath);
  File f = InternalFS.open(kPath, FILE_O_WRITE);
  if (!f) return false;
  DynamicJsonDocument doc(3072);
  settingsToJson(doc.to<JsonObject>());
  serializeJson(doc, f);
  f.close();
  return true;
}

void settingsErase() {
  S = Settings();
  if (fsOk) InternalFS.remove(kPath);
  settingsSave();
}

bool axisReversed(uint8_t bit) { return (S.servoreverse & bit) != 0; }

void settingsToJson(JsonObject obj) {
  obj["rll_min"] = S.rll_min;
  obj["rll_max"] = S.rll_max;
  obj["rll_cnt"] = S.rll_cnt;
  obj["rll_gain"] = S.rll_gain;
  obj["tlt_min"] = S.tlt_min;
  obj["tlt_max"] = S.tlt_max;
  obj["tlt_cnt"] = S.tlt_cnt;
  obj["tlt_gain"] = S.tlt_gain;
  obj["pan_min"] = S.pan_min;
  obj["pan_max"] = S.pan_max;
  obj["pan_cnt"] = S.pan_cnt;
  obj["pan_gain"] = S.pan_gain;
  obj["tltch"] = S.tltch;
  obj["rllch"] = S.rllch;
  obj["panch"] = S.panch;
  obj["alertch"] = S.alertch;
  obj["pwm0"] = S.pwm0;
  obj["pwm1"] = S.pwm1;
  obj["pwm2"] = S.pwm2;
  obj["pwm3"] = S.pwm3;
  obj["an0ch"] = S.an0ch;
  obj["an1ch"] = S.an1ch;
  obj["an2ch"] = S.an2ch;
  obj["an3ch"] = S.an3ch;
  obj["aux0ch"] = S.aux0ch;
  obj["aux1ch"] = S.aux1ch;
  obj["aux2ch"] = S.aux2ch;
  obj["rstppm"] = S.rstppm;
  obj["aux0func"] = S.aux0func;
  obj["aux1func"] = S.aux1func;
  obj["aux2func"] = S.aux2func;
  obj["an0gain"] = S.an0gain;
  obj["an1gain"] = S.an1gain;
  obj["an2gain"] = S.an2gain;
  obj["an3gain"] = S.an3gain;
  obj["an0off"] = S.an0off;
  obj["an1off"] = S.an1off;
  obj["an2off"] = S.an2off;
  obj["an3off"] = S.an3off;
  obj["servoreverse"] = S.servoreverse;
  obj["magxoff"] = S.magxoff;
  obj["magyoff"] = S.magyoff;
  obj["magzoff"] = S.magzoff;
  obj["accxoff"] = S.accxoff;
  obj["accyoff"] = S.accyoff;
  obj["acczoff"] = S.acczoff;
  obj["gyrxoff"] = S.gyrxoff;
  obj["gyryoff"] = S.gyryoff;
  obj["gyrzoff"] = S.gyrzoff;
  obj["so00"] = S.so00;
  obj["so01"] = S.so01;
  obj["so02"] = S.so02;
  obj["so10"] = S.so10;
  obj["so11"] = S.so11;
  obj["so12"] = S.so12;
  obj["so20"] = S.so20;
  obj["so21"] = S.so21;
  obj["so22"] = S.so22;
  obj["dismag"] = S.dismag;
  obj["rotx"] = S.rotx;
  obj["roty"] = S.roty;
  obj["rotz"] = S.rotz;
  obj["uartmode"] = S.uartmode;
  obj["crsftxrate"] = S.crsftxrate;
  obj["sbustxrate"] = S.sbustxrate;
  obj["sbininv"] = S.sbininv;
  obj["sboutinv"] = S.sboutinv;
  obj["crsftxinv"] = S.crsftxinv;
  obj["ch5arm"] = S.ch5arm;
  obj["btmode"] = S.btmode;
  obj["rstonwave"] = S.rstonwave;
  obj["butlngps"] = S.butlngps;
  obj["rstontlt"] = S.rstontlt;
  obj["rstondblttap"] = S.rstondblttap;
  obj["rstondbltapthres"] = S.rstondbltapthres;
  obj["rstondbltapmin"] = S.rstondbltapmin;
  obj["rstondbltapmax"] = S.rstondbltapmax;
  obj["dbltapmig"] = S.dbltapmig;
  obj["ppmoutinvert"] = S.ppmoutinvert;
  obj["ppmininvert"] = S.ppmininvert;
  obj["ppmframe"] = S.ppmframe;
  obj["ppmsync"] = S.ppmsync;
  obj["ppmchcnt"] = S.ppmchcnt;
  obj["btpairedaddress"] = S.btpairedaddress;
}

#define SET_I(name) if (!strcmp(k, #name)) S.name = asInt(kv.value(), S.name)
#define SET_F(name) if (!strcmp(k, #name)) S.name = asFloat(kv.value(), S.name)
#define SET_B(name) if (!strcmp(k, #name)) S.name = asBool(kv.value(), S.name)

void settingsFromJson(JsonObjectConst obj) {
  for (JsonPairConst kv : obj) {
    const char *k = kv.key().c_str();
    if (!strcmp(k, "Cmd")) continue;
    SET_I(rll_min); SET_I(rll_max); SET_I(rll_cnt); SET_F(rll_gain);
    SET_I(tlt_min); SET_I(tlt_max); SET_I(tlt_cnt); SET_F(tlt_gain);
    SET_I(pan_min); SET_I(pan_max); SET_I(pan_cnt); SET_F(pan_gain);
    SET_I(tltch); SET_I(rllch); SET_I(panch); SET_I(alertch);
    SET_I(pwm0); SET_I(pwm1); SET_I(pwm2); SET_I(pwm3);
    SET_I(an0ch); SET_I(an1ch); SET_I(an2ch); SET_I(an3ch);
    SET_I(aux0ch); SET_I(aux1ch); SET_I(aux2ch); SET_I(rstppm);
    SET_I(aux0func); SET_I(aux1func); SET_I(aux2func);
    SET_F(an0gain); SET_F(an1gain); SET_F(an2gain); SET_F(an3gain);
    SET_F(an0off); SET_F(an1off); SET_F(an2off); SET_F(an3off);
    if (!strcmp(k, "servoreverse")) S.servoreverse = (uint8_t)asInt(kv.value(), S.servoreverse);
    SET_F(magxoff); SET_F(magyoff); SET_F(magzoff);
    SET_F(accxoff); SET_F(accyoff); SET_F(acczoff);
    SET_F(gyrxoff); SET_F(gyryoff); SET_F(gyrzoff);
    SET_F(so00); SET_F(so01); SET_F(so02);
    SET_F(so10); SET_F(so11); SET_F(so12);
    SET_F(so20); SET_F(so21); SET_F(so22);
    SET_B(dismag);
    SET_F(rotx); SET_F(roty); SET_F(rotz);
    SET_I(uartmode); SET_I(crsftxrate); SET_I(sbustxrate);
    SET_B(sbininv); SET_B(sboutinv); SET_B(crsftxinv); SET_B(ch5arm);
    SET_I(btmode);
    SET_B(rstonwave); SET_B(butlngps); SET_B(rstontlt); SET_B(rstondblttap);
    SET_F(rstondbltapthres); SET_F(rstondbltapmin); SET_F(rstondbltapmax);
    SET_B(dbltapmig);
    SET_B(ppmoutinvert); SET_B(ppmininvert);
    SET_I(ppmframe); SET_I(ppmsync); SET_I(ppmchcnt);
    if (!strcmp(k, "btpairedaddress")) S.btpairedaddress = kv.value().as<const char *>();
  }
  if (S.crsftxrate < 10) S.crsftxrate = 10;
  if (S.crsftxrate > 500) S.crsftxrate = 500;
  if (S.tlt_gain < 0.01f) S.tlt_gain = 0.01f;
  if (S.rll_gain < 0.01f) S.rll_gain = 0.01f;
  if (S.pan_gain < 0.01f) S.pan_gain = 0.01f;
}
