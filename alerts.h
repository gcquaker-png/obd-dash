#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// Аварийные предупреждения — полноэкранное красное окно поверх
// любого экрана. Пороги хранятся в NVS ("obd"), правятся в веб-морде.
// ============================================================

struct AlertCfg {
  int   coolMax   = 105;   // °C — перегрев ОЖ
  float voltMin   = 12.0;  // В — на заведённой (RPM>500) ниже = не заряжает
  int   rpmMax    = 6500;  // об/мин — отсечка
  uint8_t magic   = 0xA5;
};
static AlertCfg alertCfg;

enum AlertKind { AL_NONE, AL_OVERHEAT, AL_LOWVOLT, AL_REDLINE, AL_NEWDTC };
static AlertKind alertActive = AL_NONE;
static uint32_t  alertSince  = 0;

inline void alertLoad(Preferences& p) {
  p.begin("obd", true);
  if (p.getBytesLength("alertcfg") == sizeof(AlertCfg))
    p.getBytes("alertcfg", &alertCfg, sizeof(AlertCfg));
  p.end();
  if (alertCfg.magic != 0xA5) alertCfg = AlertCfg();
}
inline void alertSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("alertcfg", &alertCfg, sizeof(AlertCfg));
  p.end();
}

// Проверить условия. rpm/coolant/voltage — текущие; newDtc — флаг что
// при последнем чтении кодов появился новый. Возвращает вид тревоги.
inline AlertKind alertCheck(int rpm, int coolant, float voltage, bool newDtc) {
  AlertKind k = AL_NONE;
  if (coolant > -200 && coolant >= alertCfg.coolMax)              k = AL_OVERHEAT;
  else if (rpm > 500 && voltage > 0 && voltage < alertCfg.voltMin) k = AL_LOWVOLT;
  else if (rpm >= alertCfg.rpmMax)                                k = AL_REDLINE;
  else if (newDtc)                                                k = AL_NEWDTC;

  uint32_t now = millis();
  if (k != AL_NONE) {
    if (alertActive != k) { alertActive = k; alertSince = now; }
  } else if (alertActive != AL_NONE && now - alertSince > 3000) {
    alertActive = AL_NONE;   // отпустило + 3 c
  }
  return alertActive;
}

inline const char* alertText(AlertKind k) {
  switch (k) {
    case AL_OVERHEAT: return "OVERHEAT";
    case AL_LOWVOLT:  return "NO CHARGE";
    case AL_REDLINE:  return "REDLINE";
    case AL_NEWDTC:   return "DTC FAULT";
    default:          return "";
  }
}
