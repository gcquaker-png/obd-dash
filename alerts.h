#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// Аварийные предупреждения — полноэкранное красное окно поверх
// любого экрана. Пороги хранятся в NVS ("obd"), правятся в веб-морде.
// ============================================================

struct AlertCfg {
  int   coolMax   = 105;   // °C — перегрев ОЖ
  float voltMin   = 11.6;  // В — ниже на прогретой = генератор не заряжает
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
  // Миграция: 12.0 В было слишком высоко — на прогретой машине штатные
  // 11.7-11.9 В давали ложную тревогу «нет заряда» (видно в журнале
  // аномалий: три срабатывания на 11.5-11.8 В). Опускаем до 11.6.
  if (alertCfg.voltMin > 11.9f) alertCfg.voltMin = 11.6f;
}
inline void alertSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("alertcfg", &alertCfg, sizeof(AlertCfg));
  p.end();
}

// Проверить условия. rpm/coolant/voltage — текущие; newDtc — флаг что
// при последнем чтении кодов появился новый. Возвращает вид тревоги.
// При пуске стартер сажает бортсеть до 9-10 В — это норма, а не «нет заряда».
// Обороты за 500 появляются уже во время прокрутки, поэтому одного порога
// по RPM мало: ждём, пока двигатель устойчиво поработает ALERT_VOLT_DELAY_MS,
// и только потом смотрим на напряжение. Счётчик сбрасывается при глушении.
#define ALERT_VOLT_DELAY_MS 8000
#define ALERT_RUN_RPM       600
static uint32_t engRunSince = 0;

inline AlertKind alertCheck(int rpm, int coolant, float voltage, bool newDtc) {
  uint32_t now = millis();

  // отслеживаем, сколько двигатель непрерывно работает
  if (rpm > ALERT_RUN_RPM) { if (engRunSince == 0) engRunSince = now; }
  else                       engRunSince = 0;
  bool engineWarm = engRunSince && (now - engRunSince >= ALERT_VOLT_DELAY_MS);

  AlertKind k = AL_NONE;
  if (coolant > -200 && coolant >= alertCfg.coolMax)                 k = AL_OVERHEAT;
  else if (engineWarm && voltage > 0 && voltage < alertCfg.voltMin)   k = AL_LOWVOLT;
  else if (rpm >= alertCfg.rpmMax)                                   k = AL_REDLINE;
  else if (newDtc)                                                   k = AL_NEWDTC;

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
