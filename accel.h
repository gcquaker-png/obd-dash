#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// Автозамер разгона. Ловит старт с 0 км/ч, засекает время до
// контрольных скоростей. Лучшие результаты хранятся в NVS.
// ============================================================

// контрольные точки, км/ч
static const int ACCEL_MARKS[] = { 60, 100, 150 };
#define ACCEL_NMARKS 3

struct AccelBest {
  float t[ACCEL_NMARKS];   // лучшее время до каждой отметки, сек (0 = нет)
  uint8_t magic;
};
static AccelBest accelBest = { {0,0,0}, 0x3C };

// состояние текущего замера
static bool  accRunning = false;
static uint32_t accStartMs = 0;
static float accLastT[ACCEL_NMARKS] = {0,0,0};   // время в текущем/последнем заезде
static int   accReached = 0;                      // сколько отметок взято в этом заезде
static uint32_t accShowUntil = 0;                 // до какого времени показывать результат

inline void accelLoad(Preferences& p) {
  p.begin("obd", true);
  if (p.getBytesLength("accelbest") == sizeof(AccelBest))
    p.getBytes("accelbest", &accelBest, sizeof(AccelBest));
  p.end();
  if (accelBest.magic != 0x3C) { for (int i=0;i<ACCEL_NMARKS;i++) accelBest.t[i]=0; accelBest.magic=0x3C; }
}
inline void accelSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("accelbest", &accelBest, sizeof(AccelBest));
  p.end();
}
inline void accelReset(Preferences& p) {
  for (int i=0;i<ACCEL_NMARKS;i++) accelBest.t[i]=0;
  p.begin("obd", false); p.remove("accelbest"); p.end();
}

// вызывать с текущей скоростью. Возвращает true, если обновился рекорд
// (стоит сохранить). Логика: скорость упала до 0 -> вооружаемся;
// пошла с 0 -> старт таймера; на каждой отметке фиксируем время.
inline bool accelUpdate(int speed) {
  static int lastSpeed = -1;
  static bool armed = false;
  bool saved = false;
  uint32_t now = millis();

  if (speed < 0) return false;

  if (speed == 0) {
    armed = true;
    if (accRunning) { accRunning = false; }   // остановились — заезд прерван
  }

  // старт: были вооружены на 0, поехали
  if (armed && speed >= 3 && !accRunning) {
    accRunning = true;
    armed = false;
    accStartMs = now;
    accReached = 0;
    for (int i=0;i<ACCEL_NMARKS;i++) accLastT[i] = 0;
  }

  if (accRunning) {
    float el = (now - accStartMs) / 1000.0f;
    // прошли очередную отметку?
    while (accReached < ACCEL_NMARKS && speed >= ACCEL_MARKS[accReached]) {
      accLastT[accReached] = el;
      if (accelBest.t[accReached] == 0 || el < accelBest.t[accReached]) {
        accelBest.t[accReached] = el;
        saved = true;
      }
      accReached++;
      accShowUntil = now + 8000;   // показать результат 8 сек
    }
    // взяли все отметки или разгон затянулся (>40 c) — заезд завершён
    if (accReached >= ACCEL_NMARKS || el > 40.0f) accRunning = false;
  }

  lastSpeed = speed;
  return saved;
}
