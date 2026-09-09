#pragma once
#include <Arduino.h>
#include <Preferences.h>

// ============================================================
// Автокалибровка передачи по соотношению RPM/скорость.
// На устоявшемся ходу ПОД ТЯГОЙ собираем ratio = rpm/speed,
// кластеризуем в таблицу (1..N), в реальном времени определяем.
// Когда набрано gearTarget передач — калибровка ЗАМОРАЖИВАЕТСЯ.
// Таблица + число передач в NVS ("obd"). Сброс — через веб-морду.
// ============================================================

#define GEAR_MAX        7      // физический предел массива
#define GEAR_MIN_SPEED  20     // ниже — не калибруем и не показываем
#define GEAR_MIN_RPM    1100   // ниже — считаем нейтраль/накат (не ищем передачу)
#define GEAR_STABLE_MS  1600   // держать стабильные rpm/speed столько
#define GEAR_RPM_TOL    100    // допуск дрожания rpm за окно
#define GEAR_SPD_TOL    2      // допуск дрожания скорости, км/ч
#define GEAR_THR_MIN    8      // дроссель НИЖЕ — накат, НЕ калибруем (ключевой фикс)
#define GEAR_THR_MAX    70     // дроссель выше — разгон/переход, НЕ калибруем
#define GEAR_MATCH_TOL  0.10f  // относит. допуск при определении (люфты трансмиссии)
#define GEAR_MERGE_TOL  0.11f  // ближе этого — та же передача (кластеризация, шире из-за люфтов)

struct GearTable {
  float   ratio[GEAR_MAX];
  uint8_t count;
  uint8_t target;             // сколько передач всего у машины (по умолчанию 5)
};

static GearTable gears = { {0}, 0, 5 };
static int  gearCurrent = 0;          // 0 = не определена / нейтраль
static bool gearCalibrating = false;  // идёт набор стабильного окна
static bool gearLocked = false;       // калибровка завершена (набрано target)

// ---- NVS ----
inline void gearRecalcLock() { gearLocked = (gears.count >= gears.target); }

inline void gearLoad(Preferences& p) {
  p.begin("obd", true);
  size_t n = p.getBytesLength("gears");
  if (n == sizeof(GearTable)) p.getBytes("gears", &gears, sizeof(GearTable));
  else { gears.count = 0; gears.target = 5; }
  p.end();
  if (gears.count > GEAR_MAX) gears.count = 0;
  if (gears.target < 3 || gears.target > GEAR_MAX) gears.target = 5;
  gearRecalcLock();
}
inline void gearSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("gears", &gears, sizeof(GearTable));
  p.end();
}
inline void gearReset(Preferences& p) {
  uint8_t keepTarget = gears.target;
  gears.count = 0;
  gears.target = keepTarget;
  gearCurrent = 0;
  gearLocked = false;
  p.begin("obd", false);
  p.putBytes("gears", &gears, sizeof(GearTable));
  p.end();
}
inline void gearSetTarget(Preferences& p, int t) {
  if (t < 3) t = 3;
  if (t > GEAR_MAX) t = GEAR_MAX;
  gears.target = t;
  gearRecalcLock();
  gearSave(p);
}

// ---- вставить новый ratio (с кластеризацией) ----
inline void gearLearn(float r) {
  if (r <= 0) return;
  for (int i = 0; i < gears.count; i++) {
    if (fabsf(r - gears.ratio[i]) / gears.ratio[i] < GEAR_MERGE_TOL) {
      gears.ratio[i] = gears.ratio[i] * 0.75f + r * 0.25f;   // усреднить
      return;
    }
  }
  if (gears.count >= gears.target || gears.count >= GEAR_MAX) return;  // лимит
  gears.ratio[gears.count++] = r;
  // сортировка по убыванию (1-я передача — самый большой ratio)
  for (int i = 0; i < gears.count; i++)
    for (int j = i + 1; j < gears.count; j++)
      if (gears.ratio[j] > gears.ratio[i]) {
        float t = gears.ratio[i]; gears.ratio[i] = gears.ratio[j]; gears.ratio[j] = t;
      }
  gearRecalcLock();
}

// ---- вызывать периодически ----
// возвращает true, если таблица изменилась (сохранить в NVS)
inline bool gearUpdate(int rpm, int speed, int throttle) {
  static uint32_t winStart = 0;
  static int rMin, rMax, sMin, sMax;

  gearCurrent = 0;
  gearCalibrating = false;

  // нейтраль / накат / стоп — передачу не показываем
  if (speed < GEAR_MIN_SPEED || rpm < GEAR_MIN_RPM) { winStart = 0; return false; }

  float r = (float)rpm / speed;

  // определить текущую передачу по таблице
  if (gears.count > 0) {
    int best = -1; float bestd = 1e9;
    for (int i = 0; i < gears.count; i++) {
      float d = fabsf(r - gears.ratio[i]) / gears.ratio[i];
      if (d < bestd) { bestd = d; best = i; }
    }
    if (best >= 0 && bestd < GEAR_MATCH_TOL) gearCurrent = best + 1;
  }

  // калибровка заморожена — дальше только определение
  if (gearLocked) { winStart = 0; return false; }

  // калибруем ТОЛЬКО под умеренной тягой (не накат, не полный газ)
  bool goodThrottle = (throttle >= GEAR_THR_MIN && throttle <= GEAR_THR_MAX);
  if (!goodThrottle) { winStart = 0; return false; }

  uint32_t now = millis();
  if (winStart == 0) {
    winStart = now; rMin = rMax = rpm; sMin = sMax = speed;
    gearCalibrating = true;
    return false;
  }
  gearCalibrating = true;
  if (rpm < rMin) rMin = rpm;  if (rpm > rMax) rMax = rpm;
  if (speed < sMin) sMin = speed;  if (speed > sMax) sMax = speed;

  if (rMax - rMin > GEAR_RPM_TOL || sMax - sMin > GEAR_SPD_TOL) {
    winStart = now; rMin = rMax = rpm; sMin = sMax = speed;   // дёрнулось — заново
    return false;
  }

  if (now - winStart >= GEAR_STABLE_MS) {
    float rr = (float)((rMin + rMax) / 2) / ((sMin + sMax) / 2.0f);
    int before = gears.count;
    gearLearn(rr);
    winStart = 0;
    gearCalibrating = false;
    return true;                 // сохранить (новая передача или усреднение)
  }
  return false;
}
