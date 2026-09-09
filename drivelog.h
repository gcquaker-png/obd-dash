#pragma once
// ============================================================
// DRIVELOG — запись замеров производительности в NVS.
// Смысл: ноут в машину не унести, поэтому плата сама пишет статистику
// в флеш. Дома подключаем по USB и забираем: в Serial по команде 'L'
// (или через веб-морду /log) выводится накопленное.
//
// Пишем НЕ каждый замер (убьём флеш), а агрегат раз в LOG_SLOT_MS:
// за окно считаем частоту опроса, средние/мин/макс, число потерь.
// Слотов LOG_SLOTS — кольцевой буфер, самые старые затираются.
//
// Ресурс флеша: слот пишется раз в 10 с, всего 60 слотов = один putBytes
// раз в 10 с. За часовую поездку ~360 записей. NVS выдерживает ~100k
// циклов стирания на сектор — запас огромный.
// ============================================================
#include <Preferences.h>

#define LOG_SLOTS    60      // 60 слотов x 10 с = 10 минут истории
#define LOG_SLOT_MS  10000   // длительность одного слота, мс

struct LogSlot {
  uint16_t sec;        // время от старта, с
  uint16_t polls;      // сколько успешных проходов опроса за слот
  uint16_t fails;      // сколько неудачных запросов (таймаут/NO DATA)
  uint16_t rttAvg;     // средний отклик адаптера, мс
  uint16_t rttMin;
  uint16_t rttMax;
  uint16_t rpmMax;     // максимум RPM за слот (виден ли отклик на газ)
  uint8_t  spdMax;     // максимум скорости
  uint8_t  flags;      // bit0 = была потеря линка за слот
};

struct DriveLog {
  uint8_t  magic;                 // 0x5A = валидный лог
  uint8_t  count;                 // сколько слотов заполнено (до LOG_SLOTS)
  uint8_t  head;                  // индекс следующего слота (кольцо)
  uint8_t  boots;                 // счётчик запусков платы
  LogSlot  slot[LOG_SLOTS];
};

static DriveLog dlog;

// --- накопители текущего слота ---
static uint32_t dlSlotStart = 0;
static uint32_t dlPolls = 0, dlFails = 0;
static uint32_t dlRttSum = 0;
static uint16_t dlRttMin = 0xFFFF, dlRttMax = 0;
static uint16_t dlRpmMax = 0;
static uint8_t  dlSpdMax = 0;
static uint8_t  dlFlags  = 0;
static bool     dlDirty  = false;

inline void dlogLoad(Preferences& p) {
  p.begin("obd", true);
  size_t n = p.getBytesLength("dlog");
  if (n == sizeof(DriveLog)) p.getBytes("dlog", &dlog, sizeof(DriveLog));
  p.end();
  if (dlog.magic != 0x5A) {          // первый запуск — чистим
    memset(&dlog, 0, sizeof(dlog));
    dlog.magic = 0x5A;
  }
  dlog.boots++;
  dlSlotStart = millis();
}

inline void dlogSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("dlog", &dlog, sizeof(DriveLog));
  p.end();
  dlDirty = false;
}

inline void dlogClear(Preferences& p) {
  memset(&dlog, 0, sizeof(dlog));
  dlog.magic = 0x5A;
  dlogSave(p);
}

// Зафиксировать один успешный опрос: rtt — время ответа адаптера, мс.
inline void dlogPoll(uint16_t rtt, int rpm, int spd) {
  dlPolls++;
  dlRttSum += rtt;
  if (rtt < dlRttMin) dlRttMin = rtt;
  if (rtt > dlRttMax) dlRttMax = rtt;
  if (rpm > 0 && rpm < 9000 && rpm > (int)dlRpmMax) dlRpmMax = rpm;
  if (spd > 0 && spd < 255 && spd > (int)dlSpdMax) dlSpdMax = spd;
}

inline void dlogFail()      { dlFails++; }
inline void dlogLinkLost()  { dlFlags |= 0x01; }

// Вызывать в loop: закрывает слот раз в LOG_SLOT_MS. Возвращает true,
// если слот закрыт (пора сохранить в NVS).
inline bool dlogTick() {
  if (millis() - dlSlotStart < LOG_SLOT_MS) return false;
  uint32_t dur = millis() - dlSlotStart;
  LogSlot& s = dlog.slot[dlog.head];
  s.sec    = (uint16_t)(millis() / 1000);
  s.polls  = (uint16_t)(dlPolls * 1000UL / (dur ? dur : 1) * 10);  // 0.1 Гц
  s.fails  = (uint16_t)dlFails;
  s.rttAvg = dlPolls ? (uint16_t)(dlRttSum / dlPolls) : 0;
  s.rttMin = (dlRttMin == 0xFFFF) ? 0 : dlRttMin;
  s.rttMax = dlRttMax;
  s.rpmMax = dlRpmMax;
  s.spdMax = dlSpdMax;
  s.flags  = dlFlags;

  dlog.head = (dlog.head + 1) % LOG_SLOTS;
  if (dlog.count < LOG_SLOTS) dlog.count++;

  dlSlotStart = millis();
  dlPolls = dlFails = dlRttSum = 0;
  dlRttMin = 0xFFFF; dlRttMax = 0;
  dlRpmMax = 0; dlSpdMax = 0; dlFlags = 0;
  dlDirty = true;
  return true;
}

// Печать всего лога в Serial — забираем дома по USB.
inline void dlogDump() {
  Serial.println();
  Serial.println("===== DRIVE LOG BEGIN =====");
  Serial.printf("boots=%u slots=%u\n", dlog.boots, dlog.count);
  Serial.println("sec\thz\tfails\trtt_avg\trtt_min\trtt_max\trpm_max\tspd_max\tlink_lost");
  int start = (dlog.count < LOG_SLOTS) ? 0 : dlog.head;
  for (int i = 0; i < dlog.count; i++) {
    const LogSlot& s = dlog.slot[(start + i) % LOG_SLOTS];
    Serial.printf("%u\t%.1f\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n",
                  s.sec, s.polls / 10.0f, s.fails,
                  s.rttAvg, s.rttMin, s.rttMax,
                  s.rpmMax, s.spdMax, (s.flags & 1) ? 1 : 0);
  }
  Serial.println("===== DRIVE LOG END =====");
}
