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

// 40 слотов x 15 с = 10 минут истории, DriveLog ~910 байт.
// Запись в NVS раз в 15 с: за часовую поездку ~240 записей — для флеша
// с ресурсом ~100k циклов это ничтожно.
#define LOG_SLOTS    40
#define LOG_SLOT_MS  15000   // длительность одного слота, мс

struct LogSlot {
  uint16_t sec;        // время от старта, с
  uint16_t polls;      // сколько успешных проходов опроса за слот
  uint16_t fails;      // неудачные запросы ВСЕГО
  uint16_t touts;      // из них таймаутов (ответа не было совсем)
  uint16_t pend;       // из них «ЭБУ занят» (7F..78) даже после повтора
  uint16_t rttAvg;     // средний отклик адаптера, мс
  uint16_t rttMin;
  uint16_t rttMax;
  uint16_t rpmMax;     // максимум RPM за слот (виден ли отклик на газ)
  uint8_t  spdMax;     // максимум скорости
  uint8_t  flags;      // bit0 = потеря линка, bit1 = своя AP была поднята
  uint8_t  apClients;  // сколько клиентов висело на нашей точке доступа
  int8_t   rssi;       // уровень сигнала сети адаптера, dBm
};

// ---- виды аномалий для журнала ----
enum {
  EV_OVERHEAT = 1, EV_LOWVOLT, EV_REDLINE, EV_DTC,
  EV_LINKLOST, EV_WIFILOST, EV_BADRESP, EV_MAXSPEED, EV_MAXRPM
};
#define EV_MAX 24               // 24 * 6 байт = 144 байта

struct LogEvent {
  uint16_t sec;                 // время от старта, с
  uint8_t  kind;                // EV_*
  int16_t  val;                 // значение (темп, вольты*10, RPM, км/ч)
};

struct DriveLog {
  uint8_t  magic;                 // 0x5A = валидный лог
  uint8_t  count;                 // сколько слотов заполнено (до LOG_SLOTS)
  uint8_t  head;                  // индекс следующего слота (кольцо)
  uint8_t  boots;                 // счётчик запусков платы
  char     badResp[40];           // образец нераспознанного ответа адаптера
  char     badResp2[40];          // ВТОРОЙ образец, из середины поездки
  uint8_t  suffixOff;             // 1 = адаптер не понял суффикс кадров
  uint8_t  evCount;               // сколько аномалий в журнале
  LogEvent ev[EV_MAX];            // журнал аномалий
  LogSlot  slot[LOG_SLOTS];
};

static DriveLog dlog;

// --- накопители текущего слота ---
static uint32_t dlSlotStart = 0;
static uint32_t dlPolls = 0, dlFails = 0, dlTouts = 0, dlPend = 0;
static uint32_t dlRttSum = 0;
static uint16_t dlRttMin = 0xFFFF, dlRttMax = 0;
static uint16_t dlRpmMax = 0;
static uint8_t  dlSpdMax = 0;
static uint8_t  dlFlags  = 0;
static bool     dlDirty  = false;
static uint8_t  dlApCli  = 0;     // максимум клиентов на нашей AP за слот
static int8_t   dlRssi   = 0;     // последний RSSI сети адаптера

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

// timeout=true — ответа не было совсем; false — ответ пришёл, но не распознан
// (NO DATA / ошибка протокола). Лечится это по-разному, потому и считаем врозь.
inline void dlogFail(bool timeout) { dlFails++; if (timeout) dlTouts++; }
// «занят» и после повторного запроса — считаем отдельно от обычных сбоев
inline void dlogFailPending() { dlPend++; }

// Состояние радио на момент слота — чтобы проверить гипотезу «своя точка
// доступа отъедает эфир у связи с адаптером» (у ESP32-C3 одно радио).
// apUp — поднята ли наша AP, clients — сколько клиентов на ней висит,
// rssi — уровень сигнала сети адаптера.
inline void dlogRadio(bool apUp, uint8_t clients, int rssi) {
  if (apUp) dlFlags |= 0x02;
  if (clients > dlApCli) dlApCli = clients;
  if (rssi < 0 && rssi > -128) dlRssi = (int8_t)rssi;
}

// Запомнить образец нераспознанного ответа — чтобы дома понять, что
// именно шлёт адаптер вместо данных (NO DATA, мусор, чужой PID...).
// Пишем только первый за сессию: важен характер, а не количество.
inline void dlogSuffixOff() { dlog.suffixOff = 1; dlDirty = true; }

// ---- ЖУРНАЛ АНОМАЛИЙ ------------------------------------------------
// Пишем не текст, а код события + значение + время: 6 байт на запись.
// 24 записи = 144 байта. Дублирующиеся события подряд не пишем.
inline void dlogEvent(uint8_t kind, int16_t val) {
  if (dlog.evCount >= EV_MAX) {                 // кольцо: сдвигаем на 1
    memmove(&dlog.ev[0], &dlog.ev[1], sizeof(LogEvent) * (EV_MAX - 1));
    dlog.evCount = EV_MAX - 1;
  }
  // тот же вид события за последние 30 с — не дублируем
  if (dlog.evCount > 0) {
    const LogEvent& last = dlog.ev[dlog.evCount - 1];
    if (last.kind == kind && (uint16_t)(millis() / 1000) - last.sec < 30) return;
  }
  LogEvent& e = dlog.ev[dlog.evCount++];
  e.sec  = (uint16_t)(millis() / 1000);
  e.kind = kind;
  e.val  = val;
  dlDirty = true;
}

inline const char* dlogEventName(uint8_t k) {
  switch (k) {
    case EV_OVERHEAT:  return "перегрев";
    case EV_LOWVOLT:   return "низкое напряжение";
    case EV_REDLINE:   return "отсечка";
    case EV_DTC:       return "новая ошибка DTC";
    case EV_LINKLOST:  return "обрыв связи с адаптером";
    case EV_WIFILOST:  return "потеря WiFi";
    case EV_BADRESP:   return "нераспознанный ответ";
    case EV_MAXSPEED:  return "максимальная скорость";
    case EV_MAXRPM:    return "максимальные обороты";
    default:           return "?";
  }
}

// Два образца: первый (обычно при инициализации) и «поздний» — снятый
// после 3 минут работы. Первый часто ловит стартовый мусор и заслоняет
// то, что реально происходит в поездке.
inline void dlogBadResp(const char* s) {
  if (!dlog.badResp[0]) {
    strncpy(dlog.badResp, s, sizeof(dlog.badResp) - 1);
    dlog.badResp[sizeof(dlog.badResp) - 1] = 0;
    dlDirty = true;
    return;
  }
  if (!dlog.badResp2[0] && millis() > 180000) {
    strncpy(dlog.badResp2, s, sizeof(dlog.badResp2) - 1);
    dlog.badResp2[sizeof(dlog.badResp2) - 1] = 0;
    dlDirty = true;
  }
}
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
  s.touts  = (uint16_t)dlTouts;
  s.pend   = (uint16_t)dlPend;
  s.rttAvg = dlPolls ? (uint16_t)(dlRttSum / dlPolls) : 0;
  s.rttMin = (dlRttMin == 0xFFFF) ? 0 : dlRttMin;
  s.rttMax = dlRttMax;
  s.rpmMax = dlRpmMax;
  s.spdMax = dlSpdMax;
  s.flags  = dlFlags;
  s.apClients = dlApCli;
  s.rssi   = dlRssi;

  dlog.head = (dlog.head + 1) % LOG_SLOTS;
  if (dlog.count < LOG_SLOTS) dlog.count++;

  dlSlotStart = millis();
  dlPolls = dlFails = dlTouts = dlPend = dlRttSum = 0;
  dlRttMin = 0xFFFF; dlRttMax = 0;
  dlRpmMax = 0; dlSpdMax = 0; dlFlags = 0; dlApCli = 0;
  dlDirty = true;
  return true;
}

// Печать всего лога в Serial — забираем дома по USB.
inline void dlogDump() {
  Serial.println();
  Serial.println("===== DRIVE LOG BEGIN =====");
  Serial.printf("boots=%u slots=%u\n", dlog.boots, dlog.count);
  Serial.printf("bad1=[%s]\nbad2=[%s]\nsuffix_off=%u\n",
                dlog.badResp[0]  ? dlog.badResp  : "-",
                dlog.badResp2[0] ? dlog.badResp2 : "-", dlog.suffixOff);
  Serial.println("sec\thz\tfails\ttouts\tpend\trtt_avg\trtt_min\trtt_max\trpm_max\tspd_max\tlink\tap\tapcli\trssi");
  int start = (dlog.count < LOG_SLOTS) ? 0 : dlog.head;
  for (int i = 0; i < dlog.count; i++) {
    const LogSlot& s = dlog.slot[(start + i) % LOG_SLOTS];
    Serial.printf("%u\t%.1f\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%d\n",
                  s.sec, s.polls / 10.0f, s.fails, s.touts, s.pend,
                  s.rttAvg, s.rttMin, s.rttMax,
                  s.rpmMax, s.spdMax, (s.flags & 1) ? 1 : 0,
                  (s.flags & 2) ? 1 : 0, s.apClients, s.rssi);
  }
  Serial.printf("--- аномалии (%u) ---\n", dlog.evCount);
  for (int i = 0; i < dlog.evCount; i++) {
    const LogEvent& e = dlog.ev[i];
    Serial.printf("%5u с  %-26s %d\n", e.sec, dlogEventName(e.kind), e.val);
  }
  Serial.println("===== DRIVE LOG END =====");
}
