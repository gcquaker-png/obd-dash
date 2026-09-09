#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <LovyanGFX.hpp>
#include <WebServer.h>
#include <Update.h>          // OTA по WiFi
#include <HTTPClient.h>      // OTA по URL (GitHub releases)
#include <WiFiClientSecure.h>

// URL прошивки для обновления «по воздуху» одной кнопкой.
// GitHub Actions собирает и кладёт firmware.bin в релиз 'latest' при каждом пуше.
#define FW_URL "https://github.com/gcquaker-png/obd-dash/releases/latest/download/firmware.bin"
#define FW_MD5_URL "https://github.com/gcquaker-png/obd-dash/releases/latest/download/firmware.md5"
#include "version.h"
#include "dtc_db.h"
#include "pids.h"
#include "gear.h"
#include "alerts.h"
#include "accel.h"
#include "drivelog.h"
#include "netlist.h"

// ============================================================
// ESP32-2424S012 (ESP32-C3) OBD-II DASHBOARD  — версия без тача
// Display : GC9A01 240x240 round
// OBD     : ELM327 WiFi clone (KINGBOLEN), TCP 192.168.0.10:35000
// Кнопка  : сенсорная TTP223 на GPIO 20 (VCC->3V3, GND->GND, IO->20)
//           короткое касание -> следующий экран
//           долгое касание   -> предыдущий экран
//
// WiFi через WiFiManager: первый запуск поднимает точку OBD-Dash-XXXX,
// на 192.168.4.1 выбираешь сеть адаптера + правишь IP:порт. Далее авто.
// Держать сенсор при включении ~1.5 c -> сброс сети и портал.
// ============================================================

// ---- дисплей (как в рабочих скетчах 2424S012) ----
#define TFT_SCLK  6
#define TFT_MOSI  7
#define TFT_DC    2
#define TFT_CS    10
#define TFT_BL    3

// ---- кнопка навигации ----
// Сенсорная кнопка TTP223: VCC->3V3, GND->GND, IO->GPIO 20 (активна HIGH).
// (GPIO 21 на этой плате шумит — не использовать)
// Логика простая: любое чистое касание >= BTN_MIN_MS -> следующий экран.
// Опрос по аппаратному таймеру каждые 5 мс, независимо от отрисовки.
#define BTN_PIN       20
#define BTN_ACTIVE    HIGH
// TTP223 на этой плате «звенит»: одно касание = серия импульсов ~90 мс.
// Ловим ПЕРВЫЙ фронт LOW->HIGH как нажатие, дальше глухая пауза —
// весь звон и дребезг внутри неё игнорируются. Работает и в toggle-режиме.
#define BTN_LOCKOUT   400    // мс: после нажатия глухая пауза (гасит звон TTP223)

// ---- OBD-адаптер: значения по умолчанию (типовые для ELM327 WiFi) ----
const char*    DEF_OBD_HOST = "192.168.0.10";
const uint16_t DEF_OBD_PORT = 35000;

IPAddress OBD_IP;
uint16_t  OBD_PORT = DEF_OBD_PORT;
Preferences prefs;

// как часто опрашивать быстрые/медленные параметры
const uint32_t FAST_POLL_MS = 250;
const uint32_t SLOW_POLL_MS = 1000;

// ============================================================
// DISPLAY
// ============================================================
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_GC9A01 _panel;
  lgfx::Bus_SPI _bus;
  lgfx::Light_PWM _light;
public:
  LGFX() {
    { auto cfg = _bus.config();
      cfg.spi_host = SPI2_HOST;
      cfg.spi_mode = 0;
      cfg.freq_write = 40000000;
      cfg.freq_read  = 16000000;
      cfg.pin_sclk = TFT_SCLK;
      cfg.pin_mosi = TFT_MOSI;
      cfg.pin_miso = -1;
      cfg.pin_dc   = TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { auto cfg = _panel.config();
      cfg.pin_cs   = TFT_CS;
      cfg.pin_rst  = -1;
      cfg.pin_busy = -1;
      cfg.panel_width  = 240;
      cfg.panel_height = 240;
      cfg.memory_width  = 240;
      cfg.memory_height = 240;
      cfg.offset_x = 0;
      cfg.offset_y = 0;
      cfg.invert = true;
      cfg.rgb_order = false;
      _panel.config(cfg);
    }
    { auto cfg = _light.config();
      cfg.pin_bl = TFT_BL;
      cfg.invert = false;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX lcd;

// ============================================================
// ЭКРАНЫ
// ============================================================
enum Screen { SCREEN_GAUGE, SCREEN_PARAMS, SCREEN_ACCEL, SCREEN_DTC };
const int SCREEN_COUNT = 4;
volatile Screen currentScreen = SCREEN_GAUGE;   // меняется из ISR кнопки

// выбранные для экрана PARAMS параметры (битовая маска по PID_DEFS)
uint32_t pidMask = PID_MASK_DEFAULT;
PidVal   pidVals[PID_DEFS_LEN];        // последние декодированные значения

WebServer web(80);

volatile bool dirtyFull = true;   // требуется полная перерисовка экрана (ISR/веб)

// Собственная точка доступа: поднимается ТОЛЬКО в режиме настройки
// (долгое удержание кнопки). Пароль — чтобы никто посторонний не влез.
char apSsidGlobal[32] = "";
#define AP_PASS "obddash1"

// ============================================================
// СОСТОЯНИЕ OBD
// ============================================================
struct ObdData {
  int   rpm      = -1;
  int   speed    = -1;
  int   coolant  = -999;
  int   load     = -1;
  int   throttle = -1;
  float voltage  = -1;
  bool  linkUp   = false;
};
ObdData obd;

String dtcList = "";
volatile bool dtcValid = false;   // сбрасывается из ISR кнопки

WiFiClient elm;
enum ElmState { ELM_DISCONNECTED, ELM_CONNECTING, ELM_INIT, ELM_READY };
ElmState elmState = ELM_DISCONNECTED;

// ============================================================
// ELM327 ОБМЕН
// ============================================================
// screenChanged ставит ISR кнопки; любой висящий запрос сразу бросаем.
volatile bool screenChanged = false;

// Команда + CR ОДНИМ write(). Критично: ESP32-core не включает TCP_NODELAY
// (см. WiFiClient.cpp — setsockopt закомментирован), поэтому Nagle придерживал
// бы второй write (одиночный '\r'), пока не придёт ACK — ~40 мс задержки
// delayed-ACK НА КАЖДУЮ команду. Один write + setNoDelay при коннекте убирают это.
static char elmBuf[24];
String elmCmd(const String& cmd, uint32_t timeoutMs = 800) {
  while (elm.available()) elm.read();
  int n = cmd.length();
  if (n > (int)sizeof(elmBuf) - 2) n = sizeof(elmBuf) - 2;
  memcpy(elmBuf, cmd.c_str(), n);
  elmBuf[n] = '\r';
  elm.write((const uint8_t*)elmBuf, n + 1);   // один сегмент, без Nagle-паузы

  String resp;
  resp.reserve(48);                            // без реаллокаций на каждый +=
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    int avail = elm.available();
    if (avail > 0) {
      while (avail-- > 0) {
        char c = elm.read();
        if (c == '>') {
          resp.replace(cmd, "");
          resp.replace("\r", " ");
          resp.replace("\n", " ");
          resp.trim();
          return resp;
        }
        resp += c;
      }
      t0 = millis();
      continue;                                // данные идут — не спим
    }
    if (screenChanged) return "";   // кнопка сменила экран — бросаем запрос
    delay(1);
  }
  return "";
}

int parsePid(const String& resp, uint8_t expectMode, uint8_t expectPid, uint8_t* out, int maxOut) {
  String s = resp;
  s.toUpperCase();
  if (s.indexOf("NO DATA") >= 0 || s.indexOf("STOPPED") >= 0 ||
      s.indexOf("ERROR")   >= 0 || s.indexOf("UNABLE") >= 0 || s.indexOf("?") >= 0) return 0;

  uint8_t bytes[32]; int n = 0; int i = 0;
  while (i < (int)s.length() && n < 32) {
    while (i < (int)s.length() && !isxdigit(s[i])) i++;
    if (i + 1 >= (int)s.length()) break;
    if (!isxdigit(s[i]) || !isxdigit(s[i + 1])) { i++; continue; }
    bytes[n++] = (uint8_t)strtol(s.substring(i, i + 2).c_str(), nullptr, 16);
    i += 2;
  }
  uint8_t respMode = 0x40 | expectMode;
  for (int k = 0; k + 1 < n; k++) {
    if (bytes[k] == respMode && bytes[k + 1] == expectPid) {
      int c = 0;
      for (int m = k + 2; m < n && c < maxOut; m++) out[c++] = bytes[m];
      return c;
    }
  }
  return 0;
}

bool queryPid01(uint8_t pid, uint8_t* out, int maxOut, int& cnt) {
  char cmd[8];
  snprintf(cmd, sizeof(cmd), "01%02X", pid);
  String r = elmCmd(cmd, 800);
  if (r.isEmpty()) return false;
  cnt = parsePid(r, 0x01, pid, out, maxOut);
  return cnt > 0;
}

// БЫСТРЫЙ опрос одного PID (mode 01) — для RPM и скорости на GAUGE.
// Суффикс "1" ("010C1") = «жди ровно 1 фрейм»: ELM отдаёт ответ сразу, как
// получил его от ЭБУ, вместо ожидания своего межфреймового таймаута (~200 мс).
// Это главный протокольный ускоритель на клонах.
// Таймаут 300 мс: при ATST20 живой ответ приходит за ~30-80 мс, всё что дольше —
// уже потеря, и ждать её незачем (быстрее сделать следующий запрос).
// lastRtt — время последнего обмена с адаптером, мс.
// Замер показал rtt ~37 мс стабильно: адаптер быстрый, потолок ~27 Гц.
// Реальная частота падала из-за ПОТЕРЬ: каждый таймаут съедал 300 мс.
// Теперь таймаут 150 мс (в 4 раза больше типичного ответа — с запасом),
// и различаем причину: пустой ответ (таймаут) vs ответ есть, но не распарсен.
static uint16_t lastRtt = 0;
static bool     lastWasTimeout = false;
#define FAST_TIMEOUT_MS 150

// Суффикс числа фреймов ("010C1") ускоряет ответ: ELM отдаёт данные сразу,
// не дожидаясь межфреймового таймаута. НО не все клоны его понимают —
// замер показал, что этот KINGBOLEN на часть таких команд отвечает "?"
// («не понял»), и запрос уходит в никуда. Поэтому: пробуем с суффиксом,
// на первом же "?" переключаемся на обычный формат до перезагрузки.
static bool fastSuffixOk = true;

bool queryFast01(uint8_t pid, uint8_t* out, int maxOut, int& cnt) {
  char cmd[10];
  if (fastSuffixOk) snprintf(cmd, sizeof(cmd), "01%02X1", pid);
  else              snprintf(cmd, sizeof(cmd), "01%02X",  pid);

  uint32_t t0 = millis();
  String r = elmCmd(cmd, FAST_TIMEOUT_MS);
  lastRtt = (uint16_t)(millis() - t0);
  if (r.isEmpty()) { lastWasTimeout = true; return false; }
  lastWasTimeout = false;

  // "?" = адаптер не понял команду. Если это был запрос с суффиксом —
  // значит клон его не поддерживает: отключаем и повторяем без него.
  if (r.indexOf('?') >= 0) {
    if (fastSuffixOk) {
      fastSuffixOk = false;
      Serial.println("ELM: суффикс кадров не поддержан -> обычный формат");
      dlogSuffixOff();
      snprintf(cmd, sizeof(cmd), "01%02X", pid);
      t0 = millis();
      r = elmCmd(cmd, FAST_TIMEOUT_MS);
      lastRtt = (uint16_t)(millis() - t0);
      if (r.isEmpty()) { lastWasTimeout = true; return false; }
    }
  }

  cnt = parsePid(r, 0x01, pid, out, maxOut);
  if (cnt <= 0) {
    dlogBadResp(r.c_str());               // образец в лог (переживёт поездку)
    static uint32_t tBad = 0;
    if (millis() - tBad >= 2000) {
      tBad = millis();
      Serial.printf("BAD RESP pid=%02X: [%s]\n", pid, r.c_str());
    }
  }
  return cnt > 0;
}

// Мультизапрос: одна команда "01 <p1><p2>..." -> один ответ со всеми PID.
// ELM327 (и большинство клонов) поддерживают до 6 PID в запросе.
// lastMultiOk = сработал ли мультирежим (иначе откат на поштучный опрос).
static bool lastMultiOk = true;
String queryMulti(const uint8_t* pids, int npid) {
  char cmd[24] = "01";
  for (int i = 0; i < npid && i < 6; i++)
    sprintf(cmd + strlen(cmd), "%02X", pids[i]);
  return elmCmd(cmd, 900);
}

// ============================================================
// ОПРОС — мелкими быстрыми запросами (по одному PID)
// ============================================================
// счётчики промахов: значение гасим только после MISS_LIMIT неудач подряд,
// иначе держим последнее — так одиночные тайм-ауты не рисуют «пустые полосы»
#define MISS_LIMIT 5
static uint8_t missRpm, missSpd, missCool, missLoad, missThr, missVolt;
static void bumpMiss(uint8_t& m, int& val, int deadVal) {
  if (++m >= MISS_LIMIT) { m = MISS_LIMIT; val = deadVal; }
}

// KINGBOLEN не понял мультизапрос -> опрашиваем поштучно, но РАЗДЕЛЬНО:
// быстрые (RPM+скорость) — каждый цикл; медленные — по одному за проход.

// RPM — самый частый запрос (для тахо-стрелки). Вызывать каждый цикл.
void pollRpm() {
  uint8_t b[8]; int c;
  if (queryFast01(0x0C, b, 8, c) && c >= 2) {
    obd.rpm = ((b[0] << 8) | b[1]) / 4; missRpm = 0;
    dlogPoll(lastRtt, obd.rpm, obd.speed);      // замер в лог
  } else {
    bumpMiss(missRpm, obd.rpm, -1);
    dlogFail(lastWasTimeout);
  }
}

// Скорость — реже (для цифры и передачи хватает). Вызывать раз в ~3 цикла.
void pollSpeed() {
  uint8_t b[8]; int c;
  if (queryFast01(0x0D, b, 8, c) && c >= 1) { obd.speed = b[0]; missSpd = 0; }
  else bumpMiss(missSpd, obd.speed, -1);
}

// оставлено для alert-оверлея: RPM+скорость за раз
void pollFast() { pollRpm(); pollSpeed(); }

// МЕДЛЕННОЕ: один параметр за вызов, по кругу (темп/нагрузка/дроссель/вольты).
void pollSlowStep() {
  static uint8_t idx = 0;
  uint8_t b[8]; int c;
  switch (idx) {
    case 0:
      if (queryPid01(0x05, b, 8, c) && c >= 1) { obd.coolant = b[0] - 40; missCool = 0; } else bumpMiss(missCool, obd.coolant, -999);
      break;
    case 1:
      if (queryPid01(0x04, b, 8, c) && c >= 1) { obd.load = b[0] * 100 / 255; missLoad = 0; } else bumpMiss(missLoad, obd.load, -1);
      break;
    case 2:
      if (queryPid01(0x11, b, 8, c) && c >= 1) { obd.throttle = b[0] * 100 / 255; missThr = 0; } else bumpMiss(missThr, obd.throttle, -1);
      break;
    case 3:
      pollVoltage();
      break;
  }
  idx = (idx + 1) % 4;
}

// ============================================================
// ДЕМО-РЕЖИМ — генерит правдоподобные данные без адаптера.
// Включить: собрать с -DDEMO, либо кнопкой в веб-морде (demoOn).
// ============================================================
bool demoOn =
#ifdef DEMO
  true;
#else
  false;
#endif

void pollDemo() {
  uint32_t t = millis();
  float s1 = sinf(t / 1700.0f);          // "газ-тормоз" ~11 c период
  float s2 = sinf(t / 400.0f);           // мелкая дрожь
  int rpm = 900 + (int)((s1 * 0.5f + 0.5f) * 5200) + (int)(s2 * 60);
  int spd = (int)((s1 * 0.5f + 0.5f) * 140) + (int)(sinf(t/900.0f) * 3);
  if (rpm < 700) rpm = 700;
  if (spd < 0)   spd = 0;
  obd.rpm      = rpm;
  obd.speed    = spd;
  obd.coolant  = 88 + (int)(sinf(t / 9000.0f) * 6);           // 82..94
  obd.throttle = (int)((s1 * 0.5f + 0.5f) * 90);
  obd.load     = 15 + (int)((s1 * 0.5f + 0.5f) * 70);
  obd.voltage  = 14.1f + sinf(t / 3000.0f) * 0.3f;
  obd.linkUp   = true;
  missRpm = missSpd = missCool = missLoad = missThr = missVolt = 0;
}

// только RPM+скорость (для экрана ACCEL — нужна быстрая скорость)
void pollRpmSpeed() {
  uint8_t b[8]; int c;
  if (queryPid01(0x0C, b, 8, c) && c >= 2) { obd.rpm = ((b[0] << 8) | b[1]) / 4; missRpm = 0; }
  else bumpMiss(missRpm, obd.rpm, -1);
  if (queryPid01(0x0D, b, 8, c) && c >= 1) { obd.speed = b[0]; missSpd = 0; }
  else bumpMiss(missSpd, obd.speed, -1);
}

void pollVoltage() {
  String rv = elmCmd("ATRV", 800);
  rv.toUpperCase(); rv.replace("V", ""); rv.trim();
  float v = rv.toFloat();
  if (v > 5.0 && v < 20.0) { obd.voltage = v; missVolt = 0; }
  else if (++missVolt >= MISS_LIMIT) { missVolt = MISS_LIMIT; obd.voltage = -1; }
}

// Опрос одного параметра из каталога PID_DEFS по индексу.
// При промахе держим прежнее значение MISS_LIMIT раз, потом гасим.
static uint8_t pidMiss[PID_DEFS_LEN];
void pollOneParam(int idx) {
  if (idx < 0 || idx >= PID_DEFS_LEN) return;
  const PidDef& d = PID_DEFS[idx];
  PidRaw raw; raw.n = 0;
  uint8_t b[8]; int c = 0;
  if (queryPid01(d.pid, b, 6, c) && c > 0) {
    for (int i = 0; i < c && i < 6; i++) raw.b[i] = b[i];
    raw.n = c;
    pidVals[idx] = d.decode(raw);
    pidMiss[idx] = 0;
  } else if (++pidMiss[idx] >= MISS_LIMIT) {
    pidMiss[idx] = MISS_LIMIT;
    pidVals[idx] = bad();
  }
}

// Опросить по кругу все ВЫБРАННЫЕ параметры (по одному за вызов).
void pollParamsStep() {
  static int cursor = 0;
  for (int tries = 0; tries < PID_DEFS_LEN; tries++) {
    int idx = (cursor + tries) % PID_DEFS_LEN;
    if (pidMask & (1u << idx)) {
      pollOneParam(idx);
      cursor = (idx + 1) % PID_DEFS_LEN;
      return;
    }
  }
}

bool     dtcNewFlag = false;        // появился новый код с прошлого чтения
String   dtcPrev    = "\x01";      // предыдущий список (для сравнения)

void pollDtc() {
  String r = elmCmd("03", 1800);
  if (screenChanged) return;        // ушли с экрана во время запроса
  r.toUpperCase();
  dtcValid = true;
  dtcList = "";
  if (r.isEmpty() || r.indexOf("NO DATA") >= 0 || r.indexOf("ERROR") >= 0 ||
      r.indexOf("UNABLE") >= 0 || r.indexOf("?") >= 0) return;

  uint8_t bytes[64]; int n = 0;
  for (int i = 0; i + 1 < (int)r.length() && n < 64; ) {
    while (i < (int)r.length() && !isxdigit(r[i])) i++;
    if (i + 1 >= (int)r.length() || !isxdigit(r[i]) || !isxdigit(r[i + 1])) { i++; continue; }
    bytes[n++] = (uint8_t)strtol(r.substring(i, i + 2).c_str(), nullptr, 16);
    i += 2;
  }
  int start = (n > 0 && bytes[0] == 0x43) ? 1 : 0;
  const char sys[] = {'P', 'C', 'B', 'U'};
  int count = 0;
  for (int i = start; i + 1 < n; i += 2) {
    uint16_t code = (bytes[i] << 8) | bytes[i + 1];
    if (code == 0) continue;
    char buf[8];
    snprintf(buf, sizeof(buf), "%c%01X%03X",
             sys[(code >> 14) & 0x03], (code >> 12) & 0x03, code & 0x0FFF);
    if (count) dtcList += "\n";
    dtcList += buf;
    if (++count >= 8) break;
  }
  // новый код? (список стал длиннее или изменился, и не пустой)
  if (dtcList.length() > 0 && dtcList != dtcPrev && dtcPrev != "\x01")
    dtcNewFlag = true;
  dtcPrev = dtcList;
}

// ============================================================
// NVS: маска выбранных параметров
// ============================================================
void loadPidMask() {
  prefs.begin("obd", true);
  pidMask = prefs.getUInt("pidmask", PID_MASK_DEFAULT);
  prefs.end();
  if (pidMask == 0) pidMask = PID_MASK_DEFAULT;
}
void savePidMask() {
  prefs.begin("obd", false);
  prefs.putUInt("pidmask", pidMask);
  prefs.end();
}

// ============================================================
// WEB-МОРДА (порт 80, всегда доступна пока плата в сети)
// ============================================================
String webPage() {
  String h = F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
               "<style>body{font:16px/1.5 sans-serif;margin:16px;background:#111;color:#eee}"
               "h1{font-size:18px}label{display:block;padding:6px 0;border-bottom:1px solid #333}"
               "input{transform:scale(1.4);margin-right:12px}"
               "button{margin-top:16px;padding:10px 24px;font-size:16px;background:#2a7;color:#fff;border:0;border-radius:6px}"
               "</style><h1>OBD Dash</h1>");
  h += String("<p style=color:#888;margin-top:-8px>прошивка: <b>") + FW_VERSION + " / " + FW_BUILD_DATE + "</b></p>";
  h += F("<h1 style=font-size:17px>Параметры экрана</h1><form method=POST action=/save>");
  for (int i = 0; i < PID_DEFS_LEN; i++) {
    bool on = pidMask & (1u << i);
    h += "<label><input type=checkbox name=p" + String(i) + (on ? " checked>" : ">");
    h += String(PID_DEFS[i].label) + " <span style=color:#888>(" + PID_DEFS[i].key + ")</span></label>";
  }
  h += F("<button type=submit>Сохранить</button></form>");

  // --- WiFi-сети с приоритетом ---
  h += F("<h1 style=margin-top:28px>WiFi-сети</h1>"
         "<p style=color:#888>Слот 1 — сеть OBD-адаптера, у неё приоритет. "
         "Если её нет в эфире, плата подключится к следующей из списка.</p>"
         "<form method=POST action=/nets>");
  for (int i = 0; i < NET_MAX; i++) {
    h += "<div style=margin-bottom:6px>";
    if (i == 0) h += F("<b>1. OBD-адаптер</b><br>");
    else        h += "<b>" + String(i + 1) + ". запасная</b><br>";
    h += "<input name=s" + String(i) + " placeholder='SSID' value='" +
         String(nets.net[i].ssid) + "' style='width:45%'> ";
    // Пароль не подставляем в поле — не светим его в HTML.
    // Пусто = оставить прежний, ввод = заменить.
    h += "<input name=p" + String(i) + " type=password style='width:45%' placeholder='";
    h += nets.net[i].pass[0] ? "пароль сохранён" : "пароль";
    h += "'>";
    h += "</div>";
  }
  h += F("<button type=submit>Сохранить сети</button></form>");
  h += "<p style=color:#888>Сейчас: ";
  if (WiFi.status() == WL_CONNECTED)
    h += "<b>" + WiFi.SSID() + "</b>" + (netIsObd ? " (OBD)" : " (запасная)");
  else h += "нет подключения";
  h += F("</p>");

  // --- блок калибровки передач ---
  h += F("<h1 style=margin-top:28px>Передачи</h1>");
  h += F("<form method=POST action=/geartarget>Всего передач у машины: "
         "<input name=t type=number min=3 max=7 style='width:60px' value='");
  h += String(gears.target) + F("'> <button type=submit>OK</button></form>");

  h += "<p>Запомнено: <b>" + String(gears.count) + " / " + String(gears.target) + "</b> ";
  h += gearLocked ? F("<span style=color:#2a7>(готово, калибровка заморожена)</span>")
                  : F("<span style=color:#fa0>(идёт калибровка)</span>");
  h += F("<br>");
  for (int i = 0; i < gears.count; i++)
    h += String(i + 1) + ": ratio " + String(gears.ratio[i], 1) + "<br>";
  if (gears.count == 0)
    h += F("<span style=color:#888>Проедь на каждой передаче на ровном "
           "умеренном газу ~2 сек — плата запомнит сама.</span>");
  h += F("</p><form method=POST action=/gearreset>"
         "<button style=background:#a33>Сбросить калибровку передач</button></form>");

  // --- пороги предупреждений ---
  h += F("<h1 style=margin-top:28px>Предупреждения</h1>"
         "<form method=POST action=/alerts>"
         "<label>Перегрев ОЖ, &deg;C <input name=cm type=number style='width:80px' value='");
  h += String(alertCfg.coolMax) + F("'></label>"
         "<label>Мин. напряжение (на заведённой), В <input name=vm style='width:80px' value='");
  h += String(alertCfg.voltMin, 1) + F("'></label>"
         "<label>Отсечка, об/мин <input name=rm type=number style='width:90px' value='");
  h += String(alertCfg.rpmMax) + F("'></label>"
         "<button type=submit>Сохранить пороги</button></form>");

  // --- разгон ---
  h += F("<h1 style=margin-top:28px>Разгон</h1><p>");
  for (int i = 0; i < ACCEL_NMARKS; i++) {
    h += "0-" + String(ACCEL_MARKS[i]) + ": ";
    h += (accelBest.t[i] > 0 ? String(accelBest.t[i], 1) + " с" : String("--"));
    h += "<br>";
  }
  h += F("</p><form method=POST action=/accelreset>"
         "<button style=background:#a33>Сбросить рекорды разгона</button></form>");

  // --- демо-режим ---
  h += F("<h1 style=margin-top:28px>Демо</h1><form method=POST action=/demo>");
  h += String("<p>Сейчас: <b>") + (demoOn ? "ВКЛ" : "выкл") + "</b></p>";
  h += F("<button type=submit>Переключить демо-режим</button></form>");

  h += F("<h1 style=margin-top:28px>Прошивка</h1>"
         "<p><a href=/ota style='color:#2a7'>Обновить прошивку по WiFi &raquo;</a></p>");
  return h;
}

void webBegin() {
  web.on("/", []() { web.send(200, "text/html; charset=utf-8", webPage()); });
  web.on("/save", HTTP_POST, []() {
    uint32_t m = 0;
    for (int i = 0; i < PID_DEFS_LEN; i++)
      if (web.hasArg("p" + String(i))) m |= (1u << i);
    if (m == 0) m = PID_MASK_DEFAULT;
    pidMask = m;
    savePidMask();
    dirtyFull = true;
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Сохранено."));
  });
  // Лог замеров текстом: и с телефона в setup-режиме, и с компа по /log
  web.on("/log", []() {
    String s = "sec\thz\tfails\ttouts\trtt_avg\trtt_min\trtt_max\trpm_max\tspd_max\tlink\n";
    s.reserve(2048);
    int start = (dlog.count < LOG_SLOTS) ? 0 : dlog.head;
    for (int i = 0; i < dlog.count; i++) {
      const LogSlot& sl = dlog.slot[(start + i) % LOG_SLOTS];
      char line[110];
      snprintf(line, sizeof(line), "%u\t%.1f\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\n",
               sl.sec, sl.polls / 10.0f, sl.fails, sl.touts, sl.rttAvg, sl.rttMin,
               sl.rttMax, sl.rpmMax, sl.spdMax, (sl.flags & 1) ? 1 : 0);
      s += line;
    }
    web.send(200, "text/plain; charset=utf-8", s);
  });
  // Сохранённые WiFi-сети. Слот 0 — сеть OBD-адаптера (приоритет),
  // слоты 1..3 — запасные (дом/работа) для OTA и веб-морды.
  web.on("/nets", HTTP_POST, []() {
    for (int i = 0; i < NET_MAX; i++) {
      String ks = "s" + String(i), kp = "p" + String(i);
      if (web.hasArg(ks)) {
        String ss = web.arg(ks), pp = web.hasArg(kp) ? web.arg(kp) : "";
        ss.trim();
        if (ss.length()) {
          // пустое поле пароля = оставить сохранённый (его не показываем).
          // Копируем в буфер: netSet пишет в ту же структуру, откуда читаем.
          char keep[NET_PASS_LEN];
          strncpy(keep, nets.net[i].pass, sizeof(keep) - 1);
          keep[sizeof(keep) - 1] = 0;
          netSet(prefs, i, ss.c_str(), pp.length() ? pp.c_str() : keep);
        } else if (i > 0) netClear(prefs, i);
      }
    }
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Сети сохранены."));
  });

  web.on("/logclear", HTTP_POST, []() {
    dlogClear(prefs);
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Лог очищен."));
  });

  web.on("/gearreset", HTTP_POST, []() {
    gearReset(prefs);
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Калибровка передач сброшена."));
  });
  web.on("/geartarget", HTTP_POST, []() {
    if (web.hasArg("t")) gearSetTarget(prefs, web.arg("t").toInt());
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>OK"));
  });
  web.on("/alerts", HTTP_POST, []() {
    if (web.hasArg("cm")) alertCfg.coolMax = web.arg("cm").toInt();
    if (web.hasArg("vm")) alertCfg.voltMin = web.arg("vm").toFloat();
    if (web.hasArg("rm")) alertCfg.rpmMax  = web.arg("rm").toInt();
    if (alertCfg.coolMax < 60)  alertCfg.coolMax = 105;
    if (alertCfg.voltMin < 8)   alertCfg.voltMin = 12.0;
    if (alertCfg.rpmMax  < 3000) alertCfg.rpmMax = 6500;
    alertSave(prefs);
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Пороги сохранены."));
  });
  web.on("/accelreset", HTTP_POST, []() {
    accelReset(prefs);
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>Рекорды разгона сброшены."));
  });
  web.on("/demo", HTTP_POST, []() {
    demoOn = !demoOn;
    dirtyFull = true;
    web.send(200, "text/html; charset=utf-8",
             F("<meta http-equiv=refresh content='1;url=/'>OK"));
  });

  // ---- OTA: страница ----
  web.on("/ota", HTTP_GET, []() {
    web.send(200, "text/html; charset=utf-8",
      F("<!doctype html><meta name=viewport content='width=device-width,initial-scale=1'>"
        "<style>body{font:16px sans-serif;margin:16px;background:#111;color:#eee}"
        "button{padding:10px 24px;font-size:16px;background:#2a7;color:#fff;border:0;border-radius:6px;margin-top:10px}"
        "a{color:#2a7}</style>"
        "<h1>Обновление прошивки</h1>"
        "<h2 style=font-size:17px>1. Из интернета (GitHub)</h2>"
        "<p>Плата сама скачает свежую сборку и прошьётся.<br>"
        "<b>Нужен интернет</b> в сети, к которой подключена плата (не только сеть адаптера).</p>"
        "<form method=POST action=/otaurl><button type=submit>Обновить из GitHub</button></form>"
        "<h2 style=font-size:17px;margin-top:24px>2. Файлом .bin</h2>"
        "<form method=POST action=/ota enctype=multipart/form-data>"
        "<input type=file name=f accept='.bin' required><br>"
        "<button type=submit>Прошить файлом</button></form>"));
  });

  // ---- OTA по URL (GitHub releases) ----
  web.on("/otaurl", HTTP_POST, []() {
    if (WiFi.status() != WL_CONNECTED) {
      web.send(200, "text/html; charset=utf-8",
        F("<meta http-equiv=refresh content='3;url=/ota'>Нет интернета: плата не в сети со свободным выходом. "
          "Подключи её к домашней/телефонной сети через портал (держи сенсор при включении)."));
      return;
    }
    web.send(200, "text/html; charset=utf-8",
      F("<meta http-equiv=refresh content='20;url=/'>Качаю прошивку с GitHub, проверяю контрольную сумму... "
        "если всё ок — перезагрузка через ~15-20 сек. Иначе плата останется на текущей версии."));
    delay(300);

    WiFiClientSecure sec;
    sec.setInsecure();
    HTTPClient http;

    // 1) забрать ожидаемый MD5
    http.begin(sec, FW_MD5_URL);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    String wantMd5;
    if (http.GET() == HTTP_CODE_OK) { wantMd5 = http.getString(); wantMd5.trim(); wantMd5.toLowerCase(); }
    http.end();
    Serial.printf("OTA-URL expected md5: %s\n", wantMd5.c_str());

    // 2) скачать сам .bin
    http.begin(sec, FW_URL);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();
    Serial.printf("OTA-URL HTTP %d\n", code);
    if (code != HTTP_CODE_OK) { http.end(); return; }

    int len = http.getSize();
    if (len < 100000) { Serial.println("OTA-URL: too small, abort"); http.end(); return; }

    if (wantMd5.length() == 32) Update.setMD5(wantMd5.c_str());   // Update сам сверит
    if (!Update.begin(len)) { Update.printError(Serial); http.end(); return; }

    size_t written = Update.writeStream(http.getStream());
    Serial.printf("OTA-URL written %u / %d\n", written, len);
    http.end();

    if (written != (size_t)len) {
      Serial.println("OTA-URL: size mismatch, abort");
      Update.abort();
      return;
    }
    if (Update.end(true) && !Update.hasError()) {
      Serial.println("OTA-URL OK (md5 verified), restart");
      delay(400);
      ESP.restart();
    } else {
      Serial.print("OTA-URL FAIL: "); Update.printError(Serial);
    }
  });
  web.on("/ota", HTTP_POST,
    []() {   // финал
      bool ok = !Update.hasError() && Update.isFinished();
      web.send(200, "text/html; charset=utf-8",
        ok ? F("<meta http-equiv=refresh content='4;url=/'>OK, перезагрузка...")
           : F("ОШИБКА прошивки (файл битый/неполный). Плата осталась на текущей версии. Скачай .bin заново."));
      delay(400);
      if (ok) ESP.restart();
    },
    []() {   // приём кусками
      static bool bad = false;
      HTTPUpload& up = web.upload();
      if (up.status == UPLOAD_FILE_START) {
        bad = false;
        Serial.printf("OTA file: %s\n", up.filename.c_str());
        // первый байт образа ESP32 = 0xE9; проверим когда придёт
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); bad = true; }
      } else if (up.status == UPLOAD_FILE_WRITE) {
        if (bad) return;
        if (up.totalSize == 0 && up.currentSize > 0 && up.buf[0] != 0xE9) {
          Serial.println("OTA: not an ESP32 image (magic != E9)");
          Update.abort(); bad = true; return;
        }
        if (Update.write(up.buf, up.currentSize) != up.currentSize) { Update.printError(Serial); bad = true; }
      } else if (up.status == UPLOAD_FILE_END) {
        if (bad) { Update.abort(); return; }
        if (Update.end(true)) Serial.printf("OTA done: %u bytes\n", up.totalSize);
        else Update.printError(Serial);
      } else if (up.status == UPLOAD_FILE_ABORTED) {
        Update.abort(); bad = true;
      }
    });

  web.begin();
  Serial.println("WebServer on :80  (OTA: /ota)");
}

// ============================================================
// WIFI ЧЕРЕЗ WIFIMANAGER
// ============================================================
void connectWiFi(bool forcePortal) {
  uint64_t chipid = ESP.getEfuseMac();
  char apName[32];
  snprintf(apName, sizeof(apName), "OBD-Dash-%04X", (uint16_t)(chipid & 0xFFFF));

  prefs.begin("obd", true);
  String host = prefs.getString("host", DEF_OBD_HOST);
  uint16_t port = prefs.getUShort("port", DEF_OBD_PORT);
  prefs.end();
  OBD_IP.fromString(host);
  OBD_PORT = port;

#ifdef FORCE_PORTAL
  forcePortal = true;
#endif

  // --- НЕ форс-портал: пробуем сохранённую сеть, БЕЗ портала ---
  // Режим уже WIFI_AP_STA (AP поднята в setup ДО нас) — не трогаем mode,
  // иначе уроним свою точку доступа.
  if (!forcePortal) {
    // Приоритет: сеть OBD-адаптера (slot 0). Если её нет в эфире —
    // подключаемся к сохранённой домашней сети (для OTA/веб-морды).
    // Достаточно ЛЮБОЙ заполненной сети в списке — слот 0 (OBD) может
    // быть пустым, если настроена только запасная домашняя сеть.
    bool haveAny = false;
    for (int i = 0; i < NET_MAX; i++) if (nets.net[i].ssid[0]) { haveAny = true; break; }
    if (haveAny) {
      if (netConnectBest() < 0)
        Serial.println("известных сетей нет в эфире — ретрай в фоне");
      if (netDirty) { netSave(prefs); netDirty = false; }
    } else {
      // Список пуст (первый запуск после обновления). Сначала пробуем
      // автоопределение адаптера по эфиру, затем — последнюю сеть из
      // памяти WiFiManager.
      if (netConnectBest() >= 0) {
        if (netDirty) { netSave(prefs); netDirty = false; }
      } else {
        WiFi.begin();
        Serial.print("WiFi connecting");
        uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
          delay(200); Serial.print(".");
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
          Serial.printf("WiFi OK  SSID=%s  IP=%s\n",
                        WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
          netSet(prefs, 0, WiFi.SSID().c_str(), WiFi.psk().c_str());
          netActive = 0; netIsObd = true;
          Serial.println("сеть запомнена как OBD (slot 1)");
        } else {
          Serial.println("WiFi not connected — работаем без сети, ретрай в фоне");
        }
      }
    }
    Serial.printf("OBD target %s:%u\n", host.c_str(), port);
    return;
  }

  // --- форс-портал (held-кнопка): блокирующий config portal ---
  WiFiManager wm;
  wm.setTitle("OBD Dashboard");
  wm.setConfigPortalTimeout(300);
  wm.resetSettings();

  char portBuf[8];
  snprintf(portBuf, sizeof(portBuf), "%u", port);
  WiFiManagerParameter pHost("host", "OBD adapter IP", host.c_str(), 20);
  WiFiManagerParameter pPort("port", "OBD adapter port", portBuf, 6);
  wm.addParameter(&pHost);
  wm.addParameter(&pPort);

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(1);
  lcd.drawString("WiFi setup", 120, 100);
  lcd.drawString(apName, 120, 120);
  lcd.drawString("-> 192.168.4.1", 120, 140);

  if (!wm.startConfigPortal(apName)) {
    Serial.println("portal timeout -> restart");
    ESP.restart();
  }

  String newHost = pHost.getValue();
  uint16_t newPort = (uint16_t)atoi(pPort.getValue());
  if (newHost.length() < 7) newHost = DEF_OBD_HOST;
  if (newPort == 0) newPort = DEF_OBD_PORT;

  prefs.begin("obd", false);
  prefs.putString("host", newHost);
  prefs.putUShort("port", newPort);
  prefs.end();

  OBD_IP.fromString(newHost);
  OBD_PORT = newPort;
  Serial.printf("WiFi OK  SSID=%s  IP=%s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  Serial.printf("OBD target %s:%u\n", newHost.c_str(), newPort);
}

// ============================================================
// TCP К АДАПТЕРУ (неблокирующий автомат)
// ============================================================
void elmService() {
  static uint32_t tMark = 0, tRetry = 0;
  switch (elmState) {
    case ELM_DISCONNECTED:
      if (millis() - tRetry < 3000) break;   // без адаптера — пробуем раз в 3 с
      tRetry = millis(); tMark = millis();
      elmState = ELM_CONNECTING;
      break;

    case ELM_CONNECTING:
      if (WiFi.status() == WL_CONNECTED) {
        elm.stop();
        // connect с коротким таймаутом — иначе без адаптера блокирует
        // loop на несколько секунд и кнопка «протупливает».
        if (elm.connect(OBD_IP, OBD_PORT, 600)) {
          // Nagle OFF: ядро ESP32 его НЕ выключает само. Без этого каждый
          // короткий запрос ждёт delayed-ACK (~40 мс) — главный тормоз опроса.
          elm.setNoDelay(true);
          Serial.println("TCP to ELM327 OK (nodelay)");
          elmState = ELM_INIT;
        } else {
          tRetry = millis();
          elmState = ELM_DISCONNECTED;   // пауза 1.5 c до следующей попытки
        }
      } else if (millis() - tMark > 15000) {
        // Переподключаемся по списку с приоритетом: в движении сеть
        // адаптера может появиться позже домашней (или наоборот).
        Serial.println("WiFi lost, reconnecting");
        if (netConnectBest(6000) < 0) WiFi.reconnect();
        else if (netDirty) { netSave(prefs); netDirty = false; }
        tRetry = millis();
        elmState = ELM_DISCONNECTED;
      }
      break;

    case ELM_INIT: {
      elmCmd("ATZ", 1000);   if (screenChanged) break;
      elmCmd("ATE0", 400);   if (screenChanged) break;   // эхо off
      elmCmd("ATL0", 400);   if (screenChanged) break;   // линефиды off
      elmCmd("ATS0", 400);   if (screenChanged) break;   // пробелы off
      elmCmd("ATH0", 400);   if (screenChanged) break;   // заголовки off
      elmCmd("ATAT1", 400);  if (screenChanged) break;   // адаптивный тайминг
      elmCmd("ATST20", 400); if (screenChanged) break;   // потолок ожидания ЭБУ = 0x20*4 ≈ 128 мс (по умолч. ~200)
      elmCmd("ATCAF1", 400); if (screenChanged) break;   // авто-формат CAN
      // жёстко ISO 15765 500k 11-bit (почти все машины 2008+) — без авто-детекта
      elmCmd("ATSP6", 400);  if (screenChanged) break;
      lastMultiOk = true;                                 // пробуем мультизапрос заново
      String r = elmCmd("0100", 1500);
      if (r.indexOf("41") >= 0 || r.indexOf("SEARCHING") >= 0) {
        Serial.println("ELM ready (SP6)");
        obd.linkUp = true;
        elmState = ELM_READY;
      } else {
        // SP6 не подошёл — откат на авто-детект
        elmCmd("ATSP0", 400);
        String r2 = elmCmd("0100", 3000);
        if (r2.indexOf("41") >= 0 || r2.indexOf("SEARCHING") >= 0) {
          Serial.println("ELM ready (SP0 auto)");
          obd.linkUp = true;
          elmState = ELM_READY;
        } else {
          Serial.println("ELM no bus, retry init");
          for (int i = 0; i < 8 && !screenChanged; i++) delay(100);
        }
      }
      break;
    }

    case ELM_READY: {
      // WiFi совсем упал — переподключаемся
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost -> reconnect");
        obd.linkUp = false;
        elmState = ELM_DISCONNECTED;
        break;
      }
      // TCP: клоны иногда «моргают» connected() — не рвём сразу,
      // терпим до 3 подряд неудачных проверок. Проверяем раз в 500 мс:
      // connected() — системный вызов, на горячем пути опроса он лишний.
      static uint8_t  tcpMiss = 0;
      static uint32_t tLink = 0;
      if (millis() - tLink >= 500) {
        tLink = millis();
        if (!elm.connected()) {
          if (++tcpMiss >= 3) {
            Serial.println("TCP link lost -> reconnect");
            obd.linkUp = false;
            tcpMiss = 0;
            dlogLinkLost();                  // отметить обрыв в слоте
            dlogEvent(EV_LINKLOST, 0);       // и в журнале аномалий
            elmState = ELM_DISCONNECTED;
          }
        } else {
          tcpMiss = 0;
        }
      }
      break;
    }
  }
}

// ============================================================
// ОТРИСОВКА — частичное обновление, без мельканий.
// Статика рисуется один раз при входе на экран (dirtyFull),
// числа перерисовываются только в своих зонах поверх фона.
// ============================================================
// dirtyFull объявлен выше, рядом с глобальными

// --- геометрия круглого дисплея ---
#define CX 120
#define CY 120

// --- шкала тахометра: 270°, сегментами по 250 об/мин ---
#define TACH_R_OUT   119
#define TACH_R_IN    98
#define TACH_A_START 135.0f      // градусы (0 = вправо, по часовой вниз)
#define TACH_A_SPAN  270.0f
#define RPM_MAX      7000
#define TACH_SEG_RPM 250                       // 1 сегмент = 250 об/мин
#define TACH_NSEG    (RPM_MAX / TACH_SEG_RPM)  // 28 сегментов
#define TACH_GAP_DEG 1.6f                      // зазор между сегментами, град.
#define TACH_OFF     TFT_BLACK                 // «пустой» сегмент = фон (не видно)

static uint16_t tachColor(int rpm) {
  if (rpm >= 5500) return TFT_RED;
  if (rpm >= 3000) return TFT_YELLOW;
  return TFT_GREEN;
}

// Сегмент как сплошная трапеция: две пары треугольников + заливка щелей
// дуговыми линиями. Без просветов на любом радиусе.
static void tachDrawSeg(int seg, uint16_t col) {
  float a0 = TACH_A_START + TACH_A_SPAN * seg       / (float)TACH_NSEG + TACH_GAP_DEG;
  float a1 = TACH_A_START + TACH_A_SPAN * (seg + 1) / (float)TACH_NSEG - TACH_GAP_DEG;
  float r0 = a0 * DEG_TO_RAD, r1 = a1 * DEG_TO_RAD;
  int x0i = CX + cosf(r0) * TACH_R_IN,  y0i = CY + sinf(r0) * TACH_R_IN;
  int x0o = CX + cosf(r0) * TACH_R_OUT, y0o = CY + sinf(r0) * TACH_R_OUT;
  int x1i = CX + cosf(r1) * TACH_R_IN,  y1i = CY + sinf(r1) * TACH_R_IN;
  int x1o = CX + cosf(r1) * TACH_R_OUT, y1o = CY + sinf(r1) * TACH_R_OUT;
  // трапеция = 2 треугольника
  lcd.fillTriangle(x0i, y0i, x0o, y0o, x1o, y1o, col);
  lcd.fillTriangle(x0i, y0i, x1o, y1o, x1i, y1i, col);
  // дуговая «крышка» — залить остаточную кривизну по внешнему радиусу
  for (float a = a0; a <= a1; a += 0.6f) {
    float rad = a * DEG_TO_RAD, ca = cosf(rad), sa = sinf(rad);
    lcd.drawLine(CX + ca * (TACH_R_IN + 1),  CY + sa * (TACH_R_IN + 1),
                 CX + ca * TACH_R_OUT,       CY + sa * TACH_R_OUT, col);
  }
}

// сброс кэша — при полной перерисовке экрана
static int tachShownSeg = -1;
static void tachReset() { tachShownSeg = -1; }

// нарисовать шкалу: только горящие сегменты (фон уже чёрный)
static void tachDrawAll(int active) {
  for (int s = 0; s < active; s++)
    tachDrawSeg(s, tachColor((s + 1) * TACH_SEG_RPM));
}

// инкрементально: дорисовать/погасить только изменившиеся сегменты
static void drawTach(int rpm) {
  if (rpm < 0) rpm = 0;
  if (rpm > RPM_MAX) rpm = RPM_MAX;
  int active = (rpm + TACH_SEG_RPM - 1) / TACH_SEG_RPM;
  if (active > TACH_NSEG) active = TACH_NSEG;

  if (tachShownSeg < 0) { tachDrawAll(active); tachShownSeg = active; return; }
  if (active == tachShownSeg) return;

  if (active > tachShownSeg) {
    for (int s = tachShownSeg; s < active; s++)
      tachDrawSeg(s, tachColor((s + 1) * TACH_SEG_RPM));
  } else {
    for (int s = active; s < tachShownSeg; s++)
      tachDrawSeg(s, TACH_OFF);
  }
  tachShownSeg = active;
}

static void drawTachAnim() { }   // no-op (анимация убрана)

// Значение в зоне (x,y,w,h). Перерисовывает ТОЛЬКО если текст/цвет/размер
// изменились — иначе не трогает пиксели (нет мелькания при том же значении).
// id — уникальный номер поля 0..FIELD_MAX-1.
#define FIELD_MAX 16
struct FieldCache { String s; uint16_t col; uint8_t size; bool init; };
static FieldCache fcache[FIELD_MAX];

static void fieldId(int id, int x, int y, int w, int h, const String& s,
                    uint16_t col, uint8_t size) {
  FieldCache& c = fcache[id];
  if (c.init && c.s == s && c.col == col && c.size == size) return;   // не изменилось

  bool sameLayout = c.init && c.size == size && c.s.length() == s.length();
  c.s = s; c.col = col; c.size = size; c.init = true;

  lcd.setTextDatum(middle_center);
  lcd.setTextSize(size);
  if (sameLayout) {
    // та же длина/размер — рисуем текст С ФОНОМ за один проход, без вспышки
    lcd.setTextColor(col, TFT_BLACK);
    lcd.drawString(s, x + w / 2, y + h / 2);
  } else {
    // длина или размер изменились — чистим зону и рисуем
    lcd.fillRect(x, y, w, h, TFT_BLACK);
    lcd.setTextColor(col);
    lcd.drawString(s, x + w / 2, y + h / 2);
  }
}

// сброс кэша полей — при полной перерисовке экрана
static void fieldsReset() { for (int i = 0; i < FIELD_MAX; i++) fcache[i].init = false; }

// старый вариант без кэша (для DTC и разовых мест)
static void field(int x, int y, int w, int h, const String& s,
                  uint16_t col, uint8_t size) {
  lcd.fillRect(x, y, w, h, TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(col);
  lcd.setTextSize(size);
  lcd.drawString(s, x + w / 2, y + h / 2);
}

static void label(int x, int y, const char* t) {
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_DARKGREY);
  lcd.setTextSize(1);
  lcd.drawString(t, x, y);
}

// ------------------------------------------------------------
// ЕДИНЫЙ ЭКРАН: тахо-дуга + скорость + температура + вольты + нагрузка
// ------------------------------------------------------------
void drawGaugeStatic() {
  lcd.fillScreen(TFT_BLACK);
  tachReset();               // дуга нарисуется заново при первом drawTach
  // цифры шкалы убраны: сегменты по 250 об/мин с зазорами уже дают деление,
  // а числа по краю круга задевались сегментами и рябили.
  label(120, 44, "RPM x1000");
  label(78,  137, "km/h");
  label(164, 137, "GEAR");
  label(78,  185, "TEMP");
  label(164, 185, "BATT");
  // статус связи и значок ошибок рисует drawGaugeIcons() —
  // они меняются на ходу, поэтому не в статике
}

// --- ИКОНКИ СОСТОЯНИЯ (низ экрана, внутри дуги) ---
// Слева: связь. "OBD" зелёным — подключены к адаптеру и он отвечает;
//        "WIFI" синим — сети адаптера нет, сидим на запасной сети;
//        "--" серым — связи нет вовсе.
// Справа: восклицательный знак в круге, если есть коды ошибок.
static int iconLastLink = -99;
static int iconLastErr  = -1;
static void gaugeIconsReset() { iconLastLink = -99; iconLastErr = -1; }

static void drawGaugeIcons() {
  int& lastLink = iconLastLink;
  int& lastErr  = iconLastErr;

  // 2 = OBD-адаптер отвечает, 1 = только WiFi (запасная сеть), 0 = нет связи
  int link = 0;
  if (obd.linkUp)                                  link = 2;
  else if (WiFi.status() == WL_CONNECTED)          link = netIsObd ? 2 : 1;
  int err = (dtcValid && dtcList.length() > 0) ? 1 : 0;

  if (link != lastLink) {
    lastLink = link;
    lcd.fillRect(60, 204, 84, 20, TFT_BLACK);
    lcd.setTextDatum(middle_center);
    lcd.setTextSize(1);
    if (link == 2)      { lcd.setTextColor(TFT_GREEN);    lcd.drawString("OBD",  102, 214); }
    else if (link == 1) { lcd.setTextColor(TFT_CYAN);     lcd.drawString("WIFI", 102, 214); }
    else                { lcd.setTextColor(TFT_DARKGREY); lcd.drawString("--",   102, 214); }
  }

  if (err != lastErr) {
    lastErr = err;
    lcd.fillRect(146, 202, 24, 24, TFT_BLACK);
    if (err) {                                  // (!) — есть коды ошибок
      lcd.drawCircle(157, 214, 9, TFT_RED);
      lcd.drawFastVLine(157, 209, 6, TFT_RED);
      lcd.drawPixel(157, 218, TFT_RED);
      lcd.drawPixel(158, 209, TFT_RED);         // чуть жирнее ствол
      lcd.drawFastVLine(158, 209, 6, TFT_RED);
      lcd.drawPixel(158, 218, TFT_RED);
    }
  }
}


void drawGaugeValues() {
  char b[16];

  // --- тахометр (дуга) + число оборотов (фикс. ширина 4 — без вспышки) ---
  drawTach(obd.rpm);
  if (obd.rpm >= 0) snprintf(b, sizeof(b), "%4d", obd.rpm);
  else              snprintf(b, sizeof(b), "----");
  fieldId(0, 60, 58, 120, 28, b, TFT_WHITE, 3);

  // --- скорость (слева, фикс. ширина 3) ---
  if (obd.speed >= 0) snprintf(b, sizeof(b), "%3d", obd.speed);
  else                snprintf(b, sizeof(b), "  -");
  fieldId(1, 38, 98, 80, 34, b, TFT_CYAN, 4);

  // --- передача (справа) ---
  String g; uint16_t gcol;
  if      (gearCurrent > 0)   { g = String(gearCurrent); gcol = TFT_WHITE; }
  else if (gearCalibrating)   { g = "c"; gcol = TFT_ORANGE; }
  else if (gears.count == 0)  { g = "-"; gcol = TFT_DARKGREY; }
  else                        { g = "N"; gcol = TFT_DARKGREY; }
  fieldId(2, 124, 98, 80, 34, g, gcol, 4);

  // --- температура ОЖ (слева): >95 красным крупнее, иначе зелёным ---
  bool overheat = (obd.coolant > 95);
  uint16_t tcol = (obd.coolant <= -200) ? TFT_DARKGREY
                : overheat ? TFT_RED : TFT_GREEN;
  if (obd.coolant > -200) snprintf(b, sizeof(b), "%d", obd.coolant);
  else                    snprintf(b, sizeof(b), "--");
  fieldId(3, 36, 150, 84, 30, b, tcol, overheat ? 4 : 3);

  // --- напряжение АКБ (справа) ---
  uint16_t vcol = TFT_WHITE;
  if (obd.voltage > 0) {
    if (obd.voltage < 11.8 || obd.voltage > 14.8) vcol = TFT_ORANGE;
    else if (obd.voltage < 12.2) vcol = TFT_YELLOW;
    else vcol = TFT_GREEN;
    dtostrf(obd.voltage, 0, 1, b);
  } else { strcpy(b, "--"); vcol = TFT_DARKGREY; }
  fieldId(4, 120, 150, 84, 30, b, vcol, 3);

  drawGaugeIcons();          // связь (OBD/WIFI) + значок ошибок
}

// ------------------------------------------------------------
// ЭКРАН PARAMS — настраиваемая табличка (выбор через веб-морду)
// ------------------------------------------------------------
static bool paramsStaticDrawn = false;

// 2 колонки x до 4 строк, всё в центральном квадрате (круглый экран
// режет углы). Ячейка: подпись мелко сверху, значение крупно снизу.
// Колонки по центрам X = 68 и 172, строки Y от 58 с шагом 42.
#define PCOL_L   68
#define PCOL_R   172
#define PROW_Y0  58
#define PROW_DY  42
#define PCELL_W  92

void drawParamsStatic() {
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_LIGHTGREY);
  lcd.setTextSize(1);
  lcd.drawString(String("PARAMS  v") + FW_VER, 120, 26);
  // адреса веб-морды: своя точка "OBD-Dash-XXXX" -> 192.168.4.1
  lcd.setTextColor(TFT_DARKGREY);
  lcd.drawString("AP: 192.168.4.1", 120, 216);
  if (WiFi.status() == WL_CONNECTED)
    lcd.drawString("net: " + WiFi.localIP().toString(), 120, 228);
  paramsStaticDrawn = true;
}

void drawParamsValues() {
  int rows[8], nr = 0;
  for (int i = 0; i < PID_DEFS_LEN && nr < 8; i++)
    if (pidMask & (1u << i)) rows[nr++] = i;

  for (int k = 0; k < nr; k++) {
    int i = rows[k];
    int cx = (k % 2) ? PCOL_R : PCOL_L;
    int cy = PROW_Y0 + (k / 2) * PROW_DY;

    // подпись — статична, рисуем один раз (в drawParamsStatic не рисуется,
    // т.к. набор меняется; ставим через fieldId со своим id-диапазоном)
    PidVal& v = pidVals[i];
    // значение с кэшем: id = 5 + k (0..4 занял GAUGE)
    fieldId(5 + k, cx - PCELL_W / 2, cy - 2, PCELL_W, 22,
            v.valid ? v.text : String("--"),
            v.valid ? v.color : C_GREY, 2);
  }
}

// подписи параметров (статика PARAMS) — рисуются при полной перерисовке
void drawParamsLabels() {
  int nr = 0;
  for (int i = 0; i < PID_DEFS_LEN && nr < 8; i++) {
    if (!(pidMask & (1u << i))) continue;
    int cx = (nr % 2) ? PCOL_R : PCOL_L;
    int cy = PROW_Y0 + (nr / 2) * PROW_DY;
    lcd.setTextDatum(middle_center);
    lcd.setTextColor(TFT_DARKGREY);
    lcd.setTextSize(1);
    lcd.drawString(PID_DEFS[i].label, cx, cy - 10);
    nr++;
  }
}

void drawParams() {
  if (!paramsStaticDrawn || dirtyFull) {
    drawParamsStatic();
    drawParamsLabels();      // подписи параметров (набор может меняться)
  }
  drawParamsValues();
}

// ------------------------------------------------------------
// ЭКРАН КОДОВ (по кнопке)
// ------------------------------------------------------------
void drawDtc() {
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_LIGHTGREY);
  lcd.setTextSize(1);
  lcd.drawString("TROUBLE CODES", 120, 16);

  if (!dtcValid) {
    lcd.setTextColor(TFT_DARKGREY);
    lcd.drawString("reading...", 120, 120);
    return;
  }
  if (dtcList.length() == 0) {
    lcd.setTextColor(TFT_GREEN);
    lcd.setTextSize(2);
    lcd.drawString("NO CODES", 120, 115);
    return;
  }

  int y = 42, from = 0, shown = 0;
  while (from < (int)dtcList.length() && shown < 5) {
    int nl = dtcList.indexOf('\n', from);
    String code = (nl < 0) ? dtcList.substring(from) : dtcList.substring(from, nl);
    String desc = dtcDescribe(code);

    lcd.setTextDatum(middle_center);
    lcd.setTextColor(TFT_RED);
    lcd.setTextSize(2);
    lcd.drawString(code, 120, y);
    y += 18;
    if (desc.length()) {
      lcd.setTextColor(TFT_LIGHTGREY);
      lcd.setTextSize(1);
      // перенос длинной строки на 2 части
      if (desc.length() > 24) {
        int sp = desc.lastIndexOf(' ', 24);
        if (sp < 0) sp = 24;
        lcd.drawString(desc.substring(0, sp), 120, y);       y += 12;
        lcd.drawString(desc.substring(sp + 1), 120, y);      y += 16;
      } else {
        lcd.drawString(desc, 120, y);                        y += 20;
      }
    } else {
      y += 8;
    }
    shown++;
    if (nl < 0) break;
    from = nl + 1;
  }
}

// ------------------------------------------------------------
// ЭКРАН РАЗГОНА — лучшие времена + текущий заезд
// ------------------------------------------------------------
void drawAccel() {
  if (dirtyFull) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextDatum(middle_center);
    lcd.setTextColor(TFT_LIGHTGREY);
    lcd.setTextSize(1);
    lcd.drawString("ACCEL - best", 120, 22);
  }
  for (int i = 0; i < ACCEL_NMARKS; i++) {
    int y = 58 + i * 44;
    char lab[8]; snprintf(lab, sizeof(lab), "0-%d", ACCEL_MARKS[i]);
    lcd.setTextDatum(middle_left);
    lcd.setTextColor(TFT_DARKGREY);
    lcd.setTextSize(1);
    lcd.fillRect(40, y - 8, 60, 14, TFT_BLACK);
    lcd.drawString(lab, 42, y);

    // текущий заезд активнее — оранжевым, иначе лучшее — белым
    bool live = accRunning && millis() < accShowUntil + 100;
    float t = (live && accLastT[i] > 0) ? accLastT[i] : accelBest.t[i];
    uint16_t col = (live && accLastT[i] > 0) ? TFT_ORANGE
                 : (accelBest.t[i] > 0) ? TFT_WHITE : TFT_DARKGREY;
    String s = (t > 0) ? String(t, 1) + "s" : "--";
    lcd.setTextDatum(middle_right);
    lcd.setTextColor(col);
    lcd.setTextSize(3);
    lcd.fillRect(110, y - 14, 96, 28, TFT_BLACK);
    lcd.drawString(s, 200, y);
  }
  // статус
  lcd.setTextDatum(middle_center);
  lcd.setTextSize(1);
  lcd.fillRect(40, 196, 160, 14, TFT_BLACK);
  if (accRunning)      { lcd.setTextColor(TFT_ORANGE); lcd.drawString("RUN...", 120, 202); }
  else                 { lcd.setTextColor(TFT_DARKGREY); lcd.drawString("start from 0", 120, 202); }
}

// ------------------------------------------------------------
// ОВЕРЛЕЙ ПРЕДУПРЕЖДЕНИЯ — поверх любого экрана
// ------------------------------------------------------------
void drawAlertOverlay(AlertKind k) {
  static AlertKind lastK = AL_NONE;
  static bool blink = false;
  blink = !blink;
  uint16_t bg = blink ? TFT_RED : 0x8000;   // мигание красный/тёмно-красный
  lcd.fillScreen(bg);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(1);
  lcd.drawString("!! WARNING !!", 120, 88);
  lcd.setTextSize(3);
  lcd.drawString(alertText(k), 120, 122);
  // поясняющее значение
  char b[24];
  lcd.setTextSize(2);
  switch (k) {
    case AL_OVERHEAT: snprintf(b, sizeof(b), "%d C", obd.coolant); break;
    case AL_LOWVOLT:  dtostrf(obd.voltage, 0, 1, b); strcat(b, " V"); break;
    case AL_REDLINE:  snprintf(b, sizeof(b), "%d rpm", obd.rpm); break;
    default: b[0] = 0;
  }
  if (b[0]) lcd.drawString(b, 120, 156);
  lastK = k;
}

// ============================================================
void drawScreen(Screen s) {
  // dirtyFull = требуется полная перерисовка (смена экрана / веб-сохранение).
  if (dirtyFull) {
    lcd.fillScreen(TFT_BLACK);
    tachReset();
    paramsStaticDrawn = false;
    fieldsReset();
    gaugeIconsReset();
  }
  switch (s) {
    case SCREEN_GAUGE:
      if (dirtyFull) drawGaugeStatic();
      drawGaugeValues();
      break;
    case SCREEN_PARAMS:
      drawParams();
      break;
    case SCREEN_ACCEL:
      drawAccel();
      break;
    case SCREEN_DTC:
      drawDtc();
      break;
  }
  dirtyFull = false;
}

// Только меняем состояние; тяжёлую отрисовку сделает loop() —
// так обработка нажатия мгновенная и очередь тапов не копится.
void gotoScreen(Screen s) {
  currentScreen = s;
  dirtyFull = true;
  screenChanged = true;                     // прервать активный OBD-запрос
  if (s == SCREEN_DTC) dtcValid = false;    // перечитать коды при входе
}

void nextScreen() { gotoScreen((Screen)((currentScreen + 1) % SCREEN_COUNT)); }

// ============================================================
// КНОПКА (сенсорная TTP223, активна BTN_ACTIVE) — ПРИОРИТЕТНАЯ
// ============================================================
// Таймер-ISR каждые 5 мс: ловит касание И СРАЗУ переключает currentScreen
// (это просто присваивание — безопасно из ISR). Флаг screenChanged
// прерывает любой висящий OBD-запрос. Рисование делает loop в первой же
// свободной точке (LovyanGFX из ISR звать нельзя).
bool btnRaw() { return digitalRead(BTN_PIN) == BTN_ACTIVE; }

volatile uint8_t btnQueue     = 0;   // 1 = короткое нажатие ждёт обработки
volatile uint8_t btnHoldQueue = 0;   // 1 = долгое удержание (режим настройки)

// Таймер 1 мс. Нажатие = HIGH держится непрерывно >= 40 мс.
// Следующее принимается только после LOW >= 40 мс (защита от повторов).
// Удержание >= BTN_SETUP_MS непрерывно -> вход в режим настройки (AP + web).
#define BTN_HOLD_MS    40
#define BTN_RELEASE_MS 40
#define BTN_SETUP_MS   1800
void IRAM_ATTR btnTick() {
  static uint16_t highMs = 0;
  static uint16_t lowMs  = BTN_RELEASE_MS;
  static bool     armed  = true;
  static bool     holdFired = false;

  if (btnRaw()) {
    lowMs = 0;
    if (highMs < 60000) highMs++;
    if (armed && highMs >= BTN_HOLD_MS) {
      btnQueue = 1;
      screenChanged = true;
      armed = false;
    }
    if (!holdFired && highMs >= BTN_SETUP_MS) {
      btnHoldQueue = 1;              // держим долго -> setup-режим
      holdFired = true;
    }
  } else {
    highMs = 0;
    holdFired = false;
    if (lowMs < 60000) lowMs++;
    if (lowMs >= BTN_RELEASE_MS) armed = true;
  }
}

hw_timer_t* btnTimer = nullptr;

void btnBegin() {
  pinMode(BTN_PIN, INPUT);
  btnTimer = timerBegin(0, 80, true);          // 1 МГц
  timerAttachInterrupt(btnTimer, &btnTick, true);
  timerAlarmWrite(btnTimer, 1000, true);       // 1 мс — счётчики highMs/lowMs в мс
  timerAlarmEnable(btnTimer);
}

// РЕЖИМ НАСТРОЙКИ: глушим STA, поднимаем чистую AP на канале 1, крутим
// только веб-сервер. Вход и выход — долгим удержанием кнопки. Выход НЕ
// перезагружает плату: восстанавливает AP_STA, переподключает STA и
// возвращает управление в loop() (дашборд).
// ESP32-C3 не держит AP+STA на разных каналах, поэтому в машине (STA к адаптеру)
// своя точка «уезжает» за каналом адаптера и телефон её теряет. Здесь STA выключен —
// AP стабильно на канале 1.
void enterSetupMode() {
  Serial.println("=== SETUP MODE (hold) ===");
  const char* ssid = apSsidGlobal;

  elm.stop();
  elmState = ELM_DISCONNECTED;
  obd.linkUp = false;
  WiFi.disconnect(true, false);        // рвём STA, настройки сети сохраняем
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  bool apok = WiFi.softAP(ssid, AP_PASS, 1);   // закрытая сеть
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("SETUP AP: %s ch1 %s  http://%s/\n",
                ssid, apok ? "OK" : "FAIL", ip.toString().c_str());

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_YELLOW);
  lcd.setTextSize(2);
  lcd.drawString("SETUP MODE", CX, CY - 44);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE);
  lcd.drawString("WiFi:", CX, CY - 8);
  lcd.setTextColor(TFT_CYAN);
  lcd.drawString(ssid, CX, CY + 8);
  lcd.setTextColor(TFT_WHITE);
  lcd.drawString(String("pass: ") + AP_PASS, CX, CY + 26);
  lcd.drawString(String("http://") + ip.toString() + "/", CX, CY + 44);
  lcd.setTextColor(TFT_DARKGREY);
  lcd.drawString("hold btn to exit", CX, CY + 66);

  btnHoldQueue = 0;                    // сбросить событие входа
  btnQueue = 0;
  uint32_t t0 = millis();
  for (;;) {
    web.handleClient();
    // выход из setup — снова долгое удержание. Игнорируем первые 1.5 с,
    // чтобы «дожатие» кнопки при входе не выкинуло сразу обратно.
    if (btnHoldQueue && millis() - t0 > 1500) {
      btnHoldQueue = 0;
      break;
    }
    delay(2);
  }

  // --- выход: восстановить рабочий режим без рестарта ---
  Serial.println("=== SETUP MODE exit (hold) ===");
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(1);
  lcd.drawString("reconnecting...", CX, CY);

  // AP гасим: в рабочем режиме она не нужна и мешает опросу адаптера
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  if (netConnectBest() < 0) {          // по приоритету: сперва сеть OBD
    Serial.println("сеть не найдена — ретрай в фоне");
  }

  elmState = ELM_DISCONNECTED;         // elmService переподключится сам
  btnQueue = 0; btnHoldQueue = 0;
  gotoScreen(currentScreen);           // перерисовать дашборд с нуля
}

// вызывается из loop() — выполняет переключение, если ISR засёк нажатие
void handleButton() {
  if (btnHoldQueue) { btnHoldQueue = 0; enterSetupMode(); return; }  // вход/выход по удержанию
  if (btnQueue == 0) return;
  btnQueue = 0;
  Serial.println("BTN tap -> next");
  currentScreen = (Screen)((currentScreen + 1) % SCREEN_COUNT);
  dirtyFull = true;
  if (currentScreen == SCREEN_DTC) dtcValid = false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 OBD DASH (no-touch) ===");

  btnBegin();                // опрос кнопки по таймеру 5 мс

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  bool disp = lcd.init();
  Serial.printf("lcd.init() -> %d\n", disp);
  lcd.setRotation(2);   // экран перевёрнут на 180°
  lcd.setBrightness(255);

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(2);
  lcd.drawString("OBD Dash", 120, 92);
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_DARKGREY);
  lcd.drawString(String("v") + FW_VER, 120, 116);
  lcd.drawString(String("rev ") + FW_REV, 120, 132);
  lcd.setTextColor(TFT_WHITE);
  lcd.drawString("starting...", 120, 152);
  delay(300);

  // держишь кнопку при включении ~1.5 c -> портал
  bool forcePortal = false;
  if (btnRaw()) {
    lcd.fillScreen(TFT_BLACK);
    lcd.setTextColor(TFT_YELLOW);
    lcd.setTextSize(1);
    lcd.drawString("hold for setup...", 120, 120);
    uint32_t t0 = millis();
    while (btnRaw() && millis() - t0 < 2000) delay(50);
    if (millis() - t0 >= 1500) forcePortal = true;
  }

  loadPidMask();
  gearLoad(prefs);
  alertLoad(prefs);
  accelLoad(prefs);
  dlogLoad(prefs);            // лог замеров (переживает перезагрузки)
  netLoad(prefs);             // список WiFi-сетей с приоритетом

  // Своя точка доступа в обычной работе НЕ поднимается: у ESP32-C3 одно
  // радио на AP и STA, и AP отбирает эфирное время у опроса адаптера.
  // Она нужна только для настройки — включается в setup-режиме
  // (долгое удержание кнопки).
  uint64_t cid = ESP.getEfuseMac();
  snprintf(apSsidGlobal, sizeof(apSsidGlobal), "OBD-Dash-%04X", (uint16_t)(cid & 0xFFFF));
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  Serial.printf("AP выключена (настройка: удержать кнопку). SSID=%s\n", apSsidGlobal);

  connectWiFi(forcePortal);     // подключение к сети адаптера (STA)

  webBegin();

  gotoScreen(SCREEN_GAUGE);
  Serial.println("=== READY ===");
}

// ============================================================
// LOOP
// ============================================================
// Опрос: RPM+скорость почти каждый цикл (быстрая стрелка тахо),
// медленные параметры (темп/вольты/нагрузка/дроссель) по очереди
// вперемешку с быстрыми — так экран обновляется плавно, без пауз.
void loop() {
  static uint32_t tPoll = 0, tRedraw = 0, tDtc = 0, tAlert = 0;

  static Screen   drawnScreen = (Screen)255;
  static bool     alertOn = false;

  handleButton();

  // --- ЛОГ ЗАМЕРОВ: закрыть слот и сохранить в NVS ---
  if (dlogTick()) dlogSave(prefs);

  // Состояние радио в лог: поднята ли своя AP и сколько на ней клиентов.
  // Нужно, чтобы по логу проверить, мешает ли AP опросу адаптера
  // (у ESP32-C3 одно радио на AP и STA).
  {
    static uint32_t tRadio = 0;
    if (millis() - tRadio >= 1000) {
      tRadio = millis();
      wifi_mode_t m = WiFi.getMode();
      bool apUp = (m == WIFI_AP || m == WIFI_AP_STA);
      dlogRadio(apUp, apUp ? WiFi.softAPgetStationNum() : 0, WiFi.RSSI());
    }
  }


  // Дома по USB: отправить 'L' в Serial -> дамп лога, 'C' -> очистить.
  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'L' || c == 'l') dlogDump();
    if (c == 'C' || c == 'c') { dlogClear(prefs); Serial.println("log cleared"); }
    if (c == 'N' || c == 'n') {          // показать сохранённые сети
      Serial.printf("nets: count=%u\n", nets.count);
      for (int i = 0; i < NET_MAX; i++)
        Serial.printf("  [%d] ssid='%s' pass=%s\n", i, nets.net[i].ssid,
                      nets.net[i].pass[0] ? "есть" : "(нет)");
      Serial.printf("STA status=%d SSID='%s'\n", WiFi.status(), WiFi.SSID().c_str());
    }
    if (c == 'S' || c == 's') {          // скан эфира
      int n = WiFi.scanNetworks();
      Serial.printf("видно %d сетей:\n", n);
      for (int i = 0; i < n; i++)
        Serial.printf("  %s  %d dBm  ch%d\n", WiFi.SSID(i).c_str(),
                      WiFi.RSSI(i), WiFi.channel(i));
      WiFi.scanDelete();
    }
  }

  // --- ОВЕРЛЕЙ ПРЕДУПРЕЖДЕНИЯ (приоритет над всем) ---
  AlertKind al = alertCheck(obd.rpm, obd.coolant, obd.voltage, dtcNewFlag);
  if (al != AL_NONE) {
    dtcNewFlag = false;                     // приняли к сведению
    alertOn = true;
    // записать аномалию в журнал (дубликаты за 30 с отсекаются внутри)
    switch (al) {
      case AL_OVERHEAT: dlogEvent(EV_OVERHEAT, obd.coolant); break;
      case AL_LOWVOLT:  dlogEvent(EV_LOWVOLT, (int16_t)(obd.voltage * 10)); break;
      case AL_REDLINE:  dlogEvent(EV_REDLINE, obd.rpm); break;
      case AL_NEWDTC:   dlogEvent(EV_DTC, 0); break;
      default: break;
    }
    if (millis() - tAlert >= 400) {         // мигание 2.5 Гц
      tAlert = millis();
      drawAlertOverlay(al);
    }
    // всё равно опрашиваем OBD, чтобы отследить снятие тревоги
    web.handleClient();
    elmService();
    if (elmState == ELM_READY && millis() - tPoll >= 150) {
      tPoll = millis();
      pollFast();
      pollSlowStep();
    }
    return;                                 // ничего больше не рисуем
  }
  if (alertOn) { alertOn = false; dirtyFull = true; drawnScreen = (Screen)255; }  // тревога снята — вернуть экран

  // перерисовать экран, если сменился
  if (currentScreen != drawnScreen || dirtyFull) {
    Screen s = currentScreen;
    if (s != drawnScreen) dirtyFull = true;
    drawnScreen = s;
    screenChanged = false;
    drawScreen(s);
  }
  screenChanged = false;

  // Веб-сервер не на горячем пути опроса: 20 Гц человеку незаметно,
  // но освобождает время между OBD-запросами. В setup-режиме крутится отдельно.
  static uint32_t tWeb = 0;
  if (millis() - tWeb >= 50) { tWeb = millis(); web.handleClient(); }
  elmService();
  bool ready = demoOn || (elmState == ELM_READY);

  // --- ДЕМО: гоним данные без адаптера на максимальной частоте ---
  if (demoOn) {
    static uint32_t demoFps = 0, demoCnt = 0, demoT = 0;
    pollDemo();
    if (currentScreen == SCREEN_GAUGE)  drawGaugeValues();
    if (currentScreen == SCREEN_ACCEL)  { accelUpdate(obd.speed); drawAccel(); }
    if (currentScreen == SCREEN_PARAMS) drawParamsValues();
    // счётчик FPS в Serial раз в секунду
    demoCnt++;
    if (millis() - demoT >= 1000) {
      Serial.printf("DEMO fps=%lu\n", demoCnt);
      demoCnt = 0; demoT = millis();
    }
    return;
  }

  // --- GAUGE: RPM и скорость чередуются 2:1, медленное 1/1.2с ---
  // Скорость нужна не только для цифры, но и для авто-калибровки передач,
  // поэтому тянем её почаще: каждый 2-й опрос + гарантированно раз в 400 мс.
  if (currentScreen == SCREEN_GAUGE) {
    static uint32_t tSlow = 0, tSpd = 0;
    static uint8_t  fastCnt = 0;
    if (ready && millis() - tPoll >= 10) {
      tPoll = millis();
      pollRpm();                             // приоритет — тахо
      if (++fastCnt >= 2 || millis() - tSpd >= 400) {
        fastCnt = 0; tSpd = millis();
        pollSpeed();
      }
      if (millis() - tSlow >= 1200) { tSlow = millis(); pollSlowStep(); }
      // Фоновое чтение кодов раз в 10 минут — только чтобы знать, зажигать ли
      // значок (!) на главном. Запрос "03" небыстрый, потому так редко.
      static uint32_t tDtcBg = 0;
      if (millis() - tDtcBg >= 600000) { tDtcBg = millis(); pollDtc(); }
      if (gearUpdate(obd.rpm, obd.speed, obd.throttle)) gearSave(prefs);
      if (accelUpdate(obd.speed)) accelSave(prefs);
      drawGaugeValues();
    } else {
      // между опросами — гоним интерполяцию тахо-дуги для плавности
      static uint32_t tAnim = 0;
      if (millis() - tAnim >= 30) { tAnim = millis(); drawTachAnim(); }
      if (!ready && millis() - tRedraw >= 300) { tRedraw = millis(); drawGaugeValues(); }
    }
  }

  // --- ACCEL: только скорость, максимально часто ---
  else if (currentScreen == SCREEN_ACCEL) {
    if (ready && millis() - tPoll >= 20) {
      tPoll = millis();
      pollRpmSpeed();
      if (accelUpdate(obd.speed)) accelSave(prefs);
    }
    if (millis() - tRedraw >= 120) { tRedraw = millis(); drawAccel(); }
  }

  // --- PARAMS ---
  else if (currentScreen == SCREEN_PARAMS) {
    if (ready && millis() - tPoll >= 20) {
      tPoll = millis();
      pollParamsStep();
      drawParamsValues();
    }
  }

  // --- DTC ---
  else if (currentScreen == SCREEN_DTC) {
    if (ready && (!dtcValid || millis() - tDtc >= 15000)) {
      tDtc = millis();
      pollDtc();
      drawScreen(SCREEN_DTC);
    }
  }
}
