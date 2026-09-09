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
#include "dtc_db.h"
#include "pids.h"
#include "gear.h"
#include "alerts.h"
#include "accel.h"

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

String elmCmd(const String& cmd, uint32_t timeoutMs = 800) {
  while (elm.available()) elm.read();
  elm.print(cmd);
  elm.print('\r');
  String resp;
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    while (elm.available()) {
      char c = elm.read();
      if (c == '>') {
        resp.replace(cmd, "");
        resp.replace("\r", " ");
        resp.replace("\n", " ");
        resp.trim();
        return resp;
      }
      resp += c;
      t0 = millis();
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

// --- ГЛАВНЫЙ ЭКРАН: один мультизапрос на все 5 параметров ---
// RPM(0C) SPEED(0D) COOLANT(05) LOAD(04) THROTTLE(11)
void pollGauge() {
  static const uint8_t P[5] = { 0x0C, 0x0D, 0x05, 0x04, 0x11 };
  uint8_t b[8]; int c;

  if (lastMultiOk) {
    String r = queryMulti(P, 5);
    if (!r.isEmpty() && r.indexOf("41") >= 0) {
      // вытащить каждый PID из общего ответа
      c = parsePid(r, 0x01, 0x0C, b, 8);
      if (c >= 2) { obd.rpm = ((b[0] << 8) | b[1]) / 4; missRpm = 0; } else bumpMiss(missRpm, obd.rpm, -1);
      c = parsePid(r, 0x01, 0x0D, b, 8);
      if (c >= 1) { obd.speed = b[0]; missSpd = 0; } else bumpMiss(missSpd, obd.speed, -1);
      c = parsePid(r, 0x01, 0x05, b, 8);
      if (c >= 1) { obd.coolant = b[0] - 40; missCool = 0; } else bumpMiss(missCool, obd.coolant, -999);
      c = parsePid(r, 0x01, 0x04, b, 8);
      if (c >= 1) { obd.load = b[0] * 100 / 255; missLoad = 0; } else bumpMiss(missLoad, obd.load, -1);
      c = parsePid(r, 0x01, 0x11, b, 8);
      if (c >= 1) { obd.throttle = b[0] * 100 / 255; missThr = 0; } else bumpMiss(missThr, obd.throttle, -1);
      return;
    }
    lastMultiOk = false;          // клон не понял мультизапрос — дальше поштучно
    Serial.println("multi-PID not supported, fallback");
  }

  // fallback: поштучный опрос
  if (queryPid01(0x0C, b, 8, c) && c >= 2) { obd.rpm = ((b[0] << 8) | b[1]) / 4; missRpm = 0; } else bumpMiss(missRpm, obd.rpm, -1);
  if (queryPid01(0x0D, b, 8, c) && c >= 1) { obd.speed = b[0]; missSpd = 0; } else bumpMiss(missSpd, obd.speed, -1);
  if (queryPid01(0x05, b, 8, c) && c >= 1) { obd.coolant = b[0] - 40; missCool = 0; } else bumpMiss(missCool, obd.coolant, -999);
  if (queryPid01(0x04, b, 8, c) && c >= 1) { obd.load = b[0] * 100 / 255; missLoad = 0; } else bumpMiss(missLoad, obd.load, -1);
  if (queryPid01(0x11, b, 8, c) && c >= 1) { obd.throttle = b[0] * 100 / 255; missThr = 0; } else bumpMiss(missThr, obd.throttle, -1);
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
               "</style><h1>OBD Dash — параметры экрана</h1><form method=POST action=/save>");
  for (int i = 0; i < PID_DEFS_LEN; i++) {
    bool on = pidMask & (1u << i);
    h += "<label><input type=checkbox name=p" + String(i) + (on ? " checked>" : ">");
    h += String(PID_DEFS[i].label) + " <span style=color:#888>(" + PID_DEFS[i].key + ")</span></label>";
  }
  h += F("<button type=submit>Сохранить</button></form>");

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
      F("<meta http-equiv=refresh content='15;url=/'>Качаю прошивку с GitHub... "
        "плата перезагрузится через ~15 сек, если всё ок. Смотри Serial при отладке."));
    delay(300);

    WiFiClientSecure sec;
    sec.setInsecure();               // без проверки cert — проще, для домашнего проекта ок
    HTTPClient http;
    http.begin(sec, FW_URL);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // GitHub редиректит на CDN
    int code = http.GET();
    Serial.printf("OTA-URL HTTP %d\n", code);
    if (code != HTTP_CODE_OK) { http.end(); return; }

    int len = http.getSize();
    if (!Update.begin(len > 0 ? len : UPDATE_SIZE_UNKNOWN)) { Update.printError(Serial); http.end(); return; }
    size_t written = Update.writeStream(http.getStream());
    Serial.printf("OTA-URL written %u / %d\n", written, len);
    if (Update.end(true) && !Update.hasError()) {
      Serial.println("OTA-URL OK, restart");
      http.end();
      delay(300);
      ESP.restart();
    } else {
      Update.printError(Serial);
      http.end();
    }
  });
  web.on("/ota", HTTP_POST,
    []() {   // финал
      bool ok = !Update.hasError();
      web.send(200, "text/html; charset=utf-8",
        ok ? F("<meta http-equiv=refresh content='4;url=/'>OK, перезагрузка...")
           : F("ОШИБКА прошивки. Плата не тронута."));
      delay(400);
      if (ok) ESP.restart();
    },
    []() {   // приём кусками
      HTTPUpload& up = web.upload();
      if (up.status == UPLOAD_FILE_START) {
        Serial.printf("OTA start: %s\n", up.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
      } else if (up.status == UPLOAD_FILE_WRITE) {
        if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
      } else if (up.status == UPLOAD_FILE_END) {
        if (Update.end(true)) Serial.printf("OTA done: %u bytes\n", up.totalSize);
        else Update.printError(Serial);
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
    WiFi.begin();                     // подключение к последней сохранённой STA
    Serial.print("WiFi connecting");
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(200); Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED)
      Serial.printf("WiFi OK  SSID=%s  IP=%s\n",
                    WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
    else
      Serial.println("WiFi not connected — работаем без сети, ретрай в фоне");
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
          Serial.println("TCP to ELM327 OK");
          elmState = ELM_INIT;
        } else {
          tRetry = millis();
          elmState = ELM_DISCONNECTED;   // пауза 1.5 c до следующей попытки
        }
      } else if (millis() - tMark > 15000) {
        Serial.println("WiFi lost, reconnecting");
        WiFi.reconnect();
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

    case ELM_READY:
      if (!elm.connected() || WiFi.status() != WL_CONNECTED) {
        Serial.println("Link lost");
        obd.linkUp = false;
        elmState = ELM_DISCONNECTED;
      }
      break;
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
  int active = (rpm + TACH_SEG_RPM - 1) / TACH_SEG_RPM;   // сколько сегментов «горит»
  if (active > TACH_NSEG) active = TACH_NSEG;

  if (tachShownSeg < 0) { tachDrawAll(active); tachShownSeg = active; return; }
  if (active == tachShownSeg) return;

  if (active > tachShownSeg) {
    for (int s = tachShownSeg; s < active; s++)
      tachDrawSeg(s, tachColor((s + 1) * TACH_SEG_RPM));
  } else {
    for (int s = active; s < tachShownSeg; s++)
      tachDrawSeg(s, TACH_OFF);              // упали — гасим хвост
  }
  tachShownSeg = active;
}

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
  // индикатор связи — по центру снизу, внутри дуги
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(obd.linkUp ? TFT_GREEN : TFT_RED);
  lcd.setTextSize(1);
  lcd.drawString(obd.linkUp ? "OBD OK" : "OBD --", 120, 214);
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
  lcd.drawString("PARAMS", 120, 26);
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

volatile uint8_t btnQueue = 0;   // 1 = нажатие ждёт обработки

// Таймер 1 мс. Нажатие = HIGH держится непрерывно >= 40 мс.
// Следующее принимается только после LOW >= 40 мс (защита от повторов).
#define BTN_HOLD_MS    40
#define BTN_RELEASE_MS 40
void IRAM_ATTR btnTick() {
  static uint16_t highMs = 0;
  static uint16_t lowMs  = BTN_RELEASE_MS;
  static bool     armed  = true;

  if (btnRaw()) {
    lowMs = 0;
    if (highMs < 60000) highMs++;
    if (armed && highMs >= BTN_HOLD_MS) {
      btnQueue = 1;
      screenChanged = true;
      armed = false;
    }
  } else {
    highMs = 0;
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

// вызывается из loop() — выполняет переключение, если ISR засёк нажатие
void handleButton() {
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
  lcd.setRotation(0);
  lcd.setBrightness(255);

  lcd.fillScreen(TFT_BLACK);
  lcd.setTextDatum(middle_center);
  lcd.setTextColor(TFT_WHITE);
  lcd.setTextSize(2);
  lcd.drawString("OBD Dash", 120, 100);
  lcd.setTextSize(1);
  lcd.drawString("starting...", 120, 125);
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

  // --- СНАЧАЛА поднимаем свою AP (фикс. канал 1), потом STA к адаптеру ---
  // Так AP не «прыгает» за каналом STA и телефон её видит.
  uint64_t cid = ESP.getEfuseMac();
  char apSsid[32];
  snprintf(apSsid, sizeof(apSsid), "OBD-Dash-%04X", (uint16_t)(cid & 0xFFFF));
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);                            // sleep ломает видимость AP
  bool apok = WiFi.softAP(apSsid, nullptr, 1);     // открытая, канал 1
  Serial.printf("Config AP: %s ch1 %s  http://%s/\n",
                apSsid, apok ? "OK" : "FAIL",
                WiFi.softAPIP().toString().c_str());

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
  static uint8_t  slowIdx = 0;
  static Screen   drawnScreen = (Screen)255;
  static bool     alertOn = false;

  handleButton();

  // --- ОВЕРЛЕЙ ПРЕДУПРЕЖДЕНИЯ (приоритет над всем) ---
  AlertKind al = alertCheck(obd.rpm, obd.coolant, obd.voltage, dtcNewFlag);
  if (al != AL_NONE) {
    dtcNewFlag = false;                     // приняли к сведению
    alertOn = true;
    if (millis() - tAlert >= 400) {         // мигание 2.5 Гц
      tAlert = millis();
      drawAlertOverlay(al);
    }
    // всё равно опрашиваем OBD, чтобы отследить снятие тревоги
    web.handleClient();
    elmService();
    if (elmState == ELM_READY && millis() - tPoll >= 150) {
      tPoll = millis();
      pollGauge();
      if (++slowIdx >= 4) { pollVoltage(); slowIdx = 0; }   // ATRV изредка
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

  web.handleClient();
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

  // --- GAUGE: мультизапрос как можно чаще, перерисовка СРАЗУ после ---
  if (currentScreen == SCREEN_GAUGE) {
    if (ready && millis() - tPoll >= 20) {   // почти без паузы — упираемся в адаптер
      tPoll = millis();
      pollGauge();
      if (++slowIdx >= 6) { pollVoltage(); slowIdx = 0; }
      if (gearUpdate(obd.rpm, obd.speed, obd.throttle)) gearSave(prefs);
      if (accelUpdate(obd.speed)) accelSave(prefs);
      drawGaugeValues();                     // рисуем со свежими данными
    } else if (!ready && millis() - tRedraw >= 300) {
      tRedraw = millis();
      drawGaugeValues();                     // без адаптера — просто освежаем "--"
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
