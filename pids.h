#pragma once
#include <Arduino.h>

// ============================================================
// Каталог параметров для настраиваемого экрана PARAMS.
// Каждый параметр: короткое имя, PID (mode 01), функция-декодер,
// единица, функция цвета (аномалии). Выбор — битовая маска в NVS.
// ============================================================

// сырые байты ответа PID (после mode+pid), их количество
struct PidRaw { uint8_t b[6]; int n; };

// значение параметра: число + строка для показа + цвет
struct PidVal { float v; bool valid; String text; uint16_t color; };

typedef PidVal (*PidDecoder)(const PidRaw&);

struct PidDef {
  const char* key;      // короткий ключ (для NVS и веб-морды)
  const char* label;    // подпись на экране (<=8 симв)
  uint8_t     mode;     // 0x01 обычно; 0xFF = спец (ATRV)
  uint8_t     pid;      // код
  PidDecoder  decode;
};

// ---- палитра ----
#define C_WHITE  0xFFFF
#define C_GREEN  0x07E0
#define C_YELLOW 0xFFE0
#define C_ORANGE 0xFD20
#define C_RED    0xF800
#define C_CYAN   0x07FF
#define C_GREY   0x8410

static PidVal mk(float v, const String& t, uint16_t c) { return {v, true, t, c}; }
static PidVal bad() { return {0, false, "--", C_GREY}; }

// ---------- декодеры ----------
static PidVal d_rpm(const PidRaw& r) {
  if (r.n < 2) return bad();
  float v = ((r.b[0] << 8) | r.b[1]) / 4.0f;
  return mk(v, String((int)v), C_WHITE);
}
static PidVal d_speed(const PidRaw& r) {
  if (r.n < 1) return bad();
  return mk(r.b[0], String(r.b[0]), C_CYAN);
}
static PidVal d_coolant(const PidRaw& r) {
  if (r.n < 1) return bad();
  int t = r.b[0] - 40;
  uint16_t c = (t > 105) ? C_RED : (t > 95) ? C_ORANGE : (t < 60) ? C_CYAN : C_GREEN;
  return mk(t, String(t) + "C", c);
}
static PidVal d_iat(const PidRaw& r) {
  if (r.n < 1) return bad();
  int t = r.b[0] - 40;
  uint16_t c = (t > 70) ? C_ORANGE : C_WHITE;
  return mk(t, String(t) + "C", c);
}
static PidVal d_load(const PidRaw& r) {
  if (r.n < 1) return bad();
  int p = r.b[0] * 100 / 255;
  return mk(p, String(p) + "%", C_WHITE);
}
static PidVal d_thr(const PidRaw& r) {
  if (r.n < 1) return bad();
  int p = r.b[0] * 100 / 255;
  return mk(p, String(p) + "%", C_WHITE);
}
static PidVal d_timing(const PidRaw& r) {          // угол опережения
  if (r.n < 1) return bad();
  float a = r.b[0] / 2.0f - 64.0f;
  return mk(a, String(a, 0) + "\xF8", C_WHITE);    // \xF8 = знак градуса
}
static PidVal d_map(const PidRaw& r) {             // давление коллектора, кПа
  if (r.n < 1) return bad();
  return mk(r.b[0], String(r.b[0]) + "kPa", C_WHITE);
}
static PidVal d_maf(const PidRaw& r) {             // расход воздуха, г/с
  if (r.n < 2) return bad();
  float g = ((r.b[0] << 8) | r.b[1]) / 100.0f;
  return mk(g, String(g, 1) + "g/s", C_WHITE);
}
static PidVal d_stft(const PidRaw& r) {            // краткосрочная коррекция, %
  if (r.n < 1) return bad();
  float p = r.b[0] * 100.0f / 128.0f - 100.0f;
  uint16_t c = (fabs(p) > 15) ? C_RED : (fabs(p) > 8) ? C_YELLOW : C_GREEN;
  return mk(p, String(p, 1) + "%", c);
}
static PidVal d_ltft(const PidRaw& r) {            // долгосрочная коррекция, %
  if (r.n < 1) return bad();
  float p = r.b[0] * 100.0f / 128.0f - 100.0f;
  uint16_t c = (fabs(p) > 12) ? C_RED : (fabs(p) > 7) ? C_YELLOW : C_GREEN;
  return mk(p, String(p, 1) + "%", c);
}
static PidVal d_ecuv(const PidRaw& r) {            // напряжение ЭБУ, В (PID 0142)
  if (r.n < 2) return bad();
  float v = ((r.b[0] << 8) | r.b[1]) / 1000.0f;
  uint16_t c = (v < 11.8 || v > 14.8) ? C_ORANGE : (v < 12.2) ? C_YELLOW : C_GREEN;
  return mk(v, String(v, 1) + "V", c);
}
static PidVal d_milkm(const PidRaw& r) {           // пробег с лампой MIL, км
  if (r.n < 2) return bad();
  int km = (r.b[0] << 8) | r.b[1];
  return mk(km, String(km) + "km", km > 0 ? C_ORANGE : C_GREEN);
}
static PidVal d_clrkm(const PidRaw& r) {           // пробег с очистки кодов, км
  if (r.n < 2) return bad();
  int km = (r.b[0] << 8) | r.b[1];
  return mk(km, String(km) + "km", C_WHITE);
}

// ============================================================
// Таблица параметров. Порядок = порядок битов в маске NVS.
// ============================================================
static const PidDef PID_DEFS[] = {
  {"rpm",    "RPM",    0x01, 0x0C, d_rpm},
  {"speed",  "SPEED",  0x01, 0x0D, d_speed},
  {"cool",   "COOLANT",0x01, 0x05, d_coolant},
  {"iat",    "IN-AIR", 0x01, 0x0F, d_iat},
  {"load",   "LOAD",   0x01, 0x04, d_load},
  {"thr",    "THROT",  0x01, 0x11, d_thr},
  {"timing", "TIMING", 0x01, 0x0E, d_timing},
  {"map",    "MAP",    0x01, 0x0B, d_map},
  {"maf",    "MAF",    0x01, 0x10, d_maf},
  {"stft",   "STFT",   0x01, 0x06, d_stft},
  {"ltft",   "LTFT",   0x01, 0x07, d_ltft},
  {"ecuv",   "ECU V",  0x01, 0x42, d_ecuv},
  {"milkm",  "MIL km", 0x01, 0x21, d_milkm},
  {"clrkm",  "CLR km", 0x01, 0x31, d_clrkm},
};
static const int PID_DEFS_LEN = sizeof(PID_DEFS) / sizeof(PID_DEFS[0]);

// маска по умолчанию: RPM, SPEED, COOLANT, LOAD, THROT, TIMING (набор «Работа ДВС»)
static const uint32_t PID_MASK_DEFAULT =
  (1u << 0) | (1u << 1) | (1u << 2) | (1u << 4) | (1u << 5) | (1u << 6);
