#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>

// ============================================================
// СПИСОК WiFi-СЕТЕЙ С ПРИОРИТЕТОМ.
// Сеть №0 — сеть OBD-адаптера, у неё абсолютный приоритет: если она
// в эфире, подключаемся только к ней. Если её нет — идём по списку
// сохранённых домашних/рабочих сетей (для OTA, веб-морды, отладки).
//
// Хранение: одна запись в NVS ("obd"/"nets"), фиксированный размер —
// без динамики и фрагментации кучи.
// ============================================================

#define NET_MAX      4       // сколько сетей помним всего (вкл. OBD)
#define NET_SSID_LEN 33
#define NET_PASS_LEN 33

struct NetEntry {
  char ssid[NET_SSID_LEN];
  char pass[NET_PASS_LEN];
};
struct NetList {
  uint8_t   magic;           // 0x7E = валидно
  uint8_t   count;
  NetEntry  net[NET_MAX];
};
static NetList nets = { 0x7E, 0, {} };

// какая сеть сейчас используется: 0 = OBD, >0 = запасная, -1 = нет связи
static int  netActive = -1;
static bool netIsObd  = false;

inline void netLoad(Preferences& p) {
  p.begin("obd", true);
  if (p.getBytesLength("nets") == sizeof(NetList))
    p.getBytes("nets", &nets, sizeof(NetList));
  p.end();
  if (nets.magic != 0x7E) { memset(&nets, 0, sizeof(nets)); nets.magic = 0x7E; }
  if (nets.count > NET_MAX) nets.count = 0;
}

inline void netSave(Preferences& p) {
  p.begin("obd", false);
  p.putBytes("nets", &nets, sizeof(NetList));
  p.end();
}

// Добавить/обновить сеть. slot 0 — всегда сеть OBD-адаптера.
inline void netSet(Preferences& p, int slot, const char* ssid, const char* pass) {
  if (slot < 0 || slot >= NET_MAX) return;
  strncpy(nets.net[slot].ssid, ssid ? ssid : "", NET_SSID_LEN - 1);
  nets.net[slot].ssid[NET_SSID_LEN - 1] = 0;
  strncpy(nets.net[slot].pass, pass ? pass : "", NET_PASS_LEN - 1);
  nets.net[slot].pass[NET_PASS_LEN - 1] = 0;
  if (slot >= nets.count) nets.count = slot + 1;
  netSave(p);
}

inline void netClear(Preferences& p, int slot) {
  if (slot <= 0 || slot >= NET_MAX) return;      // слот 0 (OBD) не стираем
  nets.net[slot].ssid[0] = 0;
  nets.net[slot].pass[0] = 0;
  netSave(p);
}

// Подключиться по приоритету. Возвращает индекс сети или -1.
// Сканируем эфир один раз, затем выбираем: если видна сеть OBD (slot 0) —
// только она. Иначе первая найденная из запасных.
inline int netConnectBest(uint32_t perNetTimeoutMs = 8000) {
  netActive = -1; netIsObd = false;

  int n = WiFi.scanNetworks(false, false);
  Serial.printf("scan: %d сетей\n", n);
  if (n <= 0) { WiFi.scanDelete(); return -1; }

  // Идём по ВСЕМ слотам, а не по nets.count: count — это «до какого слота
  // писали», а заполненным может быть любой (например только запасной).
  int pick = -1;
  for (int slot = 0; slot < NET_MAX && pick < 0; slot++) {
    if (!nets.net[slot].ssid[0]) continue;
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == nets.net[slot].ssid) { pick = slot; break; }
    }
    // слот 0 = OBD: если он есть в эфире, других не рассматриваем
    if (pick == 0) break;
  }
  WiFi.scanDelete();
  if (pick < 0) { Serial.println("ни одна известная сеть не видна"); return -1; }

  const NetEntry& e = nets.net[pick];
  Serial.printf("подключаемся к [%s] (slot %d)\n", e.ssid, pick);
  if (e.pass[0]) WiFi.begin(e.ssid, e.pass);
  else           WiFi.begin(e.ssid);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < perNetTimeoutMs) delay(150);

  if (WiFi.status() == WL_CONNECTED) {
    netActive = pick;
    netIsObd  = (pick == 0);
    Serial.printf("WiFi OK [%s] IP=%s%s\n", e.ssid,
                  WiFi.localIP().toString().c_str(), netIsObd ? "  (OBD)" : "");
    return pick;
  }
  Serial.printf("не удалось подключиться к [%s]\n", e.ssid);
  return -1;
}
