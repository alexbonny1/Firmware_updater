/*
 * timbry NFC Reader Firmware
 * ESP32-WROOM + RC522 + ILI9488 TFT 480x320 + Buzzer
 * v3.4
 *
 * PIN MAP:
 * ─────────────────────────────────────
 * TFT ILI9488 (SPI)
 *   MOSI → GPIO 23 / SCK → GPIO 18
 *   CS   → GPIO 15 / DC  → GPIO 2
 *   RST  → GPIO 4  / BL  → GPIO 32
 * RC522:
 *   MOSI → GPIO 23 / MISO → GPIO 19
 *   SCK  → GPIO 18 / SS   → GPIO 21 / RST → GPIO 22
 * BUZZER → GPIO 33
 *
 * TAG ADMIN: 3605CA06 / F917C906
 *   1a lettura → schermata configurazione
 *   2a lettura entro 60s → entra in PROVISIONING (portale WiFi)
 *   timeout 60s → torna in idle
 *
 * COMANDI SERIALI:
 *   RESET           → cancella config e riavvia
 *   STATUS          → stampa stato corrente
 *   FLUSH           → svuota coda offline
 *   DEBOUNCE <ms>   → debounce tag (500-30000)
 *   DISPLAY <ms>    → durata schermata risultato (500-10000)
 *   THEME <0|1>     → 0=scuro, 1=chiaro
 *   PROV            → entra in provisioning
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include "certs.h"
#include <Preferences.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <MFRC522.h>
#include <TFT_eSPI.h>
#include <time.h>
#include "esp_log.h"
#include "esp_wifi.h"
#include "lopaka_assets.h"

// ── PIN ──────────────────────────────
#define PIN_RC522_SS    21
#define PIN_RC522_RST   22
#define PIN_RC522_MISO  19
#define PIN_RC522_MOSI  23
#define PIN_RC522_SCK   18
#define BUZZER_PIN      33
#define TFT_BL_PIN      32

// ── CONFIG ───────────────────────────
#define FW_VERSION       "3.4"
#define PREF_NAMESPACE   "timrbry"
#define QUEUE_MAX        100
#define HEARTBEAT_MS     60000UL
#define DEBOUNCE_DEFAULT 5000UL
#define RECONNECT_MS     10000UL
#define NTP_SERVER       "pool.ntp.org"
#define TZ_POSIX         "CET-1CEST,M3.5.0,M10.5.0/3"
#define NTP_RETRY_MS     10000UL

// ── TAG ADMIN ────────────────────────
#define ADMIN_UID        "3605CA06"
#define ADMIN_UID_2      "F917C906"
#define ADMIN_TIMEOUT_MS 60000UL

// ── LAYOUT (design Lopaka, 480×320) ──
#define LOGO_X      14
#define LOGO_Y      13
#define LOGO_W      58
#define LOGO_H      50
#define HDR_TXT_X   83
#define HDR_TXT_Y   17
#define DATE_TXT_X  82
#define DATE_TXT_Y  45
#define WIFI_X     435
#define WIFI_Y      11
#define CLK_X       0    // Calcolato dinamicamente (centrato)
#define CLK_Y      100
#define CLK_SIZE    3    // Moltiplicatore di dimensione per FONT8
#define CLK_W      480   // Occupa tutta la larghezza
#define CLK_H      150   // Altezza della zona orario
#define FTR_TXT_X  118
#define FTR_TXT_Y  278

// ── COLORI FISSI ─────────────────────
#define C_GREEN   0x07E0
#define C_RED     0xF800
#define C_YELLOW  0xFFE0
#define C_ORANGE  0xFC00
#define C_WHITE   0xFFFF
#define C_BLACK   0x0000
#define C_ACCENT  0x02BA   // blu logo Timbry
#define C_WIFI    0x64E6   // verde barre WiFi

// Colori tema (aggiornati da applyTheme)
uint16_t C_BG     = C_BLACK;
uint16_t C_TEXT   = C_WHITE;
uint16_t C_DIM    = 0x4208;
uint16_t C_YELLOW_DYN = C_YELLOW;

// ── OGGETTI ──────────────────────────
MFRC522     rfid(PIN_RC522_SS, PIN_RC522_RST);
TFT_eSPI    tft = TFT_eSPI();
Preferences prefs;

// ── STRUTTURE ────────────────────────
struct Config {
  char     backend[128];
  char     readerId[64];
  char     companyId[64];
  char     sede[64];
  uint8_t  theme;     // 0=scuro, 1=chiaro
  uint32_t debounce;
  uint32_t displayMs;
  bool     valid;
};
Config cfg;

struct DisplayState {
  String status;
  String oraCorrente;
};
DisplayState ds;

struct QueueEntry {
  char uid[64];
  char timestamp[32];
};
QueueEntry g_queue[QUEUE_MAX];

// ── COMPONENT HEALTH ─────────────────
bool g_rfidOk    = false;
bool g_displayOk = false;

// ── GLOBALS ──────────────────────────
char          g_uid[64];
char          g_payload[600];
char          g_url[256];
unsigned long g_lastHeartbeat   = 0;
unsigned long g_lastRead        = 0;
unsigned long g_lastReconnect   = 0;
unsigned long g_lastClockUpdate = 0;
uint32_t      g_lastHash        = 0;
bool          g_wifiOffline     = false;
int           g_queueSize       = 0;
bool          g_ntpSynced       = false;
unsigned long g_debouncMs       = DEBOUNCE_DEFAULT;
unsigned long g_resultTimer     = 0;
unsigned long g_resultTimeout   = 3000UL;
bool          g_adminMode       = false;
unsigned long g_adminTimer      = 0;
bool          g_waitingNtp      = false;
unsigned long g_lastNtpRetry    = 0;

// ── QUEUE FLUSH NON-BLOCCANTE ────────
int           g_flushRd     = -1;
int           g_flushWr     = 0;
unsigned long g_flushLastMs = 0;

// forward declarations
void showIdle();
void showWaitingNtp();
void startProvisioning();

// ── JSON ESCAPE ──────────────────────
static void jsonEscape(const char* src, char* dst, size_t maxLen) {
  size_t j = 0;
  for (size_t i = 0; src[i] != '\0' && j + 2 < maxLen; i++) {
    if (src[i] == '"' || src[i] == '\\') {
      if (j + 3 >= maxLen) break;
      dst[j++] = '\\';
    }
    dst[j++] = src[i];
  }
  dst[j] = '\0';
}

// ── TEMA ─────────────────────────────
void applyTheme(uint8_t idx) {
  bool dark = (idx == 0);
  C_BG        = dark ? C_BLACK : C_WHITE;
  C_TEXT      = dark ? C_WHITE : C_BLACK;
  C_DIM       = dark ? (uint16_t)0x4208 : (uint16_t)0xC618;
  C_YELLOW_DYN = dark ? (uint16_t)C_YELLOW : (uint16_t)0xAE00; // Giallo puro scuro nel tema chiaro
}

// ── WIFI SIGNAL ──────────────────────
int rssiToBars() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  int rssi = WiFi.RSSI();
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  return 1;
}

// Barre WiFi dinamiche: più segnale = più barre attive (30×32px, bottom-aligned)
void drawWifiBars(int bars) {
  tft.fillRect(WIFI_X, WIFI_Y, 30, 32, C_BG);
  const int bw = 5, gap = 2;
  const int heights[4] = {8, 16, 24, 32};
  uint16_t activeColor = (bars == 0) ? (uint16_t)C_RED : (uint16_t)C_WIFI;
  for (int i = 0; i < 4; i++) {
    int bh = heights[i];
    tft.fillRect(WIFI_X + i * (bw + gap), WIFI_Y + (32 - bh), bw, bh,
                 (i < bars) ? activeColor : C_DIM);
  }
}

// ── UTILITÀ TEMPO ────────────────────
String getLocalDate() {
  time_t now = time(nullptr);
  if (now < 1000000000UL) return "--/--/----";
  struct tm* t = localtime(&now);
  char buf[12];
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d",
    t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
  return String(buf);
}

String getISOTimestamp() {
  time_t now = time(nullptr);
  if (now < 1000000000UL) return "";
  struct tm* t = gmtime(&now);
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
    t->tm_year+1900, t->tm_mon+1, t->tm_mday,
    t->tm_hour, t->tm_min, t->tm_sec);
  return String(buf);
}

String getLocalTime() {
  time_t now = time(nullptr);
  if (now < 1000000000UL) return "--:--:--";
  struct tm* t = localtime(&now);
  char buf[10];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
  return String(buf);
}

// ── BUZZER ───────────────────────────
void beepOk()      { tone(BUZZER_PIN, 1800, 80); delay(100); tone(BUZZER_PIN, 2200, 80); }
void beepErr()     { tone(BUZZER_PIN, 900, 150); delay(80);  tone(BUZZER_PIN, 700, 150); }
void beepOffline() { tone(BUZZER_PIN, 1600, 100); }

// ── NTP ──────────────────────────────
bool syncNTP() {
  configTzTime(TZ_POSIX, NTP_SERVER);
  Serial.print("NTP sync");
  unsigned long s = millis();
  while (time(nullptr) < 1000000000UL) {
    if (millis() - s > 8000) { Serial.println(" FAIL"); return false; }
    delay(200); Serial.print(".");
  }
  Serial.println(" OK");
  return true;
}

// ── RFID INIT ────────────────────────
void rfidInit() {
  SPI.begin(PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SS);
  // RST basso→alto: garantisce stato noto dopo power cycle
  pinMode(PIN_RC522_RST, OUTPUT);
  digitalWrite(PIN_RC522_RST, LOW);
  delay(10);
  digitalWrite(PIN_RC522_RST, HIGH);
  delay(50);
  rfid.PCD_Init();
  delay(50);
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  g_rfidOk = (v != 0x00 && v != 0xFF);
  Serial.printf("RC522 version: 0x%02X %s\n", v, g_rfidOk ? "OK" : "WARN");
}

// ── DISPLAY ──────────────────────────

void drawClock() {
  tft.fillRect(0, CLK_Y, 480, CLK_H, C_BG);
  tft.setTextFont(7);                    // Font per orario
  tft.setTextSize(CLK_SIZE);
  tft.setTextColor(C_TEXT, C_BG);

  String oraDisplay = (ds.oraCorrente.length() >= 5)
                      ? ds.oraCorrente.substring(0, 5)
                      : "--:--";

  int textWidth = tft.textWidth(oraDisplay);
  int x = (480 - textWidth) / 2 ;         // Centra orizzontalmente
  int y = CLK_Y;                    // Posiziona verticalmente

  tft.drawString(oraDisplay, x, y);
  tft.setTextFont(1);                    // ← RIPRISTINA font 1 (default impostato nel setup)
}

void showIdle() {
  if (!g_ntpSynced) { g_waitingNtp = true; showWaitingNtp(); return; }
  g_waitingNtp = false;
  ds.status    = "ATTESA";

  tft.fillScreen(C_BG);

  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);

  tft.setTextColor(C_TEXT, C_BG);
  tft.setTextSize(2);
  tft.drawString(cfg.readerId[0] ? cfg.readerId : "Reader", HDR_TXT_X, HDR_TXT_Y);
  tft.drawString(getLocalDate(), DATE_TXT_X, DATE_TXT_Y);

  drawWifiBars(rssiToBars());



  drawClock();
}

void showResult(String tipo, String nome, String orario) {
  ds.status = tipo;

  uint16_t bg, fg;
  if      (tipo == "ENTRATA") { bg = 0x0320; fg = C_GREEN;  }
  else if (tipo == "USCITA")  { bg = 0x8200; fg = C_RED;    }
  else if (tipo == "ERRORE")  { bg = 0x4000; fg = C_ORANGE; }
  else                         { bg = 0x2104; fg = C_YELLOW; }

  tft.fillScreen(bg);

  tft.setTextColor(fg, bg); tft.setTextSize(5);
  int sw = (int)tipo.length() * 30;
  tft.drawString(tipo, (480 - sw) / 2 > 10 ? (480 - sw) / 2 : 10, 70);

  if (nome.length() > 0) {
    tft.setTextColor(C_WHITE, bg); tft.setTextSize(3);
    int nw = (int)nome.length() * 18;
    tft.drawString(nome, (480 - nw) / 2 > 10 ? (480 - nw) / 2 : 10, 175);
  }
  if (orario.length() > 0) {
    tft.setTextColor(C_YELLOW_DYN, bg); tft.setTextSize(2);
    int ow = (int)orario.length() * 12;
    tft.drawString(orario, (480 - ow) / 2 > 10 ? (480 - ow) / 2 : 10, 250);
  }

  g_resultTimer = millis();
}

void drawAdmin() {
  tft.fillScreen(C_BG);

  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);
  tft.setTextColor(C_YELLOW_DYN, C_BG); tft.setTextSize(2);
  tft.drawString("CONFIGURAZIONE", HDR_TXT_X, HDR_TXT_Y);

  int y = 80;
  tft.setTextSize(1);

#define ADM_ROW(label, value, col) \
  tft.setTextColor(C_TEXT, C_BG); tft.drawString(label, 10, y); \
  tft.setTextColor(col, C_BG);   tft.drawString(value, 10 + sizeof(label)*6, y); \
  y += 18;

  char qStr[6]; snprintf(qStr, sizeof(qStr), "%d", g_queueSize);

  ADM_ROW("Backend:  ", cfg.backend,                              C_TEXT)
  ADM_ROW("Reader:   ", cfg.readerId,                             C_TEXT)
  ADM_ROW("Company:  ", cfg.companyId,                            C_TEXT)
  ADM_ROW("Sede:     ", cfg.sede[0] ? cfg.sede : "-",             C_TEXT)
  ADM_ROW("Tema:     ", cfg.theme == 0 ? "Scuro" : "Chiaro",      C_TEXT)
  ADM_ROW("NTP:      ", g_ntpSynced ? "OK" : "NO SYNC",           g_ntpSynced ? C_GREEN : C_RED)
  ADM_ROW("Queue:    ", qStr,                                      g_queueSize > 0 ? C_YELLOW_DYN : C_TEXT)
  ADM_ROW("FW:       ", FW_VERSION,                               C_TEXT)
  ADM_ROW("NFC:      ", g_rfidOk ? "OK" : "ERRORE",               g_rfidOk ? C_GREEN : C_RED)

#undef ADM_ROW

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  tft.setTextColor(C_TEXT, C_BG); tft.drawString("WiFi:     ", 10, y);
  tft.setTextColor(wifiOk ? (uint16_t)C_GREEN : (uint16_t)C_RED, C_BG);
  tft.drawString(wifiOk ? WiFi.localIP().toString().c_str() : "OFFLINE", 10 + 10*6, y);
  y += 28;

  tft.setTextColor(C_ACCENT, C_BG); tft.setTextSize(2);
  tft.drawString("Ripassare: ENTRA PORTALE", 10, y);
  y += 28;
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
  tft.drawString("Attendi 60s per annullare", 10, y);
}

void showWaitingNtp() {
  g_waitingNtp = true;
  tft.fillScreen(C_BG);

  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(2);
  tft.drawString(cfg.readerId[0] ? cfg.readerId : "Reader", HDR_TXT_X, HDR_TXT_Y);
  tft.drawString(getLocalDate(), DATE_TXT_X, DATE_TXT_Y);
  drawWifiBars(rssiToBars());

  tft.setTextFont(7);
  tft.setTextSize(CLK_SIZE);
  tft.setTextColor(C_TEXT, C_BG);
  String placeholder = "--:--";
  int ph_width = tft.textWidth(placeholder);
  int ph_x = (480 - ph_width) / 2 ;
  tft.drawString(placeholder, ph_x, CLK_Y);
  tft.setTextFont(1);                    // ← RIPRISTINA font 1 (default impostato nel setup)

  tft.setTextColor(C_YELLOW_DYN, C_BG); tft.setTextSize(3);
  tft.drawString(g_wifiOffline ? "attesa WiFi..." : "sync NTP...", 50, FTR_TXT_Y);

  g_lastNtpRetry = millis();
}

void taskNtpRetry() {
  if (!g_waitingNtp) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - g_lastNtpRetry < NTP_RETRY_MS) return;
  g_lastNtpRetry = millis();
  Serial.println("NTP retry...");
  if (syncNTP()) { g_ntpSynced = true; g_waitingNtp = false; showIdle(); }
  else { drawWifiBars(rssiToBars()); }
}

void updateClock() {
  if (millis() - g_lastClockUpdate < 1000) return;
  g_lastClockUpdate = millis();
  if (!g_ntpSynced) return;

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min);
  String newOra = String(buf);

  // Aggiorna barre WiFi se il segnale è cambiato
  static int lastBars = -1;
  int bars = rssiToBars();
  if (bars != lastBars && ds.status == "ATTESA") {
    lastBars = bars;
    drawWifiBars(bars);
  }

  // Aggiorna orologio solo quando i minuti cambiano
  if (ds.status == "ATTESA" && newOra != ds.oraCorrente) {
    ds.oraCorrente = newOra;
    drawClock();
  } else {
    ds.oraCorrente = newOra;
  }
}

// ── CONFIG NVS ───────────────────────
bool loadConfig() {
  prefs.begin(PREF_NAMESPACE, true);
  String backend    = prefs.getString("backend",   "");
  String readerId   = prefs.getString("readerId",  "");
  String companyId  = prefs.getString("companyId", "");
  String sede       = prefs.getString("sede",      "");
  uint8_t  rawTheme  = (uint8_t)prefs.getUInt("theme",     0);
  uint32_t debounce  = prefs.getUInt("debounce",  (uint32_t)DEBOUNCE_DEFAULT);
  uint32_t displayMs = prefs.getUInt("displayMs", 3000);
  prefs.end();
  if (backend.length() < 4 || readerId.length() < 2 || companyId.length() < 10) return false;
  strlcpy(cfg.backend,   backend.c_str(),   sizeof(cfg.backend));
  strlcpy(cfg.readerId,  readerId.c_str(),  sizeof(cfg.readerId));
  strlcpy(cfg.companyId, companyId.c_str(), sizeof(cfg.companyId));
  strlcpy(cfg.sede,      sede.c_str(),      sizeof(cfg.sede));
  // Migrazione da vecchi 9 temi: solo il tema 4 (Bianco) era chiaro
  cfg.theme      = (rawTheme == 4 || rawTheme == 1) ? 1 : 0;
  cfg.debounce   = (debounce >= 500 && debounce <= 30000) ? debounce : (uint32_t)DEBOUNCE_DEFAULT;
  cfg.displayMs  = (displayMs >= 500 && displayMs <= 10000) ? displayMs : 3000UL;
  cfg.valid      = true;
  applyTheme(cfg.theme);
  g_debouncMs     = cfg.debounce;
  g_resultTimeout = cfg.displayMs;
  return true;
}

void saveConfig() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putString("backend",   cfg.backend);
  prefs.putString("readerId",  cfg.readerId);
  prefs.putString("companyId", cfg.companyId);
  prefs.putString("sede",      cfg.sede);
  prefs.putUInt("theme",       cfg.theme);
  prefs.putUInt("debounce",    cfg.debounce);
  prefs.putUInt("displayMs",   cfg.displayMs);
  prefs.end();
}

void clearConfig() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.clear();
  prefs.end();
}

// ── QUEUE ────────────────────────────
void saveQueue() {
  prefs.begin(PREF_NAMESPACE, false);
  prefs.putInt("qsize", g_queueSize);
  for (int i = 0; i < g_queueSize; i++) {
    char key[16]; snprintf(key, sizeof(key), "q%d", i);
    prefs.putString(key, String(g_queue[i].uid) + "|" + String(g_queue[i].timestamp));
  }
  prefs.end();
}

void loadQueue() {
  prefs.begin(PREF_NAMESPACE, true);
  g_queueSize = min((int)prefs.getInt("qsize", 0), QUEUE_MAX);
  for (int i = 0; i < g_queueSize; i++) {
    char key[16]; snprintf(key, sizeof(key), "q%d", i);
    String val = prefs.getString(key, "");
    int sep = val.indexOf('|');
    if (sep > 0) {
      strlcpy(g_queue[i].uid,       val.substring(0, sep).c_str(),  sizeof(g_queue[i].uid));
      strlcpy(g_queue[i].timestamp, val.substring(sep + 1).c_str(), sizeof(g_queue[i].timestamp));
    }
  }
  prefs.end();
}

void queueAdd(const char* uid, const char* ts) {
  if (g_queueSize >= QUEUE_MAX) return;
  strlcpy(g_queue[g_queueSize].uid,       uid, sizeof(g_queue[0].uid));
  strlcpy(g_queue[g_queueSize].timestamp, ts,  sizeof(g_queue[0].timestamp));
  g_queueSize++;
  saveQueue();
  Serial.printf("QUEUE +1 (tot: %d)\n", g_queueSize);
}

// ── HTTP POST ────────────────────────
int httpPost(const char* path, const char* payload) {
  if (WiFi.status() != WL_CONNECTED) return -1;
  snprintf(g_url, sizeof(g_url), "%s%s", cfg.backend, path);
  Serial.printf("POST %s\n", g_url);
  bool   isHttps = strncmp(cfg.backend, "https", 5) == 0;
  int    code    = -1;
  String body    = "";
  if (isHttps) {
    WiFiClientSecure client; client.setCACert(ROOT_CA); HTTPClient http;
    if (!http.begin(client, g_url)) return -1;
    http.setTimeout(15000); http.setReuse(false);
    http.addHeader("Content-Type", "application/json");
    code = http.POST(payload); body = http.getString(); http.end();
  } else {
    WiFiClient client; HTTPClient http;
    if (!http.begin(client, g_url)) return -1;
    http.setTimeout(15000); http.setReuse(false);
    http.addHeader("Content-Type", "application/json");
    code = http.POST(payload); body = http.getString(); http.end();
  }
  if (code == 200 && body.length() > 2) {
    StaticJsonDocument<512> doc;
    if (!deserializeJson(doc, body)) {
      String tipo    = doc["tipo"]       | "";
      String dipName = doc["dipendente"] | "";
      if (tipo.length() > 0) showResult(tipo, dipName, getLocalTime());
    }
  }
  Serial.printf("CODE: %d\n", code);
  return code;
}

// ── QUEUE FLUSH NON-BLOCCANTE ────────
void startQueueFlush() {
  if (WiFi.status() != WL_CONNECTED || g_queueSize == 0) return;
  if (g_flushRd >= 0) return;
  Serial.printf("FLUSH QUEUE avviato (%d elementi)...\n", g_queueSize);
  g_flushRd     = 0;
  g_flushWr     = 0;
  g_flushLastMs = millis() - 300;
}

void taskQueueFlush() {
  if (g_flushRd < 0) return;
  if (WiFi.status() != WL_CONNECTED) { g_flushRd = -1; g_flushWr = 0; return; }
  if (millis() - g_flushLastMs < 300) return;

  if (g_flushRd >= g_queueSize) {
    int sent = g_queueSize - g_flushWr;
    g_queueSize = g_flushWr;
    g_flushRd   = -1;
    g_flushWr   = 0;
    saveQueue();
    Serial.printf("FLUSH completato: sent=%d failed=%d\n", sent, g_queueSize);
    return;
  }

  char eReader[130], eCompany[130];
  jsonEscape(cfg.readerId,  eReader,  sizeof(eReader));
  jsonEscape(cfg.companyId, eCompany, sizeof(eCompany));

  bool isHttps = strncmp(cfg.backend, "https", 5) == 0;
  snprintf(g_url, sizeof(g_url), "%s/api/hardware/tag", cfg.backend);
  if (strlen(g_queue[g_flushRd].timestamp) > 0)
    snprintf(g_payload, sizeof(g_payload),
      "{\"uid\":\"%s\",\"reader_id\":\"%s\",\"company_id\":\"%s\","
      "\"timestamp\":\"%s\",\"offline\":true}",
      g_queue[g_flushRd].uid, eReader, eCompany, g_queue[g_flushRd].timestamp);
  else
    snprintf(g_payload, sizeof(g_payload),
      "{\"uid\":\"%s\",\"reader_id\":\"%s\",\"company_id\":\"%s\",\"offline\":true}",
      g_queue[g_flushRd].uid, eReader, eCompany);

  int rc = -1;
  if (isHttps) {
    WiFiClientSecure c; c.setCACert(ROOT_CA); HTTPClient h;
    if (h.begin(c, g_url)) { h.setTimeout(15000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json"); rc = h.POST(g_payload); h.end(); }
  } else {
    WiFiClient c; HTTPClient h;
    if (h.begin(c, g_url)) { h.setTimeout(15000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json"); rc = h.POST(g_payload); h.end(); }
  }

  if (rc == 200 || rc == 201) {
    Serial.printf("FLUSH[%d]: OK\n", g_flushRd);
  } else {
    if (g_flushWr != g_flushRd) g_queue[g_flushWr] = g_queue[g_flushRd];
    g_flushWr++;
    Serial.printf("FLUSH[%d]: FAIL (%d)\n", g_flushRd, rc);
  }
  g_flushLastMs = millis();
  g_flushRd++;
}

// ── OTA UPDATE ───────────────────────
// Segue la catena di redirect (GitHub → CDN, max 3 hop)
static String resolveOtaUrl(const String& startUrl) {
  const char* hdrKeys[] = {"Location"};
  String url = startUrl;
  for (int hop = 0; hop < 3; hop++) {
    String loc = "";
    if (url.startsWith("https")) {
      WiFiClientSecure c; c.setCACert(ROOT_CA); c.setInsecure(); HTTPClient h;
      h.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
      h.collectHeaders(hdrKeys, 1);
      if (h.begin(c, url)) {
        int code = h.GET();
        if (code >= 300 && code < 400) loc = h.header("Location");
        h.end();
      }
    } else {
      WiFiClient c; HTTPClient h;
      h.setFollowRedirects(HTTPC_DISABLE_FOLLOW_REDIRECTS);
      h.collectHeaders(hdrKeys, 1);
      if (h.begin(c, url)) {
        int code = h.GET();
        if (code >= 300 && code < 400) loc = h.header("Location");
        h.end();
      }
    }
    if (loc.length() <= 8) break;
    Serial.printf("OTA redirect %d: %s\n", hop + 1, loc.c_str());
    url = loc;
  }
  Serial.printf("OTA URL finale: %s\n", url.c_str());
  return url;
}

void doOTA(String url, String newVersion) {
  Serial.printf("OTA: aggiornamento v%s → v%s\n", FW_VERSION, newVersion.c_str());

  // Callback per aggiornare progress bar
  int otaProgress = 0;
  auto onHttpEvent = [&](httpsClient_event_t *event) {
    if (event->event_id == HTTP_EVENT_ON_DATA) {
      otaProgress = event->state;
      // Ridisegna progress bar
      const int pb_y = 220;
      const int pb_h = 20;
      const int pb_w = 400;
      const int pb_x = (480 - pb_w) / 2;

      // Sfondo della progress bar
      tft.fillRect(pb_x, pb_y, pb_w, pb_h, C_DIM);
      // Barra di progresso
      if (otaProgress > 0) {
        int filled = (pb_w * otaProgress) / 100;
        tft.fillRect(pb_x, pb_y, filled, pb_h, C_GREEN);
      }
      // Percentuale
      char percStr[8]; snprintf(percStr, sizeof(percStr), "%d%%", otaProgress);
      tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
      int perc_w = tft.textWidth(percStr);
      tft.fillRect(pb_x + pb_w / 2 - perc_w / 2 - 2, pb_y + 5, perc_w + 4, 12, C_BG);
      tft.drawString(percStr, pb_x + pb_w / 2 - perc_w / 2, pb_y + 5);
    }
  };

  // Schermata iniziale
  tft.fillScreen(C_BG);
  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);

  tft.setTextColor(C_ACCENT, C_BG); tft.setTextSize(2);
  String titleText = "Aggiornamento FW";
  int title_w = tft.textWidth(titleText);
  tft.drawString(titleText, (480 - title_w) / 2, 50);

  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(3);
  char verStr[32];
  snprintf(verStr, sizeof(verStr), "v%s → v%s", FW_VERSION, newVersion.c_str());
  int ver_w = tft.textWidth(verStr);
  tft.drawString(verStr, (480 - ver_w) / 2, 130);

  // Progress bar background
  const int pb_y = 220;
  const int pb_h = 20;
  const int pb_w = 400;
  const int pb_x = (480 - pb_w) / 2;
  tft.fillRect(pb_x, pb_y, pb_w, pb_h, C_DIM);

  tft.setTextColor(C_YELLOW_DYN, C_BG); tft.setTextSize(1);
  tft.drawString("Scaricamento in corso...", pb_x, pb_y + pb_h + 8);

  String finalUrl = resolveOtaUrl(url);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret;
  if (finalUrl.startsWith("https")) {
    WiFiClientSecure client; client.setCACert(ROOT_CA); client.setInsecure();
    ret = httpUpdate.update(client, finalUrl);
  } else {
    WiFiClient client;
    ret = httpUpdate.update(client, finalUrl);
  }

  // Arriva qui solo se OTA fallita (altrimenti si riavvia)
  Serial.printf("OTA FALLITO (%d): %s\n", httpUpdate.getLastError(),
    httpUpdate.getLastErrorString().c_str());
  tft.fillScreen(C_BG);
  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);
  tft.setTextColor(C_RED, C_BG); tft.setTextSize(2);
  String errText = "Errore aggiornamento";
  int err_w = tft.textWidth(errText);
  tft.drawString(errText, (480 - err_w) / 2, 150);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
  tft.drawString(httpUpdate.getLastErrorString().c_str(), 20, 200);
  delay(4000);
  showIdle();
}

// ── HEARTBEAT ────────────────────────
void sendHeartbeat() {
  if (millis() - g_lastHeartbeat < HEARTBEAT_MS) return;
  g_lastHeartbeat = millis();

  char eReader[130], eCompany[130], eSede[130];
  jsonEscape(cfg.readerId,  eReader,  sizeof(eReader));
  jsonEscape(cfg.companyId, eCompany, sizeof(eCompany));
  jsonEscape(cfg.sede,      eSede,    sizeof(eSede));

  snprintf(g_payload, sizeof(g_payload),
    "{\"reader_id\":\"%s\",\"company_id\":\"%s\",\"firmware\":\"%s\",\"queue\":%d"
    ",\"sede\":\"%s\",\"nfc_ok\":%s,\"display_ok\":%s}",
    eReader, eCompany, FW_VERSION, g_queueSize,
    eSede, g_rfidOk ? "true" : "false", g_displayOk ? "true" : "false");

  snprintf(g_url, sizeof(g_url), "%s/api/hardware/ping", cfg.backend);
  bool isHttps = strncmp(cfg.backend, "https", 5) == 0;

  String otaUrl     = "";
  String otaVersion = "";
  int    pingCode   = -1;

  auto handlePingResponse = [&](HTTPClient& h, int code) {
    pingCode = code;
    Serial.printf("PING: %d\n", code);
    if (code == 200) {
      String body = h.getString();
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, body)) {
        otaUrl     = doc["ota_url"]     | "";
        otaVersion = doc["ota_version"] | "";
      }
    }
  };

  if (isHttps) {
    WiFiClientSecure c; c.setCACert(ROOT_CA); HTTPClient h;
    if (h.begin(c, g_url)) {
      h.setTimeout(10000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json");
      handlePingResponse(h, h.POST(g_payload));
      h.end();
    }
  } else {
    WiFiClient c; HTTPClient h;
    if (h.begin(c, g_url)) {
      h.setTimeout(10000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json");
      handlePingResponse(h, h.POST(g_payload));
      h.end();
    }
  }

  if (pingCode <= 0) {
    unsigned long _now = millis();
    g_lastHeartbeat = (_now >= HEARTBEAT_MS - 10000UL)
                      ? _now - HEARTBEAT_MS + 10000UL
                      : 0UL;
  }

  if (otaUrl.length() > 0 && otaVersion.length() > 0 && otaVersion != FW_VERSION) {
    doOTA(otaUrl, otaVersion);
  }
}

// ── RFID ─────────────────────────────
uint32_t fnv1a(const char* s) {
  uint32_t h = 0x811c9dc5;
  while (*s) { h ^= (uint8_t)*s++; h *= 0x01000193; }
  return h;
}

void taskRfid() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  g_uid[0] = '\0';
  for (byte i = 0; i < rfid.uid.size; i++) {
    char hex[5]; snprintf(hex, sizeof(hex), "%02X", rfid.uid.uidByte[i]);
    strlcat(g_uid, hex, sizeof(g_uid));
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  uint32_t hash = fnv1a(g_uid);

  // ── TAG ADMIN ──────────────────────
  bool isAdmin = (strcmp(g_uid, ADMIN_UID) == 0) ||
                 (strcmp(g_uid, ADMIN_UID_2) == 0);
  if (isAdmin) {
    if (g_adminMode) {
      Serial.println("ADMIN → PROVISIONING");
      beepOk();
      g_adminMode = false;
      tft.fillScreen(C_BG);
      tft.setTextColor(C_ACCENT, C_BG); tft.setTextSize(3);
      tft.drawString("Avvio portale...", 100, 140);
      delay(1200);
      startProvisioning();
    } else {
      Serial.println("ADMIN MODE attivato");
      beepOk();
      g_adminMode  = true;
      g_adminTimer = millis();
      g_lastHash   = 0;
      drawAdmin();
    }
    return;
  }

  if (g_adminMode) {
    g_adminMode = false;
    if (g_waitingNtp) showWaitingNtp(); else showIdle();
  }

  if (g_waitingNtp) {
    tft.fillRect(0, FTR_TXT_Y, 480, 30, C_BG);
    tft.setTextColor(C_RED, C_BG); tft.setTextSize(2);
    tft.drawString("Orario non pronto", 80, FTR_TXT_Y);
    delay(1500); showWaitingNtp();
    return;
  }

  if (hash == g_lastHash && millis() - g_lastRead < g_debouncMs) return;
  g_lastHash = hash;
  g_lastRead = millis();

  Serial.printf("TAG: %s\n", g_uid);
  beepOk();

  char eReader[130], eCompany[130];
  jsonEscape(cfg.readerId,  eReader,  sizeof(eReader));
  jsonEscape(cfg.companyId, eCompany, sizeof(eCompany));

  String isoTs = getISOTimestamp();
  if (isoTs.length() > 0)
    snprintf(g_payload, sizeof(g_payload),
      "{\"uid\":\"%s\",\"reader_id\":\"%s\",\"company_id\":\"%s\",\"timestamp\":\"%s\",\"offline\":false}",
      g_uid, eReader, eCompany, isoTs.c_str());
  else
    snprintf(g_payload, sizeof(g_payload),
      "{\"uid\":\"%s\",\"reader_id\":\"%s\",\"company_id\":\"%s\",\"offline\":false}",
      g_uid, eReader, eCompany);

  int code = httpPost("/api/hardware/tag", g_payload);
  if (code <= 0) {
    beepOffline();
    queueAdd(g_uid, isoTs.length() > 0 ? isoTs.c_str() : "");
    showResult("OFFLINE", "Salvato in coda",
      isoTs.length() > 0 ? isoTs.substring(11, 19) : "");
  } else if (code != 200 && code != 201) {
    beepErr();
    showResult("ERRORE", "Tag non registrato", "");
  }
}

// ── WIFI ─────────────────────────────
void taskWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    if (g_wifiOffline) {
      g_wifiOffline = false;
      Serial.println("WIFI RESTORED");
      rfidInit();
      if (syncNTP()) { g_ntpSynced = true; }
      else { showWaitingNtp(); }
      g_lastHeartbeat = millis() - HEARTBEAT_MS;
      sendHeartbeat();
      startQueueFlush();
      if (g_ntpSynced) showIdle();
    }
    return;
  }
  if (!g_wifiOffline) {
    g_wifiOffline   = true;
    g_lastReconnect = millis();
    Serial.println("WIFI OFFLINE");
    if (!g_ntpSynced) showWaitingNtp();
    else if (ds.status == "ATTESA") drawWifiBars(0);
  }
  if (millis() - g_lastReconnect > RECONNECT_MS) {
    g_lastReconnect = millis();
    String ssid = WiFi.SSID(), psk = WiFi.psk();
    Serial.printf("WiFi reconnect... SSID='%s'\n", ssid.c_str());
    if (ssid.length() > 0) WiFi.begin(ssid.c_str(), psk.c_str());
    else WiFi.begin();
  }
}

// ── PROVISIONING ─────────────────────
void startProvisioning() {
  tft.fillScreen(C_BG);

  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(2);
  tft.drawString("Configurazione WiFi", HDR_TXT_X, HDR_TXT_Y);

  char apName[32];
  snprintf(apName, sizeof(apName), "timbry-%06X", (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));

  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
  tft.drawString("Connetti al WiFi:", 14, 80);
  tft.setTextColor(C_ACCENT, C_BG); tft.setTextSize(3);
  tft.drawString(apName, 14, 100);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
  tft.drawString("Poi vai su: 192.168.4.1", 14, 148);
  tft.drawString("Scrivi RESET nel campo Backend per cancellare tutto", 14, 164);
  tft.drawString("Tema attuale:", 14, 190);
  tft.setTextColor(C_TEXT, C_BG);
  tft.drawString(cfg.theme == 0 ? "0 = Scuro" : "1 = Chiaro", 14 + 14*6, 190);

  WiFiManager wm;
  wm.setConfigPortalTimeout(300);

  WiFiManagerParameter p_b("backend", "Backend URL",         cfg.backend,   127);
  WiFiManagerParameter p_r("reader",  "Reader ID",           cfg.readerId,   63);
  WiFiManagerParameter p_c("company", "Company ID",          cfg.companyId,  63);
  WiFiManagerParameter p_s("sede",    "Sede / Ubicazione",   cfg.sede,       63);
  char themeStr[3];
  snprintf(themeStr, sizeof(themeStr), "%d", cfg.theme);
  WiFiManagerParameter p_t("theme", "Tema: 0=Scuro  1=Chiaro", themeStr, 2);
  char debounceStr[8];
  snprintf(debounceStr, sizeof(debounceStr), "%lu", (unsigned long)cfg.debounce);
  WiFiManagerParameter p_d("debounce", "Debounce tag (ms, 500-30000)", debounceStr, 6);
  WiFiManagerParameter p_reset(
    "<br><hr style='margin:16px 0'>"
    "<p style='color:#c00;font-weight:bold;margin-bottom:6px'>Reset completo (cancella config e coda offline):</p>"
    "<input type='button' value='RESET TUTTO' "
    "onclick=\"if(confirm('Confermi reset completo?')){document.getElementById('backend').value='RESET';document.querySelector('form').submit();}\" "
    "style='background:#c00;color:#fff;padding:10px 24px;border:none;border-radius:4px;"
    "font-size:15px;cursor:pointer;width:100%'>"
  );

  wm.addParameter(&p_b);
  wm.addParameter(&p_r);
  wm.addParameter(&p_c);
  wm.addParameter(&p_s);
  wm.addParameter(&p_t);
  wm.addParameter(&p_d);
  wm.addParameter(&p_reset);

  if (!wm.startConfigPortal(apName)) { ESP.restart(); return; }

  const char* newBackend = p_b.getValue();

  if (strcasecmp(newBackend, "RESET") == 0) {
    Serial.println("RESET richiesto dal portale");
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED, C_BG); tft.setTextSize(4);
    tft.drawString("RESET...", 140, 140);
    WiFi.disconnect(true);
    clearConfig();
    delay(1500);
    ESP.restart();
    return;
  }

  strlcpy(cfg.backend,   newBackend,         sizeof(cfg.backend));
  strlcpy(cfg.readerId,  p_r.getValue(),     sizeof(cfg.readerId));
  strlcpy(cfg.companyId, p_c.getValue(),     sizeof(cfg.companyId));
  strlcpy(cfg.sede,      p_s.getValue(),     sizeof(cfg.sede));
  cfg.theme = (atoi(p_t.getValue()) == 1) ? 1 : 0;

  unsigned long debounceVal = strtoul(p_d.getValue(), nullptr, 10);
  cfg.debounce = (debounceVal >= 500 && debounceVal <= 30000)
                 ? (uint32_t)debounceVal
                 : (uint32_t)DEBOUNCE_DEFAULT;
  g_debouncMs = cfg.debounce;

  int len = strlen(cfg.backend);
  if (len > 0 && cfg.backend[len-1] == '/') cfg.backend[len-1] = '\0';

  saveConfig();
  delay(500);
  ESP.restart();
}

// ── SERIAL ───────────────────────────
void taskSerial() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  String cmdU = cmd; cmdU.toUpperCase();

  if (cmdU == "RESET") {
    Serial.println("RESET CONFIG");
    WiFi.disconnect(true); clearConfig(); delay(1000); ESP.restart();
  } else if (cmdU == "STATUS") {
    Serial.printf("Backend:  %s\n", cfg.backend);
    Serial.printf("Reader:   %s\n", cfg.readerId);
    Serial.printf("Company:  %s\n", cfg.companyId);
    Serial.printf("Tema:     %d (%s)\n", cfg.theme, cfg.theme == 0 ? "Scuro" : "Chiaro");
    Serial.printf("Queue:    %d\n", g_queueSize);
    Serial.printf("WiFi:     %s\n", WiFi.status()==WL_CONNECTED ? "OK" : "OFFLINE");
    Serial.printf("IP:       %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("RSSI:     %d dBm (%d barre)\n", WiFi.RSSI(), rssiToBars());
    Serial.printf("NTP:      %s\n", g_ntpSynced ? "OK" : "NO SYNC");
    Serial.printf("Time:     %s\n", getISOTimestamp().c_str());
    Serial.printf("Sede:     %s\n", cfg.sede);
    Serial.printf("DEBOUNCE: %lu ms\n", g_debouncMs);
    Serial.printf("DISPLAY:  %lu ms\n", g_resultTimeout);
    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    Serial.printf("RC522:    0x%02X\n", v);
    Serial.printf("NFC OK:   %s\n", g_rfidOk    ? "SI" : "NO");
    Serial.printf("DISP OK:  %s\n", g_displayOk ? "SI" : "NO");
  } else if (cmdU == "FLUSH") {
    startQueueFlush();
  } else if (cmdU == "PROV") {
    Serial.println("Avvio provisioning da seriale...");
    startProvisioning();
  } else if (cmdU.startsWith("DEBOUNCE ")) {
    unsigned long ms = cmdU.substring(9).toInt();
    if (ms >= 500 && ms <= 30000) {
      g_debouncMs = ms; cfg.debounce = (uint32_t)ms; saveConfig();
      Serial.printf("DEBOUNCE → %lu ms (salvato)\n", ms);
    } else Serial.println("Valore non valido (500-30000)");
  } else if (cmdU.startsWith("DISPLAY ")) {
    unsigned long ms = cmdU.substring(8).toInt();
    if (ms >= 500 && ms <= 10000) {
      g_resultTimeout = ms; cfg.displayMs = (uint32_t)ms; saveConfig();
      Serial.printf("DISPLAY → %lu ms (salvato)\n", ms);
    } else Serial.println("Valore non valido (500-10000)");
  } else if (cmdU.startsWith("THEME ")) {
    int t = cmdU.substring(6).toInt();
    if (t == 0 || t == 1) {
      cfg.theme = (uint8_t)t;
      applyTheme(cfg.theme);
      saveConfig();
      Serial.printf("THEME → %d (%s)\n", cfg.theme, cfg.theme == 0 ? "Scuro" : "Chiaro");
      if (g_waitingNtp) showWaitingNtp(); else showIdle();
    } else {
      Serial.println("Tema non valido (0=Scuro, 1=Chiaro)");
    }
  }
}

// ── SETUP ────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.setDebugOutput(false);
  esp_log_level_set("*", ESP_LOG_NONE);
  Serial.println("\n\ntimbry NFC ESP32 v" FW_VERSION);

  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);

  tft.init();
  tft.setRotation(1);
  tft.setTextFont(1);  // Setta il font di default una volta
  g_displayOk = (tft.width() == 480 && tft.height() == 320);
  if (!g_displayOk) Serial.println("WARN: display dimensioni inattese");

  bool hasConfig = loadConfig();
  if (!hasConfig) {
    cfg.theme = 0;
    applyTheme(0);
    memset(cfg.backend,   0, sizeof(cfg.backend));
    memset(cfg.readerId,  0, sizeof(cfg.readerId));
    memset(cfg.companyId, 0, sizeof(cfg.companyId));
    cfg.valid = false;
  }

  // Splash
  tft.fillScreen(C_BG);
  tft.drawBitmap(LOGO_X, LOGO_Y, image_IMG_9600_bits, LOGO_W, LOGO_H, C_ACCENT);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(3);
  tft.drawString("TIMBRY", HDR_TXT_X, HDR_TXT_Y);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(1);
  tft.drawString("v" FW_VERSION, HDR_TXT_X, 50);
  delay(2000);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  rfidInit();
  loadQueue();

  if (!cfg.valid) {
    Serial.println("Nessuna config → provisioning");
    startProvisioning();
    return;
  }

  Serial.printf("Backend: %s\nReader:  %s\nCompany: %s\nTema:    %d (%s)\nQueue:   %d\n",
    cfg.backend, cfg.readerId, cfg.companyId,
    cfg.theme, cfg.theme == 0 ? "Scuro" : "Chiaro", g_queueSize);

  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT, C_BG); tft.setTextSize(2);
  tft.drawString("Connessione WiFi...", 20, 140);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  // Abilita canali 1-13 (router italiani usano spesso ch 12-13)
  wifi_country_t country = {"IT", 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL};
  esp_wifi_set_country(&country);
  // Usa SSID+PSK espliciti per evitare il BSSID lock di WiFiManager
  { String ssid = WiFi.SSID(), psk = WiFi.psk();
    Serial.printf("WiFi SSID: '%s'\n", ssid.c_str());
    if (ssid.length() > 0) WiFi.begin(ssid.c_str(), psk.c_str());
    else WiFi.begin(); }
  unsigned long s = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - s < 30000) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nWiFi status: %d\n", WiFi.status());

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WIFI OK - IP: %s\n", WiFi.localIP().toString().c_str());
    rfidInit(); beepOk();
    // NTP prima di TLS: senza ora valida la verifica cert fallisce
    if (syncNTP()) { g_ntpSynced = true; }
    else showWaitingNtp();
    g_lastHeartbeat = millis() - HEARTBEAT_MS;
    sendHeartbeat();
    startQueueFlush();
    if (g_ntpSynced) showIdle();
  } else {
    Serial.println("WIFI OFFLINE - modalita offline attiva");
    g_wifiOffline = true; beepErr(); rfidInit(); showWaitingNtp();
  }
}

// ── LOOP ─────────────────────────────
void taskResult() {
  if (g_resultTimer == 0) return;
  if (millis() - g_resultTimer >= g_resultTimeout) {
    g_resultTimer = 0; showIdle();
  }
}

void taskRfidHealth() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  g_rfidOk = (v != 0x00 && v != 0xFF);
  if (!g_rfidOk) { Serial.println("RFID drift, reinit..."); rfidInit(); }
}

void taskAdmin() {
  if (!g_adminMode) return;
  if (millis() - g_adminTimer >= ADMIN_TIMEOUT_MS) {
    g_adminMode = false;
    Serial.println("ADMIN timeout");
    showIdle();
  }
}

void loop() {
  taskSerial();
  taskWifi();
  taskNtpRetry();
  updateClock();
  taskResult();
  taskAdmin();
  taskRfid();
  sendHeartbeat();
  taskQueueFlush();
  taskRfidHealth();
  delay(50);
}
