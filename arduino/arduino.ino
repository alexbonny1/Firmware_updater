/*
 * timbry NFC Reader Firmware
 * ESP32-WROOM + RC522 + ILI9488 TFT 480x320 + Buzzer
 * v3.1
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
 * ─────────────────────────────────────
 * User_Setup.h:
 *   #define ILI9488_DRIVER
 *   #define TFT_MOSI 23 / TFT_SCLK 18 / TFT_CS 15
 *   #define TFT_DC 2 / TFT_RST 4
 *   #define SPI_FREQUENCY 27000000
 *
 * TAG ADMIN: 3605CA06
 *   1a lettura → mostra config (schermata admin)
 *   2a lettura entro 60s → entra in PROVISIONING (WiFi portal)
 *     Nel portale puoi:
 *       - cambiare Backend URL, Reader ID, Company ID, Tema, Debounce
 *       - scrivere "RESET" nel campo Backend per cancellare tutto
 *   timeout 60s → torna normale senza fare nulla
 *
 * COMANDI SERIALI:
 *   RESET                 → cancella config e riavvia
 *   STATUS                → stampa stato
 *   FLUSH                 → svuota coda offline
 *   DEBOUNCE <ms>         → debounce tag (500-30000)
 *   DISPLAY <ms>          → durata schermata risultato (500-10000)
 *   THEME <0-7>           → cambia tema al volo
 *   PROV                  → entra in provisioning da seriale
 *
 * TEMI (campo "Tema" nel portale):
 *   0 = Nero
 *   1 = Blu navy
 *   2 = Verde scuro
 *   3 = Viola
 *   4 = Bianco
 *   5 = Grigio
 *   6 = Bordeaux
 *   7 = Arancio scuro
 *   8 = Teal
 *
 * NOTE TEMA BIANCO/GRIGIO:
 *   Su temi chiari (Bianco, Grigio) il testo dell'orologio
 *   diventa nero e il testo data diventa blu scuro per
 *   garantire leggibilità.
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

// ── PIN ──────────────────────────────
#define PIN_RC522_SS    21
#define PIN_RC522_RST   22
#define PIN_RC522_MISO  19
#define PIN_RC522_MOSI  23
#define PIN_RC522_SCK   18
#define BUZZER_PIN      33
#define TFT_BL_PIN      32

// ── CONFIG ───────────────────────────
#define FW_VERSION          "3.2"
#define PREF_NAMESPACE      "timrbry"
#define QUEUE_MAX           100
#define HEARTBEAT_MS        60000UL
#define DEBOUNCE_DEFAULT    5000UL
#define RECONNECT_MS        10000UL
#define NTP_SERVER          "pool.ntp.org"
#define TZ_POSIX            "CET-1CEST,M3.5.0,M10.5.0/3"

// ── TAG ADMIN ────────────────────────
#define ADMIN_UID           "3605CA06"
#define ADMIN_UID_2         "F917C906"
#define ADMIN_TIMEOUT_MS    60000UL

// ── LAYOUT ───────────────────────────
#define HDR_H     40
#define FTR_Y     285
#define CLK_SIZE  8
#define CLK_SPR_W 240   // 5 chars × 6px × size 8
#define CLK_SPR_H 64    // 8px × size 8
#define CLK_SPR_X 120   // (480 - 240) / 2
#define CLK_Y     120
#define DATE_SIZE  3
#define DATE_SPR_W 180  // 10 chars × 6px × size 3
#define DATE_SPR_H 24   // 8px × size 3
#define DATE_X     150  // (480 - 180) / 2
#define DATE_Y     235

// ── COLORI FISSI ─────────────────────
#define C_WHITE    0xFFFF
#define C_BLACK    0x0000
#define C_GREEN    0x07E0
#define C_RED      0xF800
#define C_YELLOW   0xFFE0
#define C_ORANGE   0xFC00
#define C_GRAY     0x8410
#define C_CYAN     0x07FF
#define C_TIMBRY   0x051F   // azzurro ~#0099FF
#define C_DARKBLUE 0x000F   // blu scuro per testo su sfondo chiaro

// ── TEMI ─────────────────────────────
// bg        = colore sfondo principale
// header    = colore header e footer
// textClock = colore testo orologio grande
// textDate  = colore testo data
// light     = true se sfondo chiaro (adatta testo header)
struct Theme {
  uint16_t    bg;
  uint16_t    header;
  uint16_t    textClock;
  uint16_t    textDate;
  bool        light;
  const char* name;
};

static const Theme THEMES[] = {
  // bg        header    clock     date      light  nome
  { 0x0000,  0x1082,  C_WHITE,  C_CYAN,   false, "Nero"         }, // 0
  { 0x000D,  0x0010,  C_WHITE,  C_CYAN,   false, "Blu"          }, // 1
  { 0x0200,  0x0180,  C_WHITE,  C_CYAN,   false, "Verde"        }, // 2
  { 0x1008,  0x2010,  C_WHITE,  C_CYAN,   false, "Viola"        }, // 3
  { 0xFFFF,  0xC618,  C_BLACK,  C_DARKBLUE, true, "Bianco"      }, // 4
  { 0x8C51,  0x6B4D,  C_WHITE,  C_WHITE,  false, "Grigio"       }, // 5
  { 0x4000,  0x2800,  C_WHITE,  C_YELLOW, false, "Bordeaux"     }, // 6
  { 0x6200,  0x4100,  C_WHITE,  C_YELLOW, false, "Arancio sc."  }, // 7
  { 0x0410,  0x0208,  C_WHITE,  C_CYAN,   false, "Teal"         }, // 8
};
#define THEME_COUNT 9

// Colori attivi (aggiornati da applyTheme)
uint16_t C_BG         = THEMES[0].bg;
uint16_t C_HEADER     = THEMES[0].header;
uint16_t C_CLK_TEXT   = THEMES[0].textClock;
uint16_t C_DATE_TEXT  = THEMES[0].textDate;
bool     g_themeLight = false;

// ── OGGETTI ──────────────────────────
MFRC522     rfid(PIN_RC522_SS, PIN_RC522_RST);
TFT_eSPI    tft = TFT_eSPI();
TFT_eSprite g_clockSprite = TFT_eSprite(&tft);
TFT_eSprite g_dateSprite  = TFT_eSprite(&tft);
Preferences prefs;

// ── STRUTTURE ────────────────────────
struct Config {
  char     backend[128];
  char     readerId[64];
  char     companyId[64];
  char     sede[64];
  uint8_t  theme;
  uint32_t debounce;
  uint32_t displayMs;
  bool     valid;
};
Config cfg;

struct DisplayState {
  String status;
  String dipendente;
  String orario;
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
#define NTP_RETRY_MS  10000UL

// ── QUEUE FLUSH NON-BLOCCANTE ────────
int           g_flushRd     = -1;   // -1=idle, >=0=indice corrente
int           g_flushWr     = 0;    // indice compattazione elementi falliti
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
  if (idx >= THEME_COUNT) idx = 0;
  C_BG        = THEMES[idx].bg;
  C_HEADER    = THEMES[idx].header;
  C_CLK_TEXT  = THEMES[idx].textClock;
  C_DATE_TEXT = THEMES[idx].textDate;
  g_themeLight = THEMES[idx].light;
}

// ── SPLASH ───────────────────────────
void drawSplashLogo() {
  tft.fillScreen(C_BG);

  // Card centrale: leggermente più chiara/scura del bg
  uint16_t cardBg = g_themeLight
    ? tft.alphaBlend(200, C_BG, 0xC618)   // sui temi chiari: grigio chiaro
    : (C_BG == 0x0000 ? 0x0841 : tft.alphaBlend(128, C_BG, 0x0000));

  tft.fillRect(80, 60, 320, 160, cardBg);
  tft.drawRect(80, 60, 320, 160, C_TIMBRY);

  // Badge NFC
  int16_t bx = 130, by = 90;
  tft.drawRect(bx, by, 36, 48, C_TIMBRY);
  tft.fillRect(bx+1, by+1, 34, 46, cardBg);
  tft.drawRect(bx+8, by+12, 20, 24, C_TIMBRY);
  tft.fillRect(bx+10, by+14, 16, 20, 0x051F);
  for (int i = 0; i < 3; i++) {
    tft.drawFastHLine(bx+5,  by+17+i*6, 3, C_TIMBRY);
    tft.drawFastHLine(bx+28, by+17+i*6, 3, C_TIMBRY);
  }
  tft.drawCircle(bx+36, by+24, 8,  C_TIMBRY);
  tft.drawCircle(bx+36, by+24, 15, C_TIMBRY);
  tft.drawCircle(bx+36, by+24, 22, 0x02DF);
  tft.fillRect(bx-4, by-4, 42, 56, cardBg);
  tft.drawRect(bx, by, 36, 48, C_TIMBRY);
  tft.drawRect(bx+8, by+12, 20, 24, C_TIMBRY);

  uint16_t titleColor = g_themeLight ? C_BLACK : C_WHITE;
  tft.setTextColor(titleColor, cardBg);
  tft.setTextSize(5);
  tft.setCursor(185, 100);
  tft.print("TIMBRY");

  tft.drawFastHLine(185, 138, 190, C_TIMBRY);

  tft.setTextColor(C_TIMBRY, cardBg);
  tft.setTextSize(1);
  tft.setCursor(185, 148);
  tft.print("NFC READER  v");
  tft.print(FW_VERSION);

  uint16_t subtitleColor = g_themeLight ? 0x6B4D : C_GRAY;
  tft.setTextColor(subtitleColor, C_BG);
  tft.setTextSize(1);
  tft.setCursor(170, 240);
  tft.print("Sistema Presenze NFC");
}

// ── BUZZER ───────────────────────────
void beepOk() {
  tone(BUZZER_PIN, 1800, 80); delay(100);
  tone(BUZZER_PIN, 2200, 80);
}
void beepErr() {
  tone(BUZZER_PIN, 900, 150); delay(80);
  tone(BUZZER_PIN, 700, 150);
}
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

// ── RFID INIT ────────────────────────
void rfidInit() {
  SPI.begin(PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SS);
  // RST basso → alto: forza hard reset garantendo stato noto dopo power cycle
  pinMode(PIN_RC522_RST, OUTPUT);
  digitalWrite(PIN_RC522_RST, LOW);
  delay(10);
  digitalWrite(PIN_RC522_RST, HIGH);
  delay(50); // tempo di avvio oscillatore (datasheet: ~37μs + margine)
  rfid.PCD_Init();
  delay(50);
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  g_rfidOk = (v != 0x00 && v != 0xFF); // accetta tutti i cloni (es. 0x82, 0x88, 0x12)
  Serial.printf("RC522 version: 0x%02X %s\n", v, g_rfidOk ? "OK" : "WARN");
}

// ── DISPLAY ──────────────────────────
void drawHeader() {
  tft.fillRect(0, 0, 480, HDR_H, C_HEADER);

  // Logo mini
  int16_t lx = 6, ly = 8;
  tft.drawRect(lx, ly, 12, 16, C_TIMBRY);
  tft.fillRect(lx+1, ly+1, 10, 14, C_HEADER);
  tft.drawRect(lx+3, ly+4, 6, 7, C_TIMBRY);
  tft.drawCircle(lx+12, ly+8, 4, C_TIMBRY);
  tft.drawCircle(lx+12, ly+8, 7, 0x02DF);
  tft.fillRect(lx-1, ly-1, 14, 18, C_HEADER);
  tft.drawRect(lx, ly, 12, 16, C_TIMBRY);
  tft.drawRect(lx+3, ly+4, 6, 7, C_TIMBRY);

  // Testi header: su temi chiari usa nero invece di bianco
  uint16_t hTxt = g_themeLight ? C_BLACK : C_WHITE;

  tft.setTextColor(C_TIMBRY, C_HEADER);
  tft.setTextSize(1); tft.setCursor(22, 8);
  tft.print("TIMBRY");

  tft.setTextColor(hTxt, C_HEADER);
  tft.setTextSize(1); tft.setCursor(22, 20);
  tft.print(cfg.readerId[0] ? cfg.readerId : "reader");

  tft.fillCircle(355, 20, 7, g_wifiOffline ? C_RED : C_GREEN);
  tft.setTextColor(hTxt, C_HEADER);
  tft.setTextSize(2); tft.setCursor(368, 12);
  tft.print(g_wifiOffline ? "OFFLINE" : "ONLINE ");
}

void drawFooter() {
  tft.fillRect(0, FTR_Y, 480, 320 - FTR_Y, C_HEADER);
  uint16_t hTxt = g_themeLight ? C_BLACK : C_GRAY;
  tft.setTextColor(hTxt, C_HEADER);
  tft.setTextSize(1); tft.setCursor(162, FTR_Y + 12);
  tft.print("Avvicina badge al lettore");
  if (g_queueSize > 0) {
    tft.setTextColor(C_YELLOW, C_HEADER);
    tft.setCursor(340, FTR_Y + 12);
    tft.print("Coda: "); tft.print(g_queueSize);
  }
}

void drawClock() {
  g_clockSprite.fillSprite(C_BG);
  g_clockSprite.setTextColor(C_CLK_TEXT, C_BG);
  g_clockSprite.setTextSize(CLK_SIZE);
  int16_t tw = g_clockSprite.textWidth(ds.oraCorrente.length() > 0 ? ds.oraCorrente : "00:00");
  g_clockSprite.setCursor((CLK_SPR_W - tw) / 2, 0);
  if (ds.oraCorrente.length() > 0) g_clockSprite.print(ds.oraCorrente);
  g_clockSprite.pushSprite(CLK_SPR_X, CLK_Y);
}

void drawDate() {
  if (!g_ntpSynced) return;
  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d/%02d/%04d",
    t->tm_mday, t->tm_mon + 1, t->tm_year + 1900);
  g_dateSprite.fillSprite(C_BG);
  g_dateSprite.setTextColor(C_DATE_TEXT, C_BG);
  g_dateSprite.setTextSize(DATE_SIZE);
  g_dateSprite.setCursor(0, 0);
  g_dateSprite.print(buf);
  g_dateSprite.pushSprite(DATE_X, DATE_Y);
}

void showIdle() {
  if (!g_ntpSynced) { g_waitingNtp = true; showWaitingNtp(); return; }
  g_waitingNtp  = false;
  ds.status     = "ATTESA";
  ds.dipendente = "";
  ds.orario     = "";
  tft.fillRect(0, HDR_H, 480, FTR_Y - HDR_H, C_BG);
  drawHeader(); drawClock(); drawDate(); drawFooter();
}

void showResult(String tipo, String nome, String orario) {
  ds.status     = tipo;
  ds.dipendente = nome;
  ds.orario     = orario;

  uint16_t bg, fg;
  if      (tipo == "ENTRATA") { bg = 0x0320; fg = C_GREEN;  }
  else if (tipo == "USCITA")  { bg = 0x8200; fg = C_RED;    }
  else if (tipo == "ERRORE")  { bg = 0x4000; fg = C_ORANGE; }
  else                         { bg = 0x2104; fg = C_YELLOW; }

  tft.fillRect(0, HDR_H, 480, FTR_Y - HDR_H, bg);

  tft.setTextColor(fg, bg); tft.setTextSize(6);
  int16_t sw = tipo.length() * 36;
  tft.setCursor(max((int16_t)10, (int16_t)((480 - sw) / 2)), HDR_H + 22);
  tft.print(tipo);

  if (nome.length() > 0) {
    tft.setTextColor(C_WHITE, bg); tft.setTextSize(3);
    int16_t nw = nome.length() * 18;
    tft.setCursor(max((int16_t)10, (int16_t)((480 - nw) / 2)), HDR_H + 110);
    tft.print(nome);
  }
  if (orario.length() > 0) {
    tft.setTextColor(C_YELLOW, bg); tft.setTextSize(3);
    int16_t ow = orario.length() * 18;
    tft.setCursor(max((int16_t)10, (int16_t)((480 - ow) / 2)), HDR_H + 165);
    tft.print(orario);
  }

  drawHeader(); drawFooter();
  g_resultTimer = millis();
}

// Schermata admin: mostra config e istruzioni
// 2a lettura → entrerà nel provisioning (non reset)
void drawAdmin() {
  tft.fillRect(0, HDR_H, 480, FTR_Y - HDR_H, C_BG);

  uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;

  tft.setTextColor(C_YELLOW, C_BG); tft.setTextSize(2);
  tft.setCursor(100, HDR_H + 8);
  tft.print("-- CONFIGURAZIONE --");

  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(1);
  int y = HDR_H + 32;
#define ADMIN_ROW(label, value) \
  if (y < FTR_Y - 16) { tft.setCursor(10, y); tft.print(label); tft.print(value); y += 18; }

  ADMIN_ROW("Backend: ", cfg.backend)
  ADMIN_ROW("Reader:  ", cfg.readerId)
  ADMIN_ROW("Company: ", cfg.companyId)
  if (y < FTR_Y - 16) {
    tft.setCursor(10, y); tft.print("Tema:    ");
    tft.print(cfg.theme < THEME_COUNT ? THEMES[cfg.theme].name : "?");
    tft.print(" ("); tft.print(cfg.theme); tft.print(")");
    y += 18;
  }
  if (y < FTR_Y - 16) {
    tft.setCursor(10, y); tft.print("WiFi:    ");
    if (WiFi.status() == WL_CONNECTED) {
      tft.print("OK  "); tft.print(WiFi.localIP().toString());
    } else { tft.print("OFFLINE"); }
    y += 18;
  }
  ADMIN_ROW("Queue:   ", g_queueSize)
  ADMIN_ROW("NTP:     ", g_ntpSynced ? "OK" : "NO SYNC")
  ADMIN_ROW("Time:    ", getISOTimestamp())
  ADMIN_ROW("FW:      ", FW_VERSION)
  ADMIN_ROW("Sede:    ", cfg.sede[0] ? cfg.sede : "(non impostata)")

  if (y < FTR_Y - 16) {
    tft.setCursor(10, y); tft.print("NFC:     ");
    tft.setTextColor(g_rfidOk ? C_GREEN : C_RED, C_BG);
    tft.print(g_rfidOk ? "OK" : "ERRORE");
    tft.setTextColor(bodyTxt, C_BG); y += 18;
  }
  if (y < FTR_Y - 16) {
    tft.setCursor(10, y); tft.print("Display: ");
    tft.setTextColor(g_displayOk ? C_GREEN : C_RED, C_BG);
    tft.print(g_displayOk ? "OK" : "ERRORE");
    tft.setTextColor(bodyTxt, C_BG); y += 28;
  }
#undef ADMIN_ROW

  // Istruzione 2a lettura → provisioning (non reset!)
  tft.setTextColor(C_CYAN, C_BG); tft.setTextSize(2);
  tft.setCursor(30, y);
  tft.print("Ripassare: ENTRA IN PORTALE");
  y += 22;

  tft.setTextColor(C_GRAY, C_BG); tft.setTextSize(1);
  tft.setCursor(30, y);
  tft.print("(Nel portale puoi modificare o resettare tutto)");
  y += 14;
  tft.setCursor(30, y);
  tft.print("Attendi 60s per annullare");

  drawHeader(); drawFooter();
}

void showWaitingNtp() {
  g_waitingNtp = true;
  tft.fillScreen(C_BG);
  drawHeader();
  tft.fillRect(0, FTR_Y, 480, 320 - FTR_Y, C_HEADER);

  tft.setTextColor(C_YELLOW, C_BG); tft.setTextSize(4);
  tft.setCursor(195, 65); tft.print("?:??");

  uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;
  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(2);
  tft.setCursor(110, 145); tft.print("Orario non disponibile");

  tft.setTextColor(C_YELLOW, C_BG); tft.setTextSize(1);
  tft.setCursor(180, 180);
  tft.print(g_wifiOffline ? "In attesa del WiFi..." : "Sincronizzazione NTP...");
  tft.setCursor(160, 198);
  tft.print("Le timbrature sono bloccate");

  g_lastNtpRetry = millis();
}

void taskNtpRetry() {
  if (!g_waitingNtp) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (millis() - g_lastNtpRetry < NTP_RETRY_MS) return;
  g_lastNtpRetry = millis();
  Serial.println("NTP retry...");
  if (syncNTP()) { g_ntpSynced = true; g_waitingNtp = false; showIdle(); }
  else {
    uint16_t bodyTxt = g_themeLight ? C_BLACK : C_GRAY;
    tft.fillRect(0, HDR_H + 180, 480, 30, C_BG);
    tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(1);
    tft.setCursor(90, HDR_H + 185);
    tft.print("Ultimo tentativo: "); tft.print(millis() / 1000); tft.print("s");
  }
}

void updateClock() {
  if (millis() - g_lastClockUpdate < 1000) return;
  g_lastClockUpdate = millis();

  if (!g_ntpSynced) {
    static bool lwOff2 = false;
    if (lwOff2 != g_wifiOffline) { lwOff2 = g_wifiOffline; drawHeader(); }
    return;
  }

  time_t now = time(nullptr);
  struct tm* t = localtime(&now);
  char buf[8];
  snprintf(buf, sizeof(buf), "%02d:%02d", t->tm_hour, t->tm_min);
  String newOra = String(buf);

  static bool lwOff = false;
  if (lwOff != g_wifiOffline) { lwOff = g_wifiOffline; drawHeader(); }

  if (ds.status == "ATTESA" && newOra != ds.oraCorrente) {
    ds.oraCorrente = newOra; drawClock(); drawDate();
  } else { ds.oraCorrente = newOra; }
}

// ── CONFIG NVS ───────────────────────
bool loadConfig() {
  prefs.begin(PREF_NAMESPACE, true);
  String backend    = prefs.getString("backend",    "");
  String readerId   = prefs.getString("readerId",   "");
  String companyId  = prefs.getString("companyId",  "");
  String sede       = prefs.getString("sede",       "");
  uint8_t  theme     = (uint8_t)prefs.getUInt("theme", 0);
  uint32_t debounce  = prefs.getUInt("debounce",   (uint32_t)DEBOUNCE_DEFAULT);
  uint32_t displayMs = prefs.getUInt("displayMs",  3000);
  prefs.end();
  if (backend.length() < 4 || readerId.length() < 2 || companyId.length() < 10) return false;
  strlcpy(cfg.backend,    backend.c_str(),    sizeof(cfg.backend));
  strlcpy(cfg.readerId,   readerId.c_str(),   sizeof(cfg.readerId));
  strlcpy(cfg.companyId,  companyId.c_str(),  sizeof(cfg.companyId));
  strlcpy(cfg.sede,       sede.c_str(),       sizeof(cfg.sede));
  cfg.theme      = (theme < THEME_COUNT) ? theme : 0;
  cfg.debounce   = (debounce >= 500 && debounce <= 30000) ? debounce : (uint32_t)DEBOUNCE_DEFAULT;
  cfg.displayMs  = (displayMs >= 500 && displayMs <= 10000) ? displayMs : 3000UL;
  cfg.valid      = true;
  applyTheme(cfg.theme);
  g_debouncMs    = cfg.debounce;
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
  if (g_flushRd >= 0) return;  // già in corso
  Serial.printf("FLUSH QUEUE avviato (%d elementi)...\n", g_queueSize);
  g_flushRd     = 0;
  g_flushWr     = 0;
  g_flushLastMs = millis() - 300;  // forza invio immediato del primo elemento
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
      h.addHeader("Content-Type","application/json"); rc = h.POST(g_payload); h.end(); }
  } else {
    WiFiClient c; HTTPClient h;
    if (h.begin(c, g_url)) { h.setTimeout(15000); h.setReuse(false);
      h.addHeader("Content-Type","application/json"); rc = h.POST(g_payload); h.end(); }
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
// Segue la catena di redirect (GitHub → CDN, max 3 hop) prima di passare l'URL
// a HTTPUpdate che non gestisce autonomamente i redirect.
static String resolveOtaUrl(const String& startUrl) {
  const char* hdrKeys[] = {"Location"};
  String url = startUrl;
  for (int hop = 0; hop < 3; hop++) {
    String loc = "";
    if (url.startsWith("https")) {
      WiFiClientSecure c; c.setCACert(ROOT_CA); HTTPClient h;
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

  tft.fillRect(0, HDR_H, 480, FTR_Y - HDR_H, C_BG);
  uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;
  tft.setTextColor(C_CYAN, C_BG); tft.setTextSize(3);
  tft.setCursor(50, HDR_H + 25);
  tft.print("Aggiornamento FW");
  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(2);
  tft.setCursor(60, HDR_H + 90);
  tft.print("v"); tft.print(FW_VERSION);
  tft.print(" -> v"); tft.print(newVersion);
  tft.setTextColor(C_YELLOW, C_BG); tft.setTextSize(1);
  tft.setCursor(70, HDR_H + 140);
  tft.print("Non spegnere il dispositivo...");
  drawHeader(); drawFooter();

  String finalUrl = resolveOtaUrl(url);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret;
  if (finalUrl.startsWith("https")) {
    WiFiClientSecure client; client.setCACert(ROOT_CA);
    ret = httpUpdate.update(client, finalUrl);
  } else {
    WiFiClient client;
    ret = httpUpdate.update(client, finalUrl);
  }

  // Arriva qui solo se OTA non è andata a buon fine (altrimenti si riavvia)
  Serial.printf("OTA FALLITO (%d): %s\n", httpUpdate.getLastError(),
    httpUpdate.getLastErrorString().c_str());
  tft.fillRect(0, HDR_H + 155, 480, 50, C_BG);
  tft.setTextColor(C_RED, C_BG); tft.setTextSize(2);
  tft.setCursor(80, HDR_H + 165);
  tft.print("Errore aggiornamento");
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

  // Riprova dopo 10s invece di 60s se il PING è fallito
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
      // 2a lettura → entra in provisioning (NON resetta)
      Serial.println("ADMIN → PROVISIONING");
      beepOk();
      g_adminMode = false;
      tft.fillRect(0, HDR_H, 480, FTR_Y - HDR_H, C_BG);
      uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;
      tft.setTextColor(C_CYAN, C_BG); tft.setTextSize(3);
      tft.setCursor(100, 130); tft.print("Avvio portale...");
      drawHeader(); drawFooter();
      delay(1200);
      startProvisioning();
    } else {
      // 1a lettura → mostra config
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
    tft.fillRect(0, HDR_H + 180, 480, 40, C_BG);
    tft.setTextColor(C_RED, C_BG); tft.setTextSize(2);
    int16_t tw = 18 * 14;
    tft.setCursor((480 - tw) / 2, HDR_H + 190);
    tft.print("Orario non pronto");
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
      g_lastHeartbeat = millis() - HEARTBEAT_MS; // forza PING immediato per scaldare Railway
      sendHeartbeat();
      startQueueFlush();
      if (syncNTP()) { g_ntpSynced = true; showIdle(); }
      else showWaitingNtp();
    }
    return;
  }
  if (!g_wifiOffline) {
    g_wifiOffline   = true;
    g_lastReconnect = millis();
    Serial.println("WIFI OFFLINE");
    if (!g_ntpSynced) showWaitingNtp(); else drawHeader();
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
//
// Schermata TFT: mostra SSID AP + anteprima temi
// Portale WiFiManager: Backend, Reader, Company, Tema
//
// RESET: se nel campo Backend si scrive "RESET"
//        cancella tutta la config e riavvia pulito.
//
void startProvisioning() {
  // ── Schermata TFT ──
  tft.fillScreen(C_BG);
  uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;

  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(2);
  tft.setCursor(20, 18); tft.print("Configurazione WiFi");

  tft.setTextColor(C_GRAY, C_BG); tft.setTextSize(1);
  tft.setCursor(20, 46); tft.print("Connetti al WiFi:");

  char apName[32];
  snprintf(apName, sizeof(apName), "timbry-%06X", (uint32_t)(ESP.getEfuseMac() & 0xFFFFFF));
  tft.setTextColor(C_TIMBRY, C_BG); tft.setTextSize(3);
  tft.setCursor(20, 62); tft.print(apName);

  tft.setTextColor(C_GRAY, C_BG); tft.setTextSize(1);
  tft.setCursor(20, 102); tft.print("Poi vai su: 192.168.4.1");
  tft.setCursor(20, 116);
  tft.print("Per RESET: scrivi RESET nel campo Backend");

  // Anteprima temi: 2 colonne × 5 righe (9 temi)
  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(1);
  tft.setCursor(20, 135); tft.print("Temi (campo Tema nel portale):");

  int cols = 2;
  int startY = 148;
  int rowH   = 18;
  int col0x  = 20;
  int col1x  = 250;

  for (int i = 0; i < THEME_COUNT; i++) {
    int col  = i % cols;
    int row  = i / cols;
    int bx   = (col == 0) ? col0x : col1x;
    int by2  = startY + row * rowH;

    // Campione sfondo + header
    tft.fillRect(bx,    by2, 14, 10, THEMES[i].bg);
    tft.drawRect(bx,    by2, 14, 10, C_GRAY);
    tft.fillRect(bx+16, by2, 14, 10, THEMES[i].header);
    tft.drawRect(bx+16, by2, 14, 10, C_GRAY);

    // Numero + nome
    tft.setTextColor(bodyTxt, C_BG);
    tft.setCursor(bx + 34, by2 + 2);
    tft.print(i); tft.print("="); tft.print(THEMES[i].name);

    // Marcatore tema attuale
    if (i == cfg.theme) {
      tft.setTextColor(C_TIMBRY, C_BG);
      tft.print(" <");
    }
  }

  // ── Portale WiFiManager ──
  WiFiManager wm;
  wm.setConfigPortalTimeout(300);

  WiFiManagerParameter p_b("backend", "Backend URL",     cfg.backend,   127);
  WiFiManagerParameter p_r("reader",  "Reader ID",       cfg.readerId,   63);
  WiFiManagerParameter p_c("company", "Company ID",      cfg.companyId,  63);
  WiFiManagerParameter p_s("sede",    "Sede / Ubicazione", cfg.sede,     63);
  char themeStr[4];
  snprintf(themeStr, sizeof(themeStr), "%d", cfg.theme);
  WiFiManagerParameter p_t("theme",
    "Tema: 0=Nero 1=Blu 2=Verde 3=Viola 4=Bianco 5=Grigio 6=Bordeaux 7=Arancio 8=Teal",
    themeStr, 2);

  char debounceStr[8];
  snprintf(debounceStr, sizeof(debounceStr), "%lu", (unsigned long)cfg.debounce);
  WiFiManagerParameter p_d("debounce", "Debounce tag (ms, 500-30000)", debounceStr, 6);

  // Bottone reset: compila il campo backend con "RESET" e invia il form
  WiFiManagerParameter p_reset(
    "<br><hr style='margin:16px 0'>"
    "<p style='color:#c00;font-weight:bold;margin-bottom:6px'>Reset completo (cancella config e code offline):</p>"
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

  // ── Leggi valori ──
  const char* newBackend = p_b.getValue();

  // Controlla RESET
  if (strcasecmp(newBackend, "RESET") == 0) {
    Serial.println("RESET richiesto dal portale");
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED, C_BG); tft.setTextSize(4);
    tft.setCursor(140, 130); tft.print("RESET...");
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

  int themeVal = atoi(p_t.getValue());
  cfg.theme = (themeVal >= 0 && themeVal < THEME_COUNT) ? (uint8_t)themeVal : 0;

  unsigned long debounceVal = strtoul(p_d.getValue(), nullptr, 10);
  cfg.debounce = (debounceVal >= 500 && debounceVal <= 30000)
                 ? (uint32_t)debounceVal
                 : (uint32_t)DEBOUNCE_DEFAULT;
  g_debouncMs = cfg.debounce;

  // Rimuovi trailing slash
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
    Serial.printf("Tema:     %d (%s)\n", cfg.theme,
      cfg.theme < THEME_COUNT ? THEMES[cfg.theme].name : "?");
    Serial.printf("Queue:    %d\n", g_queueSize);
    Serial.printf("WiFi:     %s\n", WiFi.status()==WL_CONNECTED?"OK":"OFFLINE");
    Serial.printf("IP:       %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("NTP:      %s\n", g_ntpSynced?"OK":"NO SYNC");
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
    if (t >= 0 && t < THEME_COUNT) {
      cfg.theme = (uint8_t)t;
      applyTheme(cfg.theme);
      saveConfig();
      Serial.printf("THEME → %d (%s)\n", cfg.theme, THEMES[cfg.theme].name);
      if (g_waitingNtp) showWaitingNtp(); else showIdle();
    } else {
      Serial.printf("Tema non valido (0-%d)\n", THEME_COUNT - 1);
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
  g_clockSprite.createSprite(CLK_SPR_W, CLK_SPR_H);
  g_dateSprite.createSprite(DATE_SPR_W, DATE_SPR_H);
  g_displayOk = (tft.width() == 480 && tft.height() == 320);
  if (!g_displayOk) Serial.println("WARN: display dimensioni inattese");

  // Carica config (e tema) prima della splash
  bool hasConfig = loadConfig();
  if (!hasConfig) {
    cfg.theme = 0;
    applyTheme(0);
    memset(cfg.backend,   0, sizeof(cfg.backend));
    memset(cfg.readerId,  0, sizeof(cfg.readerId));
    memset(cfg.companyId, 0, sizeof(cfg.companyId));
    cfg.valid = false;
  }

  tft.fillScreen(C_BG);
  drawSplashLogo();
  delay(2500);

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
    cfg.theme, THEMES[cfg.theme].name, g_queueSize);

  tft.fillScreen(C_BG);
  uint16_t bodyTxt = g_themeLight ? C_BLACK : C_WHITE;
  tft.setTextColor(bodyTxt, C_BG); tft.setTextSize(2);
  tft.setCursor(20, 100); tft.print("Connessione WiFi...");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  // Imposta country code IT: abilita canali 1-13 (router italiani usano
  // spesso ch 12-13 in "auto" che l'ESP32 con country USA non vede)
  wifi_country_t country = {"IT", 1, 13, 20, WIFI_COUNTRY_POLICY_MANUAL};
  esp_wifi_set_country(&country);
  // Usa SSID+PSK espliciti invece di WiFi.begin() per evitare il BSSID
  // lock salvato da WiFiManager (causa problemi con mesh e router che
  // cambiano canale)
  { String ssid = WiFi.SSID(), psk = WiFi.psk();
    Serial.printf("WiFi SSID: '%s'\n", ssid.c_str());
    if (ssid.length() > 0) WiFi.begin(ssid.c_str(), psk.c_str());
    else WiFi.begin(); }
  unsigned long s = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - s < 30000) {
    delay(500); Serial.print(".");
  }
  Serial.printf("\nWiFi status: %d\n", WiFi.status());
  // status: 3=OK 1=SSID non trovato 4=password sbagliata 6=disconnesso

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WIFI OK - IP: %s\n", WiFi.localIP().toString().c_str());
    rfidInit(); beepOk();
    g_lastHeartbeat = millis() - HEARTBEAT_MS; // forza PING immediato per scaldare Railway
    sendHeartbeat();
    startQueueFlush();
    if (syncNTP()) { g_ntpSynced = true; showIdle(); }
    else showWaitingNtp();
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
