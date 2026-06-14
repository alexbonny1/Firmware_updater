/*
 * timbry TEST Firmware
 * ESP32-WROOM + RC522 + ILI9488 TFT 480x320 + Buzzer
 *
 * Sequenza di test:
 *   1. TEST DISPLAY  → cicla tutti i colori (1s ciascuno)
 *   2. TEST BUZZER   → scala di frequenze dal grave all'acuto
 *   3. TEST RFID     → aspetta tag, lo legge, visualizza UID e bip
 *                      poi torna ad aspettare (loop continuo)
 *
 * OTA: stessa logica del firmware principale — heartbeat ogni 60s
 *      verso il backend salvato in NVS; aggiorna se la versione
 *      restituita differisce da FW_VERSION.
 *
 * PIN MAP:
 *   TFT ILI9488:  MOSI→23 / SCK→18 / CS→15 / DC→2 / RST→4 / BL→32
 *   RC522:        MOSI→23 / MISO→19 / SCK→18 / SS→21 / RST→22
 *   BUZZER:       GPIO 33
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

// ── PIN ──────────────────────────────
#define PIN_RC522_SS    21
#define PIN_RC522_RST   22
#define PIN_RC522_MISO  19
#define PIN_RC522_MOSI  23
#define PIN_RC522_SCK   18
#define BUZZER_PIN      33
#define TFT_BL_PIN      32

// ── CONFIG ───────────────────────────
#define FW_VERSION     "test-1.0"
#define PREF_NAMESPACE "timrbry"
#define HEARTBEAT_MS   30000UL

// ── COSTANTI COLORI ──────────────────
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_YELLOW  0xFFE0
#define C_CYAN    0x07FF
#define C_MAGENTA 0xF81F
#define C_ORANGE  0xFC00
#define C_TIMBRY  0x051F
#define C_GRAY    0x8410
#define C_BORD    0x4000
#define C_TEAL    0x0410

// ── OGGETTI ──────────────────────────
MFRC522  rfid(PIN_RC522_SS, PIN_RC522_RST);
TFT_eSPI tft = TFT_eSPI();
Preferences prefs;

// ── STATO GLOBALE ────────────────────
char          g_uid[64];
unsigned long g_lastHeartbeat  = 0;
unsigned long g_lastOtaTick    = 0;
String        g_backendUrl     = "";
String        g_readerId       = "";
String        g_companyId      = "";
String        g_sede           = "";
bool          g_rfidOk         = false;

// ── TEST COLORI ──────────────────────
struct ColorEntry { uint16_t color; const char* name; };
static const ColorEntry COLORS[] = {
  { C_BLACK,   "NERO"       },
  { C_WHITE,   "BIANCO"     },
  { C_RED,     "ROSSO"      },
  { C_GREEN,   "VERDE"      },
  { C_BLUE,    "BLU"        },
  { C_YELLOW,  "GIALLO"     },
  { C_CYAN,    "CIANO"      },
  { C_MAGENTA, "MAGENTA"    },
  { C_ORANGE,  "ARANCIO"    },
  { C_TIMBRY,  "TIMBRY BLU" },
  { C_GRAY,    "GRIGIO"     },
  { C_BORD,    "BORDEAUX"   },
  { C_TEAL,    "TEAL"       },
};
#define COLOR_COUNT       13
#define COLOR_DURATION_MS 1200

// ── TEST BUZZER ──────────────────────
static const int BUZZER_FREQS[] = {
  200, 300, 400, 500, 700, 900, 1100, 1400, 1600,
  1800, 2000, 2200, 2500, 2800, 3200
};
#define BUZZER_FREQ_COUNT 15
#define BUZZER_STEP_MS    650

// ── FASI ─────────────────────────────
enum Phase { PHASE_COLOR, PHASE_BUZZER, PHASE_RFID };
Phase         g_phase      = PHASE_COLOR;
int           g_step       = 0;
unsigned long g_stepTimer  = 0;

// ─────────────────────────────────────
// UTILITÀ
// ─────────────────────────────────────

// Sceglie colore testo contrastante per qualsiasi sfondo RGB565
static uint16_t contrastColor(uint16_t bg) {
  uint8_t r = (bg >> 11) & 0x1F;
  uint8_t g = (bg >> 5)  & 0x3F;
  uint8_t b = (bg >> 0)  & 0x1F;
  // Approssimazione ITU-R BT.601 per RGB565 (r∈[0,31], g∈[0,63], b∈[0,31])
  uint32_t lum = (uint32_t)r * 54 + (uint32_t)g * 183 + (uint32_t)b * 18;
  return (lum > 12000) ? C_BLACK : C_WHITE;
}

void loadConfig() {
  prefs.begin(PREF_NAMESPACE, true);
  g_backendUrl = prefs.getString("backend",   "");
  g_readerId   = prefs.getString("readerId",  "");
  g_companyId  = prefs.getString("companyId", "");
  g_sede       = prefs.getString("sede",      "");
  prefs.end();
}

// versionReg letto durante l'init — mostrato sul display per diagnosi
byte g_rfidVersion = 0x00;

// ─────────────────────────────────────
// RFID
// ─────────────────────────────────────
void rfidInit() {
  SPI.begin(PIN_RC522_SCK, PIN_RC522_MISO, PIN_RC522_MOSI, PIN_RC522_SS);

  pinMode(PIN_RC522_RST, OUTPUT);
  digitalWrite(PIN_RC522_RST, LOW);
  delay(10);
  digitalWrite(PIN_RC522_RST, HIGH);
  delay(50);

  rfid.PCD_Init();
  delay(50);

  g_rfidVersion = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  g_rfidOk = (g_rfidVersion != 0x00 && g_rfidVersion != 0xFF);
  Serial.printf("RC522 v=0x%02X %s\n", g_rfidVersion, g_rfidOk ? "OK" : "WARN");
}

// ─────────────────────────────────────
// BUZZER
// ─────────────────────────────────────
void beepRead() {
  tone(BUZZER_PIN, 1800, 80);
  delay(100);
  tone(BUZZER_PIN, 2400, 80);
}

// ─────────────────────────────────────
// OTA
// ─────────────────────────────────────
static String resolveOtaUrl(const String& startUrl) {
  const char* hdrKeys[] = {"Location"};
  String url = startUrl;
  for (int hop = 0; hop < 3; hop++) {
    String loc = "";
    if (url.startsWith("https")) {
      WiFiClientSecure c; c.setCACert(ROOT_CA);
      HTTPClient h;
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

void doOTA(const String& url, const String& newVersion) {
  Serial.printf("OTA: %s → %s\n", FW_VERSION, newVersion.c_str());
  tft.fillScreen(C_BLACK);
  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(40, 100); tft.print("Aggiornamento OTA");
  tft.setCursor(40, 130); tft.printf("Versione: %s", newVersion.c_str());
  tft.setCursor(40, 160); tft.print("Non spegnere...");

  String finalUrl = resolveOtaUrl(url);
  httpUpdate.rebootOnUpdate(true);
  t_httpUpdate_return ret;
  if (finalUrl.startsWith("https")) {
    WiFiClientSecure c; c.setCACert(ROOT_CA);
    ret = httpUpdate.update(c, finalUrl);
  } else {
    WiFiClient c;
    ret = httpUpdate.update(c, finalUrl);
  }
  // Arriva qui solo se OTA fallita
  Serial.printf("OTA FALLITO (%d): %s\n",
    httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
  tft.fillScreen(C_BLACK);
  tft.setTextColor(C_RED, C_BLACK); tft.setTextSize(2);
  tft.setCursor(40, 140); tft.print("OTA FALLITO");
  delay(3000);
}

void taskHeartbeat() {
  if (millis() - g_lastHeartbeat < HEARTBEAT_MS) return;
  g_lastHeartbeat = millis();
  if (g_backendUrl.length() == 0 || WiFi.status() != WL_CONNECTED) return;

  // Stesso payload POST del firmware principale
  char payload[256];
  snprintf(payload, sizeof(payload),
    "{\"reader_id\":\"%s\",\"company_id\":\"%s\",\"firmware\":\"%s\","
    "\"queue\":0,\"sede\":\"%s\",\"nfc_ok\":%s,\"display_ok\":true}",
    g_readerId.c_str(), g_companyId.c_str(), FW_VERSION,
    g_sede.c_str(), g_rfidOk ? "true" : "false");

  char url[256];
  snprintf(url, sizeof(url), "%s/api/hardware/ping", g_backendUrl.c_str());
  String otaUrl = "", otaVersion = "";
  int pingCode = -1;

  auto handleResp = [&](HTTPClient& h, int code) {
    pingCode = code;
    Serial.printf("PING: %d\n", code);
    if (code == 200) {
      StaticJsonDocument<256> doc;
      if (!deserializeJson(doc, h.getString())) {
        otaUrl     = doc["ota_url"]     | "";
        otaVersion = doc["ota_version"] | "";
      }
    }
  };

  if (g_backendUrl.startsWith("https")) {
    WiFiClientSecure c; c.setCACert(ROOT_CA); HTTPClient h;
    if (h.begin(c, url)) {
      h.setTimeout(10000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json");
      handleResp(h, h.POST(payload)); h.end();
    }
  } else {
    WiFiClient c; HTTPClient h;
    if (h.begin(c, url)) {
      h.setTimeout(10000); h.setReuse(false);
      h.addHeader("Content-Type", "application/json");
      handleResp(h, h.POST(payload)); h.end();
    }
  }

  // Se ping fallisce, riprova prima (10s)
  if (pingCode <= 0) {
    unsigned long _now = millis();
    g_lastHeartbeat = (_now >= HEARTBEAT_MS - 10000UL)
                      ? _now - HEARTBEAT_MS + 10000UL
                      : 0UL;
  }

  if (otaUrl.length() > 0 && otaVersion.length() > 0 && otaVersion != FW_VERSION)
    doOTA(otaUrl, otaVersion);
}

// ─────────────────────────────────────
// TEST: DISPLAY COLORI
// ─────────────────────────────────────
void drawColorScreen(int idx) {
  uint16_t col  = COLORS[idx].color;
  uint16_t txt  = contrastColor(col);
  uint16_t acc  = (col == C_WHITE || col == C_YELLOW) ? C_BLUE : C_YELLOW;

  tft.fillScreen(col);
  tft.setTextColor(acc, col); tft.setTextSize(3);
  tft.setCursor(70, 80);
  tft.print("TEST DISPLAY");

  tft.setTextColor(txt, col); tft.setTextSize(4);
  tft.setCursor(120, 140);
  tft.print(COLORS[idx].name);

  tft.setTextColor(txt, col); tft.setTextSize(2);
  tft.setCursor(70, 220);
  tft.printf("%d / %d", idx + 1, COLOR_COUNT);
}

void startColorTest() {
  g_step      = 0;
  g_stepTimer = millis();
  drawColorScreen(0);
}

void runColorTest() {
  if (millis() - g_stepTimer < COLOR_DURATION_MS) return;
  g_step++;
  if (g_step >= COLOR_COUNT) {
    g_phase = PHASE_BUZZER;
    startBuzzerTest();
    return;
  }
  g_stepTimer = millis();
  drawColorScreen(g_step);
}

// ─────────────────────────────────────
// TEST: BUZZER FREQUENZE
// ─────────────────────────────────────
void drawBuzzerScreen(int idx) {
  tft.fillRect(0, 120, 480, 120, C_BLACK);
  tft.setTextColor(C_CYAN, C_BLACK); tft.setTextSize(3);
  tft.setCursor(60, 130);
  tft.printf("%d Hz", BUZZER_FREQS[idx]);

  tft.setTextColor(C_GRAY, C_BLACK); tft.setTextSize(2);
  tft.setCursor(60, 180);
  tft.printf("Step %d / %d", idx + 1, BUZZER_FREQ_COUNT);

  // Barra progresso
  int barW = (int)((float)(idx + 1) / BUZZER_FREQ_COUNT * 360);
  tft.fillRect(60, 215, 360, 12, C_GRAY);
  tft.fillRect(60, 215, barW, 12, C_TIMBRY);
}

void startBuzzerTest() {
  g_step      = 0;
  g_stepTimer = millis();

  tft.fillScreen(C_BLACK);
  tft.setTextColor(C_YELLOW, C_BLACK); tft.setTextSize(3);
  tft.setCursor(80, 60);
  tft.print("TEST BUZZER");
  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(80, 105);
  tft.print("Scala di frequenze...");

  drawBuzzerScreen(0);
  tone(BUZZER_PIN, BUZZER_FREQS[0], 500);
}

void runBuzzerTest() {
  if (millis() - g_stepTimer < BUZZER_STEP_MS) return;
  g_step++;
  if (g_step >= BUZZER_FREQ_COUNT) {
    g_phase = PHASE_RFID;
    startRfidTest();
    return;
  }
  g_stepTimer = millis();
  drawBuzzerScreen(g_step);
  tone(BUZZER_PIN, BUZZER_FREQS[g_step], 500);
}

// ─────────────────────────────────────
// TEST: RFID READER
// ─────────────────────────────────────
void updateOtaFooter();  // forward declaration

void startRfidTest() {
  tft.fillScreen(C_BLACK);

  tft.setTextColor(C_TIMBRY, C_BLACK); tft.setTextSize(4);
  tft.setCursor(100, 50);
  tft.print("TEST RFID");

  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(50, 135);
  tft.print("Avvicina un tag NFC...");

  tft.setTextColor(C_GRAY, C_BLACK); tft.setTextSize(2);
  tft.setCursor(50, 165);
  tft.print("(qualsiasi tessera/tag)");

  // Rettangolo stato lettore
  tft.drawRoundRect(170, 215, 140, 50, 8, C_TIMBRY);
  tft.setTextColor(C_TIMBRY, C_BLACK); tft.setTextSize(2);
  tft.setCursor(193, 232);
  tft.print("IN ATTESA");

  // Riga versione RC522
  tft.setTextSize(1);
  uint16_t rcCol = g_rfidOk ? C_GREEN : C_YELLOW;
  tft.setTextColor(rcCol, C_BLACK);
  tft.setCursor(50, 285);
  tft.printf("RC522 v=0x%02X", g_rfidVersion);

  // Footer OTA (aggiornato ogni secondo da updateOtaFooter)
  g_lastOtaTick = millis();
  updateOtaFooter();
}

// Aggiorna la riga OTA in fondo allo schermo senza ridisegnare tutto
void updateOtaFooter() {
  unsigned long elapsed   = millis() - g_lastHeartbeat;
  unsigned long secToNext = (elapsed >= HEARTBEAT_MS) ? 0UL : (HEARTBEAT_MS - elapsed) / 1000;

  tft.fillRect(0, 298, 480, 22, C_BLACK);
  tft.setTextSize(1);

  if (g_backendUrl.length() == 0) {
    tft.setTextColor(C_GRAY, C_BLACK);
    tft.setCursor(50, 303);
    tft.print("OTA: backend non configurato");
  } else if (WiFi.status() != WL_CONNECTED) {
    tft.setTextColor(C_GRAY, C_BLACK);
    tft.setCursor(50, 303);
    tft.print("OTA: WiFi non connesso");
  } else {
    tft.setTextColor(C_CYAN, C_BLACK);
    tft.setCursor(50, 303);
    tft.printf("OTA check in %lus  (v=%s)", secToNext, FW_VERSION);
  }
}

void runRfidTest() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  // Leggi UID
  g_uid[0] = '\0';
  for (byte i = 0; i < rfid.uid.size; i++) {
    char hex[5];
    snprintf(hex, sizeof(hex), "%02X", rfid.uid.uidByte[i]);
    strlcat(g_uid, hex, sizeof(g_uid));
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  Serial.printf("TAG LETTO: %s\n", g_uid);
  beepRead();

  // Schermata risultato
  tft.fillScreen(C_GREEN);
  tft.setTextColor(C_BLACK, C_GREEN); tft.setTextSize(5);
  tft.setCursor(60, 60);
  tft.print("LETTO!");

  tft.setTextColor(C_BLACK, C_GREEN); tft.setTextSize(2);
  tft.setCursor(60, 150);
  tft.print("UID:");
  tft.setTextSize(3);
  tft.setCursor(60, 178);
  tft.print(g_uid);

  tft.setTextSize(2);
  tft.setCursor(60, 240);
  tft.setTextColor(0x0200, C_GREEN);
  tft.print("Avvicina ancora per rileggere");

  delay(3000);

  // Torna ad aspettare
  startRfidTest();
}

// ─────────────────────────────────────
// SETUP
// ─────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.printf("\n=== TIMBRY TEST v%s ===\n", FW_VERSION);

  // Buzzer pin idle
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // RFID prima del TFT: bus SPI vergine, nessuna interferenza da TFT_eSPI.
  // Se il chip risponde qui, significa che TFT_eSPI altera il bus dopo.
  rfidInit();
  Serial.printf("RC522 pre-TFT: v=0x%02X\n", g_rfidVersion);

  // TFT
  pinMode(TFT_BL_PIN, OUTPUT);
  digitalWrite(TFT_BL_PIN, HIGH);
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(C_BLACK);

  // Schermata di avvio
  tft.setTextColor(C_TIMBRY, C_BLACK); tft.setTextSize(5);
  tft.setCursor(100, 80);
  tft.print("TIMBRY");
  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(100, 148);
  tft.print("TEST FIRMWARE  v");
  tft.print(FW_VERSION);
  tft.setTextColor(C_GRAY, C_BLACK); tft.setTextSize(1);
  tft.setCursor(100, 175);
  tft.print("Display • Buzzer • RFID");
  delay(1500);

  // Mostra risultato RFID (init già fatto prima del TFT)
  tft.fillScreen(C_BLACK);
  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(60, 110);
  tft.print("RC522:");
  if (g_rfidOk) {
    tft.setTextColor(C_GREEN, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 160);
    tft.printf("RC522 OK  (v=0x%02X)", g_rfidVersion);
  } else if (g_rfidVersion == 0xFF) {
    tft.setTextColor(C_RED, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 160);
    tft.print("v=0xFF  MISO scollegato?");
    tft.setCursor(60, 185);
    tft.print("Verifica pin MISO GPIO19");
  } else if (g_rfidVersion == 0x00) {
    tft.setTextColor(C_RED, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 160);
    tft.print("v=0x00  SPI non comunica");
    tft.setCursor(60, 185);
    tft.print("Verifica MOSI/SCK/SS/3.3V");
  } else {
    tft.setTextColor(C_YELLOW, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 160);
    tft.printf("v=0x%02X  clone RC522", g_rfidVersion);
    tft.setCursor(60, 185);
    tft.print("Potrebbe funzionare ugualmente");
  }
  delay(3000);

  // WiFi
  tft.fillScreen(C_BLACK);
  tft.setTextColor(C_WHITE, C_BLACK); tft.setTextSize(2);
  tft.setCursor(60, 140);
  tft.print("Connessione WiFi...");

  WiFiManager wm;
  wm.setConfigPortalTimeout(120);
  wm.setSaveConfigCallback([]() {});
  if (!wm.autoConnect("timbry-test")) {
    tft.setTextColor(C_YELLOW, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 175);
    tft.print("Nessun WiFi — OTA disabilitato");
    delay(2000);
  } else {
    tft.setTextColor(C_GREEN, C_BLACK); tft.setTextSize(2);
    tft.setCursor(60, 175);
    tft.print("WiFi OK: ");
    tft.print(WiFi.localIP().toString().c_str());
    delay(1500);
  }

  loadConfig();

  // Avvia sequenza test
  g_phase = PHASE_COLOR;
  startColorTest();
}

// ─────────────────────────────────────
// LOOP
// ─────────────────────────────────────
void loop() {
  switch (g_phase) {
    case PHASE_COLOR:  runColorTest();  break;
    case PHASE_BUZZER: runBuzzerTest(); break;
    case PHASE_RFID:   runRfidTest();   break;
  }

  // Aggiorna footer OTA ogni secondo (solo nella fase RFID)
  if (g_phase == PHASE_RFID && millis() - g_lastOtaTick >= 1000) {
    g_lastOtaTick = millis();
    updateOtaFooter();
  }

  taskHeartbeat();
}
