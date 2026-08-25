#include <Arduino.h>
#include <stdarg.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <Preferences.h>

// =====================================================================
//  ARCHITEKTÚRA (v3):
//  - Core 0: motionExecTask - KIZÁRÓLAG mozgás. Egy FreeRTOS queue-ból
//            (motionQueue) pop-ol egy-egy szekvenciát/tesztet és blokkolva
//            végrehajtja. Semmi mást nem csinál, nem tudja, honnan jött
//            az elem (gomb, web-parancs, teszt - mindegy neki).
//  - Core 1 (loop()): webszerver (async, gyakorlatilag ingyen fut), OTA,
//            ÉS a gombolvasás is itt van. Mivel a gombolvasás fizikailag
//            el van választva a mozgató magtól, mozgás közben is mindig
//            fut - nincs többé "vak" időszak gombnyomásra.
//  Producer-ek (bárki dobálhat a motionQueue-ba): gombkód-feloldás,
//  webes "test"/"errorGesture"/"testMotor"/"testMagnet" parancsok.
// =====================================================================

// ============ PIN KIOSZTÁS ============
#define STEP_A_PIN     25
#define DIR_A_PIN      26
#define STEP_B_PIN     27
#define DIR_B_PIN      14
#define ENABLE_PIN     13   // közös ENABLE mindkét A4988-nak

#define MAGNET_PWM_PIN 32   // IBT-2 RPWM
#define MAGNET_EN_PIN  33   // IBT-2 R_EN (folyamatosan HIGH-on tartva)
// IBT-2 L_EN / LPWM nincs bekötve az ESP32-ről (fizikailag GND-re kötve a panelen).

// Gomb 6 = GPIO23 (belső + külső 4.7k pull-up), a többi a sémának megfelelő.
const int buttonPins[8] = {4, 5, 18, 19, 34, 23, 36, 39};
bool lastButtonState[8]  = {true, true, true, true, true, true, true, true};
bool stableState[8]      = {true, true, true, true, true, true, true, true};
unsigned long lastDebounceTime[8] = {0};
const unsigned long debounceDelay = 30;
String codeBuffer = "";

// gomb-index nevek olvashatosaghoz (0-alapu, buttonPins/stableState indexei)
const int BTN1 = 0, BTN2 = 1, BTN3 = 2, BTN4 = 3, BTN5 = 4, BTN6 = 5, BTN7 = 6, BTN8 = 7;

// ============ PLAY / EDIT MOD ============
enum AppMode : uint8_t { MODE_PLAY = 0, MODE_EDIT = 1 };
enum EditSubMode : uint8_t { EDIT_MENU = 0, EDIT_SETUP = 1, EDIT_MOVE = 3, EDIT_CELLS = 4, EDIT_PATH = 5 };
AppMode appMode = MODE_PLAY;
EditSubMode editSubMode = EDIT_MENU;

// tobbszori gyors egymas utani lenyomas szamlalasa (8x mod-valtas, 3x ESC)
unsigned long tapLastTime[8] = {0};
int tapCount[8] = {0};
const unsigned long TAP_WINDOW_MS = 700;

// az EDIT menuben a 3-as gomb ketfele jelentesenek (1x=belepes SETUP-ba, 8x=vissza
// PLAY-be) szetvalasztasahoz kell egy rovid, fuggo allapotu keslekedes - a SETUP-ba
// lepest csak akkor hajtjuk vegre, ha a keslekedesi ablak alatt nem jon meg 8 koppintas.
bool pendingSetupEntry = false;
unsigned long pendingSetupEntryTime = 0;
const unsigned long MENU_COMMIT_DELAY_MS = 450;

// Ugyanez a problema SETUP-on belul: gomb3 1x = XMAX,0 sarok tarolasa, 3x = ESC.
// A tarolast csak akkor hajtjuk vegre, ha a keslekedesi ablak alatt nem jon meg
// harmadik koppintas (lasd updatePendingMenuActions()).
bool pendingSetupCornerStore = false;
unsigned long pendingSetupCornerTime = 0;

// ============ GÉP GEOMETRIA ============
const float ANCHOR_A_X = -200, ANCHOR_A_Y = -200;
const float ANCHOR_B_X =  920, ANCHOR_B_Y = -200;

// Motor: 1.8 fok/lepes (200 lepes/fordulat). A4988 microstepping: MS2 +3.3V-ra
// kotve -> 1/4 step (MS1/MS3 GND-n). Ha az MS1/2/3 bekotes valtozik, csak ezt
// az egy konstanst kell modositani (1=full, 2=half, 4=quarter, 8=1/8, 16=1/16).
const float MICROSTEPPING = 4.0;
// Orso atmero (kozepertek, csavarodo zsinorral): 20mm -> kerulet = pi*20 =~ 62.83mm/fordulat.
const float MOTOR_STEPS_PER_REV = 200.0;
const float SPOOL_DIAMETER_MM   = 20.0;
const float STEPS_PER_MM = (MOTOR_STEPS_PER_REV * MICROSTEPPING) / (PI * SPOOL_DIAMETER_MM); // =~ 12.73

const float SPEED_TRAVEL = 50.0;   // mm/s - ures kocsi (nem penDown)
const float SPEED_CARRY  = 25.0;   // mm/s - targy szallitasa (penDown)
const float ACCEL_MM_S2  = 400.0;  // mm/s^2 - fo mozgas gyorsulasa/lassulasa
const float TESTMOTOR_SPEED_MM_S  = 20.0;  // testMotor diagnosztika sebessege
const float TESTMOTOR_ACCEL_MM_S2 = 300.0;

// ============ ELEKTROMÁGNES PWM PARAMÉTEREK (core 2.x LEDC API) ============
const int PWM_CHANNEL   = 0;
const int PWM_FREQ_HZ   = 1000;
const int PWM_RES_BITS  = 8;
const unsigned long MAGNET_BOOST_MS = 200;
const uint8_t MAGNET_HOLD_DUTY = 130;

// ============ CELLA-TÁROLÁS: LITTLEFS + NVS ============
#define MAX_STEPS            12   // queue-elemenkénti max lépésszám
#define MAX_CELL_CAPACITY    64   // fix RAM/fájl-kapacitás (processedBits is ennyi bitre épül)
#define CELL_FORMAT_VERSION  1    // ha a struktúra bővül, ezt emelni kell

struct MoveStep {
  float x, y;
  uint8_t t;   // varakozas mp, 0=azonnal, 255=vegtelen (megszakitja a szekvenciat)
  uint8_t m;   // magnes celallapot: 1=felveszi, 0=elengedi
};

struct CellDef {
  char code[16];
  uint8_t stepCount;
  MoveStep steps[MAX_STEPS];
  bool used;
};

CellDef cellCache[MAX_CELL_CAPACITY + 1]; // 1-alapú indexelés, [0] nincs hasznalva
int cellCount = 0;              // aktív cellák száma (NVS "cellNum")
uint64_t processedBits = 0;     // bit(idx-1) = 1, ha az a cella mar aktivalodott
bool cellsFormatOk = false;     // false = formátumverzió-eltérés, cellák NEM elérhetőek

Preferences prefs;

String cellFilePath(int idx) { return "/cell_" + String(idx); }

// x,y integer-kent jelenik meg, ha egesz, egyebkent 2 tizedessel - igy a
// listCell kimenete a lehető legjobban egyezik a setCell-nel beírt formával.
String formatNum(float v) {
  if (v == (long)v) return String((long)v);
  return String(v, 2);
}

String serializeSteps(const MoveStep *steps, uint8_t count) {
  String out;
  for (int i = 0; i < count; i++) {
    out += "{";
    out += formatNum(steps[i].x); out += ",";
    out += formatNum(steps[i].y); out += ",";
    out += String(steps[i].t);    out += ",";
    out += String(steps[i].m);    out += "}";
    if (i < count - 1) out += ",";
  }
  return out;
}

// Szigorú tuple-parser: "{x,y,t,m},{x,y,t,m},..." - szóköz sehol nem megengedett.
// Visszaadja a beolvasott lepesek szamat, vagy -1-et hibas formatum eseten.
int parseSteps(const String &s, MoveStep *out, uint8_t maxCount) {
  int count = 0, i = 0, n = s.length();
  while (i < n) {
    if (s[i] != '{') return -1;
    i++;
    float vals[4];
    for (int f = 0; f < 4; f++) {
      int start = i;
      while (i < n && s[i] != ',' && s[i] != '}') i++;
      if (i >= n || i == start) return -1;
      vals[f] = s.substring(start, i).toFloat();
      if (f < 3) {
        if (i >= n || s[i] != ',') return -1;
        i++;
      }
    }
    if (i >= n || s[i] != '}') return -1;
    i++;
    if (count >= maxCount) return -1; // tul sok lepes
    out[count].x = vals[0]; out[count].y = vals[1];
    out[count].t = (uint8_t)vals[2]; out[count].m = (uint8_t)vals[3];
    count++;
    if (i < n) {
      if (s[i] != ',') return -1;
      i++;
    }
  }
  return count;
}

bool isProcessed(int idx) { return (processedBits >> (idx - 1)) & 1ULL; }
void setProcessedBit(int idx) {
  processedBits |= (1ULL << (idx - 1));
  prefs.putULong64("procBits", processedBits);
}
void clearProcessedBits() {
  processedBits = 0;
  prefs.putULong64("procBits", 0);
}

bool saveCellToFile(int idx, const CellDef &c) {
  File f = LittleFS.open(cellFilePath(idx), "w");
  if (!f) return false;
  f.println(c.code);
  f.println(serializeSteps(c.steps, c.stepCount));
  f.close();
  return true;
}

bool loadCellFromFile(int idx, CellDef &out) {
  String path = cellFilePath(idx);
  out.used = false;
  if (!LittleFS.exists(path)) return false;
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  String codeLine  = f.readStringUntil('\n'); codeLine.trim();
  String stepsLine = f.readStringUntil('\n'); stepsLine.trim();
  f.close();
  if (codeLine.length() == 0) return false;
  int cnt = parseSteps(stepsLine, out.steps, MAX_STEPS);
  if (cnt <= 0) return false;
  codeLine.toCharArray(out.code, sizeof(out.code));
  out.stepCount = cnt;
  out.used = true;
  return true;
}

void loadAllCells() {
  for (int idx = 1; idx <= cellCount; idx++) {
    loadCellFromFile(idx, cellCache[idx]);
  }
}

void deleteAllCellFiles() {
  for (int idx = 1; idx <= MAX_CELL_CAPACITY; idx++) {
    String path = cellFilePath(idx);
    if (LittleFS.exists(path)) LittleFS.remove(path);
    cellCache[idx].used = false;
  }
}

int findAvailableCellByCode(const String &code) {
  for (int idx = 1; idx <= cellCount; idx++) {
    if (cellCache[idx].used && code.equals(cellCache[idx].code) && !isProcessed(idx)) {
      return idx;
    }
  }
  return -1;
}

// ============ MOZGÁS-QUEUE (Core0 fogyasztja) ============
enum QueueItemKind : uint8_t { QI_SEQUENCE = 0, QI_MOTOR_TEST = 1, QI_MAGNET_TEST = 2 };

struct QueueItem {
  QueueItemKind kind;
  unsigned long startDelayMs;   // pop utáni relatív várakozás, mielőtt elkezdi
  bool relative;                // true = errorGesture-stílusú relatív mozgás
  uint8_t stepCount;
  MoveStep steps[MAX_STEPS];
  uint8_t motorIndex;           // QI_MOTOR_TEST: 0=A, 1=B
  long testSteps;               // QI_MOTOR_TEST: lépésszám oda-vissza
  uint8_t magnetDutyPercent;    // QI_MAGNET_TEST: 0-100
  uint16_t magnetDurationSec;   // QI_MAGNET_TEST: mp
};

#define MOTION_QUEUE_LEN 8
QueueHandle_t motionQueue;

// ============ WIFI: ISMERT HÁLÓZATOK, AP-FALLBACK ============
struct WifiCred { const char* ssid; const char* password; };
const WifiCred KNOWN_NETWORKS[] = {
  { "HGPLSOFT",          "***REMOVED***" },
  { "HGPLSOFT_EXT2.4G",  "***REMOVED***" },
  { "HGPLSOFT2",         "***REMOVED***" }
};
const int KNOWN_NETWORK_COUNT = sizeof(KNOWN_NETWORKS) / sizeof(KNOWN_NETWORKS[0]);

const char* AP_SSID     = "WICKEDPAP";
const char* AP_PASSWORD = "***REMOVED***";
const char* MDNS_HOSTNAME = "wickedpickandplace";

AsyncWebServer server(80);
AsyncEventSource events("/events");

// ============ ÉLŐ WEB-LOG (SSE) ============
String logHistory = "";
const size_t LOG_HISTORY_MAX = 4000;

void webLog(const String &msg) {
  String line = "[" + String(millis()) + "] " + msg;
  Serial.println(line);
  logHistory += line + "\n";
  if (logHistory.length() > LOG_HISTORY_MAX) {
    logHistory = logHistory.substring(logHistory.length() - LOG_HISTORY_MAX);
  }
  events.send(line.c_str(), "log", millis());
}

void webLogf(const char *fmt, ...) {
  char buf[200];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  webLog(String(buf));
}

const char LOG_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>Wicked Pick and Place - Log</title>
<style>
  body { background:#111; color:#0f0; font-family:Consolas,monospace; margin:0; padding:10px; }
  #log { white-space:pre-wrap; font-size:13px; line-height:1.4; }
  h2 { color:#eee; font-family:sans-serif; }
</style></head><body>
<h2>Wicked Pick and Place - élő log</h2>
<div id="log"></div>
<script>
  const logDiv = document.getElementById('log');
  const src = new EventSource('/events');
  src.addEventListener('log', function(e) {
    logDiv.textContent += e.data + "\n";
    window.scrollTo(0, document.body.scrollHeight);
  });
</script>
</body></html>
)HTML";

// Egyszerű on-the-fly programozó felület: textarea + fetch('/cmd', POST).
const char PROGRAM_PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8"><title>Wicked Pick and Place - Program</title>
<style>
  body { background:#111; color:#ddd; font-family:Consolas,monospace; margin:0; padding:10px; }
  textarea { width:100%; height:220px; background:#000; color:#0f0; font-family:Consolas,monospace; font-size:13px; }
  #out { white-space:pre-wrap; background:#000; color:#0f0; padding:8px; margin-top:8px; min-height:120px; }
  button { padding:8px 16px; margin-top:8px; }
  .quick button { margin-right:8px; }
  h2 { color:#eee; font-family:sans-serif; }
  .hint { color:#888; font-size:12px; }
</style></head><body>
<h2>Wicked Pick and Place - cella-programozás</h2>
<p class="hint">Soronként egy parancs. Nincs szóköz a parancson kívül. Pl:<br>
setCell 1,100001,{70,70,1,1},{100,100,0,1},{680,70,255,0}<br>
listCell</p>
<textarea id="src" placeholder="setCell 1,100001,{70,70,1,1},..."></textarea><br>
<button onclick="run()">Futtat</button>
<div class="quick">
  <button onclick="quick('testMotor 0,200')">motor1</button>
  <button onclick="quick('testMotor 1,200')">motor2</button>
  <button onclick="quick('testMagnet 100,3')">magnet100</button>
  <button onclick="quick('testMagnet 50,3')">magnet50</button>
</div>
<div id="out"></div>
<script>
function send(body) {
  fetch('/cmd', { method: 'POST', body: body })
    .then(r => r.text())
    .then(t => document.getElementById('out').textContent = t)
    .catch(e => document.getElementById('out').textContent = 'HIBA: ' + e);
}
function quick(cmdLine) { send(cmdLine); }
function run() {
  const body = document.getElementById('src').value;
  send(body);
}
</script>
</body></html>
)HTML";

bool connectToKnownWifi(unsigned long timeoutMsPerNetwork = 8000) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME);
  for (int i = 0; i < KNOWN_NETWORK_COUNT; i++) {
    webLogf("WiFi probalkozas: %s", KNOWN_NETWORKS[i].ssid);
    WiFi.begin(KNOWN_NETWORKS[i].ssid, KNOWN_NETWORKS[i].password);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMsPerNetwork) {
      delay(250);
      Serial.print(".");
    }
    Serial.println();
    if (WiFi.status() == WL_CONNECTED) {
      webLogf("Csatlakozva: %s, IP: %s", KNOWN_NETWORKS[i].ssid, WiFi.localIP().toString().c_str());
      return true;
    }
    WiFi.disconnect(true);
    delay(200);
  }
  return false;
}

void startAccessPoint() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  webLogf("Sajat AP inditva: %s, IP: %s", AP_SSID, WiFi.softAPIP().toString().c_str());
}

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motorA = NULL;
FastAccelStepper *motorB = NULL;

float currentX = 680.0, currentY = 70.0;
bool penDown = false;

// ============ KINEMATIKA ============
void computeStringLengths(float x, float y, float &lenA, float &lenB) {
  float dxA = x - ANCHOR_A_X, dyA = y - ANCHOR_A_Y;
  float dxB = x - ANCHOR_B_X, dyB = y - ANCHOR_B_Y;
  lenA = sqrt(dxA * dxA + dyA * dyA);
  lenB = sqrt(dxB * dxB + dyB * dyB);
}

long mmToSteps(float mm) { return (long)round(mm * STEPS_PER_MM); }
// Sebesseg/gyorsulas mm/s (^2) -> steps/s (^2), ugyanazzal a linearis skalazassal mint a pozicio.
uint32_t mmSpeedToStepsHz(float mmPerSec)   { return (uint32_t)max(1L, (long)round(mmPerSec * STEPS_PER_MM)); }
uint32_t mmAccelToStepsS2(float mmPerSec2)  { return (uint32_t)max(1L, (long)round(mmPerSec2 * STEPS_PER_MM)); }

void moveTo(float x, float y, float speedMmPerSec) {
  float lenA, lenB, curLenA, curLenB;
  computeStringLengths(x, y, lenA, lenB);
  computeStringLengths(currentX, currentY, curLenA, curLenB);

  long targetStepsA = mmToSteps(lenA);
  long targetStepsB = mmToSteps(lenB);
  long deltaA = abs(targetStepsA - motorA->getCurrentPosition());
  long deltaB = abs(targetStepsB - motorB->getCurrentPosition());
  long maxDelta = max(deltaA, deltaB);
  if (maxDelta == 0) { currentX = x; currentY = y; return; }

  float totalLenDelta = max(abs(lenA - curLenA), abs(lenB - curLenB));
  float durationSec = totalLenDelta / speedMmPerSec;
  if (durationSec <= 0) durationSec = 0.05;

  uint32_t speedA = max(1L, (long)(deltaA / durationSec));
  uint32_t speedB = max(1L, (long)(deltaB / durationSec));

  motorA->setSpeedInHz(speedA);
  motorB->setSpeedInHz(speedB);
  motorA->setAcceleration(mmAccelToStepsS2(ACCEL_MM_S2));
  motorB->setAcceleration(mmAccelToStepsS2(ACCEL_MM_S2));
  motorA->moveTo(targetStepsA);
  motorB->moveTo(targetStepsB);

  currentX = x;
  currentY = y;
}

// Csak a Core0-motionExecTask hívja - itt a blokkolás nem gond, mert ez a mag
// KIZÁRÓLAG ezt csinálja, a gombolvasás/webszerver a másik magon fut tovább.
void moveToBlocking(float x, float y, float speed) {
  moveTo(x, y, speed);
  while (motorA->isRunning() || motorB->isRunning()) {
    delay(2);
  }
}

// ============ ELEKTROMÁGNES: BOOST-THEN-HOLD ============
unsigned long engageMagnet() {
  ledcWrite(PWM_CHANNEL, 255);
  delay(MAGNET_BOOST_MS);
  ledcWrite(PWM_CHANNEL, MAGNET_HOLD_DUTY);
  penDown = true;
  return MAGNET_BOOST_MS;
}

unsigned long releaseMagnet() {
  ledcWrite(PWM_CHANNEL, 0);
  penDown = false;
  return 0;
}

void executeResolvedStep(float targetX, float targetY, uint8_t t, uint8_t m) {
  float speed = penDown ? SPEED_CARRY : SPEED_TRAVEL;
  moveToBlocking(targetX, targetY, speed);

  delay(200);
  unsigned long extraMs = (m == 1) ? engageMagnet() : releaseMagnet();

  if (t == 255) return;
  if (t > 0) {
    long remainingMs = (long)t * 1000L - 200L - (long)extraMs;
    if (remainingMs > 0) delay(remainingMs);
  }
}

// ============ QUEUE VÉGREHAJTÓ FÜGGVÉNYEK (Core0) ============
void executeQueueSequence(const QueueItem &item) {
  float baseX = currentX, baseY = currentY; // relatív szekvenciák (errorGesture) erre vonatkoznak
  for (int i = 0; i < item.stepCount; i++) {
    const MoveStep &s = item.steps[i];
    float tx = item.relative ? (baseX + s.x) : s.x;
    float ty = item.relative ? (baseY + s.y) : s.y;
    executeResolvedStep(tx, ty, s.t, s.m);
    if (s.t == 255) break;
  }
}

// ============ MOTOR-DIAGNOSZTIKA ============
// Célja: egyértelműen szétválasztani, hogy a hiba a szoftver/könyvtár
// rétegben van-e (motor objektum, engine, enable-logika), vagy tisztán
// hardver szinten (STEP-jel nem jut ki, driver, tekercs, táp).
// - A move() visszatérési értékét is naplózzuk: ha ez false, a könyvtár már
//   a hívás pillanatában elutasította a parancsot.
// - A pozíciószámlálót (getCurrentPosition()) mozgás előtt/után kiírjuk: ha
//   ez ténylegesen elmozdul, a szoftver-oldal biztosan jó, a hiba hardver.
// A MoveResultCode enumot olvashato szoveggé alakitja a loghoz.
const char* moveResultToStr(MoveResultCode code) {
  switch (code) {
    case MOVE_OK: return "MOVE_OK";
    case MOVE_ERR_SPEED_IS_UNDEFINED: return "MOVE_ERR_SPEED_IS_UNDEFINED";
    case MOVE_ERR_NO_DIRECTION_PIN: return "MOVE_ERR_NO_DIRECTION_PIN";
#ifdef MOVE_ERR_SPEED_TOO_LOW
    case MOVE_ERR_SPEED_TOO_LOW: return "MOVE_ERR_SPEED_TOO_LOW";
#endif
    default: return "ISMERETLEN_HIBAKOD";
  }
}

void executeMotorTest(const QueueItem &item) {
  FastAccelStepper *mot = (item.motorIndex == 0) ? motorA : motorB;
  if (!mot) {
    webLogf("MOTOR TESZT HIBA: motor%d pointer NULL - az engine.stepperConnectToPin() nem sikerult a setup()-ban!", item.motorIndex);
    return;
  }

  bool wasRunningBefore = mot->isRunning();
  long posBefore = mot->getCurrentPosition();
  webLogf("Motor%d teszt inditasa: pozicio=%ld, isRunning=%d, celSteps=%ld",
          item.motorIndex, posBefore, wasRunningBefore, item.testSteps);

  mot->setSpeedInHz(mmSpeedToStepsHz(TESTMOTOR_SPEED_MM_S));
  mot->setAcceleration(mmAccelToStepsS2(TESTMOTOR_ACCEL_MM_S2));

  MoveResultCode result1 = mot->move(item.testSteps, true); // blokkolo relativ mozgas oda
  long posAfterForward = mot->getCurrentPosition();
  webLogf("Motor%d oda: move()=%s, pozicio %ld -> %ld (delta=%ld, elvart=%ld)",
          item.motorIndex, moveResultToStr(result1),
          posBefore, posAfterForward, posAfterForward - posBefore, item.testSteps);

  delay(200);

  MoveResultCode result2 = mot->move(-item.testSteps, true); // vissza
  long posAfterBack = mot->getCurrentPosition();
  webLogf("Motor%d vissza: move()=%s, pozicio %ld -> %ld (delta=%ld, elvart=%ld)",
          item.motorIndex, moveResultToStr(result2),
          posAfterForward, posAfterBack, posAfterBack - posAfterForward, -item.testSteps);

  // Osszegzo ertekeles - ez adja meg a leggyorsabb valaszt a kerdesre:
  // szoftver vagy hardver oldalon van-e a hiba.
  bool positionMovedAsExpected =
    (posAfterForward - posBefore == item.testSteps) &&
    (posAfterBack - posAfterForward == -item.testSteps);

  if (result1 != MOVE_OK || result2 != MOVE_OK) {
    webLogf("Motor%d OSSZEGZES: a konyvtar ELUTASITOTTA a mozgast - SZOFTVER/KONYVTAR oldali hiba valoszinu (engine init, motor objektum, enable-logika).", item.motorIndex);
  } else if (!positionMovedAsExpected) {
    webLogf("Motor%d OSSZEGZES: a move() OK-t adott vissza, de a pozicioszamlalo NEM a vart merteket valtozott - ez szokatlan, ellenorizd az engine/queue konfiguraciot.", item.motorIndex);
  } else {
    webLogf("Motor%d OSSZEGZES: a pozicioszamlalo pontosan a vart merteket valtozott - a SZOFTVER oldal jonak tunik, a hiba nagy valoszinuseggel HARDVER szinten van (STEP-jel, driver, tekercs, tap).", item.motorIndex);
  }
}

void executeMagnetTest(const QueueItem &item) {
  uint8_t duty = (uint8_t)((int)item.magnetDutyPercent * 255 / 100);
  ledcWrite(PWM_CHANNEL, duty);
  delay((unsigned long)item.magnetDurationSec * 1000UL);
  ledcWrite(PWM_CHANNEL, 0);
}

void motionExecTask(void *parameter) {
  QueueItem item;
  for (;;) {
    if (xQueueReceive(motionQueue, &item, portMAX_DELAY) == pdTRUE) {
      if (item.startDelayMs > 0) delay(item.startDelayMs);
      switch (item.kind) {
        case QI_SEQUENCE:    executeQueueSequence(item); break;
        case QI_MOTOR_TEST:  executeMotorTest(item);     break;
        case QI_MAGNET_TEST: executeMagnetTest(item);    break;
      }
      webLogf("Vegrehajtva. Pufferben meg: %d elem.", (int)uxQueueMessagesWaiting(motionQueue));
    }
  }
}

// ============ GOMBKÓD -> QUEUE ============
void dispatchCodeToQueue(const String &code) {
  if (!cellsFormatOk) {
    webLog("Cellak nem elerhetoek (formatumverzio-elteres) - futtass clearCells-t elobb.");
    return;
  }
  int idx = findAvailableCellByCode(code);
  if (idx < 0) {
    webLogf("Nincs elerheto (meg nem aktivalt) cella a kodhoz: %s", code.c_str());
    return;
  }
  QueueItem item = {};
  item.kind = QI_SEQUENCE;
  item.relative = false;
  item.startDelayMs = 0;
  item.stepCount = cellCache[idx].stepCount;
  memcpy(item.steps, cellCache[idx].steps, item.stepCount * sizeof(MoveStep));

  if (xQueueSend(motionQueue, &item, 0) != pdTRUE) {
    webLog("HIBA: mozgas-puffer tele, a kod elveszett.");
    return;
  }
  setProcessedBit(idx);
  webLogf("Cella #%d (kod %s) sorba allitva. Pufferben: %d elem.",
          idx, code.c_str(), (int)uxQueueMessagesWaiting(motionQueue));
}

void handleDigit(char digit) {
  codeBuffer += digit;
  webLogf("Kod: %s", codeBuffer.c_str());
  if (codeBuffer.length() >= 6) {
    String code = codeBuffer;
    codeBuffer = "";
    dispatchCodeToQueue(code);
  }
}

// =====================================================================
//  EDIT MOD - KALIBRACIOS RACS
//  A gombkod-alapu PLAY mod valtozatlan marad. Az EDIT mod egy uj,
//  fuggetlen also-szintu koordinatarendszert epit fel: a motorA ("LEFT",
//  A-horgony) es motorB ("RIGHT", B-horgony) nyers lepesszamai alapjan,
//  a SETUP-ban rogzitett 4 sarokpontbol bilinearis interpolacioval
//  szamolva - igy nem kell megbizni a hardkodolt ANCHOR_A/B / STEPS_PER_MM
//  allandokban, azokat a SETUP kalibracio valtja ki.
// =====================================================================
#define GRID_COLS        7
#define GRID_ROWS        5
#define GRID_CELL_COUNT  (GRID_COLS * GRID_ROWS)

// sarokindex: 0=(0,0) 1=(0,YMAX) 2=(XMAX,0) 3=(XMAX,YMAX)
long gridCornerA[4] = {0};
long gridCornerB[4] = {0};
uint8_t gridCornerMask = 0; // bit(i)=1, ha az i. sarok mar el van mentve

long gridCellA[GRID_CELL_COUNT] = {0};
long gridCellB[GRID_CELL_COUNT] = {0};
bool gridCellsComputed = false;

// A 4 sarok VALODI (X,Y) helye (trilateracioval szamolva a sarkok mert
// kotelhosszaibol) - ez kell ahhoz, hogy a soron kovetkezo interpolacio a
// valos sikban legyen linearis (egyenes vonalu), ne a kotelhossz-terben
// (ami ivet adna, lasd lejjebb gridTargetForUV).
float gridCornerX[4] = {0}, gridCornerY[4] = {0};
bool gridCornerXYValid = false;

float moveU = 0.5f, moveV = 0.5f; // MOVE/CELLS almod aktualis normalizalt (0..1) pozicioja

// A horgonyok (ANCHOR_A/B) fix, kezzel megmert/becsult konstansok - nincs
// automatikus horgony-illesztes (az korabban instabil volt: a horgonypoziciot
// es a step->mm eltolast egyutt oldva a legkisebb negyzetek konnyen egy
// fizikailag ertelmetlen, tavoli megoldasba futott). A pontatlan kezi SETUP
// sarok-bemerest a gridTargetForUV()/gridUVForSteps() bilinearis interpolacioja
// kezeli (a VALODI, trilateracioval visszaszamolt sarok-negyszogon dolgozik),
// nem egy elmeleti tokeletes teglalapra kenyszeritve az adatokat.

void saveGridCorners() {
  prefs.putBytes("gridCornA", gridCornerA, sizeof(gridCornerA));
  prefs.putBytes("gridCornB", gridCornerB, sizeof(gridCornerB));
  prefs.putUChar("gridCornMask", gridCornerMask);
}

void loadGridCorners() {
  size_t lenA = prefs.getBytes("gridCornA", gridCornerA, sizeof(gridCornerA));
  size_t lenB = prefs.getBytes("gridCornB", gridCornerB, sizeof(gridCornerB));
  gridCornerMask = prefs.getUChar("gridCornMask", 0);
  if (lenA != sizeof(gridCornerA) || lenB != sizeof(gridCornerB)) gridCornerMask = 0;
}

void saveGridCells() {
  prefs.putBytes("gridCellA", gridCellA, sizeof(gridCellA));
  prefs.putBytes("gridCellB", gridCellB, sizeof(gridCellB));
}

void loadGridCells() {
  size_t lenA = prefs.getBytes("gridCellA", gridCellA, sizeof(gridCellA));
  size_t lenB = prefs.getBytes("gridCellB", gridCellB, sizeof(gridCellB));
  gridCellsComputed = (lenA == sizeof(gridCellA) && lenB == sizeof(gridCellB));
}

long bilerpLong(long v00, long v10, long v01, long v11, float u, float v) {
  double top = v00 + (double)(v10 - v00) * u;
  double bot = v01 + (double)(v11 - v01) * u;
  return (long)round(top + (bot - top) * v);
}

float bilerpFloat(float v00, float v10, float v01, float v11, float u, float v) {
  float top = v00 + (v10 - v00) * u;
  float bot = v01 + (v11 - v01) * u;
  return top + (bot - top) * v;
}

// Ket kor (ax,ay es bx,by kozeppontal, lenA/lenB sugarral) metszespontjabol
// azt az (X,Y)-t adja vissza, amelyik a horgonyok "alatt" van (nagyobb Y).
// Altalanos valtozat, bar jelenleg mindig a fix ANCHOR_A/B-vel hivjuk.
bool circleIntersectBelow(float ax, float ay, float bx, float by,
                           float lenA, float lenB, float &outX, float &outY) {
  float dx = bx - ax, dy = by - ay;
  float d2 = dx * dx + dy * dy;
  float d = sqrt(d2);
  if (d < 1e-3f) return false;
  float a = (lenA * lenA - lenB * lenB + d2) / (2.0f * d);
  float h2 = lenA * lenA - a * a;
  // Kis negativ h2 meg mert-hiba miatti numerikus zaj lehet (0-nak vesszuk),
  // de ha tul negativ, a ket kor egyaltalan nem metszi egymast - HIBA (M4).
  if (h2 < -1.0f) return false;
  float h = (h2 > 0) ? sqrt(h2) : 0.0f;
  float px = ax + a * dx / d, py = ay + a * dy / d;
  float rx = -dy / d, ry = dx / d; // egysegnyi merolegesvektor az AB egyenesre
  float x1 = px + h * rx, y1 = py + h * ry;
  float x2 = px - h * rx, y2 = py - h * ry;
  if (y1 >= y2) { outX = x1; outY = y1; } else { outX = x2; outY = y2; }
  return true;
}

// step-parbol (motorA/motorB nyers pozicio) valodi (X,Y)-t szamol - fix,
// kezzel megmert ANCHOR_A/B horgonypoziciokat hasznalja (nincs horgony-
// illesztes/eltolas) - ez az EGYETLEN hely, ahol step->(X,Y) tortenik az
// EDIT-mod racs-rendszereben (M1: egyetlen igazsagforras).
bool trilaterateSteps(long stepsA, long stepsB, float &outX, float &outY) {
  float lenA = stepsA / STEPS_PER_MM;
  float lenB = stepsB / STEPS_PER_MM;
  return circleIntersectBelow(ANCHOR_A_X, ANCHOR_A_Y, ANCHOR_B_X, ANCHOR_B_Y, lenA, lenB, outX, outY);
}

// (X,Y) -> step-par, a trilaterateSteps() inverze - ugyanazt a fix ANCHOR_A/B-t
// hasznalja, mint trilaterateSteps().
void xyToSteps(float x, float y, long &outA, long &outB) {
  float lenA = sqrt(sq(x - ANCHOR_A_X) + sq(y - ANCHOR_A_Y));
  float lenB = sqrt(sq(x - ANCHOR_B_X) + sq(y - ANCHOR_B_Y));
  outA = mmToSteps(lenA);
  outB = mmToSteps(lenB);
}

// M4: ellenorzi, hogy a 4 szamolt sarok (X,Y) egy ertelmes, konvex
// negyszoget alkot-e (a kerulet menten 0->1->3->2 sorrendben, mert a
// sarokindexek: 0=(0,0) 1=(0,YMAX) 2=(XMAX,0) 3=(XMAX,YMAX)). Ha nem
// (elfajult vagy "atcsavarodott" negyszog), a kalibracio hasznalhatatlan.
bool cornersFormReasonableRect() {
  int order[4] = {0, 1, 3, 2};
  float refSign = 0;
  double area2 = 0;
  for (int k = 0; k < 4; k++) {
    int i0 = order[k], i1 = order[(k + 1) % 4], i2 = order[(k + 2) % 4];
    float ex = gridCornerX[i1] - gridCornerX[i0], ey = gridCornerY[i1] - gridCornerY[i0];
    float fx = gridCornerX[i2] - gridCornerX[i1], fy = gridCornerY[i2] - gridCornerY[i1];
    float cross = ex * fy - ey * fx;
    float s = (cross > 0) ? 1.0f : -1.0f;
    if (k == 0) refSign = s;
    else if (s != refSign) return false; // nem konvex / atcsavarodott
    area2 += (double)gridCornerX[i0] * gridCornerY[i1] - (double)gridCornerX[i1] * gridCornerY[i0];
  }
  return fabs(area2) > 100.0; // legalabb nehany cm^2 terulet, ne fajuljon el
}

// Ujraszamolja a 4 sarok valos (X,Y) helyet a mert kotelhosszakbol - hivd
// meg mindig, amikor gridCornerA/B (vagy a kalibracio) valtozik (4. sarok
// tarolasakor, auto-kalibracio utan, illetve boot-kor a betoltes utan).
void computeGridCornerXY() {
  if (gridCornerMask != 0x0F) { gridCornerXYValid = false; return; }
  gridCornerXYValid = true;
  for (int i = 0; i < 4; i++) {
    if (!trilaterateSteps(gridCornerA[i], gridCornerB[i], gridCornerX[i], gridCornerY[i])) gridCornerXYValid = false;
  }
  if (gridCornerXYValid && !cornersFormReasonableRect()) {
    gridCornerXYValid = false;
    webLog("SETUP: HIBA - a szamitott sarok-negyszog ertelmetlen (nem konvex vagy elfajult). Ellenorizd a SETUP sarkokat / az ANCHOR_A/B allandokat.");
  }
}

// u: 0=X-min .. 1=X-max oldal, v: 0=Y-min .. 1=Y-max oldal
// A cel (X,Y)-t a sarkok VALOS (nem kotelhossz-terbeli) koordinatai kozott
// interpolaljuk linearisan, majd abbol szamoljuk a lepesszamokat
// (xyToSteps) - igy a jog egyenes vonalat ir le, nem ivet.
void gridTargetForUV(float u, float v, long &outA, long &outB) {
  if (!gridCornerXYValid) {
    // biztonsagi tartalek, ha meg nincs kesz mind a 4 sarok (elvileg ide nem
    // szabadna eljutni, mert az EDIT_MOVE/CELLS csak teljes SETUP utan ertelmes).
    outA = bilerpLong(gridCornerA[0], gridCornerA[2], gridCornerA[1], gridCornerA[3], u, v);
    outB = bilerpLong(gridCornerB[0], gridCornerB[2], gridCornerB[1], gridCornerB[3], u, v);
    return;
  }
  float x = bilerpFloat(gridCornerX[0], gridCornerX[2], gridCornerX[1], gridCornerX[3], u, v);
  float y = bilerpFloat(gridCornerY[0], gridCornerY[2], gridCornerY[1], gridCornerY[3], u, v);
  xyToSteps(x, y, outA, outB);
}

// step-par -> (u,v), a gridTargetForUV() inverze (M1: single source of
// truth - ebbol frissul moveU/moveV a TENYLEGES motorpoziciobol, sose egy
// szabadon "szamolt" belso valtozobol). Mivel a sarkok (X,Y) alapjan
// bilinearisan interpolalunk, az inverz egy 2D nemlinearis egyenletrendszer
// - ezt egy rovid, veges-differencia Newton-iteracioval oldjuk meg (a
// fuggveny sima es jol viselkedik, 8 iteracio bven eleg a kivant <0.1mm
// pontossaghoz).
bool gridUVForSteps(long stepsA, long stepsB, float &outU, float &outV) {
  if (!gridCornerXYValid) return false;
  float x, y;
  if (!trilaterateSteps(stepsA, stepsB, x, y)) return false;

  float u = moveU, v = moveV; // jo kezdobecsles: az utoljara ismert (u,v)
  const float h = 0.001f;
  for (int iter = 0; iter < 8; iter++) {
    float fx = bilerpFloat(gridCornerX[0], gridCornerX[2], gridCornerX[1], gridCornerX[3], u, v) - x;
    float fy = bilerpFloat(gridCornerY[0], gridCornerY[2], gridCornerY[1], gridCornerY[3], u, v) - y;
    if (fabs(fx) < 0.05f && fabs(fy) < 0.05f) break; // 0.05mm-en belul mar eleg jo
    float fxu = (bilerpFloat(gridCornerX[0], gridCornerX[2], gridCornerX[1], gridCornerX[3], u + h, v) - x - fx) / h;
    float fyu = (bilerpFloat(gridCornerY[0], gridCornerY[2], gridCornerY[1], gridCornerY[3], u + h, v) - y - fy) / h;
    float fxv = (bilerpFloat(gridCornerX[0], gridCornerX[2], gridCornerX[1], gridCornerX[3], u, v + h) - x - fx) / h;
    float fyv = (bilerpFloat(gridCornerY[0], gridCornerY[2], gridCornerY[1], gridCornerY[3], u, v + h) - y - fy) / h;
    float det = fxu * fyv - fxv * fyu;
    if (fabs(det) < 1e-6f) break;
    float du = (fyv * fx - fxv * fy) / det;
    float dv = (fxu * fy - fyu * fx) / det;
    u -= du; v -= dv;
    u = constrain(u, -0.5f, 1.5f);
    v = constrain(v, -0.5f, 1.5f);
  }
  outU = constrain(u, 0.0f, 1.0f);
  outV = constrain(v, 0.0f, 1.0f);
  return true;
}

// A motorok TENYLEGES step-poziciojabol frissiti a szarmaztatott EDIT-mod
// allapotot (moveU/moveV) - ez az egyetlen hely, ahol ezek a valtozok
// ervenyes erteket kapnak nyers jog (SETUP), sarok-mentes vagy
// (ujra)kalibracio utan (M1: igy moveU/moveV sose fut el a valositol).
void syncStateFromMotors() {
  if (!motorA || !motorB || !gridCornerXYValid) return;
  long a = motorA->getCurrentPosition();
  long b = motorB->getCurrentPosition();
  float u, v;
  if (gridUVForSteps(a, b, u, v)) {
    moveU = u; moveV = v;
  }
}

void computeDefaultGridCells() {
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      float u = (col + 0.5f) / GRID_COLS;
      float v = (row + 0.5f) / GRID_ROWS;
      int idx = row * GRID_COLS + col;
      gridTargetForUV(u, v, gridCellA[idx], gridCellB[idx]);
    }
  }
  gridCellsComputed = true;
  saveGridCells();
}

// ============ ACK VISSZAJELZES: motor oda-vissza mozgatas gombnyomasra ============
const float ACK_SPEED_MM_S  = 40.0;
const float ACK_ACCEL_MM_S2 = 400.0;

void ackPulse(FastAccelStepper *mot, int times) {
  if (!mot) return;
  long amplitude = mmToSteps(1);
  mot->setSpeedInHz(mmSpeedToStepsHz(ACK_SPEED_MM_S));
  mot->setAcceleration(mmAccelToStepsS2(ACK_ACCEL_MM_S2));
  for (int t = 0; t < times; t++) {
    mot->move(amplitude, true);
    mot->move(-amplitude, true);
  }
}
void ackLeft(int times)  { ackPulse(motorA, times); }  // motorA = A-horgony ("LEFT")
void ackRight(int times) { ackPulse(motorB, times); }  // motorB = B-horgony ("RIGHT")

// ============ NYERS MOTOR-JOG (SETUP): nyomva tartas alatt folyamatos ============
const float JOG_SPEED_MM_S  = 80.0;
const float JOG_ACCEL_MM_S2 = 500.0;

void jogMotorStart(FastAccelStepper *mot, int dir) {
  mot->setSpeedInHz(mmSpeedToStepsHz(JOG_SPEED_MM_S));
  mot->setAcceleration(mmAccelToStepsS2(JOG_ACCEL_MM_S2));
  if (dir > 0) mot->runForward(); else mot->runBackward();
}
void jogMotorStop(FastAccelStepper *mot) {
  mot->stopMove();
}

// ============ MOVE/CELLS JOG: kalibralt racs-terben, normalizalt (u,v) ============
// M2: mindket motor a SAJAT delta-javal aranyos sebesseget kap (nem egyforma
// maximum sebesseget), igy egy szerre erkeznek a celba - ez adja az egyenes
// vonalu, nem "foghijas" mozgast (H4 javitasa).
void moveJogApply(float u, float v) {
  if (gridCornerMask != 0x0F || !motorA || !motorB) return;
  long ta, tb;
  gridTargetForUV(u, v, ta, tb);
  long curA = motorA->getCurrentPosition(), curB = motorB->getCurrentPosition();
  long dA = abs(ta - curA), dB = abs(tb - curB);
  long dMax = max(dA, dB);
  if (dMax == 0) return;
  uint32_t vMax = mmSpeedToStepsHz(JOG_SPEED_MM_S);
  uint32_t speedA = max(1UL, (unsigned long)(((uint64_t)vMax * (uint64_t)dA) / (uint64_t)dMax));
  uint32_t speedB = max(1UL, (unsigned long)(((uint64_t)vMax * (uint64_t)dB) / (uint64_t)dMax));
  motorA->setSpeedInHz(speedA);
  motorB->setSpeedInHz(speedB);
  motorA->setAcceleration(mmAccelToStepsS2(JOG_ACCEL_MM_S2));
  motorB->setAcceleration(mmAccelToStepsS2(JOG_ACCEL_MM_S2));
  motorA->moveTo(ta);
  motorB->moveTo(tb);
}

// A loop()-bol minden ciklusban hivva - amig LEFT/RIGHT/UP/DOWN nyomva van,
// periodikusan (JOG_REPLAN_MS-enkent) ujratervezi a celt, MINDIG a motorok
// TENYLEGES aktualis poziciojabol kiindulva (gridUVForSteps - M1/H5 javitas,
// nem egy szabadon "elszaladhato" belso u/v valtozobol), egy rovid
// "lookahead"-del a nyomott irany(ok)ba. Elengedeskor azonnal megallitja a
// motort es szinkronizalja az allapotot a tenyleges pozicioval.
const float JOG_LOOKAHEAD = 0.05f; // a 0..1 racs-tartomany ennyi resze / ujratervezes
const unsigned long JOG_REPLAN_MS = 30;
void updateMoveJogTick() {
  if (appMode != MODE_EDIT) return;
  if (editSubMode != EDIT_MOVE && editSubMode != EDIT_CELLS) return;

  static unsigned long lastReplanMs = 0;
  static bool jogWasActive = false;

  bool anyHeld = (stableState[BTN8] == LOW) || (stableState[BTN2] == LOW) ||
                 (stableState[BTN4] == LOW) || (stableState[BTN5] == LOW);
  if (!anyHeld) {
    if (jogWasActive) {
      motorA->stopMove();
      motorB->stopMove();
      jogWasActive = false;
      syncStateFromMotors();
    }
    return;
  }

  unsigned long now = millis();
  if (jogWasActive && (now - lastReplanMs) < JOG_REPLAN_MS) return; // meg ne tervezzunk ujra
  lastReplanMs = now;
  jogWasActive = true;

  float curU, curV;
  if (!gridUVForSteps(motorA->getCurrentPosition(), motorB->getCurrentPosition(), curU, curV)) return;

  float tgtU = curU, tgtV = curV;
  if (stableState[BTN8] == LOW) tgtU -= JOG_LOOKAHEAD; // LEFT
  if (stableState[BTN2] == LOW) tgtU += JOG_LOOKAHEAD; // RIGHT
  if (stableState[BTN4] == LOW) tgtV -= JOG_LOOKAHEAD; // UP
  if (stableState[BTN5] == LOW) tgtV += JOG_LOOKAHEAD; // DOWN
  tgtU = constrain(tgtU, 0.0f, 1.0f);
  tgtV = constrain(tgtV, 0.0f, 1.0f);
  moveU = tgtU; moveV = tgtV; // tajekoztato allapot (pl. MOVE ENTER log-jahoz)
  moveJogApply(tgtU, tgtV);
}

// Az EDIT menu SETUP-belepes (3-as gomb 1x) fuggoben tartasat zarja le, ha
// letelt a keslekedesi ablak es kozben nem erkezett meg 8 koppintas (lasd fent).
void storeCorner(int cornerIdx, const char *label);

void updatePendingMenuActions() {
  if (pendingSetupEntry && millis() - pendingSetupEntryTime >= MENU_COMMIT_DELAY_MS) {
    pendingSetupEntry = false;
    if (appMode == MODE_EDIT && editSubMode == EDIT_MENU) {
      editSubMode = EDIT_SETUP;
      webLog("EDIT almod: SETUP");
      ackRight(2);
    }
  }
  if (pendingSetupCornerStore && millis() - pendingSetupCornerTime >= MENU_COMMIT_DELAY_MS) {
    pendingSetupCornerStore = false;
    if (appMode == MODE_EDIT && editSubMode == EDIT_SETUP) {
      storeCorner(2, "(XMAX,0)");
    }
  }
}

// ============ EDIT ALMOD: SETUP ============
void storeCorner(int cornerIdx, const char *label) {
  gridCornerA[cornerIdx] = motorA->getCurrentPosition();
  gridCornerB[cornerIdx] = motorB->getCurrentPosition();
  gridCornerMask |= (1 << cornerIdx);
  saveGridCorners();
  webLogf("SETUP: sarok '%s' tarolva (A=%ld, B=%ld)", label, gridCornerA[cornerIdx], gridCornerB[cornerIdx]);
  ackRight(1);
  if (gridCornerMask == 0x0F) {
    computeGridCornerXY();
    computeDefaultGridCells();
    syncStateFromMotors();
    if (gridCornerXYValid) {
      webLogf("SETUP: mind a 4 sarok megvan - cellaracs (ujra)szamolva. Sarkok (X,Y) mm: (0,0)=(%.1f,%.1f) (0,YMAX)=(%.1f,%.1f) (XMAX,0)=(%.1f,%.1f) (XMAX,YMAX)=(%.1f,%.1f)",
              gridCornerX[0], gridCornerY[0], gridCornerX[1], gridCornerY[1],
              gridCornerX[2], gridCornerY[2], gridCornerX[3], gridCornerY[3]);
    } else {
      webLog("SETUP: FIGYELEM - a sarok-geometria ervenytelen (lasd fenti hibauzenet), MOVE/CELLS nem lesz elerheto.");
    }
  }
}

void setupHandlePress(int i) {
  switch (i) {
    case BTN6: storeCorner(0, "(0,0)");       break;
    case BTN7: storeCorner(1, "(0,YMAX)");    break;
    // BTN3 (XMAX,0) nincs itt: az onButtonPressed fuggoben tartja (lasd
    // pendingSetupCornerStore), mert 3x lenyomas ESC-et jelent.
    case BTN1: storeCorner(3, "(XMAX,YMAX)"); break;
    case BTN5: jogMotorStart(motorA, -1); break; // LEFT motor
    case BTN8: jogMotorStart(motorA, +1); break; // LEFT motor
    case BTN2: jogMotorStart(motorB, -1); break; // RIGHT motor
    case BTN4: jogMotorStart(motorB, +1); break; // RIGHT motor
    default: break;
  }
}

void setupHandleRelease(int i) {
  switch (i) {
    case BTN5: case BTN8: jogMotorStop(motorA); syncStateFromMotors(); break;
    case BTN2: case BTN4: jogMotorStop(motorB); syncStateFromMotors(); break;
    default: break;
  }
}

// ============ EDIT ALMOD: MOVE ============
void moveHandlePress(int i) {
  switch (i) {
    case BTN6: engageMagnet(); break; // momentary - amig nyomva tartjuk
    case BTN1:
      webLogf("MOVE: pozicio u=%.3f v=%.3f (A=%ld, B=%ld)",
              moveU, moveV, motorA->getCurrentPosition(), motorB->getCurrentPosition());
      break;
    default: break; // BTN8/2/4/5 jog - lasd updateMoveJogTick()
  }
}

void moveHandleRelease(int i) {
  if (i == BTN6) releaseMagnet();
}

// ============ EDIT ALMOD: CELLS ============
void cellsStoreNearest() {
  if (gridCornerMask != 0x0F) { webLog("CELLS: HIBA - a SETUP meg nincs kesz."); return; }
  long curA = motorA->getCurrentPosition();
  long curB = motorB->getCurrentPosition();
  int best = -1;
  double bestDist = 1e18;
  for (int idx = 0; idx < GRID_CELL_COUNT; idx++) {
    double dA = (double)(curA - gridCellA[idx]);
    double dB = (double)(curB - gridCellB[idx]);
    double dist = dA * dA + dB * dB;
    if (dist < bestDist) { bestDist = dist; best = idx; }
  }
  gridCellA[best] = curA;
  gridCellB[best] = curB;
  saveGridCells();
  webLogf("CELLS: cella #%d (sor %d, oszlop %d) pozicioja frissitve (A=%ld, B=%ld).",
          best, best / GRID_COLS, best % GRID_COLS, curA, curB);
  ackRight(1);
}

void cellsHandlePress(int i) {
  switch (i) {
    case BTN6: engageMagnet(); break; // momentary - amig nyomva tartjuk
    case BTN1: cellsStoreNearest(); break;
    default: break; // BTN8/2/4/5 jog - lasd updateMoveJogTick()
  }
}

void cellsHandleRelease(int i) {
  if (i == BTN6) releaseMagnet();
}

// ============ EDIT FOMENU + MOD-VALTAS ============
// M4: MOVE/CELLS csak ervenyes racs-kalibracioval erheto el (gridCornerXYValid) -
// kulonben nem lepunk be, csak logolunk es hangjelzes/ack nelkul visszaterunk,
// hogy ne lehessen ervenytelen geometrian jogolni. PATH nem igenyli.
void enterEditSubMode(EditSubMode m, const char *name) {
  if ((m == EDIT_MOVE || m == EDIT_CELLS) && !gridCornerXYValid) {
    webLogf("EDIT: '%s' almod elutasitva - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).", name);
    return;
  }
  editSubMode = m;
  webLogf("EDIT almod: %s", name);
  ackRight(2);
  syncStateFromMotors(); // M1: moveU/moveV frissul a tenyleges motorpoziciobol
}

void onButtonPressed(int i) {
  unsigned long now = millis();
  if (now - tapLastTime[i] > TAP_WINDOW_MS) tapCount[i] = 0;
  tapCount[i]++;
  tapLastTime[i] = now;

  if (appMode == MODE_PLAY) {
    if (i == BTN6 && tapCount[BTN6] == 8) {
      appMode = MODE_EDIT;
      editSubMode = EDIT_MENU;
      codeBuffer = "";
      tapCount[BTN6] = 0;
      webLog("EDIT mod aktivalva (gomb6 x8).");
      ackLeft(2);
      return;
    }
    handleDigit('1' + i);
    return;
  }

  // MODE_EDIT
  if (editSubMode == EDIT_MENU) {
    if (i == BTN3) {
      if (tapCount[BTN3] == 8) {
        pendingSetupEntry = false;
        // H2 (reszleges javitas): a PLAY mod sajat, regi ANCHOR_A/B-alapu
        // kinematikat hasznal (valtozatlan), de legalabb a "hol vagyunk"
        // becslest frissitjuk a legjobb ismert (kalibralt) adatbol, hogy ne
        // maradjon egy teljesen stale (boot-kori) ertek.
        if (gridCornerXYValid && motorA && motorB) {
          float x, y;
          if (trilaterateSteps(motorA->getCurrentPosition(), motorB->getCurrentPosition(), x, y)) {
            currentX = x; currentY = y;
          }
        }
        appMode = MODE_PLAY;
        tapCount[BTN3] = 0;
        webLog("PLAY mod aktivalva (gomb3 x8).");
        ackRight(4);
        return;
      }
      // 1x = belepes SETUP-ba, de lehet, hogy meg jon tobb koppintas 8x-ig -
      // ezert csak fuggoben jelezzuk, lasd updatePendingMenuActions().
      pendingSetupEntry = true;
      pendingSetupEntryTime = now;
      return;
    }
    switch (i) {
      case BTN1: enterEditSubMode(EDIT_PATH,   "PATH");   break;
      case BTN2: enterEditSubMode(EDIT_CELLS,  "CELLS");  break;
      case BTN8: enterEditSubMode(EDIT_MOVE,   "MOVE");   break;
      default: break;
    }
    return;
  }

  // Kozos ESC minden almodban: gomb3 x3 egymas utan
  if (i == BTN3 && tapCount[BTN3] == 3) {
    tapCount[BTN3] = 0;
    pendingSetupCornerStore = false; // ne fusson le kesobb egy mar ervenytelenitett tarolas
    editSubMode = EDIT_MENU;
    webLog("Vissza az EDIT menube (ESC x3).");
    ackLeft(2);
    return;
  }

  // SETUP-ban a gomb3-nak is ketfele jelentese van (1x=XMAX,0 tarolas, 3x=ESC
  // fentebb), ezert ugyanugy fuggoben tartjuk, mint az EDIT menu SETUP-belepeset.
  if (editSubMode == EDIT_SETUP && i == BTN3) {
    pendingSetupCornerStore = true;
    pendingSetupCornerTime = now;
    return;
  }

  switch (editSubMode) {
    case EDIT_SETUP:  setupHandlePress(i);  break;
    case EDIT_MOVE:   moveHandlePress(i);   break;
    case EDIT_CELLS:  cellsHandlePress(i);  break;
    case EDIT_PATH:   break; // meg nincs kifejtve - csak a belepes logolodik
    default: break;
  }
}

void onButtonReleased(int i) {
  if (appMode != MODE_EDIT) return;
  switch (editSubMode) {
    case EDIT_SETUP:  setupHandleRelease(i);  break;
    case EDIT_MOVE:   moveHandleRelease(i);   break;
    case EDIT_CELLS:  cellsHandleRelease(i);  break;
    default: break;
  }
}

// Ez most a loop()-ból (Core1) fut - mozgástól teljesen fuggetlenul, igy
// mindig, mozgas kozben is azonnal reagal.
void scanButtonsAndDispatch() {
  for (int i = 0; i < 8; i++) {
    bool reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonState[i]) lastDebounceTime[i] = millis();
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != stableState[i]) {
        stableState[i] = reading;
        if (reading == LOW) {
          webLogf("Gomb %d lenyomva (GPIO%d)", i + 1, buttonPins[i]);
          onButtonPressed(i);
        } else {
          onButtonReleased(i);
        }
      }
    }
    lastButtonState[i] = reading;
  }
}

// ============ PARANCS-PARSER (szigorú, szóköz nélküli argumentumok) ============
String splitToken(const String &s, int &pos, char sep) {
  int idx = s.indexOf(sep, pos);
  String tok = (idx < 0) ? s.substring(pos) : s.substring(pos, idx);
  pos = (idx < 0) ? s.length() : idx + 1;
  return tok;
}

String cmdSetCell(const String &args) {
  if (!cellsFormatOk) return "HIBA: formatumverzio-elteres, futtass clearCells-t elobb.";
  int c1 = args.indexOf(',');
  if (c1 < 0) return "HIBA: setCell formatum: IDX,CODE,{...},...";
  int c2 = args.indexOf(',', c1 + 1);
  if (c2 < 0) return "HIBA: setCell formatum: IDX,CODE,{...},...";

  int idx = args.substring(0, c1).toInt();
  String code = args.substring(c1 + 1, c2);
  String stepsStr = args.substring(c2 + 1);

  if (idx < 1 || idx > cellCount) return "HIBA: ervenytelen index (cellNum=" + String(cellCount) + ")";
  if (code.length() == 0 || code.length() > 15) return "HIBA: ervenytelen kod";

  MoveStep steps[MAX_STEPS];
  int cnt = parseSteps(stepsStr, steps, MAX_STEPS);
  if (cnt <= 0) return "HIBA: hibas lepes-lista formatum";

  CellDef &c = cellCache[idx];
  code.toCharArray(c.code, sizeof(c.code));
  c.stepCount = cnt;
  memcpy(c.steps, steps, cnt * sizeof(MoveStep));
  c.used = true;

  if (!saveCellToFile(idx, c)) return "HIBA: LittleFS iras sikertelen";
  // uj tartalom - a processed-bitet nem bantjuk automatikusan itt, mert azt
  // kulon a clearCells vagy explicit logika kezeli; uj kod = uj cella tartalmilag ujra hasznalhato:
  processedBits &= ~(1ULL << (idx - 1));
  prefs.putULong64("procBits", processedBits);

  return "OK: cella #" + String(idx) + " mentve (kod " + code + ")";
}

String cmdTest(const String &args) {
  MoveStep steps[MAX_STEPS];
  int cnt = parseSteps(args, steps, MAX_STEPS);
  if (cnt <= 0) return "HIBA: hibas lepes-lista formatum";
  QueueItem item = {};
  item.kind = QI_SEQUENCE;
  item.relative = false;
  item.stepCount = cnt;
  memcpy(item.steps, steps, cnt * sizeof(MoveStep));
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (test). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

String cmdErrorGesture(const String &args) {
  MoveStep steps[MAX_STEPS];
  int cnt = parseSteps(args, steps, MAX_STEPS);
  if (cnt <= 0) return "HIBA: hibas lepes-lista formatum";
  QueueItem item = {};
  item.kind = QI_SEQUENCE;
  item.relative = true;
  item.stepCount = cnt;
  memcpy(item.steps, steps, cnt * sizeof(MoveStep));
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (errorGesture, relativ). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

String cmdCellNum(const String &args) {
  int n = args.toInt();
  if (n < 1 || n > MAX_CELL_CAPACITY) return "HIBA: cellNum 1.." + String(MAX_CELL_CAPACITY) + " kozott lehet";
  cellCount = n;
  prefs.putUInt("cellNum", cellCount);
  if (cellsFormatOk) loadAllCells();
  return "OK: cellNum=" + String(cellCount);
}

String cmdTestMotor(const String &args) {
  int pos = 0;
  String mStr = splitToken(args, pos, ',');
  String stepsStr = args.substring(pos);
  if (mStr.length() == 0 || stepsStr.length() == 0) return "HIBA: testMotor formatum: M,STEPS";
  int m = mStr.toInt();
  long testSteps = stepsStr.toInt();
  if (m != 0 && m != 1) return "HIBA: motor index csak 0 vagy 1 lehet";
  if (testSteps <= 0) return "HIBA: STEPS pozitiv egesz kell legyen";

  QueueItem item = {};
  item.kind = QI_MOTOR_TEST;
  item.motorIndex = (uint8_t)m;
  item.testSteps = testSteps;
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (testMotor). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

String cmdTestMagnet(const String &args) {
  int pos = 0;
  String pctStr = splitToken(args, pos, ',');
  String secStr = args.substring(pos);
  if (pctStr.length() == 0 || secStr.length() == 0) return "HIBA: testMagnet formatum: PCT,SEC";
  int pct = pctStr.toInt();
  int sec = secStr.toInt();
  if (pct < 0 || pct > 100) return "HIBA: PCT 0..100 kozott lehet";
  if (sec <= 0 || sec > 3600) return "HIBA: SEC 1..3600 kozott lehet";

  QueueItem item = {};
  item.kind = QI_MAGNET_TEST;
  item.magnetDutyPercent = (uint8_t)pct;
  item.magnetDurationSec = (uint16_t)sec;
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (testMagnet). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

String cmdListCell() {
  if (!cellsFormatOk) return "HIBA: formatumverzio-elteres, futtass clearCells-t elobb.";
  String out;
  for (int idx = 1; idx <= cellCount; idx++) {
    out += String(idx) + ",";
    if (cellCache[idx].used) {
      out += String(cellCache[idx].code) + ",";
      out += serializeSteps(cellCache[idx].steps, cellCache[idx].stepCount);
      out += isProcessed(idx) ? " [processed]" : " [ready]";
    } else {
      out += "-,-";
    }
    out += "\n";
  }
  return out;
}

String cmdClearCells() {
  deleteAllCellFiles();
  clearProcessedBits();
  prefs.putUInt("cellFmtVer", CELL_FORMAT_VERSION);
  cellsFormatOk = true;
  return "OK: minden cella torolve, formatum ujrainicializalva (v" + String(CELL_FORMAT_VERSION) + ")";
}

// M5.1: diagnosztikai lenyomat mindenrol, amit a MOVE/CELLS geometria hasznal -
// gyors hibakereseshez (nem kell ujra beeploidolni logolashoz).
String cmdGeom() {
  String out;
  out += "appMode=" + String(appMode == MODE_PLAY ? "PLAY" : "EDIT") + " editSubMode=" + String((int)editSubMode) + "\n";
  if (motorA) out += "stepsA=" + String(motorA->getCurrentPosition()) + " ";
  if (motorB) out += "stepsB=" + String(motorB->getCurrentPosition()) + "\n";
  out += "moveU=" + String(moveU, 4) + " moveV=" + String(moveV, 4) + "\n";
  out += "currentX=" + String(currentX, 2) + " currentY=" + String(currentY, 2) + " (PLAY mod, ANCHOR_A/B alapon)\n";
  out += "gridCornerMask=" + String(gridCornerMask) + " gridCornerXYValid=" + String(gridCornerXYValid ? 1 : 0) + "\n";
  for (int i = 0; i < 4; i++) {
    out += "corner" + String(i) + ": A=" + String(gridCornerA[i]) + " B=" + String(gridCornerB[i]) +
           " X=" + String(gridCornerX[i], 2) + " Y=" + String(gridCornerY[i], 2) + "\n";
  }
  out += "ANCHOR_A=(" + String(ANCHOR_A_X, 1) + "," + String(ANCHOR_A_Y, 1) + ") ANCHOR_B=(" +
         String(ANCHOR_B_X, 1) + "," + String(ANCHOR_B_Y, 1) + ")\n";
  out += "STEPS_PER_MM=" + String(STEPS_PER_MM, 4) + " MICROSTEPPING=" + String(MICROSTEPPING, 0) + "\n";
  return out;
}

String executeCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return "";
  int sp = line.indexOf(' ');
  String cmd  = (sp < 0) ? line : line.substring(0, sp);
  String args = (sp < 0) ? ""   : line.substring(sp + 1);

  if (cmd == "setCell")      return cmdSetCell(args);
  if (cmd == "test")         return cmdTest(args);
  if (cmd == "cellNum")      return cmdCellNum(args);
  if (cmd == "errorGesture") return cmdErrorGesture(args);
  if (cmd == "testMotor")    return cmdTestMotor(args);
  if (cmd == "testMagnet")   return cmdTestMagnet(args);
  if (cmd == "listCell")     return cmdListCell();
  if (cmd == "clearCells")   return cmdClearCells();
  if (cmd == "geom")         return cmdGeom();
  return "HIBA: ismeretlen parancs: " + cmd;
}


// ============ SETUP / LOOP ============
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.printf("PSRAM meret: %d bajt\n", ESP.getPsramSize());

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(MAGNET_EN_PIN, OUTPUT);
  digitalWrite(MAGNET_EN_PIN, HIGH);

  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MAGNET_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);

  for (int i = 0; i < 4; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  pinMode(buttonPins[4], INPUT);
  pinMode(buttonPins[5], INPUT_PULLUP); // GPIO23
  pinMode(buttonPins[6], INPUT);
  pinMode(buttonPins[7], INPUT);

  // ---- LittleFS + NVS init ----
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount/format sikertelen!");
  }
  prefs.begin("wpp", false);

  uint32_t storedVer = prefs.getUInt("cellFmtVer", 0);
  if (storedVer == 0) {
    // gyari/uj eszkoz - inicializaljuk az aktualis verziora
    prefs.putUInt("cellFmtVer", CELL_FORMAT_VERSION);
    storedVer = CELL_FORMAT_VERSION;
  }
  cellCount = prefs.getUInt("cellNum", 35);
  if (cellCount < 1 || cellCount > MAX_CELL_CAPACITY) cellCount = 35;
  processedBits = prefs.getULong64("procBits", 0);

  if (storedVer != CELL_FORMAT_VERSION) {
    cellsFormatOk = false;
    Serial.println("FIGYELEM: cella-formatum verzio elteres - cellak NEM toltodnek be. Futtass clearCells-t.");
  } else {
    cellsFormatOk = true;
    loadAllCells();
  }

  loadGridCorners();
  loadGridCells();
  computeGridCornerXY();
  webLogf("EDIT kalibracio: sarkak=%d/4, cellaracs=%s", __builtin_popcount(gridCornerMask),
          gridCellsComputed ? "betoltve" : "nincs");

  // ---- Mozgas-queue ----
  motionQueue = xQueueCreate(MOTION_QUEUE_LEN, sizeof(QueueItem));

  engine.init();
  motorA = engine.stepperConnectToPin(STEP_A_PIN);
  motorB = engine.stepperConnectToPin(STEP_B_PIN);

  // Diagnosztika: azonnal lássuk soros/web-logon, ha az engine nem tudta
  // csatlakoztatni valamelyik pint (pl. pin-ütközés vagy hardvertimer hiány).
  if (!motorA) Serial.println("FIGYELEM: motorA (STEP_A_PIN=25) csatlakoztatasa sikertelen!");
  if (!motorB) Serial.println("FIGYELEM: motorB (STEP_B_PIN=27) csatlakoztatasa sikertelen!");

  if (motorA) {
    motorA->setDirectionPin(DIR_A_PIN);
    motorA->setAutoEnable(false);
  }
  if (motorB) {
    motorB->setDirectionPin(DIR_B_PIN);
    motorB->setAutoEnable(false);
  }

  float lenA, lenB;
  computeStringLengths(currentX, currentY, lenA, lenB);
  if (motorA) motorA->setCurrentPosition(mmToSteps(lenA));
  if (motorB) motorB->setCurrentPosition(mmToSteps(lenB));
  syncStateFromMotors(); // M1: legjobb ismert (u,v) becsles boot-kor (ha van ervenyes kalibracio)

  Serial.println("Pick-and-place polargraph keszul (queue-alapu mozgatas, IBT-2 PWM magnes).");

  // ---- WiFi ----
  if (!connectToKnownWifi()) {
    startAccessPoint();
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    webLogf("mDNS aktiv: http://%s.local", MDNS_HOSTNAME);
  } else {
    webLog("mDNS inditasa sikertelen.");
  }

  // Motor-diagnosztika a webLog-ba is, hogy /log-on tavolrol is lathato legyen.
  webLogf("Motor diagnosztika: motorA=%s, motorB=%s, ENABLE_PIN(13)=%d",
          motorA ? "OK" : "NULL", motorB ? "OK" : "NULL", digitalRead(ENABLE_PIN));

  // ---- Webszerver ----
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html",
      "<h1>Wicked Pick and Place</h1>"
      "<p>Firmware fut.</p>"
      "<p><a href='/update'>Firmware feltoltes (OTA)</a></p>"
      "<p><a href='/log'>Elo log</a></p>"
      "<p><a href='/program'>Cella-programozas</a></p>");
  });
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", LOG_PAGE_HTML);
  });
  server.on("/program", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html", PROGRAM_PAGE_HTML);
  });

  // POST /cmd - textarea tartalma soronkent vegrehajtva. Egyszerre egy
  // kliens hasznalatara szant (statikus buffer), ami erre az eszkozre elegendo.
  static String cmdBodyAccum;
  server.on("/cmd", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      // a tenyleges valaszt az onBody kuldi, ha total==0 (ures body) ide jutunk
      if (request->contentLength() == 0) request->send(200, "text/plain", "");
    },
    NULL,
    [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
      if (index == 0) cmdBodyAccum = "";
      cmdBodyAccum += String((const char*)data).substring(0, len);
      if (index + len == total) {
        String result;
        int start = 0;
        while (start < (int)cmdBodyAccum.length()) {
          int nl = cmdBodyAccum.indexOf('\n', start);
          String line = (nl < 0) ? cmdBodyAccum.substring(start) : cmdBodyAccum.substring(start, nl);
          String r = executeCommandLine(line);
          if (r.length() > 0) result += r + "\n";
          start = (nl < 0) ? cmdBodyAccum.length() : nl + 1;
        }
        request->send(200, "text/plain", result);
      }
    }
  );

  events.onConnect([](AsyncEventSourceClient *client) {
    if (logHistory.length() > 0) {
      client->send(logHistory.c_str(), "log", millis(), 1000);
    }
  });
  server.addHandler(&events);
  ElegantOTA.begin(&server);
  server.begin();
  webLog("Webszerver elindult (port 80).");

  // ---- ArduinoOTA ----
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  ArduinoOTA.onStart([]() { webLog("ArduinoOTA: feltoltes inditva"); });
  ArduinoOTA.onEnd([]()   { webLog("ArduinoOTA: feltoltes kesz, ujraindulas..."); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("ArduinoOTA: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) { webLogf("ArduinoOTA hiba [%u]", error); });
  ArduinoOTA.begin();
  webLog("ArduinoOTA aktiv.");

  // ---- Core0: kizarolag mozgas-vegrehajtas ----
  xTaskCreatePinnedToCore(
    motionExecTask,
    "motionExecTask",
    8192,
    NULL,
    1,
    NULL,
    0   // Core 0 - csak ez fut itt
  );
}

void loop() {
  ElegantOTA.loop();
  ArduinoOTA.handle();
  scanButtonsAndDispatch(); // Core1 - fuggetlen a mozgas-magtol, mindig fut
  updateMoveJogTick();
  updatePendingMenuActions();
  delay(2);
}