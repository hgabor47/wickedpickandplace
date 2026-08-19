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
unsigned long lastDebounceTime[8] = {0};
const unsigned long debounceDelay = 30;
String codeBuffer = "";

// ============ GÉP GEOMETRIA ============
const float ANCHOR_A_X = -200, ANCHOR_A_Y = -200;
const float ANCHOR_B_X =  920, ANCHOR_B_Y = -200;
const float STEPS_PER_MM = 80.0;

const float SPEED_TRAVEL = 120.0;
const float SPEED_CARRY  = 60.0;

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
  motorA->setAcceleration(4000);
  motorB->setAcceleration(4000);
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

void executeMotorTest(const QueueItem &item) {
  FastAccelStepper *mot = (item.motorIndex == 0) ? motorA : motorB;
  if (!mot) return;
  mot->setSpeedInHz(2000);
  mot->setAcceleration(4000);
  mot->move(item.testSteps, true);   // blokkoló relatív mozgás
  delay(200);
  mot->move(-item.testSteps, true);  // vissza
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

// Ez most a loop()-ból (Core1) fut - mozgástól teljesen fuggetlenul, igy
// mindig, mozgas kozben is azonnal reagal.
void scanButtonsAndDispatch() {
  static bool stableState[8] = {true,true,true,true,true,true,true,true};
  for (int i = 0; i < 8; i++) {
    bool reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonState[i]) lastDebounceTime[i] = millis();
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != stableState[i]) {
        stableState[i] = reading;
        if (reading == LOW) {
          webLogf("Gomb %d lenyomva (GPIO%d)", i + 1, buttonPins[i]);
          handleDigit('1' + i);
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

  // ---- Mozgas-queue ----
  motionQueue = xQueueCreate(MOTION_QUEUE_LEN, sizeof(QueueItem));

  engine.init();
  motorA = engine.stepperConnectToPin(STEP_A_PIN);
  motorA->setDirectionPin(DIR_A_PIN);
  motorA->setAutoEnable(false);
  motorB = engine.stepperConnectToPin(STEP_B_PIN);
  motorB->setDirectionPin(DIR_B_PIN);
  motorB->setAutoEnable(false);

  float lenA, lenB;
  computeStringLengths(currentX, currentY, lenA, lenB);
  motorA->setCurrentPosition(mmToSteps(lenA));
  motorB->setCurrentPosition(mmToSteps(lenB));

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
  delay(2);
}