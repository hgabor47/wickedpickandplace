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

#define MAGNET_PWM_PIN  32  // IBT-2 RPWM (Eszak-fel hid)
#define MAGNET_LPWM_PIN 21  // IBT-2 LPWM (Del-fel hid) - GPIO16/17 WROVER-en PSRAM-nak foglalt, nem hasznalhato
#define MAGNET_EN_PIN   33  // IBT-2 R_EN + L_EN kozosen bekotve (a modulon athidalva), folyamatosan HIGH-on tartva

// Gomb 6 = GPIO23 (belső + külső 4.7k pull-up), a többi a sémának megfelelő.
const int buttonPins[8] = {4, 5, 18, 19, 34, 23, 36, 39};
bool lastButtonState[8]  = {true, true, true, true, true, true, true, true};
bool stableState[8]      = {true, true, true, true, true, true, true, true};
unsigned long lastDebounceTime[8] = {0};
const unsigned long debounceDelay = 30;
String codeBuffer = "";
const int PLAY_CODE_LENGTH = 4; // ennyi szamjegy utan dispatchol a PLAY-mod egy kodot

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
// Merve: origo (0,0)-ban a kotelhossz A-n 260mm, B-n 780mm - ebbol vissza-
// szamolva a horgonypoziciok (a korabbi -200/-200 es 920/-200 becslesek voltak).
const float ANCHOR_A_X = -148.0, ANCHOR_A_Y = -210.0;
const float ANCHOR_B_X =  756.0, ANCHOR_B_Y = -210.0;
// Origoban (0,0) mert kotelhosszak - ezek a hiteles referenciak, nem az
// ANCHOR_A/B-bol computeStringLengths()-szel visszaszamolt (kozelito) ertekek.
const float ORIGIN_LEN_A_MM = 170.0; //266.0;
const float ORIGIN_LEN_B_MM = 700.0; // 786.0;
// A kulissza (gondola) egy szabadon forgo korong: a ket fonal nem a kozeppontban,
// hanem a keruleten, a korong sajat "felfele" jelehez kepest -/+45 fokban van
// rogzitve. A korong elfordulasi szoge (theta) helyzetfuggo, statikai
// egyensulybol adodik - ezert a kinematika nem pontszeru (lasd lejjebb).
const float GONDOLA_RADIUS_MM = 85.0;
const float GONDOLA_ARM_ANGLE_RAD = 45.0 * (PI / 180.0);
// Kulon warm-start a ket iranynak: kereszt-szennyezes nelkul gyorsabban konvergalnak.
static float lastThetaForward = 0.0f;
static float lastThetaInverse = 0.0f;
// A motorok forgasiranya (bekotes/konfiguracio fuggo) - ha a mechanika/DIR
// bekotes valtozik, csak ezt kell modositani, a tobbi szamitason nem valtoztat.
const bool DIR_A_INVERT = false;
const bool DIR_B_INVERT = false;

// Motor: 1.8 fok/lepes (200 lepes/fordulat). A4988 microstepping: MS2 +3.3V-ra
// kotve -> 1/4 step (MS1/MS3 GND-n). Ha az MS1/2/3 bekotes valtozik, csak ezt
// az egy konstanst kell modositani (1=full, 2=half, 4=quarter, 8=1/8, 16=1/16).
const float MICROSTEPPING = 16.0;
// Orso atmero (kozepertek, csavarodo zsinorral): 20mm -> kerulet = pi*20 =~ 62.83mm/fordulat.
const float MOTOR_STEPS_PER_REV = 200.0;
const float SPOOL_DIAMETER_MM   = 20.0;
const float STEPS_PER_MM = (MOTOR_STEPS_PER_REV * MICROSTEPPING) / (PI * SPOOL_DIAMETER_MM); // =~ 12.73

const float SPEED_TRAVEL = 50.0;   // mm/s - ures kocsi (nem penDown)
const float SPEED_CARRY  = 22.0;   // mm/s - targy szallitasa (penDown)
const float ACCEL_MM_S2  = 400.0;  // mm/s^2 - fo mozgas gyorsulasa/lassulasa
const float TESTMOTOR_SPEED_MM_S  = 20.0;  // testMotor diagnosztika sebessege
const float TESTMOTOR_ACCEL_MM_S2 = 300.0;

// ============ ELEKTROMÁGNES PWM PARAMÉTEREK (core 2.x LEDC API) ============
// Teljes H-hid: PWM_CHANNEL_R -> RPWM (Eszak), PWM_CHANNEL_L -> LPWM (Del).
// A ket csatorna sose lehet egyszerre nem-nulla (atloves a hidban) - lasd magnetSetDuty().
const int PWM_CHANNEL_R  = 0;
const int PWM_CHANNEL_L  = 1;
const int PWM_FREQ_HZ   = 1000;
const int PWM_RES_BITS  = 8;
const unsigned long MAGNET_BOOST_MS = 200;
const uint8_t MAGNET_HOLD_DUTY = 130;
// Iranyvaltaskor mindket csatorna 0-ra all, es ennyit varunk mielott az uj oldalt
// bekapcsolnank - a tekercs induktiv, az aram nem tud pillanatszeruen megfordulni.
const unsigned long MAGNET_DIR_SWITCH_DEADTIME_MS = 5;

enum MagnetDirection : uint8_t { MAGNET_NORTH = 0, MAGNET_SOUTH = 1 };
MagnetDirection magnetDirection = MAGNET_NORTH; // legutobb aktivalt/kivalasztott irany

// ============ PICKANDPLACE ELJARAS PARAMETEREI ============
// A kartya alul 1mm-es vajatban all, ezert FELFELE kell kihuzni. A tapadas
// akkor biztos, ha a magnes mar teljesen felmagnesezodott, MIELOTT a kar
// megmozdul - ezert a boost alatt all a mozgas (lasd executePickAndPlace()).
// Futasidoben allithatoak a setPick paranccsal (lasd cmdSetPick).
float         PICKANDPLACE_DIP_MM    = 5.0f;                     // cellakozep alatt ennyivel tapad ra (Y no = lefele)
unsigned long PICKANDPLACE_SETTLE_MS = 150;                      // leereszkedes utani megnyugvas, meg a magnes elott
unsigned long PICKANDPLACE_BOOST_MS  = 500;                      // 100% duty, MOZGAS NELKUL - ez tapasztja ra a kartyat
float         PICKANDPLACE_LIFT_MM   = 10.0f;                    // cellakozep FOLE ennyivel emel (a dip-bol indulva)
uint8_t       PICKANDPLACE_HOLD_DUTY = (uint8_t)(70 * 255 / 100); // 70% tartoero utazas kozben
unsigned long PICKANDPLACE_KICK_MS   = 500;                      // ellentetes polaritasu ledobo-impulzus hossza
unsigned long PICKANDPLACE_CLEAR_MS  = 1000;                     // varakozas, amig az ajto biztosan eltunik

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

// ============ REKESZ-KOD TABLA (gombkod -> automatikus PICKANDPLACE) ============
// A 7x5 SETUP-racs celláit 1-alapú "rekesz" sorszámmal azonosítjuk (rekesz = sor*GRID_COLS+oszlop+1).
// A 32. rekesz (col=3,row=4) a FIX ledobó hely - minden más rekeszből ide ürít a PICKANDPLACE.
#define REKESZ_COUNT 35
const int DROP_REKESZ = 32;
char rekeszCodeTable[REKESZ_COUNT + 1][5] = {{0}}; // 1-alapú, üres string = nincs kód erre a rekeszre
uint64_t rekeszProcessedBits = 0; // bit(rekesz-1)=1, ha az a rekesz mar kiadasra kerult
// false (alapertelmezett) = egy mar kiadott rekesz kodja ujra elfogadhato; true = csak egyszer
// adhato ki (lasd findAvailableRekeszByCode()). Futasidoben allithato: cellRepeat 0|1.
bool rekeszBlockRepeat = false;

struct RekeszCodeDefault { uint8_t rekesz; const char *code; };
// Gyari alapertek - csak akkor toltodik be, ha meg soha nem volt beallitva rekesz-kod tabla (lasd setup()).
const RekeszCodeDefault REKESZ_CODE_DEFAULTS[] = {
  {1,"1537"},{2,"7126"},{3,"3526"},{4,"7531"},{5,"4318"},{6,"3751"},{7,"1762"},
  {8,"3724"},{9,"4615"},{10,"8431"},{11,"3678"},{12,"2671"},{13,"6415"},{14,"3876"},
  {15,"8736"},{16,"8376"},{17,"1375"},{18,"6738"},{19,"4516"},{20,"6514"},{21,"7548"},
  {22,"5182"},{23,"1843"},{24,"2185"},{25,"2536"},{26,"3472"},{27,"5821"},{28,"3571"},
  {29,"6145"},{30,"6721"},{31,"6541"},{33,"1348"},{34,"1267"},{35,"3481"}
};
const int REKESZ_CODE_DEFAULT_COUNT = sizeof(REKESZ_CODE_DEFAULTS) / sizeof(REKESZ_CODE_DEFAULTS[0]);

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

void saveRekeszCodes() {
  prefs.putBytes("rekeszCodes", rekeszCodeTable, sizeof(rekeszCodeTable));
}
void loadRekeszCodes() {
  size_t len = prefs.getBytes("rekeszCodes", rekeszCodeTable, sizeof(rekeszCodeTable));
  if (len != sizeof(rekeszCodeTable)) memset(rekeszCodeTable, 0, sizeof(rekeszCodeTable));
}
void saveRekeszProcessedBits() {
  prefs.putULong64("rekeszProcBits", rekeszProcessedBits);
}

int findAvailableRekeszByCode(const String &code) {
  for (int r = 1; r <= REKESZ_COUNT; r++) {
    bool processed = rekeszBlockRepeat && ((rekeszProcessedBits >> (r - 1)) & 1ULL);
    if (rekeszCodeTable[r][0] != '\0' && code.equals(rekeszCodeTable[r]) && !processed) {
      return r;
    }
  }
  return -1;
}

// ============ MOZGÁS-QUEUE (Core0 fogyasztja) ============
enum QueueItemKind : uint8_t { QI_SEQUENCE = 0, QI_MOTOR_TEST = 1, QI_MAGNET_TEST = 2, QI_SHOWCELLS = 3, QI_PICKANDPLACE = 4, QI_GOHOME = 5 };

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
  uint8_t magnetDirection;      // QI_MAGNET_TEST: MagnetDirection (0=Eszak, 1=Del)
  uint8_t pickCol, pickRow;     // QI_PICKANDPLACE: felveteli cella (0-alapu, GRID_COLS/GRID_ROWS racs)
  uint8_t dropCol, dropRow;     // QI_PICKANDPLACE: ledobo cella (0-alapu)
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

// ============ HANGJELZESEK (kliens-oldali lejatszas /log oldalon) ============
// A 4 mp3 nyers binarisan van a firmware-be egetve (lasd platformio.ini
// board_build.embed_files) - a szimbolumnevek az objcopy konvenciot kovetik.
extern const uint8_t click_mp3_start[] asm("_binary_data_click_mp3_start");
extern const uint8_t click_mp3_end[]   asm("_binary_data_click_mp3_end");
extern const uint8_t err_mp3_start[]   asm("_binary_data_err_mp3_start");
extern const uint8_t err_mp3_end[]     asm("_binary_data_err_mp3_end");
extern const uint8_t start_mp3_start[] asm("_binary_data_start_mp3_start");
extern const uint8_t start_mp3_end[]   asm("_binary_data_start_mp3_end");
extern const uint8_t drop_mp3_start[]  asm("_binary_data_drop_mp3_start");
extern const uint8_t drop_mp3_end[]    asm("_binary_data_drop_mp3_end");

// Kulon SSE esemenynev ('sound'), NEM a 'log'-ba - igy nem kerul bele a
// logHistory-ba es nem jatszodik le ujra minden reconnectnel (lasd events.onConnect()).
void soundEvent(const char *name) {
  events.send(name, "sound", millis());
}

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
<button id="unlockBtn" onclick="unlockAudio()">Hang engedélyezése</button>
<div id="log"></div>
<audio id="aClick" src="/sound/click.mp3" preload="auto"></audio>
<audio id="aErr"   src="/sound/err.mp3"   preload="auto"></audio>
<audio id="aStart" src="/sound/start.mp3" preload="auto"></audio>
<audio id="aDrop"  src="/sound/drop.mp3"  preload="auto"></audio>
<script>
  const logDiv = document.getElementById('log');
  const src = new EventSource('/events');
  src.addEventListener('log', function(e) {
    logDiv.textContent += e.data + "\n";
    window.scrollTo(0, document.body.scrollHeight);
  });
  const soundMap = { click: 'aClick', err: 'aErr', start: 'aStart', drop: 'aDrop' };
  src.addEventListener('sound', function(e) {
    const a = document.getElementById(soundMap[e.data]);
    if (a) { a.currentTime = 0; a.play().catch(()=>{}); }
  });
  // Mobil autoplay-korlatozas miatt kell egy user-gesture, ami "feloldja" a lejatszast.
  function unlockAudio() {
    ['aClick','aErr','aStart','aDrop'].forEach(id => {
      const a = document.getElementById(id);
      a.play().then(() => { a.pause(); a.currentTime = 0; }).catch(()=>{});
    });
  }
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
<p class="hint">Hasznalhato utasitasok: setCell, test, cellNum, errorGesture, testMotor, testMagnet, pickAndPlace, goHome, cellCode, cellReset, cellRepeat, listCell, clearCells, setSpeed, setPick, geom, genAutocells, restartAP</p>
<p class="hint">Soronként egy parancs. Nincs szóköz a parancson kívül. Pl:<br>
setCell 1,100001,{70,70,1,1},{100,100,0,1},{680,70,255,0}<br>
listCell</p>
<textarea id="src" placeholder="setCell 1,100001,{70,70,1,1},..."></textarea><br>
<button onclick="run()">Futtat</button>
<div class="quick">
  <button onclick="quick('testMotor 0,200')">motor1</button>
  <button onclick="quick('testMotor 1,200')">motor2</button>
  <button onclick="quick('testMagnet 100,3,n')">magnet100 eszak</button>
  <button onclick="quick('testMagnet 50,3,n')">magnet50 eszak</button>
  <button onclick="quick('testMagnet 100,3,s')">magnet100 del</button>
  <button onclick="quick('testMagnet 50,3,s')">magnet50 del</button>
  <button onclick="quick('pickAndPlace')">pickAndPlace</button>
  <button onclick="quick('goHome')">goHome</button>
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

float currentX = 0.0, currentY = 0.0;
bool penDown = false;

// ============ KINEMATIKA ============
// ---- iteracios parameterek (forward es inverz irany kozosen hasznalja) ----
static const int   TRI_MAX_ITER      = 12;      // 1D Newton lepesek
static const int   TRI_SCAN_STEPS    = 48;      // durva scan felbontas (tartalek)
static const int   TRI_BISECT_ITER   = 24;      // bisekcio lepesszam
static const float TRI_TORQUE_TOL    = 2.0e-5f; // elfogadasi kuszob (normalt)
static const float TRI_TORQUE_ACCEPT = 1.0e-3f; // "meg elfogadhato" kuszob
static const float TRI_THETA_H       = 0.002f;  // rad - numerikus derivalt lepese
static const float THETA_MIN         = -1.2f;
static const float THETA_MAX         =  1.2f;

// A gondola-modell EGYETLEN definicios helye - a forward es az inverz irany is
// ezt hasznalja, igy nem tudnak szetcsuszni.
//
// A nyomateki reziduum POLUSMENTES alakban: a naiv `tA*mA + mB` (ahol
// tA = -uBx/uAx) ott szingularis, ahol az A kotel fuggolegesse valik. uAx-szel
// felszorozva a gyokok ugyanazok maradnak, de eltunik az osztas es az eseti ag.
// R-rel normalva dimenziotlan, O(1) nagysagrendu.
//   visszateres: g (normalt nyomateki reziduum, gyoke = statikai egyensuly)
static float gondolaTorqueAt(float x, float y, float theta,
                             float &lenA, float &lenB, float &tA) {
  float phiA = theta - GONDOLA_ARM_ANGLE_RAD;
  float rAx  =  GONDOLA_RADIUS_MM * sin(phiA);
  float rAy  = -GONDOLA_RADIUS_MM * cos(phiA);
  float vAx  = ANCHOR_A_X - (x + rAx), vAy = ANCHOR_A_Y - (y + rAy);
  float dA   = sqrt(vAx * vAx + vAy * vAy);
  if (dA < 1e-3f) { lenA = lenB = 0; tA = -1.0f; return 0.0f; }
  float uAx = vAx / dA, uAy = vAy / dA;

  float phiB = theta + GONDOLA_ARM_ANGLE_RAD;
  float rBx  =  GONDOLA_RADIUS_MM * sin(phiB);
  float rBy  = -GONDOLA_RADIUS_MM * cos(phiB);
  float vBx  = ANCHOR_B_X - (x + rBx), vBy = ANCHOR_B_Y - (y + rBy);
  float dB   = sqrt(vBx * vBx + vBy * vBy);
  if (dB < 1e-3f) { lenA = lenB = 0; tA = -1.0f; return 0.0f; }
  float uBx = vBx / dB, uBy = vBy / dB;

  float mA = rAx * uAy - rAy * uAx;
  float mB = rBx * uBy - rBy * uBx;

  lenA = dA;
  lenB = dB;
  // tA csak ERVENYESSEG-ELLENORZESRE kell (pozitiv koteleroet varunk),
  // a megoldasba nem szol bele - ezert itt nyugodtan lehet osztani.
  tA = (fabs(uAx) > 1e-6f) ? (-uBx / uAx) : -1.0f;
  return (-uBx * mA + uAx * mB) / GONDOLA_RADIUS_MM;
}

// FORWARD: (X,Y) -> kotelhosszak. Egyetlen ismeretlen (theta), csillapitott
// 1D Newton - a line search miatt nem tud tullendulni. A suly csak
// fuggolegesen hat, igy az eroaranyt (es ezzel theta-t) nem befolyasolja:
// a modell sulyfuggetlen.
void computeStringLengths(float x, float y, float &lenA, float &lenB) {
  float theta = constrain(lastThetaForward, THETA_MIN, THETA_MAX);
  float tA;
  float g = gondolaTorqueAt(x, y, theta, lenA, lenB, tA);

  for (int iter = 0; iter < TRI_MAX_ITER; iter++) {
    if (fabs(g) < TRI_TORQUE_TOL) break;
    float thH = min(THETA_MAX, theta + TRI_THETA_H);
    float lA_h, lB_h, tA_h;
    float gH = gondolaTorqueAt(x, y, thH, lA_h, lB_h, tA_h);
    float deriv = (gH - g) / (thH - theta);
    if (fabs(deriv) < 1e-9f) break;

    float step = g / deriv;
    float lambda = 1.0f;
    bool improved = false;
    for (int bt = 0; bt < 8; bt++) {
      float thNew = constrain(theta - lambda * step, THETA_MIN, THETA_MAX);
      float lA_n, lB_n, tA_n;
      float gNew = gondolaTorqueAt(x, y, thNew, lA_n, lB_n, tA_n);
      if (fabs(gNew) < fabs(g)) {
        theta = thNew; g = gNew; lenA = lA_n; lenB = lB_n; tA = tA_n;
        improved = true;
        break;
      }
      lambda *= 0.5f;
    }
    if (!improved) break; // elertuk a float32 zajszintet
  }

  lastThetaForward = theta;
}

long mmToSteps(float mm) { return (long)round(mm * STEPS_PER_MM); }
// Sebesseg/gyorsulas mm/s (^2) -> steps/s (^2), ugyanazzal a linearis skalazassal mint a pozicio.
uint32_t mmSpeedToStepsHz(float mmPerSec)   { return (uint32_t)max(1L, (long)round(mmPerSec * STEPS_PER_MM)); }
uint32_t mmAccelToStepsS2(float mmPerSec2)  { return (uint32_t)max(1L, (long)round(mmPerSec2 * STEPS_PER_MM)); }

// Kozos "aranyos" mozgas-inditas: mindket motor a SAJAT delta-javal aranyos
// sebesseget ES gyorsulast kap, igy egyszerre indulnak/ernek celba - a mozgas
// TELJES idotartama alatt egyenes vonalu marad, nem csak az allando sebessegu
// szakaszban. Ezt hasznalja a jog (moveJogApply) es a PLAY-mod szegmentalt
// vonalkovetese (moveToBlocking) is - igy a ket mozgatasi ut egysegesitve van.
void applyProportionalMove(long targetA, long targetB, float speedMmPerSec, float accelMmPerS2) {
  long curA = motorA->getCurrentPosition(), curB = motorB->getCurrentPosition();
  long dA = abs(targetA - curA), dB = abs(targetB - curB);
  long dMax = max(dA, dB);
  if (dMax == 0) return;
  uint32_t vMax = mmSpeedToStepsHz(speedMmPerSec);
  uint32_t aMax = mmAccelToStepsS2(accelMmPerS2);
  uint32_t speedA = max(1UL, (unsigned long)(((uint64_t)vMax * (uint64_t)dA) / (uint64_t)dMax));
  uint32_t speedB = max(1UL, (unsigned long)(((uint64_t)vMax * (uint64_t)dB) / (uint64_t)dMax));
  uint32_t accelA = max(1UL, (unsigned long)(((uint64_t)aMax * (uint64_t)dA) / (uint64_t)dMax));
  uint32_t accelB = max(1UL, (unsigned long)(((uint64_t)aMax * (uint64_t)dB) / (uint64_t)dMax));
  motorA->setSpeedInHz(speedA);
  motorB->setSpeedInHz(speedB);
  motorA->setAcceleration(accelA);
  motorB->setAcceleration(accelB);
  motorA->moveTo(targetA);
  motorB->moveTo(targetB);
}

// Ennel hosszabb mozgast szegmensekre bontunk: a step-terbeli egyenes
// interpolacio hosszu tavon a valos XY-sikban ivnek latszana (a trilateracios
// step<->XY lekepezes nemlinearis), ezert a VALODI XY-egyenes menten
// szamolunk kozbenso pontokat.
// M6: ugyanaz a hossz, mint JOG_SEGMENT_LEN_MM - igy a tolerancia (lasd lejjebb)
// nagyobb marad a szegmenshossznal, es a motor sose fekezik le teljesen egy-egy
// kozbenso waypointnal (korabban 10mm-es szegmenssel a fekezes/ujragyorsulas
// dodogo, "nyogvenyelos" mozgast okozott).
const float PLAY_SEGMENT_LEN_MM = 3.0f;
// Kozbenso szegmensnel ennyire kell csak megkozeliteni a celt ahhoz, hogy a
// kovetkezo szegmensre valthassunk - igy a motor nem all meg teljesen minden
// waypointnal, csak a mozgas legvegen (a valodi celnal).
const long PLAY_WAYPOINT_TOLERANCE_STEPS = 60;

// Csak a Core0-motionExecTask hívja - itt a blokkolás nem gond, mert ez a mag
// KIZÁRÓLAG ezt csinálja, a gombolvasás/webszerver a másik magon fut tovább.
void moveToBlocking(float x, float y, float speedMmPerSec) {
  float startX = currentX, startY = currentY;
  float distX = x - startX, distY = y - startY;
  float totalDist = sqrt(distX * distX + distY * distY);
  if (totalDist < 0.01f) { currentX = x; currentY = y; return; }

  int segments = max(1, (int)ceil(totalDist / PLAY_SEGMENT_LEN_MM));
  for (int i = 1; i <= segments; i++) {
    float t = (float)i / segments;
    float lenA, lenB;
    computeStringLengths(startX + distX * t, startY + distY * t, lenA, lenB);
    long ta = mmToSteps(lenA), tb = mmToSteps(lenB);
    applyProportionalMove(ta, tb, speedMmPerSec, ACCEL_MM_S2);

    bool isLast = (i == segments);
    while (motorA->isRunning() || motorB->isRunning()) {
      if (!isLast &&
          abs(ta - motorA->getCurrentPosition()) <= PLAY_WAYPOINT_TOLERANCE_STEPS &&
          abs(tb - motorB->getCurrentPosition()) <= PLAY_WAYPOINT_TOLERANCE_STEPS) {
        break; // kozel eleg a kozbenso waypointhoz - folytatjuk megallas nelkul
      }
      delay(2);
    }
  }
  currentX = x;
  currentY = y;
}

// ============ ELEKTROMÁGNES: IRÁNYVÁLTÁS + BOOST-THEN-HOLD ============
// Biztonsagos duty-iras: ha iranyt valt, eloszor mindket felhidat 0-ra huzza es
// holtidot tart, csak utana kapcsolja be az uj oldalt - sose fut egyszerre a ket oldal.
void magnetSetDuty(MagnetDirection dir, uint8_t duty) {
  if (dir != magnetDirection) {
    ledcWrite(PWM_CHANNEL_R, 0);
    ledcWrite(PWM_CHANNEL_L, 0);
    delay(MAGNET_DIR_SWITCH_DEADTIME_MS);
    magnetDirection = dir;
  }
  ledcWrite((dir == MAGNET_NORTH) ? PWM_CHANNEL_R : PWM_CHANNEL_L, duty);
  webLogf("MAGNET: irany=%s duty=%d/255 (csatorna=%s)",
          (dir == MAGNET_NORTH) ? "ESZAK" : "DEL", duty,
          (dir == MAGNET_NORTH) ? "R/GPIO32" : "L/GPIO16");
}

unsigned long engageMagnet(MagnetDirection dir = magnetDirection) {
  magnetSetDuty(dir, 255);
  delay(MAGNET_BOOST_MS);
  magnetSetDuty(dir, MAGNET_HOLD_DUTY);
  penDown = true;
  return MAGNET_BOOST_MS;
}

unsigned long releaseMagnet() {
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);
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
  soundEvent("start");
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
  magnetSetDuty((MagnetDirection)item.magnetDirection, duty);
  delay((unsigned long)item.magnetDurationSec * 1000UL);
  ledcWrite(PWM_CHANNEL_R, 0);
  ledcWrite(PWM_CHANNEL_L, 0);
}

// Definialva lejjebb, a racs-kalibracios (GRID_COLS/gridCellA/B) szekcio utan -
// itt csak a motionExecTask dispatch-hez kell a prototipus.
void executeShowCells();
void executePickAndPlace(uint8_t pickCol, uint8_t pickRow, uint8_t dropCol, uint8_t dropRow);
void executeGoHome();
// Ugyanigy a rekesz->racs (col,row) atszamitas is a GRID_COLS/GRID_ROWS utan van definialva,
// de a gombkod-feloldashoz (dispatchCodeToQueue) mar itt kell a prototipus.
bool rekeszToColRow(int rekesz, uint8_t &col, uint8_t &row);

void motionExecTask(void *parameter) {
  QueueItem item;
  for (;;) {
    if (xQueueReceive(motionQueue, &item, portMAX_DELAY) == pdTRUE) {
      if (item.startDelayMs > 0) delay(item.startDelayMs);
      switch (item.kind) {
        case QI_SEQUENCE:    executeQueueSequence(item); break;
        case QI_MOTOR_TEST:  executeMotorTest(item);     break;
        case QI_MAGNET_TEST: executeMagnetTest(item);    break;
        case QI_SHOWCELLS:   executeShowCells();          break;
        case QI_PICKANDPLACE: executePickAndPlace(item.pickCol, item.pickRow, item.dropCol, item.dropRow); break;
        case QI_GOHOME:       executeGoHome();             break;
      }
      webLogf("Vegrehajtva. Pufferben meg: %d elem.", (int)uxQueueMessagesWaiting(motionQueue));
    }
  }
}

// ============ GOMBKÓD -> QUEUE ============
void dispatchCodeToQueue(const String &code) {
  // 1) rekesz-kod tabla (cellCode paranccsal beallitott, automatikus PICKANDPLACE-t inditó kodok) -
  // ez az elsodleges ut, a regi setCell/cellCache-alapu szekvenciak csak masodlagosan probalkoznak.
  int rekesz = findAvailableRekeszByCode(code);
  if (rekesz >= 0) {
    uint8_t pickCol, pickRow, dropCol, dropRow;
    if (!rekeszToColRow(rekesz, pickCol, pickRow) || !rekeszToColRow(DROP_REKESZ, dropCol, dropRow)) {
      webLogf("REKESZKOD: HIBA - rekesz->racs atszamitas sikertelen (rekesz=%d).", rekesz);
      soundEvent("err");
      return;
    }
    QueueItem item = {};
    item.kind = QI_PICKANDPLACE;
    item.pickCol = pickCol; item.pickRow = pickRow;
    item.dropCol = dropCol; item.dropRow = dropRow;
    if (xQueueSend(motionQueue, &item, 0) != pdTRUE) {
      webLog("HIBA: mozgas-puffer tele, a kod elveszett.");
      return;
    }
    rekeszProcessedBits |= (1ULL << (rekesz - 1));
    saveRekeszProcessedBits();
    webLogf("Rekesz #%d (kod %s) sorba allitva PICKANDPLACE-kent. Pufferben: %d elem.",
            rekesz, code.c_str(), (int)uxQueueMessagesWaiting(motionQueue));
    return;
  }

  if (!cellsFormatOk) {
    webLog("Cellak nem elerhetoek (formatumverzio-elteres) - futtass clearCells-t elobb.");
    soundEvent("err");
    return;
  }
  int idx = findAvailableCellByCode(code);
  if (idx < 0) {
    webLogf("Nincs elerheto (meg nem aktivalt) cella a kodhoz: %s", code.c_str());
    soundEvent("err");
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
  if (codeBuffer.length() >= PLAY_CODE_LENGTH) {
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

// 1-alapu rekeszszam -> 0-alapu (col,row) a 7x5 racson (sor-major: rekesz = row*GRID_COLS+col+1).
bool rekeszToColRow(int rekesz, uint8_t &col, uint8_t &row) {
  if (rekesz < 1 || rekesz > REKESZ_COUNT) return false;
  int idx = rekesz - 1;
  col = idx % GRID_COLS;
  row = idx / GRID_COLS;
  return true;
}

// sarokindex: 0=(0,0) 1=(0,YMAX) 2=(XMAX,0) 3=(XMAX,YMAX)
long gridCornerA[4] = {0};
long gridCornerB[4] = {0};
uint8_t gridCornerMask = 0; // bit(i)=1, ha az i. sarok mar el van mentve

long gridCellA[GRID_CELL_COUNT] = {0};
long gridCellB[GRID_CELL_COUNT] = {0};
bool gridCellsComputed = false;

// M7: oszloponkenti Y-eltolas szazalekban (pozitiv = lejjebb/nagyobb Y, negativ = feljebb) -
// a sor-lepeskoz (egy cellasor Y-tavolsaganak) szazalekaban ertelmezve, lasd computeDefaultGridCells().
// col index: 0..GRID_COLS-1, balrol jobbra. Futasidoben modosithato a genAutocells paranccsal.
float COL_Y_COMP_PERCENT[GRID_COLS] = { 5.0f, 3.0f, 0.0f, -3.0f, 0.0f, 3.0f, 5.0f };

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

// Adott theta mellett a korong kozeppontja ZART KEPLETTEL: a rogzitesi pont
// eltolasat levonjuk a horgonybol ("virtualis horgony"), es a ket kotelhossz
// mint sugar egy sima kor-metszest ad. Igy a kotelhossz-egyenletek MINDIG
// pontosan teljesulnek, barmilyen theta mellett - csak theta marad ismeretlen.
static bool gondolaSolveAtTheta(float theta, float targetLenA, float targetLenB,
                                float &x, float &y, float &g, float &tA) {
  float phiA = theta - GONDOLA_ARM_ANGLE_RAD;
  float rAx  =  GONDOLA_RADIUS_MM * sin(phiA);
  float rAy  = -GONDOLA_RADIUS_MM * cos(phiA);
  float phiB = theta + GONDOLA_ARM_ANGLE_RAD;
  float rBx  =  GONDOLA_RADIUS_MM * sin(phiB);
  float rBy  = -GONDOLA_RADIUS_MM * cos(phiB);

  if (!circleIntersectBelow(ANCHOR_A_X - rAx, ANCHOR_A_Y - rAy,
                            ANCHOR_B_X - rBx, ANCHOR_B_Y - rBy,
                            targetLenA, targetLenB, x, y)) {
    return false;
  }
  float lA, lB;
  g = gondolaTorqueAt(x, y, theta, lA, lB, tA);
  return true;
}

// INVERZ: step-par -> valodi (X,Y).
// Ez az EGYETLEN hely, ahol step->(X,Y) tortenik (M1: egyetlen igazsagforras).
bool trilaterateSteps(long stepsA, long stepsB, float &outX, float &outY) {
  float targetLenA = stepsA / STEPS_PER_MM;
  float targetLenB = stepsB / STEPS_PER_MM;

  float theta = constrain(lastThetaInverse, THETA_MIN, THETA_MAX);
  float x, y, g, tA;
  bool  have = gondolaSolveAtTheta(theta, targetLenA, targetLenB, x, y, g, tA);
  if (!have) { // warm-start nem hasznalhato -> hideg indulas
    theta = 0.0f;
    have  = gondolaSolveAtTheta(theta, targetLenA, targetLenB, x, y, g, tA);
  }

  // --- 1. fazis: csillapitott 1D Newton theta-ra ---
  if (have) {
    for (int iter = 0; iter < TRI_MAX_ITER; iter++) {
      if (fabs(g) < TRI_TORQUE_TOL) break;
      float thH = min(THETA_MAX, theta + TRI_THETA_H);
      float xH, yH, gH, tAH;
      if (!gondolaSolveAtTheta(thH, targetLenA, targetLenB, xH, yH, gH, tAH)) break;
      float deriv = (gH - g) / (thH - theta);
      if (fabs(deriv) < 1e-9f) break;

      float step = g / deriv;
      float lambda = 1.0f;
      bool improved = false;
      for (int bt = 0; bt < 8; bt++) {
        float thNew = constrain(theta - lambda * step, THETA_MIN, THETA_MAX);
        float xN, yN, gN, tAN;
        if (gondolaSolveAtTheta(thNew, targetLenA, targetLenB, xN, yN, gN, tAN) &&
            fabs(gN) < fabs(g)) {
          theta = thNew; x = xN; y = yN; g = gN; tA = tAN;
          improved = true;
          break;
        }
        lambda *= 0.5f;
      }
      if (!improved) break; // float32 zajszint
    }
  }

  // --- 2. fazis (tartalek): durva scan + bisekcio, ha a Newton nem ert celba ---
  if (!have || fabs(g) > TRI_TORQUE_ACCEPT) {
    bool  havePrev = false;
    float prevTh = 0, prevG = 0;
    float loTh = 0, loG = 0, hiTh = 0;
    bool  bracketed = false;

    for (int i = 0; i <= TRI_SCAN_STEPS; i++) {
      float t = THETA_MIN + (THETA_MAX - THETA_MIN) * ((float)i / (float)TRI_SCAN_STEPS);
      float xs, ys, gs, tAs;
      if (!gondolaSolveAtTheta(t, targetLenA, targetLenB, xs, ys, gs, tAs)) {
        havePrev = false; // ertelmezesi tartomanyon kivul
        continue;
      }
      // Elojelvaltast csak akkor fogadunk el gyoknek, ha mindket oldal KICSI -
      // igy nem ulunk fel egy esetleges ugrasnak.
      if (havePrev && prevG * gs < 0 && fabs(prevG) < 2.0f && fabs(gs) < 2.0f) {
        loTh = prevTh; loG = prevG; hiTh = t;
        bracketed = true;
        break;
      }
      havePrev = true; prevTh = t; prevG = gs;
    }

    if (!bracketed) {
      lastThetaInverse = 0.0f; // ne mergezzuk a kovetkezo hivast
      webLogf("KINEMATIKA: nincs egyensulyi theta (stepsA=%ld, stepsB=%ld, lenA=%.1f, lenB=%.1f)",
              stepsA, stepsB, targetLenA, targetLenB);
      return false;
    }

    for (int b = 0; b < TRI_BISECT_ITER; b++) {
      float mid = 0.5f * (loTh + hiTh);
      float xm, ym, gm, tAm;
      if (!gondolaSolveAtTheta(mid, targetLenA, targetLenB, xm, ym, gm, tAm)) break;
      if (loG * gm <= 0) { hiTh = mid; }
      else               { loTh = mid; loG = gm; }
      theta = mid; x = xm; y = ym; g = gm; tA = tAm;
    }
  }

  // --- 3. fazis: fizikai ervenyesseg ---
  // A kotel csak HUZNI tud: tA <= 0 egy matematikailag letezo, de fizikailag
  // lehetetlen gyokot jelent.
  if (tA <= 0.0f) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: fizikailag ervenytelen gyok - negativ koteleroe (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f, tA=%.2f)",
            stepsA, stepsB, x, y, theta, tA);
    return false;
  }
  if (y <= ANCHOR_A_Y || y <= ANCHOR_B_Y) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: fizikailag ervenytelen gyok - a kulissza a horgonyok folott (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f)",
            stepsA, stepsB, x, y, theta);
    return false;
  }
  if (fabs(g) > TRI_TORQUE_ACCEPT) {
    lastThetaInverse = 0.0f;
    webLogf("KINEMATIKA: nem konvergalt (stepsA=%ld, stepsB=%ld, x=%.1f, y=%.1f, th=%.3f, g=%.6f, tA=%.2f)",
            stepsA, stepsB, x, y, theta, g, tA);
    return false;
  }

  outX = x;
  outY = y;
  lastThetaInverse = theta; // warm-start CSAK sikeres gyokre
  return true;
}

// (X,Y) -> step-par, a trilaterateSteps() inverze - ugyanazt a gondola-modellt
// hasznalja (computeStringLengths), igy a ket irany konzisztens.
void xyToSteps(float x, float y, long &outA, long &outB) {
  float lenA, lenB;
  computeStringLengths(x, y, lenA, lenB);
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

// u: 0=X-min .. 1=X-max oldal, v: 0=Y-min .. 1=Y-max oldal - a cel VALOS
// (X,Y) koordinatait adja vissza (a sarkok kozotti linearis interpolaciobol).
void gridUVToXY(float u, float v, float &outX, float &outY) {
  outX = bilerpFloat(gridCornerX[0], gridCornerX[2], gridCornerX[1], gridCornerX[3], u, v);
  outY = bilerpFloat(gridCornerY[0], gridCornerY[2], gridCornerY[1], gridCornerY[3], u, v);
}

// A cel (X,Y)-t a sarkok VALOS (nem kotelhossz-terbeli) koordinatai kozott
// interpolaljuk linearisan, majd abbol szamoljuk a lepesszamokat (xyToSteps).
void gridTargetForUV(float u, float v, long &outA, long &outB) {
  if (!gridCornerXYValid) {
    // biztonsagi tartalek, ha meg nincs kesz mind a 4 sarok (elvileg ide nem
    // szabadna eljutni, mert az EDIT_MOVE/CELLS csak teljes SETUP utan ertelmes).
    outA = bilerpLong(gridCornerA[0], gridCornerA[2], gridCornerA[1], gridCornerA[3], u, v);
    outB = bilerpLong(gridCornerB[0], gridCornerB[2], gridCornerB[1], gridCornerB[3], u, v);
    return;
  }
  float x, y;
  gridUVToXY(u, v, x, y);
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
  // Sarok-illeszkedo racs: col=0/row=0 pontosan a (0,0) sarokra esik, col=GRID_COLS-1/
  // row=GRID_ROWS-1 pontosan a szemkozti sarokra - NEM cella-kozep-inset (ami felcellanyit
  // beljebb tolna az elso/utolso oszlopot/sort, kb 40mm-es "csuszast" okozva a szeleken).
  // M7: oszloponkenti Y-korrekcio (mert-tapasztalati) - a naiv bilinearis interpolaciotol
  // szisztematikusan elter (pl. kotel-athajlas), ezert egy oszloponkenti V-eltolast
  // adunk hozza, a sor-lepeskoz (1/(GRID_ROWS-1)) szazalekaban kifejezve. Ujraszamolhato
  // a genAutocells parancssal is, a teljes SETUP (sarok-ujramerés) nelkul.
  float rowStepV = (GRID_ROWS > 1) ? 1.0f / (GRID_ROWS - 1) : 0.0f;
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      float u = (GRID_COLS > 1) ? (float)col / (GRID_COLS - 1) : 0.5f;
      float v = (GRID_ROWS > 1) ? (float)row / (GRID_ROWS - 1) : 0.5f;
      v += (COL_Y_COMP_PERCENT[col] / 100.0f) * rowStepV;
      int idx = row * GRID_COLS + col;
      gridTargetForUV(u, v, gridCellA[idx], gridCellB[idx]);
    }
  }
  gridCellsComputed = true;
  saveGridCells();
}

// ============ SHOWCELLS: automatikus cellabejaras (BTN6 az EDIT_MENU-ben) ============
// A gridCellA/B mar sor-major (idx = row*GRID_COLS+col) sorrendben van feltoltve,
// igy a novekvo idx bejaras onmagaban Z-alaku (soronkent balrol jobbra, majd a
// kovetkezo sor elejere ugorva) - nincs szukseg kulon kigyozo (serpentine) logikara.
const unsigned long SHOWCELLS_DWELL_MS = 1000;

void executeShowCells() {
  if (!gridCornerXYValid) {
    webLog("SHOWCELLS: HIBA - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
    return;
  }
  webLog("SHOWCELLS: cellabejaras inditva (0..34).");
  for (int idx = 0; idx < GRID_CELL_COUNT; idx++) {
    float x, y;
    if (!trilaterateSteps(gridCellA[idx], gridCellB[idx], x, y)) {
      webLogf("SHOWCELLS: cella #%d (sor %d, oszlop %d) step->XY hiba, kihagyva.",
              idx, idx / GRID_COLS, idx % GRID_COLS);
      continue;
    }
    moveToBlocking(x, y, SPEED_TRAVEL);
    webLogf("SHOWCELLS: cella #%d (sor %d, oszlop %d) elerve.",
            idx, idx / GRID_COLS, idx % GRID_COLS);
    delay(SHOWCELLS_DWELL_MS);
  }
  syncStateFromMotors();
  webLog("SHOWCELLS: bejaras kesz (35/35 cella).");
}

// ============ PICKANDPLACE: felveteli+ledobo teszt-eljaras (BTN7 az EDIT_MENU-ben) ============
// 1) cellakozep -> DIP_MM-rel lejjebb ereszkedes: a kartya alsó felen levo vaslemez
//    igy a magnes kozepe ala kerul, es az emeles a kartyat FELFELE huzza ki az
//    1mm-es vajatbol (korabban a kozeprol indulva a kartya csak lecsuszott rola).
// 2) 100% boost MOZGAS NELKUL - a tapadas csak a teljes felmagnesezodes utan biztos.
// 3) emeles a cellakozep FOLE, majd 70% tartoerovel utazas a ledobo cellara.
// 4) celban ellentetes polaritasu "kick" ledobja az ajtot, majd varunk, amig eltunik.
void executePickAndPlace(uint8_t pickCol, uint8_t pickRow, uint8_t dropCol, uint8_t dropRow) {
  soundEvent("start");
  if (!gridCornerXYValid) {
    webLog("PICKANDPLACE: HIBA - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
    return;
  }
  int pickIdx = pickRow * GRID_COLS + pickCol;
  int dropIdx = dropRow * GRID_COLS + dropCol;
  if (pickCol >= GRID_COLS || pickRow >= GRID_ROWS || dropCol >= GRID_COLS || dropRow >= GRID_ROWS) {
    webLog("PICKANDPLACE: HIBA - ervenytelen cella-index.");
    return;
  }

  float pickX, pickY, dropX, dropY;
  if (!trilaterateSteps(gridCellA[pickIdx], gridCellB[pickIdx], pickX, pickY) ||
      !trilaterateSteps(gridCellA[dropIdx], gridCellB[dropIdx], dropX, dropY)) {
    webLog("PICKANDPLACE: HIBA - step->XY atszamitas sikertelen.");
    return;
  }

  webLogf("PICKANDPLACE: felvetel cella(col=%d,row=%d) -> ledobas cella(col=%d,row=%d).",
          pickCol, pickRow, dropCol, dropRow);

  moveToBlocking(pickX, pickY, SPEED_TRAVEL);

  // --- 1) leereszkedes a cellakozep ala, majd megnyugvas ---
  moveToBlocking(pickX, pickY + PICKANDPLACE_DIP_MM, SPEED_CARRY);
  delay(PICKANDPLACE_SETTLE_MS);

  // --- 2) 100% boost ALLO karral - csak igy tapad ra biztosan a kartya ---
  magnetSetDuty(MAGNET_NORTH, 255);
  delay(PICKANDPLACE_BOOST_MS);
  penDown = true;

  // --- 3) emeles a cellakozep foles, teljes eroben (a vajatbol valo kihuzas) ---
  moveToBlocking(pickX, pickY - PICKANDPLACE_LIFT_MM, SPEED_CARRY);

  // --- 4) 70% tartoero, utazas a ledobo cellara ---
  magnetSetDuty(MAGNET_NORTH, PICKANDPLACE_HOLD_DUTY);
  moveToBlocking(dropX, dropY, SPEED_CARRY);

  // --- 5) ellentetes polaritasu kick, majd varakozas amig eltunik ---
  soundEvent("drop");
  magnetSetDuty(MAGNET_SOUTH, 255);
  delay(PICKANDPLACE_KICK_MS);
  releaseMagnet();
  delay(PICKANDPLACE_CLEAR_MS);

  webLog("PICKANDPLACE: kesz.");
}

// ============ GOHOME: cella(0,0)-ra all (BTN5 az EDIT_MENU-ben) ============
void executeGoHome() {
  if (!gridCornerXYValid) {
    webLog("GOHOME: HIBA - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
    return;
  }
  float x, y;
  if (!trilaterateSteps(gridCellA[0], gridCellB[0], x, y)) {
    webLog("GOHOME: HIBA - step->XY atszamitas sikertelen.");
    return;
  }
  moveToBlocking(x, y, SPEED_TRAVEL);
  syncStateFromMotors();
  webLog("GOHOME: cella(0,0) elerve.");
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
// Uzem kozben allithato a setSpeed paranccsal (lasd cmdSetSpeed) - tul magas
// ertekek mellett a stepper (kulonosen A) idonkent megszaladt/lepesvesztett.
float JOG_SPEED_MM_S  = 40.0;
float JOG_ACCEL_MM_S2 = 250.0;

void jogMotorStart(FastAccelStepper *mot, int dir) {
  mot->setSpeedInHz(mmSpeedToStepsHz(JOG_SPEED_MM_S));
  mot->setAcceleration(mmAccelToStepsS2(JOG_ACCEL_MM_S2));
  if (dir > 0) mot->runForward(); else mot->runBackward();
}
void jogMotorStop(FastAccelStepper *mot) {
  mot->stopMove();
}

// ============ MOVE/CELLS JOG: kalibralt racs-terben, normalizalt (u,v) ============
// M2: mindket motor a SAJAT delta-javal aranyos sebesseget ES gyorsulast
// kap (nem csak a sebesseget) - igy v_A/a_A = v_B/a_B = v_max/a_max
// mindket tengelyen, tehat a gyorsitasi/lassitasi ido (t=v/a) egyenlo.
// Ez onmagaban meg NEM eleg: egy tavoli celra torteno EGYETLEN, step-terben
// egyenes mozgas a valos XY-sikban akkor is ivnek latszik, ha a cel maga
// egy valodi XY-egyenesen van (a step<->XY lekepezes nemlinearis - ugyanaz
// a jelenseg, mint a PLAY-mod moveToBlocking()-jaban, l. ott). Ezert a jog is
// a VALODI XY-egyenes menten halad, JOG_SEGMENT_LEN_MM-es kozbenso pontokkal,
// amiket replan-cikkusonkent (JOG_REPLAN_MS) egyesevel tovabb tolunk, amint a
// motor eleg kozel ert az elozohoz - igy a nyomva tartas alatt a mozgas nem
// all meg kozben, csak a celzott pont valik fokozatosan tavolabbivá.
const float JOG_SEGMENT_LEN_MM = 3.0f;
float jogLineStartX = 0, jogLineStartY = 0;
float jogLineEndX = 0, jogLineEndY = 0;
int jogSegIndex = 0, jogSegCount = 1;
long jogAimA = 0, jogAimB = 0;

void moveJogApply(float u, float v, bool startNewLine) {
  if (gridCornerMask != 0x0F || !motorA || !motorB || !gridCornerXYValid) return;

  if (startNewLine) {
    // uj egyenes indul a motor TENYLEGES jelenlegi poziciojabol a tavoli
    // celig - ez csak az iranyt/hosszat hatarozza meg, allapotot nem ir felul.
    if (!trilaterateSteps(motorA->getCurrentPosition(), motorB->getCurrentPosition(), jogLineStartX, jogLineStartY)) return;
    gridUVToXY(u, v, jogLineEndX, jogLineEndY);
    float dist = sqrt(sq(jogLineEndX - jogLineStartX) + sq(jogLineEndY - jogLineStartY));
    jogSegCount = max(1, (int)ceil(dist / JOG_SEGMENT_LEN_MM));
    jogSegIndex = 0;
    jogAimA = motorA->getCurrentPosition();
    jogAimB = motorB->getCurrentPosition();
  }

  if (jogSegIndex < jogSegCount &&
      abs(jogAimA - motorA->getCurrentPosition()) <= PLAY_WAYPOINT_TOLERANCE_STEPS &&
      abs(jogAimB - motorB->getCurrentPosition()) <= PLAY_WAYPOINT_TOLERANCE_STEPS) {
    jogSegIndex++;
    float t = (float)jogSegIndex / jogSegCount;
    float ax = jogLineStartX + (jogLineEndX - jogLineStartX) * t;
    float ay = jogLineStartY + (jogLineEndY - jogLineStartY) * t;
    xyToSteps(ax, ay, jogAimA, jogAimB);
  }

  applyProportionalMove(jogAimA, jogAimB, JOG_SPEED_MM_S, JOG_ACCEL_MM_S2);
}

// A loop()-bol minden ciklusban hivva - amig LEFT/RIGHT/UP/DOWN nyomva van,
// periodikusan (JOG_REPLAN_MS-enkent) ujratervezi a celt: a nyomott
// irany(ok)ban a racs SZELET (0.0/1.0) celozzuk meg, nem egy apro lepest -
// igy a motor eleri es tartja a nevleges sebesseget a fogva tartas alatt
// (nem all vissza folyamatosan gyorsulasi rampara, ami korabban a lassu
// bepottyanast/ivelodest okozta). A nem hajtott tengely celja a PARANCSOLT
// (moveU/moveV) ertek marad - sose a mozgas kozben mert poziciobol
// ujraszamolt ertek, mert az a kotelhossz-interpolacio maradek-hibajat
// minden replannal beleepitene az allapotba. Elengedeskor azonnal megallitja
// a motort es szinkronizalja az allapotot a tenyleges pozicioval.
unsigned long JOG_REPLAN_MS = 20;
void updateMoveJogTick() {
  if (appMode != MODE_EDIT) return;
  if (editSubMode != EDIT_MOVE && editSubMode != EDIT_CELLS) return;

  static unsigned long lastReplanMs = 0;
  static bool jogWasActive = false;
  // Uj allapot: gomb mar elengedve, de a motorok meg fekeznek - amig ez
  // igaz, NEM szabad syncStateFromMotors()-t hivni, mert a lepesszamlalo
  // meg egy koztes, tengelyenkent elteru pillanatban van (ez okozta az
  // eredeti kumulalodo sullyedest).
  static bool waitingForFullStop = false;
  // melyik tengelyt hajtottuk a most vegzodo jog alatt (lasd a visszaszinkronizalast)
  static bool drivenU = false, drivenV = false;

  bool anyHeld = (stableState[BTN8] == LOW) || (stableState[BTN2] == LOW) ||
                 (stableState[BTN4] == LOW) || (stableState[BTN5] == LOW);

  if (!anyHeld) {
    if (jogWasActive) {
      // A moveJogApply() aranyos gyorsulast is ad, igy a fekut (v^2/2a) is
      // aranyos a tengelyek delta-javal - a szabalyos stopMove() fekezes
      // ezert egyenes vonalu marad mindket oldalon.
      motorA->stopMove();
      motorB->stopMove();
      jogWasActive = false;
      waitingForFullStop = true;
    }
    if (waitingForFullStop) {
      if (!motorA->isRunning() && !motorB->isRunning()) {
        float readU, readV;
        if (gridUVForSteps(motorA->getCurrentPosition(), motorB->getCurrentPosition(), readU, readV)) {
          // A hajtott tengelyen a pozicio lemarad a celtol -> merni kell, kulonben
          // a parancsolt ertek elszaladna. A nem hajtott tengelyen viszont csak a
          // kotelhossz-interpolacio ive latszik -> a parancsolt erteket tartjuk,
          // kulonben az iv ciklusonkent beleepulne (ez volt a kumulalodo sullyedes).
          if (drivenU) moveU = readU;
          if (drivenV) moveV = readV;
        }
        waitingForFullStop = false;
        drivenU = drivenV = false;
      }
    }
    return;
  }

  // Uj gombnyomas jott, mielott a korabbi megallas lezarult volna -
  // ervenytelenitjuk a varakozast, a kovetkezo replan ugyis uj celt ad.
  waitingForFullStop = false;

  unsigned long now = millis();
  if (jogWasActive && (now - lastReplanMs) < JOG_REPLAN_MS) return; // meg ne tervezzunk ujra
  bool startNewLine = !jogWasActive; // friss gombnyomas - uj egyenest kell inditani a jelenlegi poziciobol
  lastReplanMs = now;
  jogWasActive = true;

  // alapertelmezesben a parancsolt U/V pozicio marad a cel
  float tgtU = moveU;
  float tgtV = moveV;

  if (stableState[BTN8] == LOW) { tgtU = 0.0f; drivenU = true; } // LEFT
  if (stableState[BTN2] == LOW) { tgtU = 1.0f; drivenU = true; } // RIGHT
  if (stableState[BTN4] == LOW) { tgtV = 0.0f; drivenV = true; } // UP
  if (stableState[BTN5] == LOW) { tgtV = 1.0f; drivenV = true; } // DOWN

  // ha nyomva tartas kozben valtozik a celzott irany (pl. uj tengely is
  // csatlakozik), az is uj egyenest igenyel, kulonben a regi vonal menten
  // haladna tovabb a mar ervenytelen celhoz.
  static float lastTgtU = -999.0f, lastTgtV = -999.0f;
  if (tgtU != lastTgtU || tgtV != lastTgtV) startNewLine = true;
  lastTgtU = tgtU;
  lastTgtV = tgtV;

  moveJogApply(tgtU, tgtV, startNewLine);
}

// Az EDIT menu SETUP-belepes (3-as gomb 1x) fuggoben tartasat zarja le, ha
// letelt a keslekedesi ablak es kozben nem erkezett meg 8 koppintas (lasd fent).
void storeCorner(int cornerIdx, const char *label);

void updatePendingMenuActions() {
  if (pendingSetupEntry && millis() - pendingSetupEntryTime >= MENU_COMMIT_DELAY_MS) {
    pendingSetupEntry = false;
    if (appMode == MODE_EDIT && editSubMode == EDIT_MENU) {
      editSubMode = EDIT_SETUP;
      // Friss kalibracio indul: a regi sarkok mar nem ervenyesek. Enelkul a
      // felkesz SETUP alatt a mar betoltott (regi) sarkokhoz hasonlitanank
      // (hamis "nem mozdult el eleget" elutasitas), es minden mentes teljes
      // ujraszamolast valtana ki hibauzenettel.
      gridCornerMask = 0;
      gridCornerXYValid = false;
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
// Minimum lepestavolsag, aminek meg kell lennie egy uj sarok es az osszes mar
// tarolt sarok kozott - ha nem mozdultak el eleg messze a motorok (pl. a
// gomb tevedesbol mozgatas nelkul lett megnyomva), a mentes elutasitva.
const long MIN_CORNER_DELTA_MM = 10;

void storeCorner(int cornerIdx, const char *label) {
  long curA = motorA->getCurrentPosition();
  long curB = motorB->getCurrentPosition();
  long minDeltaSteps = mmToSteps(MIN_CORNER_DELTA_MM);
  for (int j = 0; j < 4; j++) {
    if (j == cornerIdx || !(gridCornerMask & (1 << j))) continue;
    // Polargraph geometriaban egyenes vonalu elmozdulasnal is mindket
    // kotelhossznak valtoznia kell - ezert barmelyik motor kis elmozdulasa
    // (nem csak mindketto egyutt) mar ervenytelenne teszi a sarkot.
    if (abs(curA - gridCornerA[j]) < minDeltaSteps || abs(curB - gridCornerB[j]) < minDeltaSteps) {
      webLogf("SETUP: sarok '%s' ELUTASITVA - valamelyik motor nem mozdult el elegendoet a %d. sarokhoz kepest.", label, j);
      ackLeft(4);
      return;
    }
  }
  gridCornerA[cornerIdx] = curA;
  gridCornerB[cornerIdx] = curB;
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
      case BTN6: {
        if (!gridCornerXYValid) {
          webLog("SHOWCELLS: elutasitva - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
          break;
        }
        QueueItem item = {};
        item.kind = QI_SHOWCELLS;
        if (xQueueSend(motionQueue, &item, 0) != pdTRUE) {
          webLog("SHOWCELLS: HIBA - mozgas-puffer tele.");
        } else {
          webLog("SHOWCELLS: bejaras sorba allitva.");
          ackRight(2);
        }
        break;
      }
      case BTN7: {
        if (!gridCornerXYValid) {
          webLog("PICKANDPLACE: elutasitva - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
          break;
        }
        QueueItem item = {};
        item.kind = QI_PICKANDPLACE;
        item.pickCol = 2; item.pickRow = 1;
        item.dropCol = 3; item.dropRow = 4;
        if (xQueueSend(motionQueue, &item, 0) != pdTRUE) {
          webLog("PICKANDPLACE: HIBA - mozgas-puffer tele.");
        } else {
          webLog("PICKANDPLACE: teszt sorba allitva.");
          ackRight(2);
        }
        break;
      }
      case BTN5: {
        if (!gridCornerXYValid) {
          webLog("GOHOME: elutasitva - nincs ervenyes racs-kalibracio (fejezd be eloszor a SETUP-ot).");
          break;
        }
        QueueItem item = {};
        item.kind = QI_GOHOME;
        if (xQueueSend(motionQueue, &item, 0) != pdTRUE) {
          webLog("GOHOME: HIBA - mozgas-puffer tele.");
        } else {
          webLog("GOHOME: sorba allitva.");
          ackRight(2);
        }
        break;
      }
      default: break;
    }
    return;
  }

  // Kozos ESC minden almodban: gomb3 x3 egymas utan
  if (i == BTN3 && tapCount[BTN3] == 3) {
    tapCount[BTN3] = 0;
    pendingSetupCornerStore = false; // ne fusson le kesobb egy mar ervenytelenitett tarolas
    // Felbehagyott SETUP: a belepeskor kinullazott RAM-allapot helyett allitsuk
    // vissza az NVS-ben meg meglevo, ervenyes kalibraciot.
    if (editSubMode == EDIT_SETUP && gridCornerMask != 0x0F) {
      loadGridCorners();
      computeGridCornerXY();
      webLog("SETUP megszakitva - a korabbi kalibracio visszaallitva.");
    }
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
          soundEvent("click");
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
  String secStr = splitToken(args, pos, ',');
  String dirStr = args.substring(pos);
  if (pctStr.length() == 0 || secStr.length() == 0) return "HIBA: testMagnet formatum: PCT,SEC[,DIR]";
  int pct = pctStr.toInt();
  int sec = secStr.toInt();
  if (pct < 0 || pct > 100) return "HIBA: PCT 0..100 kozott lehet";
  if (sec <= 0 || sec > 3600) return "HIBA: SEC 1..3600 kozott lehet";

  dirStr.trim();
  dirStr.toLowerCase();
  MagnetDirection dir = MAGNET_NORTH;
  if (dirStr == "s" || dirStr == "south" || dirStr == "1") dir = MAGNET_SOUTH;
  else if (dirStr.length() > 0 && dirStr != "n" && dirStr != "north" && dirStr != "0")
    return "HIBA: DIR csak n/north/0 (eszak) vagy s/south/1 (del) lehet";

  QueueItem item = {};
  item.kind = QI_MAGNET_TEST;
  item.magnetDutyPercent = (uint8_t)pct;
  item.magnetDurationSec = (uint16_t)sec;
  item.magnetDirection = (uint8_t)dir;
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (testMagnet). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

// Fix teszt-eljaras: 0-alapu (2,1) cellabol felveszi, (3,4) cellara ledobja - lasd executePickAndPlace().
String cmdPickAndPlace(const String &args) {
  (void)args;
  QueueItem item = {};
  item.kind = QI_PICKANDPLACE;
  item.pickCol = 2; item.pickRow = 1;
  item.dropCol = 3; item.dropRow = 4;
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (pickAndPlace). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

String cmdGoHome(const String &args) {
  (void)args;
  QueueItem item = {};
  item.kind = QI_GOHOME;
  if (xQueueSend(motionQueue, &item, pdMS_TO_TICKS(100)) != pdTRUE) return "HIBA: puffer tele";
  return "OK: sorba allitva (goHome). Pufferben: " + String((int)uxQueueMessagesWaiting(motionQueue));
}

// restartAP - a KOVETKEZO (es csak azon egyetlen) bootnal AP modot kenyszerit,
// fuggetlenul az ismert WiFi-halozatok elerhetosegetol. A flag-et setup() a
// legelso dolgok kozott torli, igy utana megint a normal STA->AP-fallback fut.
String cmdRestartAP(const String &args) {
  (void)args;
  prefs.putBool("forceApOnce", true);
  webLog("restartAP: kovetkezo inditasnal AP mod kenyszeritve, ujrainditas...");
  delay(500);
  ESP.restart();
  return "";
}

// cellCode {REKESZ,KOD},{REKESZ,KOD},... - rekesz-kod parok beallitasa/felulirasa (a 32-es
// rekesz a fix ledobo hely, nem kene felvetelre kodolni, de nincs kulon tiltva). Parameter
// nelkul a jelenlegi teljes tablat listazza ki.
String cmdCellCode(const String &args) {
  if (args.length() == 0) {
    String out;
    for (int r = 1; r <= REKESZ_COUNT; r++) {
      out += String(r) + ",";
      out += (rekeszCodeTable[r][0] != '\0') ? String(rekeszCodeTable[r]) : "-";
      if (rekeszCodeTable[r][0] != '\0') {
        bool proc = (rekeszProcessedBits >> (r - 1)) & 1ULL;
        out += proc ? " [processed]" : " [ready]";
      }
      if (r == DROP_REKESZ) out += " (LEDOBO)";
      out += "\n";
    }
    return out;
  }

  int pos = 0, n = args.length(), count = 0;
  while (pos < n) {
    if (args[pos] != '{') return "HIBA: cellCode formatum: {REKESZ,KOD},{REKESZ,KOD},...";
    pos++;
    int c = args.indexOf(',', pos);
    if (c < 0) return "HIBA: cellCode formatum: {REKESZ,KOD},{REKESZ,KOD},...";
    int rekesz = args.substring(pos, c).toInt();
    pos = c + 1;
    int e = args.indexOf('}', pos);
    if (e < 0) return "HIBA: cellCode formatum: {REKESZ,KOD},{REKESZ,KOD},...";
    String code = args.substring(pos, e);
    pos = e + 1;
    if (pos < n) {
      if (args[pos] != ',') return "HIBA: cellCode formatum: {REKESZ,KOD},{REKESZ,KOD},...";
      pos++;
    }

    if (rekesz < 1 || rekesz > REKESZ_COUNT) return "HIBA: ervenytelen rekeszszam: " + String(rekesz);
    if (code.length() == 0 || code.length() > 4) return "HIBA: a kod max 4 karakter lehet (rekesz " + String(rekesz) + ")";

    code.toCharArray(rekeszCodeTable[rekesz], sizeof(rekeszCodeTable[rekesz]));
    rekeszProcessedBits &= ~(1ULL << (rekesz - 1)); // uj/felulirt kod - ujra kiadhato
    count++;
  }

  saveRekeszCodes();
  saveRekeszProcessedBits();
  return "OK: " + String(count) + " rekesz-kod par mentve.";
}

// cellReset - CSAK a rekesz-kod rendszer "mar kiadva" jelzeseit nullazza (a regi
// setCell/cellCache processedBits-jet nem erinti, arra a clearCells valo).
String cmdCellReset(const String &args) {
  (void)args;
  rekeszProcessedBits = 0;
  saveRekeszProcessedBits();
  return "OK: rekesz-kod kiadottsag nullazva.";
}

// cellRepeat 0|1 - parameterezi, hogy egy mar egyszer kiadott rekesz kodja ujra
// elfogadhato-e. 0 (alapertelmezett) = ujra elfogadhato, 1 = csak egyszer adhato ki.
// Parameter nelkul a jelenlegi beallitast irja ki.
String cmdCellRepeat(const String &args) {
  if (args.length() == 0) {
    return "cellRepeat=" + String(rekeszBlockRepeat ? 1 : 0) + " (" +
           (rekeszBlockRepeat ? "tiltva az ujboli kiadas" : "ujra kiadhato") + ")";
  }
  int v = args.toInt();
  if (v != 0 && v != 1) return "HIBA: cellRepeat 0 (ujra kiadhato) vagy 1 (csak egyszer) lehet";
  rekeszBlockRepeat = (v == 1);
  prefs.putBool("rekeszBlockRep", rekeszBlockRepeat);
  return "OK: cellRepeat=" + String(rekeszBlockRepeat ? 1 : 0);
}

// setPick DIP_MM,SETTLE_MS,BOOST_MS,LIFT_MM,HOLD_PCT,KICK_MS,CLEAR_MS
// pl. setPick 5,150,300,10,70,500,1000 - parameter nelkul a jelenlegi ertekeket irja ki.
String cmdSetPick(const String &args) {
  if (args.length() == 0) {
    return "setPick DIP_MM=" + String(PICKANDPLACE_DIP_MM, 1) +
           " SETTLE_MS=" + String(PICKANDPLACE_SETTLE_MS) +
           " BOOST_MS=" + String(PICKANDPLACE_BOOST_MS) +
           " LIFT_MM=" + String(PICKANDPLACE_LIFT_MM, 1) +
           " HOLD_PCT=" + String((int)((int)PICKANDPLACE_HOLD_DUTY * 100 / 255)) +
           " KICK_MS=" + String(PICKANDPLACE_KICK_MS) +
           " CLEAR_MS=" + String(PICKANDPLACE_CLEAR_MS);
  }

  int pos = 0;
  String tok[7];
  for (int i = 0; i < 7; i++) {
    tok[i] = (i == 6) ? args.substring(pos) : splitToken(args, pos, ',');
    if (tok[i].length() == 0) {
      return "HIBA: setPick formatum: DIP_MM,SETTLE_MS,BOOST_MS,LIFT_MM,HOLD_PCT,KICK_MS,CLEAR_MS";
    }
  }

  float dip     = tok[0].toFloat();
  long  settle  = tok[1].toInt();
  long  boost   = tok[2].toInt();
  float lift    = tok[3].toFloat();
  int   holdPct = tok[4].toInt();
  long  kick    = tok[5].toInt();
  long  clear   = tok[6].toInt();

  if (dip < 0 || dip > 50)        return "HIBA: DIP_MM 0..50 kozott lehet";
  if (settle < 0 || settle > 5000) return "HIBA: SETTLE_MS 0..5000 kozott lehet";
  if (boost < 0 || boost > 5000)   return "HIBA: BOOST_MS 0..5000 kozott lehet";
  if (lift < 0 || lift > 100)      return "HIBA: LIFT_MM 0..100 kozott lehet";
  if (holdPct < 0 || holdPct > 100) return "HIBA: HOLD_PCT 0..100 kozott lehet";
  if (kick < 0 || kick > 5000)     return "HIBA: KICK_MS 0..5000 kozott lehet";
  if (clear < 0 || clear > 10000)  return "HIBA: CLEAR_MS 0..10000 kozott lehet";

  PICKANDPLACE_DIP_MM    = dip;
  PICKANDPLACE_SETTLE_MS = (unsigned long)settle;
  PICKANDPLACE_BOOST_MS  = (unsigned long)boost;
  PICKANDPLACE_LIFT_MM   = lift;
  PICKANDPLACE_HOLD_DUTY = (uint8_t)(holdPct * 255 / 100);
  PICKANDPLACE_KICK_MS   = (unsigned long)kick;
  PICKANDPLACE_CLEAR_MS  = (unsigned long)clear;
  return "OK: " + cmdSetPick("");
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

// setSpeed JOG_SPEED_MM_S,JOG_ACCEL_MM_S2,JOG_REPLAN_MS - pl. 40.00,250.00,20
String cmdSetSpeed(const String &args) {
  int pos = 0;
  String speedStr = splitToken(args, pos, ',');
  String accelStr = splitToken(args, pos, ',');
  String replanStr = args.substring(pos);
  if (speedStr.length() == 0 || accelStr.length() == 0 || replanStr.length() == 0) {
    return "HIBA: setSpeed formatum: SPEED_MM_S,ACCEL_MM_S2,REPLAN_MS";
  }
  float speed = speedStr.toFloat();
  float accel = accelStr.toFloat();
  long replan = replanStr.toInt();
  if (speed <= 0 || accel <= 0 || replan <= 0) return "HIBA: minden ertek pozitiv kell legyen";
  JOG_SPEED_MM_S = speed;
  JOG_ACCEL_MM_S2 = accel;
  JOG_REPLAN_MS = (unsigned long)replan;
  return "OK: JOG_SPEED_MM_S=" + String(JOG_SPEED_MM_S, 2) + " JOG_ACCEL_MM_S2=" + String(JOG_ACCEL_MM_S2, 2) +
         " JOG_REPLAN_MS=" + String(JOG_REPLAN_MS);
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
  out += "GONDOLA_RADIUS_MM=" + String(GONDOLA_RADIUS_MM, 1) +
         " ARM_ANGLE_DEG=" + String(GONDOLA_ARM_ANGLE_RAD * 180.0 / PI, 1) +
         " thetaFwd=" + String(lastThetaForward, 4) + " thetaInv=" + String(lastThetaInverse, 4) + "\n";
  out += "STEPS_PER_MM=" + String(STEPS_PER_MM, 4) + " MICROSTEPPING=" + String(MICROSTEPPING, 0) + "\n";
  return out;
}

// genAutocells [P0,P1,...,P6] - opcionalis GRID_COLS db vesszovel elvalasztott szazalek
// (COL_Y_COMP_PERCENT felulirasa); parameter nelkul csak a betarolt ertekeket irja ki
// es (ha van ervenyes kalibracio) ujraszamolja veluk a cellakat.
String cmdGenAutocells(const String &args) {
  bool paramsProvided = (args.length() > 0);
  if (paramsProvided) {
    float vals[GRID_COLS];
    int pos = 0;
    int count = 0;
    for (count = 0; count < GRID_COLS; count++) {
      String tok = (count == GRID_COLS - 1) ? args.substring(pos) : splitToken(args, pos, ',');
      if (tok.length() == 0) break;
      vals[count] = tok.toFloat();
    }
    if (count != GRID_COLS) {
      return "HIBA: genAutocells " + String(GRID_COLS) + " vesszovel elvalasztott szazalekot var (pl. 5,3,0,-3,0,3,5).";
    }
    for (int i = 0; i < GRID_COLS; i++) COL_Y_COMP_PERCENT[i] = vals[i];
  }

  String percentList;
  for (int i = 0; i < GRID_COLS; i++) {
    percentList += formatNum(COL_Y_COMP_PERCENT[i]);
    if (i < GRID_COLS - 1) percentList += ",";
  }

  if (!gridCornerXYValid) {
    return "COL_Y_COMP_PERCENT=" + percentList + " (nincs ervenyes racs-kalibracio, cellak nem lettek ujraszamolva)";
  }
  if (!paramsProvided) {
    return "COL_Y_COMP_PERCENT (betarolt ertekek)=" + percentList;
  }
  computeDefaultGridCells();
  return "OK: uj COL_Y_COMP_PERCENT beallitva, cellak ujraszamolva=" + percentList;
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
  if (cmd == "pickAndPlace") return cmdPickAndPlace(args);
  if (cmd == "goHome")      return cmdGoHome(args);
  if (cmd == "restartAP")   return cmdRestartAP(args);
  if (cmd == "cellCode")    return cmdCellCode(args);
  if (cmd == "cellReset")   return cmdCellReset(args);
  if (cmd == "cellRepeat")  return cmdCellRepeat(args);
  if (cmd == "listCell")     return cmdListCell();
  if (cmd == "clearCells")   return cmdClearCells();
  if (cmd == "setSpeed")     return cmdSetSpeed(args);
  if (cmd == "setPick")      return cmdSetPick(args);
  if (cmd == "geom")         return cmdGeom();
  if (cmd == "genAutocells") return cmdGenAutocells(args);
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

  ledcSetup(PWM_CHANNEL_R, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MAGNET_PWM_PIN, PWM_CHANNEL_R);
  ledcWrite(PWM_CHANNEL_R, 0);

  ledcSetup(PWM_CHANNEL_L, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MAGNET_LPWM_PIN, PWM_CHANNEL_L);
  ledcWrite(PWM_CHANNEL_L, 0);

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

  loadRekeszCodes();
  rekeszProcessedBits = prefs.getULong64("rekeszProcBits", 0);
  rekeszBlockRepeat = prefs.getBool("rekeszBlockRep", false);
  if (!prefs.getBool("rekeszInit", false)) {
    for (int i = 0; i < REKESZ_CODE_DEFAULT_COUNT; i++) {
      const RekeszCodeDefault &d = REKESZ_CODE_DEFAULTS[i];
      strncpy(rekeszCodeTable[d.rekesz], d.code, 4);
      rekeszCodeTable[d.rekesz][4] = '\0';
    }
    saveRekeszCodes();
    prefs.putBool("rekeszInit", true);
    webLog("REKESZKOD: alapertelmezett rekesz-kod tabla betoltve (elso inditas).");
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
    motorA->setDirectionPin(DIR_A_PIN, DIR_A_INVERT);
    motorA->setAutoEnable(false);
  }
  if (motorB) {
    motorB->setDirectionPin(DIR_B_PIN, DIR_B_INVERT);
    motorB->setAutoEnable(false);
  }

  // Boot-kor a gepet mindig kezzel (0,0)-ba kell allitani - a merve ismert
  // origo-kotelhosszakat irjuk be, nem az ANCHOR_A/B-bol visszaszamolt kozelitest.
  if (motorA) motorA->setCurrentPosition(mmToSteps(ORIGIN_LEN_A_MM));
  if (motorB) motorB->setCurrentPosition(mmToSteps(ORIGIN_LEN_B_MM));
  syncStateFromMotors(); // M1: legjobb ismert (u,v) becsles boot-kor (ha van ervenyes kalibracio)

  Serial.println("Pick-and-place polargraph keszul (queue-alapu mozgatas, IBT-2 PWM magnes).");

  // ---- WiFi ----
  // A flag-et AZONNAL toroljuk (meg AP-inditas elott) - igy barhogyan is
  // vegzodik ez a boot (pl. aramkimaradas), a kenyszeritett AP mod garantaltan
  // csak EGYETLEN alkalommal ervenyesul (lasd cmdRestartAP()).
  bool forceApOnce = prefs.getBool("forceApOnce", false);
  if (forceApOnce) {
    prefs.putBool("forceApOnce", false);
    webLog("Kenyszeritett AP mod (restartAP parancs utan).");
    startAccessPoint();
  } else if (!connectToKnownWifi()) {
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

  // ---- Hangfajlok (firmware-be egetve, lasd platformio.ini embed_files) ----
  server.on("/sound/click.mp3", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "audio/mpeg", click_mp3_start, click_mp3_end - click_mp3_start);
  });
  server.on("/sound/err.mp3", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "audio/mpeg", err_mp3_start, err_mp3_end - err_mp3_start);
  });
  server.on("/sound/start.mp3", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "audio/mpeg", start_mp3_start, start_mp3_end - start_mp3_start);
  });
  server.on("/sound/drop.mp3", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "audio/mpeg", drop_mp3_start, drop_mp3_end - drop_mp3_start);
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