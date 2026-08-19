#include <Arduino.h>
#include <stdarg.h>
#include <FastAccelStepper.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <ArduinoOTA.h>

void readButtons(); // előredeklaráció - a definíció lejjebb van, a motionTask itt hívja

// ============ PIN KIOSZTÁS ============
#define STEP_A_PIN     25
#define DIR_A_PIN      26
#define STEP_B_PIN     27
#define DIR_B_PIN      14
#define ENABLE_PIN     13   // közös ENABLE mindkét A4988-nak

#define MAGNET_PWM_PIN 32   // IBT-2 RPWM
#define MAGNET_EN_PIN  33   // IBT-2 R_EN (folyamatosan HIGH-on tartva)
// IBT-2 L_EN / LPWM nincs bekötve az ESP32-ről (fizikailag GND-re kötve a panelen),
// mert csak egyirányú kapcsolásra van szükség.

// buttonPins index -> Gomb szám (1-alapú, ahogy a fizikai panelen fel van irva):
//   index0=GPIO4->Gomb1, index1=GPIO5->Gomb2, index2=GPIO18->Gomb3, index3=GPIO19->Gomb4,
//   index4=GPIO34->Gomb5, index5=GPIO23->Gomb6, index6=GPIO36->Gomb7, index7=GPIO39->Gomb8
// Gomb6 (GPIO23) csere GPIO35-rol (akku-mero utkozes miatt), belso+kulso 4.7k pull-up.
const int buttonPins[8] = {4, 5, 18, 19, 34, 23, 36, 39};
bool lastButtonState[8]  = {true, true, true, true, true, true, true, true};
unsigned long lastDebounceTime[8] = {0};
const unsigned long debounceDelay = 30;

// ============ GÉP GEOMETRIA ============
const float ANCHOR_A_X = -200, ANCHOR_A_Y = -200;
const float ANCHOR_B_X =  920, ANCHOR_B_Y = -200;
const float STEPS_PER_MM = 80.0;

const float SPEED_TRAVEL = 120.0;
const float SPEED_CARRY  = 60.0;

// ============ ELEKTROMÁGNES PWM PARAMÉTEREK ============
// A telepitett Arduino-ESP32 core 2.x-es (a ledcAttach() nem letezik nala),
// ezert a klasszikus, csatorna-alapu LEDC API kell: ledcSetup + ledcAttachPin + ledcWrite(channel,...).
const int PWM_CHANNEL    = 0;
const int PWM_FREQ_HZ    = 1000;  // 500Hz-2kHz tartományban jó, 1kHz biztos középérték
const int PWM_RES_BITS   = 8;     // 0-255 duty
const unsigned long MAGNET_BOOST_MS = 200;   // 100% duty ennyi ideig felszedéskor
const uint8_t MAGNET_HOLD_DUTY = 130;        // ~51% duty tartáshoz - MÉRÉS ALAPJÁN HANGOLD!
                                              // (tekercs ~3.4 ohm, 12V-on ~3.5A csucs, tartasnal ennel jóval kevesebb)

// ============ SZEKVENCIA FORMÁTUM ============
struct MoveStep {
  float x, y;
  uint8_t t;   // varakozas mp-ben erkezes utan, 0=azonnal, 255=vegtelen (gombsor ujra aktiv)
  uint8_t m;   // magnes celallapot: 1=felveszi (boost+hold), 0=elengedi
};

#define MAX_STEPS 10
struct CellDef {
  const char* code;
  uint8_t stepCount;
  MoveStep steps[MAX_STEPS];
};

CellDef cells[35] = {
  { "100001", 6, {
      {70,70,1,1}, {100,100,0,1}, {680,100,0,1}, {680,400,0,1},
      {710,430,2,0}, {680,70,255,0}
  }},
  { "100002", 6, {
      {170,70,1,1}, {200,100,0,1}, {680,100,0,1}, {680,400,0,1},
      {710,430,2,0}, {680,70,255,0}  // TODO: valós útvonal
  }},
  { "100001", 6, {
      {270,70,1,1}, {300,100,0,1}, {680,100,0,1}, {680,400,0,1},
      {710,430,2,0}, {680,70,255,0}  // TODO
  }},
  // TODO: a maradék 32 cella ugyanígy
};
const int CELL_COUNT = 3;

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

// ============ ÉLŐ WEB-LOG (SSE) - akkor is látszik, ha nincs USB ============
String logHistory = "";
const size_t LOG_HISTORY_MAX = 4000; // karakter, korlátozott előzmény

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
  char buf[160];
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
<h2>Wicked Pick and Place - élő log V1.34</h2>
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

bool connectToKnownWifi(unsigned long timeoutMsPerNetwork = 8000) {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(MDNS_HOSTNAME); // igy a router kliens-listajaban is "wickedpickandplace" nevvel jelenik meg
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

// ============ MOZGÁS-TASK (DEDIKÁLT MAGON, BLOKKOLÓ DELAY-EKKEL) ============
void motionTask(void *parameter) {
  for (;;) {
    readButtons();
    vTaskDelay(pdMS_TO_TICKS(5)); // a runDeliverySequence() belül futó delay()-ek itt, ezen a magon blokkolnak - a webszervert nem érintik
  }
}


FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *motorA = NULL;
FastAccelStepper *motorB = NULL;

float currentX = 680.0, currentY = 70.0;
bool penDown = false;
bool buttonsActive = true;
String codeBuffer = "";

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

void moveToBlocking(float x, float y, float speed) {
  moveTo(x, y, speed);
  while (motorA->isRunning() || motorB->isRunning()) {
    delay(2);
  }
}

// ============ ELEKTROMÁGNES: BOOST-THEN-HOLD ============
// Core 2.x LEDC API: csatorna-alapú - ledcWrite(channel, duty).
// Visszaadja, hány ms telt el a hívás alatt, hogy a hívó le tudja vonni a
// szekvencia-lépés hátralévő várakozási idejéből.
unsigned long engageMagnet() {
  ledcWrite(PWM_CHANNEL, 255);       // 100% duty - nagy fluxus a felszedéshez
  delay(MAGNET_BOOST_MS);
  ledcWrite(PWM_CHANNEL, MAGNET_HOLD_DUTY); // csökkentett duty a tartáshoz
  penDown = true;
  return MAGNET_BOOST_MS;
}

unsigned long releaseMagnet() {
  ledcWrite(PWM_CHANNEL, 0);
  penDown = false;
  return 0;
}

// ============ SZEKVENCIA VÉGREHAJTÁS ============
void executeStep(const MoveStep &step) {
  float speed = penDown ? SPEED_CARRY : SPEED_TRAVEL;
  moveToBlocking(step.x, step.y, speed);

  delay(200); // mágnesváltás mindig 200ms-mal érkezés után
  unsigned long extraMs = (step.m == 1) ? engageMagnet() : releaseMagnet();

  if (step.t == 255) return; // vegtelen varakozas - hivo kezeli a gombsor ujraaktivalasat
  if (step.t > 0) {
    long remainingMs = (long)step.t * 1000L - 200L - (long)extraMs;
    if (remainingMs > 0) delay(remainingMs);
  }
}

void runDeliverySequence(int idx) {
  buttonsActive = false;
  CellDef &cell = cells[idx];
  webLogf("Cella #%d szekvencia inditasa (%d lepes)", idx, cell.stepCount);

  for (int i = 0; i < cell.stepCount; i++) {
    executeStep(cell.steps[i]);
    if (cell.steps[i].t == 255) break;
  }

  webLog("Kesz, varakozas a kovetkezo kodra.");
  buttonsActive = true;
}

// ============ CELLAKERESÉS ============
int findCellByCode(const String &code) {
  for (int i = 0; i < CELL_COUNT; i++) {
    if (code.equals(cells[i].code)) return i;
  }
  return -1;
}

void indicateNotFound() {
  webLog("Nincs ilyen kod.");
}

void handleDigit(char digit) {
  if (!buttonsActive) return;
  codeBuffer += digit;
  webLogf("Kod: %s", codeBuffer.c_str());
  if (codeBuffer.length() >= 6) {
    int idx = findCellByCode(codeBuffer);
    codeBuffer = "";
    if (idx < 0) {
      indicateNotFound();
    } else {
      runDeliverySequence(idx);
    }
  }
}

// ============ GOMBKEZELÉS ============
void readButtons() {
  static bool stableState[8] = {true,true,true,true,true,true,true,true};
  for (int i = 0; i < 8; i++) {
    bool reading = digitalRead(buttonPins[i]);
    if (reading != lastButtonState[i]) lastDebounceTime[i] = millis();
    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (reading != stableState[i]) {
        stableState[i] = reading;
        if (reading == LOW) handleDigit('1' + i);
      }
    }
    lastButtonState[i] = reading;
  }
}

// ============ SETUP / LOOP ============
void setup() {
  Serial.begin(115200);
  delay(300); // idő, hogy a soros monitor csatlakozni tudjon indulás után
  Serial.printf("PSRAM meret: %d bajt\n", ESP.getPsramSize());
  // Ha ez 0-t ir ki, nincs PSRAM a modulon - a build_flags-bol a PSRAM sorokat
  // ki kell venni (platformio.ini), kulonben boot-hiba johet.

  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);

  pinMode(MAGNET_EN_PIN, OUTPUT);
  digitalWrite(MAGNET_EN_PIN, HIGH); // IBT-2 R_EN folyamatosan engedélyezve

  // Core 2.x LEDC API: ledcSetup(channel, freq, resolution) + ledcAttachPin(pin, channel).
  ledcSetup(PWM_CHANNEL, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(MAGNET_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0); // induláskor mágnes ki

  // Belso pull-up (INPUT_PULLUP) ott, ahol a lab tamogatja - Gomb1-4 (GPIO4,5,18,19)
  // es Gomb6 (GPIO23) -, mindegyik mellett kulso 4.7k is van a semaban.
  // Gomb5,7,8 (GPIO34,36,39) input-only labak, azoknal nincs belso pull-up,
  // csak a kulso 4.7k tartja HIGH-on nyugalmi allapotban -> sima INPUT kell.
  pinMode(buttonPins[0], INPUT_PULLUP);  // GPIO4  - Gomb1
  pinMode(buttonPins[1], INPUT_PULLUP);  // GPIO5  - Gomb2
  pinMode(buttonPins[2], INPUT_PULLUP);  // GPIO18 - Gomb3
  pinMode(buttonPins[3], INPUT_PULLUP);  // GPIO19 - Gomb4
  pinMode(buttonPins[4], INPUT);         // GPIO34 - Gomb5 (input-only)
  pinMode(buttonPins[5], INPUT_PULLUP);  // GPIO23 - Gomb6 (belso+kulso pull-up)
  pinMode(buttonPins[6], INPUT);         // GPIO36 - Gomb7 (input-only)
  pinMode(buttonPins[7], INPUT);         // GPIO39 - Gomb8 (input-only)

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

  Serial.println("Pick-and-place polargraph kesz (IBT-2 PWM magnes). Ird be a 6 jegyu kodot.");

  // ---- WiFi: ismert halozatok, ha egyik sem elerheto, sajat AP ----
  if (!connectToKnownWifi()) {
    startAccessPoint();
  }

  if (MDNS.begin(MDNS_HOSTNAME)) {
    webLogf("mDNS aktiv: http://%s.local", MDNS_HOSTNAME);
  } else {
    webLog("mDNS inditasa sikertelen.");
  }

  // ---- Webszerver: kezdolap + OTA ----
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/html",
      "<h1>Wicked Pick and Place</h1>"
      "<p>Firmware fut.</p>"
      "<p><a href='/update'>Firmware feltoltes (OTA)</a></p>"
      "<p><a href='/log'>Elo log (terminal nelkul is)</a></p>");
  });
  server.on("/log", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", LOG_PAGE_HTML);
  });
  events.onConnect([](AsyncEventSourceClient *client) {
    if (logHistory.length() > 0) {
      client->send(logHistory.c_str(), "log", millis(), 1000);
    }
  });
  server.addHandler(&events);
  ElegantOTA.begin(&server);
  server.begin();
  webLog("Webszerver elindult (port 80).");

  // ---- ArduinoOTA: PlatformIO "Upload" gombbal hálózaton keresztüli feltöltéshez ----
  ArduinoOTA.setHostname(MDNS_HOSTNAME);
  // ArduinoOTA.setPassword("VALASSZ_JELSZOT"); // ajanlott elesben bekapcsolni, ld. README
  ArduinoOTA.onStart([]() {
    webLog("ArduinoOTA: feltoltes inditva");
  });
  ArduinoOTA.onEnd([]() {
    webLog("ArduinoOTA: feltoltes kesz, ujraindulas...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("ArduinoOTA: %u%%\r", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    webLogf("ArduinoOTA hiba [%u]", error);
  });
  ArduinoOTA.begin();
  webLog("ArduinoOTA aktiv (PlatformIO 'Upload' gombhoz).");

  // ---- Mozgas-vezerles dedikalt task-ba, kulon magra pin-elve ----
  // A runDeliverySequence()-en beluli delay()-ek itt blokkolnak, a webszervert
  // (loop()-on/core1-en fut tovabb async modban) ez nem erinti.
  xTaskCreatePinnedToCore(
    motionTask,      // task fuggveny
    "motionTask",    // nev
    8192,            // stack meret
    NULL,            // parameter
    1,               // prioritas
    NULL,            // task handle (nem kell most)
    0                // core 0 - a loop()/WiFi alapertelmezetten core1-en fut
  );
}

void loop() {
  ElegantOTA.loop(); // async modban tobbnyire nem szukseges, de ez a hivatalosan javasolt biztonsagi halo
  ArduinoOTA.handle();
  delay(10);
}