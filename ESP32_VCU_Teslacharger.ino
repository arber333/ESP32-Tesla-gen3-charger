/*
  Gen2 Tesla Charger ESP32 with WiFi AP web-serial interface
  - Uses Collin Kidder esp32_can + can_common (CAN_FRAME)
  - Single CAN (Can0)
  - millis() scheduling for periodic tasks
  - esp_task_wdt compatibility wrapper
  - WiFi AP + simple HTTP web UI at http://192.168.4.2/
  - AP SSID: TeslaCharger  Password: 123456
*/

#include <Arduino.h>
#include <esp32_can.h>
#include <esp_task_wdt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include "config.h"

// LED fallback
#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// WiFi AP settings
const char* apSSID = "TeslaCharger";
const char* apPassword = "123456";
IPAddress apLocalIP(192,168,4,2);
IPAddress apGateway(192,168,4,1);
IPAddress apSubnet(255,255,255,0);

// Web server
WebServer server(80);

// Serial log buffer
#define LOG_LINES 200
String logBuffer[LOG_LINES];
int logHead = 0;
int logCount = 0;
portMUX_TYPE logMux = portMUX_INITIALIZER_UNLOCKED;

void addLog(const String &s) {
  portENTER_CRITICAL(&logMux);
  logBuffer[logHead] = s;
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
  portEXIT_CRITICAL(&logMux);
}

String getLogs(int maxLines = 200) {
  String out = "";
  portENTER_CRITICAL(&logMux);
  int start = (logHead - logCount + LOG_LINES) % LOG_LINES;
  int lines = min(maxLines, logCount);
  for (int i = 0; i < lines; i++) {
    int idx = (start + i) % LOG_LINES;
    out += logBuffer[idx];
    out += "\n";
  }
  portEXIT_CRITICAL(&logMux);
  return out;
}

void logPrint(const String &s) {
  Serial.println(s);
  addLog(s);
}

// --- Constants
#define WDT_TIMEOUT_SECONDS 5
#define CHARGER_MSG_INTERVAL_MS 100

// --- Globals (kept names)
int evsedebug = 1;
int debug = 1;

uint16_t curset = 0;
int setting = 1;
int incomingByte = 0;
int state = 0;
unsigned long slavetimeout = 0, tlast = 0, tcan = 0, tpause = 0, tboot = 0;
bool bChargerEnabled = false;

byte Proximity = 0;
uint16_t ACvoltIN = 240;
#define Unconnected 0
#define Buttonpress 1
#define Connected 2

volatile uint32_t pilottimer = 0;
volatile uint16_t timehigh = 0, duration = 0;
volatile uint16_t accurlim = 0;
volatile int dutycycle = 0;

uint16_t cablelim = 0;

#define Singlephase 0
#define Threephase 1

bool Vlimmode = true;
uint16_t modulelimcur = 0, dcaclim = 0;
uint16_t maxaccur = 16000;
uint16_t maxdccur = 72000;
int activemodules = 0, slavechargerenable = 0;

uint16_t dcvolt[3] = {0,0,0};
uint16_t dccur[3] = {0,0,0};
uint16_t totdccur = 0;
uint16_t acvolt[3] = {0,0,0};
uint16_t accur[3] = {0,0,0};
long acpower = 0;
byte inlettarg[3] = {0,0,0};
byte curtemplim[3] = {0,0,0};
byte templeg[2][3] = {{0,0,0},{0,0,0}};
bool ACpres[3] = {0,0,0};
bool ModEn[3] = {0,0,0};
bool ModFlt[3] = {0,0,0};
bool LockOut = false;
byte ModStat[3] = {0,0,0};
int newframe = 0;

ChargerParams parameters;

bool dcdcenable = true;

int ControlID = 0x300;
int StatusID = 0x410;
unsigned long ElconID = 0x18FF50E5;
unsigned long ElconControlID = 0x1806E5F4;

uint16_t LockOutCnt = 0;
int candebug = 0;
int menuload = 0;

// CAN_FRAME from can_common
CAN_FRAME incoming;
CAN_FRAME outframe;

unsigned long lastChargerMsg = 0;
unsigned long lastWDTreset = 0;

// Forward declarations
void candecode(CAN_FRAME &frame);
void canextdecode(CAN_FRAME &frame);
void Charger_msgs();
void evseread();
void Pilotread();
void Pilotcalc();
void ACcurrentlimit();
void DCcurrentlimit();
void resetFaults();
void autoShutdown();
void manualMode();
void menu_char(char c);
void watchdogEnable(int seconds);
void watchdogReset();

// --- Web UI content
const char index_html[] PROGMEM = R"rawliteral(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>Tesla Charger Web Serial</title>
  <style>
    body { font-family: monospace; background:#111; color:#eee; margin:0; padding:0; }
    #log { height:70vh; overflow:auto; padding:10px; background:#000; color:#0f0; }
    #cmd { width:80%; padding:8px; font-size:16px; }
    #send { padding:8px 12px; font-size:16px; }
    .top { padding:10px; background:#222; display:flex; gap:10px; align-items:center; }
    .small { font-size:12px; color:#aaa; }
  </style>
</head>
<body>
  <div class="top">
    <div><strong>Tesla Charger Web Serial</strong></div>
    <div class="small">AP: TeslaCharger | Pass: 123456 | IP: 192.168.4.2</div>
    <div style="margin-left:auto;"><button onclick="clearLog()">Clear</button></div>
  </div>
  <div id="log"></div>
  <div style="padding:10px; background:#222;">
    <input id="cmd" placeholder="Type command (single char or full line)"/>
    <button id="send">Send</button>
  </div>
<script>
let logEl = document.getElementById('log');
let cmd = document.getElementById('cmd');
document.getElementById('send').addEventListener('click', sendCmd);
cmd.addEventListener('keydown', function(e){ if(e.key==='Enter') sendCmd(); });

function sendCmd(){
  let v = cmd.value;
  if(!v) return;
  fetch('/cmd', { method:'POST', body: v })
    .then(r=>r.text()).then(t=>{ cmd.value=''; });
}

function clearLog(){
  fetch('/clear', { method:'POST' }).then(()=>{ logEl.innerText=''; });
}

async function poll(){
  try {
    let r = await fetch('/log');
    if(r.ok){
      let text = await r.text();
      logEl.innerText = text;
      logEl.scrollTop = logEl.scrollHeight;
    }
  } catch(e){}
  setTimeout(poll, 800);
}

poll();
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", index_html); }
void handleLog() { server.send(200, "text/plain", getLogs(200)); }
void handleClear() {
  portENTER_CRITICAL(&logMux);
  logHead = 0; logCount = 0;
  for (int i=0;i<LOG_LINES;i++) logBuffer[i] = "";
  portEXIT_CRITICAL(&logMux);
  server.send(200, "text/plain", "OK");
}
void handleCmd() {
  if (server.method() == HTTP_POST) {
    String body = server.arg(0);
    if (body.length() > 0) {
      for (size_t i=0;i<body.length();i++) menu_char(body.charAt(i));
      menu_char('\n');
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "No data");
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPdisconnect(true);
  WiFi.softAPConfig(apLocalIP, apGateway, apSubnet);
  bool ok = WiFi.softAP(apSSID, apPassword, 1, false, 4);
  if (!ok) logPrint("WiFi.softAP failed");
  else logPrint(String("AP started. IP: ") + WiFi.softAPIP().toString());

  server.on("/", handleRoot);
  server.on("/log", HTTP_GET, handleLog);
  server.on("/clear", HTTP_POST, handleClear);
  server.on("/cmd", HTTP_POST, handleCmd);
  server.begin();
  logPrint("Web server started on port 80");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  delay(10);
  logPrint("Gen2 Tesla Charger ESP32 port starting...");

  EEPROM.begin(512);
  EEPROM.get(0, parameters);
  if (parameters.version != EEPROM_VERSION) {
    parameters.version = EEPROM_VERSION;
    parameters.currReq = 0;
    parameters.enabledChargers = 123;
    parameters.can0Speed = 500000;
    parameters.can1Speed = 500000;
    parameters.mainsRelay = 48;
    parameters.voltSet = 38800;
    parameters.tVolt = 39200;
    parameters.autoEnableCharger = 0;
    parameters.canControl = 0;
    parameters.dcdcsetpoint = 14000;
    parameters.phaseconfig = Threephase;
    parameters.type = 2;
    EEPROM.put(0, parameters);
    EEPROM.commit();
  }

  if (Can0.begin(parameters.can0Speed)) logPrint("CAN0 initialized.");
  else logPrint("CAN0 init error.");

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(CHARGER1_ENABLE, OUTPUT);
  pinMode(CHARGER2_ENABLE, OUTPUT);
  pinMode(CHARGER3_ENABLE, OUTPUT);
  pinMode(CHARGER1_ACTIVATE, OUTPUT);
  pinMode(CHARGER2_ACTIVATE, OUTPUT);
  pinMode(CHARGER3_ACTIVATE, OUTPUT);

  pinMode(DIG_IN_1, INPUT);
  pinMode(DIG_IN_2, INPUT_PULLUP);

  pinMode(DIG_OUT_1, OUTPUT);
  pinMode(DIG_OUT_2, OUTPUT);
  pinMode(EVSE_ACTIVATE, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(EVSE_PILOT), Pilotread, CHANGE);

  watchdogEnable(WDT_TIMEOUT_SECONDS);

  dcaclim = maxaccur;
  bChargerEnabled = false;

  lastChargerMsg = millis();
  lastWDTreset = millis();

  setupWiFi();

  logPrint("Setup complete.");
}

// -------------------- Main loop --------------------
void loop() {
  server.handleClient();

  if (Can0.available()) {
    Can0.read(incoming);
    candecode(incoming);
  }

  while (Serial.available()) {
    char c = (char)Serial.read();
    menu_char(c);
  }

  if (parameters.canControl > 1) {
    if (state != 0) {
      if (millis() - tcan > 2000) {
        state = 0;
        logPrint("CAN time-out");
      }
    }
  }

  if (digitalRead(DIG_IN_1) == LOW) state = 0;

  switch (state) {
    case 0:
      if (bChargerEnabled) bChargerEnabled = false;
      digitalWrite(DIG_OUT_1, LOW);
      digitalWrite(EVSE_ACTIVATE, LOW);
      digitalWrite(DIG_OUT_2, LOW);
      digitalWrite(CHARGER1_ACTIVATE, LOW);
      digitalWrite(CHARGER2_ACTIVATE, LOW);
      digitalWrite(CHARGER3_ACTIVATE, LOW);
      digitalWrite(CHARGER1_ENABLE, LOW);
      digitalWrite(CHARGER2_ENABLE, LOW);
      digitalWrite(CHARGER3_ENABLE, LOW);
      break;

    case 1:
      if (digitalRead(DIG_IN_1) == HIGH) {
        if (!bChargerEnabled) {
          bChargerEnabled = true;
          switch (parameters.enabledChargers) {
            case 1: digitalWrite(CHARGER1_ACTIVATE, HIGH); activemodules = 1; break;
            case 2: digitalWrite(CHARGER2_ACTIVATE, HIGH); activemodules = 1; break;
            case 3: digitalWrite(CHARGER3_ACTIVATE, HIGH); activemodules = 1; break;
            case 12: digitalWrite(CHARGER1_ACTIVATE, HIGH); digitalWrite(CHARGER2_ACTIVATE, HIGH); activemodules = 2; break;
            case 13: digitalWrite(CHARGER1_ACTIVATE, HIGH); digitalWrite(CHARGER3_ACTIVATE, HIGH); activemodules = 2; break;
            case 123: digitalWrite(CHARGER1_ACTIVATE, HIGH); digitalWrite(CHARGER2_ACTIVATE, HIGH); digitalWrite(CHARGER3_ACTIVATE, HIGH); activemodules = 3; break;
            case 23: digitalWrite(CHARGER2_ACTIVATE, HIGH); digitalWrite(CHARGER3_ACTIVATE, HIGH); activemodules = 2; break;
            default: break;
          }
          delay(100);
          digitalWrite(EVSE_ACTIVATE, HIGH);
          digitalWrite(DIG_OUT_1, HIGH);
          digitalWrite(DIG_OUT_2, HIGH);
        }
      } else {
        bChargerEnabled = false;
        state = 0;
      }
      break;

    case 2:
      digitalWrite(DIG_OUT_1, HIGH);
      switch (parameters.enabledChargers) {
        case 1: digitalWrite(CHARGER1_ENABLE, HIGH); break;
        case 2: digitalWrite(CHARGER2_ENABLE, HIGH); break;
        case 3: digitalWrite(CHARGER3_ENABLE, HIGH); break;
        case 12: digitalWrite(CHARGER1_ENABLE, HIGH); digitalWrite(CHARGER2_ENABLE, HIGH); break;
        case 13: digitalWrite(CHARGER1_ENABLE, HIGH); digitalWrite(CHARGER3_ENABLE, HIGH); break;
        case 123: digitalWrite(CHARGER1_ENABLE, HIGH); digitalWrite(CHARGER2_ENABLE, HIGH); digitalWrite(CHARGER3_ENABLE, HIGH); break;
        case 23: digitalWrite(CHARGER2_ENABLE, HIGH); digitalWrite(CHARGER3_ENABLE, HIGH); break;
        default: break;
      }
      if (tboot < (millis() - 10)) state = 1;
      break;

    default:
      break;
  }

  if (tlast < (millis() - 4000)) {
    tlast = millis();
    evseread();
    autoShutdown();
    watchdogReset();
    manualMode();

    if (debug != 0) {
      String s = "";
      s += "\n";
      s += String(millis());
      s += " State: ";
      s += String(state);
      s += " Phases : ";
      s += String(parameters.phaseconfig);
      s += " Modules Avtive : ";
      s += String(activemodules);
      s += (bChargerEnabled ? " ON  " : " OFF ");
      s += (digitalRead(DIG_IN_1) == HIGH ? " D1 H" : " D1 L");
      logPrint(s);
      if (bChargerEnabled) {
        for (int x = 0; x < 3; x++) {
          String ss = "  Phase " + String(x+1) + " Feedback // AC present: " + String(ACpres[x]) + " AC volt: " + String(acvolt[x]) +
                      " AC cur: " + String((accur[x] * 0.06666), 2) + " DC volt: " + String(dcvolt[x]) +
                      " DC cur: " + String(dccur[x] * 0.000839233, 2);
          logPrint(ss);
        }
      } else {
        logPrint("Modules Turned OFF");
      }
    }

    if (debug == 1 && evsedebug != 0) {
      String s = "  Proximity Status : ";
      switch (Proximity) {
        case Unconnected: s += "Unconnected"; break;
        case Buttonpress: s += "Button Pressed"; break;
        case Connected: s += "Connected"; break;
      }
      s += " AC limit : " + String(accurlim) + " /Cable Limit: " + String(cablelim);
      s += " /Module Cur Request: " + String(modulelimcur / 1.5, 0);
      s += " /DC total Cur:" + String(totdccur * 0.005, 2);
      s += " /DC Setpoint:" + String(parameters.voltSet * 0.01, 0);
      s += " /DC tVolt:" + String(parameters.tVolt * 0.01, 0);
      s += " /DC driven AC Cur Lim: " + String(dcaclim);
      logPrint(s);
    }
  }

  if (state != 0 && millis() - lastChargerMsg >= CHARGER_MSG_INTERVAL_MS) {
    lastChargerMsg = millis();
    if (state == 2 || (state == 1 && bChargerEnabled)) Charger_msgs();
  }

  DCcurrentlimit();
  ACcurrentlimit();
  resetFaults();

  if (parameters.autoEnableCharger == 1) {
    if ((Proximity == Connected) && (LockOut == false)) {
      digitalWrite(EVSE_ACTIVATE, HIGH);
      if (accurlim > 1400) {
        if (state == 0) {
          if (digitalRead(DIG_IN_1) == HIGH) {
            state = 2;
            tboot = millis();
          }
        }
      }
      digitalWrite(DIG_OUT_2, HIGH);
    } else {
      state = 0;
      digitalWrite(DIG_OUT_2, LOW);
      digitalWrite(EVSE_ACTIVATE, LOW);
    }
  }
}

// -------------------- Functions --------------------

void resetFaults() {
  if ((bChargerEnabled == true) && (ACpres[0] == true) && (ModFlt[0] == true) &&
      ((parameters.enabledChargers == 1) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 123))) {
    state = 0;
    digitalWrite(DIG_OUT_2, LOW);
  }
  if ((bChargerEnabled == true) && (ACpres[1] == true) && (ModFlt[1] == true) &&
      ((parameters.enabledChargers == 2) || (parameters.enabledChargers == 12) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123))) {
    state = 0;
    digitalWrite(DIG_OUT_2, LOW);
  }
  if ((bChargerEnabled == true) && (ACpres[2] == true) && (ModFlt[2] == true) &&
      ((parameters.enabledChargers == 3) || (parameters.enabledChargers == 13) || (parameters.enabledChargers == 23) || (parameters.enabledChargers == 123))) {
    state = 0;
    digitalWrite(DIG_OUT_2, LOW);
  }
}

void autoShutdown() {
  if ((bChargerEnabled == true) && (LockOut == false)) {
    if (dcvolt[0] > (parameters.tVolt * 0.01)) LockOutCnt++;
    else LockOutCnt = 0;
  }
  if (Proximity == Unconnected && (parameters.autoEnableCharger == 1)) LockOut = false;
  if (LockOutCnt > 10) { state = 0; LockOut = true; LockOutCnt = 0; }
}

void manualMode() {
  if (parameters.autoEnableCharger == 0) {
    if (state == 0) {
      if (digitalRead(DIG_IN_1) == HIGH && (LockOut == false)) { state = 2; tboot = millis(); }
    }
    if (digitalRead(DIG_IN_1) == LOW) { state = 0; LockOut = false; }
  }
}

void candecode(CAN_FRAME &frame) {
  int x = 0;
  switch (frame.id) {
    case 0x217: ModStat[0] = frame.data.uint8[0]; break;
    case 0x219: ModStat[1] = frame.data.uint8[0]; break;
    case 0x21B: ModStat[2] = frame.data.uint8[0]; break;
    case 0x24B: curtemplim[2] = frame.data.uint8[0] * 0.234375; newframe |= 1; break;
    case 0x23B: templeg[0][2] = frame.data.uint8[0] - 40; templeg[1][2] = frame.data.uint8[1] - 40; inlettarg[2] = frame.data.uint8[5] - 40; newframe |= 1; break;
    case 0x239: templeg[0][1] = frame.data.uint8[0] - 40; templeg[1][1] = frame.data.uint8[1] - 40; inlettarg[1] = frame.data.uint8[5] - 40; newframe |= 1; break;
    case 0x249: curtemplim[1] = frame.data.uint8[0] * 0.234375; newframe |= 1; break;
    case 0x237: templeg[0][0] = frame.data.uint8[0] - 40; templeg[1][0] = frame.data.uint8[1] - 40; inlettarg[0] = frame.data.uint8[5] - 40; newframe |= 1; break;
    case 0x247: curtemplim[0] = frame.data.uint8[0] * 0.234375; newframe |= 1; break;
    case 0x207:
      acvolt[0] = frame.data.uint8[1];
      accur[0] = (uint16_t)(((frame.data.uint8[6] & 0x03) * 256) + frame.data.uint8[5]) >> 1;
      x = frame.data.uint8[1];
      ACpres[0] = (x > 0x46);
      ModEn[0] = (frame.data.uint8[2] & 0x02) != 0;
      ModFlt[0] = (frame.data.uint8[2] & 0x04) != 0;
      newframe |= 1;
      break;
    case 0x209:
      acvolt[1] = frame.data.uint8[1];
      accur[1] = (uint16_t)(((frame.data.uint8[6] & 0x03) * 256) + frame.data.uint8[5]) >> 1;
      x = frame.data.uint8[1];
      ACpres[1] = (x > 0x46);
      ModEn[1] = (frame.data.uint8[2] & 0x02) != 0;
      ModFlt[1] = (frame.data.uint8[2] & 0x04) != 0;
      newframe |= 1;
      break;
    case 0x20B:
      acvolt[2] = frame.data.uint8[1];
      accur[2] = (uint16_t)(((frame.data.uint8[6] & 0x03) * 256) + frame.data.uint8[5]) >> 1;
      x = frame.data.uint8[1];
      ACpres[2] = (x > 0x46);
      ModEn[2] = (frame.data.uint8[2] & 0x02) != 0;
      ModFlt[2] = (frame.data.uint8[2] & 0x04) != 0;
      newframe |= 1;
      break;
    case 0x227:
      dccur[0] = ((frame.data.uint8[5] << 8) + frame.data.uint8[4]);
      dcvolt[0] = ((frame.data.uint8[3] << 8) + frame.data.uint8[2]) * 0.0105286;
      newframe |= 2;
      break;
    case 0x229:
      dccur[1] = ((frame.data.uint8[5] << 8) + frame.data.uint8[4]);
      dcvolt[1] = ((frame.data.uint8[3] << 8) + frame.data.uint8[2]) * 0.0105286;
      newframe |= 2;
      break;
    case 0x22B:
      dccur[2] = ((frame.data.uint8[5] << 8) + frame.data.uint8[4]);
      dcvolt[2] = ((frame.data.uint8[3] << 8) + frame.data.uint8[2]) * 0.01052856;
      newframe |= 2;
      break;
    default:
      break;
  }
}

void Charger_msgs() {
  CAN_FRAME out;

  out.extended = 0;
  out.id = 0x45C;
  out.length = 8;
  out.rtr = 0;
  out.data.uint8[0] = lowByte(parameters.voltSet);
  out.data.uint8[1] = highByte(parameters.voltSet);
  out.data.uint8[2] = 0x14;
  out.data.uint8[3] = bChargerEnabled ? 0x2e : 0x0e;
  out.data.uint8[4] = 0x00;
  out.data.uint8[5] = 0x00;
  out.data.uint8[6] = 0x90;
  out.data.uint8[7] = 0x8c;
  Can0.sendFrame(out);

  out.id = 0x42C;
  out.length = 8;
  out.data.uint8[0] = 0x42;
  out.data.uint8[2] = lowByte(modulelimcur);
  out.data.uint8[3] = highByte(modulelimcur);
  if (bChargerEnabled) {
    out.data.uint8[1] = 0xBB;
    out.data.uint8[4] = 0xFE;
  } else {
    out.data.uint8[1] = lowByte(uint16_t(ACvoltIN / 1.2));
    out.data.uint8[4] = 0x64;
  }
  out.data.uint8[5] = 0x00;
  out.data.uint8[6] = 0x00;
  out.data.uint8[7] = 0x00;
  Can0.sendFrame(out);

  out.id = 0x43C; Can0.sendFrame(out);
  out.id = 0x44C; Can0.sendFrame(out);

  out.id = 0x368;
  out.length = 8;
  out.data.uint8[0] = 0x03;
  out.data.uint8[1] = 0x49;
  out.data.uint8[2] = 0x29;
  out.data.uint8[3] = 0x11;
  out.data.uint8[4] = 0x00;
  out.data.uint8[5] = 0x0c;
  out.data.uint8[6] = 0x40;
  out.data.uint8[7] = 0xff;
  Can0.sendFrame(out);

  uint16_t y = 0, z = 0;
  out.id = StatusID;
  out.length = 8;
  for (int x = 0; x < 3; x++) y += dcvolt[x];
  out.data.uint8[0] = y / 3;
  if (parameters.phaseconfig == Singlephase) {
    for (int x = 0; x < 3; x++) z += (accur[x] * 66.66);
  } else {
    z = accur[2] * 66.66;
  }
  out.data.uint8[1] = lowByte(z);
  out.data.uint8[2] = highByte(z);
  out.data.uint8[3] = lowByte(uint16_t(totdccur));
  out.data.uint8[4] = highByte(uint16_t(totdccur));
  out.data.uint8[5] = lowByte(uint16_t(modulelimcur * 0.66666));
  out.data.uint8[6] = highByte(uint16_t(modulelimcur * 0.66666));
  out.data.uint8[7] = (Proximity << 6) | (parameters.type << 4);
  Can0.sendFrame(out);

  CAN_FRAME ext;
  ext.extended = 1;
  ext.id = (uint32_t)ElconID;
  ext.length = 8;
  uint16_t y10 = (uint16_t)(y * 10 / 3);
  ext.data.uint8[0] = highByte(y10);
  ext.data.uint8[1] = lowByte(y10);
  ext.data.uint8[2] = highByte(uint16_t(totdccur * 20));
  ext.data.uint8[3] = lowByte(uint16_t(totdccur * 20));
  ext.data.uint8[4] = 0x00;
  ext.data.uint8[5] = 0x00;
  ext.data.uint8[6] = 0x00;
  ext.data.uint8[7] = 0x00;
  Can0.sendFrame(ext);

  if (dcdcenable) {
    CAN_FRAME dcdc;
    dcdc.extended = 0;
    dcdc.id = 0x3D8;
    dcdc.length = 3;
    dcdc.data.uint8[0] = 0xDC;
    dcdc.data.uint8[1] = 0x06;
    dcdc.data.uint8[2] = 0x00;
    Can0.sendFrame(dcdc);
  }

  if (parameters.canControl == 1) {
    CAN_FRAME ctl;
    ctl.extended = 0;
    ctl.id = ControlID;
    ctl.length = 8;
    ctl.data.uint8[0] = 0;
    if (state != 0 && slavechargerenable == 1) ctl.data.uint8[0] = 0x01;
    ctl.data.uint8[1] = highByte(parameters.voltSet);
    ctl.data.uint8[2] = lowByte(parameters.voltSet);
    ctl.data.uint8[3] = highByte(maxdccur);
    ctl.data.uint8[4] = lowByte(maxdccur);
    ctl.data.uint8[5] = highByte(modulelimcur);
    ctl.data.uint8[6] = lowByte(modulelimcur);
    ctl.data.uint8[7] = 0;
    Can0.sendFrame(ctl);
  }
}

void evseread() {
  uint16_t val = analogRead(EVSE_PROX);
  if (parameters.type == 2) {
    if (val > 950) Proximity = Unconnected;
    else {
      Proximity = Connected;
      if (val < 950 && val > 800) cablelim = 13000;
      if (val < 800 && val > 700) cablelim = 20000;
      if (val < 600 && val > 450) cablelim = 32000;
      if (val < 400 && val > 250) cablelim = 63000;
    }
  } else {
    if (val > 800) Proximity = Unconnected;
    else {
      if (val > 550) Proximity = Buttonpress;
      else Proximity = Connected;
    }
  }
}

void Pilotread() { Pilotcalc(); }

void Pilotcalc() {
  unsigned long now = micros();
  if (digitalRead(EVSE_PILOT) == HIGH) {
    duration = now - pilottimer;
    pilottimer = now;
  } else {
    if (duration != 0) accurlim = (now - pilottimer) * 100 / duration * 600;
  }
}

void ACcurrentlimit() {
  if (parameters.autoEnableCharger == 1) {
    if (micros() - pilottimer > 1200) accurlim = 0;
    if (parameters.phaseconfig == Singlephase) modulelimcur = (accurlim / 3) * 1.5;
    else {
      modulelimcur = accurlim * 1.5;
      if (modulelimcur > (cablelim * 1.5)) modulelimcur = cablelim * 1.5;
    }
  } else {
    if (parameters.phaseconfig == Singlephase) modulelimcur = (parameters.currReq / 3);
    else modulelimcur = parameters.currReq;
  }

  if (parameters.canControl == 1 || parameters.canControl == 2) {
    if (accurlim * 1.5 > (16000 * 1.5)) { modulelimcur = modulelimcur * 0.5; slavechargerenable = 1; }
    else slavechargerenable = 0;
  }

  if (parameters.phaseconfig == Threephase) {
    if (modulelimcur > parameters.currReq) modulelimcur = parameters.currReq;
  } else {
    if (activemodules > 0 && modulelimcur > (parameters.currReq / activemodules)) modulelimcur = (parameters.currReq / activemodules);
  }

  if (parameters.phaseconfig != 0 && parameters.phaseconfig != 1) modulelimcur = 0;
}

void DCcurrentlimit() {
  totdccur = 0;
  for (int x = 0; x < 3; x++) totdccur += (dccur[x] * 0.1678466);
  dcaclim = 0;
  int x = 0;
  if (totdccur > 0.2) dcaclim = (((float)dcvolt[x] / (float)acvolt[x]) * (maxdccur * 1.2));
  else dcaclim = 5000;
}

void canextdecode(CAN_FRAME &frame) {
  int x = 0;
  if (parameters.canControl == 2) {
    if (ElconControlID == frame.id) {
      parameters.voltSet = ((frame.data.uint8[0] << 8) + frame.data.uint8[1]) * 0.1;
      maxdccur = (frame.data.uint8[2] << 8) + frame.data.uint8[3];
      if ((frame.data.uint8[4] & 0x01) == 1) { if (state == 0) { state = 2; tboot = millis(); } }
      else state = 0;
      if (debug == 1 && candebug == 1) logPrint(String(state) + " " + String(parameters.voltSet) + " " + String(modulelimcur));
      tcan = millis();
    }
  }

  if (parameters.canControl == 3) {
    if (ControlID == frame.id) {
      if ((frame.data.uint8[0] & 0x01) == 1) { if (state == 0) { state = 2; tboot = millis(); } }
      else {
        if (millis() - slavetimeout > 1000) slavetimeout = millis();
        if (millis() - slavetimeout > 500) state = 0;
      }
      parameters.voltSet = (frame.data.uint8[1] << 8) + frame.data.uint8[2];
      maxdccur = (frame.data.uint8[3] << 8) + frame.data.uint8[4];
      modulelimcur = (frame.data.uint8[5] << 8) + frame.data.uint8[6];
      if (debug == 1 && candebug == 1) logPrint(String(state) + " " + String(parameters.voltSet) + " " + String(modulelimcur));
      tcan = millis();
    }
  }
}

// Menu handler for both serial and web
void menu_char(char c) {
  incomingByte = c;
  if (menuload == 1) {
    switch (incomingByte) {
      case 'q': debug = 1; menuload = 0; break;
      case 'a': candebug++; if (candebug > 1) candebug = 0; menuload = 0; break;
      case 'b': evsedebug++; if (evsedebug > 1) evsedebug = 0; menuload = 0; break;
      case '1': parameters.autoEnableCharger++; if (parameters.autoEnableCharger > 1) parameters.autoEnableCharger = 0; menuload = 0; break;
      case 's': digitalWrite(LED_BUILTIN, HIGH); state = 2; tboot = millis(); menuload = 0; break;
      case 'o': if (state > 0) { digitalWrite(LED_BUILTIN, LOW); state = 0; } menuload = 0; break;
      case 'w': EEPROM.put(0, parameters); EEPROM.commit(); debug = 1; menuload = 0; break; // save settings (write)
      case 'd':
        debug = 0; menuload = 1;
        logPrint("\nSettings Menu");
        logPrint(String("1 - Auto Enable : ") + (parameters.autoEnableCharger ? "ON" : "OFF"));
        logPrint(String("2 - Modules Enabled : ") + String(parameters.enabledChargers));
        logPrint(String("3 - Can Mode : ") + String(parameters.canControl));
        logPrint(String("4 - Port Type : ") + String(parameters.type));
        logPrint(String("5 - Phase Wiring : ") + String(parameters.phaseconfig + 1));
        logPrint(String("6 - DC Charge Voltage : ") + String(parameters.voltSet * 0.01, 0) + "V");
        logPrint(String("7 - AC Current Limit : ") + String(parameters.currReq / 1500) + "A");
        logPrint(String("8 - CAN0 Speed : ") + String(parameters.can0Speed * 0.001, 0));
        logPrint(String("9 - CAN1 Speed : ") + String(parameters.can1Speed * 0.001, 0));
        logPrint(String("a - Can Debug : ") + (candebug ? "ON" : "OFF"));
        logPrint(String("b - EVSE Debug : ") + (evsedebug ? "ON" : "OFF"));
        logPrint(String("t - termination voltage : ") + String(parameters.tVolt * 0.01, 0) + "V");
        logPrint("w - Save settings");
        logPrint("q - Quit Menu");
        break;
      default: break;
    }
  } else {
    switch (incomingByte) {
      case 's': digitalWrite(LED_BUILTIN, HIGH); state = 2; tboot = millis(); break;
      case 'o': if (state > 0) { digitalWrite(LED_BUILTIN, LOW); state = 0; } break;
      case 'w': EEPROM.put(0, parameters); EEPROM.commit(); debug = 1; menuload = 0; break; // save settings (write)
      case 'd':
        debug = 0; menuload = 1;
        logPrint("\nSettings Menu");
        logPrint(String("1 - Auto Enable : ") + (parameters.autoEnableCharger ? "ON" : "OFF"));
        logPrint(String("2 - Modules Enabled : ") + String(parameters.enabledChargers));
        logPrint(String("3 - Can Mode : ") + String(parameters.canControl));
        logPrint(String("4 - Port Type : ") + String(parameters.type));
        logPrint(String("5 - Phase Wiring : ") + String(parameters.phaseconfig + 1));
        logPrint(String("6 - DC Charge Voltage : ") + String(parameters.voltSet * 0.01, 0) + "V");
        logPrint(String("7 - AC Current Limit : ") + String(parameters.currReq / 1500) + "A");
        logPrint(String("8 - CAN0 Speed : ") + String(parameters.can0Speed * 0.001, 0));
        logPrint(String("9 - CAN1 Speed : ") + String(parameters.can1Speed * 0.001, 0));
        logPrint(String("a - Can Debug : ") + (candebug ? "ON" : "OFF"));
        logPrint(String("b - EVSE Debug : ") + (evsedebug ? "ON" : "OFF"));
        logPrint(String("t - termination voltage : ") + String(parameters.tVolt * 0.01, 0) + "V");
        logPrint("w - Save settings");
        logPrint("q - Quit Menu");
        break;
      default: break;
    }
  }
}

// Watchdog compatibility wrapper
void watchdogEnable(int seconds) {
  #if defined(ESP_IDF_VERSION_MAJOR) && (ESP_IDF_VERSION_MAJOR >= 5)
    esp_task_wdt_config_t cfg;
    cfg.timeout_ms = (uint32_t)seconds * 1000;
    cfg.idle_core_mask = 0;
    esp_task_wdt_init(&cfg);
    esp_task_wdt_add(NULL);
  #else
    esp_task_wdt_init(seconds, true);
    esp_task_wdt_add(NULL);
  #endif
}

void watchdogReset() {
  if (millis() - lastWDTreset >= 5000) {
    esp_task_wdt_reset();
    lastWDTreset = millis();
  }
}
