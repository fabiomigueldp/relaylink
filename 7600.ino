/*
  SMS-Controlled 5V Relay — Production-Focused (LILYGO TTGO T-SIM7600NA ESP32 Cat-4)
  ---------------------------------------------------------------------------------
  Goal
    - Rock-solid SMS control of a 5 V relay with minimal commands: ON, OFF, STATE.
    - Deterministic SIM7600 power-on via PWRKEY-only (no PMU), robust AT bring-up.

  Hardware
    - Board: LILYGO TTGO T-SIM7600NA (ESP32 + SIM7600NA-H)
    - Relay: IN -> GPIO23 (active-low), VCC -> 5V, GND -> GND
    - Antennas: Attach LTE antenna BEFORE power-up (mandatory)
    - Power: Provide a solid 5V or Li-Ion with burst current (Cat-4 peaks are higher than Cat-M)
    - Note: GPIO33 is RI (Ring Indicator) on T-SIM7600, avoid for relay control

  Build
    - Arduino ESP32 core, Serial 115200 (monitor)
    - Board: "ESP32 Dev Module"
    - Libraries: Preferences (in ESP32 core)

  SMS Commands (Case-Insensitive)
    - "ON"     → Turns relay ON  | Response: Relay state logged to Serial
    - "OFF"    → Turns relay OFF | Response: Relay state logged to Serial
    - "STATE"  → Reports current relay state | Response: Relay state logged to Serial
    
    Command Processing:
      • Commands are case-INSENSITIVE (on, ON, On all work)
      • Whitespace is trimmed automatically
      • Unknown commands are ignored with "UNKNOWN COMMAND" logged to Serial
      • No SMS replies sent to sender (Serial monitor only)
      • All responses include timestamp in format HH:MM:SS.mmm

  Notes
    - Adjust pins below if your board revision differs (check silkscreen/LilyGO docs).
    - We keep LTE-only (AT+CNMP=38) and auto-band. Set APN if you later enable data.
*/

#include <Arduino.h>
#include <Preferences.h>

// ------------------------------- Pins ----------------------------------------
// Typical LilyGO SIM7600/ESP32 mapping; change if your board differs.
static const int PIN_PWRKEY    = 4;   // SIM7600 PWRKEY: drive LOW for ~1s to power on
static const int PIN_MODEM_RX  = 26;  // UART RX from modem  (ESP32 reads on RX)
static const int PIN_MODEM_TX  = 27;  // UART TX to modem    (ESP32 writes on TX)
static const int PIN_MODEM_DTR = -1;  // Optional: keep LOW to avoid sleep; -1 = unused
static const int RELAY_PIN     = 23;  // Use a free GPIO; GPIO33 is RI on T-SIM7600

// -------------------------- Behavior / Timing --------------------------------
static const bool RELAY_ACTIVE_LOW       = true;

static const uint32_t AT_SHORT_TO_MS     = 1500;
static const uint32_t AT_MED_TO_MS       = 4000;
static const uint32_t AT_LONG_TO_MS      = 12000;

// Power sequencing (PWRKEY-only)
static const uint32_t PWRKEY_LOW_MS      = 1100;  // SIM7600 power-on press (low)
static const uint32_t BOOT_URC_WAIT_MS   = 15000; // wait for RDY/URCs
static const uint32_t POST_PWRKEY_GUARD  = 3000;  // guard after releasing PWRKEY

// Registration logic
static const uint32_t REGISTER_CHECK_MS  = 3000;
static const uint32_t REGISTER_WD_MS     = 90000;

// ---------------------------- Logging ----------------------------------------
#ifndef VERBOSE
#define VERBOSE 1
#endif
static inline void ts() {
  unsigned long ms = millis();
  unsigned long s  = ms / 1000; ms %= 1000;
  Serial.printf("%02lu:%02lu:%02lu.%03lu ", s / 3600UL, (s % 3600UL) / 60UL, s % 60UL, ms);
}
#define LOGF(fmt, ...)  do{ if(VERBOSE){ ts(); Serial.printf(fmt, ##__VA_ARGS__);} }while(0)
#define LOGLN(msg)      do{ if(VERBOSE){ ts(); Serial.println(msg);} }while(0)

// ---------------------------- Globals ----------------------------------------
HardwareSerial MODEM(1);
Preferences prefs;

enum class NetState { PWRON, CONFIG, REGISTER, READY };
static NetState netState = NetState::PWRON;
static unsigned long stateSince = 0, lastRegPoll = 0;
static bool cfunSent = false;

// ----------------------- Forward declarations --------------------------------
static void powerOnPulse();
static bool waitBootURCs(uint32_t ms);
static String at(const String &cmd, uint32_t timeout = AT_MED_TO_MS, bool echo = true);
static bool configureModem();
static bool ensureRegistered();
static void pumpURCs();
static void handleCMT(const String &line);
static void handleCMTI(const String &line);
static void readAndDeleteSms(int idx);
static String parseQuotedField(const String &src, int nth);
static void processSms(const String &from, const String &body);
static void setRelay(bool on);

// --------------------------------- Setup -------------------------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  LOGLN("Booting SMS Relay Controller (SIM7600NA)");

  // Relay default = OFF
  pinMode(RELAY_PIN, OUTPUT);
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, HIGH); else digitalWrite(RELAY_PIN, LOW);

  // PWRKEY idle = HIGH (we drive it LOW to press). Keep as input-pullup until needed.
  pinMode(PIN_PWRKEY, OUTPUT);
  digitalWrite(PIN_PWRKEY, HIGH);

  if (PIN_MODEM_DTR >= 0) {
    pinMode(PIN_MODEM_DTR, OUTPUT);
    digitalWrite(PIN_MODEM_DTR, LOW); // keep awake (DTR low)
  }

  // Bring up UART to modem
  MODEM.begin(115200, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);

  // PWRKEY-only power-on
  netState = NetState::PWRON;
  stateSince = millis();
}

// ---------------------------------- Loop -------------------------------------
void loop() {
  pumpURCs();

  const unsigned long now = millis();
  switch (netState) {
    case NetState::PWRON: {
      LOGLN("Asserting PWRKEY to start modem...");
      powerOnPulse();
      waitBootURCs(BOOT_URC_WAIT_MS);
      // Try a few AT pings in case URCs are filtered
      bool ok = false;
      for (int i = 0; i < 30 && !ok; ++i) {
        String r = at("AT", 500, false);
        ok = r.indexOf("OK") >= 0;
        delay(200);
      }
      if (!ok) LOGLN("Warning: No immediate AT OK; proceeding to config anyway.");
      netState = NetState::CONFIG;
      stateSince = now;
    } break;

    case NetState::CONFIG: {
      LOGLN("Configuring modem...");
      if (configureModem()) {
        LOGLN("Modem configured.");
        netState = NetState::REGISTER;
        stateSince = now;
        cfunSent = false;
        lastRegPoll = 0;
      } else {
        LOGLN("Config failed; retrying power-on sequence...");
        netState = NetState::PWRON;
        stateSince = now;
      }
    } break;

    case NetState::REGISTER: {
      if (!cfunSent) { at("AT+CFUN=1", AT_SHORT_TO_MS); cfunSent = true; }
      if (lastRegPoll == 0 || now - lastRegPoll >= REGISTER_CHECK_MS) {
        lastRegPoll = now;
        if (ensureRegistered()) {
          LOGLN("Network registered. Ready for SMS.");
          netState = NetState::READY;
          stateSince = now;
        }
      }
      if (now - stateSince >= REGISTER_WD_MS) {
        LOGLN("Register watchdog; re-asserting PWRKEY...");
        netState = NetState::PWRON;
        stateSince = now;
        cfunSent = false;
      }
    } break;

    case NetState::READY: {
      // Periodic health check
      if (lastRegPoll == 0 || now - lastRegPoll >= 30000UL) {
        lastRegPoll = now;
        String r = at("AT+CEREG?", AT_SHORT_TO_MS, false);
        int stat = -1;
        int pos = r.indexOf(',');
        if (pos >= 0 && pos + 1 < (int)r.length()) {
          char c = r.charAt(pos + 1);
          if (c >= '0' && c <= '5') stat = c - '0';
        }
        if (!(stat == 1 || stat == 5)) {
          LOGLN("Registration lost; re-entering REGISTER state.");
          netState = NetState::REGISTER;
          stateSince = now;
          cfunSent = false;
        }
      }
    } break;
  }

  delay(5);
}

// ============================== Implementation ===============================

static void powerOnPulse() {
  // SIM7600: PWRKEY pulled up internally; drive LOW (press), then release.
  digitalWrite(PIN_PWRKEY, HIGH);
  delay(50);
  digitalWrite(PIN_PWRKEY, LOW);
  delay(PWRKEY_LOW_MS);
  digitalWrite(PIN_PWRKEY, HIGH);
  delay(POST_PWRKEY_GUARD);
}

static bool waitBootURCs(uint32_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (MODEM.available()) {
      String l = MODEM.readStringUntil('\n'); l.trim();
      if (l.length()) LOGF("<< %s\n", l.c_str());
      if (l.indexOf("NORMAL POWER DOWN") >= 0) return false;
      if (l.indexOf("SMS READY") >= 0 || l.indexOf("RDY") >= 0 ||
          l.indexOf("+CPIN: READY") >= 0)
        return true;
    }
    delay(10);
  }
  return false;
}

static String at(const String &cmd, uint32_t timeout, bool echo) {
  if (echo) LOGF(">> %s\n", cmd.c_str());
  MODEM.println(cmd);
  String resp;
  unsigned long t0 = millis();
  while (millis() - t0 < timeout) {
    while (MODEM.available()) {
      String line = MODEM.readStringUntil('\n');
      line.trim();
      if (echo && line.length()) LOGF("<< %s\n", line.c_str());

      if (line.startsWith("+CMT:"))  handleCMT(line);
      else if (line.startsWith("+CMTI:")) handleCMTI(line);

      resp += line; resp += '\n';
      if (line == "OK" || line == "ERROR" ||
          line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
        return resp;
      }
    }
    delay(2);
  }
  return resp;
}

static bool configureModem() {
  at("ATE0", AT_SHORT_TO_MS);
  at("AT+CMEE=2", AT_SHORT_TO_MS);

  // Keep awake
  at("AT+CSCLK=0", AT_SHORT_TO_MS);
  if (PIN_MODEM_DTR >= 0) digitalWrite(PIN_MODEM_DTR, LOW);

  // SMS in text mode, direct URCs
  at("AT+CMGF=1", AT_SHORT_TO_MS);
  at("AT+CSCS=\"GSM\"", AT_SHORT_TO_MS);
  at("AT+CNMI=2,2,0,0,0", AT_SHORT_TO_MS); // direct +CMT
  at("AT+CEREG=2", AT_SHORT_TO_MS);

  // Radio: LTE-only
  at("AT+CNMP=38", AT_SHORT_TO_MS);  // LTE only (SIM7600)
  // (Optional) Band mask via AT+CNBP if you want to restrict to NA bands; default auto-band is robust.

  // Sanity
  return at("AT", AT_SHORT_TO_MS, false).indexOf("OK") >= 0;
}

static bool ensureRegistered() {
  String resp = at("AT+CEREG?", AT_SHORT_TO_MS, false);
  int stat = -1;
  int pos = resp.indexOf(',');
  if (pos >= 0 && pos + 1 < (int)resp.length()) {
    char c = resp.charAt(pos + 1);
    if (c >= '0' && c <= '5') stat = c - '0';
  }
  return (stat == 1 || stat == 5);
}

static void pumpURCs() {
  while (MODEM.available()) {
    String line = MODEM.readStringUntil('\n'); line.trim();
    if (!line.length()) continue;
    LOGF("<< %s\n", line.c_str());

    if (line.startsWith("+CMT:"))       handleCMT(line);
    else if (line.startsWith("+CMTI:")) handleCMTI(line);
    else if (line.indexOf("NORMAL POWER DOWN") >= 0) {
      LOGLN("URC: modem powered down; re-asserting PWRKEY...");
      netState = NetState::PWRON;
      stateSince = millis();
      cfunSent = false;
    }
  }
}

static void handleCMT(const String &line) {
  String sender = parseQuotedField(line, 1);
  unsigned long t0 = millis();
  while (!MODEM.available() && millis() - t0 < 1500) delay(10);
  String body = MODEM.readStringUntil('\n'); body.trim();

  ts(); Serial.printf("SMS from %s\n", sender.c_str());
  ts(); Serial.printf("Body: %s\n", body.c_str());

  processSms(sender, body);
}

static void handleCMTI(const String &line) {
  int comma = line.lastIndexOf(',');
  if (comma < 0) return;
  int idx = line.substring(comma + 1).toInt();
  ts(); Serial.printf("New SMS stored at index %d. Reading...\n", idx);
  readAndDeleteSms(idx);
}

static void readAndDeleteSms(int idx) {
  String resp = at(String("AT+CMGR=") + idx, AT_MED_TO_MS);
  int nl1 = resp.indexOf('\n');
  int nl2 = (nl1 >= 0) ? resp.indexOf('\n', nl1 + 1) : -1;
  String hdr  = (nl1 >= 0) ? resp.substring(0, nl1) : resp;
  String body = (nl2 > nl1) ? resp.substring(nl1 + 1, nl2) : "";
  body.trim();

  // <oa> is the 2nd quoted field per 3GPP TS 27.005
  String sender = parseQuotedField(hdr, 2);
  ts(); Serial.printf("SMS from %s\n", sender.c_str());
  ts(); Serial.printf("Body: %s\n", body.c_str());

  processSms(sender, body);
  at(String("AT+CMGD=") + idx, AT_SHORT_TO_MS);
}

static String parseQuotedField(const String &src, int nth) {
  int count = 0, i = 0;
  while (i < (int)src.length()) {
    int q1 = src.indexOf('"', i); if (q1 < 0) break;
    int q2 = src.indexOf('"', q1 + 1); if (q2 < 0) break;
    if (++count == nth) return src.substring(q1 + 1, q2);
    i = q2 + 1;
  }
  return "";
}

static void processSms(const String &from, const String &body) {
  String cmd = body; cmd.trim(); cmd.toUpperCase();

  if (cmd == "ON") {
    setRelay(true);
    ts(); Serial.println("EXECUTED: ON");
    bool on = RELAY_ACTIVE_LOW ? (digitalRead(RELAY_PIN) == LOW)
                               : (digitalRead(RELAY_PIN) == HIGH);
    ts(); Serial.printf("RELAY STATE: %s\n", on ? "ON" : "OFF");
  } else if (cmd == "OFF") {
    setRelay(false);
    ts(); Serial.println("EXECUTED: OFF");
    bool on = RELAY_ACTIVE_LOW ? (digitalRead(RELAY_PIN) == LOW)
                               : (digitalRead(RELAY_PIN) == HIGH);
    ts(); Serial.printf("RELAY STATE: %s\n", on ? "ON" : "OFF");
  } else if (cmd == "STATE") {
    bool on = RELAY_ACTIVE_LOW ? (digitalRead(RELAY_PIN) == LOW)
                               : (digitalRead(RELAY_PIN) == HIGH);
    ts(); Serial.printf("STATE: RELAY=%s\n", on ? "ON" : "OFF");
  } else {
    ts(); Serial.println("UNKNOWN COMMAND");
  }
}

static void setRelay(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}
