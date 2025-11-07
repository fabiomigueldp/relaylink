/*
  SMS-Controlled 5V Relay — Production-Focused (LilyGo T-SIM7080G-S3)
  -------------------------------------------------------------------
  Goal
    - Rock-solid SMS control of a 5 V relay with minimal commands: ON, OFF, STATE.
    - Fixes “every-other-boot” by hard power-cycling the modem rail and only then
      asserting PWRKEY, with proper stabilization delays and retry logic.

  Reliability design
    - Known OFF->ON state every MCU boot: DC3 (modem rail) OFF->ON.
    - Stabilization delay after rail ON (>= 1200 ms) before PWRKEY.
    - Power-on sequence with two attempts: normal pulse then a long-press pulse.
    - Keep modem awake (DTR set, PSM/eDRX off); non-blocking state machine; watchdog
      re-cycles the rail if not registered within a window.
    - Handle +CMT (direct) and +CMTI (stored) so no SMS is missed.
    - Deterministic relay control (active-low).

  Commands (case-insensitive)
    ON      - energize relay (active-low)
    OFF     - de-energize relay
    STATE   - print current relay state

  Hardware
    - Board: LilyGo T-SIM7080G-S3 (ESP32-S3 + SIM7080G + AXP2101)
    - Relay: IN -> GPIO46 (active-low), VCC -> 5V, GND -> GND
    - Antennas: Attach LTE antenna before power-up (mandatory)

  Build
    - Arduino ESP32 core (ESP32S3 Dev Module), Serial 115200
    - Libraries: XPowersLib (AXP2101), Preferences (in ESP32 core)

  Power notes
    - Provide a solid 5 V source and/or a battery with enough burst current. If
      the rail droops, the module won’t boot (no URCs / no OK to AT).
*/

#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <Preferences.h>

// ------------------------------- Pins ----------------------------------------
static const int PIN_PWRKEY    = 41; // SIM7080 power key driver (board: active-HIGH pulse)
static const int PIN_MODEM_RX  = 4;  // UART RX from modem
static const int PIN_MODEM_TX  = 5;  // UART TX to modem
static const int PIN_MODEM_DTR = 42; // Keep modem awake (board wiring)
static const int I2C_SDA       = 15; // AXP2101 I2C SDA
static const int I2C_SCL       = 7;  // AXP2101 I2C SCL
static const int RELAY_PIN     = 46; // Relay driver pin (active-LOW)

#ifndef AXP2101_SLAVE_ADDRESS
#define AXP2101_SLAVE_ADDRESS 0x34
#endif

// -------------------------- Behavior / Timing --------------------------------
static const bool RELAY_ACTIVE_LOW       = true;

static const uint32_t AT_SHORT_TO_MS     = 1500;
static const uint32_t AT_MED_TO_MS       = 4000;
static const uint32_t AT_LONG_TO_MS      = 12000;

// Power sequencing
static const uint32_t RAIL_OFF_MS        = 600;   // ensure full discharge
static const uint32_t RAIL_STABLE_MS     = 1200;  // time to let DC3 settle before PWRKEY
static const uint32_t PWRKEY_HIGH_MS     = 1500;  // normal ON pulse
static const uint32_t PWRKEY_LONG_MS     = 3500;  // long-press fallback
static const uint32_t BOOT_URC_WAIT_MS   = 12000;

// Registration logic
static const int      CNMP_MODE_LTE_ONLY = 38;    // LTE only
static const int      CMNB_MODE_CAT_M1   = 1;     // Cat-M1
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
XPowersAXP2101 PMU;
Preferences prefs;

enum class NetState { POWER, PWRON, CONFIG, REGISTER, READY };
static NetState netState = NetState::POWER;
static unsigned long stateSince = 0, lastRegPoll = 0;
static bool cfunSent = false;

// ----------------------- Forward declarations --------------------------------
static bool initPMU_andColdCycle();
static void railColdCycle();
static bool powerOnModem();
static bool powerOnPulse(uint32_t pulseMs);
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
  LOGLN("Booting SMS Relay Controller (Production Mode)");

  // Relay default = OFF
  pinMode(RELAY_PIN, OUTPUT);
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, HIGH); else digitalWrite(RELAY_PIN, LOW);

  pinMode(PIN_MODEM_DTR, OUTPUT);
  digitalWrite(PIN_MODEM_DTR, HIGH); // board wiring keeps modem awake with HIGH

  pinMode(PIN_PWRKEY, OUTPUT);
  digitalWrite(PIN_PWRKEY, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!initPMU_andColdCycle()) {
    LOGLN("WARNING: PMU init/cold-cycle failed; proceeding but modem may not power.");
  }

  MODEM.begin(115200, SERIAL_8N1, PIN_MODEM_RX, PIN_MODEM_TX);
  prefs.begin("cfg", false);

  netState = NetState::PWRON;
  stateSince = millis();
}

// ---------------------------------- Loop -------------------------------------
void loop() {
  pumpURCs();

  const unsigned long now = millis();
  switch (netState) {
    case NetState::POWER:
      // Unused; we jump straight to PWRON in setup.
      break;

    case NetState::PWRON: {
      LOGLN("Starting modem power-on sequence...");
      if (!powerOnModem()) {
        LOGLN("Power-on failed; rail cold-cycle and retry...");
        railColdCycle();
        if (!powerOnModem()) {
          LOGLN("ERROR: Modem did not come up after retries.");
        }
      }
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
        LOGLN("Config failed; rail cold-cycle and retry...");
        railColdCycle();
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
        LOGLN("Register watchdog fired; cycling rail...");
        railColdCycle();
        netState = NetState::PWRON;
        stateSince = now;
        cfunSent = false;
      }
    } break;

    case NetState::READY: {
      // Periodic health check (optional)
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

static bool initPMU_andColdCycle() {
  if (!PMU.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    LOGLN("PMU init FAILED");
    return false;
  }
  LOGLN("Power-cycling modem rail (AXP2101 DC3) for a known OFF->ON state...");
  PMU.disableDC3();
  delay(RAIL_OFF_MS);
  PMU.setDC3Voltage(3800);   // 3.8 V for SIM7080G
  PMU.enableDC3();

  PMU.setBLDO1Voltage(1800); // 1.8 V level translator (board-specific)
  PMU.enableBLDO1();

  PMU.disableTSPinMeasure();
  PMU.enableVbusVoltageMeasure();
  PMU.enableBattVoltageMeasure();

  delay(RAIL_STABLE_MS);     // <-- IMPORTANT: allow the rail to stabilize
  LOGLN("Modem rail ON and stable; proceeding to PWRKEY sequence.");
  return true;
}

static void railColdCycle() {
  PMU.disableDC3();
  delay(RAIL_OFF_MS);
  PMU.setDC3Voltage(3800);
  PMU.enableDC3();
  delay(RAIL_STABLE_MS);
}

static bool powerOnPulse(uint32_t pulseMs) {
  // Board uses an inverter: drive HIGH to assert PWRKEY to the module.
  digitalWrite(PIN_PWRKEY, LOW);
  delay(80);
  digitalWrite(PIN_PWRKEY, HIGH);
  delay(pulseMs);
  digitalWrite(PIN_PWRKEY, LOW);
  return true;
}

static bool waitBootURCs(uint32_t ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    while (MODEM.available()) {
      String l = MODEM.readStringUntil('\n'); l.trim();
      if (l.length()) LOGF("<< %s\n", l.c_str());
      if (l.indexOf("NORMAL POWER DOWN") >= 0) return false;
      if (l.indexOf("+CPIN: READY") >= 0 || l.indexOf("SMS READY") >= 0 || l.indexOf("RDY") >= 0)
        return true;
    }
    delay(10);
  }
  return false;
}

static bool powerOnModem() {
  // In case the rail is good and the module already came up (rare but safe):
  for (int i = 0; i < 5; ++i) {
    if (at("AT", 400, false).indexOf("OK") >= 0) return true;
    delay(120);
  }

  // Normal ON pulse after stabilized rail
  powerOnPulse(PWRKEY_HIGH_MS);
  if (!waitBootURCs(BOOT_URC_WAIT_MS)) {
    LOGF("Boot URCs not seen; trying AT pings...\n");
  }

  for (int i = 0; i < 20; ++i) {
    if (at("AT", 500, false).indexOf("OK") >= 0) return true;
    delay(200);
  }

  // Fallback: long-press pulse (some boards need it after cold rail)
  LOGLN("No response; attempting long-press PWRKEY...");
  powerOnPulse(PWRKEY_LONG_MS);
  if (!waitBootURCs(BOOT_URC_WAIT_MS)) {
    LOGF("Still no boot URCs; will try AT pings anyway...\n");
  }

  for (int i = 0; i < 30; ++i) {
    if (at("AT", 600, false).indexOf("OK") >= 0) return true;
    delay(220);
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
  digitalWrite(PIN_MODEM_DTR, HIGH);
  at("AT+CPSMS=0", AT_SHORT_TO_MS);
  at("AT+CEDRXS=0", AT_SHORT_TO_MS);

  // SMS in text mode, direct URCs
  at("AT+CMGF=1", AT_SHORT_TO_MS);
  at("AT+CSCS=\"GSM\"", AT_SHORT_TO_MS);
  at("AT+CNMI=2,2,0,0,0", AT_SHORT_TO_MS);
  at("AT+CEREG=2", AT_SHORT_TO_MS);

  // Radio
  at(String("AT+CNMP=") + CNMP_MODE_LTE_ONLY, AT_SHORT_TO_MS);
  at(String("AT+CMNB=") + CMNB_MODE_CAT_M1,   AT_SHORT_TO_MS);

  // Broad Cat-M bandmask (safe default). Adjust if your carrier requires a
  // smaller mask for faster search.
  at("AT+CBANDCFG=\"CAT-M\",1,2,3,4,5,8,12,13,14,18,19,20,25,26,27,28,66,85", AT_LONG_TO_MS);

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

    if (line.startsWith("+CMT:"))      handleCMT(line);
    else if (line.startsWith("+CMTI:")) handleCMTI(line);
    else if (line.indexOf("NORMAL POWER DOWN") >= 0) {
      LOGLN("URC: modem powered down; recycling rail...");
      railColdCycle();
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

  String sender = parseQuotedField(hdr, 3);
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