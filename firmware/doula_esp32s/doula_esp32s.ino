/*
 * DOULA Knee Rehabilitation System — ESP32-S (U1 Master / Laptop Bench Bridge)
 * Target Board : ESP32-S NodeMCU V1.1 (DOIT / NodeMCU-32S / ESP32 Dev Module)
 * Comms        : Wi-Fi STA + mDNS (doula-esp32.local) + WebSocket Server (port 81)
 * Website Path : Website (/ws/device bridge) <-> ws://<ESP32_IP>:81/
 *
 * HARDWARE SAFETY NOTES (from docs/WIRING_AND_COMMISSIONING_GUIDE.md):
 *  - Disconnect external +3V3_LDO rail BEFORE connecting USB-C/Micro-USB for flashing.
 *  - NEVER use ADC2 pins (GPIO0, 2, 4, 12..15, 25..27 analogRead) while Wi-Fi is active.
 *  - Physical HW E-Stop button goes on GPIO4 (digitalRead). Set ENABLE_HW_ESTOP_PIN true
 *    only after wiring the normally-open switch to GND.
 *
 * REQUIRED ARDUINO LIBRARY (Library Manager):
 *  - "WebSockets" by Markus Sattler (Links2004) v2.4.0+
 *  (No ArduinoJson required — avoids v6 vs v7 API mismatches.)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>

// ============================================================================
// 1. USER CONFIGURATION — EDIT YOUR WI-FI / HOTSPOT CREDENTIALS BEFORE FLASH
// ============================================================================
const char* WIFI_SSID     = "DPLab";
const char* WIFI_PASSWORD = "DPL@1234#";
const char* MDNS_HOST     = "doula-esp32";   // Reachable at ws://doula-esp32.local:81/
const uint16_t WS_PORT    = 81;

// true  = Laptop USB bench testing (simulates realistic knee/cycling telemetry,
//         responds immediately to + / - torque, Start, Pause, Resume, Stop, E-Stop).
// false = Switch to real ESP-NOW U2/U3 hardware telemetry hooks below.
#define BENCH_SIMULATION_MODE true

// Keep false while testing a bare ESP32-S board with nothing wired to GPIO4.
// Flip to true once the physical active-LOW E-Stop pushbutton is wired to GPIO4 + GND.
#define ENABLE_HW_ESTOP_PIN false

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define HW_ESTOP_PIN 4

// ============================================================================
// 2. REHAB SESSION STATE
// ============================================================================
enum OpMode : uint8_t {
  OP_IDLE         = 0,
  OP_CPM          = 3,  // Flexion/Extension Passive Assist (Support Torque)
  OP_ISOTONIC     = 4,  // Flexion/Extension Physiological Curve Isotonic (Resistance)
  OP_ISOKINETIC   = 5,  // Flexion/Extension Accommodating Isokinetic (Resistance)
  OP_ISOMETRIC    = 6,  // Flexion/Extension Isometric Hold
  OP_CYCLE_TORQUE = 7,  // Cycling Constant Torque / Ergometer Mode
  OP_CYCLE_POWER  = 8   // Cycling Constant Power Mode
};

struct SessionState {
  bool active        = false;
  bool paused        = false;
  bool estopped      = false;
  uint8_t opMode     = OP_CYCLE_TORQUE;
  char modeName[24]  = "cycling";
  char role[16]      = "resistance";

  float targetTorqueNm = 14.0f;   // Updated via start_session & + / - set_torque
  float liveTorqueNm   = 0.0f;    // Measured/simulated applied torque
  int   targetReps     = 15;
  int   completedReps  = 0;
  float romMinDeg      = 0.0f;
  float romMaxDeg      = 110.0f;

  // Live sensor / simulation kinematics
  float kneeAngleDeg   = 0.0f;
  float rpm            = 0.0f;
  float speedKmh       = 0.0f;
  float cycleProgress  = 0.0f;    // 0.0 .. 1.0 within current rep / pedal stroke
  uint32_t activeMs    = 0;
};

SessionState gSession;
WebSocketsServer webSocket(WS_PORT);

uint32_t lastLoopMs      = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastLedToggleMs = 0;
bool ledState            = false;
const uint32_t TELEMETRY_PERIOD_MS = 50; // 20 Hz live stream

// ============================================================================
// 2B. ESP-NOW STEPPER NODE (U3) PROTOCOL & STATE
// ============================================================================
constexpr uint16_t STEPPER_CMD_MAGIC       = 0xD01A;
constexpr uint16_t STEPPER_STATUS_MAGIC    = 0xD01B;
constexpr uint32_t ESPNOW_HEARTBEAT_MS     = 500;
constexpr uint32_t WIFI_RECONNECT_RETRY_MS = 15000;

enum StepperCmdType : uint8_t {
  CMD_SYNC_HEARTBEAT = 0,
  CMD_START_SESSION  = 1,
  CMD_STEP_PLUS      = 2,
  CMD_STEP_MINUS     = 3,
  CMD_END_SESSION    = 4
};

struct __attribute__((packed)) StepperCommandPacket_t {
  uint16_t magic;
  uint8_t  cmd;
  uint8_t  wifiChannel;
  uint8_t  sessionActive;
  uint32_t bootId;
  uint32_t seq;
};

struct __attribute__((packed)) StepperStatusPacket_t {
  uint16_t magic;
  uint8_t  sessionActive;
  int32_t  currentSteps;
  int32_t  targetSteps;
  float    angleDeg;
};

struct QueuedStepperStatus {
  uint8_t               srcMac[6];
  StepperStatusPacket_t status;
};

// Broadcast by default so U3 connects automatically without hardcoding its MAC;
// once U3 replies, U1 locks its MAC for unicast ACK delivery as well.
uint8_t  STEPPER_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t  learnedStepperMac[6]     = {0};
bool     hasLearnedStepper        = false;
bool     espNowReady              = false;
bool     cyclingStepperActive     = false;
uint32_t masterBootId             = 0;
uint32_t espNowSeq                = 0;
uint32_t lastEspNowHeartbeatMs    = 0;
uint32_t lastStepperRxMs          = 0;
uint32_t lastWifiRetryMs          = 0;
uint8_t  lastAnnouncedChannel     = 0;

QueueHandle_t         stepperStatusQueue = nullptr;
StepperStatusPacket_t gStepperStatus     = {STEPPER_STATUS_MAGIC, 0, 0, 0, 0.0f};

// ============================================================================
// 3. LIGHTWEIGHT JSON FIELD PARSERS (NO EXTERNAL DEPENDENCY NEEDED)
// ============================================================================
bool jsonGetString(const String& json, const char* key, String& out) {
  String pattern = String("\"") + key + "\"";
  int keyPos = json.indexOf(pattern);
  if (keyPos < 0) return false;
  int colonPos = json.indexOf(':', keyPos + pattern.length());
  if (colonPos < 0) return false;
  int quoteStart = json.indexOf('"', colonPos + 1);
  if (quoteStart < 0) return false;
  int quoteEnd = json.indexOf('"', quoteStart + 1);
  if (quoteEnd < 0) return false;
  out = json.substring(quoteStart + 1, quoteEnd);
  return true;
}

bool jsonGetFloat(const String& json, const char* key, float& out) {
  String pattern = String("\"") + key + "\"";
  int keyPos = json.indexOf(pattern);
  if (keyPos < 0) return false;
  int colonPos = json.indexOf(':', keyPos + pattern.length());
  if (colonPos < 0) return false;

  int idx = colonPos + 1;
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) idx++;
  if (idx >= (int)json.length()) return false;

  // Ignore string-quoted non-numeric values
  if (json[idx] == '"') return false;

  int endIdx = idx;
  while (endIdx < (int)json.length()) {
    char c = json[endIdx];
    if ((c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.') {
      endIdx++;
    } else {
      break;
    }
  }
  if (endIdx == idx) return false;
  out = json.substring(idx, endIdx).toFloat();
  return true;
}

float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

bool isCyclingMode(uint8_t opMode, const char* modeStr) {
  return opMode == OP_CYCLE_TORQUE || opMode == OP_CYCLE_POWER || strcmp(modeStr, "cycling") == 0;
}

float maxAllowedTorque(uint8_t opMode, const char* role) {
  // Passive CPM (Mode 1) caps support torque safely at 25 N*m; isotonic/cycling cap at 85 N*m.
  if (opMode == OP_CPM || strcmp(role, "support") == 0) return 25.0f;
  return 85.0f;
}

// ============================================================================
// 4. TELEMETRY BROADCAST & ESP-NOW STEPPER BRIDGE
// ============================================================================
uint8_t currentMasterChannel() {
  int32_t ch = WiFi.channel();
  if (ch >= 1 && ch <= 13) return (uint8_t)ch;
  uint8_t primary = 1;
  wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary, &sec) == ESP_OK && primary >= 1 && primary <= 13) {
    return primary;
  }
  return 1;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !info->src_addr || !data || len != sizeof(StepperStatusPacket_t)) return;
  const uint8_t* srcMac = info->src_addr;
#else
void onEspNowRecv(const uint8_t* srcMac, const uint8_t* data, int len) {
  if (!srcMac || !data || len != sizeof(StepperStatusPacket_t)) return;
#endif
  StepperStatusPacket_t st{};
  memcpy(&st, data, sizeof(st));
  if (st.magic != STEPPER_STATUS_MAGIC || stepperStatusQueue == nullptr) return;

  QueuedStepperStatus item{};
  memcpy(item.srcMac, srcMac, 6);
  item.status = st;
  xQueueSend(stepperStatusQueue, &item, 0);
}

void initEspNow() {
  if (espNowReady) return;
  if (stepperStatusQueue == nullptr) {
    stepperStatusQueue = xQueueCreate(8, sizeof(QueuedStepperStatus));
  }
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: esp_now_init() failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t bcastPeer{};
  memcpy(bcastPeer.peer_addr, STEPPER_BROADCAST_MAC, 6);
  bcastPeer.channel = 0; // Follow Master's active STA channel automatically
  bcastPeer.ifidx   = WIFI_IF_STA;
  bcastPeer.encrypt = false;
  esp_now_add_peer(&bcastPeer);

  espNowReady = true;
  Serial.printf("[ESP-NOW] Ready on MAC %s (channel %u)\n",
                WiFi.macAddress().c_str(),
                (unsigned)currentMasterChannel());
}

void processStepperStatusQueue() {
  if (stepperStatusQueue == nullptr) return;
  QueuedStepperStatus item{};
  while (xQueueReceive(stepperStatusQueue, &item, 0) == pdTRUE) {
    lastStepperRxMs = millis();
    if (!hasLearnedStepper || memcmp(learnedStepperMac, item.srcMac, 6) != 0) {
      if (hasLearnedStepper) esp_now_del_peer(learnedStepperMac);
      memcpy(learnedStepperMac, item.srcMac, 6);
      esp_now_peer_info_t peer{};
      memcpy(peer.peer_addr, learnedStepperMac, 6);
      peer.channel = 0;
      peer.ifidx   = WIFI_IF_STA;
      peer.encrypt = false;
      if (esp_now_add_peer(&peer) == ESP_OK) {
        hasLearnedStepper = true;
        Serial.printf(
          "[ESP-NOW] Paired Stepper U3 MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
          learnedStepperMac[0], learnedStepperMac[1], learnedStepperMac[2],
          learnedStepperMac[3], learnedStepperMac[4], learnedStepperMac[5]
        );
      }
    }

    bool changed = (item.status.currentSteps != gStepperStatus.currentSteps) ||
                   (item.status.targetSteps  != gStepperStatus.targetSteps)  ||
                   (item.status.sessionActive != gStepperStatus.sessionActive);
    gStepperStatus = item.status;
    if (changed) {
      Serial.printf(
        "[STEPPER RX] session=%u steps=%ld/%ld angle=%.2f deg\n",
        (unsigned)gStepperStatus.sessionActive,
        (long)gStepperStatus.currentSteps,
        (long)gStepperStatus.targetSteps,
        gStepperStatus.angleDeg
      );
    }
  }
}

void sendStepperCommand(StepperCmdType cmd, uint8_t repeatCount = 1) {
  if (!espNowReady) return;

  StepperCommandPacket_t pkt{};
  pkt.magic         = STEPPER_CMD_MAGIC;
  pkt.cmd           = static_cast<uint8_t>(cmd);
  pkt.wifiChannel   = currentMasterChannel();
  pkt.sessionActive = cyclingStepperActive ? 1 : 0;
  pkt.bootId        = masterBootId;
  pkt.seq           = ++espNowSeq;

  const uint8_t* targetMac = hasLearnedStepper ? learnedStepperMac : STEPPER_BROADCAST_MAC;
  for (uint8_t i = 0; i < repeatCount; i++) {
    esp_now_send(targetMac, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (i + 1 < repeatCount) delay(4);
  }
}

void serviceEspNowHeartbeat() {
  if (!espNowReady) return;
  uint8_t ch = currentMasterChannel();
  if (ch != lastAnnouncedChannel) {
    lastAnnouncedChannel = ch;
    Serial.printf("[ESP-NOW] Active Wi-Fi channel=%u\n", (unsigned)ch);
  }
  uint32_t now = millis();
  if (hasLearnedStepper && (now - lastStepperRxMs > 5000)) {
    esp_now_del_peer(learnedStepperMac);
    hasLearnedStepper = false;
    Serial.println("[ESP-NOW] Stepper U3 reply timeout — reverting to broadcast discovery");
  }
  if (now - lastEspNowHeartbeatMs >= ESPNOW_HEARTBEAT_MS) {
    lastEspNowHeartbeatMs = now;
    sendStepperCommand(CMD_SYNC_HEARTBEAT, 1);
  }
}

void sendTelemetryFrame() {
  if (webSocket.connectedClients() == 0) return;

  char buf[384];
  if (isCyclingMode(gSession.opMode, gSession.modeName)) {
    snprintf(
      buf,
      sizeof(buf),
      "{\"type\":\"telemetry\",\"mode\":\"cycling\",\"op_mode\":%u,"
      "\"rpm\":%.1f,\"speed\":%.1f,\"torque\":%.1f,\"target_torque\":%.1f,"
      "\"stepper_angle_deg\":%.2f,\"stepper_steps\":%ld,"
      "\"reps\":%d,\"estop\":%d,\"paused\":%d,\"uptime_ms\":%lu}",
      (unsigned)gSession.opMode,
      gSession.rpm,
      gSession.speedKmh,
      gSession.liveTorqueNm,
      gSession.targetTorqueNm,
      gStepperStatus.angleDeg,
      (long)gStepperStatus.currentSteps,
      gSession.completedReps,
      gSession.estopped ? 1 : 0,
      gSession.paused ? 1 : 0,
      (unsigned long)millis()
    );
  } else {
    bool isSupport = (gSession.opMode == OP_CPM || strcmp(gSession.role, "support") == 0);
    int remaining = gSession.targetReps - gSession.completedReps;
    if (remaining < 0) remaining = 0;

    snprintf(
      buf,
      sizeof(buf),
      "{\"type\":\"telemetry\",\"mode\":\"%s\",\"op_mode\":%u,"
      "\"support_torque\":%.1f,\"resistance_torque\":%.1f,\"torque\":%.1f,"
      "\"target_torque\":%.1f,\"knee_angle_deg\":%.1f,"
      "\"completed_reps\":%d,\"reps_remaining\":%d,\"estop\":%d,\"paused\":%d,\"uptime_ms\":%lu}",
      gSession.modeName,
      (unsigned)gSession.opMode,
      isSupport ? gSession.liveTorqueNm : 0.0f,
      isSupport ? 0.0f : gSession.liveTorqueNm,
      gSession.liveTorqueNm,
      gSession.targetTorqueNm,
      gSession.kneeAngleDeg,
      gSession.completedReps,
      remaining,
      gSession.estopped ? 1 : 0,
      gSession.paused ? 1 : 0,
      (unsigned long)millis()
    );
  }
  webSocket.broadcastTXT(buf);
}

void forwardCommandToHardware(const char* eventName) {
  Serial.printf(
    "[CMD:%s] active=%d paused=%d estop=%d mode=%s op=%u target=%.1f Nm reps=%d/%d stepper=%.2f deg\n",
    eventName,
    gSession.active ? 1 : 0,
    gSession.paused ? 1 : 0,
    gSession.estopped ? 1 : 0,
    gSession.modeName,
    (unsigned)gSession.opMode,
    gSession.targetTorqueNm,
    gSession.completedReps,
    gSession.targetReps,
    gStepperStatus.angleDeg
  );
}

void startCyclingStepperSession() {
  cyclingStepperActive = true;
  sendStepperCommand(CMD_START_SESSION, 2);
}

void stepCyclingStepper(int dir) {
  if (dir > 0) {
    sendStepperCommand(CMD_STEP_PLUS, 2);
  } else if (dir < 0) {
    sendStepperCommand(CMD_STEP_MINUS, 2);
  }
}

void endCyclingStepperSession() {
  cyclingStepperActive = false;
  sendStepperCommand(CMD_END_SESSION, 2);
}

// ============================================================================
// 5. INCOMING WEBSITE COMMAND HANDLER
// ============================================================================
void handleWebsiteJson(const String& payload) {
  String cmdType;
  if (!jsonGetString(payload, "type", cmdType)) {
    Serial.println("[WS] Ignored non-typed payload");
    return;
  }

  if (cmdType == "start_session") {
    String modeStr;
    if (jsonGetString(payload, "mode", modeStr)) {
      modeStr.toCharArray(gSession.modeName, sizeof(gSession.modeName));
    }
    String roleStr;
    if (jsonGetString(payload, "role", roleStr)) {
      roleStr.toCharArray(gSession.role, sizeof(gSession.role));
    }

    float opVal = 0.0f;
    if (jsonGetFloat(payload, "op_mode", opVal)) {
      gSession.opMode = (uint8_t)lroundf(opVal);
    } else if (strcmp(gSession.modeName, "passive") == 0) {
      gSession.opMode = OP_CPM;
    } else if (strcmp(gSession.modeName, "isotonic") == 0) {
      gSession.opMode = OP_ISOTONIC;
    } else if (strcmp(gSession.modeName, "isokinetic") == 0) {
      gSession.opMode = OP_ISOKINETIC;
    } else {
      gSession.opMode = OP_CYCLE_TORQUE;
    }

    float tqVal = 0.0f;
    if (jsonGetFloat(payload, "torque_nm", tqVal) || jsonGetFloat(payload, "target_torque_ext", tqVal)) {
      gSession.targetTorqueNm = clampf(tqVal, 0.0f, maxAllowedTorque(gSession.opMode, gSession.role));
    }

    float repsVal = 0.0f;
    if (jsonGetFloat(payload, "target_reps", repsVal) && repsVal >= 1.0f) {
      gSession.targetReps = (int)clampf(repsVal, 1.0f, 99.0f);
    }

    float rMin = 0.0f, rMax = 110.0f;
    if (jsonGetFloat(payload, "rom_min_deg", rMin)) gSession.romMinDeg = clampf(rMin, 0.0f, 60.0f);
    if (jsonGetFloat(payload, "rom_max_deg", rMax)) gSession.romMaxDeg = clampf(rMax, gSession.romMinDeg + 10.0f, 150.0f);

    gSession.active        = true;
    gSession.paused        = false;
    gSession.estopped      = false;
    gSession.completedReps = 0;
    gSession.cycleProgress = 0.0f;
    gSession.activeMs      = 0;
    gSession.kneeAngleDeg  = gSession.romMinDeg;
    if (isCyclingMode(gSession.opMode, gSession.modeName)) {
      gSession.rpm          = 66.0f;
      gSession.speedKmh     = gSession.rpm * 0.31f;
      gSession.liveTorqueNm = gSession.targetTorqueNm;
      startCyclingStepperSession();
    } else {
      if (cyclingStepperActive) endCyclingStepperSession();
      gSession.liveTorqueNm = gSession.targetTorqueNm * 0.5f;
    }

    forwardCommandToHardware("start_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "set_torque") {
    String modeStr;
    if (jsonGetString(payload, "mode", modeStr)) {
      modeStr.toCharArray(gSession.modeName, sizeof(gSession.modeName));
    }
    String roleStr;
    if (jsonGetString(payload, "role", roleStr)) {
      roleStr.toCharArray(gSession.role, sizeof(gSession.role));
    }
    float opVal = 0.0f;
    if (jsonGetFloat(payload, "op_mode", opVal)) {
      gSession.opMode = (uint8_t)lroundf(opVal);
    }
    float prevTq = gSession.targetTorqueNm;
    float nextTq = gSession.targetTorqueNm;
    if (jsonGetFloat(payload, "torque_nm", nextTq) || jsonGetFloat(payload, "target_torque_ext", nextTq)) {
      gSession.targetTorqueNm = clampf(nextTq, 0.0f, maxAllowedTorque(gSession.opMode, gSession.role));
    }
    if (isCyclingMode(gSession.opMode, gSession.modeName)) {
      float deltaVal = 0.0f;
      bool hasDelta = jsonGetFloat(payload, "delta", deltaVal);
      if (hasDelta && deltaVal > 0.0f) {
        stepCyclingStepper(1);
      } else if (hasDelta && deltaVal < 0.0f) {
        stepCyclingStepper(-1);
      } else if (gSession.targetTorqueNm > prevTq + 0.001f) {
        stepCyclingStepper(1);
      } else if (gSession.targetTorqueNm < prevTq - 0.001f) {
        stepCyclingStepper(-1);
      }
    }
    forwardCommandToHardware("set_torque");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "pause_session") {
    gSession.paused = true;
    forwardCommandToHardware("pause_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "resume_session") {
    if (!gSession.estopped) {
      gSession.paused = false;
      if (!gSession.active) gSession.active = true;
      forwardCommandToHardware("resume_session");
      sendTelemetryFrame();
    }
    return;
  }

  if (cmdType == "stop_session") {
    gSession.active       = false;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    gSession.rpm          = 0.0f;
    gSession.speedKmh     = 0.0f;
    if (cyclingStepperActive || isCyclingMode(gSession.opMode, gSession.modeName)) {
      endCyclingStepperSession();
    }
    forwardCommandToHardware("stop_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "estop") {
    float estopFlag = 1.0f;
    jsonGetFloat(payload, "estop_active", estopFlag);
    if (estopFlag >= 0.5f) {
      gSession.estopped     = true;
      gSession.paused       = true;
      gSession.liveTorqueNm = 0.0f;
      gSession.rpm          = 0.0f;
      gSession.speedKmh     = 0.0f;
      if (cyclingStepperActive || isCyclingMode(gSession.opMode, gSession.modeName)) {
        endCyclingStepperSession();
      }
    } else {
      gSession.estopped = false;
    }
    forwardCommandToHardware("estop");
    sendTelemetryFrame();
    return;
  }
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress remoteIp = webSocket.remoteIP(clientNum);
      Serial.printf("[WS #%u] Client connected from %s\n", clientNum, remoteIp.toString().c_str());
      if (gSession.active) sendTelemetryFrame();
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[WS #%u] Client disconnected\n", clientNum);
      if (webSocket.connectedClients() == 0 && gSession.active) {
        gSession.paused = true;
        gSession.liveTorqueNm = 0.0f;
        forwardCommandToHardware("ws_all_clients_disconnected_pause");
      }
      break;
    case WStype_TEXT: {
      String msg;
      msg.reserve(length + 1);
      for (size_t i = 0; i < length; i++) msg += (char)payload[i];
      Serial.printf("[WS #%u RX] %s\n", clientNum, msg.c_str());
      handleWebsiteJson(msg);
      break;
    }
    default:
      break;
  }
}

// ============================================================================
// 6. STEP SIMULATION OR READ REAL HARDWARE TELEMETRY
// ============================================================================
void updateRehabKinematics(uint32_t dtMs) {
  if (!gSession.active || gSession.paused || gSession.estopped) {
    gSession.liveTorqueNm = 0.0f;
    gSession.rpm          = 0.0f;
    gSession.speedKmh     = 0.0f;
    return;
  }

  float dtSec = (float)dtMs / 1000.0f;
  gSession.activeMs += dtMs;

#if BENCH_SIMULATION_MODE
  if (isCyclingMode(gSession.opMode, gSession.modeName)) {
    // Realistic ergometer cadence coupled lightly to target resistance torque
    float t = (float)gSession.activeMs * 0.001f;
    float baseRpm = clampf(78.0f - (gSession.targetTorqueNm * 0.25f), 42.0f, 92.0f);
    gSession.rpm = clampf(baseRpm + 3.5f * sinf(t * 1.1f), 25.0f, 120.0f);
    gSession.speedKmh = gSession.rpm * 0.31f;

    // Live torque tracks setpoint smoothly with dual-leg pedal stroke ripple
    float pedalRipple = 1.6f * sinf(t * (gSession.rpm / 60.0f) * 2.0f * PI);
    float desiredTorque = clampf(gSession.targetTorqueNm + pedalRipple, 0.0f, 85.0f);
    gSession.liveTorqueNm += (desiredTorque - gSession.liveTorqueNm) * clampf(dtSec * 6.0f, 0.0f, 1.0f);

    // Count 1 pedal cycle roughly every ~2.5s..3.5s proportional to cadence
    gSession.cycleProgress += dtSec * (gSession.rpm / 180.0f);
    if (gSession.cycleProgress >= 1.0f) {
      gSession.cycleProgress -= 1.0f;
      gSession.completedReps += 1;
    }
  } else {
    // Flexion / Extension stroke: stop adding torque automatically once all target reps complete
    if (gSession.completedReps >= gSession.targetReps) {
      gSession.liveTorqueNm = 0.0f;
      gSession.kneeAngleDeg = gSession.romMinDeg;
      return;
    }

    // ~3.6 seconds per full flexion + extension rep
    gSession.cycleProgress += dtSec / 3.6f;
    if (gSession.cycleProgress >= 1.0f) {
      gSession.cycleProgress -= 1.0f;
      if (gSession.completedReps < gSession.targetReps) {
        gSession.completedReps += 1;
      }
    }

    // Cosine ease between romMinDeg and romMaxDeg
    float span = gSession.romMaxDeg - gSession.romMinDeg;
    float normAngle = 0.5f * (1.0f - cosf(gSession.cycleProgress * 2.0f * PI));
    gSession.kneeAngleDeg = gSession.romMinDeg + span * normAngle;

    float maxTq = maxAllowedTorque(gSession.opMode, gSession.role);
    float curveGain = (gSession.opMode == OP_CPM)
      ? (0.85f + 0.15f * normAngle)
      : (0.65f + 0.35f * sinf(normAngle * PI));
    float desiredTorque = clampf(gSession.targetTorqueNm * curveGain, 0.0f, maxTq);
    gSession.liveTorqueNm += (desiredTorque - gSession.liveTorqueNm) * clampf(dtSec * 8.0f, 0.0f, 1.0f);
  }
#else
  // Hardware telemetry integration point:
  // Populate gSession.liveTorqueNm, gSession.rpm, gSession.speedKmh, gSession.kneeAngleDeg,
  // and gSession.completedReps inside your ESP-NOW OnDataRecv callback from U2/U3.
  (void)dtSec;
#endif
}

// ============================================================================
// 7. USB SERIAL BENCH CONSOLE + WIFI CONNECTION + MAIN LOOP
// ============================================================================
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  if (line.equalsIgnoreCase("IP") || line.equalsIgnoreCase("STATUS")) {
    Serial.printf("Wi-Fi SSID=%s IP=%s ch=%u WS=ws://%s:%u/\n",
                  WIFI_SSID,
                  WiFi.localIP().toString().c_str(),
                  (unsigned)currentMasterChannel(),
                  WiFi.localIP().toString().c_str(),
                  (unsigned)WS_PORT);
    Serial.printf("Stepper U3: paired=%s steps=%ld/%ld angle=%.2f deg\n",
                  hasLearnedStepper ? "YES" : "BROADCAST",
                  (long)gStepperStatus.currentSteps,
                  (long)gStepperStatus.targetSteps,
                  gStepperStatus.angleDeg);
    forwardCommandToHardware("serial_status");
  } else if (line.equalsIgnoreCase("START")) {
    gSession.opMode        = OP_CYCLE_TORQUE;
    strcpy(gSession.modeName, "cycling");
    strcpy(gSession.role, "resistance");
    gSession.active        = true;
    gSession.paused        = false;
    gSession.estopped      = false;
    gSession.completedReps = 0;
    gSession.rpm           = 66.0f;
    gSession.speedKmh      = gSession.rpm * 0.31f;
    gSession.liveTorqueNm  = gSession.targetTorqueNm;
    startCyclingStepperSession();
    forwardCommandToHardware("serial_start");
    sendTelemetryFrame();
  } else if (line.equalsIgnoreCase("END") || line.equalsIgnoreCase("STOP")) {
    gSession.active       = false;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    gSession.rpm          = 0.0f;
    gSession.speedKmh     = 0.0f;
    endCyclingStepperSession();
    forwardCommandToHardware("serial_end");
    sendTelemetryFrame();
  } else if (line.equalsIgnoreCase("STEP+") || line == "+") {
    stepCyclingStepper(1);
    Serial.println("[ESP-NOW TX] Sent +1 step to Stepper ESP");
  } else if (line.equalsIgnoreCase("STEP-") || line == "-") {
    stepCyclingStepper(-1);
    Serial.println("[ESP-NOW TX] Sent -1 step to Stepper ESP");
  } else if (line.equalsIgnoreCase("REP")) {
    if (gSession.active && !gSession.paused && gSession.completedReps < gSession.targetReps) {
      gSession.completedReps += 1;
      sendTelemetryFrame();
      Serial.printf("Incremented reps -> %d/%d\n", gSession.completedReps, gSession.targetReps);
    }
  } else if (line.equalsIgnoreCase("ESTOP")) {
    gSession.estopped = true;
    gSession.paused   = true;
    if (cyclingStepperActive || isCyclingMode(gSession.opMode, gSession.modeName)) {
      endCyclingStepperSession();
    }
    sendTelemetryFrame();
    Serial.println("HW/Serial E-Stop ENGAGED");
  } else if (line.equalsIgnoreCase("CLEAR")) {
    gSession.estopped = false;
    sendTelemetryFrame();
    Serial.println("E-Stop CLEARED (press Resume on website)");
  } else {
    // Allow multiple '+' or '-' characters on one line (e.g. "+++")
    bool allStepChars = true;
    for (size_t i = 0; i < line.length(); i++) {
      if (line[i] != '+' && line[i] != '-') {
        allStepChars = false;
        break;
      }
    }
    if (allStepChars) {
      for (size_t i = 0; i < line.length(); i++) {
        stepCyclingStepper(line[i] == '+' ? 1 : -1);
      }
      Serial.printf("[ESP-NOW TX] Sent '%s' to Stepper ESP\n", line.c_str());
    }
  }
}

bool wifiWasConnected = false;
bool mdnsStarted      = false;

void onWiFiGotConnected() {
  wifiWasConnected = true;
  Serial.printf("Wi-Fi connected! ESP32 IP: %s (channel %u)\n",
                WiFi.localIP().toString().c_str(),
                (unsigned)currentMasterChannel());
  Serial.printf("Enter in DOULA Dashboard: ws://%s:%u/\n",
                WiFi.localIP().toString().c_str(),
                (unsigned)WS_PORT);
  if (!mdnsStarted && MDNS.begin(MDNS_HOST)) {
    MDNS.addService("ws", "tcp", WS_PORT);
    mdnsStarted = true;
    Serial.printf("mDNS active: ws://%s.local:%u/\n", MDNS_HOST, (unsigned)WS_PORT);
  }
  digitalWrite(LED_BUILTIN, HIGH);
}

void ensureWiFiConnected(bool waitForConnect) {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) onWiFiGotConnected();
    return;
  }

  wifiWasConnected = false;
  uint32_t now = millis();
  if (!waitForConnect && (now - lastWifiRetryMs < WIFI_RECONNECT_RETRY_MS)) {
    return;
  }
  lastWifiRetryMs = now;

  Serial.printf("Connecting to Wi-Fi SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // Low-latency WebSocket & ESP-NOW streaming
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  if (!waitForConnect) return;

  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
    delay(250);
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    onWiFiGotConnected();
  } else {
    Serial.println("Wi-Fi connection timed out — continuing ESP-NOW & retrying in background...");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  delay(300);
  Serial.println("\n=== DOULA ESP32-S NodeMCU V1.1 Master Bridge ===");
  Serial.println("Serial commands: STATUS | IP | START | + | - | END | REP | ESTOP | CLEAR");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

#if ENABLE_HW_ESTOP_PIN
  pinMode(HW_ESTOP_PIN, INPUT_PULLUP);
#endif

  masterBootId = esp_random();
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  ensureWiFiConnected(true);
  initEspNow();

  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.printf("WebSocket server listening on port %u\n", (unsigned)WS_PORT);

  lastLoopMs = millis();
  lastTelemetryMs = millis();
}

void loop() {
  ensureWiFiConnected(false);
  webSocket.loop();
  processStepperStatusQueue();
  serviceEspNowHeartbeat();
  handleSerialCommands();

#if ENABLE_HW_ESTOP_PIN
  if (digitalRead(HW_ESTOP_PIN) == LOW && !gSession.estopped) {
    gSession.estopped     = true;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    if (cyclingStepperActive || isCyclingMode(gSession.opMode, gSession.modeName)) {
      endCyclingStepperSession();
    }
    forwardCommandToHardware("hw_gpio4_estop");
    sendTelemetryFrame();
  }
#endif

  uint32_t now = millis();
  uint32_t dt = now - lastLoopMs;
  lastLoopMs = now;

  updateRehabKinematics(dt);

  if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
    lastTelemetryMs = now;
    if (gSession.active) {
      sendTelemetryFrame();
    }
  }

  // Heartbeat LED status: solid when session active, fast blink if E-Stop, slow blink when idle
  uint32_t blinkPeriod = gSession.estopped ? 140 : (gSession.active && !gSession.paused ? 0 : 900);
  if (blinkPeriod == 0) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (now - lastLedToggleMs >= blinkPeriod) {
    lastLedToggleMs = now;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
}
