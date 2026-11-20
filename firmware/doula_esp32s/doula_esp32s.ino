/*
 * DOULA Knee Rehabilitation System — ESP32-S (U1 Master / WebSocket + ESP-NOW Coordinator)
 * Target Board : ESP32-S NodeMCU V1.1 (DOIT / NodeMCU-32S / ESP32 Dev Module)
 * Comms        : Wi-Fi STA + mDNS (doula-esp32.local) + WebSocket Server (port 81)
 *                + Unified ESP-NOW Link to U2 (esp_tophub_staged) & U3 (doula_stepper_esp32)
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
#include "rehab_espnow_protocol.h"

// ============================================================================
// 1. USER CONFIGURATION — EDIT YOUR WI-FI / HOTSPOT CREDENTIALS BEFORE FLASH
// ============================================================================
const char* WIFI_SSID     = "kanishkshotspot";
const char* WIFI_PASSWORD = "123345678";
const char* MDNS_HOST     = "doula-esp32";   // Reachable at ws://doula-esp32.local:81/
const uint16_t WS_PORT    = 81;

// Per-node auto-live arbitration:
//  - When U2 or U3 is linked over ESP-NOW, U1 always uses live hardware telemetry for that node.
//  - BENCH_SIMULATION_FALLBACK == true simulates only an unlinked node for USB bench testing.
#define BENCH_SIMULATION_FALLBACK      true

// When U3 is linked with ENABLE_FLYWHEEL_ENCODER == false, use U3's real stepper position
// while synthesizing cycling RPM/speed/torque around targetTorqueNm for the cycling HUD.
#define CYCLING_RPM_FALLBACK_NO_ENC2   true

// Require explicit homing (Set 0 deg Home or Auto-Home STOP1) when U2 is unhomed (H:0).
#define AUTO_HOME_U2_ON_START          false

// Keep false while testing a bare ESP32-S board with nothing wired to GPIO4.
// Flip to true once the physical active-LOW E-Stop pushbutton is wired to GPIO4 + GND.
#define ENABLE_HW_ESTOP_PIN            false

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif
#define HW_ESTOP_PIN 4

// ============================================================================
// 2. REHAB SESSION STATE & ESP-NOW COORDINATOR STATE
// ============================================================================
constexpr uint32_t TELEMETRY_PERIOD_MS     = 50;    // 20 Hz WebSocket live stream
constexpr uint32_t ESPNOW_HEARTBEAT_MS     = 100;   // 10 Hz heartbeat (< 500 ms U2 watchdog)
constexpr uint32_t NODE_LINK_TIMEOUT_MS    = 3500;  // Revert peer to broadcast discovery after 3.5s
constexpr uint32_t AUTOHOME_TIMEOUT_MS     = 16000; // Abort auto-home wait after 16s
constexpr uint32_t MANUAL_SETUP_TIMEOUT_MS = 26000; // Max wait for 25s full setup
constexpr uint32_t WIFI_RECONNECT_RETRY_MS = 15000;

struct SessionState {
  bool active        = false;
  bool paused        = false;
  bool estopped      = false;
  uint8_t opMode     = OP_CYCLE_TORQUE;
  uint8_t pathology  = PATH_STANDARD;
  char modeName[24]  = "cycling";
  char role[16]      = "resistance";

  float targetTorqueNm     = 14.0f;  // Extension / Cycling target [N*m]
  float targetTorqueFlexNm = 9.1f;   // Flexion target [N*m]
  float targetVelDegS      = 45.0f;  // Speed [deg/s] or Cycling cadence [RPM]
  float liveTorqueNm       = 0.0f;   // Measured/simulated applied torque
  int   targetReps         = 15;
  int   completedReps      = 0;
  float romMinDeg          = 120.0f;
  float romMaxDeg          = 180.0f;
  float dOffsetM           = 0.250f;
  float lLegM              = 0.300f;

  // Live sensor / simulation kinematics
  float kneeAngleDeg   = 0.0f;
  float rpm            = 0.0f;
  float speedKmh       = 0.0f;
  float cycleProgress  = 0.0f;       // 0.0 .. 1.0 within current rep / pedal stroke
  uint32_t activeMs    = 0;
};

struct WorkerPeerState {
  const char* label;
  uint8_t     mac[6];
  bool        linked;
  uint32_t    lastRxMs;
  uint32_t    rxCount;

  explicit WorkerPeerState(const char* lbl)
    : label(lbl), mac{0, 0, 0, 0, 0, 0}, linked(false), lastRxMs(0), rxCount(0) {}
};

struct QueuedTopHubTelem {
  uint8_t                 srcMac[6];
  TopHubTelemetryPacket_t pkt;
};

struct QueuedTopHubRep {
  uint8_t                  srcMac[6];
  TopHubRepSummaryPacket_t pkt;
};

struct QueuedCyclingTelem {
  uint8_t                  srcMac[6];
  CyclingTelemetryPacket_t pkt;
};

SessionState     gSession;
WebSocketsServer webSocket(WS_PORT);

uint32_t lastLoopMs      = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastLedToggleMs = 0;
bool     ledState        = false;

uint8_t  BROADCAST_MAC[6]        = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool     espNowReady             = false;
uint32_t masterBootId            = 0;
uint16_t espNowHbSeq             = 0;
uint16_t espNowActionSeq         = 0;
uint32_t lastEspNowHeartbeatMs   = 0;
uint32_t lastWifiRetryMs         = 0;
uint8_t  lastAnnouncedChannel    = 0;
volatile uint32_t espNowRxBad    = 0;

WorkerPeerState u2Peer("TopHub U2");
WorkerPeerState u3Peer("Cycling U3");

QueueHandle_t topHubTelemQueue = nullptr;
QueueHandle_t topHubRepQueue   = nullptr;
QueueHandle_t cyclingTelemQueue = nullptr;

TopHubTelemetryPacket_t  gTopHubTelem     = {};
TopHubRepSummaryPacket_t gLastRepSummary  = {};
CyclingTelemetryPacket_t gCyclingTelem    = {};
bool                     gHasRepSummary   = false;

// Top-Hub session coordination & safe positioning state
enum SetupPhase : uint8_t {
  SETUP_NONE            = 0,
  SETUP_AUTOHOME        = 1,
  SETUP_MOVING_TO_START = 2,
  SETUP_READY_AT_START  = 3,
  SETUP_PARKING_HOME    = 4
};

SetupPhase setupPhase        = SETUP_NONE;
uint32_t   setupStartedMs    = 0;
bool       sawU2Positioning  = false;
float      setupTargetDeg    = 120.0f;

bool     pendingAutoHome     = false;
uint8_t  postHomeOpMode      = OP_IDLE;
uint32_t autoHomeStartedMs   = 0;
bool     sawU2BusyHoming     = false;
uint8_t  manualSetupOp       = OP_IDLE;
uint32_t manualSetupStartMs  = 0;
bool     sawManualBusyHoming = false;
uint16_t topHubRepBaseline   = 0;
bool     repLimitAutoStopped = false;
bool     u2FaultNotified     = false;
bool     serialStreamTopHub  = false;
uint32_t lastSerialStreamMs  = 0;

const char* setupPhaseName(SetupPhase p) {
  switch (p) {
    case SETUP_AUTOHOME:        return "autohoming";
    case SETUP_MOVING_TO_START: return "moving_to_start";
    case SETUP_READY_AT_START:  return "ready_at_start";
    case SETUP_PARKING_HOME:    return "parking_home";
    case SETUP_NONE:
    default:                    return "idle";
  }
}

bool isSetupMotionWarningActive(uint8_t u2Flags) {
  if ((u2Flags & (STATE_FLAG_BUSY_HOMING | STATE_FLAG_POSITIONING)) != 0) return true;
  if (manualSetupOp == OP_SETUP_FULL_25S) return true;
  return setupPhase == SETUP_AUTOHOME ||
         setupPhase == SETUP_MOVING_TO_START ||
         setupPhase == SETUP_PARKING_HOME;
}

// ============================================================================
// 3. LIGHTWEIGHT JSON FIELD PARSERS & HELPERS
// ============================================================================
bool jsonGetString(const String& json, const char* key, String& out) {
  String pattern = String("\"") + key + "\"";
  int keyPos = json.indexOf(pattern);
  if (keyPos < 0) return false;
  int colonPos = json.indexOf(':', keyPos + pattern.length());
  if (colonPos < 0) return false;

  int idx = colonPos + 1;
  while (idx < (int)json.length() && (json[idx] == ' ' || json[idx] == '\t')) idx++;
  if (idx >= (int)json.length() || json[idx] != '"') return false;

  int quoteEnd = json.indexOf('"', idx + 1);
  if (quoteEnd < 0) return false;
  out = json.substring(idx + 1, quoteEnd);
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
  if (idx >= (int)json.length() || json[idx] == '"') return false;

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
  if (isCyclingMode(opMode, "")) return 85.0f;
  if (opMode == OP_CPM || opMode == OP_ACTIVE_ASSIST || strcmp(role, "support") == 0) return 25.0f;
  return 45.0f;
}

uint8_t parsePathologyString(const String& pathStr) {
  if (pathStr.equalsIgnoreCase("ACLR") || pathStr.equalsIgnoreCase("ACL")) return PATH_ACL_PROTECTED;
  if (pathStr.equalsIgnoreCase("TKA")) return PATH_TKA;
  if (pathStr.equalsIgnoreCase("PFPS")) return PATH_PFPS;
  return PATH_STANDARD;
}

void sendWsAck(bool forwarded, const char* message, const char* severity = "info") {
  if (webSocket.connectedClients() == 0 || !message) return;
  char buf[320];
  snprintf(
    buf,
    sizeof(buf),
    "{\"type\":\"esp_ack\",\"forwarded\":%s,\"severity\":\"%s\",\"message\":\"%s\"}",
    forwarded ? "true" : "false",
    severity ? severity : "info",
    message
  );
  webSocket.broadcastTXT(buf);
}

// ============================================================================
// 4. ESP-NOW PEER MANAGEMENT, COMMAND TX & QUEUE PROCESSING
// ============================================================================
uint8_t currentMasterChannel() {
  int32_t ch = WiFi.channel();
  if (ch >= 1 && ch <= 13) return (uint8_t)ch;
  uint8_t primary = REHAB_WIFI_CHANNEL;
  wifi_second_chan_t sec = WIFI_SECOND_CHAN_NONE;
  if (esp_wifi_get_channel(&primary, &sec) == ESP_OK && primary >= 1 && primary <= 13) {
    return primary;
  }
  return REHAB_WIFI_CHANNEL;
}

void registerOrRefreshWorkerPeer(WorkerPeerState& peerState, const uint8_t* srcMac) {
  peerState.lastRxMs = millis();
  peerState.rxCount++;
  if (peerState.linked && memcmp(peerState.mac, srcMac, 6) == 0) return;

  if (peerState.linked) {
    esp_now_del_peer(peerState.mac);
    peerState.linked = false;
  }

  memcpy(peerState.mac, srcMac, 6);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, peerState.mac, 6);
  peer.channel = 0; // Follow Master's active STA channel
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    peerState.linked = true;
    Serial.printf(
      "[ESP-NOW] Linked %s MAC %02X:%02X:%02X:%02X:%02X:%02X on ch %u\n",
      peerState.label,
      peerState.mac[0], peerState.mac[1], peerState.mac[2],
      peerState.mac[3], peerState.mac[4], peerState.mac[5],
      (unsigned)currentMasterChannel()
    );
  }
}

void expireWorkerPeerIfStale(WorkerPeerState& peerState, uint32_t now) {
  if (peerState.linked && (now - peerState.lastRxMs > NODE_LINK_TIMEOUT_MS)) {
    esp_now_del_peer(peerState.mac);
    peerState.linked = false;
    Serial.printf("[ESP-NOW] %s telemetry timeout — reverting to broadcast discovery\n", peerState.label);
  }
}

void handleEspNowRxPacket(const uint8_t* srcMac, const uint8_t* data, int len) {
  if (!srcMac || !data || len < 1) return;
  uint8_t pktType = data[0];

  if (pktType == PKT_TOPHUB_TELEMETRY && len == (int)sizeof(TopHubTelemetryPacket_t)) {
    TopHubTelemetryPacket_t pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.crc16 == computeCRC16(&pkt, sizeof(pkt) - sizeof(uint16_t)) && topHubTelemQueue != nullptr) {
      QueuedTopHubTelem item{};
      memcpy(item.srcMac, srcMac, 6);
      item.pkt = pkt;
      xQueueSend(topHubTelemQueue, &item, 0);
      return;
    }
  } else if (pktType == PKT_TOPHUB_REP_SUMMARY && len == (int)sizeof(TopHubRepSummaryPacket_t)) {
    TopHubRepSummaryPacket_t pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.crc16 == computeCRC16(&pkt, sizeof(pkt) - sizeof(uint16_t)) && topHubRepQueue != nullptr) {
      QueuedTopHubRep item{};
      memcpy(item.srcMac, srcMac, 6);
      item.pkt = pkt;
      xQueueSend(topHubRepQueue, &item, 0);
      return;
    }
  } else if (pktType == PKT_CYCLING_TELEMETRY && len == (int)sizeof(CyclingTelemetryPacket_t)) {
    CyclingTelemetryPacket_t pkt{};
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.crc16 == computeCRC16(&pkt, sizeof(pkt) - sizeof(uint16_t)) && cyclingTelemQueue != nullptr) {
      QueuedCyclingTelem item{};
      memcpy(item.srcMac, srcMac, 6);
      item.pkt = pkt;
      xQueueSend(cyclingTelemQueue, &item, 0);
      return;
    }
  }

  espNowRxBad++;
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info) return;
  handleEspNowRxPacket(info->src_addr, data, len);
}
#else
static void onEspNowRecv(const uint8_t* srcMac, const uint8_t* data, int len) {
  handleEspNowRxPacket(srcMac, data, len);
}
#endif

void initEspNow() {
  if (espNowReady) return;
  if (topHubTelemQueue == nullptr)  topHubTelemQueue  = xQueueCreate(8, sizeof(QueuedTopHubTelem));
  if (topHubRepQueue == nullptr)    topHubRepQueue    = xQueueCreate(8, sizeof(QueuedTopHubRep));
  if (cyclingTelemQueue == nullptr) cyclingTelemQueue = xQueueCreate(8, sizeof(QueuedCyclingTelem));

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: esp_now_init() failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t bcastPeer{};
  memcpy(bcastPeer.peer_addr, BROADCAST_MAC, 6);
  bcastPeer.channel = 0; // Follow active STA channel
  bcastPeer.ifidx   = WIFI_IF_STA;
  bcastPeer.encrypt = false;
  esp_now_add_peer(&bcastPeer);

  espNowReady = true;
  Serial.printf("[ESP-NOW] Ready on MAC %s (channel %u)\n",
                WiFi.macAddress().c_str(),
                (unsigned)currentMasterChannel());
}

uint8_t desiredTopHubOpMode() {
  if (gSession.estopped) return OP_IDLE;
  if (pendingAutoHome) return OP_SETUP_QUICK_6S;
  if (manualSetupOp != OP_IDLE) return manualSetupOp;
  if (setupPhase == SETUP_MOVING_TO_START || setupPhase == SETUP_READY_AT_START) return OP_GOTO_START;
  if (setupPhase == SETUP_PARKING_HOME) return OP_PARK_HOME;
  if (gSession.active && !gSession.paused && !isCyclingMode(gSession.opMode, gSession.modeName)) {
    if (repLimitAutoStopped) return OP_IDLE;
    return gSession.opMode;
  }
  return OP_IDLE;
}

uint8_t desiredCyclingOpMode() {
  if (gSession.estopped) return OP_IDLE;
  if (gSession.active && isCyclingMode(gSession.opMode, gSession.modeName)) {
    // Keep cycling stepper position engaged across Pause/Resume; release on Stop/E-Stop.
    return gSession.opMode;
  }
  return OP_IDLE;
}

void sendRehabCommand(uint8_t targetNode,
                      uint8_t opMode,
                      uint8_t estopMode,
                      uint8_t stepAction = STEP_ACT_NONE,
                      bool newActionSeq  = false,
                      uint8_t repeatCount = 1) {
  if (!espNowReady) return;

  RehabCommandPacket_t pkt{};
  pkt.pkt_type           = PKT_HEARTBEAT_CMD;
  pkt.target_node        = targetNode;
  pkt.seq_num            = ++espNowHbSeq;
  pkt.boot_id            = masterBootId;
  pkt.wifi_channel       = currentMasterChannel();
  pkt.op_mode            = opMode;
  pkt.pathology_profile  = gSession.pathology;
  pkt.estop_active       = estopMode;
  pkt.stepper_action     = stepAction;
  pkt.action_seq         = newActionSeq ? (++espNowActionSeq == 0 ? ++espNowActionSeq : espNowActionSeq) : 0;
  pkt.d_offset_m         = gSession.dOffsetM;
  pkt.L_leg_m            = gSession.lLegM;
  pkt.rom_min_deg        = gSession.romMinDeg;
  pkt.rom_max_deg        = gSession.romMaxDeg;
  pkt.target_vel_deg_s   = gSession.targetVelDegS;
  pkt.target_torque_ext  = gSession.targetTorqueNm;
  pkt.target_torque_flex = gSession.targetTorqueFlexNm;
  pkt.crc16              = computeCRC16(&pkt, sizeof(pkt) - sizeof(uint16_t));

  const uint8_t* destMac = BROADCAST_MAC;
  if (targetNode == NODE_TOPHUB && u2Peer.linked) {
    destMac = u2Peer.mac;
  } else if (targetNode == NODE_CYCLING && u3Peer.linked) {
    destMac = u3Peer.mac;
  }

  for (uint8_t i = 0; i < repeatCount; i++) {
    esp_now_send(destMac, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    if (i + 1 < repeatCount) delay(2);
  }
}

void syncAllNodesImmediate(uint8_t repeatCount = 1) {
  uint8_t estop = gSession.estopped ? ESTOP_HARD_STOP_LOCKED : ESTOP_NORMAL;
  sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(), estop, STEP_ACT_NONE, false, repeatCount);
  sendRehabCommand(NODE_CYCLING, desiredCyclingOpMode(), estop, STEP_ACT_NONE, false, repeatCount);
}

void triggerHardStopReleaseBurst() {
  gSession.estopped = false;
  pendingAutoHome   = false;
  manualSetupOp     = OP_IDLE;
  setupPhase        = SETUP_NONE;
  sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
  sendRehabCommand(NODE_CYCLING, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_HOME, true, 2);
}

void stepCyclingStepper(int dir) {
  uint8_t estop = gSession.estopped ? ESTOP_HARD_STOP_LOCKED : ESTOP_NORMAL;
  uint8_t act   = (dir > 0) ? STEP_ACT_PLUS : STEP_ACT_MINUS;
  sendRehabCommand(NODE_CYCLING, desiredCyclingOpMode(), estop, act, true, 2);
}

void processTopHubQueues() {
  uint32_t now = millis();

  if (topHubTelemQueue != nullptr) {
    QueuedTopHubTelem item{};
    while (xQueueReceive(topHubTelemQueue, &item, 0) == pdTRUE) {
      registerOrRefreshWorkerPeer(u2Peer, item.srcMac);
      gTopHubTelem = item.pkt;

      bool isHomed       = (gTopHubTelem.state_flags & STATE_FLAG_HOMED) != 0;
      bool isBusyHome    = (gTopHubTelem.state_flags & STATE_FLAG_BUSY_HOMING) != 0;
      bool isPositioning = (gTopHubTelem.state_flags & STATE_FLAG_POSITIONING) != 0;
      bool isHardStop    = (gTopHubTelem.state_flags & STATE_FLAG_HARDSTOP_LOCKED) != 0;
      bool isSpasm       = (gTopHubTelem.state_flags & STATE_FLAG_SPASM_STOP) != 0;
      bool isFaultLock   = (gTopHubTelem.state_flags & STATE_FLAG_FAULT_LOCKOUT) != 0;

      // Always mirror live knee angle from U2
      gSession.kneeAngleDeg = gTopHubTelem.knee_angle_deg;

      // 1. Auto-Home before starting Flexion therapy (legacy compile-time flag if enabled)
      if (pendingAutoHome) {
        if (isBusyHome) sawU2BusyHoming = true;
        if (isHomed && !isBusyHome && (now - autoHomeStartedMs >= 150)) {
          pendingAutoHome = false;
          sawU2BusyHoming = false;
          gSession.opMode = postHomeOpMode;
          topHubRepBaseline = gTopHubTelem.current_rep;
          Serial.println("[U2] Auto-Home (0 deg) complete -> entering therapy mode");
          sendWsAck(true, "Top-Hub homed (0 deg) — therapy started.", "info");
          sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(), ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
        } else if ((sawU2BusyHoming && !isBusyHome && !isHomed) ||
                   (now - autoHomeStartedMs > AUTOHOME_TIMEOUT_MS)) {
          pendingAutoHome = false;
          sawU2BusyHoming = false;
          gSession.active = false;
          gSession.paused = true;
          Serial.println("[U2] Auto-Home aborted or timed out — check STOP1 & 24V power");
          sendWsAck(true, "Top-Hub auto-home aborted/timed out — check 24V power & STOP1.", "fault");
          sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 1);
        }
      }

      // 2. Manual setup/home/bench progress
      if (manualSetupOp != OP_IDLE) {
        if (isBusyHome) sawManualBusyHoming = true;
        uint32_t elapsedSetup = now - manualSetupStartMs;

        if (manualSetupOp == OP_SETUP_QUICK_6S) {
          if ((isHomed && !isBusyHome && elapsedSetup >= 220) || elapsedSetup > 4000) {
            manualSetupOp       = OP_IDLE;
            sawManualBusyHoming = false;
            if (setupPhase == SETUP_AUTOHOME) setupPhase = SETUP_NONE;
            sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(), ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
            Serial.printf("[U2] Quick 0 deg Home complete (homed=%d, Tk=%.2f deg)\n",
                          isHomed ? 1 : 0, gTopHubTelem.knee_angle_deg);
            sendWsAck(true, isHomed
              ? "Top-Hub 0.0° Home & ACS712 zero latched (H:1)."
              : "Top-Hub quick home timed out (check U2 link).",
              isHomed ? "info" : "warning");
          }
        } else if (manualSetupOp == OP_AUTO_BENCH_TEST) {
          if (elapsedSetup > 6500) {
            manualSetupOp       = OP_IDLE;
            sawManualBusyHoming = false;
            sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(), ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
            Serial.println("[U2] Auto bench pulse test finished -> OP_IDLE");
            sendWsAck(true, "Top-Hub bench pulse test finished.", "info");
          }
        } else {
          bool abortedEarly = (!sawManualBusyHoming && !isBusyHome && elapsedSetup >= 2200);
          if ((sawManualBusyHoming && !isBusyHome) || abortedEarly ||
              (elapsedSetup > MANUAL_SETUP_TIMEOUT_MS)) {
            manualSetupOp       = OP_IDLE;
            sawManualBusyHoming = false;
            if (setupPhase == SETUP_AUTOHOME) setupPhase = SETUP_NONE;
            sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(), ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
            Serial.printf("[U2] Full Auto-Home finished (homed=%d, Tk=%.2f deg)\n",
                          isHomed ? 1 : 0, gTopHubTelem.knee_angle_deg);
            sendWsAck(true, isHomed
              ? "Top-Hub motorized Auto-Home complete (STOP1 latched)."
              : "Top-Hub Auto-Home aborted — check 24V power (>=15V) & motor leads, or use Set 0° Home.",
              isHomed ? "info" : "warning");
          }
        }
      }

      // 2B. Safe Two-Stage Setup Positioning (0->120 deg) and Safe Return to 0 deg Park
      if (setupPhase == SETUP_MOVING_TO_START) {
        if (isPositioning) sawU2Positioning = true;
        uint32_t elapsedPos = now - setupStartedMs;
        if (isHardStop || isSpasm || isFaultLock) {
          setupPhase      = SETUP_NONE;
          gSession.active = false;
          gSession.paused = true;
          if (isHardStop) gSession.estopped = true;
          else sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 1);
          Serial.println("[U2] Setup positioning stopped by safety/obstruction");
          sendWsAck(true, "Setup stopped — obstruction or safety fault detected. Check clearance and try again.", "fault");
        } else if (!isPositioning && fabsf(gTopHubTelem.knee_angle_deg - setupTargetDeg) <= 3.5f && elapsedPos >= 300) {
          setupPhase = SETUP_READY_AT_START;
          Serial.printf("[U2] Reached start position %.1f deg -> holding for leg placement & Begin Exercise\n",
                        gTopHubTelem.knee_angle_deg);
          sendWsAck(true, "Starting position reached — place your leg securely on the shin pad and click Begin Exercise.", "info");
        } else if (elapsedPos > 25000) {
          setupPhase      = SETUP_NONE;
          gSession.active = false;
          gSession.paused = true;
          sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
          Serial.println("[U2] Setup positioning timed out");
          sendWsAck(true, "Setup positioning timed out — check 24V power and clearance.", "fault");
        }
      } else if (setupPhase == SETUP_READY_AT_START) {
        if (isHardStop || isSpasm || isFaultLock) {
          setupPhase      = SETUP_NONE;
          gSession.active = false;
          gSession.paused = true;
          if (isHardStop) gSession.estopped = true;
          else sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 1);
          sendWsAck(true, "Setup hold interrupted by safety fault — check clearance before retrying.", "fault");
        }
      } else if (setupPhase == SETUP_PARKING_HOME) {
        if (isPositioning) sawU2Positioning = true;
        uint32_t elapsedPark = now - setupStartedMs;
        if (isHardStop || isSpasm || isFaultLock) {
          setupPhase = SETUP_NONE;
          if (isHardStop) gSession.estopped = true;
          else sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 1);
          sendWsAck(true, "Return to 0 deg Park interrupted by obstruction/safety stop.", "fault");
        } else if (!isPositioning && gTopHubTelem.knee_angle_deg <= 4.5f && elapsedPark >= 300) {
          setupPhase = SETUP_NONE;
          sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
          Serial.printf("[U2] Parked safely at Tk=%.2f deg -> OP_IDLE\n", gTopHubTelem.knee_angle_deg);
          sendWsAck(true, "Arm safely parked at 0 deg Home.", "info");
        } else if (elapsedPark > 25000) {
          setupPhase = SETUP_NONE;
          sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
          sendWsAck(true, "Return to 0 deg Park timed out — motor released to IDLE.", "warning");
        }
      }

      // 3. Mirror U2 safety lockouts back to Master & WebSocket
      bool inFlexSession = gSession.active && !gSession.paused &&
                           !isCyclingMode(gSession.opMode, gSession.modeName) &&
                           !pendingAutoHome && setupPhase == SETUP_NONE;
      if (inFlexSession) {
        if (isHardStop) {
          gSession.estopped     = true;
          gSession.paused       = true;
          gSession.liveTorqueNm = 0.0f;
          if (!u2FaultNotified) {
            u2FaultNotified = true;
            Serial.println("[U2 SAFETY] Hard-Stop locked on Top-Hub");
            sendWsAck(true, "Top-Hub Hard-Stop locked — clear E-Stop to release.", "fault");
          }
          continue;
        }
        if (isSpasm || isFaultLock) {
          gSession.paused       = true;
          gSession.liveTorqueNm = 0.0f;
          if (!u2FaultNotified) {
            u2FaultNotified = true;
            Serial.printf("[U2 SAFETY] %s triggered -> session paused\n",
                          isSpasm ? "Spasm backoff" : "Fault lockout");
            sendWsAck(true, isSpasm
              ? "Top-Hub spasm guard triggered — press Resume or Stop."
              : "Top-Hub fault lockout — check 24V/current & press Resume.",
              isSpasm ? "spasm" : "fault");
            sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 1);
          }
          continue;
        }
        u2FaultNotified = false;

        gSession.kneeAngleDeg = gTopHubTelem.knee_angle_deg;
        gSession.liveTorqueNm = fabsf(gTopHubTelem.knee_torque_nm);

        if (gTopHubTelem.current_rep < topHubRepBaseline) {
          topHubRepBaseline = 0; // Handle U2 reboot mid-session
        }
        int sessionReps = (int)(gTopHubTelem.current_rep - topHubRepBaseline);
        if (sessionReps > gSession.completedReps) {
          gSession.completedReps = (sessionReps > gSession.targetReps) ? gSession.targetReps : sessionReps;
        }
        if (gSession.targetReps > 0 && gSession.completedReps >= gSession.targetReps && !repLimitAutoStopped) {
          repLimitAutoStopped   = true;
          gSession.liveTorqueNm = 0.0f;
          Serial.printf("[U2] Target reps (%d) reached -> releasing motor to OP_IDLE\n", gSession.targetReps);
          sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_NORMAL, STEP_ACT_NONE, false, 2);
        }
      }
    }
  }

  if (topHubRepQueue != nullptr) {
    QueuedTopHubRep repItem{};
    while (xQueueReceive(topHubRepQueue, &repItem, 0) == pdTRUE) {
      registerOrRefreshWorkerPeer(u2Peer, repItem.srcMac);
      gLastRepSummary = repItem.pkt;
      gHasRepSummary  = true;
      uint16_t sessRep = (repItem.pkt.rep_index >= topHubRepBaseline)
                           ? (uint16_t)(repItem.pkt.rep_index - topHubRepBaseline)
                           : repItem.pkt.rep_index;
      Serial.printf(
        "[U2 REP #%u] ROM %.1f..%.1f deg | PkExt %.2f Nm@%.1f | PkFlex %.2f Nm@%.1f | Work %.2f J | CoV %.1f%%\n",
        (unsigned)sessRep,
        gLastRepSummary.achieved_rom_min,
        gLastRepSummary.achieved_rom_max,
        gLastRepSummary.peak_torque_ext_nm,
        gLastRepSummary.angle_at_peak_ext,
        gLastRepSummary.peak_torque_flex_nm,
        gLastRepSummary.angle_at_peak_flex,
        gLastRepSummary.rep_work_joules,
        gLastRepSummary.vel_cov_pct
      );
    }
  }
}

void processCyclingQueue() {
  if (cyclingTelemQueue == nullptr) return;
  QueuedCyclingTelem item{};
  while (xQueueReceive(cyclingTelemQueue, &item, 0) == pdTRUE) {
    registerOrRefreshWorkerPeer(u3Peer, item.srcMac);
    bool changed = (item.pkt.current_steps != gCyclingTelem.current_steps) ||
                   (item.pkt.target_steps  != gCyclingTelem.target_steps)  ||
                   ((item.pkt.pedal_state_flags & CYCLE_FLAG_SESSION_ACTIVE) !=
                    (gCyclingTelem.pedal_state_flags & CYCLE_FLAG_SESSION_ACTIVE));
    gCyclingTelem = item.pkt;

    if (changed) {
      Serial.printf(
        "[U3 RX] active=%d steps=%ld/%ld angle=%.2f deg enc2=%d\n",
        (gCyclingTelem.pedal_state_flags & CYCLE_FLAG_SESSION_ACTIVE) ? 1 : 0,
        (long)gCyclingTelem.current_steps,
        (long)gCyclingTelem.target_steps,
        gCyclingTelem.stepper_angle_deg,
        (gCyclingTelem.pedal_state_flags & CYCLE_FLAG_ENC2_ACTIVE) ? 1 : 0
      );
    }

    if (gSession.active && !gSession.paused && !gSession.estopped &&
        isCyclingMode(gSession.opMode, gSession.modeName) &&
        (gCyclingTelem.pedal_state_flags & CYCLE_FLAG_ENC2_ACTIVE) != 0) {
      gSession.rpm          = gCyclingTelem.pedal_rpm;
      gSession.speedKmh     = gSession.rpm * 0.31f;
      gSession.liveTorqueNm = fmaxf(gCyclingTelem.eddy_brake_torque, gCyclingTelem.user_pedal_torque);
    }
  }
}

void serviceEspNowHeartbeats() {
  if (!espNowReady) return;
  uint8_t ch = currentMasterChannel();
  if (ch != lastAnnouncedChannel) {
    lastAnnouncedChannel = ch;
    Serial.printf("[ESP-NOW] Active Wi-Fi channel=%u\n", (unsigned)ch);
  }
  uint32_t now = millis();
  expireWorkerPeerIfStale(u2Peer, now);
  expireWorkerPeerIfStale(u3Peer, now);

  if (now - lastEspNowHeartbeatMs >= ESPNOW_HEARTBEAT_MS) {
    lastEspNowHeartbeatMs = now;
    syncAllNodesImmediate(1);
  }
}

void sendTelemetryFrame() {
  if (webSocket.connectedClients() == 0) return;

  uint8_t flags       = u2Peer.linked ? gTopHubTelem.state_flags : 0;
  int     homed       = (flags & STATE_FLAG_HOMED) ? 1 : 0;
  int     calibrated  = (flags & STATE_FLAG_CALIBRATED) ? 1 : 0;
  int     spasm       = (flags & STATE_FLAG_SPASM_STOP) ? 1 : 0;
  int     hardLock    = (flags & STATE_FLAG_HARDSTOP_LOCKED) ? 1 : 0;
  int     busyHoming  = ((flags & STATE_FLAG_BUSY_HOMING) || pendingAutoHome || manualSetupOp == OP_SETUP_FULL_25S) ? 1 : 0;
  int     positioning = (flags & STATE_FLAG_POSITIONING) ? 1 : 0;
  int     faultLock   = (flags & STATE_FLAG_FAULT_LOCKOUT) ? 1 : 0;
  int     setupWarn   = isSetupMotionWarningActive(flags) ? 1 : 0;
  const char* setupStateStr = setupPhaseName(setupPhase);
  if (setupPhase == SETUP_NONE && (flags & STATE_FLAG_BUSY_HOMING)) {
    setupStateStr = "autohoming";
  } else if (setupPhase == SETUP_NONE && positioning) {
    setupStateStr = "moving_to_start";
  }

  char buf[1024];
  if (isCyclingMode(gSession.opMode, gSession.modeName)) {
    bool enc2On = u3Peer.linked && ((gCyclingTelem.pedal_state_flags & CYCLE_FLAG_ENC2_ACTIVE) != 0);
    snprintf(
      buf,
      sizeof(buf),
      "{\"type\":\"telemetry\",\"mode\":\"cycling\",\"op_mode\":%u,\"active\":%d,"
      "\"rpm\":%.1f,\"speed\":%.1f,\"torque\":%.1f,\"target_torque\":%.1f,"
      "\"stepper_angle_deg\":%.2f,\"stepper_steps\":%ld,"
      "\"u3_linked\":%d,\"enc2_active\":%d,\"user_pedal_torque\":%.2f,\"user_power_watts\":%.1f,"
      "\"u2_linked\":%d,\"homed\":%d,\"calibrated\":%d,\"spasm\":%d,\"hardstop_locked\":%d,"
      "\"busy_homing\":%d,\"positioning\":%d,\"fault_lockout\":%d,"
      "\"setup_state\":\"%s\",\"setup_warning\":%d,\"setup_target_deg\":%.1f,"
      "\"knee_angle_deg\":%.1f,\"knee_vel_deg_s\":%.1f,"
      "\"motor_current_a\":%.2f,\"vbus_v\":%.2f,\"rom_min_deg\":%.1f,\"rom_max_deg\":%.1f,"
      "\"target_vel_deg_s\":%.1f,\"last_rep_work_j\":%.2f,"
      "\"reps\":%d,\"estop\":%d,\"paused\":%d,\"uptime_ms\":%lu}",
      (unsigned)gSession.opMode,
      gSession.active ? 1 : 0,
      gSession.rpm,
      gSession.speedKmh,
      gSession.liveTorqueNm,
      gSession.targetTorqueNm,
      gCyclingTelem.stepper_angle_deg,
      (long)gCyclingTelem.current_steps,
      u3Peer.linked ? 1 : 0,
      enc2On ? 1 : 0,
      gCyclingTelem.user_pedal_torque,
      gCyclingTelem.user_power_watts,
      u2Peer.linked ? 1 : 0,
      homed,
      calibrated,
      spasm,
      hardLock,
      busyHoming,
      positioning,
      faultLock,
      setupStateStr,
      setupWarn,
      setupTargetDeg,
      gSession.kneeAngleDeg,
      u2Peer.linked ? gTopHubTelem.knee_vel_deg_s : 0.0f,
      u2Peer.linked ? gTopHubTelem.motor_current_amps : 0.0f,
      u2Peer.linked ? gTopHubTelem.vbus_volts : 0.0f,
      gSession.romMinDeg,
      gSession.romMaxDeg,
      gSession.targetVelDegS,
      gHasRepSummary ? gLastRepSummary.rep_work_joules : 0.0f,
      gSession.completedReps,
      gSession.estopped ? 1 : 0,
      gSession.paused ? 1 : 0,
      (unsigned long)millis()
    );
  } else {
    bool isSupport = (gSession.opMode == OP_CPM || gSession.opMode == OP_ACTIVE_ASSIST || strcmp(gSession.role, "support") == 0);
    int remaining = gSession.targetReps - gSession.completedReps;
    if (remaining < 0) remaining = 0;

    snprintf(
      buf,
      sizeof(buf),
      "{\"type\":\"telemetry\",\"mode\":\"%s\",\"op_mode\":%u,\"active\":%d,"
      "\"support_torque\":%.1f,\"resistance_torque\":%.1f,\"torque\":%.1f,"
      "\"target_torque\":%.1f,\"target_torque_flex\":%.1f,\"target_curve_nm\":%.1f,"
      "\"knee_angle_deg\":%.1f,\"rom_min_deg\":%.1f,\"rom_max_deg\":%.1f,\"target_vel_deg_s\":%.1f,"
      "\"completed_reps\":%d,\"target_reps\":%d,\"reps_remaining\":%d,\"estop\":%d,\"paused\":%d,"
      "\"u2_linked\":%d,\"homed\":%d,\"calibrated\":%d,\"spasm\":%d,\"hardstop_locked\":%d,"
      "\"busy_homing\":%d,\"positioning\":%d,\"fault_lockout\":%d,\"state_flags\":%u,"
      "\"setup_state\":\"%s\",\"setup_warning\":%d,\"setup_target_deg\":%.1f,"
      "\"knee_vel_deg_s\":%.1f,\"pad_force_n\":%.1f,"
      "\"motor_current_a\":%.2f,\"vbus_v\":%.2f,\"chopper_pct\":%.1f,"
      "\"last_rep_work_j\":%.2f,\"last_rep_peak_ext_nm\":%.2f,\"last_rep_peak_flex_nm\":%.2f,"
      "\"uptime_ms\":%lu}",
      gSession.modeName,
      (unsigned)gSession.opMode,
      gSession.active ? 1 : 0,
      isSupport ? gSession.liveTorqueNm : 0.0f,
      isSupport ? 0.0f : gSession.liveTorqueNm,
      gSession.liveTorqueNm,
      gSession.targetTorqueNm,
      gSession.targetTorqueFlexNm,
      u2Peer.linked ? gTopHubTelem.target_curve_nm : gSession.targetTorqueNm,
      gSession.kneeAngleDeg,
      gSession.romMinDeg,
      gSession.romMaxDeg,
      gSession.targetVelDegS,
      gSession.completedReps,
      gSession.targetReps,
      remaining,
      gSession.estopped ? 1 : 0,
      gSession.paused ? 1 : 0,
      u2Peer.linked ? 1 : 0,
      homed,
      calibrated,
      spasm,
      hardLock,
      busyHoming,
      positioning,
      faultLock,
      (unsigned)flags,
      setupStateStr,
      setupWarn,
      setupTargetDeg,
      u2Peer.linked ? gTopHubTelem.knee_vel_deg_s : 0.0f,
      u2Peer.linked ? gTopHubTelem.pad_force_newtons : 0.0f,
      u2Peer.linked ? gTopHubTelem.motor_current_amps : 0.0f,
      u2Peer.linked ? gTopHubTelem.vbus_volts : 0.0f,
      u2Peer.linked ? gTopHubTelem.chopper_duty_pct : 0.0f,
      gHasRepSummary ? gLastRepSummary.rep_work_joules : 0.0f,
      gHasRepSummary ? gLastRepSummary.peak_torque_ext_nm : 0.0f,
      gHasRepSummary ? gLastRepSummary.peak_torque_flex_nm : 0.0f,
      (unsigned long)millis()
    );
  }
  webSocket.broadcastTXT(buf);
}

void logCommandState(const char* eventName) {
  Serial.printf(
    "[CMD:%s] active=%d paused=%d estop=%d mode=%s op=%u target=%.1f Nm reps=%d/%d | U2:%s U3:%s(%.2f deg)\n",
    eventName,
    gSession.active ? 1 : 0,
    gSession.paused ? 1 : 0,
    gSession.estopped ? 1 : 0,
    gSession.modeName,
    (unsigned)gSession.opMode,
    gSession.targetTorqueNm,
    gSession.completedReps,
    gSession.targetReps,
    u2Peer.linked ? "LNK" : "SCAN",
    u3Peer.linked ? "LNK" : "SCAN",
    gCyclingTelem.stepper_angle_deg
  );
}

// ============================================================================
// 5. INCOMING WEBSITE COMMAND HANDLER
// ============================================================================
void startSessionCommon(uint8_t targetOpMode) {
  gSession.opMode        = targetOpMode;
  gSession.active        = true;
  gSession.paused        = false;
  gSession.estopped      = false;
  gSession.completedReps = 0;
  gSession.cycleProgress = 0.0f;
  gSession.activeMs      = 0;
  repLimitAutoStopped    = false;
  u2FaultNotified        = false;
  pendingAutoHome        = false;
  manualSetupOp          = OP_IDLE;
  setupPhase             = SETUP_NONE;
  sawU2Positioning       = false;

  if (isCyclingMode(gSession.opMode, gSession.modeName)) {
    gSession.rpm          = 66.0f;
    gSession.speedKmh     = gSession.rpm * 0.31f;
    gSession.liveTorqueNm = gSession.targetTorqueNm;
    syncAllNodesImmediate(2);
    return;
  }

  // Flexion / Extension on U2
  topHubRepBaseline     = u2Peer.linked ? gTopHubTelem.current_rep : 0;
  gSession.kneeAngleDeg = u2Peer.linked ? gTopHubTelem.knee_angle_deg : gSession.romMinDeg;
  gSession.liveTorqueNm = u2Peer.linked ? fabsf(gTopHubTelem.knee_torque_nm) : (gSession.targetTorqueNm * 0.5f);

  if (u2Peer.linked) {
    bool u2Homed = (gTopHubTelem.state_flags & STATE_FLAG_HOMED) != 0;
    if (!u2Homed) {
#if AUTO_HOME_U2_ON_START
      pendingAutoHome   = true;
      postHomeOpMode    = gSession.opMode;
      autoHomeStartedMs = millis();
      sawU2BusyHoming   = false;
      Serial.println("[U2] Unhomed on start — running OP_SETUP_QUICK_6S Auto-Home first...");
      sendWsAck(true, "Homing Top-Hub to 0 deg before starting therapy...", "warning");
      syncAllNodesImmediate(2);
#else
      gSession.active = false;
      gSession.paused = true;
      setupPhase      = SETUP_NONE;
      Serial.println("[U2] Blocked start: Top-Hub is unhomed (H:0) -> require Set 0 deg Home or Auto-Home");
      sendWsAck(true, "Top-Hub is unhomed (H:0). Click Set 0 deg Home (if at 0 deg stop) or Auto-Home (STOP1) first.", "warning");
      syncAllNodesImmediate(2);
#endif
      return;
    }

    if ((gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT | STATE_FLAG_HARDSTOP_LOCKED)) != 0) {
      sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
      delay(6);
    }

    if (gTopHubTelem.knee_angle_deg < (gSession.romMinDeg - 4.0f) ||
        gTopHubTelem.knee_angle_deg > (gSession.romMaxDeg + 4.0f)) {
      gSession.active  = true;
      gSession.paused  = true;
      setupPhase       = SETUP_MOVING_TO_START;
      setupTargetDeg   = gSession.romMinDeg;
      setupStartedMs   = millis();
      sawU2Positioning = false;
      Serial.printf("[U2] Arm at %.1f deg -> slow setup move to %.1f deg\n",
                    gTopHubTelem.knee_angle_deg, setupTargetDeg);
      sendWsAck(true, "Setup in progress — moving arm slowly to starting position. Please stay away!", "warning");
      syncAllNodesImmediate(2);
      return;
    }
  }

  syncAllNodesImmediate(2);
}

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

    uint8_t nextOp = OP_CYCLE_TORQUE;
    float opVal = 0.0f;
    if (jsonGetFloat(payload, "op_mode", opVal)) {
      nextOp = (uint8_t)lroundf(opVal);
    } else if (strcmp(gSession.modeName, "passive") == 0) {
      nextOp = OP_CPM;
    } else if (strcmp(gSession.modeName, "isotonic") == 0) {
      nextOp = OP_ISOTONIC;
    } else if (strcmp(gSession.modeName, "isokinetic") == 0) {
      nextOp = OP_ISOKINETIC;
    } else if (strcmp(gSession.modeName, "isometric") == 0) {
      nextOp = OP_ISOMETRIC;
    } else if (strcmp(gSession.modeName, "active_assist") == 0) {
      nextOp = OP_ACTIVE_ASSIST;
    }

    float tqVal = 0.0f;
    if (jsonGetFloat(payload, "torque_nm", tqVal) || jsonGetFloat(payload, "target_torque_ext", tqVal)) {
      gSession.targetTorqueNm = clampf(tqVal, 0.0f, maxAllowedTorque(nextOp, gSession.role));
    }
    float tqFlexVal = 0.0f;
    if (jsonGetFloat(payload, "target_torque_flex", tqFlexVal)) {
      gSession.targetTorqueFlexNm = clampf(tqFlexVal, 0.0f, 45.0f);
    } else {
      gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 45.0f);
    }

    float velVal = 0.0f;
    bool hasVel = jsonGetFloat(payload, "target_vel_deg_s", velVal) && velVal > 0.0f;
    if (hasVel) {
      gSession.targetVelDegS = velVal;
    } else {
      gSession.targetVelDegS = (nextOp == OP_CPM) ? 18.0f : (nextOp == OP_ISOKINETIC ? 45.0f : (nextOp == OP_ISOMETRIC ? 150.0f : 60.0f));
    }

    float repsVal = 0.0f;
    if (jsonGetFloat(payload, "target_reps", repsVal) && repsVal >= 1.0f) {
      gSession.targetReps = (int)clampf(repsVal, 1.0f, 99.0f);
    }

    float rMin = 120.0f, rMax = 180.0f;
    bool hasRMin = jsonGetFloat(payload, "rom_min_deg", rMin);
    bool hasRMax = jsonGetFloat(payload, "rom_max_deg", rMax);
    if (!isCyclingMode(nextOp, gSession.modeName)) {
      gSession.romMinDeg = clampf(hasRMin ? rMin : gSession.romMinDeg, 120.0f, 170.0f);
      gSession.romMaxDeg = clampf(hasRMax ? rMax : gSession.romMaxDeg, gSession.romMinDeg + 10.0f, 180.0f);
      if (nextOp == OP_ISOMETRIC) {
        if (!hasVel) gSession.targetVelDegS = 150.0f;
        gSession.targetVelDegS = clampf(gSession.targetVelDegS, gSession.romMinDeg, gSession.romMaxDeg);
      } else {
        gSession.targetVelDegS = clampf(gSession.targetVelDegS, 3.0f, 180.0f);
      }
    } else {
      if (hasRMin) gSession.romMinDeg = clampf(rMin, 0.0f, 170.0f);
      if (hasRMax) gSession.romMaxDeg = clampf(rMax, gSession.romMinDeg + 10.0f, 180.0f);
    }

    float dOff = 0.0f, lLeg = 0.0f;
    if (jsonGetFloat(payload, "L_leg_m", lLeg) && lLeg >= 0.15f && lLeg <= 0.60f) {
      gSession.lLegM = lLeg;
    }
    if (jsonGetFloat(payload, "d_offset_m", dOff) && dOff >= 0.0f && dOff <= 0.95f * gSession.lLegM) {
      gSession.dOffsetM = dOff;
    }

    float pathNum = 0.0f;
    String pathStr;
    if (jsonGetFloat(payload, "pathology_profile", pathNum)) {
      uint8_t p = (uint8_t)lroundf(pathNum);
      gSession.pathology = (p <= PATH_PFPS) ? p : (uint8_t)PATH_STANDARD;
    } else if (jsonGetString(payload, "pathology", pathStr)) {
      gSession.pathology = parsePathologyString(pathStr);
    } else {
      gSession.pathology = PATH_STANDARD;
    }

    startSessionCommon(nextOp);
    logCommandState("start_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "set_torque" || cmdType == "set_params") {
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
    } else if (strcmp(gSession.modeName, "isometric") == 0) {
      gSession.opMode = OP_ISOMETRIC;
    } else if (strcmp(gSession.modeName, "active_assist") == 0) {
      gSession.opMode = OP_ACTIVE_ASSIST;
    }

    float rMin = 120.0f, rMax = 180.0f;
    bool hasRMin = jsonGetFloat(payload, "rom_min_deg", rMin);
    bool hasRMax = jsonGetFloat(payload, "rom_max_deg", rMax);
    if (!isCyclingMode(gSession.opMode, gSession.modeName)) {
      gSession.romMinDeg = clampf(hasRMin ? rMin : gSession.romMinDeg, 120.0f, 170.0f);
      gSession.romMaxDeg = clampf(hasRMax ? rMax : gSession.romMaxDeg, gSession.romMinDeg + 10.0f, 180.0f);
    } else {
      if (hasRMin) gSession.romMinDeg = clampf(rMin, 0.0f, 170.0f);
      if (hasRMax) gSession.romMaxDeg = clampf(rMax, gSession.romMinDeg + 10.0f, 180.0f);
    }
    float velVal = 0.0f;
    if (jsonGetFloat(payload, "target_vel_deg_s", velVal) && velVal > 0.0f) {
      gSession.targetVelDegS = clampf(velVal, 3.0f, 180.0f);
    }
    if (!isCyclingMode(gSession.opMode, gSession.modeName) && gSession.opMode == OP_ISOMETRIC) {
      gSession.targetVelDegS = clampf(gSession.targetVelDegS, gSession.romMinDeg, gSession.romMaxDeg);
    }
    float repsVal = 0.0f;
    if (jsonGetFloat(payload, "target_reps", repsVal) && repsVal >= 1.0f) {
      gSession.targetReps = (int)clampf(repsVal, 1.0f, 99.0f);
    }
    float dOff = 0.0f, lLeg = 0.0f;
    if (jsonGetFloat(payload, "L_leg_m", lLeg) && lLeg >= 0.15f && lLeg <= 0.60f) {
      gSession.lLegM = lLeg;
    }
    if (jsonGetFloat(payload, "d_offset_m", dOff) && dOff >= 0.0f && dOff <= 0.95f * gSession.lLegM) {
      gSession.dOffsetM = dOff;
    }
    float pathNum = 0.0f;
    String pathStr;
    if (jsonGetFloat(payload, "pathology_profile", pathNum)) {
      uint8_t p = (uint8_t)lroundf(pathNum);
      gSession.pathology = (p <= PATH_PFPS) ? p : (uint8_t)PATH_STANDARD;
    } else if (jsonGetString(payload, "pathology", pathStr)) {
      gSession.pathology = parsePathologyString(pathStr);
    }

    float prevTq = gSession.targetTorqueNm;
    float nextTq = gSession.targetTorqueNm;
    if (jsonGetFloat(payload, "torque_nm", nextTq) || jsonGetFloat(payload, "target_torque_ext", nextTq)) {
      gSession.targetTorqueNm = clampf(nextTq, 0.0f, maxAllowedTorque(gSession.opMode, gSession.role));
    }
    float tqFlexVal = 0.0f;
    if (jsonGetFloat(payload, "target_torque_flex", tqFlexVal)) {
      gSession.targetTorqueFlexNm = clampf(tqFlexVal, 0.0f, 45.0f);
    } else {
      gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 45.0f);
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
    } else {
      sendRehabCommand(NODE_TOPHUB, desiredTopHubOpMode(),
                       gSession.estopped ? ESTOP_HARD_STOP_LOCKED : ESTOP_NORMAL,
                       STEP_ACT_NONE, false, 1);
    }
    logCommandState(cmdType.c_str());
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "begin_exercise") {
    if (gSession.estopped) {
      sendWsAck(true, "Hard-Stop is engaged — release Hard-Stop before beginning exercise.", "warning");
      sendTelemetryFrame();
      return;
    }
    bool readyByPos = (setupPhase == SETUP_READY_AT_START) ||
                      (gSession.active && gSession.paused && setupPhase == SETUP_NONE &&
                       (!u2Peer.linked ||
                        (gTopHubTelem.knee_angle_deg >= gSession.romMinDeg - 5.0f &&
                         gTopHubTelem.knee_angle_deg <= gSession.romMaxDeg + 5.0f)));
    if (readyByPos) {
      setupPhase             = SETUP_NONE;
      sawU2Positioning       = false;
      gSession.active        = true;
      gSession.paused        = false;
      gSession.completedReps = 0;
      gSession.cycleProgress = 0.0f;
      gSession.activeMs      = 0;
      topHubRepBaseline      = u2Peer.linked ? gTopHubTelem.current_rep : 0;
      repLimitAutoStopped    = false;
      u2FaultNotified        = false;
      syncAllNodesImmediate(2);
      sendWsAck(true, "Exercise started!", "info");
      logCommandState("begin_exercise");
    } else {
      sendWsAck(true, "Arm is not yet at starting position — wait for setup positioning to complete.", "warning");
    }
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "park_home") {
    if (gSession.estopped) {
      sendWsAck(true, "Release Hard-Stop before returning arm to 0 deg Park.", "warning");
      sendTelemetryFrame();
      return;
    }
    if (u2Peer.linked && !(gTopHubTelem.state_flags & STATE_FLAG_HOMED)) {
      sendWsAck(true, "Top-Hub is unhomed (H:0) — use Auto-Home (STOP1) to seek 0 deg.", "warning");
      sendTelemetryFrame();
      return;
    }
    if (u2Peer.linked &&
        (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT | STATE_FLAG_HARDSTOP_LOCKED)) != 0) {
      sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
      delay(6);
    }
    gSession.active       = false;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    pendingAutoHome       = false;
    manualSetupOp         = OP_IDLE;
    setupPhase            = SETUP_PARKING_HOME;
    setupTargetDeg        = 0.0f;
    setupStartedMs        = millis();
    sawU2Positioning      = false;
    syncAllNodesImmediate(2);
    sendWsAck(true, "Returning arm slowly to 0 deg Park — please stay away!", "warning");
    logCommandState("park_home");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "pause_session") {
    gSession.paused = true;
    syncAllNodesImmediate(1);
    logCommandState("pause_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "resume_session") {
    if (setupPhase == SETUP_MOVING_TO_START || setupPhase == SETUP_PARKING_HOME || setupPhase == SETUP_AUTOHOME) {
      sendWsAck(true, "Setup movement in progress — please stay away and wait until complete.", "warning");
      sendTelemetryFrame();
      return;
    }
    if (setupPhase == SETUP_READY_AT_START) {
      sendWsAck(true, "Arm is waiting at start — click Begin Exercise once leg is secured.", "info");
      sendTelemetryFrame();
      return;
    }
    if (gSession.estopped) {
      triggerHardStopReleaseBurst();
      delay(6);
    } else if (u2Peer.linked &&
               (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT | STATE_FLAG_HARDSTOP_LOCKED)) != 0) {
      sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
      delay(6);
    }
    gSession.paused = false;
    if (!gSession.active) gSession.active = true;
    u2FaultNotified = false;
    syncAllNodesImmediate(2);
    logCommandState("resume_session");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "stop_session") {
    gSession.active       = false;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    gSession.rpm          = 0.0f;
    gSession.speedKmh     = 0.0f;
    pendingAutoHome       = false;
    manualSetupOp         = OP_IDLE;
    setupPhase            = SETUP_NONE;
    sawU2Positioning      = false;
    if (u2Peer.linked &&
        (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT)) != 0) {
      sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
      delay(4);
    }
    syncAllNodesImmediate(2);
    logCommandState("stop_session");
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
      pendingAutoHome       = false;
      manualSetupOp         = OP_IDLE;
      setupPhase            = SETUP_NONE;
      sawU2Positioning      = false;
      syncAllNodesImmediate(3); // 3-packet burst for <3ms E-Stop
    } else {
      triggerHardStopReleaseBurst();
    }
    logCommandState("estop");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "hardstop_release") {
    triggerHardStopReleaseBurst();
    sendWsAck(true, "Hard-Stop released — Top-Hub & Cycling unlocked to OP_IDLE.");
    logCommandState("hardstop_release");
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "setup_home" || cmdType == "setup_full") {
    if (gSession.estopped) {
      triggerHardStopReleaseBurst();
      delay(4);
    }
    if (gSession.active && !gSession.paused) {
      gSession.paused = true;
    }
    bool isFull         = (cmdType == "setup_full");
    manualSetupOp       = isFull ? OP_SETUP_FULL_25S : OP_SETUP_QUICK_6S;
    manualSetupStartMs  = millis();
    sawManualBusyHoming = false;
    setupPhase          = isFull ? SETUP_AUTOHOME : SETUP_NONE;
    setupTargetDeg      = 0.0f;
    setupStartedMs      = millis();
    sawU2Positioning    = false;
    syncAllNodesImmediate(2);
    sendWsAck(true, isFull
      ? "Setup in progress — Auto-Homing arm toward 0 deg STOP1. Please stay away!"
      : "Latching 0.0° Home & ACS712 zero on Top-Hub...",
      isFull ? "warning" : "info");
    logCommandState(cmdType.c_str());
    sendTelemetryFrame();
    return;
  }

  if (cmdType == "bench_test") {
    if (!gSession.active || gSession.paused) {
      manualSetupOp       = OP_AUTO_BENCH_TEST;
      manualSetupStartMs  = millis();
      sawManualBusyHoming = false;
      syncAllNodesImmediate(2);
      sendWsAck(true, "Started Top-Hub 10-pulse auto bench test.");
      logCommandState("bench_test");
      sendTelemetryFrame();
    }
    return;
  }
}

void onWebSocketEvent(uint8_t clientNum, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress remoteIp = webSocket.remoteIP(clientNum);
      Serial.printf("[WS #%u] Client connected from %s\n", clientNum, remoteIp.toString().c_str());
      sendTelemetryFrame();
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[WS #%u] Client disconnected\n", clientNum);
      if (webSocket.connectedClients() == 0 && (gSession.active || setupPhase != SETUP_NONE)) {
        gSession.paused       = true;
        gSession.liveTorqueNm = 0.0f;
        pendingAutoHome       = false;
        manualSetupOp         = OP_IDLE;
        setupPhase            = SETUP_NONE;
        sawU2Positioning      = false;
        syncAllNodesImmediate(2);
        logCommandState("ws_all_clients_disconnected_pause");
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
// 6. PER-NODE TELEMETRY / SIMULATION ARBITRATION
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

  if (isCyclingMode(gSession.opMode, gSession.modeName)) {
    bool useLiveEnc2 = u3Peer.linked && ((gCyclingTelem.pedal_state_flags & CYCLE_FLAG_ENC2_ACTIVE) != 0);
    if (useLiveEnc2) {
      gSession.cycleProgress += dtSec * (gSession.rpm / 180.0f);
      if (gSession.cycleProgress >= 1.0f) {
        gSession.cycleProgress -= 1.0f;
        gSession.completedReps += 1;
      }
      return;
    }

    bool allowCyclingSim = (!u3Peer.linked && BENCH_SIMULATION_FALLBACK) ||
                           (u3Peer.linked && CYCLING_RPM_FALLBACK_NO_ENC2);
    if (!allowCyclingSim) {
      gSession.rpm          = 0.0f;
      gSession.speedKmh     = 0.0f;
      gSession.liveTorqueNm = 0.0f;
      return;
    }

    float t = (float)gSession.activeMs * 0.001f;
    float baseRpm = clampf(78.0f - (gSession.targetTorqueNm * 0.25f), 42.0f, 92.0f);
    gSession.rpm = clampf(baseRpm + 3.5f * sinf(t * 1.1f), 25.0f, 120.0f);
    gSession.speedKmh = gSession.rpm * 0.31f;

    float pedalRipple = 1.6f * sinf(t * (gSession.rpm / 60.0f) * 2.0f * PI);
    float desiredTorque = clampf(gSession.targetTorqueNm + pedalRipple, 0.0f, 85.0f);
    gSession.liveTorqueNm += (desiredTorque - gSession.liveTorqueNm) * clampf(dtSec * 6.0f, 0.0f, 1.0f);

    gSession.cycleProgress += dtSec * (gSession.rpm / 180.0f);
    if (gSession.cycleProgress >= 1.0f) {
      gSession.cycleProgress -= 1.0f;
      gSession.completedReps += 1;
    }
    return;
  }

  // Flexion / Extension: real U2 telemetry overrides simulation whenever U2 is linked
  if (u2Peer.linked) {
    return;
  }

#if BENCH_SIMULATION_FALLBACK
  if (gSession.completedReps >= gSession.targetReps) {
    gSession.liveTorqueNm = 0.0f;
    gSession.kneeAngleDeg = gSession.romMinDeg;
    return;
  }

  gSession.cycleProgress += dtSec / 3.6f;
  if (gSession.cycleProgress >= 1.0f) {
    gSession.cycleProgress -= 1.0f;
    if (gSession.completedReps < gSession.targetReps) {
      gSession.completedReps += 1;
    }
  }

  float span = gSession.romMaxDeg - gSession.romMinDeg;
  float normAngle = 0.5f * (1.0f - cosf(gSession.cycleProgress * 2.0f * PI));
  gSession.kneeAngleDeg = gSession.romMinDeg + span * normAngle;

  float maxTq = maxAllowedTorque(gSession.opMode, gSession.role);
  float curveGain = (gSession.opMode == OP_CPM)
    ? (0.85f + 0.15f * normAngle)
    : (0.65f + 0.35f * sinf(normAngle * PI));
  float desiredTorque = clampf(gSession.targetTorqueNm * curveGain, 0.0f, maxTq);
  gSession.liveTorqueNm += (desiredTorque - gSession.liveTorqueNm) * clampf(dtSec * 8.0f, 0.0f, 1.0f);
#else
  gSession.liveTorqueNm = 0.0f;
#endif
}

// ============================================================================
// 7. USB SERIAL BENCH CONSOLE + WIFI CONNECTION + MAIN LOOP
// ============================================================================
void printSerialHelp() {
  Serial.println("\n--- U1 Master USB Serial Commands (115200 baud) ---");
  Serial.println("  STATUS / IP / NODES        : Print Wi-Fi IP, WS URL, U2/U3 ESP-NOW link & telemetry");
  Serial.println("  STREAM / P                 : Toggle 5 Hz live Top-Hub U2 telemetry stream on Serial");
  Serial.println("  HOME / ZERO                : Latch 0.0 deg Home & ACS712 zero on Top-Hub U2 (OP_SETUP_QUICK_6S)");
  Serial.println("  AUTOHOME / CAL / SETUP     : Motorized seek to STOP1 hard-stop on Top-Hub U2 (OP_SETUP_FULL_25S)");
  Serial.println("  CPM [deg_s] [Nm]           : Start Passive Assist (OP_CPM=3, e.g. 'CPM 18 12')");
  Serial.println("  ASSIST [deg_s] [Nm]        : Start Active Assist (OP_ACTIVE_ASSIST=11, e.g. 'ASSIST 25 12')");
  Serial.println("  ISO [ext_Nm] [flex_Nm]     : Start Isotonic Mode (OP_ISOTONIC=4, e.g. 'ISO 18 12')");
  Serial.println("  ISOK [deg_s] [cap_Nm]      : Start Isokinetic Mode (OP_ISOKINETIC=5, e.g. 'ISOK 45 25')");
  Serial.println("  ISOM [hold_deg] [cap_Nm]   : Start Isometric Hold (OP_ISOMETRIC=6, e.g. 'ISOM 150 20')");
  Serial.println("  ROM <min_deg> <max_deg>    : Set Flexion ROM window (e.g. 'ROM 120 180')");
  Serial.println("  BEGIN | PARK               : Begin exercise after 120 deg setup or slow-park to 0 deg");
  Serial.println("  SPEED <deg_s>              : Set target velocity deg/s (e.g. 'SPEED 25')");
  Serial.println("  TORQUE <ext_Nm> [flex_Nm]  : Set target torque live (e.g. 'TORQUE 16 10')");
  Serial.println("  PWM <signed_pct>           : Direct PWM test on U2 (-60..+60, 0 to stop)");
  Serial.println("  BENCH                      : Run 10-pulse auto bench test on U2");
  Serial.println("  PAUSE | RESUME | STOP/END  : Pause, resume, or stop active session");
  Serial.println("  ESTOP / LOCK               : Engage electronic Hard-Stop on U2 & unload U3");
  Serial.println("  RELEASE / CLEAR            : Release Hard-Stop lock on U2 & U3 -> OP_IDLE");
  Serial.println("  START | + | -              : Start Cycling session or step U3 stepper +/-7.5 deg\n");
}

void printNodesStatus() {
  Serial.printf("[NODES] Master BootID=0x%08lX | WiFi Ch=%u | WS Clients=%u | RX_bad=%lu\n",
                (unsigned long)masterBootId,
                (unsigned)currentMasterChannel(),
                (unsigned)webSocket.connectedClients(),
                (unsigned long)espNowRxBad);
  Serial.printf("  U2 TopHub  : %s", u2Peer.linked ? "LINKED" : "SCANNING");
  if (u2Peer.linked) {
    uint8_t fl = gTopHubTelem.state_flags;
    Serial.printf(" (%02X:%02X:%02X:%02X:%02X:%02X, rx=%lu, H=%d CAL=%d LCK=%d SPM=%d, Tk=%.2f deg, Wk=%.1f deg/s, Tq=%.2f Nm, Im=%.2f A, Vbus=%.2f V, Rep=%u)",
                  u2Peer.mac[0], u2Peer.mac[1], u2Peer.mac[2],
                  u2Peer.mac[3], u2Peer.mac[4], u2Peer.mac[5],
                  (unsigned long)u2Peer.rxCount,
                  (fl & STATE_FLAG_HOMED) ? 1 : 0,
                  (fl & STATE_FLAG_CALIBRATED) ? 1 : 0,
                  (fl & STATE_FLAG_HARDSTOP_LOCKED) ? 1 : 0,
                  (fl & STATE_FLAG_SPASM_STOP) ? 1 : 0,
                  gTopHubTelem.knee_angle_deg,
                  gTopHubTelem.knee_vel_deg_s,
                  gTopHubTelem.knee_torque_nm,
                  gTopHubTelem.motor_current_amps,
                  gTopHubTelem.vbus_volts,
                  (unsigned)gTopHubTelem.current_rep);
  }
  Serial.println();
  Serial.printf("  U3 Cycling : %s", u3Peer.linked ? "LINKED" : "SCANNING");
  if (u3Peer.linked) {
    Serial.printf(" (%02X:%02X:%02X:%02X:%02X:%02X, rx=%lu, steps=%ld/%ld, angle=%.2f deg, RPM=%.1f)",
                  u3Peer.mac[0], u3Peer.mac[1], u3Peer.mac[2],
                  u3Peer.mac[3], u3Peer.mac[4], u3Peer.mac[5],
                  (unsigned long)u3Peer.rxCount,
                  (long)gCyclingTelem.current_steps,
                  (long)gCyclingTelem.target_steps,
                  gCyclingTelem.stepper_angle_deg,
                  gCyclingTelem.pedal_rpm);
  }
  Serial.println();
}

void parseTwoFloats(const String& args, float& outA, bool& hasA, float& outB, bool& hasB) {
  String s = args;
  s.trim();
  hasA = false;
  hasB = false;
  if (s.length() == 0) return;
  int sp = s.indexOf(' ');
  if (sp < 0) {
    outA = s.toFloat();
    hasA = true;
    return;
  }
  outA = s.substring(0, sp).toFloat();
  hasA = true;
  String s2 = s.substring(sp + 1);
  s2.trim();
  if (s2.length() > 0) {
    outB = s2.toFloat();
    hasB = true;
  }
}

void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  String upper = line;
  upper.toUpperCase();

  if (upper == "HELP" || upper == "?") {
    printSerialHelp();
  } else if (upper == "IP" || upper == "STATUS") {
    Serial.printf("Wi-Fi SSID=%s IP=%s ch=%u WS=ws://%s:%u/\n",
                  WIFI_SSID,
                  WiFi.localIP().toString().c_str(),
                  (unsigned)currentMasterChannel(),
                  WiFi.localIP().toString().c_str(),
                  (unsigned)WS_PORT);
    printNodesStatus();
    logCommandState("serial_status");
  } else if (upper == "NODES") {
    printNodesStatus();
  } else if (upper == "STREAM" || upper == "P") {
    serialStreamTopHub = !serialStreamTopHub;
    Serial.printf("[SERIAL] Top-Hub 5 Hz live telemetry stream: %s\n", serialStreamTopHub ? "ON" : "OFF");
  } else if (upper == "HOME" || upper == "ZERO") {
    if (gSession.estopped) triggerHardStopReleaseBurst();
    manualSetupOp       = OP_SETUP_QUICK_6S;
    manualSetupStartMs  = millis();
    sawManualBusyHoming = false;
    setupPhase          = SETUP_NONE;
    sawU2Positioning    = false;
    syncAllNodesImmediate(2);
    sendTelemetryFrame();
    Serial.println("[ESP-NOW TX] Triggered OP_SETUP_QUICK_6S (0.0 deg Home + ACS712 zero) on U2");
  } else if (upper == "AUTOHOME" || upper == "CAL" || upper == "SETUP") {
    if (gSession.estopped) triggerHardStopReleaseBurst();
    manualSetupOp       = OP_SETUP_FULL_25S;
    manualSetupStartMs  = millis();
    sawManualBusyHoming = false;
    setupPhase          = SETUP_AUTOHOME;
    setupTargetDeg      = 0.0f;
    setupStartedMs      = millis();
    sawU2Positioning    = false;
    syncAllNodesImmediate(2);
    sendTelemetryFrame();
    Serial.println("[ESP-NOW TX] Triggered OP_SETUP_FULL_25S (motorized STOP1 Auto-Home) on U2");
  } else if (upper == "BENCH") {
    manualSetupOp       = OP_AUTO_BENCH_TEST;
    manualSetupStartMs  = millis();
    sawManualBusyHoming = false;
    syncAllNodesImmediate(2);
    sendTelemetryFrame();
    Serial.println("[ESP-NOW TX] Triggered OP_AUTO_BENCH_TEST (10 pulses) on U2");
  } else if (upper == "START") {
    strcpy(gSession.modeName, "cycling");
    strcpy(gSession.role, "resistance");
    startSessionCommon(OP_CYCLE_TORQUE);
    logCommandState("serial_start");
    sendTelemetryFrame();
  } else if (upper == "CPM" || upper.startsWith("CPM ")) {
    float spd = 18.0f, tq = 12.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(3), spd, hasA, tq, hasB);
    strcpy(gSession.modeName, "passive");
    strcpy(gSession.role, "support");
    gSession.targetVelDegS      = clampf(hasA ? spd : 18.0f, 3.0f, 60.0f);
    gSession.targetTorqueNm     = clampf(hasB ? tq : 12.0f, 4.0f, 20.0f);
    gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 20.0f);
    startSessionCommon(OP_CPM);
    logCommandState("serial_cpm");
    sendTelemetryFrame();
  } else if (upper == "ASSIST" || upper.startsWith("ASSIST ")) {
    float spd = 25.0f, tq = 12.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(6), spd, hasA, tq, hasB);
    strcpy(gSession.modeName, "active_assist");
    strcpy(gSession.role, "support");
    gSession.targetVelDegS      = clampf(hasA ? spd : 25.0f, 5.0f, 60.0f);
    gSession.targetTorqueNm     = clampf(hasB ? tq : 12.0f, 4.0f, 20.0f);
    gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 20.0f);
    startSessionCommon(OP_ACTIVE_ASSIST);
    logCommandState("serial_assist");
    sendTelemetryFrame();
  } else if (upper == "ISO" || upper.startsWith("ISO ")) {
    float extNm = 18.0f, flexNm = 12.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(3), extNm, hasA, flexNm, hasB);
    strcpy(gSession.modeName, "isotonic");
    strcpy(gSession.role, "resistance");
    gSession.targetTorqueNm     = clampf(hasA ? extNm : 18.0f, 0.0f, 45.0f);
    gSession.targetTorqueFlexNm = clampf(hasB ? flexNm : (gSession.targetTorqueNm * 0.65f), 0.0f, 45.0f);
    startSessionCommon(OP_ISOTONIC);
    logCommandState("serial_iso");
    sendTelemetryFrame();
  } else if (upper == "ISOK" || upper.startsWith("ISOK ")) {
    float vel = 45.0f, capNm = 25.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(4), vel, hasA, capNm, hasB);
    strcpy(gSession.modeName, "isokinetic");
    strcpy(gSession.role, "resistance");
    gSession.targetVelDegS      = clampf(hasA ? vel : 45.0f, 10.0f, 180.0f);
    gSession.targetTorqueNm     = clampf(hasB ? capNm : 25.0f, 0.0f, 45.0f);
    gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 45.0f);
    startSessionCommon(OP_ISOKINETIC);
    logCommandState("serial_isok");
    sendTelemetryFrame();
  } else if (upper == "ISOM" || upper.startsWith("ISOM ")) {
    float holdDeg = 150.0f, capNm = 20.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(4), holdDeg, hasA, capNm, hasB);
    strcpy(gSession.modeName, "isometric");
    strcpy(gSession.role, "resistance");
    gSession.targetVelDegS      = clampf(hasA ? holdDeg : 150.0f, gSession.romMinDeg, gSession.romMaxDeg);
    gSession.targetTorqueNm     = clampf(hasB ? capNm : 20.0f, 1.0f, 45.0f);
    gSession.targetTorqueFlexNm = gSession.targetTorqueNm;
    startSessionCommon(OP_ISOMETRIC);
    logCommandState("serial_isom");
    sendTelemetryFrame();
  } else if (upper.startsWith("ROM ")) {
    float rMin = 120.0f, rMax = 180.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(4), rMin, hasA, rMax, hasB);
    if (hasA && hasB) {
      gSession.romMinDeg = clampf(rMin, 120.0f, 170.0f);
      gSession.romMaxDeg = clampf(rMax, gSession.romMinDeg + 10.0f, 180.0f);
      if (gSession.opMode == OP_ISOMETRIC) {
        gSession.targetVelDegS = clampf(gSession.targetVelDegS, gSession.romMinDeg, gSession.romMaxDeg);
      }
      syncAllNodesImmediate(2);
      sendTelemetryFrame();
      Serial.printf("[ROM] Updated ROM window -> %.1f .. %.1f deg\n", gSession.romMinDeg, gSession.romMaxDeg);
    }
  } else if (upper.startsWith("SPEED ")) {
    float spd = line.substring(6).toFloat();
    gSession.targetVelDegS = clampf(spd, 3.0f, 180.0f);
    syncAllNodesImmediate(2);
    sendTelemetryFrame();
    Serial.printf("[SPEED] Updated target velocity -> %.1f deg/s\n", gSession.targetVelDegS);
  } else if (upper.startsWith("TORQUE ")) {
    float extNm = 0.0f, flexNm = 0.0f;
    bool hasA = false, hasB = false;
    parseTwoFloats(line.substring(7), extNm, hasA, flexNm, hasB);
    if (hasA) {
      gSession.targetTorqueNm     = clampf(extNm, 0.0f, maxAllowedTorque(gSession.opMode, gSession.role));
      gSession.targetTorqueFlexNm = clampf(hasB ? flexNm : (gSession.targetTorqueNm * 0.65f), 0.0f, 45.0f);
      syncAllNodesImmediate(2);
      sendTelemetryFrame();
      Serial.printf("[TORQUE] Updated target torque -> Ext=%.1f Nm, Flex=%.1f Nm\n",
                    gSession.targetTorqueNm, gSession.targetTorqueFlexNm);
    }
  } else if (upper.startsWith("PWM ")) {
    float pct = clampf(line.substring(4).toFloat(), -60.0f, 60.0f);
    if (fabsf(pct) < 0.5f) {
      gSession.active = false;
      gSession.paused = true;
      syncAllNodesImmediate(2);
      Serial.println("[PWM] Direct PWM stopped -> OP_IDLE");
    } else {
      strcpy(gSession.modeName, "direct_pwm");
      strcpy(gSession.role, "support");
      gSession.opMode        = OP_DIRECT_PWM_TEST;
      gSession.targetVelDegS = pct;
      gSession.active        = true;
      gSession.paused        = false;
      gSession.estopped      = false;
      pendingAutoHome        = false;
      manualSetupOp          = OP_IDLE;
      syncAllNodesImmediate(2);
      Serial.printf("[PWM] Direct PWM test on U2 -> %+.0f%%\n", pct);
    }
    sendTelemetryFrame();
  } else if (upper.startsWith("FLEX ")) {
    String rest = line.substring(5);
    rest.trim();
    int sp = rest.indexOf(' ');
    String mStr = (sp > 0) ? rest.substring(0, sp) : rest;
    float tq = (sp > 0) ? rest.substring(sp + 1).toFloat() : 18.0f;
    uint8_t op = OP_ISOTONIC;
    if (mStr.equalsIgnoreCase("cpm")) {
      op = OP_CPM;
      strcpy(gSession.modeName, "passive");
      strcpy(gSession.role, "support");
    } else if (mStr.equalsIgnoreCase("assist")) {
      op = OP_ACTIVE_ASSIST;
      strcpy(gSession.modeName, "active_assist");
      strcpy(gSession.role, "support");
    } else if (mStr.equalsIgnoreCase("isok")) {
      op = OP_ISOKINETIC;
      strcpy(gSession.modeName, "isokinetic");
      strcpy(gSession.role, "resistance");
    } else if (mStr.equalsIgnoreCase("isom")) {
      op = OP_ISOMETRIC;
      strcpy(gSession.modeName, "isometric");
      strcpy(gSession.role, "resistance");
    } else {
      op = OP_ISOTONIC;
      strcpy(gSession.modeName, "isotonic");
      strcpy(gSession.role, "resistance");
    }
    gSession.targetTorqueNm     = clampf(tq, 0.0f, maxAllowedTorque(op, gSession.role));
    gSession.targetTorqueFlexNm = clampf(gSession.targetTorqueNm * 0.65f, 0.0f, 45.0f);
    startSessionCommon(op);
    logCommandState("serial_flex");
    sendTelemetryFrame();
  } else if (upper == "BEGIN") {
    if (gSession.estopped) {
      Serial.println("[SERIAL] Cannot BEGIN while E-Stop is active. Run RELEASE first.");
    } else if (setupPhase == SETUP_READY_AT_START ||
               (gSession.active && gSession.paused && setupPhase == SETUP_NONE &&
                (!u2Peer.linked ||
                 (gTopHubTelem.knee_angle_deg >= gSession.romMinDeg - 5.0f &&
                  gTopHubTelem.knee_angle_deg <= gSession.romMaxDeg + 5.0f)))) {
      setupPhase             = SETUP_NONE;
      sawU2Positioning       = false;
      gSession.active        = true;
      gSession.paused        = false;
      gSession.completedReps = 0;
      gSession.cycleProgress = 0.0f;
      gSession.activeMs      = 0;
      topHubRepBaseline      = u2Peer.linked ? gTopHubTelem.current_rep : 0;
      repLimitAutoStopped    = false;
      u2FaultNotified        = false;
      syncAllNodesImmediate(2);
      sendWsAck(true, "Exercise started!", "info");
      logCommandState("serial_begin");
      sendTelemetryFrame();
      Serial.println("[SERIAL] Exercise STARTED");
    } else {
      Serial.println("[SERIAL] Arm not ready at start position yet.");
    }
  } else if (upper == "PARK") {
    if (gSession.estopped) {
      Serial.println("[SERIAL] Cannot PARK while E-Stop is active. Run RELEASE first.");
    } else if (u2Peer.linked && !(gTopHubTelem.state_flags & STATE_FLAG_HOMED)) {
      Serial.println("[SERIAL] Top-Hub is unhomed (H:0). Run AUTOHOME first.");
    } else {
      if (u2Peer.linked &&
          (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT | STATE_FLAG_HARDSTOP_LOCKED)) != 0) {
        sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
        delay(6);
      }
      gSession.active       = false;
      gSession.paused       = true;
      gSession.liveTorqueNm = 0.0f;
      pendingAutoHome       = false;
      manualSetupOp         = OP_IDLE;
      setupPhase            = SETUP_PARKING_HOME;
      setupTargetDeg        = 0.0f;
      setupStartedMs        = millis();
      sawU2Positioning      = false;
      syncAllNodesImmediate(2);
      sendWsAck(true, "Returning arm slowly to 0 deg Park — please stay away!", "warning");
      logCommandState("serial_park");
      sendTelemetryFrame();
      Serial.println("[ESP-NOW TX] Returning U2 slowly to 0 deg Park (OP_PARK_HOME)");
    }
  } else if (upper == "PAUSE") {
    gSession.paused = true;
    syncAllNodesImmediate(2);
    logCommandState("serial_pause");
    sendTelemetryFrame();
  } else if (upper == "RESUME") {
    if (setupPhase != SETUP_NONE) {
      Serial.println("[SERIAL] Setup phase active — wait for setup or type BEGIN / END.");
    } else {
      if (gSession.estopped) {
        triggerHardStopReleaseBurst();
        delay(6);
      } else if (u2Peer.linked &&
                 (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT | STATE_FLAG_HARDSTOP_LOCKED)) != 0) {
        sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
        delay(6);
      }
      gSession.paused = false;
      if (!gSession.active) gSession.active = true;
      u2FaultNotified = false;
      syncAllNodesImmediate(2);
      logCommandState("serial_resume");
      sendTelemetryFrame();
    }
  } else if (upper == "END" || upper == "STOP") {
    gSession.active       = false;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    gSession.rpm          = 0.0f;
    gSession.speedKmh     = 0.0f;
    pendingAutoHome       = false;
    manualSetupOp         = OP_IDLE;
    setupPhase            = SETUP_NONE;
    sawU2Positioning      = false;
    if (u2Peer.linked &&
        (gTopHubTelem.state_flags & (STATE_FLAG_SPASM_STOP | STATE_FLAG_FAULT_LOCKOUT)) != 0) {
      sendRehabCommand(NODE_TOPHUB, OP_IDLE, ESTOP_HARD_STOP_RELEASE, STEP_ACT_NONE, false, 2);
      delay(4);
    }
    syncAllNodesImmediate(2);
    logCommandState("serial_end");
    sendTelemetryFrame();
  } else if (upper == "STEP+" || line == "+") {
    stepCyclingStepper(1);
    Serial.println("[ESP-NOW TX] Sent +1 step to Stepper U3");
  } else if (upper == "STEP-" || line == "-") {
    stepCyclingStepper(-1);
    Serial.println("[ESP-NOW TX] Sent -1 step to Stepper U3");
  } else if (upper == "REP") {
    if (gSession.active && !gSession.paused && gSession.completedReps < gSession.targetReps) {
      gSession.completedReps += 1;
      sendTelemetryFrame();
      Serial.printf("Incremented reps -> %d/%d\n", gSession.completedReps, gSession.targetReps);
    }
  } else if (upper == "ESTOP" || upper == "LOCK") {
    gSession.estopped     = true;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    pendingAutoHome       = false;
    manualSetupOp         = OP_IDLE;
    setupPhase            = SETUP_NONE;
    sawU2Positioning      = false;
    syncAllNodesImmediate(3);
    sendTelemetryFrame();
    Serial.println("HW/Serial E-Stop ENGAGED (U2 Hard-Stop Locked, U3 Unloaded)");
  } else if (upper == "CLEAR" || upper == "RELEASE") {
    triggerHardStopReleaseBurst();
    sendTelemetryFrame();
    Serial.println("E-Stop RELEASED & CLEARED (U2 & U3 -> OP_IDLE)");
  } else {
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
      Serial.printf("[ESP-NOW TX] Sent '%s' to Stepper U3\n", line.c_str());
    } else {
      Serial.printf("[SERIAL] Unknown command '%s'. Type HELP for commands.\n", line.c_str());
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
  Serial.println("\n=== DOULA ESP32-S NodeMCU V1.1 U1 Master Coordinator ===");
  Serial.println("Serial: HELP | STATUS | NODES | STREAM | HOME/ZERO | AUTOHOME | CPM | ASSIST | ISO | ISOK | ISOM | ROM | TORQUE | PAUSE | RESUME | STOP | ESTOP | RELEASE");

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

  lastLoopMs      = millis();
  lastTelemetryMs = millis();
}

void loop() {
  ensureWiFiConnected(false);
  webSocket.loop();
  processTopHubQueues();
  processCyclingQueue();
  serviceEspNowHeartbeats();
  handleSerialCommands();

#if ENABLE_HW_ESTOP_PIN
  if (digitalRead(HW_ESTOP_PIN) == LOW && !gSession.estopped) {
    gSession.estopped     = true;
    gSession.paused       = true;
    gSession.liveTorqueNm = 0.0f;
    pendingAutoHome       = false;
    manualSetupOp         = OP_IDLE;
    setupPhase            = SETUP_NONE;
    sawU2Positioning      = false;
    syncAllNodesImmediate(3);
    logCommandState("hw_gpio4_estop");
    sendTelemetryFrame();
  }
#endif

  uint32_t now = millis();
  uint32_t dt  = now - lastLoopMs;
  lastLoopMs   = now;

  updateRehabKinematics(dt);

  uint32_t wsPeriod = (gSession.active || pendingAutoHome || manualSetupOp != OP_IDLE || setupPhase != SETUP_NONE)
                        ? TELEMETRY_PERIOD_MS
                        : 250;
  if (now - lastTelemetryMs >= wsPeriod) {
    lastTelemetryMs = now;
    if (webSocket.connectedClients() > 0) {
      sendTelemetryFrame();
    }
  }

  if (serialStreamTopHub && (now - lastSerialStreamMs >= 200)) {
    lastSerialStreamMs = now;
    uint8_t fl = u2Peer.linked ? gTopHubTelem.state_flags : 0;
    Serial.printf("[U2 LIVE] LNK=%d H=%d LCK=%d SPM=%d | Mode=%s(op=%u) | Tk=%+6.2f deg (%+6.1f deg/s) | Tq=%+5.2f/%.1f Nm | Im=%+5.2f A | Vbus=%5.2f V | Rep=%d/%d\n",
                  u2Peer.linked ? 1 : 0,
                  (fl & STATE_FLAG_HOMED) ? 1 : 0,
                  (fl & STATE_FLAG_HARDSTOP_LOCKED) ? 1 : 0,
                  (fl & STATE_FLAG_SPASM_STOP) ? 1 : 0,
                  gSession.modeName,
                  (unsigned)desiredTopHubOpMode(),
                  gSession.kneeAngleDeg,
                  u2Peer.linked ? gTopHubTelem.knee_vel_deg_s : 0.0f,
                  u2Peer.linked ? gTopHubTelem.knee_torque_nm : gSession.liveTorqueNm,
                  gSession.targetTorqueNm,
                  u2Peer.linked ? gTopHubTelem.motor_current_amps : 0.0f,
                  u2Peer.linked ? gTopHubTelem.vbus_volts : 0.0f,
                  gSession.completedReps,
                  gSession.targetReps);
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
