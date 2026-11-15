/*
 * DOULA Cycling Eddy-Brake Stepper Controller — U3 (ESP_CYCLING / Stepper Node)
 * Target Board : ESP32-S NodeMCU V1.1 (NodeMCU-32S / ESP32 Dev Module)
 * Driver       : TB6600 / DM542 (Common Cathode: PUL- and DIR- tied to GND)
 *
 * Behavior:
 *  1. Starts at 0.0 deg home reference on boot (sessionActive = false).
 *  2. On session start from Master ESP (ESP-NOW) or Serial ('s' / "START"),
 *     moves to DEFAULT_START_DEG (22.5 deg -> nearest step).
 *  3. On '+' or '-' input from Master ESP (ESP-NOW) or USB Serial Monitor,
 *     moves by +1 or -1 step and updates tracked currentSteps & angleDeg.
 *  4. On session end from Master ESP (ESP-NOW) or Serial ('e' / "END"),
 *     returns smoothly back to 0.0 deg (step 0).
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>

// ============================================================================
// 1. PIN & STEPPER CONFIGURATION
// ============================================================================
constexpr int   DIR_PIN            = 19;     // Connected to DIR+
constexpr int   STEP_PIN           = 18;     // Connected to PUL+
constexpr uint8_t STEP_DIR_UP      = HIGH;   // Swap HIGH/LOW if '+' spins the wrong way
constexpr uint8_t STEP_DIR_DOWN    = LOW;

// Step angle configuration:
// Matches angle / 1.8 in full-step mode (200 steps/rev, 1:1 motor shaft).
// Change MICROSTEPS (e.g., 2, 4, 8) or GEAR_RATIO (e.g., 6.0f for 1:6 GBX2) if needed.
constexpr float MOTOR_STEP_DEG     = 1.8f;
constexpr int   MICROSTEPS         = 1;
constexpr float GEAR_RATIO         = 6.0f;
constexpr float DEG_PER_STEP       = MOTOR_STEP_DEG / (MICROSTEPS * GEAR_RATIO);

constexpr float DEFAULT_START_DEG  = 22.5f;
constexpr float MIN_ANGLE_DEG      = 0.0f;
constexpr float MAX_ANGLE_DEG      = 42.0f; // Set to 45.0f if enforcing a 45 deg hard ceiling

constexpr uint32_t STEP_HALF_PERIOD_US = 3000; // 3000 us HIGH + 3000 us LOW for full-step stability
constexpr uint32_t DIR_SETUP_US        = 20;   // TB6600/DM542 DIR setup margin before PUL rising edge

const int32_t MIN_STEPS           = lroundf(MIN_ANGLE_DEG / DEG_PER_STEP);
const int32_t MAX_STEPS           = lroundf(MAX_ANGLE_DEG / DEG_PER_STEP);
const int32_t DEFAULT_START_STEPS = lroundf(DEFAULT_START_DEG / DEG_PER_STEP);

// ============================================================================
// 2. ESP-NOW PROTOCOL (MUST MATCH MASTER U1 doula_esp32s.ino)
// ============================================================================
constexpr uint16_t STEPPER_CMD_MAGIC    = 0xD01A;
constexpr uint16_t STEPPER_STATUS_MAGIC = 0xD01B;
constexpr uint8_t  MAX_WIFI_CHANNEL     = 13;
constexpr uint32_t CHANNEL_HOP_MS       = 180;
constexpr uint32_t MASTER_TIMEOUT_MS    = 4000;

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

struct QueuedCommand {
  uint8_t                srcMac[6];
  StepperCommandPacket_t pkt;
};

// ============================================================================
// 3. RUNTIME STATE
// ============================================================================
enum StepPulsePhase : uint8_t {
  PULSE_IDLE = 0,
  PULSE_HIGH = 1,
  PULSE_LOW  = 2
};

QueueHandle_t cmdQueue = nullptr;

bool     sessionActive           = false;
int32_t  currentSteps            = 0;
int32_t  targetSteps             = 0;
uint8_t  currentDirState         = STEP_DIR_UP;
bool     steppingUp              = true;
bool     wasMoving               = false;
StepPulsePhase pulsePhase        = PULSE_IDLE;
uint32_t pulseTimestampUs        = 0;

// Master ESP-NOW tracking & channel sync
bool     masterSynced            = false;
bool     masterPeerAdded         = false;
uint8_t  masterMac[6]            = {0};
uint8_t  currentWifiChannel      = 1;
uint32_t lastMasterRxMs          = 0;
uint32_t lastChannelHopMs        = 0;
bool     hasRxSeq                = false;
uint32_t lastRxBootId            = 0;
uint32_t lastRxSeq               = 0;
bool     masterStateKnown        = false;
bool     lastMasterSessionActive = false;

// Serial Monitor buffer
String   serialWordBuf;
uint32_t lastSerialByteMs        = 0;

// ============================================================================
// 4. POSITION & TELEMETRY HELPERS
// ============================================================================
float stepsToAngleDeg(int32_t steps) {
  return (float)steps * DEG_PER_STEP;
}

int32_t clampSteps(int32_t steps) {
  if (steps < MIN_STEPS) return MIN_STEPS;
  if (steps > MAX_STEPS) return MAX_STEPS;
  return steps;
}

void setTargetSteps(int32_t nextSteps) {
  targetSteps = clampSteps(nextSteps);
}

void printStatus() {
  Serial.printf(
    "[STATUS] session=%s | steps=%ld/%ld | angle=%.2f deg (target %.2f deg) | "
    "step=%.2f deg | ch=%u | master=%s\n",
    sessionActive ? "ACTIVE" : "IDLE",
    (long)currentSteps,
    (long)targetSteps,
    stepsToAngleDeg(currentSteps),
    stepsToAngleDeg(targetSteps),
    DEG_PER_STEP,
    (unsigned)currentWifiChannel,
    masterSynced ? "LINKED" : "SCANNING"
  );
}

void sendStatusToMaster() {
  if (!masterPeerAdded) return;

  StepperStatusPacket_t status{};
  status.magic         = STEPPER_STATUS_MAGIC;
  status.sessionActive = sessionActive ? 1 : 0;
  status.currentSteps  = currentSteps;
  status.targetSteps   = targetSteps;
  status.angleDeg      = stepsToAngleDeg(currentSteps);

  esp_now_send(masterMac, reinterpret_cast<const uint8_t*>(&status), sizeof(status));
}

void ensureMasterPeer(const uint8_t* srcMac) {
  if (masterPeerAdded && memcmp(masterMac, srcMac, 6) == 0) return;

  if (masterPeerAdded) {
    esp_now_del_peer(masterMac);
    masterPeerAdded = false;
  }

  memcpy(masterMac, srcMac, 6);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, masterMac, 6);
  peer.channel = 0; // Use current STA channel
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) == ESP_OK) {
    masterPeerAdded = true;
    Serial.printf(
      "[ESP-NOW] Paired Master MAC %02X:%02X:%02X:%02X:%02X:%02X on ch %u\n",
      masterMac[0], masterMac[1], masterMac[2], masterMac[3], masterMac[4], masterMac[5],
      (unsigned)currentWifiChannel
    );
  }
}

void applyStepperCommand(uint8_t cmd, const char* source) {
  switch (cmd) {
    case CMD_START_SESSION:
      sessionActive = true;
      setTargetSteps(DEFAULT_START_STEPS);
      Serial.printf(
        "[%s] SESSION START -> moving to %.2f deg target (%ld steps, actual %.2f deg) | current=%.2f deg\n",
        source,
        DEFAULT_START_DEG,
        (long)targetSteps,
        stepsToAngleDeg(targetSteps),
        stepsToAngleDeg(currentSteps)
      );
      sendStatusToMaster();
      break;

    case CMD_STEP_PLUS:
      setTargetSteps(targetSteps + 1);
      Serial.printf(
        "[%s] +1 step -> target=%ld steps (%.2f deg) | current=%ld steps (%.2f deg)\n",
        source,
        (long)targetSteps,
        stepsToAngleDeg(targetSteps),
        (long)currentSteps,
        stepsToAngleDeg(currentSteps)
      );
      sendStatusToMaster();
      break;

    case CMD_STEP_MINUS:
      setTargetSteps(targetSteps - 1);
      Serial.printf(
        "[%s] -1 step -> target=%ld steps (%.2f deg) | current=%ld steps (%.2f deg)\n",
        source,
        (long)targetSteps,
        stepsToAngleDeg(targetSteps),
        (long)currentSteps,
        stepsToAngleDeg(currentSteps)
      );
      sendStatusToMaster();
      break;

    case CMD_END_SESSION:
      sessionActive = false;
      setTargetSteps(MIN_STEPS);
      Serial.printf(
        "[%s] SESSION END -> returning to 0.00 deg (step 0) from %.2f deg (%ld steps)\n",
        source,
        stepsToAngleDeg(currentSteps),
        (long)currentSteps
      );
      sendStatusToMaster();
      break;

    default:
      break;
  }
}

// ============================================================================
// 5. ESP-NOW RECEIVE CALLBACK & QUEUE DRAIN
// ============================================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void onEspNowRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  if (!info || !info->src_addr || !data || len != sizeof(StepperCommandPacket_t)) return;
  const uint8_t* srcMac = info->src_addr;
#else
void onEspNowRecv(const uint8_t* srcMac, const uint8_t* data, int len) {
  if (!srcMac || !data || len != sizeof(StepperCommandPacket_t)) return;
#endif
  StepperCommandPacket_t pkt{};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != STEPPER_CMD_MAGIC || cmdQueue == nullptr) return;

  QueuedCommand item{};
  memcpy(item.srcMac, srcMac, 6);
  item.pkt = pkt;
  xQueueSend(cmdQueue, &item, 0);
}

void processEspNowQueue() {
  QueuedCommand item{};
  while (xQueueReceive(cmdQueue, &item, 0) == pdTRUE) {
    lastMasterRxMs = millis();
    masterSynced   = true;

    if (item.pkt.wifiChannel >= 1 && item.pkt.wifiChannel <= MAX_WIFI_CHANNEL &&
        item.pkt.wifiChannel != currentWifiChannel) {
      currentWifiChannel = item.pkt.wifiChannel;
      esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
      Serial.printf("[ESP-NOW] Locked Wi-Fi channel %u from Master\n", (unsigned)currentWifiChannel);
    }

    ensureMasterPeer(item.srcMac);

    bool isDuplicate = hasRxSeq &&
                       (item.pkt.bootId == lastRxBootId) &&
                       (item.pkt.seq == lastRxSeq);
    if (isDuplicate) continue;

    hasRxSeq     = true;
    lastRxBootId = item.pkt.bootId;
    lastRxSeq    = item.pkt.seq;

    bool masterActive = (item.pkt.sessionActive != 0);

    if (item.pkt.cmd == CMD_SYNC_HEARTBEAT) {
      if (!masterStateKnown) {
        masterStateKnown        = true;
        lastMasterSessionActive = masterActive;
        if (masterActive && !sessionActive) {
          applyStepperCommand(CMD_START_SESSION, "ESP-NOW SYNC");
        }
      } else if (masterActive != lastMasterSessionActive) {
        lastMasterSessionActive = masterActive;
        if (masterActive && !sessionActive) {
          applyStepperCommand(CMD_START_SESSION, "ESP-NOW SYNC");
        } else if (!masterActive && sessionActive) {
          applyStepperCommand(CMD_END_SESSION, "ESP-NOW SYNC");
        }
      }
      sendStatusToMaster();
    } else {
      masterStateKnown        = true;
      lastMasterSessionActive = (item.pkt.cmd == CMD_END_SESSION) ? false : masterActive;
      applyStepperCommand(item.pkt.cmd, "ESP-NOW");
    }
  }
}

void serviceChannelSync() {
  uint32_t now = millis();
  if (masterSynced && (now - lastMasterRxMs > MASTER_TIMEOUT_MS)) {
    masterSynced = false;
    Serial.println("[ESP-NOW] Master heartbeat timeout — scanning channels...");
  }

  if (!masterSynced && (now - lastChannelHopMs >= CHANNEL_HOP_MS)) {
    lastChannelHopMs   = now;
    currentWifiChannel = (currentWifiChannel % MAX_WIFI_CHANNEL) + 1;
    esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
  }
}

// ============================================================================
// 6. SERIAL MONITOR INPUT (+ / - / START / END / HOME / ZERO / STATUS)
// ============================================================================
void executeSerialWord(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("S") || cmd.equalsIgnoreCase("START")) {
    applyStepperCommand(CMD_START_SESSION, "Serial");
  } else if (cmd.equalsIgnoreCase("E") || cmd.equalsIgnoreCase("END") || cmd.equalsIgnoreCase("STOP")) {
    applyStepperCommand(CMD_END_SESSION, "Serial");
  } else if (cmd.equalsIgnoreCase("0") || cmd.equalsIgnoreCase("H") || cmd.equalsIgnoreCase("HOME")) {
    setTargetSteps(MIN_STEPS);
    Serial.printf("[Serial] Moving to 0.00 deg (step 0) from %.2f deg\n", stepsToAngleDeg(currentSteps));
    sendStatusToMaster();
  } else if (cmd.equalsIgnoreCase("ZERO")) {
    currentSteps = 0;
    targetSteps  = 0;
    pulsePhase   = PULSE_IDLE;
    digitalWrite(STEP_PIN, LOW);
    Serial.println("[Serial] Current physical position zeroed -> 0 steps (0.00 deg)");
    sendStatusToMaster();
  } else if (cmd.equals("?") || cmd.equalsIgnoreCase("STATUS")) {
    printStatus();
  } else {
    Serial.printf("[Serial] Unknown command '%s'. Use: + | - | START | END | HOME | ZERO | STATUS\n", cmd.c_str());
  }
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    lastSerialByteMs = millis();

    if (ch == '+') {
      executeSerialWord(serialWordBuf);
      serialWordBuf = "";
      applyStepperCommand(CMD_STEP_PLUS, "Serial");
    } else if (ch == '-') {
      executeSerialWord(serialWordBuf);
      serialWordBuf = "";
      applyStepperCommand(CMD_STEP_MINUS, "Serial");
    } else if (ch == '\n' || ch == '\r') {
      executeSerialWord(serialWordBuf);
      serialWordBuf = "";
    } else if (ch == ' ' || ch == '\t') {
      if (serialWordBuf.length() > 0) {
        executeSerialWord(serialWordBuf);
        serialWordBuf = "";
      }
    } else {
      serialWordBuf += ch;
    }
  }

  // Support Serial Monitor "No line ending" mode after a short idle pause
  if (serialWordBuf.length() > 0 && (millis() - lastSerialByteMs >= 100)) {
    executeSerialWord(serialWordBuf);
    serialWordBuf = "";
  }
}

// ============================================================================
// 7. NON-BLOCKING STEPPER PULSE ENGINE
// ============================================================================
void serviceStepper() {
  uint32_t nowUs = micros();

  if (pulsePhase == PULSE_HIGH) {
    if ((uint32_t)(nowUs - pulseTimestampUs) >= STEP_HALF_PERIOD_US) {
      digitalWrite(STEP_PIN, LOW);
      pulsePhase       = PULSE_LOW;
      pulseTimestampUs = nowUs;
    }
    return;
  }

  if (pulsePhase == PULSE_LOW) {
    if ((uint32_t)(nowUs - pulseTimestampUs) >= STEP_HALF_PERIOD_US) {
      currentSteps += steppingUp ? 1 : -1;
      pulsePhase    = PULSE_IDLE;

      if (currentSteps == targetSteps) {
        wasMoving = false;
        Serial.printf(
          "[POS] Reached target: %ld steps | angle=%.2f deg\n",
          (long)currentSteps,
          stepsToAngleDeg(currentSteps)
        );
        sendStatusToMaster();
      }
    }
    return;
  }

  // PULSE_IDLE: check whether more steps are needed
  if (currentSteps == targetSteps) {
    if (wasMoving) {
      wasMoving = false;
      sendStatusToMaster();
    }
    return;
  }

  steppingUp = (targetSteps > currentSteps);
  uint8_t desiredDir = steppingUp ? STEP_DIR_UP : STEP_DIR_DOWN;
  if (desiredDir != currentDirState) {
    digitalWrite(DIR_PIN, desiredDir);
    currentDirState = desiredDir;
    delayMicroseconds(DIR_SETUP_US);
  }

  wasMoving        = true;
  digitalWrite(STEP_PIN, HIGH);
  pulsePhase       = PULSE_HIGH;
  pulseTimestampUs = micros();
}

// ============================================================================
// 8. SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, STEP_DIR_UP);
  currentDirState = STEP_DIR_UP;

  cmdQueue = xQueueCreate(16, sizeof(QueuedCommand));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n=== DOULA Stepper ESP32 (ESP-NOW + Serial) ===");
  Serial.printf("Stepper MAC Address : %s\n", WiFi.macAddress().c_str());
  Serial.printf(
    "Step Scale          : %.2f deg/step (default 22.5 deg -> %ld steps = %.2f deg)\n",
    DEG_PER_STEP,
    (long)DEFAULT_START_STEPS,
    stepsToAngleDeg(DEFAULT_START_STEPS)
  );
  Serial.println("Serial Commands     : + | - | START (s) | END (e) | HOME (0) | ZERO | STATUS (?)");

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: esp_now_init() failed!");
  } else {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("[ESP-NOW] Initialized and listening for Master ESP commands...");
  }

  printStatus();
}

void loop() {
  processEspNowQueue();
  serviceChannelSync();
  handleSerialInput();
  serviceStepper();
}
