/*
 * DOULA Cycling Eddy-Brake Stepper Controller — U3 (ESP_CYCLING / Stepper Node)
 * Target Board : ESP32-S NodeMCU V1.1 (NodeMCU-32S / ESP32 Dev Module)
 * Driver       : TB6600 / DM542 (Common Cathode: PUL- and DIR- tied to GND)
 * Comms        : Unified ESP-NOW (rehab_espnow_protocol.h) with Ch 1..13 Auto-Sync
 *
 * Behavior:
 *  1. Starts at 0.0 deg home reference on boot (sessionActive = false).
 *  2. Scans Wi-Fi channels 1..13 to discover Master U1 (doula_esp32s.ino) automatically.
 *  3. On cycling session start (OP_CYCLE_TORQUE / OP_CYCLE_POWER or Serial 's' / "START"),
 *     moves to DEFAULT_START_DEG (22.5 deg -> nearest step).
 *  4. On '+' / '-' from Master ESP (ESP-NOW STEP_ACT_PLUS/MINUS) or USB Serial Monitor,
 *     moves by +1 or -1 step and updates tracked currentSteps & angleDeg.
 *  5. On session end (OP_IDLE), E-Stop, or 1500 ms active-session watchdog timeout,
 *     returns smoothly back to 0.0 deg (step 0).
 *  6. Optional ENC2 (GPIO32/33) flywheel PCNT + 100 Hz Momentum Torque Observer
 *     when ENABLE_FLYWHEEL_ENCODER is set to true.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>
#include "rehab_espnow_protocol.h"

// Flip to true once ENC2 (360 PPR flywheel optical encoder) is wired to GPIO32/33
#define ENABLE_FLYWHEEL_ENCODER false

#if ENABLE_FLYWHEEL_ENCODER
#include <driver/pcnt.h>
#endif

// ============================================================================
// 1. PIN & STEPPER CONFIGURATION
// ============================================================================
constexpr int   DIR_PIN            = 25;     // Connected to DIR+
constexpr int   STEP_PIN           = 26;     // Connected to PUL+
constexpr uint8_t STEP_DIR_UP      = LOW;   // Swap HIGH/LOW if '+' spins the wrong way
constexpr uint8_t STEP_DIR_DOWN    = HIGH;

// Step angle configuration:
// 1.8 deg/step with 1:6 gear ratio (GBX2) -> 0.30 deg/step in full-step mode.
constexpr float MOTOR_STEP_DEG       = 1.8f;
constexpr int   MICROSTEPS           = 1;
constexpr float GEAR_RATIO           = 6.0f;
constexpr float DEG_PER_STEP         = MOTOR_STEP_DEG / (MICROSTEPS * GEAR_RATIO);

constexpr float DEFAULT_START_DEG    = 22.5f;
constexpr float MIN_ANGLE_DEG        = 0.0f;
constexpr float MAX_ANGLE_DEG        = 42.0f; // Set to 45.0f if enforcing a 45 deg hard ceiling

constexpr uint32_t STEP_HALF_PERIOD_US = 3000; // 3000 us HIGH + 3000 us LOW for full-step stability
constexpr uint32_t DIR_SETUP_US        = 1000;   // TB6600/DM542 DIR setup margin before PUL rising edge

const int32_t MIN_STEPS             = lroundf(MIN_ANGLE_DEG / DEG_PER_STEP);
const int32_t MAX_STEPS             = lroundf(MAX_ANGLE_DEG / DEG_PER_STEP);
const int32_t DEFAULT_START_STEPS   = lroundf(DEFAULT_START_DEG / DEG_PER_STEP);

#if ENABLE_FLYWHEEL_ENCODER
constexpr int   PIN_ENC2_A           = 32;
constexpr int   PIN_ENC2_B           = 33;
constexpr float ENC2_CPR_FLYWHEEL    = 1440.0f;  // 360 PPR * 4x quadrature
constexpr float FLY_TO_PEDAL_RATIO   = 5.0f;     // 1:5 internal gear step-up
constexpr float CRANK_LENGTH_M       = 0.17f;
constexpr float J_FLY_EFF            = 0.01630f; // Effective flywheel inertia [kg*m^2]
constexpr float L_OBS_RAD_S          = 37.7f;    // 2*pi*6 Hz momentum DOB bandwidth
constexpr float K_EDDY_EST           = 0.018f;   // Nominal eddy torque gain [Nm/(rad/s)]
constexpr float TAU_C0_FLY           = 0.12f;    // Coulomb seal drag [Nm]
constexpr float B_VISC_FLY           = 0.004f;   // Viscous bearing drag [Nm/(rad/s)]
constexpr uint32_t OBSERVER_PERIOD_US = 10000U;  // 100 Hz observer loop
#endif

// ============================================================================
// 2. ESP-NOW TIMING & WATCHDOG CONSTANTS
// ============================================================================
constexpr uint8_t  MAX_WIFI_CHANNEL     = 13;
constexpr uint32_t CHANNEL_HOP_MS       = 180;
constexpr uint32_t ACTIVE_WATCHDOG_MS   = 1500; // Retract brake if Master silent >1.5s mid-session
constexpr uint32_t MASTER_TIMEOUT_MS    = 4000; // Re-enter channel scan after 4.0s silence
constexpr uint32_t TELEM_ACTIVE_MS      = 50;   // 20 Hz telemetry during active session
constexpr uint32_t TELEM_IDLE_MS        = 200;  // 5 Hz keepalive telemetry while idle

struct QueuedCommand {
  uint8_t              srcMac[6];
  RehabCommandPacket_t pkt;
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

bool           sessionActive     = false;
bool           estopped          = false;
int32_t        currentSteps      = 0;
int32_t        targetSteps       = 0;
uint8_t        currentDirState   = STEP_DIR_UP;
bool           steppingUp        = true;
bool           wasMoving         = false;
StepPulsePhase pulsePhase        = PULSE_IDLE;
uint32_t       pulseTimestampUs  = 0;

// Master ESP-NOW tracking & channel sync
bool     masterSynced            = false;
bool     masterPeerAdded         = false;
uint8_t  masterMac[6]            = {0};
uint8_t  currentWifiChannel      = REHAB_WIFI_CHANNEL;
uint32_t lastMasterRxMs          = 0;
uint32_t lastChannelHopMs        = 0;
uint32_t lastTelemTxMs           = 0;
uint16_t telemSeq                = 0;
bool     hasActionSeq            = false;
uint32_t lastActionBootId        = 0;
uint16_t lastActionSeq           = 0;

// Flywheel / observer telemetry fields (populated when ENABLE_FLYWHEEL_ENCODER is true)
float    g_flywheelRpm           = 0.0f;
float    g_pedalRpm              = 0.0f;
float    g_userPedalTorqueNm     = 0.0f;
float    g_userPedalForceN       = 0.0f;
float    g_eddyBrakeTorqueNm     = 0.0f;
float    g_userPowerWatts        = 0.0f;
float    g_totalWorkJoules       = 0.0f;
bool     g_pedalingActive        = false;
bool     g_coasting              = false;

#if ENABLE_FLYWHEEL_ENCODER
int16_t  g_prevEnc2Raw           = 0;
float    g_omegaFlyFiltRadS      = 0.0f;
float    g_zDob                  = 0.0f;
uint32_t g_lastObserverUs        = 0;
#endif

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
    "[STATUS] session=%s estop=%d | steps=%ld/%ld | angle=%.2f deg (target %.2f deg) | "
    "step=%.2f deg | ch=%u | master=%s | enc2=%s\n",
    sessionActive ? "ACTIVE" : "IDLE",
    estopped ? 1 : 0,
    (long)currentSteps,
    (long)targetSteps,
    stepsToAngleDeg(currentSteps),
    stepsToAngleDeg(targetSteps),
    DEG_PER_STEP,
    (unsigned)currentWifiChannel,
    masterSynced ? "LINKED" : "SCANNING",
    ENABLE_FLYWHEEL_ENCODER ? "ON" : "OFF"
  );
}

void sendTelemetryToMaster() {
  if (!masterPeerAdded || !masterSynced) return;

  CyclingTelemetryPacket_t telem{};
  telem.pkt_type           = PKT_CYCLING_TELEMETRY;
  telem.node_id            = NODE_CYCLING;
  telem.seq_num            = telemSeq++;
  telem.pedal_state_flags  = CYCLE_FLAG_STEPPER_HOMED;
  if (sessionActive)               telem.pedal_state_flags |= CYCLE_FLAG_SESSION_ACTIVE;
  if (g_pedalingActive)            telem.pedal_state_flags |= CYCLE_FLAG_PEDALING_ACTIVE;
  if (g_coasting)                  telem.pedal_state_flags |= CYCLE_FLAG_COASTING;
  if (estopped)                    telem.pedal_state_flags |= CYCLE_FLAG_ESTOP_UNLOADED;
  if (ENABLE_FLYWHEEL_ENCODER)     telem.pedal_state_flags |= CYCLE_FLAG_ENC2_ACTIVE;
  telem.current_steps      = currentSteps;
  telem.target_steps       = targetSteps;
  telem.stepper_angle_deg  = stepsToAngleDeg(currentSteps);
  telem.pedal_rpm          = g_pedalRpm;
  telem.flywheel_rpm       = g_flywheelRpm;
  telem.user_pedal_torque  = g_userPedalTorqueNm;
  telem.user_pedal_force_n = g_userPedalForceN;
  telem.eddy_brake_torque  = g_eddyBrakeTorqueNm;
  telem.user_power_watts   = g_userPowerWatts;
  telem.total_work_joules  = g_totalWorkJoules;
  telem.crc16              = computeCRC16(&telem, sizeof(telem) - sizeof(uint16_t));

  esp_now_send(masterMac, reinterpret_cast<const uint8_t*>(&telem), sizeof(telem));
  lastTelemTxMs = millis();
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
  peer.channel = 0; // Follow active STA channel
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

void startCyclingSession(const char* source) {
  estopped      = false;
  sessionActive = true;
  g_totalWorkJoules = 0.0f;
  setTargetSteps(DEFAULT_START_STEPS);
  Serial.printf(
    "[%s] SESSION START -> moving to %.2f deg target (%ld steps, actual %.2f deg) | current=%.2f deg\n",
    source,
    DEFAULT_START_DEG,
    (long)targetSteps,
    stepsToAngleDeg(targetSteps),
    stepsToAngleDeg(currentSteps)
  );
  sendTelemetryToMaster();
}

void endCyclingSession(const char* source) {
  sessionActive = false;
  setTargetSteps(MIN_STEPS);
  Serial.printf(
    "[%s] SESSION END -> returning to 0.00 deg (step 0) from %.2f deg (%ld steps)\n",
    source,
    stepsToAngleDeg(currentSteps),
    (long)currentSteps
  );
  sendTelemetryToMaster();
}

void stepDelta(int delta, const char* source) {
  setTargetSteps(targetSteps + delta);
  Serial.printf(
    "[%s] %+d step -> target=%ld steps (%.2f deg) | current=%ld steps (%.2f deg)\n",
    source,
    delta,
    (long)targetSteps,
    stepsToAngleDeg(targetSteps),
    (long)currentSteps,
    stepsToAngleDeg(currentSteps)
  );
  sendTelemetryToMaster();
}

// ============================================================================
// 5. ESP-NOW RECEIVE CALLBACK & QUEUE DRAIN
// ============================================================================
void handleEspNowRxPacket(const uint8_t* srcMac, const uint8_t* data, int len) {
  if (!srcMac || !data || len != (int)sizeof(RehabCommandPacket_t)) return;
  RehabCommandPacket_t pkt{};
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.pkt_type != PKT_HEARTBEAT_CMD) return;
  if (pkt.target_node != NODE_CYCLING && pkt.target_node != NODE_BROADCAST) return;
  uint16_t expectedCrc = computeCRC16(&pkt, sizeof(pkt) - sizeof(uint16_t));
  if (pkt.crc16 != expectedCrc || cmdQueue == nullptr) return;

  QueuedCommand item{};
  memcpy(item.srcMac, srcMac, 6);
  item.pkt = pkt;
  xQueueSend(cmdQueue, &item, 0);
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

void processEspNowQueue() {
  QueuedCommand item{};
  while (xQueueReceive(cmdQueue, &item, 0) == pdTRUE) {
    lastMasterRxMs = millis();
    bool newlyLinked = !masterSynced;
    masterSynced     = true;

    if (item.pkt.wifi_channel >= 1 && item.pkt.wifi_channel <= MAX_WIFI_CHANNEL &&
        item.pkt.wifi_channel != currentWifiChannel) {
      currentWifiChannel = item.pkt.wifi_channel;
      esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
      Serial.printf("[ESP-NOW] Locked Wi-Fi channel %u from Master\n", (unsigned)currentWifiChannel);
    }

    ensureMasterPeer(item.srcMac);

    // 1. E-Stop Arbitration (highest priority)
    if (item.pkt.estop_active == ESTOP_HARD_STOP_LOCKED ||
        item.pkt.estop_active == ESTOP_HARD_STOP_RELEASE) {
      bool changed = !estopped || sessionActive || (targetSteps != MIN_STEPS);
      estopped      = true;
      sessionActive = false;
      setTargetSteps(MIN_STEPS);
      if (changed) {
        Serial.println("[ESP-NOW] E-STOP -> unloading brake to 0.00 deg");
        sendTelemetryToMaster();
      }
      continue;
    }

    if (estopped && item.pkt.estop_active == ESTOP_NORMAL && item.pkt.op_mode == OP_IDLE) {
      estopped = false;
      Serial.println("[ESP-NOW] E-Stop cleared -> IDLE");
      sendTelemetryToMaster();
    }

    if (estopped) continue;

    // 2. Mode / Session Sync (only act on targeted NODE_CYCLING or idle broadcast)
    if (item.pkt.target_node == NODE_CYCLING || item.pkt.op_mode == OP_IDLE) {
      bool wantActive = (item.pkt.op_mode == OP_CYCLE_TORQUE || item.pkt.op_mode == OP_CYCLE_POWER);
      if (wantActive && !sessionActive) {
        startCyclingSession("ESP-NOW");
      } else if (!wantActive && sessionActive && item.pkt.op_mode == OP_IDLE) {
        endCyclingSession("ESP-NOW");
      }
    }

    // 3. Discrete Stepper Action (deduplicated strictly by boot_id + action_seq)
    if (item.pkt.stepper_action != STEP_ACT_NONE && item.pkt.action_seq != 0) {
      bool isNewAction = !hasActionSeq ||
                         (item.pkt.boot_id != lastActionBootId) ||
                         (item.pkt.action_seq != lastActionSeq);
      if (isNewAction) {
        hasActionSeq     = true;
        lastActionBootId = item.pkt.boot_id;
        lastActionSeq    = item.pkt.action_seq;
        switch (item.pkt.stepper_action) {
          case STEP_ACT_PLUS:
            stepDelta(1, "ESP-NOW");
            break;
          case STEP_ACT_MINUS:
            stepDelta(-1, "ESP-NOW");
            break;
          case STEP_ACT_HOME:
            setTargetSteps(MIN_STEPS);
            Serial.println("[ESP-NOW] HOME -> returning to 0.00 deg");
            sendTelemetryToMaster();
            break;
          default:
            break;
        }
      }
    }

    if (newlyLinked) {
      sendTelemetryToMaster();
    }
  }
}

void serviceChannelSyncAndWatchdog() {
  uint32_t now = millis();

  if (sessionActive && masterSynced && (now - lastMasterRxMs > ACTIVE_WATCHDOG_MS)) {
    sessionActive = false;
    setTargetSteps(MIN_STEPS);
    Serial.println("[SAFETY] Master heartbeat lost (>1500 ms) — retracting brake to 0.00 deg");
    sendTelemetryToMaster();
  }

  if (masterSynced && (now - lastMasterRxMs > MASTER_TIMEOUT_MS)) {
    masterSynced = false;
    Serial.println("[ESP-NOW] Master heartbeat timeout — scanning channels 1..13...");
  }

  if (!masterSynced && (now - lastChannelHopMs >= CHANNEL_HOP_MS)) {
    lastChannelHopMs   = now;
    currentWifiChannel = (currentWifiChannel % MAX_WIFI_CHANNEL) + 1;
    esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
  }

  uint32_t telemPeriod = sessionActive ? TELEM_ACTIVE_MS : TELEM_IDLE_MS;
  if (masterSynced && masterPeerAdded && (now - lastTelemTxMs >= telemPeriod)) {
    sendTelemetryToMaster();
  }
}

// ============================================================================
// 6. OPTIONAL ENC2 FLYWHEEL OBSERVER (ENABLE_FLYWHEEL_ENCODER == true)
// ============================================================================
#if ENABLE_FLYWHEEL_ENCODER
void initFlywheelEncoder() {
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);

  pcnt_config_t ch0{};
  ch0.pulse_gpio_num = PIN_ENC2_A;
  ch0.ctrl_gpio_num  = PIN_ENC2_B;
  ch0.unit           = PCNT_UNIT_0;
  ch0.channel        = PCNT_CHANNEL_0;
  ch0.pos_mode       = PCNT_COUNT_INC;
  ch0.neg_mode       = PCNT_COUNT_DEC;
  ch0.lctrl_mode     = PCNT_MODE_REVERSE;
  ch0.hctrl_mode     = PCNT_MODE_KEEP;
  ch0.counter_h_lim  = 30000;
  ch0.counter_l_lim  = -30000;
  pcnt_unit_config(&ch0);

  pcnt_config_t ch1{};
  ch1.pulse_gpio_num = PIN_ENC2_B;
  ch1.ctrl_gpio_num  = PIN_ENC2_A;
  ch1.unit           = PCNT_UNIT_0;
  ch1.channel        = PCNT_CHANNEL_1;
  ch1.pos_mode       = PCNT_COUNT_DEC;
  ch1.neg_mode       = PCNT_COUNT_INC;
  ch1.lctrl_mode     = PCNT_MODE_REVERSE;
  ch1.hctrl_mode     = PCNT_MODE_KEEP;
  ch1.counter_h_lim  = 30000;
  ch1.counter_l_lim  = -30000;
  pcnt_unit_config(&ch1);

  pcnt_set_filter_value(PCNT_UNIT_0, 100);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
  g_lastObserverUs = micros();
}

void serviceFlywheelObserver100Hz() {
  uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - g_lastObserverUs) < OBSERVER_PERIOD_US) return;
  float dt = (float)(nowUs - g_lastObserverUs) * 1e-6f;
  if (dt <= 0.0f || dt > 0.1f) dt = 0.01f;
  g_lastObserverUs = nowUs;

  int16_t raw = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &raw);
  int16_t delta = raw - g_prevEnc2Raw;
  g_prevEnc2Raw = raw;
  if (raw > 25000 || raw < -25000) {
    pcnt_counter_clear(PCNT_UNIT_0);
    g_prevEnc2Raw = 0;
  }

  float dThetaRad = ((float)delta / ENC2_CPR_FLYWHEEL) * (2.0f * PI);
  float omegaRaw  = fabsf(dThetaRad / dt);
  float alphaLpf  = dt / (dt + (1.0f / (2.0f * PI * 15.0f)));
  g_omegaFlyFiltRadS += alphaLpf * (omegaRaw - g_omegaFlyFiltRadS);

  g_flywheelRpm = g_omegaFlyFiltRadS * (60.0f / (2.0f * PI));
  g_pedalRpm    = g_flywheelRpm / FLY_TO_PEDAL_RATIO;

  float alphaRad     = stepsToAngleDeg(currentSteps) * (PI / 180.0f);
  float sin2a        = sinf(2.0f * alphaRad);
  float tauEddyFly   = K_EDDY_EST * sin2a * sin2a * g_omegaFlyFiltRadS;
  float tauDragFly   = (g_omegaFlyFiltRadS > 0.3f) ? (TAU_C0_FLY + B_VISC_FLY * g_omegaFlyFiltRadS) : 0.0f;
  float dz           = L_OBS_RAD_S * (-g_zDob - L_OBS_RAD_S * J_FLY_EFF * g_omegaFlyFiltRadS + tauEddyFly + tauDragFly);
  g_zDob            += dz * dt;

  float tauUserFly   = fmaxf(0.0f, g_zDob + L_OBS_RAD_S * J_FLY_EFF * g_omegaFlyFiltRadS);
  g_userPedalTorqueNm = tauUserFly * FLY_TO_PEDAL_RATIO;
  g_userPedalForceN   = g_userPedalTorqueNm / CRANK_LENGTH_M;
  g_eddyBrakeTorqueNm = tauEddyFly * FLY_TO_PEDAL_RATIO;
  g_userPowerWatts    = g_userPedalTorqueNm * (g_omegaFlyFiltRadS / FLY_TO_PEDAL_RATIO);
  if (g_userPowerWatts > 0.0f) {
    g_totalWorkJoules += g_userPowerWatts * dt;
  }

  g_pedalingActive = (g_userPedalTorqueNm >= 2.5f);
  g_coasting       = (!g_pedalingActive && g_pedalRpm > 5.0f);
}
#endif

// ============================================================================
// 7. SERIAL MONITOR INPUT (+ / - / START / END / HOME / ZERO / STATUS)
// ============================================================================
void executeSerialWord(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd.equalsIgnoreCase("S") || cmd.equalsIgnoreCase("START")) {
    startCyclingSession("Serial");
  } else if (cmd.equalsIgnoreCase("E") || cmd.equalsIgnoreCase("END") || cmd.equalsIgnoreCase("STOP")) {
    endCyclingSession("Serial");
  } else if (cmd.equalsIgnoreCase("0") || cmd.equalsIgnoreCase("H") || cmd.equalsIgnoreCase("HOME")) {
    setTargetSteps(MIN_STEPS);
    Serial.printf("[Serial] Moving to 0.00 deg (step 0) from %.2f deg\n", stepsToAngleDeg(currentSteps));
    sendTelemetryToMaster();
  } else if (cmd.equalsIgnoreCase("ZERO")) {
    currentSteps = 0;
    targetSteps  = 0;
    pulsePhase   = PULSE_IDLE;
    digitalWrite(STEP_PIN, LOW);
    Serial.println("[Serial] Current physical position zeroed -> 0 steps (0.00 deg)");
    sendTelemetryToMaster();
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
      stepDelta(1, "Serial");
    } else if (ch == '-') {
      executeSerialWord(serialWordBuf);
      serialWordBuf = "";
      stepDelta(-1, "Serial");
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
// 8. NON-BLOCKING STEPPER PULSE ENGINE
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
        sendTelemetryToMaster();
      }
    }
    return;
  }

  // PULSE_IDLE: check whether more steps are needed
  if (currentSteps == targetSteps) {
    if (wasMoving) {
      wasMoving = false;
      sendTelemetryToMaster();
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
// 9. SETUP & MAIN LOOP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, STEP_DIR_UP);
  currentDirState = STEP_DIR_UP;

#if ENABLE_FLYWHEEL_ENCODER
  initFlywheelEncoder();
#endif

  cmdQueue = xQueueCreate(16, sizeof(QueuedCommand));

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n=== DOULA Stepper ESP32 U3 (Unified ESP-NOW + Serial) ===");
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
    Serial.println("[ESP-NOW] Initialized (Ch 1..13 scan) and listening for Master U1...");
  }

  printStatus();
}

void loop() {
  processEspNowQueue();
  serviceChannelSyncAndWatchdog();
#if ENABLE_FLYWHEEL_ENCODER
  serviceFlywheelObserver100Hz();
#endif
  handleSerialInput();
  serviceStepper();
}
