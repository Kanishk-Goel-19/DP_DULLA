/*
 * DOULA Cycling Eddy-Brake Multi-Mode Controller — U3 (Unified ESP-NOW + Serial Edition)
 * Target Board : ESP32-S NodeMCU V1.1 (NodeMCU-32S / ESP32 Dev Module)
 * Stepper      : U301 (TB6600 / DM542 Common Cathode: DIR+ IO25, PUL+ IO26, PUL-/DIR- GND)
 * Encoder      : ENC2 (360 PPR NPN Open-Collector on Flywheel Shaft: Ch A IO32, Ch B IO33)
 * Comms        : Unified ESP-NOW (rehab_espnow_protocol.h) with Ch 1..13 Auto-Sync
 *                + Full USB Serial Monitor CLI @ 115200 baud (works with or without Master U1)
 *
 * Modes (selectable via Master ESP32 / Website OR USB Serial Monitor @ 115200 baud):
 *   1. Manual (OP_CYCLE_MANUAL=15)         : START -> 22.5 deg (75 steps), +/- -> +/-1 step (0.30 deg), END -> 0.0 deg
 *   2. Constant Torque (OP_CYCLE_TORQUE=7) : Regulates stator angle alpha in [0, 42 deg] for target pedal torque [Nm]
 *   3. Constant Power (OP_CYCLE_POWER=8)   : Regulates eddy brake for target mechanical power [W] (ergometer)
 *   4. Isokinetic (OP_CYCLE_ISOKINETIC=14) : PI cadence-clamping brake for target pedal RPM
 *
 * Noise Immunity & Safety:
 *   - Hardware ESP32 PCNT 4x quadrature decoding with 5.0 us APB glitch filter
 *   - Physical per-tick delta spike rejection + lightweight EMA velocity low-pass filter
 *   - Luenberger Momentum Disturbance Observer (DOB) + coasting auto-unload to 5 deg standby
 *   - 1.5 s soft-start ramp, 30 deg/s slew limiter, 2-step closed-loop deadband, NVS config storage
 *   - 1.5 s Master heartbeat watchdog during ESP-NOW sessions + <3 ms E-Stop unload to 0.0 deg
 */

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <driver/pcnt.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>
#include "rehab_espnow_protocol.h"

// ============================================================================
// 1. HARDWARE PINOUT & MECHANICAL CONSTANTS
// ============================================================================
constexpr int     DIR_PIN           = 25;    // Connected to TB6600/DM542 DIR+
constexpr int     STEP_PIN          = 26;    // Connected to TB6600/DM542 PUL+
constexpr uint8_t STEP_DIR_UP       = HIGH;  // Swap to LOW if '+' spins the opposite way
constexpr uint8_t STEP_DIR_DOWN     = (STEP_DIR_UP == HIGH) ? LOW : HIGH;

constexpr int     PIN_ENC2_A        = 32;    // ENC2 Out A (80 Ohm series + INPUT_PULLUP)
constexpr int     PIN_ENC2_B        = 33;    // ENC2 Out B (80 Ohm series + INPUT_PULLUP)

// Stepper geometry (1.8 deg/step, full-step, 1:6 GBX2 gear reduction -> 0.30 deg/step)
constexpr float   MOTOR_STEP_DEG    = 1.8f;
constexpr int     MICROSTEPS        = 1;
constexpr float   GEAR_RATIO        = 6.0f;
constexpr float   DEG_PER_STEP      = MOTOR_STEP_DEG / (MICROSTEPS * GEAR_RATIO);

constexpr float   DEFAULT_START_DEG = 22.5f;
constexpr float   MIN_ANGLE_DEG     = 0.0f;
constexpr float   MAX_ANGLE_DEG     = 42.0f;
constexpr float   STANDBY_ANGLE_DEG = 5.0f;

constexpr uint32_t STEP_HALF_PERIOD_US = 3000; // 3000 us HIGH + 3000 us LOW (166.7 steps/s max)
constexpr uint32_t DIR_SETUP_US        = 1000; // 1 ms DIR settling margin for slow optocouplers

const int32_t MIN_STEPS           = lroundf(MIN_ANGLE_DEG / DEG_PER_STEP);         // 0
const int32_t MAX_STEPS           = lroundf(MAX_ANGLE_DEG / DEG_PER_STEP);         // 140
const int32_t DEFAULT_START_STEPS = lroundf(DEFAULT_START_DEG / DEG_PER_STEP);     // 75
const int32_t STANDBY_STEPS       = lroundf(STANDBY_ANGLE_DEG / DEG_PER_STEP);     // 17

// Flywheel drivetrain & quadrature encoder constants (docs/README_CYCLING_EDDY_BRAKE.md)
constexpr float   GEAR_PEDAL_TO_FLY       = 5.0f;    // 1:5 ring gear to flywheel pinion
constexpr float   ENC_CPR_4X              = 1440.0f; // 360 PPR * 4x = 1440 counts/flywheel rev (7200/pedal rev)
constexpr float   RAD_PER_COUNT           = (2.0f * (float)M_PI) / ENC_CPR_4X;
constexpr float   J_FLY_EFF               = 0.01630f;// Effective flywheel inertia [kg*m^2]
constexpr float   OMEGA_CRIT              = 80.0f;   // Eddy inductive reaction speed [rad/s]
constexpr float   CRANK_LENGTH_M          = 0.170f;  // Pedal crank radius [m]
constexpr float   OBS_BW_RAD_S            = 37.7f;   // 6 Hz Luenberger DOB bandwidth [rad/s]
constexpr float   SPEED_KMH_PER_PEDAL_RPM = 0.31f;   // Virtual ergometer road speed scale (60 RPM -> 18.6 km/h)

// Noise immunity & control loop timing
constexpr pcnt_unit_t ENC_PCNT_UNIT         = PCNT_UNIT_0;
constexpr int16_t PCNT_HIGH_LIMIT           = 30000;
constexpr int16_t PCNT_LOW_LIMIT            = -30000;
constexpr int16_t PCNT_WRAP_RESET_THRESH    = 20000;
constexpr uint16_t PCNT_GLITCH_FILTER_APB   = 400;   // 400 APB cycles @ 80 MHz = 5.0 us pulse reject
constexpr int32_t MAX_VALID_DELTA_10MS      = 220;   // ~916 RPM flywheel physical sanity cap per 10 ms

constexpr uint32_t CONTROL_PERIOD_US        = 10000; // 100 Hz control & observer loop
constexpr uint32_t LOG_PERIOD_MS            = 250;   // 4 Hz Serial Monitor log
constexpr uint32_t PLOT_PERIOD_MS           = 100;   // 10 Hz Arduino Serial Plotter stream

// ESP-NOW timing & watchdog constants
constexpr uint8_t  MAX_WIFI_CHANNEL         = 13;
constexpr uint32_t CHANNEL_HOP_MS           = 180;
constexpr uint32_t ACTIVE_WATCHDOG_MS       = 1500;  // Retract brake if Master silent >1.5s in ESP-NOW session
constexpr uint32_t MASTER_TIMEOUT_MS        = 4000;  // Re-enter channel scan after 4.0s silence
constexpr uint32_t TELEM_ACTIVE_MS          = 50;    // 20 Hz telemetry while active/pedaling
constexpr uint32_t TELEM_IDLE_MS            = 200;   // 5 Hz keepalive telemetry while idle

// Mode bounds & anti-stall thresholds
constexpr float   MIN_TORQUE_NM             = 2.0f;
constexpr float   MAX_TORQUE_NM             = 85.0f;
constexpr float   STEP_TORQUE_NM            = 2.0f;

constexpr float   MIN_POWER_W               = 15.0f;
constexpr float   MAX_POWER_W               = 250.0f;
constexpr float   STEP_POWER_W              = 10.0f;

constexpr float   MIN_ISOK_RPM              = 15.0f;
constexpr float   MAX_ISOK_RPM              = 100.0f;
constexpr float   STEP_ISOK_RPM             = 5.0f;

constexpr float   DOB_PEDAL_ON_NM           = 2.5f;
constexpr float   DOB_PEDAL_OFF_NM          = 1.2f;
constexpr float   SPEED_ONLY_ON_RPM         = 10.0f;
constexpr float   SPEED_ONLY_OFF_RPM        = 6.0f;
constexpr float   COAST_DEBOUNCE_S          = 0.30f;
constexpr float   SOFT_START_RAMP_S         = 1.50f;
constexpr float   MAX_DISK_SLEW_DEG_S       = 30.0f;
constexpr int32_t CLOSED_LOOP_STEP_DEADBAND = 2;     // 2 steps = 0.60 deg anti-chatter deadband

// ============================================================================
// 2. TYPES & PERSISTENT TUNING CONFIGURATION (NVS)
// ============================================================================
constexpr uint32_t CONFIG_MAGIC = 0x444F5533; // "DOU3"
constexpr char     NVS_NS[]     = "doula_cyc";
constexpr char     NVS_KEY[]    = "cfg";

struct CyclingConfig {
  uint32_t magic;
  float    lpfAlpha;
  bool     dobEnabled;
  int8_t   encPolarity;
  float    kEddyMax;
  float    tauC0Fric;
  float    bViscFric;
  float    isokKp;
  float    isokKi;
};

enum CyclingMode : uint8_t {
  MODE_MANUAL       = 0,
  MODE_CONST_TORQUE = 1,
  MODE_CONST_POWER  = 2,
  MODE_ISOKINETIC   = 3
};

enum ControlState : uint8_t {
  CTRL_STOPPED         = 0,
  CTRL_IDLE_READY      = 1,
  CTRL_SOFT_START_RAMP = 2,
  CTRL_ACTIVE_CONTROL  = 3,
  CTRL_COAST_UNLOAD    = 4
};

enum SessionSource : uint8_t {
  SRC_NONE   = 0,
  SRC_SERIAL = 1,
  SRC_ESPNOW = 2
};

enum StepPulsePhase : uint8_t {
  PULSE_IDLE = 0,
  PULSE_HIGH = 1,
  PULSE_LOW  = 2
};

struct QueuedCommand {
  uint8_t              srcMac[6];
  RehabCommandPacket_t pkt;
};

Preferences   gPrefs;
CyclingConfig cfg{};

CyclingConfig defaultConfig() {
  CyclingConfig c{};
  c.magic       = CONFIG_MAGIC;
  c.lpfAlpha    = 0.35f;
  c.dobEnabled  = true;
  c.encPolarity = 1;
  c.kEddyMax    = 0.715f;
  c.tauC0Fric   = 0.18f;
  c.bViscFric   = 0.004f;
  c.isokKp      = 1.2f;
  c.isokKi      = 0.8f;
  return c;
}

bool isValidConfig(const CyclingConfig& c) {
  if (c.magic != CONFIG_MAGIC) return false;
  if (!isfinite(c.lpfAlpha) || c.lpfAlpha < 0.05f || c.lpfAlpha > 1.0f) return false;
  if (c.encPolarity != 1 && c.encPolarity != -1) return false;
  if (!isfinite(c.kEddyMax) || c.kEddyMax <= 0.05f || c.kEddyMax > 10.0f) return false;
  if (!isfinite(c.tauC0Fric) || c.tauC0Fric < 0.0f || c.tauC0Fric > 5.0f) return false;
  if (!isfinite(c.bViscFric) || c.bViscFric < 0.0f || c.bViscFric > 1.0f) return false;
  if (!isfinite(c.isokKp) || c.isokKp < 0.0f || c.isokKp > 20.0f) return false;
  if (!isfinite(c.isokKi) || c.isokKi < 0.0f || c.isokKi > 20.0f) return false;
  return true;
}

void saveConfigToNVS() {
  cfg.magic = CONFIG_MAGIC;
  gPrefs.begin(NVS_NS, false);
  gPrefs.putBytes(NVS_KEY, &cfg, sizeof(cfg));
  gPrefs.end();
}

void loadConfigFromNVS() {
  cfg = defaultConfig();
  gPrefs.begin(NVS_NS, true);
  size_t len = gPrefs.getBytesLength(NVS_KEY);
  if (len == sizeof(CyclingConfig)) {
    CyclingConfig loaded{};
    gPrefs.getBytes(NVS_KEY, &loaded, sizeof(loaded));
    if (isValidConfig(loaded)) {
      cfg = loaded;
    }
  }
  gPrefs.end();
}

// ============================================================================
// 3. RUNTIME STATE
// ============================================================================
// Stepper pulse engine state
int32_t        currentSteps      = 0;
int32_t        targetSteps       = 0;
uint8_t        currentDirState   = STEP_DIR_UP;
bool           steppingUp        = true;
bool           wasMoving         = false;
StepPulsePhase pulsePhase        = PULSE_IDLE;
uint32_t       pulseTimestampUs  = 0;

// Session & mode state
CyclingMode    activeMode        = MODE_MANUAL;
ControlState   ctrlState         = CTRL_STOPPED;
SessionSource  sessionSource     = SRC_NONE;
bool           sessionActive     = false;
bool           estopped          = false;
float          targetTorqueNm    = 15.0f;
float          targetPowerW      = 60.0f;
float          targetIsokRpm     = 45.0f;

// Encoder & observer state
int16_t        prevPcntRaw       = 0;
int64_t        totalEncCounts    = 0;
uint32_t       glitchRejectCount = 0;
float          omegaSignedRaw    = 0.0f;
float          omegaSignedFilt   = 0.0f;
float          omegaMag          = 0.0f;
float          flywheelRpm       = 0.0f;
float          pedalRpm          = 0.0f;
float          speedKmh          = 0.0f;
float          completedPedalRevs = 0.0f;

float          zObs              = 0.0f;
float          tauEddyFly        = 0.0f;
float          tauPedalEst       = 0.0f;
float          forcePedalEst     = 0.0f;
float          powerUserW        = 0.0f;
float          totalWorkJoules   = 0.0f;

// Closed-loop control state
float          coastTimerSec     = 0.0f;
float          rampElapsedSec    = 0.0f;
float          rampStartAlpha    = 0.0f;
float          alphaTargetDeg    = 0.0f;
float          alphaCommandDeg   = 0.0f;
float          isokIntegral      = 0.0f;

// ESP-NOW Master link & command state
QueueHandle_t  cmdQueue          = nullptr;
bool           masterSynced      = false;
bool           masterPeerAdded   = false;
uint8_t        masterMac[6]      = {0};
uint8_t        currentWifiChannel = REHAB_WIFI_CHANNEL;
uint32_t       lastMasterRxMs    = 0;
uint32_t       lastChannelHopMs  = 0;
uint32_t       lastTelemTxMs     = 0;
uint16_t       telemSeq          = 0;

bool           hasMasterOpState  = false;
uint8_t        lastMasterOpMode  = OP_IDLE;
uint32_t       lastMasterBootId  = 0;
bool           hasActionSeq      = false;
uint32_t       lastActionBootId  = 0;
uint16_t       lastActionSeq     = 0;

// Scheduling & Serial CLI state
uint32_t       lastControlUs     = 0;
uint32_t       lastLogMs         = 0;
uint32_t       lastPlotMs        = 0;
bool           logEnabled        = false;
bool           plotEnabled       = false;
String         serialWordBuf;
uint32_t       lastSerialByteMs  = 0;

// ============================================================================
// 4. GENERAL HELPERS & TELEMETRY TX
// ============================================================================
float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

float stepsToAngleDeg(int32_t steps) {
  return (float)steps * DEG_PER_STEP;
}

int32_t clampSteps(int32_t steps) {
  if (steps < MIN_STEPS) return MIN_STEPS;
  if (steps > MAX_STEPS) return MAX_STEPS;
  return steps;
}

uint8_t cyclingModeToOpMode(CyclingMode m) {
  switch (m) {
    case MODE_CONST_TORQUE: return OP_CYCLE_TORQUE;
    case MODE_CONST_POWER:  return OP_CYCLE_POWER;
    case MODE_ISOKINETIC:   return OP_CYCLE_ISOKINETIC;
    case MODE_MANUAL:
    default:                return OP_CYCLE_MANUAL;
  }
}

bool isCyclingOpMode(uint8_t op) {
  return op == OP_CYCLE_TORQUE ||
         op == OP_CYCLE_POWER ||
         op == OP_CYCLE_ISOKINETIC ||
         op == OP_CYCLE_MANUAL;
}

CyclingMode opModeToCyclingMode(uint8_t op) {
  switch (op) {
    case OP_CYCLE_TORQUE:     return MODE_CONST_TORQUE;
    case OP_CYCLE_POWER:      return MODE_CONST_POWER;
    case OP_CYCLE_ISOKINETIC: return MODE_ISOKINETIC;
    case OP_CYCLE_MANUAL:
    default:                  return MODE_MANUAL;
  }
}

const char* modeName(CyclingMode m) {
  switch (m) {
    case MODE_MANUAL:       return "MANUAL";
    case MODE_CONST_TORQUE: return "TORQUE";
    case MODE_CONST_POWER:  return "POWER";
    case MODE_ISOKINETIC:   return "ISOK";
    default:                return "UNKNOWN";
  }
}

const char* stateName(ControlState s) {
  switch (s) {
    case CTRL_STOPPED:         return "STOPPED";
    case CTRL_IDLE_READY:      return "STANDBY";
    case CTRL_SOFT_START_RAMP: return "RAMP";
    case CTRL_ACTIVE_CONTROL:  return "ACTIVE";
    case CTRL_COAST_UNLOAD:    return "UNLOAD";
    default:                   return "UNKNOWN";
  }
}

void sendTelemetryToMaster() {
  if (!masterPeerAdded || !masterSynced) return;

  CyclingTelemetryPacket_t telem{};
  telem.pkt_type          = PKT_CYCLING_TELEMETRY;
  telem.node_id           = NODE_CYCLING;
  telem.seq_num           = telemSeq++;
  telem.pedal_state_flags = CYCLE_FLAG_STEPPER_HOMED | CYCLE_FLAG_ENC2_ACTIVE;
  if (sessionActive) telem.pedal_state_flags |= CYCLE_FLAG_SESSION_ACTIVE;
  bool isPedaling = (ctrlState == CTRL_ACTIVE_CONTROL || ctrlState == CTRL_SOFT_START_RAMP) ||
                    (activeMode == MODE_MANUAL && pedalRpm >= 3.0f);
  if (isPedaling && pedalRpm >= 1.5f) telem.pedal_state_flags |= CYCLE_FLAG_PEDALING_ACTIVE;
  if (ctrlState == CTRL_COAST_UNLOAD) telem.pedal_state_flags |= CYCLE_FLAG_COASTING;
  if (estopped)                       telem.pedal_state_flags |= CYCLE_FLAG_ESTOP_UNLOADED;
  if (cfg.dobEnabled)                 telem.pedal_state_flags |= CYCLE_FLAG_DOB_ENABLED;

  telem.active_mode        = sessionActive ? cyclingModeToOpMode(activeMode) : (uint8_t)OP_IDLE;
  telem.current_steps      = currentSteps;
  telem.target_steps       = targetSteps;
  telem.stepper_angle_deg  = stepsToAngleDeg(currentSteps);
  telem.pedal_rpm          = pedalRpm;
  telem.flywheel_rpm       = flywheelRpm;
  telem.speed_kmh          = speedKmh;
  telem.user_pedal_torque  = fmaxf(0.0f, tauPedalEst);
  telem.user_pedal_force_n = fmaxf(0.0f, forcePedalEst);
  telem.eddy_brake_torque  = tauEddyFly * GEAR_PEDAL_TO_FLY;
  telem.user_power_watts   = powerUserW;
  telem.total_work_joules  = totalWorkJoules;
  telem.completed_revs     = completedPedalRevs;
  telem.crc16              = computeCRC16(&telem, sizeof(telem) - sizeof(uint16_t));

  esp_now_send(masterMac, reinterpret_cast<const uint8_t*>(&telem), sizeof(telem));
  lastTelemTxMs = millis();
}

void printStatus() {
  int8_t dirSign = (omegaSignedFilt > 0.2f) ? 1 : ((omegaSignedFilt < -0.2f) ? -1 : 0);
  Serial.printf(
    "[STATUS] mode=%s state=%s estop=%d master=%s(ch=%u) | ",
    modeName(activeMode),
    stateName(ctrlState),
    estopped ? 1 : 0,
    masterSynced ? "LINKED" : "SCAN",
    (unsigned)currentWifiChannel
  );
  if (activeMode == MODE_MANUAL) {
    Serial.printf("set=MANUAL");
  } else if (activeMode == MODE_CONST_TORQUE) {
    Serial.printf("set=%.1f Nm", targetTorqueNm);
  } else if (activeMode == MODE_CONST_POWER) {
    Serial.printf("set=%.0f W", targetPowerW);
  } else {
    Serial.printf("set=%.0f RPM", targetIsokRpm);
  }
  Serial.printf(
    " | step=%ld/%ld (%.2f deg, cmd=%.2f deg) | pedal=%.1f RPM (%.1f km/h, fly=%.1f RPM, dir=%+d) | "
    "tau=%.1f Nm (eddy=%.1f Nm, F=%.1f N) P=%.1f W W=%.0f J revs=%.2f | dob=%s lpf=%.2f glitches=%lu\n",
    (long)currentSteps,
    (long)targetSteps,
    stepsToAngleDeg(currentSteps),
    alphaTargetDeg,
    pedalRpm,
    speedKmh,
    flywheelRpm,
    (int)dirSign,
    tauPedalEst,
    tauEddyFly * GEAR_PEDAL_TO_FLY,
    forcePedalEst,
    powerUserW,
    totalWorkJoules,
    completedPedalRevs,
    cfg.dobEnabled ? "ON" : "OFF",
    cfg.lpfAlpha,
    (unsigned long)glitchRejectCount
  );
}

void printHelp() {
  Serial.println("\n--- DOULA Cycling Modes U3 (ESP-NOW + Serial @ 115200 baud) ---");
  Serial.println("  manual | m0           : Switch to Manual step/angle mode");
  Serial.println("  torque [Nm] | m1 [Nm] : Constant Torque mode (2..85 Nm) & start");
  Serial.println("  power [W]   | m2 [W]  : Constant Power mode (15..250 W) & start");
  Serial.println("  isok [RPM]  | m3 [RPM]: Isokinetic Cadence mode (15..100 RPM) & start");
  Serial.println("  start | s             : Start active mode (22.5 deg Manual; 5.0 deg standby Closed-Loop)");
  Serial.println("  end | e | stop        : End session & return stepper smoothly to 0.00 deg");
  Serial.println("  + / -                 : Adjust setpoint (Manual +-1 step, Torque +-2 Nm, Power +-10 W, Isok +-5 RPM)");
  Serial.println("  angle <deg>           : Switch to Manual & move directly to <deg> (0..42)");
  Serial.println("  home | 0              : Move stepper to 0.00 deg");
  Serial.println("  zero                  : Zero current physical stepper position & clear encoder totals");
  Serial.println("  status | ?            : Print full status summary & Master link info");
  Serial.println("  enc                   : Print encoder diagnostics (PCNT raw, total counts, RPM, glitches)");
  Serial.println("  log [on|off]          : Toggle 4 Hz live Serial Monitor telemetry");
  Serial.println("  plot                  : Toggle 10 Hz Arduino Serial Plotter output");
  Serial.println("  lpf <0.05..1.0>       : Set velocity EMA low-pass alpha");
  Serial.println("  dob on|off            : Enable/disable Luenberger DOB (off = speed-only anti-stall)");
  Serial.println("  encpol [1|-1]         : Toggle or set encoder forward polarity");
  Serial.println("  keddy <val>           : Set eddy constant K_eddy_max (default 0.715)");
  Serial.println("  fric <c0> <bvisc>     : Set friction constants (default 0.18 0.004)");
  Serial.println("  pi <kp> <ki>          : Set Isokinetic PI gains (default 1.2 0.8)");
  Serial.println("  save | resetcal       : Save tuning to NVS flash / restore defaults");
  Serial.println("-------------------------------------------------------------\n");
}

// ============================================================================
// 5. HARDWARE PCNT 4X QUADRATURE ENCODER (GPIO32 / GPIO33)
// ============================================================================
void initEncoderPCNT() {
  pinMode(PIN_ENC2_A, INPUT_PULLUP);
  pinMode(PIN_ENC2_B, INPUT_PULLUP);

  pcnt_config_t ch0{};
  ch0.pulse_gpio_num = PIN_ENC2_A;
  ch0.ctrl_gpio_num  = PIN_ENC2_B;
  ch0.unit           = ENC_PCNT_UNIT;
  ch0.channel        = PCNT_CHANNEL_0;
  ch0.pos_mode       = PCNT_COUNT_INC;
  ch0.neg_mode       = PCNT_COUNT_DEC;
  ch0.lctrl_mode     = PCNT_MODE_REVERSE;
  ch0.hctrl_mode     = PCNT_MODE_KEEP;
  ch0.counter_h_lim  = PCNT_HIGH_LIMIT;
  ch0.counter_l_lim  = PCNT_LOW_LIMIT;
  pcnt_unit_config(&ch0);

  pcnt_config_t ch1{};
  ch1.pulse_gpio_num = PIN_ENC2_B;
  ch1.ctrl_gpio_num  = PIN_ENC2_A;
  ch1.unit           = ENC_PCNT_UNIT;
  ch1.channel        = PCNT_CHANNEL_1;
  ch1.pos_mode       = PCNT_COUNT_DEC;
  ch1.neg_mode       = PCNT_COUNT_INC;
  ch1.lctrl_mode     = PCNT_MODE_REVERSE;
  ch1.hctrl_mode     = PCNT_MODE_KEEP;
  ch1.counter_h_lim  = PCNT_HIGH_LIMIT;
  ch1.counter_l_lim  = PCNT_LOW_LIMIT;
  pcnt_unit_config(&ch1);

  // Re-enable internal pull-ups after pcnt_unit_config() for NPN open-collector lines (IDF 5.x)
  gpio_pullup_en((gpio_num_t)PIN_ENC2_A);
  gpio_pullup_en((gpio_num_t)PIN_ENC2_B);

  pcnt_set_filter_value(ENC_PCNT_UNIT, PCNT_GLITCH_FILTER_APB);
  pcnt_filter_enable(ENC_PCNT_UNIT);

  pcnt_counter_pause(ENC_PCNT_UNIT);
  pcnt_counter_clear(ENC_PCNT_UNIT);
  pcnt_counter_resume(ENC_PCNT_UNIT);
  prevPcntRaw = 0;
}

int32_t readEncoderDelta() {
  int16_t rawCnt = 0;
  pcnt_get_counter_value(ENC_PCNT_UNIT, &rawCnt);
  int32_t delta = (int32_t)rawCnt - (int32_t)prevPcntRaw;
  prevPcntRaw   = rawCnt;

  if (abs(rawCnt) > PCNT_WRAP_RESET_THRESH) {
    pcnt_counter_pause(ENC_PCNT_UNIT);
    pcnt_counter_clear(ENC_PCNT_UNIT);
    pcnt_counter_resume(ENC_PCNT_UNIT);
    prevPcntRaw = 0;
  }

  if (abs(delta) > MAX_VALID_DELTA_10MS) {
    glitchRejectCount++;
    return 0;
  }

  totalEncCounts += (int64_t)delta * (int64_t)cfg.encPolarity;
  completedPedalRevs = fabsf((float)totalEncCounts) / (ENC_CPR_4X * GEAR_PEDAL_TO_FLY);
  return delta;
}

void resetStepperAndEncoderZero(const char* source) {
  currentSteps       = 0;
  targetSteps        = 0;
  alphaTargetDeg     = 0.0f;
  alphaCommandDeg    = 0.0f;
  pulsePhase         = PULSE_IDLE;
  digitalWrite(STEP_PIN, LOW);
  pcnt_counter_pause(ENC_PCNT_UNIT);
  pcnt_counter_clear(ENC_PCNT_UNIT);
  pcnt_counter_resume(ENC_PCNT_UNIT);
  prevPcntRaw        = 0;
  totalEncCounts     = 0;
  completedPedalRevs = 0.0f;
  totalWorkJoules    = 0.0f;
  Serial.printf("[%s] Stepper zeroed at 0.00 deg & encoder counters reset\n", source);
  sendTelemetryToMaster();
}

// ============================================================================
// 6. EDDY BRAKE PHYSICS & MODE CONTROL LAWS (100 HZ)
// ============================================================================
float computeEddyTorqueFly(float alphaDeg, float omegaFly) {
  if (omegaFly <= 0.05f) return 0.0f;
  float alphaRad = clampf(alphaDeg, 0.0f, 45.0f) * ((float)M_PI / 180.0f);
  float sinTerm  = sinf(2.0f * alphaRad);
  float uAlpha   = sinTerm * sinTerm;
  float ratio    = omegaFly / OMEGA_CRIT;
  float kloss    = omegaFly / (1.0f + ratio * ratio);
  return cfg.kEddyMax * uAlpha * kloss;
}

float invertEddyTorqueToAlphaDeg(float tauEddyReq, float omegaFly) {
  if (tauEddyReq <= 0.0f) return 0.0f;
  float tauEddyMaxNow = computeEddyTorqueFly(45.0f, omegaFly);
  if (tauEddyMaxNow <= 0.05f) return STANDBY_ANGLE_DEG;
  float uCmd = clampf(tauEddyReq / tauEddyMaxNow, 0.0f, 1.0f);
  float alphaDeg = 0.5f * asinf(sqrtf(uCmd)) * (180.0f / (float)M_PI);
  return clampf(alphaDeg, MIN_ANGLE_DEG, MAX_ANGLE_DEG);
}

float computeModeTargetAlphaDeg(float dt, float omegaFly, float tauLossFly) {
  if (activeMode == MODE_CONST_TORQUE) {
    float tauEddyReq = (targetTorqueNm / GEAR_PEDAL_TO_FLY) - tauLossFly;
    return invertEddyTorqueToAlphaDeg(tauEddyReq, omegaFly);
  }

  if (activeMode == MODE_CONST_POWER) {
    float safeOmega  = fmaxf(omegaFly, 3.0f);
    float tauEddyReq = (targetPowerW / safeOmega) - tauLossFly;
    return invertEddyTorqueToAlphaDeg(tauEddyReq, omegaFly);
  }

  if (activeMode == MODE_ISOKINETIC) {
    float omegaTarget = GEAR_PEDAL_TO_FLY * (targetIsokRpm * (2.0f * (float)M_PI / 60.0f));
    float err         = omegaFly - omegaTarget; // Positive error = pedaling faster than clamp -> add brake
    float tauMaxFly   = computeEddyTorqueFly(MAX_ANGLE_DEG, omegaFly);

    float pTerm       = cfg.isokKp * err;
    float candidate   = pTerm + cfg.isokKi * isokIntegral;
    bool  satHigh     = (candidate >= tauMaxFly) && (err > 0.0f);
    bool  satLow      = (candidate <= 0.0f) && (err < 0.0f);
    if (!satHigh && !satLow) {
      isokIntegral += err * dt;
    }

    float tauEddyReq = clampf(pTerm + cfg.isokKi * isokIntegral, 0.0f, tauMaxFly);
    return invertEddyTorqueToAlphaDeg(tauEddyReq, omegaFly);
  }

  return stepsToAngleDeg(targetSteps);
}

void updateCyclingControl(float dt) {
  int32_t deltaCounts = readEncoderDelta();

  // 1. Velocity estimation + lightweight EMA low-pass filter
  omegaSignedRaw  = ((float)deltaCounts * (float)cfg.encPolarity * RAD_PER_COUNT) / dt;
  omegaSignedFilt = cfg.lpfAlpha * omegaSignedRaw + (1.0f - cfg.lpfAlpha) * omegaSignedFilt;
  if (deltaCounts == 0 && fabsf(omegaSignedFilt) < 0.02f) {
    omegaSignedFilt = 0.0f;
  }

  omegaMag    = fabsf(omegaSignedFilt);
  flywheelRpm = omegaMag * (60.0f / (2.0f * (float)M_PI));
  pedalRpm    = flywheelRpm / GEAR_PEDAL_TO_FLY;
  speedKmh    = pedalRpm * SPEED_KMH_PER_PEDAL_RPM;

  // 2. Eddy brake & parasitic loss evaluation at actual stator disk angle
  float actualAlphaDeg = stepsToAngleDeg(currentSteps);
  tauEddyFly           = computeEddyTorqueFly(actualAlphaDeg, omegaMag);
  float tauLossFly     = (omegaMag > 0.1f) ? (cfg.tauC0Fric + cfg.bViscFric * omegaMag) : 0.0f;

  // 3. Luenberger Momentum Disturbance Observer (always updated for live telemetry)
  if (omegaMag < 0.05f) {
    zObs *= 0.7f;
    if (fabsf(zObs) < 1e-4f) zObs = 0.0f;
  } else {
    float dz = -OBS_BW_RAD_S * zObs -
               OBS_BW_RAD_S * (OBS_BW_RAD_S * J_FLY_EFF * omegaMag - tauEddyFly - tauLossFly);
    zObs += dz * dt;
  }

  float tauUserFlyEst = zObs + OBS_BW_RAD_S * J_FLY_EFF * omegaMag;
  tauPedalEst         = GEAR_PEDAL_TO_FLY * tauUserFlyEst;
  forcePedalEst       = tauPedalEst / CRANK_LENGTH_M;
  float omegaPedal    = omegaMag / GEAR_PEDAL_TO_FLY;
  powerUserW          = (tauPedalEst > 0.0f) ? (tauPedalEst * omegaPedal) : 0.0f;

  // 4. Manual or stopped/estopped mode leaves direct stepper control untouched
  if (estopped || !sessionActive || activeMode == MODE_MANUAL) {
    if (estopped || !sessionActive) {
      ctrlState       = CTRL_STOPPED;
      alphaTargetDeg  = MIN_ANGLE_DEG;
      alphaCommandDeg = stepsToAngleDeg(targetSteps);
    } else {
      ctrlState       = CTRL_ACTIVE_CONTROL;
      alphaTargetDeg  = stepsToAngleDeg(targetSteps);
      alphaCommandDeg = alphaTargetDeg;
      if (tauPedalEst >= DOB_PEDAL_ON_NM) {
        totalWorkJoules += powerUserW * dt;
      }
    }
    return;
  }

  // 5. Anti-stall pedaling detection (DOB effort-gated or speed-only fallback)
  bool startEffort = false;
  bool coastNow    = false;
  if (cfg.dobEnabled) {
    startEffort = (tauPedalEst >= DOB_PEDAL_ON_NM) && (omegaMag > 1.5f);
    coastNow    = (tauPedalEst < DOB_PEDAL_OFF_NM) || (omegaMag <= 0.8f);
  } else {
    startEffort = (pedalRpm >= SPEED_ONLY_ON_RPM);
    coastNow    = (pedalRpm < SPEED_ONLY_OFF_RPM);
  }

  if (coastNow) {
    coastTimerSec += dt;
  } else {
    coastTimerSec = 0.0f;
  }
  bool coastDebounced = (coastTimerSec >= COAST_DEBOUNCE_S);

  // 6. State machine transitions + mode target angle computation
  switch (ctrlState) {
    case CTRL_STOPPED:
      ctrlState = CTRL_IDLE_READY;
      // fall through into standby
    case CTRL_IDLE_READY:
      alphaTargetDeg = STANDBY_ANGLE_DEG;
      if (startEffort) {
        ctrlState      = CTRL_SOFT_START_RAMP;
        rampStartAlpha = alphaCommandDeg;
        rampElapsedSec = 0.0f;
        isokIntegral   = 0.0f;
        coastTimerSec  = 0.0f;
        Serial.println("[CTRL] Pedaling detected -> Soft-start ramp");
      }
      break;

    case CTRL_SOFT_START_RAMP:
      if (coastDebounced) {
        ctrlState      = CTRL_COAST_UNLOAD;
        isokIntegral   = 0.0f;
        alphaTargetDeg = STANDBY_ANGLE_DEG;
        Serial.println("[CTRL] Coast detected -> Unloading brake to 5.0 deg standby");
      } else {
        float lawAlpha = computeModeTargetAlphaDeg(dt, omegaMag, tauLossFly);
        rampElapsedSec += dt;
        float u = clampf(rampElapsedSec / SOFT_START_RAMP_S, 0.0f, 1.0f);
        alphaTargetDeg = rampStartAlpha + u * (lawAlpha - rampStartAlpha);
        if (tauPedalEst >= DOB_PEDAL_ON_NM) {
          totalWorkJoules += powerUserW * dt;
        }
        if (rampElapsedSec >= SOFT_START_RAMP_S) {
          ctrlState = CTRL_ACTIVE_CONTROL;
          Serial.println("[CTRL] Ramp complete -> Active closed-loop control");
        }
      }
      break;

    case CTRL_ACTIVE_CONTROL:
      if (coastDebounced) {
        ctrlState      = CTRL_COAST_UNLOAD;
        isokIntegral   = 0.0f;
        alphaTargetDeg = STANDBY_ANGLE_DEG;
        Serial.println("[CTRL] Coast detected -> Unloading brake to 5.0 deg standby");
      } else {
        alphaTargetDeg = computeModeTargetAlphaDeg(dt, omegaMag, tauLossFly);
        if (tauPedalEst >= DOB_PEDAL_ON_NM) {
          totalWorkJoules += powerUserW * dt;
        }
      }
      break;

    case CTRL_COAST_UNLOAD:
      alphaTargetDeg = STANDBY_ANGLE_DEG;
      if (startEffort) {
        ctrlState      = CTRL_SOFT_START_RAMP;
        rampStartAlpha = alphaCommandDeg;
        rampElapsedSec = 0.0f;
        isokIntegral   = 0.0f;
        coastTimerSec  = 0.0f;
        Serial.println("[CTRL] Pedaling resumed -> Soft-start ramp");
      } else if (omegaMag < 0.5f) {
        ctrlState = CTRL_IDLE_READY;
      }
      break;
  }

  // 7. Slew-rate limit & anti-chatter step deadband
  float maxDeltaAlpha = MAX_DISK_SLEW_DEG_S * dt;
  if (alphaTargetDeg > alphaCommandDeg + maxDeltaAlpha) {
    alphaCommandDeg += maxDeltaAlpha;
  } else if (alphaTargetDeg < alphaCommandDeg - maxDeltaAlpha) {
    alphaCommandDeg -= maxDeltaAlpha;
  } else {
    alphaCommandDeg = alphaTargetDeg;
  }

  int32_t desiredSteps = clampSteps(lroundf(alphaCommandDeg / DEG_PER_STEP));
  if (ctrlState == CTRL_IDLE_READY || ctrlState == CTRL_COAST_UNLOAD) {
    targetSteps = desiredSteps;
  } else if (abs(desiredSteps - targetSteps) >= CLOSED_LOOP_STEP_DEADBAND ||
             desiredSteps == MIN_STEPS || desiredSteps == MAX_STEPS) {
    targetSteps = desiredSteps;
  }
}

// ============================================================================
// 7. SESSION & MODE MANAGEMENT HELPERS
// ============================================================================
void startSessionForCurrentMode(const char* source, SessionSource srcEnum) {
  estopped        = false;
  sessionActive   = true;
  sessionSource   = srcEnum;
  coastTimerSec   = 0.0f;
  rampElapsedSec  = 0.0f;
  isokIntegral    = 0.0f;
  totalWorkJoules = 0.0f;

  if (activeMode == MODE_MANUAL) {
    ctrlState       = CTRL_ACTIVE_CONTROL;
    targetSteps     = DEFAULT_START_STEPS;
    alphaTargetDeg  = stepsToAngleDeg(targetSteps);
    alphaCommandDeg = alphaTargetDeg;
    Serial.printf("[%s] START MANUAL -> moving to %.2f deg (%ld steps)\n",
                  source, alphaTargetDeg, (long)targetSteps);
  } else {
    ctrlState       = CTRL_IDLE_READY;
    targetSteps     = STANDBY_STEPS;
    alphaTargetDeg  = STANDBY_ANGLE_DEG;
    alphaCommandDeg = stepsToAngleDeg(currentSteps);
    Serial.printf("[%s] START %s -> parking at %.1f deg standby, pedal to engage\n",
                  source, modeName(activeMode), STANDBY_ANGLE_DEG);
  }
  sendTelemetryToMaster();
}

void endSession(const char* source) {
  sessionActive   = false;
  sessionSource   = SRC_NONE;
  ctrlState       = CTRL_STOPPED;
  targetSteps     = MIN_STEPS;
  alphaTargetDeg  = MIN_ANGLE_DEG;
  alphaCommandDeg = stepsToAngleDeg(currentSteps);
  isokIntegral    = 0.0f;
  coastTimerSec   = 0.0f;
  Serial.printf("[%s] SESSION END -> returning to 0.00 deg (step 0) from %.2f deg (%ld steps)\n",
                source, stepsToAngleDeg(currentSteps), (long)currentSteps);
  sendTelemetryToMaster();
}

void adjustActiveTarget(int sign, const char* source) {
  if (activeMode == MODE_MANUAL) {
    targetSteps     = clampSteps(targetSteps + sign);
    alphaTargetDeg  = stepsToAngleDeg(targetSteps);
    alphaCommandDeg = alphaTargetDeg;
    Serial.printf("[%s] %s1 step -> target=%ld steps (%.2f deg) | current=%ld steps (%.2f deg)\n",
                  source, (sign > 0) ? "+" : "-",
                  (long)targetSteps, alphaTargetDeg,
                  (long)currentSteps, stepsToAngleDeg(currentSteps));
  } else if (activeMode == MODE_CONST_TORQUE) {
    targetTorqueNm = clampf(targetTorqueNm + sign * STEP_TORQUE_NM, MIN_TORQUE_NM, MAX_TORQUE_NM);
    Serial.printf("[%s] Target torque = %.1f Nm\n", source, targetTorqueNm);
  } else if (activeMode == MODE_CONST_POWER) {
    targetPowerW = clampf(targetPowerW + sign * STEP_POWER_W, MIN_POWER_W, MAX_POWER_W);
    Serial.printf("[%s] Target power = %.0f W\n", source, targetPowerW);
  } else if (activeMode == MODE_ISOKINETIC) {
    targetIsokRpm = clampf(targetIsokRpm + sign * STEP_ISOK_RPM, MIN_ISOK_RPM, MAX_ISOK_RPM);
    Serial.printf("[%s] Target cadence = %.0f RPM\n", source, targetIsokRpm);
  }
  sendTelemetryToMaster();
}

// ============================================================================
// 8. ESP-NOW RECEIVE, PEER MANAGEMENT & WATCHDOG
// ============================================================================
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

void applyMasterTargets(const RehabCommandPacket_t& pkt) {
  if (pkt.op_mode == OP_CYCLE_TORQUE && isfinite(pkt.target_torque_ext) && pkt.target_torque_ext >= MIN_TORQUE_NM) {
    float nextTq = clampf(pkt.target_torque_ext, MIN_TORQUE_NM, MAX_TORQUE_NM);
    if (fabsf(nextTq - targetTorqueNm) >= 0.05f) {
      targetTorqueNm = nextTq;
      Serial.printf("[ESP-NOW] Target torque -> %.1f Nm\n", targetTorqueNm);
    }
  } else if (pkt.op_mode == OP_CYCLE_POWER && isfinite(pkt.target_torque_flex) && pkt.target_torque_flex >= MIN_POWER_W) {
    float nextPwr = clampf(pkt.target_torque_flex, MIN_POWER_W, MAX_POWER_W);
    if (fabsf(nextPwr - targetPowerW) >= 0.5f) {
      targetPowerW = nextPwr;
      Serial.printf("[ESP-NOW] Target power -> %.0f W\n", targetPowerW);
    }
  } else if (pkt.op_mode == OP_CYCLE_ISOKINETIC && isfinite(pkt.target_vel_deg_s) && pkt.target_vel_deg_s >= MIN_ISOK_RPM) {
    float nextRpm = clampf(pkt.target_vel_deg_s, MIN_ISOK_RPM, MAX_ISOK_RPM);
    if (fabsf(nextRpm - targetIsokRpm) >= 0.25f) {
      targetIsokRpm = nextRpm;
      Serial.printf("[ESP-NOW] Target cadence -> %.0f RPM\n", targetIsokRpm);
    }
  }
}

void processEspNowQueue() {
  if (cmdQueue == nullptr) return;
  QueuedCommand item{};
  while (xQueueReceive(cmdQueue, &item, 0) == pdTRUE) {
    lastMasterRxMs   = millis();
    bool newlyLinked = !masterSynced;
    masterSynced     = true;

    if (item.pkt.wifi_channel >= 1 && item.pkt.wifi_channel <= MAX_WIFI_CHANNEL &&
        item.pkt.wifi_channel != currentWifiChannel) {
      currentWifiChannel = item.pkt.wifi_channel;
      esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
      Serial.printf("[ESP-NOW] Locked Wi-Fi channel %u from Master\n", (unsigned)currentWifiChannel);
    }

    ensureMasterPeer(item.srcMac);

    // Detect Master reboot so op-mode transitions re-sync cleanly
    if (hasMasterOpState && item.pkt.boot_id != lastMasterBootId) {
      hasMasterOpState = false;
    }

    // 1. E-Stop Arbitration (highest priority — overrides both ESP-NOW and Serial)
    if (item.pkt.estop_active == ESTOP_HARD_STOP_LOCKED ||
        item.pkt.estop_active == ESTOP_HARD_STOP_RELEASE) {
      bool changed = !estopped || sessionActive || (targetSteps != MIN_STEPS);
      estopped        = true;
      sessionActive   = false;
      sessionSource   = SRC_NONE;
      ctrlState       = CTRL_STOPPED;
      targetSteps     = MIN_STEPS;
      alphaTargetDeg  = MIN_ANGLE_DEG;
      alphaCommandDeg = stepsToAngleDeg(currentSteps);
      isokIntegral    = 0.0f;
      if (changed) {
        Serial.println("[ESP-NOW] E-STOP -> unloading brake to 0.00 deg");
        sendTelemetryToMaster();
      }
      hasMasterOpState = true;
      lastMasterOpMode = item.pkt.op_mode;
      lastMasterBootId = item.pkt.boot_id;
      continue;
    }

    if (estopped && item.pkt.estop_active == ESTOP_NORMAL && item.pkt.op_mode == OP_IDLE) {
      estopped = false;
      Serial.println("[ESP-NOW] E-Stop cleared -> IDLE");
      sendTelemetryToMaster();
    }

    if (estopped) continue;

    // 2. Mode & Session Sync (preserves standalone Serial sessions when Master stays OP_IDLE)
    if (item.pkt.target_node == NODE_CYCLING || item.pkt.op_mode == OP_IDLE) {
      bool masterOpChanged    = !hasMasterOpState || (item.pkt.op_mode != lastMasterOpMode);
      bool masterWantsCycling = isCyclingOpMode(item.pkt.op_mode);

      if (masterWantsCycling) {
        CyclingMode desiredMode = opModeToCyclingMode(item.pkt.op_mode);
        if (masterOpChanged || !sessionActive || sessionSource != SRC_ESPNOW || activeMode != desiredMode) {
          activeMode = desiredMode;
          applyMasterTargets(item.pkt);
          startSessionForCurrentMode("ESP-NOW", SRC_ESPNOW);
        } else {
          // Same active ESP-NOW cycling mode: smoothly update live setpoints
          applyMasterTargets(item.pkt);
        }
      } else if (item.pkt.op_mode == OP_IDLE) {
        if (sessionActive && sessionSource == SRC_ESPNOW) {
          endSession("ESP-NOW");
        }
      }

      hasMasterOpState = true;
      lastMasterOpMode = item.pkt.op_mode;
      lastMasterBootId = item.pkt.boot_id;
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
            adjustActiveTarget(1, "ESP-NOW");
            break;
          case STEP_ACT_MINUS:
            adjustActiveTarget(-1, "ESP-NOW");
            break;
          case STEP_ACT_HOME:
            activeMode      = MODE_MANUAL;
            sessionActive   = false;
            sessionSource   = SRC_NONE;
            ctrlState       = CTRL_STOPPED;
            targetSteps     = MIN_STEPS;
            alphaTargetDeg  = MIN_ANGLE_DEG;
            alphaCommandDeg = stepsToAngleDeg(currentSteps);
            Serial.println("[ESP-NOW] HOME -> returning to 0.00 deg");
            sendTelemetryToMaster();
            break;
          case STEP_ACT_ZERO:
            resetStepperAndEncoderZero("ESP-NOW");
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

void serviceEspNowChannelAndWatchdog() {
  uint32_t now = millis();

  // Retract brake safely if Master goes silent mid-session (only for ESP-NOW started sessions)
  if (sessionActive && sessionSource == SRC_ESPNOW && masterSynced &&
      (now - lastMasterRxMs > ACTIVE_WATCHDOG_MS)) {
    endSession("SAFETY WATCHDOG");
    Serial.println("[SAFETY] Master heartbeat lost (>1500 ms) — retracted brake to 0.00 deg");
  }

  if (masterSynced && (now - lastMasterRxMs > MASTER_TIMEOUT_MS)) {
    masterSynced     = false;
    hasMasterOpState = false;
    Serial.println("[ESP-NOW] Master heartbeat timeout — scanning channels 1..13...");
  }

  if (!masterSynced && (now - lastChannelHopMs >= CHANNEL_HOP_MS)) {
    lastChannelHopMs   = now;
    currentWifiChannel = (currentWifiChannel % MAX_WIFI_CHANNEL) + 1;
    esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);
  }

  uint32_t telemPeriod = (sessionActive || pedalRpm > 0.5f || wasMoving) ? TELEM_ACTIVE_MS : TELEM_IDLE_MS;
  if (masterSynced && masterPeerAdded && (now - lastTelemTxMs >= telemPeriod)) {
    sendTelemetryToMaster();
  }
}

// ============================================================================
// 9. SERIAL MONITOR COMMAND PARSER (100% PRESERVED)
// ============================================================================
bool parseNextFloat(const String& line, const char* prefix, float& outVal) {
  int prefixLen = (int)strlen(prefix);
  if ((int)line.length() <= prefixLen) return false;
  String rest = line.substring(prefixLen);
  rest.trim();
  if (rest.length() == 0) return false;
  outVal = rest.toFloat();
  return true;
}

void executeSerialCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  String lower = line;
  lower.toLowerCase();

  if (lower == "help") {
    printHelp();
  } else if (lower == "?" || lower == "status") {
    printStatus();
  } else if (lower == "enc") {
    int16_t raw = 0;
    pcnt_get_counter_value(ENC_PCNT_UNIT, &raw);
    Serial.printf("[ENC] pcnt_raw=%d total_counts=%lld polarity=%+d | fly=%.1f RPM pedal=%.1f RPM (%.1f km/h) | revs=%.2f | glitches=%lu\n",
                  (int)raw, (long long)totalEncCounts, (int)cfg.encPolarity,
                  flywheelRpm, pedalRpm, speedKmh, completedPedalRevs, (unsigned long)glitchRejectCount);
  } else if (lower == "manual" || lower == "m0") {
    activeMode = MODE_MANUAL;
    Serial.println("[MODE] Switched to MANUAL");
    printStatus();
    sendTelemetryToMaster();
  } else if (lower == "torque" || lower == "m1" || lower.startsWith("torque ") || lower.startsWith("m1 ")) {
    activeMode = MODE_CONST_TORQUE;
    float val = targetTorqueNm;
    const char* pfx = lower.startsWith("torque") ? "torque" : "m1";
    if (parseNextFloat(line, pfx, val)) {
      targetTorqueNm = clampf(val, MIN_TORQUE_NM, MAX_TORQUE_NM);
    }
    startSessionForCurrentMode("Serial", SRC_SERIAL);
  } else if (lower == "power" || lower == "m2" || lower.startsWith("power ") || lower.startsWith("m2 ")) {
    activeMode = MODE_CONST_POWER;
    float val = targetPowerW;
    const char* pfx = lower.startsWith("power") ? "power" : "m2";
    if (parseNextFloat(line, pfx, val)) {
      targetPowerW = clampf(val, MIN_POWER_W, MAX_POWER_W);
    }
    startSessionForCurrentMode("Serial", SRC_SERIAL);
  } else if (lower == "isok" || lower == "m3" || lower.startsWith("isok ") || lower.startsWith("m3 ")) {
    activeMode = MODE_ISOKINETIC;
    float val = targetIsokRpm;
    const char* pfx = lower.startsWith("isok") ? "isok" : "m3";
    if (parseNextFloat(line, pfx, val)) {
      targetIsokRpm = clampf(val, MIN_ISOK_RPM, MAX_ISOK_RPM);
    }
    startSessionForCurrentMode("Serial", SRC_SERIAL);
  } else if (lower == "s" || lower == "start") {
    startSessionForCurrentMode("Serial", SRC_SERIAL);
  } else if (lower == "e" || lower == "end" || lower == "stop") {
    endSession("Serial");
  } else if (lower == "0" || lower == "h" || lower == "home") {
    activeMode      = MODE_MANUAL;
    sessionActive   = false;
    sessionSource   = SRC_NONE;
    ctrlState       = CTRL_STOPPED;
    targetSteps     = MIN_STEPS;
    alphaTargetDeg  = MIN_ANGLE_DEG;
    alphaCommandDeg = stepsToAngleDeg(currentSteps);
    Serial.printf("[Serial] Moving to 0.00 deg (step 0) from %.2f deg\n", stepsToAngleDeg(currentSteps));
    sendTelemetryToMaster();
  } else if (lower == "zero") {
    resetStepperAndEncoderZero("Serial");
  } else if (lower.startsWith("angle ")) {
    float deg = 0.0f;
    if (parseNextFloat(line, "angle", deg)) {
      activeMode      = MODE_MANUAL;
      sessionActive   = true;
      sessionSource   = SRC_SERIAL;
      estopped        = false;
      ctrlState       = CTRL_ACTIVE_CONTROL;
      targetSteps     = clampSteps(lroundf(deg / DEG_PER_STEP));
      alphaTargetDeg  = stepsToAngleDeg(targetSteps);
      alphaCommandDeg = alphaTargetDeg;
      Serial.printf("[MANUAL] Target set to %.2f deg (%ld steps)\n", alphaTargetDeg, (long)targetSteps);
      sendTelemetryToMaster();
    }
  } else if (lower == "log") {
    logEnabled = !logEnabled;
    if (logEnabled) plotEnabled = false;
    Serial.printf("[Serial] Live log %s\n", logEnabled ? "ON (4 Hz)" : "OFF");
  } else if (lower == "log on") {
    logEnabled = true;
    plotEnabled = false;
    Serial.println("[Serial] Live log ON (4 Hz)");
  } else if (lower == "log off") {
    logEnabled = false;
    Serial.println("[Serial] Live log OFF");
  } else if (lower == "plot") {
    plotEnabled = !plotEnabled;
    if (plotEnabled) logEnabled = false;
    Serial.printf("[Serial] Plotter stream %s\n", plotEnabled ? "ON (10 Hz)" : "OFF");
  } else if (lower.startsWith("lpf ")) {
    float val = cfg.lpfAlpha;
    if (parseNextFloat(line, "lpf", val)) {
      cfg.lpfAlpha = clampf(val, 0.05f, 1.0f);
      Serial.printf("[CFG] lpfAlpha = %.2f\n", cfg.lpfAlpha);
    }
  } else if (lower == "dob on") {
    cfg.dobEnabled = true;
    Serial.println("[CFG] DOB effort-gated anti-stall = ON");
  } else if (lower == "dob off") {
    cfg.dobEnabled = false;
    Serial.println("[CFG] DOB = OFF (using speed-only anti-stall >= 10 RPM)");
  } else if (lower == "encpol") {
    cfg.encPolarity = (cfg.encPolarity > 0) ? -1 : 1;
    Serial.printf("[CFG] encPolarity = %+d\n", (int)cfg.encPolarity);
  } else if (lower.startsWith("encpol ")) {
    float val = 1.0f;
    if (parseNextFloat(line, "encpol", val)) {
      cfg.encPolarity = (val >= 0.0f) ? 1 : -1;
      Serial.printf("[CFG] encPolarity = %+d\n", (int)cfg.encPolarity);
    }
  } else if (lower.startsWith("keddy ")) {
    float val = cfg.kEddyMax;
    if (parseNextFloat(line, "keddy", val)) {
      cfg.kEddyMax = clampf(val, 0.05f, 10.0f);
      Serial.printf("[CFG] kEddyMax = %.4f\n", cfg.kEddyMax);
    }
  } else if (lower.startsWith("fric ")) {
    String rest = line.substring(5);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp > 0) {
      float c0 = rest.substring(0, sp).toFloat();
      float bv = rest.substring(sp + 1).toFloat();
      cfg.tauC0Fric = clampf(c0, 0.0f, 5.0f);
      cfg.bViscFric = clampf(bv, 0.0f, 1.0f);
      Serial.printf("[CFG] tauC0Fric=%.3f, bViscFric=%.4f\n", cfg.tauC0Fric, cfg.bViscFric);
    } else {
      Serial.println("[CFG] Usage: fric <c0> <bvisc> (e.g. fric 0.18 0.004)");
    }
  } else if (lower.startsWith("pi ")) {
    String rest = line.substring(3);
    rest.trim();
    int sp = rest.indexOf(' ');
    if (sp > 0) {
      float kp = rest.substring(0, sp).toFloat();
      float ki = rest.substring(sp + 1).toFloat();
      cfg.isokKp = clampf(kp, 0.0f, 20.0f);
      cfg.isokKi = clampf(ki, 0.0f, 20.0f);
      Serial.printf("[CFG] isokKp=%.3f, isokKi=%.3f\n", cfg.isokKp, cfg.isokKi);
    } else {
      Serial.println("[CFG] Usage: pi <kp> <ki> (e.g. pi 1.2 0.8)");
    }
  } else if (lower == "save") {
    saveConfigToNVS();
    Serial.println("[NVS] Configuration saved to flash");
  } else if (lower == "resetcal") {
    cfg = defaultConfig();
    saveConfigToNVS();
    Serial.println("[NVS] Factory defaults restored and saved");
  } else {
    Serial.printf("[Serial] Unknown command '%s'. Type 'help' for command list.\n", line.c_str());
  }
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    lastSerialByteMs = millis();

    if (ch == '+') {
      if (serialWordBuf.length() > 0) {
        executeSerialCommand(serialWordBuf);
        serialWordBuf = "";
      }
      adjustActiveTarget(1, "Serial");
    } else if (ch == '-' && serialWordBuf.length() == 0) {
      // Only treat standalone '-' as step/setpoint decrement (preserve negative numbers after command names)
      adjustActiveTarget(-1, "Serial");
    } else if (ch == '\n' || ch == '\r') {
      if (serialWordBuf.length() > 0) {
        executeSerialCommand(serialWordBuf);
        serialWordBuf = "";
      }
    } else {
      serialWordBuf += ch;
    }
  }

  // Support Arduino Serial Monitor "No line ending" mode after 100 ms idle
  if (serialWordBuf.length() > 0 && (millis() - lastSerialByteMs >= 100)) {
    executeSerialCommand(serialWordBuf);
    serialWordBuf = "";
  }
}

// ============================================================================
// 10. NON-BLOCKING STEPPER PULSE ENGINE
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
        if (activeMode == MODE_MANUAL || !sessionActive) {
          Serial.printf("[POS] Reached target: %ld steps | angle=%.2f deg\n",
                        (long)currentSteps, stepsToAngleDeg(currentSteps));
        }
        sendTelemetryToMaster();
      }
    }
    return;
  }

  // PULSE_IDLE: emit next step if target differs
  if (currentSteps == targetSteps) {
    if (wasMoving) {
      wasMoving = false;
      sendTelemetryToMaster();
    }
    return;
  }

  steppingUp         = (targetSteps > currentSteps);
  uint8_t desiredDir = steppingUp ? STEP_DIR_UP : STEP_DIR_DOWN;
  if (desiredDir != currentDirState) {
    digitalWrite(DIR_PIN, desiredDir);
    currentDirState = desiredDir;
    delayMicroseconds(DIR_SETUP_US);
  }

  wasMoving = true;
  digitalWrite(STEP_PIN, HIGH);
  pulsePhase       = PULSE_HIGH;
  pulseTimestampUs = micros();
}

// ============================================================================
// 11. STREAM LOGGING & ARDUINO ENTRY POINTS
// ============================================================================
void serviceTelemetryStreams() {
  uint32_t nowMs = millis();

  if (logEnabled && (nowMs - lastLogMs >= LOG_PERIOD_MS)) {
    lastLogMs = nowMs;
    printStatus();
  }

  if (plotEnabled && (nowMs - lastPlotMs >= PLOT_PERIOD_MS)) {
    lastPlotMs = nowMs;
    Serial.printf("rpm:%.2f,speed:%.2f,target_alpha:%.2f,alpha:%.2f,torque:%.2f,power:%.2f\n",
                  pedalRpm, speedKmh, alphaTargetDeg, stepsToAngleDeg(currentSteps), tauPedalEst, powerUserW);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, STEP_DIR_UP);
  currentDirState = STEP_DIR_UP;

  loadConfigFromNVS();
  initEncoderPCNT();

  // Initialize ESP-NOW (Ch 1..13 auto-sync with Master U1)
  cmdQueue = xQueueCreate(16, sizeof(QueuedCommand));
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(currentWifiChannel, WIFI_SECOND_CHAN_NONE);

  Serial.println("\n=== DOULA Cycling Multi-Mode Controller U3 (ESP-NOW + Serial) ===");
  Serial.printf("Cycling MAC  : %s\n", WiFi.macAddress().c_str());
  Serial.printf("Stepper Pins : DIR=GPIO%d, PUL=GPIO%d | %.2f deg/step (0..%.1f deg = %ld..%ld steps)\n",
                DIR_PIN, STEP_PIN, DEG_PER_STEP, MAX_ANGLE_DEG, (long)MIN_STEPS, (long)MAX_STEPS);
  Serial.printf("Encoder Pins : ChA=GPIO%d, ChB=GPIO%d (PCNT 4x, %.0f CPR, glitch=%.1f us)\n",
                PIN_ENC2_A, PIN_ENC2_B, ENC_CPR_4X, (float)PCNT_GLITCH_FILTER_APB / 80.0f);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] ERROR: esp_now_init() failed (Serial control still active)");
  } else {
    esp_now_register_recv_cb(onEspNowRecv);
    Serial.println("[ESP-NOW] Listening for Master U1 (scanning Ch 1..13)...");
  }

  lastControlUs = micros();
  lastLogMs     = millis();
  lastPlotMs    = millis();

  printHelp();
  printStatus();
}

void loop() {
  processEspNowQueue();
  serviceEspNowChannelAndWatchdog();
  handleSerialInput();
  serviceStepper();

  uint32_t nowUs = micros();
  uint32_t elapsedUs = (uint32_t)(nowUs - lastControlUs);
  if (elapsedUs >= CONTROL_PERIOD_US) {
    lastControlUs = nowUs;
    float dt = clampf((float)elapsedUs * 1e-6f, 0.005f, 0.050f);
    updateCyclingControl(dt);
  }

  serviceTelemetryStreams();
}
