/*
 * DOULA Cycling Eddy-Brake Multi-Mode Controller — Standalone Serial Edition
 * Target Board : ESP32-S NodeMCU V1.1 (NodeMCU-32S / ESP32 Dev Module)
 * Stepper      : U301 (TB6600 / DM542 Common Cathode: DIR+ IO18, PUL+ IO19, PUL-/DIR- GND)
 * Encoder      : ENC2 (360 PPR NPN Open-Collector on Flywheel Shaft: Ch A IO32, Ch B IO33)
 *
 * Modes (selectable via Serial Monitor @ 115200 baud):
 *   1. Manual (base mode)   : START -> 22.5 deg (75 steps), +/- -> +/-1 step (0.30 deg), END -> 0.0 deg
 *   2. Constant Torque      : Regulates stator angle alpha in [0, 42 deg] for target pedal torque [Nm]
 *   3. Constant Power       : Regulates eddy brake for target mechanical power [W] (ergometer)
 *   4. Isokinetic Cadence   : PI cadence-clamping brake for target pedal RPM
 *
 * Noise Immunity & Safety:
 *   - Hardware ESP32 PCNT 4x quadrature decoding with 5.0 us APB glitch filter
 *   - Physical per-tick delta spike rejection + lightweight EMA velocity low-pass filter
 *   - Luenberger Momentum Disturbance Observer (DOB) + coasting auto-unload to 5 deg standby
 *   - 1.5 s soft-start ramp, 30 deg/s slew limiter, 2-step closed-loop deadband, NVS config storage
 */

#include <Arduino.h>
#include <Preferences.h>
#include <driver/pcnt.h>
#include <driver/gpio.h>
#include <math.h>
#include <string.h>

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
constexpr float   GEAR_PEDAL_TO_FLY = 5.0f;  // 1:5 ring gear to flywheel pinion
constexpr float   ENC_CPR_4X        = 1440.0f; // 360 PPR * 4x = 1440 counts/flywheel rev (7200/pedal rev)
constexpr float   RAD_PER_COUNT     = (2.0f * (float)M_PI) / ENC_CPR_4X;
constexpr float   J_FLY_EFF         = 0.01630f; // Effective flywheel inertia [kg*m^2]
constexpr float   OMEGA_CRIT        = 80.0f;    // Eddy inductive reaction speed [rad/s]
constexpr float   CRANK_LENGTH_M    = 0.170f;   // Pedal crank radius [m]
constexpr float   OBS_BW_RAD_S      = 37.7f;    // 6 Hz Luenberger DOB bandwidth [rad/s]

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
// 2. PERSISTENT TUNING CONFIGURATION (NVS)
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

enum StepPulsePhase : uint8_t {
  PULSE_IDLE = 0,
  PULSE_HIGH = 1,
  PULSE_LOW  = 2
};

Preferences gPrefs;
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
int32_t        currentSteps     = 0;
int32_t        targetSteps      = 0;
uint8_t        currentDirState  = STEP_DIR_UP;
bool           steppingUp       = true;
bool           wasMoving        = false;
StepPulsePhase pulsePhase       = PULSE_IDLE;
uint32_t       pulseTimestampUs = 0;

// Session & mode state
CyclingMode    activeMode       = MODE_MANUAL;
ControlState   ctrlState        = CTRL_STOPPED;
bool           sessionActive    = false;
float          targetTorqueNm   = 15.0f;
float          targetPowerW     = 60.0f;
float          targetIsokRpm    = 45.0f;

// Encoder & observer state
int16_t        prevPcntRaw      = 0;
int64_t        totalEncCounts   = 0;
uint32_t       glitchRejectCount = 0;
float          omegaSignedRaw   = 0.0f;
float          omegaSignedFilt  = 0.0f;
float          omegaMag         = 0.0f;
float          flywheelRpm      = 0.0f;
float          pedalRpm         = 0.0f;
float          completedPedalRevs = 0.0f;

float          zObs             = 0.0f;
float          tauEddyFly       = 0.0f;
float          tauPedalEst      = 0.0f;
float          forcePedalEst    = 0.0f;
float          powerUserW       = 0.0f;
float          totalWorkJoules  = 0.0f;

// Closed-loop control state
float          coastTimerSec    = 0.0f;
float          rampElapsedSec   = 0.0f;
float          rampStartAlpha   = 0.0f;
float          alphaTargetDeg   = 0.0f;
float          alphaCommandDeg  = 0.0f;
float          isokIntegral     = 0.0f;

// Scheduling & Serial CLI state
uint32_t       lastControlUs    = 0;
uint32_t       lastLogMs        = 0;
uint32_t       lastPlotMs       = 0;
bool           logEnabled       = false;
bool           plotEnabled      = false;
String         serialWordBuf;
uint32_t       lastSerialByteMs = 0;

// ============================================================================
// 4. GENERAL HELPERS
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

void printStatus() {
  int8_t dirSign = (omegaSignedFilt > 0.2f) ? 1 : ((omegaSignedFilt < -0.2f) ? -1 : 0);
  Serial.printf(
    "[STATUS] mode=%s state=%s | ",
    modeName(activeMode),
    stateName(ctrlState)
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
    " | step=%ld/%ld (%.2f deg, cmd=%.2f deg) | pedal=%.1f RPM (fly=%.1f RPM, dir=%+d) | "
    "tau=%.1f Nm (F=%.1f N) P=%.1f W W=%.0f J revs=%.2f | dob=%s lpf=%.2f glitches=%lu\n",
    (long)currentSteps,
    (long)targetSteps,
    stepsToAngleDeg(currentSteps),
    alphaTargetDeg,
    pedalRpm,
    flywheelRpm,
    (int)dirSign,
    tauPedalEst,
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
  Serial.println("\n--- DOULA Cycling Modes Serial Commands (115200 baud) ---");
  Serial.println("  manual | m0           : Switch to Manual step/angle mode");
  Serial.println("  torque [Nm] | m1 [Nm] : Constant Torque mode (default/range: 2..85 Nm) & start");
  Serial.println("  power [W]   | m2 [W]  : Constant Power mode (default/range: 15..250 W) & start");
  Serial.println("  isok [RPM]  | m3 [RPM]: Isokinetic Cadence mode (default/range: 15..100 RPM) & start");
  Serial.println("  start | s             : Start active mode (22.5 deg Manual; 5.0 deg standby Closed-Loop)");
  Serial.println("  end | e | stop        : End session & return stepper smoothly to 0.00 deg");
  Serial.println("  + / -                 : Adjust setpoint (Manual +-1 step, Torque +-2 Nm, Power +-10 W, Isok +-5 RPM)");
  Serial.println("  angle <deg>           : Switch to Manual & move directly to <deg> (0..42)");
  Serial.println("  home | 0              : Move stepper to 0.00 deg");
  Serial.println("  zero                  : Zero current physical stepper position & clear encoder totals");
  Serial.println("  status | ?            : Print full status summary");
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
  Serial.println("---------------------------------------------------------\n");
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

  // 4. Manual or stopped mode leaves direct stepper control untouched
  if (!sessionActive || activeMode == MODE_MANUAL) {
    if (!sessionActive) {
      ctrlState       = CTRL_STOPPED;
      alphaTargetDeg  = MIN_ANGLE_DEG;
      alphaCommandDeg = stepsToAngleDeg(targetSteps);
    } else {
      ctrlState       = CTRL_ACTIVE_CONTROL;
      alphaTargetDeg  = stepsToAngleDeg(targetSteps);
      alphaCommandDeg = alphaTargetDeg;
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
        ctrlState     = CTRL_COAST_UNLOAD;
        isokIntegral  = 0.0f;
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
    // Bypass deadband when parking/unloading to standby or 0 deg
    targetSteps = desiredSteps;
  } else if (abs(desiredSteps - targetSteps) >= CLOSED_LOOP_STEP_DEADBAND ||
             desiredSteps == MIN_STEPS || desiredSteps == MAX_STEPS) {
    targetSteps = desiredSteps;
  }
}

// ============================================================================
// 7. SESSION & MODE MANAGEMENT HELPERS
// ============================================================================
void startSessionForCurrentMode() {
  sessionActive   = true;
  coastTimerSec   = 0.0f;
  rampElapsedSec  = 0.0f;
  isokIntegral    = 0.0f;
  totalWorkJoules = 0.0f;

  if (activeMode == MODE_MANUAL) {
    ctrlState       = CTRL_ACTIVE_CONTROL;
    targetSteps     = DEFAULT_START_STEPS;
    alphaTargetDeg  = stepsToAngleDeg(targetSteps);
    alphaCommandDeg = alphaTargetDeg;
    Serial.printf("[SESSION] START MANUAL -> moving to %.2f deg (%ld steps)\n",
                  alphaTargetDeg, (long)targetSteps);
  } else {
    ctrlState       = CTRL_IDLE_READY;
    targetSteps     = STANDBY_STEPS;
    alphaTargetDeg  = STANDBY_ANGLE_DEG;
    alphaCommandDeg = stepsToAngleDeg(currentSteps);
    Serial.printf("[SESSION] START %s -> parking at %.1f deg standby, pedal to engage\n",
                  modeName(activeMode), STANDBY_ANGLE_DEG);
  }
}

void endSession() {
  sessionActive   = false;
  ctrlState       = CTRL_STOPPED;
  targetSteps     = MIN_STEPS;
  alphaTargetDeg  = MIN_ANGLE_DEG;
  alphaCommandDeg = stepsToAngleDeg(currentSteps);
  isokIntegral    = 0.0f;
  coastTimerSec   = 0.0f;
  Serial.printf("[SESSION] END -> returning to 0.00 deg (step 0) from %.2f deg (%ld steps)\n",
                stepsToAngleDeg(currentSteps), (long)currentSteps);
}

void adjustActiveTarget(int sign) {
  if (activeMode == MODE_MANUAL) {
    targetSteps     = clampSteps(targetSteps + sign);
    alphaTargetDeg  = stepsToAngleDeg(targetSteps);
    alphaCommandDeg = alphaTargetDeg;
    Serial.printf("[Serial] %s1 step -> target=%ld steps (%.2f deg) | current=%ld steps (%.2f deg)\n",
                  (sign > 0) ? "+" : "-",
                  (long)targetSteps, alphaTargetDeg,
                  (long)currentSteps, stepsToAngleDeg(currentSteps));
  } else if (activeMode == MODE_CONST_TORQUE) {
    targetTorqueNm = clampf(targetTorqueNm + sign * STEP_TORQUE_NM, MIN_TORQUE_NM, MAX_TORQUE_NM);
    Serial.printf("[TORQUE] Target resistance = %.1f Nm\n", targetTorqueNm);
  } else if (activeMode == MODE_CONST_POWER) {
    targetPowerW = clampf(targetPowerW + sign * STEP_POWER_W, MIN_POWER_W, MAX_POWER_W);
    Serial.printf("[POWER] Target power = %.0f W\n", targetPowerW);
  } else if (activeMode == MODE_ISOKINETIC) {
    targetIsokRpm = clampf(targetIsokRpm + sign * STEP_ISOK_RPM, MIN_ISOK_RPM, MAX_ISOK_RPM);
    Serial.printf("[ISOK] Target cadence = %.0f RPM\n", targetIsokRpm);
  }
}

// ============================================================================
// 8. SERIAL MONITOR COMMAND PARSER
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
    Serial.printf("[ENC] pcnt_raw=%d total_counts=%lld polarity=%+d | fly=%.1f RPM pedal=%.1f RPM | revs=%.2f | glitches=%lu\n",
                  (int)raw, (long long)totalEncCounts, (int)cfg.encPolarity,
                  flywheelRpm, pedalRpm, completedPedalRevs, (unsigned long)glitchRejectCount);
  } else if (lower == "manual" || lower == "m0") {
    activeMode = MODE_MANUAL;
    Serial.println("[MODE] Switched to MANUAL");
    printStatus();
  } else if (lower == "torque" || lower == "m1" || lower.startsWith("torque ") || lower.startsWith("m1 ")) {
    activeMode = MODE_CONST_TORQUE;
    float val = targetTorqueNm;
    const char* pfx = lower.startsWith("torque") ? "torque" : "m1";
    if (parseNextFloat(line, pfx, val)) {
      targetTorqueNm = clampf(val, MIN_TORQUE_NM, MAX_TORQUE_NM);
    }
    startSessionForCurrentMode();
  } else if (lower == "power" || lower == "m2" || lower.startsWith("power ") || lower.startsWith("m2 ")) {
    activeMode = MODE_CONST_POWER;
    float val = targetPowerW;
    const char* pfx = lower.startsWith("power") ? "power" : "m2";
    if (parseNextFloat(line, pfx, val)) {
      targetPowerW = clampf(val, MIN_POWER_W, MAX_POWER_W);
    }
    startSessionForCurrentMode();
  } else if (lower == "isok" || lower == "m3" || lower.startsWith("isok ") || lower.startsWith("m3 ")) {
    activeMode = MODE_ISOKINETIC;
    float val = targetIsokRpm;
    const char* pfx = lower.startsWith("isok") ? "isok" : "m3";
    if (parseNextFloat(line, pfx, val)) {
      targetIsokRpm = clampf(val, MIN_ISOK_RPM, MAX_ISOK_RPM);
    }
    startSessionForCurrentMode();
  } else if (lower == "s" || lower == "start") {
    startSessionForCurrentMode();
  } else if (lower == "e" || lower == "end" || lower == "stop") {
    endSession();
  } else if (lower == "0" || lower == "h" || lower == "home") {
    activeMode      = MODE_MANUAL;
    sessionActive   = false;
    ctrlState       = CTRL_STOPPED;
    targetSteps     = MIN_STEPS;
    alphaTargetDeg  = MIN_ANGLE_DEG;
    alphaCommandDeg = stepsToAngleDeg(currentSteps);
    Serial.printf("[Serial] Moving to 0.00 deg (step 0) from %.2f deg\n", stepsToAngleDeg(currentSteps));
  } else if (lower == "zero") {
    currentSteps     = 0;
    targetSteps      = 0;
    alphaTargetDeg   = 0.0f;
    alphaCommandDeg  = 0.0f;
    pulsePhase       = PULSE_IDLE;
    digitalWrite(STEP_PIN, LOW);
    pcnt_counter_pause(ENC_PCNT_UNIT);
    pcnt_counter_clear(ENC_PCNT_UNIT);
    pcnt_counter_resume(ENC_PCNT_UNIT);
    prevPcntRaw       = 0;
    totalEncCounts    = 0;
    completedPedalRevs = 0.0f;
    totalWorkJoules   = 0.0f;
    Serial.println("[Serial] Stepper zeroed at 0.00 deg & encoder counters reset");
  } else if (lower.startsWith("angle ")) {
    float deg = 0.0f;
    if (parseNextFloat(line, "angle", deg)) {
      activeMode      = MODE_MANUAL;
      sessionActive   = true;
      ctrlState       = CTRL_ACTIVE_CONTROL;
      targetSteps     = clampSteps(lroundf(deg / DEG_PER_STEP));
      alphaTargetDeg  = stepsToAngleDeg(targetSteps);
      alphaCommandDeg = alphaTargetDeg;
      Serial.printf("[MANUAL] Target set to %.2f deg (%ld steps)\n", alphaTargetDeg, (long)targetSteps);
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
      adjustActiveTarget(1);
    } else if (ch == '-' && serialWordBuf.length() == 0) {
      // Only treat standalone '-' as step/setpoint decrement (preserve negative numbers after command names)
      adjustActiveTarget(-1);
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
// 9. NON-BLOCKING STEPPER PULSE ENGINE
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
      }
    }
    return;
  }

  // PULSE_IDLE: emit next step if target differs
  if (currentSteps == targetSteps) {
    wasMoving = false;
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
// 10. STREAM LOGGING & ARDUINO ENTRY POINTS
// ============================================================================
void serviceTelemetryStreams() {
  uint32_t nowMs = millis();

  if (logEnabled && (nowMs - lastLogMs >= LOG_PERIOD_MS)) {
    lastLogMs = nowMs;
    printStatus();
  }

  if (plotEnabled && (nowMs - lastPlotMs >= PLOT_PERIOD_MS)) {
    lastPlotMs = nowMs;
    Serial.printf("rpm:%.2f,target_alpha:%.2f,alpha:%.2f,torque:%.2f,power:%.2f\n",
                  pedalRpm, alphaTargetDeg, stepsToAngleDeg(currentSteps), tauPedalEst, powerUserW);
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

  lastControlUs = micros();
  lastLogMs     = millis();
  lastPlotMs    = millis();

  Serial.println("\n=== DOULA Cycling Multi-Mode Controller (Serial Monitor Edition) ===");
  Serial.printf("Stepper Pins : DIR=GPIO%d, PUL=GPIO%d | %.2f deg/step (0..%.1f deg = %ld..%ld steps)\n",
                DIR_PIN, STEP_PIN, DEG_PER_STEP, MAX_ANGLE_DEG, (long)MIN_STEPS, (long)MAX_STEPS);
  Serial.printf("Encoder Pins : ChA=GPIO%d, ChB=GPIO%d (PCNT 4x, %.0f CPR, glitch=%.1f us)\n",
                PIN_ENC2_A, PIN_ENC2_B, ENC_CPR_4X, (float)PCNT_GLITCH_FILTER_APB / 80.0f);
  printHelp();
  printStatus();
}

void loop() {
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
