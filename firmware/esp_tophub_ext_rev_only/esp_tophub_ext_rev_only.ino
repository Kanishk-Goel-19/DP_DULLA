// ============================================================================
// PROJECT DOULA (VK26-IITI-HT-0240) — EXTENSION-ONLY / REV-ONLY TOP-HUB (U2)
// Target MCU : ESP32-S NodeMCU V1.1 (240 MHz Dual-Core, Arduino-ESP32 v2.x / v3.x)
// Hardware   : U201 (Rhino MD20Amp, DIR stuck in REV) -> M1 (MY1016) + GBX1 (1:52)
//              ** WIRING REQUIREMENT: Wire M+/M- so REV drives into FLEXION
//                 (+theta_m / +theta_k), opposing the patient's EXTENSION! **
//              U202 (30A MOSFET) -> R1||R2 (1.0 Ohm 200W Brake Chopper)
//              ENC1 (360 PPR NPN on Gearbox Output Shaft = 1,440 CPR on GPIO32/33)
//              U203 (Inline ACS712-30A on GPIO35, 33 mV/A)
//              VBUS_SENSE (GPIO34, mult 16.942)
//
// Supported  : Isotonic Extension, Isokinetic Extension, Isometric Extension,
//              ACL Shear Taper, ROM Min Extension Wall, Brake Chopper,
//              Spasm/Clonus/Pain-Yield Safety, Manual Homing, ESP-NOW 50 Hz.
// Unsupported: Passive CPM Lift, Bidirectional Setup Sweeps, Gravity-Holding
//              Mid-Air Lock (requires working DIR).
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <driver/pcnt.h>
#include <math.h>

// ============================================================================
// 0. EMBEDDED ESP-NOW PROTOCOL DEFINITIONS (Matches U1: ESP_MASTER)
// ============================================================================
#ifndef REHAB_ESPNOW_PROTOCOL_H
#define REHAB_ESPNOW_PROTOCOL_H

#define REHAB_WIFI_CHANNEL 1

enum NodeID : uint8_t {
    NODE_MASTER  = 0x01,
    NODE_TOPHUB  = 0x02,
    NODE_CYCLING = 0x03
};

enum PacketType : uint8_t {
    PKT_HEARTBEAT_CMD      = 0x10,
    PKT_TOPHUB_TELEMETRY   = 0x20,
    PKT_TOPHUB_REP_SUMMARY = 0x21,
    PKT_CYCLING_TELEMETRY  = 0x30
};

enum OperatingMode : uint8_t {
    OP_IDLE            = 0,
    OP_SETUP_FULL_25S  = 1,
    OP_SETUP_QUICK_6S  = 2,
    OP_CPM             = 3,
    OP_ISOTONIC        = 4,
    OP_ISOKINETIC      = 5,
    OP_ISOMETRIC       = 6,
    OP_CYCLE_TORQUE    = 7,
    OP_CYCLE_POWER     = 8,
    OP_DIRECT_PWM_TEST = 9,
    OP_AUTO_BENCH_TEST = 10
};

enum PathologyProfile : uint8_t {
    PATH_STANDARD      = 0,
    PATH_ACL_PROTECTED = 1,
    PATH_TKA           = 2,
    PATH_PFPS          = 3
};

enum HardStopMode : uint8_t {
    ESTOP_NORMAL            = 0,
    ESTOP_HARD_STOP_LOCKED  = 1,
    ESTOP_HARD_STOP_RELEASE = 2
};

#define STATE_FLAG_HOMED            (1 << 0)
#define STATE_FLAG_CALIBRATED       (1 << 1)
#define STATE_FLAG_CHOPPER_ACTIVE   (1 << 2)
#define STATE_FLAG_SPASM_STOP       (1 << 3)
#define STATE_FLAG_HARDSTOP_LOCKED  (1 << 4)

struct __attribute__((packed)) RehabCommandPacket_t {
    uint8_t  pkt_type;
    uint8_t  target_node;
    uint16_t seq_num;
    uint8_t  op_mode;
    uint8_t  pathology_profile;
    uint8_t  estop_active;
    float    d_offset_m;
    float    L_leg_m;
    float    rom_min_deg;
    float    rom_max_deg;
    float    target_vel_deg_s;
    float    target_torque_ext;
    float    target_torque_flex;
    uint16_t crc16;
};

struct __attribute__((packed)) TopHubTelemetryPacket_t {
    uint8_t  pkt_type;
    uint8_t  node_id;
    uint16_t seq_num;
    uint8_t  state_flags;
    uint16_t current_rep;
    float    knee_angle_deg;
    float    knee_vel_deg_s;
    float    knee_torque_nm;
    float    gearbox_torque_nm;
    float    pad_force_newtons;
    float    motor_current_amps;
    float    target_curve_nm;
    float    vbus_volts;
    float    chopper_duty_pct;
    uint16_t crc16;
};

struct __attribute__((packed)) TopHubRepSummaryPacket_t {
    uint8_t  pkt_type;
    uint16_t rep_index;
    float    achieved_rom_min;
    float    achieved_rom_max;
    float    peak_torque_ext_nm;
    float    angle_at_peak_ext;
    float    peak_torque_flex_nm;
    float    angle_at_peak_flex;
    float    rep_work_joules;
    float    vel_cov_pct;
    uint16_t crc16;
};

static inline uint16_t computeCRC16(const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}
#endif // REHAB_ESPNOW_PROTOCOL_H

// ============================================================================
// 1. HARDWARE PIN DEFINITIONS & OPTIONAL SLP FREEWHEEL TRANSISTOR
// ============================================================================
static const int PIN_MD_PWM       = 25; // U201 Rhino PWM (R15=220k pull-down)
static const int PIN_MD_DIR       = 26; // U201 Rhino DIR (kept LOW for REV)
static const int PIN_CHOP_PWM     = 27; // U202 30A Brake Chopper MOSFET (5 kHz)
static const int PIN_ENC1_A       = 32; // ENC1 Out A (R3=80 Ohm + INPUT_PULLUP)
static const int PIN_ENC1_B       = 33; // ENC1 Out B (R5=80 Ohm + INPUT_PULLUP)
static const int PIN_VBUS_SENSE   = 34; // ADC1_CH6: R11(73.33k) / R12(4.6k)
static const int PIN_ACS712_SENSE = 35; // ADC1_CH7: R13(22k) / R14(22k)
static const int PIN_LOADCELL_ADC = 36; // Optional Expansion (VP)

// OPTIONAL FREEWHEEL RETURN VIA SLP:
// Because PWM=0% on Rhino MD20Amp engages Mode 2 low-side shorting (braking drag
// during flexion return), pulling SLP LOW tri-states the H-bridge to coast.
// WARNING: SLP idles at ~6.8V! NEVER wire an ESP32 GPIO directly to SLP!
// Only set USE_SLP_COAST_TRANSISTOR = true if PIN_MD_SLP_NPN drives the base/gate
// of an NPN/NMOS open-drain switch (HIGH = transistor ON = SLP pulled to GND).
static const bool USE_SLP_COAST_TRANSISTOR = false;
static const int  PIN_MD_SLP_NPN           = 23;

static const bool USE_OPTIONAL_LOADCELL    = false;
static const bool ENABLE_ESPNOW_WIFI       = true;  // Set false if bench-testing on weak USB without Master ESP32

// ============================================================================
// 2. TIMING, PWM & PHYSICAL CONSTANTS
// ============================================================================
static const uint32_t CONTROL_PERIOD_US  = 2000;  // 500 Hz (2.0 ms)
static const float    CONTROL_DT_SEC     = 0.002f;
static const float    CONTROL_FREQ_HZ    = 500.0f;

static uint32_t       g_md_pwm_freq_hz   = 20000; // 20 kHz default (matches working circuit test; adjustable via 'freq <hz>')
static const uint32_t CHOP_PWM_FREQ_HZ   = 5000;
static const uint8_t  PWM_RES_BITS       = 10;
static const int32_t  PWM_MAX_DUTY       = 1023;
static const int      LEDC_CH_MOTOR      = 0;     // v2.x LEDC Timer 0
static const int      LEDC_CH_CHOPPER    = 2;     // v2.x LEDC Timer 1 (avoids overwriting Timer 0 motor freq)

static const float GEAR_RATIO            = 52.0f;
static const float ENC_CPR_MOTOR         = 1440.0f; // 360 PPR * 4x quadrature
static const float ENC_CPR_OUTPUT        = 1440.0f; // Mounted on Gearbox Output Shaft (0.25 deg/count)
static const float DEG_PER_COUNT         = 360.0f / ENC_CPR_OUTPUT;
static const int   VEL_WINDOW_TICKS      = 20;      // 40 ms sliding window at 500 Hz for smooth 0.25 deg/cnt velocity
static const float RAD_PER_DEG           = 0.0174532925f;
static const float DEG_PER_RAD           = 57.2957795f;

static const float VBUS_DIV_MULT         = 16.942f;
static const float ACS712_MV_PER_AMP     = 33.0f;

static const float MOTOR_MAX_CURRENT_A   = 20.0f;
static const float GBX_MAX_TORQUE_NM     = 100.0f;
static const float VBUS_OVERVOLT_FAULT_V = 28.5f;

// ============================================================================
// 3. CALIBRATION & NVS STRUCTURE
// ============================================================================
struct CalibrationData_t {
    bool    calibrated;
    int8_t  enc_polarity;       // +1 means counts increase toward Flexion (+theta_m)
    int8_t  acs_polarity;       // +1 means positive current when REV pushes Flexion
    float   v_adc_zero_mv;
    float   v_bus_idle_v;
    float   r_loop_ohm;
    float   v_brush_v;
    float   k_t_nm_per_a;
    float   i_no_load_a;
    float   tau_fric_gbx_nm;
    float   eta_fwd;
    float   eta_back;
};

static CalibrationData_t g_cal = {
    false,      // calibrated
    +1,         // enc_polarity
    +1,         // acs_polarity
    1250.0f,    // v_adc_zero_mv
    24.00f,     // v_bus_idle_v
    0.40f,      // r_loop_ohm
    0.35f,      // v_brush_v
    0.0694f,    // k_t_nm_per_a
    1.00f,      // i_no_load_a
    2.50f,      // tau_fric_gbx_nm
    0.85f,      // eta_fwd
    0.85f       // eta_back
};

static Preferences g_prefs;

// ============================================================================
// 4. REAL-TIME STATE & COMMAND BUFFERS
// ============================================================================
static portMUX_TYPE g_rx_mux = portMUX_INITIALIZER_UNLOCKED;

struct ActiveCommand_t {
    uint8_t  op_mode;
    uint8_t  pathology_profile;
    uint8_t  estop_mode;
    float    d_offset_m;
    float    L_leg_m;
    float    rom_min_deg;
    float    rom_max_deg;
    float    target_vel_deg_s;
    float    target_torque_ext;
    float    target_torque_flex;
    uint32_t last_cmd_ms;
    bool     from_espnow;
};

static ActiveCommand_t g_cmd = {
    OP_IDLE,
    PATH_STANDARD,
    ESTOP_NORMAL,
    0.090f,  // d_offset = 9.0 cm
    0.300f,  // L_leg = 30.0 cm
    0.0f,    // rom_min_deg (full extension)
    110.0f,  // rom_max_deg (flexion)
    60.0f,   // target_vel_deg_s (isokinetic ceiling)
    35.0f,   // target_torque_ext [N*m]
    0.0f,    // target_torque_flex [N*m] (passive return in REV-only mode)
    0,
    false
};

static RehabCommandPacket_t g_rx_staging_pkt;
static volatile bool        g_rx_pkt_pending = false;

static bool     g_vbus_valid         = false;
static bool     g_acs_valid          = false;
static bool     g_enc_valid          = false;

static uint16_t g_vbus_raw_adc       = 0;
static float    g_vbus_raw_mv        = 0.0f;
static uint16_t g_acs_raw_adc        = 0;
static float    g_acs_raw_mv         = 0.0f;

static bool     g_is_homed           = false;
static bool     g_spasm_latched      = false;
static bool     g_hardstop_locked    = false;
static bool     g_is_coasting        = true;
static float    g_freeze_theta_m_deg = 60.0f;

static int64_t  g_enc_counts         = 0;
static int64_t  g_enc_boot_counts    = 0;
static int16_t  g_prev_pcnt_raw      = 0;

static float    g_theta_m_deg        = 90.0f;
static float    g_omega_m_deg_s      = 0.0f;
static float    g_theta_k_deg        = 90.0f;
static float    g_omega_k_deg_s      = 0.0f;
static float    g_gamma_factor       = 1.30f;

static float    g_vbus_volts         = 24.0f;
static float    g_chopper_duty_pct   = 0.0f;
static float    g_motor_current_a    = 0.0f;
static float    g_i_ref_a            = 0.0f;
static float    g_active_pwm_pct     = 0.0f;
static float    g_gbx_torque_nm      = 0.0f;
static float    g_knee_torque_nm     = 0.0f;
static float    g_pad_force_n        = 0.0f;
static float    g_target_curve_nm    = 0.0f;

static uint32_t g_bench_test_start_ms = 0;

static uint16_t g_rep_count          = 0;
static bool     g_rep_summary_ready  = false;
static TopHubRepSummaryPacket_t g_latest_rep_summary;

static uint8_t g_master_mac[6]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool    g_master_mac_known  = false;

// ============================================================================
// 5. REV-ONLY HARDWARE PWM, OPTIONAL SLP COAST & BRAKE CHOPPER DRIVER
// ============================================================================
static int32_t s_last_motor_duty   = -1;
static int32_t s_last_chopper_duty = -1;
static int8_t  s_last_slp_coast    = -1;
static bool    s_pwm_attached      = false;

static inline void initPWMHardware() {
    pinMode(PIN_MD_DIR, OUTPUT);
    digitalWrite(PIN_MD_DIR, LOW); // Hardware is fixed in REV

    if (USE_SLP_COAST_TRANSISTOR) {
        pinMode(PIN_MD_SLP_NPN, OUTPUT);
        digitalWrite(PIN_MD_SLP_NPN, HIGH); // Start coasting/sleeping safely
        s_last_slp_coast = 1;
    }

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    if (s_pwm_attached) {
        ledcDetach(PIN_MD_PWM);
        ledcDetach(PIN_CHOP_PWM);
    }
    ledcAttach(PIN_MD_PWM, g_md_pwm_freq_hz, PWM_RES_BITS);
    ledcWrite(PIN_MD_PWM, 0);
    ledcAttach(PIN_CHOP_PWM, CHOP_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcWrite(PIN_CHOP_PWM, 0);
#else
    ledcSetup(LEDC_CH_MOTOR, g_md_pwm_freq_hz, PWM_RES_BITS);
    ledcAttachPin(PIN_MD_PWM, LEDC_CH_MOTOR);
    ledcWrite(LEDC_CH_MOTOR, 0);

    ledcSetup(LEDC_CH_CHOPPER, CHOP_PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(PIN_CHOP_PWM, LEDC_CH_CHOPPER);
    ledcWrite(LEDC_CH_CHOPPER, 0);
#endif
    s_pwm_attached      = true;
    s_last_motor_duty   = 0;
    s_last_chopper_duty = 0;
}

static inline void writeMotorPWMRaw(uint32_t duty) {
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(PIN_MD_PWM, duty);
#else
    ledcWrite(LEDC_CH_MOTOR, duty);
#endif
}

static inline void setSleepCoast(bool coast) {
    g_is_coasting = coast;
    if (!USE_SLP_COAST_TRANSISTOR) return;
    int8_t target = coast ? 1 : 0;
    if (target != s_last_slp_coast) {
        digitalWrite(PIN_MD_SLP_NPN, coast ? HIGH : LOW);
        s_last_slp_coast = target;
    }
}

// Drives REV PWM only (duty > 0 wakes driver; duty == 0 applies Mode 2 shorting or SLP coast)
static inline void setMotorRevOutput(uint32_t duty, bool allow_coast) {
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    digitalWrite(PIN_MD_DIR, LOW);

    if (duty > 0) {
        setSleepCoast(false);
    } else {
        setSleepCoast(allow_coast);
    }

    if ((int32_t)duty != s_last_motor_duty) {
        writeMotorPWMRaw(duty);
        s_last_motor_duty = (int32_t)duty;
    }
    g_active_pwm_pct = ((float)duty / (float)PWM_MAX_DUTY) * 100.0f;
}

static inline void stopMotorDynamicBrake() {
    g_i_ref_a = 0.0f;
    setMotorRevOutput(0, false); // PWM=0%, SLP awake -> Mode 2 low-side shorting
}

static inline void stopMotorCoastRelease() {
    g_i_ref_a = 0.0f;
    setMotorRevOutput(0, true);  // PWM=0%, SLP asleep (if transistor fitted)
}

static inline void writeChopperPWM(uint32_t duty) {
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    if ((int32_t)duty != s_last_chopper_duty) {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcWrite(PIN_CHOP_PWM, duty);
#else
        ledcWrite(LEDC_CH_CHOPPER, duty);
#endif
        s_last_chopper_duty = (int32_t)duty;
    }
}

// ============================================================================
// 6. HARDWARE PCNT 4X QUADRATURE ENCODER INITIALIZATION (GPIO32 / GPIO33)
// ============================================================================
static void initEncoderPCNT() {
    pinMode(PIN_ENC1_A, INPUT_PULLUP);
    pinMode(PIN_ENC1_B, INPUT_PULLUP);

    pcnt_config_t ch0 = {};
    ch0.pulse_gpio_num = PIN_ENC1_A;
    ch0.ctrl_gpio_num  = PIN_ENC1_B;
    ch0.unit           = PCNT_UNIT_0;
    ch0.channel        = PCNT_CHANNEL_0;
    ch0.pos_mode       = PCNT_COUNT_INC;
    ch0.neg_mode       = PCNT_COUNT_DEC;
    ch0.lctrl_mode     = PCNT_MODE_REVERSE;
    ch0.hctrl_mode     = PCNT_MODE_KEEP;
    ch0.counter_h_lim  = 30000;
    ch0.counter_l_lim  = -30000;
    pcnt_unit_config(&ch0);

    pcnt_config_t ch1 = {};
    ch1.pulse_gpio_num = PIN_ENC1_B;
    ch1.ctrl_gpio_num  = PIN_ENC1_A;
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
    g_prev_pcnt_raw = 0;
}

static float g_vel_hist_tm[VEL_WINDOW_TICKS];
static float g_vel_hist_tk[VEL_WINDOW_TICKS];
static int   g_vel_hist_idx = 0;

static inline void resetVelocityWindow(float theta_m_deg, float theta_k_deg) {
    for (int i = 0; i < VEL_WINDOW_TICKS; ++i) {
        g_vel_hist_tm[i] = theta_m_deg;
        g_vel_hist_tk[i] = theta_k_deg;
    }
    g_vel_hist_idx  = 0;
    g_omega_m_deg_s = 0.0f;
    g_omega_k_deg_s = 0.0f;
}

static inline float mapArmToKneeAngleDeg(float theta_m_deg, float d_offset_m, float L_leg_m);

static inline void latchEncoderAngleDeg(float known_theta_m_deg) {
    pcnt_counter_clear(PCNT_UNIT_0);
    g_prev_pcnt_raw   = 0;
    g_enc_counts      = (int64_t)llroundf(known_theta_m_deg / DEG_PER_COUNT);
    g_enc_boot_counts = g_enc_counts;
    g_theta_m_deg     = known_theta_m_deg;
    float tk          = mapArmToKneeAngleDeg(known_theta_m_deg, g_cmd.d_offset_m, fmaxf(0.20f, g_cmd.L_leg_m));
    g_theta_k_deg     = tk;
    resetVelocityWindow(known_theta_m_deg, tk);
}

// ============================================================================
// 7. NVS FLASH STORAGE
// ============================================================================
static void loadCalibrationFromNVS() {
    g_prefs.begin("tophub_ext", true);
    if (g_prefs.getBool("valid", false)) {
        g_cal.calibrated      = true;
        g_cal.enc_polarity    = g_prefs.getChar("enc_pol", +1);
        g_cal.acs_polarity    = g_prefs.getChar("acs_pol", +1);
        g_cal.v_adc_zero_mv   = g_prefs.getFloat("v_zero", 1250.0f);
        g_cal.v_bus_idle_v    = g_prefs.getFloat("v_bus", 24.00f);
        g_cal.r_loop_ohm      = g_prefs.getFloat("r_loop", 0.40f);
        g_cal.v_brush_v       = g_prefs.getFloat("v_brush", 0.35f);
        g_cal.k_t_nm_per_a    = g_prefs.getFloat("k_t", 0.0694f);
        g_cal.i_no_load_a     = g_prefs.getFloat("i_0", 1.00f);
        g_cal.tau_fric_gbx_nm = g_prefs.getFloat("t_fric", 2.50f);
        g_cal.eta_fwd         = g_prefs.getFloat("eta_f", 0.85f);
        g_cal.eta_back        = g_prefs.getFloat("eta_b", 0.85f);

        if (g_cal.v_adc_zero_mv < 400.0f || g_cal.v_adc_zero_mv > 2750.0f) {
            g_cal.v_adc_zero_mv = 1250.0f;
        }
        if (g_cal.v_bus_idle_v < 18.0f || g_cal.v_bus_idle_v > 27.5f) {
            g_cal.v_bus_idle_v = 24.00f;
        }
    }
    g_prefs.end();
}

static void saveCalibrationToNVS() {
    g_prefs.begin("tophub_ext", false);
    g_prefs.putBool("valid", g_cal.calibrated);
    g_prefs.putChar("enc_pol", g_cal.enc_polarity);
    g_prefs.putChar("acs_pol", g_cal.acs_polarity);
    g_prefs.putFloat("v_zero", g_cal.v_adc_zero_mv);
    g_prefs.putFloat("v_bus", g_cal.v_bus_idle_v);
    g_prefs.putFloat("r_loop", g_cal.r_loop_ohm);
    g_prefs.putFloat("v_brush", g_cal.v_brush_v);
    g_prefs.putFloat("k_t", g_cal.k_t_nm_per_a);
    g_prefs.putFloat("i_0", g_cal.i_no_load_a);
    g_prefs.putFloat("t_fric", g_cal.tau_fric_gbx_nm);
    g_prefs.putFloat("eta_f", g_cal.eta_fwd);
    g_prefs.putFloat("eta_b", g_cal.eta_back);
    g_prefs.end();
}

// ============================================================================
// 8. NON-COLLINEAR OFFSET-PIVOT KINEMATICS & PHYSIOLOGICAL EXTENSION CURVE
// ============================================================================
static inline float mapKneeToArmAngleDeg(float theta_k_deg, float d_offset_m, float L_leg_m) {
    float tk_rad = theta_k_deg * RAD_PER_DEG;
    float y = L_leg_m * sinf(tk_rad) + d_offset_m;
    float x = L_leg_m * cosf(tk_rad);
    return atan2f(y, x) * DEG_PER_RAD;
}

static inline float mapArmToKneeAngleDeg(float theta_m_deg, float d_offset_m, float L_leg_m) {
    float tm_rad = theta_m_deg * RAD_PER_DEG;
    float ratio  = (L_leg_m > 0.05f) ? (d_offset_m / L_leg_m) : 0.30f;
    float arg    = constrain(ratio * cosf(tm_rad), -0.99f, 0.99f);
    return theta_m_deg - (asinf(arg) * DEG_PER_RAD);
}

static inline float computeGammaFactor(float theta_k_deg, float d_offset_m, float L_leg_m) {
    float ratio = (L_leg_m > 0.05f) ? (d_offset_m / L_leg_m) : 0.30f;
    float s     = sinf(constrain(theta_k_deg, 0.0f, 130.0f) * RAD_PER_DEG);
    return constrain(1.0f + ratio * s, 1.0f, 1.45f);
}

static float computeExtensionTorqueTarget(
    float theta_k_deg,
    float peak_ext_nm,
    uint8_t pathology_profile
) {
    float tk   = constrain(theta_k_deg, 0.0f, 125.0f);
    float mult = 1.0f;
    if (tk <= 60.0f) {
        float u = tk / 60.0f;
        mult = 0.25f + 0.75f * sinf(u * (PI * 0.5f));
        if (pathology_profile == PATH_ACL_PROTECTED && tk < 40.0f) {
            mult *= (0.35f + 0.65f * (tk / 40.0f));
        }
    } else {
        float u = (tk - 60.0f) / 60.0f;
        mult = 1.0f - 0.65f * (u * u);
    }
    return constrain(peak_ext_nm * constrain(mult, 0.20f, 1.0f), 0.0f, 68.0f);
}

// ============================================================================
// 9. ANALOG SENSING, BRAKE CHOPPER & REV-ONLY CURRENT REGULATOR
// ============================================================================
static void sampleAnalogSensors500Hz() {
    g_vbus_raw_adc = analogRead(PIN_VBUS_SENSE);
    g_vbus_raw_mv  = (float)analogReadMilliVolts(PIN_VBUS_SENSE);

    if (g_vbus_raw_adc >= 100 && g_vbus_raw_adc <= 3500 &&
        g_vbus_raw_mv >= 600.0f && g_vbus_raw_mv <= 2300.0f) {
        g_vbus_valid = true;
        float vbus_meas = (g_vbus_raw_mv * 0.001f) * VBUS_DIV_MULT;
        g_vbus_volts = 0.85f * g_vbus_volts + 0.15f * vbus_meas;
    } else {
        g_vbus_valid = false;
        g_vbus_volts = 24.0f;
    }

    g_acs_raw_adc = analogRead(PIN_ACS712_SENSE);
    g_acs_raw_mv  = (float)analogReadMilliVolts(PIN_ACS712_SENSE);

    if (g_acs_raw_adc >= 250 && g_acs_raw_adc <= 3900 &&
        g_acs_raw_mv >= 300.0f && g_acs_raw_mv <= 2850.0f) {
        g_acs_valid = true;
        float i_raw = ((g_acs_raw_mv - g_cal.v_adc_zero_mv) / ACS712_MV_PER_AMP) * (float)g_cal.acs_polarity;
        g_motor_current_a = 0.70f * g_motor_current_a + 0.30f * i_raw;
    } else {
        g_acs_valid = false;
        g_motor_current_a = g_i_ref_a;
    }
}

static void updateBrakeChopper(float vbus_v) {
    if (!g_vbus_valid ||
        g_cmd.op_mode == OP_DIRECT_PWM_TEST ||
        g_cmd.op_mode == OP_AUTO_BENCH_TEST) {
        g_chopper_duty_pct = 0.0f;
        writeChopperPWM(0);
        return;
    }

    float v_on  = g_cal.calibrated ? (g_cal.v_bus_idle_v + 0.50f) : 25.50f;
    float v_max = v_on + 2.00f;

    if (vbus_v <= v_on) {
        g_chopper_duty_pct = 0.0f;
        writeChopperPWM(0);
    } else if (vbus_v >= v_max) {
        g_chopper_duty_pct = 100.0f;
        writeChopperPWM(PWM_MAX_DUTY);
    } else {
        float frac = (vbus_v - v_on) / (v_max - v_on);
        g_chopper_duty_pct = frac * 100.0f;
        writeChopperPWM((uint32_t)(frac * PWM_MAX_DUTY));
    }
}

// Closed-loop REV-only current controller (i_resist_mag_a >= 0 pushes Flexion).
// CRITICAL: If v_cmd <= 0 (meaning passive Mode 2 shorting already meets or exceeds
// target current at high speed), we NEVER drive REV PWM > 0 (would amplify instead
// of buck!). We clamp safely to PWM = 0% (Mode 2 shorting, or coast if requested).
static float g_pi_integral_v = 0.0f;

static void executeRevOnlyCurrentControl(float i_resist_mag_a, float omega_m_deg_s, bool coast_release) {
    if (coast_release) {
        g_pi_integral_v = 0.0f;
        stopMotorCoastRelease();
        return;
    }

    i_resist_mag_a = constrain(i_resist_mag_a, 0.0f, MOTOR_MAX_CURRENT_A);
    g_i_ref_a      = i_resist_mag_a;

    if (fabsf(omega_m_deg_s) > 330.0f || i_resist_mag_a < 0.08f) {
        g_pi_integral_v = 0.0f;
        stopMotorDynamicBrake();
        return;
    }

    // Extending means omega_m < 0 -> e_bemf_v < 0 assists positive REV shorting current
    float omega_motor_rads = (omega_m_deg_s * RAD_PER_DEG) * GEAR_RATIO;
    float e_bemf_v         = g_cal.k_t_nm_per_a * omega_motor_rads;
    float v_ff             = (i_resist_mag_a * g_cal.r_loop_ohm) + e_bemf_v + g_cal.v_brush_v;

    float v_cmd = v_ff;
    if (g_acs_valid) {
        const float KP_I = 0.25f;
        const float KI_I = 8.0f;
        float i_err = i_resist_mag_a - g_motor_current_a;
        float next_int = constrain(g_pi_integral_v + (KI_I * CONTROL_DT_SEC * i_err), -2.5f, 6.0f);
        // Avoid deep negative windup while already clamped at PWM = 0% (Mode 2 shorting)
        if ((v_ff + KP_I * i_err + g_pi_integral_v) > 0.0f || i_err > 0.0f) {
            g_pi_integral_v = next_int;
        }
        v_cmd += (KP_I * i_err) + g_pi_integral_v;
    } else {
        g_pi_integral_v = 0.0f;
    }

    // Safety clamp: NEVER drive REV PWM > 0 if v_cmd <= 0 (Mode 2 shorting takes over)
    if (v_cmd <= 0.0f) {
        setMotorRevOutput(0, false);
        return;
    }

    float vbus      = (g_vbus_valid && g_vbus_volts >= 15.0f && g_vbus_volts <= 30.0f) ? g_vbus_volts : 24.0f;
    float duty_frac = constrain(v_cmd / vbus, 0.0f, 0.98f);
    setMotorRevOutput((uint32_t)(duty_frac * PWM_MAX_DUTY), false);
}

// ============================================================================
// 10. SAFE STATIC CALIBRATION & MANUAL HOMING / POLARITY VERIFICATION
// ============================================================================
enum SetupStage : uint8_t {
    SETUP_STAGE_IDLE = 0,
    SETUP_STAGE_1_STATIC_ZERO
};

static SetupStage g_setup_stage    = SETUP_STAGE_IDLE;
static uint32_t   g_setup_stage_ms = 0;
static double     g_cal_acc1       = 0.0;
static double     g_cal_acc2       = 0.0;
static uint32_t   g_cal_samples    = 0;

static bool     g_dirtest_active    = false;
static uint32_t g_dirtest_start_ms  = 0;
static int64_t  g_dirtest_start_cnt = 0;
static double   g_dirtest_acs_sum   = 0.0;
static uint32_t g_dirtest_samples   = 0;

static void startStaticCalibration() {
    g_setup_stage     = SETUP_STAGE_1_STATIC_ZERO;
    g_setup_stage_ms  = millis();
    g_cal_acc1        = 0.0;
    g_cal_acc2        = 0.0;
    g_cal_samples     = 0;
    g_spasm_latched   = false;
    g_hardstop_locked = false;
}

static void runStaticCalibrationStep() {
    uint32_t elapsed = millis() - g_setup_stage_ms;
    stopMotorCoastRelease();

    if (elapsed > 100 && elapsed <= 900) {
        g_cal_acc1 += (double)g_acs_raw_mv;
        g_cal_acc2 += (double)g_vbus_raw_mv * 0.001 * VBUS_DIV_MULT;
        g_cal_samples++;
    } else if (elapsed > 900) {
        if (g_cal_samples > 50) {
            float avg_acs_mv = (float)(g_cal_acc1 / g_cal_samples);
            float avg_vbus_v = (float)(g_cal_acc2 / g_cal_samples);

            if (avg_acs_mv >= 400.0f && avg_acs_mv <= 2750.0f) {
                g_cal.v_adc_zero_mv = avg_acs_mv;
            } else {
                g_cal.v_adc_zero_mv = 1250.0f;
            }
            if (avg_vbus_v >= 16.0f && avg_vbus_v <= 28.0f) {
                g_cal.v_bus_idle_v = avg_vbus_v;
            } else {
                g_cal.v_bus_idle_v = 24.00f;
            }
        }
        g_cal.calibrated = true;
        saveCalibrationToNVS();
        Serial.printf("[CAL] Static Zero Done: ACS_Zero=%.1f mV | Vbus_idle=%.2f V (Saved to NVS).\n",
                      g_cal.v_adc_zero_mv, g_cal.v_bus_idle_v);
        Serial.println("[CAL] Note: Bidirectional auto-sweeps skipped (REV-only). Place arm at 0 deg (or 90 deg) and send 'home 0' (or 'home 90').");
        g_setup_stage = SETUP_STAGE_IDLE;
        g_cmd.op_mode = OP_IDLE;
    }
}

static void runDirectionVerifyStep() {
    uint32_t elapsed = millis() - g_dirtest_start_ms;
    if (elapsed < 250) {
        setMotorRevOutput((uint32_t)(0.18f * PWM_MAX_DUTY), false);
        if (elapsed > 100 && g_acs_valid) {
            float i_raw = ((g_acs_raw_mv - g_cal.v_adc_zero_mv) / ACS712_MV_PER_AMP);
            g_dirtest_acs_sum += i_raw;
            g_dirtest_samples++;
        }
    } else {
        stopMotorCoastRelease();
        g_dirtest_active = false;

        int64_t d_cnt = g_enc_counts - g_dirtest_start_cnt;
        if (llabs(d_cnt) > 10) {
            if (d_cnt < 0) {
                g_cal.enc_polarity = -g_cal.enc_polarity;
                saveCalibrationToNVS();
                Serial.printf("[DIRTEST] Auto-flipped enc_polarity to %+d so REV(+Flexion) increases angle.\n",
                              g_cal.enc_polarity);
            } else {
                Serial.printf("[DIRTEST] ENC1 OK (+%lld counts toward Flexion on REV pulse).\n", (long long)d_cnt);
            }
        } else {
            Serial.println("[DIRTEST] Note: Small/no ENC1 delta during 18% REV pulse (check if resting against flexion stop or ENC1 unplugged).");
        }

        if (g_acs_valid && g_dirtest_samples > 10) {
            float avg_i = (float)(g_dirtest_acs_sum / g_dirtest_samples);
            if (avg_i < -0.15f) {
                g_cal.acs_polarity = -1;
                saveCalibrationToNVS();
                Serial.println("[DIRTEST] Auto-flipped acs_polarity to -1 so REV drive reads +Amps.");
            } else if (avg_i > 0.15f) {
                g_cal.acs_polarity = +1;
                saveCalibrationToNVS();
                Serial.printf("[DIRTEST] ACS712 OK (+%.2f A on REV pulse).\n", avg_i);
            }
        }
    }
}

// ============================================================================
// 11. EXTENSION-ONLY SPASM, CLONUS & HARD-STOP SAFETY SUPERVISOR (500 Hz)
// ============================================================================
static float    g_prev_pad_force_n     = 0.0f;
static uint32_t g_residual_spike_ticks = 0;
static int8_t   g_prev_vel_sign        = 0;
static uint32_t g_clonus_window_start  = 0;
static uint8_t  g_clonus_reversals     = 0;

static bool evaluateSafetySupervisor(uint8_t op_mode) {
    if (op_mode == OP_DIRECT_PWM_TEST || op_mode == OP_AUTO_BENCH_TEST ||
        op_mode == OP_SETUP_FULL_25S  || op_mode == OP_SETUP_QUICK_6S || g_dirtest_active) {
        g_prev_pad_force_n = g_pad_force_n;
        return false;
    }

    if (g_vbus_valid && g_vbus_volts > VBUS_OVERVOLT_FAULT_V) {
        writeChopperPWM(PWM_MAX_DUTY);
        stopMotorCoastRelease();
        return true;
    }

    if (g_cmd.estop_mode == ESTOP_HARD_STOP_RELEASE) {
        g_hardstop_locked = false;
        g_target_curve_nm = 0.0f;
        stopMotorCoastRelease();
        return true;
    }

    if (g_cmd.estop_mode == ESTOP_HARD_STOP_LOCKED && !g_hardstop_locked) {
        g_hardstop_locked    = true;
        g_freeze_theta_m_deg = g_theta_m_deg;
    }

    if (g_cmd.from_espnow && op_mode >= OP_CPM && (millis() - g_cmd.last_cmd_ms > 500)) {
        if (!g_hardstop_locked) {
            g_hardstop_locked    = true;
            g_freeze_theta_m_deg = g_theta_m_deg;
        }
    }

    if (op_mode >= OP_ISOTONIC && op_mode <= OP_ISOMETRIC && !g_hardstop_locked) {
        bool force_spike            = false;
        bool current_residual_fault = false;
        bool clonus_fault           = false;

        if (g_acs_valid) {
            float df_dt = (g_pad_force_n - g_prev_pad_force_n) * CONTROL_FREQ_HZ;
            g_prev_pad_force_n = g_pad_force_n;
            force_spike = (fabsf(df_dt) > 450.0f && fabsf(g_pad_force_n) > 35.0f);

            // Only evaluate residual current when actively controlling REV torque,
            // or compare against expected Mode 2 passive shorting current when PWM=0 & awake.
            float expected_i = g_i_ref_a;
            if (g_active_pwm_pct < 0.5f && !g_is_coasting) {
                float e_bemf = fabsf(g_cal.k_t_nm_per_a * (g_omega_m_deg_s * RAD_PER_DEG) * GEAR_RATIO);
                expected_i = e_bemf / fmaxf(0.25f, g_cal.r_loop_ohm);
            }
            if (fabsf(fabsf(g_motor_current_a) - expected_i) > 4.5f) {
                g_residual_spike_ticks++;
            } else if (g_residual_spike_ticks > 0) {
                g_residual_spike_ticks--;
            }
            current_residual_fault = (g_residual_spike_ticks > 20);
        }

        if (g_enc_valid) {
            int8_t vel_sign = (g_omega_k_deg_s > 4.0f) ? +1 : ((g_omega_k_deg_s < -4.0f) ? -1 : g_prev_vel_sign);
            if (vel_sign != 0 && vel_sign != g_prev_vel_sign) {
                uint32_t now = millis();
                if (now - g_clonus_window_start > 600) {
                    g_clonus_window_start = now;
                    g_clonus_reversals = 1;
                } else {
                    g_clonus_reversals++;
                }
                g_prev_vel_sign = vel_sign;
            }
            clonus_fault = (g_clonus_reversals >= 4);
        }

        if (force_spike || current_residual_fault || clonus_fault) {
            g_spasm_latched        = true;
            g_hardstop_locked      = true;
            g_freeze_theta_m_deg   = g_theta_m_deg;
            g_residual_spike_ticks = 0;
            g_clonus_reversals     = 0;
        }
    }

    // Hard-Stop Lock in REV-only mode:
    // - Blocks further upward Extension push (theta_m < freeze) using REV torque
    //   up to 22.5 N*m compliant ceiling.
    // - If user relaxes (theta_m >= freeze), engages Mode 2 low-side shorting
    //   (PWM=0%, SLP awake) so the leg settles slowly rather than dropping.
    if (g_hardstop_locked) {
        if (g_enc_valid && g_theta_m_deg < g_freeze_theta_m_deg) {
            float tau_pd_gbx   = 6.0f * (g_freeze_theta_m_deg - g_theta_m_deg) - 0.35f * g_omega_m_deg_s;
            float max_lock_gbx = 22.5f * g_gamma_factor;
            tau_pd_gbx         = constrain(tau_pd_gbx, 0.0f, max_lock_gbx);
            float i_lock       = tau_pd_gbx / (GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a);
            executeRevOnlyCurrentControl(i_lock, g_omega_m_deg_s, false);
        } else {
            stopMotorDynamicBrake();
        }
        return true;
    }

    return false;
}

// ============================================================================
// 12. REPETITION TRACKER & ANTI-BACKLASH SMOOTHING
// ============================================================================
static float    g_last_rev_theta_m = 90.0f;
static bool     g_rep_going_ext    = true;
static float    g_rep_min_deg      = 120.0f;
static float    g_rep_max_deg      = 0.0f;
static float    g_rep_pk_ext_nm    = 0.0f;
static float    g_rep_ang_pk_ext   = 60.0f;
static float    g_rep_work_j       = 0.0f;
static double   g_rep_vel_sum      = 0.0;
static double   g_rep_vel_sq_sum   = 0.0;
static uint32_t g_rep_samples      = 0;

static void updateRepTracker(float tk_deg, float omega_k_degs, float tau_knee_nm) {
    if (!g_enc_valid) return;

    if (tk_deg < g_rep_min_deg) g_rep_min_deg = tk_deg;
    if (tk_deg > g_rep_max_deg) g_rep_max_deg = tk_deg;

    if (omega_k_degs < -2.0f && fabsf(tau_knee_nm) > g_rep_pk_ext_nm) {
        g_rep_pk_ext_nm  = fabsf(tau_knee_nm);
        g_rep_ang_pk_ext = tk_deg;
    }

    if (omega_k_degs < -2.0f) {
        float power_w = fabsf(tau_knee_nm * (omega_k_degs * RAD_PER_DEG));
        g_rep_work_j += power_w * CONTROL_DT_SEC;

        float v_abs = fabsf(omega_k_degs);
        g_rep_vel_sum    += v_abs;
        g_rep_vel_sq_sum += (v_abs * v_abs);
        g_rep_samples++;
    }

    float span = fmaxf(15.0f, g_cmd.rom_max_deg - g_cmd.rom_min_deg);
    if (g_rep_going_ext && tk_deg <= (g_cmd.rom_min_deg + 0.20f * span)) {
        g_rep_going_ext = false;
    } else if (!g_rep_going_ext && tk_deg >= (g_cmd.rom_max_deg - 0.20f * span)) {
        g_rep_going_ext = true;
        g_rep_count++;

        g_latest_rep_summary.pkt_type            = PKT_TOPHUB_REP_SUMMARY;
        g_latest_rep_summary.rep_index           = g_rep_count;
        g_latest_rep_summary.achieved_rom_min    = g_rep_min_deg;
        g_latest_rep_summary.achieved_rom_max    = g_rep_max_deg;
        g_latest_rep_summary.peak_torque_ext_nm  = g_rep_pk_ext_nm;
        g_latest_rep_summary.angle_at_peak_ext   = g_rep_ang_pk_ext;
        g_latest_rep_summary.peak_torque_flex_nm = 0.0f;
        g_latest_rep_summary.angle_at_peak_flex  = 0.0f;
        g_latest_rep_summary.rep_work_joules     = g_rep_work_j;

        float cov = 0.0f;
        if (g_rep_samples > 10) {
            float mean = (float)(g_rep_vel_sum / g_rep_samples);
            float var  = fmaxf(0.0f, (float)(g_rep_vel_sq_sum / g_rep_samples) - (mean * mean));
            if (mean > 1.0f) cov = (sqrtf(var) / mean) * 100.0f;
        }
        g_latest_rep_summary.vel_cov_pct = cov;
        g_latest_rep_summary.crc16 = computeCRC16(&g_latest_rep_summary, sizeof(g_latest_rep_summary) - 2);
        g_rep_summary_ready = true;

        g_rep_min_deg = 120.0f; g_rep_max_deg = 0.0f;
        g_rep_pk_ext_nm = 0.0f; g_rep_work_j = 0.0f;
        g_rep_vel_sum = 0.0; g_rep_vel_sq_sum = 0.0; g_rep_samples = 0;
    }
}

static inline float computeAntiBacklashFactor(float theta_m_deg) {
    float d_theta = fabsf(theta_m_deg - g_last_rev_theta_m);
    float ratio   = d_theta / 0.65f;
    return 1.0f - 0.70f * expf(-(ratio * ratio));
}

// ============================================================================
// 13. 500 Hz REAL-TIME CONTROL STEP (EXTENSION-ONLY / REV-ONLY)
// ============================================================================
static void runControlStep500Hz() {
    if (g_rx_pkt_pending) {
        RehabCommandPacket_t pkt;
        portENTER_CRITICAL(&g_rx_mux);
        pkt = g_rx_staging_pkt;
        g_rx_pkt_pending = false;
        portEXIT_CRITICAL(&g_rx_mux);

        // Keep local USB Serial bench tests running even if U1 (ESP_MASTER) sends idle heartbeats
        bool usb_bench_active = (!g_cmd.from_espnow) &&
                                (g_cmd.op_mode == OP_DIRECT_PWM_TEST ||
                                 g_cmd.op_mode == OP_AUTO_BENCH_TEST ||
                                 g_dirtest_active ||
                                 g_cmd.op_mode == OP_SETUP_FULL_25S ||
                                 g_cmd.op_mode == OP_SETUP_QUICK_6S);
        bool ignore_idle_hb   = usb_bench_active &&
                                (pkt.op_mode == OP_IDLE) &&
                                (pkt.estop_active != ESTOP_HARD_STOP_LOCKED);

        if (!ignore_idle_hb) {
            g_cmd.op_mode            = pkt.op_mode;
            g_cmd.pathology_profile  = pkt.pathology_profile;
            g_cmd.estop_mode         = pkt.estop_active;
            g_cmd.d_offset_m         = constrain(pkt.d_offset_m, 0.05f, 0.15f);
            g_cmd.L_leg_m            = constrain(pkt.L_leg_m, 0.20f, 0.40f);
            g_cmd.rom_min_deg        = constrain(pkt.rom_min_deg, 0.0f, 90.0f);
            g_cmd.rom_max_deg        = constrain(pkt.rom_max_deg, 15.0f, 125.0f);
            g_cmd.target_vel_deg_s   = pkt.target_vel_deg_s;
            g_cmd.target_torque_ext  = constrain(pkt.target_torque_ext, 0.0f, 68.0f);
            g_cmd.target_torque_flex = 0.0f;
            g_cmd.last_cmd_ms        = millis();
            g_cmd.from_espnow        = true;
            if (pkt.estop_active == ESTOP_NORMAL) {
                g_hardstop_locked = false;
                g_spasm_latched   = false;
            }
        }
    }

    // Step 1: Read PCNT quadrature encoder
    int16_t raw_cnt = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw_cnt);
    int16_t delta_cnt = raw_cnt - g_prev_pcnt_raw;
    g_prev_pcnt_raw   = raw_cnt;
    if (abs(raw_cnt) > 20000) {
        pcnt_counter_clear(PCNT_UNIT_0);
        g_prev_pcnt_raw = 0;
    }
    g_enc_counts += (int64_t)delta_cnt * g_cal.enc_polarity;
    if (llabs(g_enc_counts - g_enc_boot_counts) > 3) {
        g_enc_valid = true;
    }

    g_theta_m_deg = (float)g_enc_counts * DEG_PER_COUNT;

    // Step 2: Offset-pivot kinematic transform & 40 ms sliding-window velocity
    float d_off    = g_cmd.d_offset_m;
    float l_leg    = fmaxf(0.20f, g_cmd.L_leg_m);
    g_theta_k_deg  = mapArmToKneeAngleDeg(g_theta_m_deg, d_off, l_leg);
    g_gamma_factor = computeGammaFactor(g_theta_k_deg, d_off, l_leg);

    float old_tm = g_vel_hist_tm[g_vel_hist_idx];
    float old_tk = g_vel_hist_tk[g_vel_hist_idx];
    g_vel_hist_tm[g_vel_hist_idx] = g_theta_m_deg;
    g_vel_hist_tk[g_vel_hist_idx] = g_theta_k_deg;
    g_vel_hist_idx = (g_vel_hist_idx + 1) % VEL_WINDOW_TICKS;

    const float dt_win = (float)VEL_WINDOW_TICKS * CONTROL_DT_SEC; // 0.04 s
    float win_wm = (g_theta_m_deg - old_tm) / dt_win;
    float win_wk = (g_theta_k_deg - old_tk) / dt_win;
    g_omega_m_deg_s = 0.80f * g_omega_m_deg_s + 0.20f * win_wm;
    g_omega_k_deg_s = 0.80f * g_omega_k_deg_s + 0.20f * win_wk;

    // Step 3: Sample analog sensors & update Brake Chopper
    sampleAnalogSensors500Hz();
    updateBrakeChopper(g_vbus_volts);

    float tau_em_gbx = GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a * g_motor_current_a;
    g_gbx_torque_nm  = constrain(tau_em_gbx, -GBX_MAX_TORQUE_NM, GBX_MAX_TORQUE_NM);
    g_knee_torque_nm = g_gbx_torque_nm / g_gamma_factor;
    g_pad_force_n    = g_knee_torque_nm / l_leg;

    // Step 4: Safety Supervisor
    if (evaluateSafetySupervisor(g_cmd.op_mode)) {
        return;
    }

    // Step 5: Direction polarity test (if active)
    if (g_dirtest_active) {
        runDirectionVerifyStep();
        return;
    }

    // Step 6: Static Setup
    if (g_cmd.op_mode == OP_SETUP_FULL_25S || g_cmd.op_mode == OP_SETUP_QUICK_6S) {
        if (g_setup_stage == SETUP_STAGE_IDLE) {
            startStaticCalibration();
        }
        runStaticCalibrationStep();
        return;
    } else {
        g_setup_stage = SETUP_STAGE_IDLE;
    }

    switch (g_cmd.op_mode) {
        case OP_IDLE:
        default: {
            g_target_curve_nm = 0.0f;
            stopMotorCoastRelease();
            break;
        }

        // Passive CPM Lift cannot operate with REV wired for Flexion resistance
        case OP_CPM: {
            g_target_curve_nm = 0.0f;
            g_cmd.op_mode     = OP_IDLE;
            stopMotorCoastRelease();
            Serial.println("[WARN] OP_CPM unavailable in REV-only Extension-Resistance mode (requires working DIR). Returning to IDLE.");
            break;
        }

        // Mode 2: Physiological Curve-Shaped Extension Isotonic Resistance
        case OP_ISOTONIC: {
            float tau_knee_ref = computeExtensionTorqueTarget(
                g_theta_k_deg, g_cmd.target_torque_ext, g_cmd.pathology_profile
            );
            g_target_curve_nm = tau_knee_ref;

            if (!g_enc_valid) {
                stopMotorCoastRelease();
                break;
            }

            // Extension motion (theta_k decreasing) -> resist with REV Flexion torque
            if (g_omega_k_deg_s < -1.5f) {
                float tau_gbx_req = constrain(g_gamma_factor * tau_knee_ref, 0.0f, 87.4f);
                float c_lash      = computeAntiBacklashFactor(g_theta_m_deg);
                float i_cmd       = fmaxf(0.0f, (c_lash * tau_gbx_req - g_cal.tau_fric_gbx_nm) / 4.246f);

                // Electronic terminal extension wall at rom_min_deg
                if (g_theta_k_deg < g_cmd.rom_min_deg) {
                    i_cmd += 3.5f * (g_cmd.rom_min_deg - g_theta_k_deg);
                }
                executeRevOnlyCurrentControl(i_cmd, g_omega_m_deg_s, false);
            } else if (g_theta_k_deg < g_cmd.rom_min_deg) {
                // Static hold against hyper-extension past rom_min_deg
                float i_wall = 3.5f * (g_cmd.rom_min_deg - g_theta_k_deg);
                executeRevOnlyCurrentControl(i_wall, g_omega_m_deg_s, false);
            } else {
                // Resting or returning into Flexion (theta_k increasing): release cleanly
                g_last_rev_theta_m = g_theta_m_deg;
                bool returning = (g_omega_k_deg_s > 1.5f);
                executeRevOnlyCurrentControl(0.0f, g_omega_m_deg_s, returning || USE_SLP_COAST_TRANSISTOR);
            }
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            break;
        }

        // Mode 3: Accommodating Extension Isokinetic Resistance
        case OP_ISOKINETIC: {
            float omega_iso = constrain(g_cmd.target_vel_deg_s, 15.0f, 120.0f);
            float tau_cap_k = computeExtensionTorqueTarget(
                g_theta_k_deg, g_cmd.target_torque_ext, g_cmd.pathology_profile
            );
            g_target_curve_nm = tau_cap_k;

            if (!g_enc_valid) {
                stopMotorCoastRelease();
                break;
            }

            if (g_omega_k_deg_s < -1.5f) {
                float abs_wk = fabsf(g_omega_k_deg_s);
                float i_cmd  = 0.0f;
                if (abs_wk > (0.85f * omega_iso)) {
                    float vel_excess = abs_wk - (0.85f * omega_iso);
                    float tau_iso_k  = constrain(3.2f * vel_excess, 0.0f, tau_cap_k);
                    float tau_gbx    = constrain(g_gamma_factor * tau_iso_k, 0.0f, 87.4f);
                    i_cmd            = fmaxf(0.0f, (tau_gbx - g_cal.tau_fric_gbx_nm) / 4.246f);
                }
                if (g_theta_k_deg < g_cmd.rom_min_deg) {
                    i_cmd += 3.5f * (g_cmd.rom_min_deg - g_theta_k_deg);
                }
                executeRevOnlyCurrentControl(i_cmd, g_omega_m_deg_s, (i_cmd < 0.08f) && USE_SLP_COAST_TRANSISTOR);
            } else if (g_theta_k_deg < g_cmd.rom_min_deg) {
                float i_wall = 3.5f * (g_cmd.rom_min_deg - g_theta_k_deg);
                executeRevOnlyCurrentControl(i_wall, g_omega_m_deg_s, false);
            } else {
                executeRevOnlyCurrentControl(0.0f, g_omega_m_deg_s, (g_omega_k_deg_s > 1.5f) || USE_SLP_COAST_TRANSISTOR);
            }
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            break;
        }

        // Mode 4: Extension Isometric Hold at lock_tk
        case OP_ISOMETRIC: {
            float lock_tk   = constrain(g_cmd.rom_max_deg, 15.0f, 105.0f);
            float lock_tm   = mapKneeToArmAngleDeg(lock_tk, d_off, l_leg);
            float max_tau_k = constrain(g_cmd.target_torque_ext, 10.0f, 68.0f);
            g_target_curve_nm = max_tau_k;

            if (!g_enc_valid) {
                stopMotorCoastRelease();
                break;
            }

            // Only push into Flexion (REV) if patient pushes above lock angle (theta_m < lock_tm).
            // Never drive downward when patient relaxes (theta_m >= lock_tm).
            if (g_theta_m_deg < lock_tm) {
                float tau_pd_gbx = 8.0f * (lock_tm - g_theta_m_deg) - 0.40f * g_omega_m_deg_s;
                float max_gbx    = constrain(g_gamma_factor * max_tau_k, 10.0f, 87.4f);
                tau_pd_gbx       = constrain(tau_pd_gbx, 0.0f, max_gbx);
                float i_cmd      = tau_pd_gbx / (GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a);
                executeRevOnlyCurrentControl(i_cmd, g_omega_m_deg_s, false);
            } else {
                stopMotorDynamicBrake();
            }
            break;
        }

        case OP_DIRECT_PWM_TEST: {
            float pct = constrain(g_cmd.target_vel_deg_s, 0.0f, 100.0f);
            uint32_t duty = (uint32_t)lroundf((pct * 0.01f) * PWM_MAX_DUTY);
            setMotorRevOutput(duty, false);
            break;
        }

        case OP_AUTO_BENCH_TEST: {
            uint32_t elapsed = millis() - g_bench_test_start_ms;
            if (elapsed < 1500) {
                setMotorRevOutput((uint32_t)(0.30f * PWM_MAX_DUTY), false);
            } else {
                stopMotorCoastRelease();
                g_cmd.op_mode = OP_IDLE;
                Serial.println("[TEST] REV-only 1.5s Motor Test Complete -> IDLE.");
            }
            break;
        }
    }
}

// ============================================================================
// 14. ESP-NOW WIRELESS CALLBACKS & 50 Hz TELEMETRY
// ============================================================================
static void processIncomingCommandPacket(const uint8_t *sender_mac, const uint8_t *data, int len) {
    if (len != sizeof(RehabCommandPacket_t)) return;
    RehabCommandPacket_t pkt;
    memcpy(&pkt, data, sizeof(pkt));
    if (pkt.pkt_type != PKT_HEARTBEAT_CMD || pkt.target_node != NODE_TOPHUB) return;

    uint16_t calc_crc = computeCRC16(&pkt, sizeof(pkt) - 2);
    if (calc_crc != pkt.crc16) return;

    if (!g_master_mac_known && sender_mac != nullptr) {
        memcpy(g_master_mac, sender_mac, 6);
        esp_now_peer_info_t peer = {};
        memcpy(peer.peer_addr, g_master_mac, 6);
        peer.channel = REHAB_WIFI_CHANNEL;
        peer.encrypt = false;
        if (!esp_now_is_peer_exist(g_master_mac)) {
            esp_now_add_peer(&peer);
        }
        g_master_mac_known = true;
    }

    portENTER_CRITICAL(&g_rx_mux);
    g_rx_staging_pkt = pkt;
    g_rx_pkt_pending = true;
    portEXIT_CRITICAL(&g_rx_mux);
}

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
static void onEspNowDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    processIncomingCommandPacket(recv_info->src_addr, data, len);
}
#else
static void onEspNowDataRecv(const uint8_t *mac_addr, const uint8_t *data, int len) {
    processIncomingCommandPacket(mac_addr, data, len);
}
#endif

static void initEspNowChannel1() {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(REHAB_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() == ESP_OK) {
        esp_now_register_recv_cb(onEspNowDataRecv);
        esp_now_peer_info_t bcast_peer = {};
        memset(bcast_peer.peer_addr, 0xFF, 6);
        bcast_peer.channel = REHAB_WIFI_CHANNEL;
        bcast_peer.encrypt = false;
        esp_now_add_peer(&bcast_peer);
    }
}

static uint16_t g_telemetry_seq = 0;

static void sendTelemetry50Hz() {
    TopHubTelemetryPacket_t pkt = {};
    pkt.pkt_type = PKT_TOPHUB_TELEMETRY;
    pkt.node_id  = NODE_TOPHUB;
    pkt.seq_num  = ++g_telemetry_seq;

    uint8_t flags = 0;
    if (g_is_homed)                 flags |= STATE_FLAG_HOMED;
    if (g_cal.calibrated)           flags |= STATE_FLAG_CALIBRATED;
    if (g_chopper_duty_pct > 1.0f)  flags |= STATE_FLAG_CHOPPER_ACTIVE;
    if (g_spasm_latched)            flags |= STATE_FLAG_SPASM_STOP;
    if (g_hardstop_locked)          flags |= STATE_FLAG_HARDSTOP_LOCKED;
    pkt.state_flags        = flags;
    pkt.current_rep        = g_rep_count;
    pkt.knee_angle_deg     = g_theta_k_deg;
    pkt.knee_vel_deg_s     = g_omega_k_deg_s;
    pkt.knee_torque_nm     = g_knee_torque_nm;
    pkt.gearbox_torque_nm  = g_gbx_torque_nm;
    pkt.pad_force_newtons  = g_pad_force_n;
    pkt.motor_current_amps = g_motor_current_a;
    pkt.target_curve_nm    = g_target_curve_nm;
    pkt.vbus_volts         = g_vbus_volts;
    pkt.chopper_duty_pct   = g_chopper_duty_pct;
    pkt.crc16              = computeCRC16(&pkt, sizeof(pkt) - 2);

    esp_now_send(g_master_mac, (const uint8_t *)&pkt, sizeof(pkt));

    if (g_rep_summary_ready) {
        g_rep_summary_ready = false;
        esp_now_send(g_master_mac, (const uint8_t *)&g_latest_rep_summary, sizeof(g_latest_rep_summary));
    }
}

// ============================================================================
// 15. USB SERIAL CLI
// ============================================================================
static char     s_cli_buf[64];
static uint8_t  s_cli_len        = 0;
static uint32_t s_cli_last_rx_ms = 0;

static void printHelp() {
    Serial.println("[CLI] Commands (Extension-Only / REV-Only):");
    Serial.println("  home [deg] : Latch current arm position as knee angle (default 0 deg at STOP1)");
    Serial.println("  cal        : Static ADC/Vbus zero calibration & save to NVS");
    Serial.println("  dirtest    : Pulse REV 18% (250ms) to auto-verify ENC1 & ACS712 polarities");
    Serial.println("  iso [nm]   : Extension Isotonic Resistance (default 35 N*m peak)");
    Serial.println("  isok [vel] : Extension Isokinetic Resistance (default 60 deg/s, 45 N*m cap)");
    Serial.println("  isom [deg] : Extension Isometric Hold at angle (default 60 deg, 30 N*m cap)");
    Serial.println("  pwm <0..100> : Direct REV PWM bench test");
    Serial.println("  test       : 1.5s 30% REV bench pulse");
    Serial.println("  freq <hz>  : Set PWM frequency (100..20000 Hz)");
    Serial.println("  stop       : Hard-Stop lock (blocks extension / Mode 2 brake)");
    Serial.println("  release    : Release motor (IDLE)");
    Serial.println("  adc / status / help");
}

static void executeCLICommand(String line) {
    line.trim();
    if (line.length() == 0) return;
    line.toLowerCase();

    g_cmd.from_espnow = false;
    g_cmd.last_cmd_ms = millis();

    if (line.equalsIgnoreCase("help") || line.equals("?")) {
        printHelp();
    } else if (line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("lock")) {
        g_cmd.estop_mode     = ESTOP_HARD_STOP_LOCKED;
        g_hardstop_locked    = true;
        g_freeze_theta_m_deg = g_theta_m_deg;
        Serial.println("[CLI] HARD STOP LOCKED engaged (blocks extension; Mode 2 damped lower).");
    } else if (line.equalsIgnoreCase("release") || line.equalsIgnoreCase("idle")) {
        g_cmd.op_mode     = OP_IDLE;
        g_cmd.estop_mode  = ESTOP_HARD_STOP_RELEASE;
        g_hardstop_locked = false;
        g_spasm_latched   = false;
        stopMotorCoastRelease();
        Serial.println("[CLI] Arm RELEASED -> IDLE.");
    } else if (line.startsWith("home") || line.startsWith("HOME")) {
        float known_tk = 0.0f;
        if (line.length() > 4) {
            known_tk = constrain(line.substring(4).toFloat(), 0.0f, 125.0f);
        }
        float known_tm = mapKneeToArmAngleDeg(known_tk, g_cmd.d_offset_m, g_cmd.L_leg_m);
        latchEncoderAngleDeg(known_tm);
        g_theta_k_deg = known_tk;
        g_is_homed    = true;
        g_enc_valid   = true;
        Serial.printf("[CLI] Homed manually: Knee=%.1f deg (Arm=%.1f deg).\n", known_tk, known_tm);
    } else if (line.equalsIgnoreCase("cal")) {
        g_cmd.estop_mode  = ESTOP_NORMAL;
        g_hardstop_locked = false;
        g_spasm_latched   = false;
        g_setup_stage     = SETUP_STAGE_IDLE;
        g_cmd.op_mode     = OP_SETUP_FULL_25S;
        Serial.println("[CLI] Running static ADC/Vbus zero calibration...");
    } else if (line.equalsIgnoreCase("dirtest")) {
        g_cmd.estop_mode    = ESTOP_NORMAL;
        g_hardstop_locked   = false;
        g_spasm_latched     = false;
        g_cmd.op_mode       = OP_IDLE;
        g_dirtest_active    = true;
        g_dirtest_start_ms  = millis();
        g_dirtest_start_cnt = g_enc_counts;
        g_dirtest_acs_sum   = 0.0;
        g_dirtest_samples   = 0;
        Serial.println("[CLI] Pulsing REV 18% (250ms) to verify Flexion (+theta_k) & ACS712 polarity...");
    } else if (line.equalsIgnoreCase("test")) {
        g_cmd.estop_mode      = ESTOP_NORMAL;
        g_hardstop_locked     = false;
        g_spasm_latched       = false;
        g_bench_test_start_ms = millis();
        g_cmd.op_mode         = OP_AUTO_BENCH_TEST;
        Serial.println("[CLI] Starting 1.5s 30% REV bench test...");
    } else if (line.startsWith("iso") && !line.startsWith("isok") && !line.startsWith("isom")) {
        float ext_nm = (line.length() > 3) ? constrain(line.substring(3).toFloat(), 5.0f, 68.0f) : 35.0f;
        g_cmd.estop_mode        = ESTOP_NORMAL;
        g_hardstop_locked       = false;
        g_spasm_latched         = false;
        g_cmd.op_mode           = OP_ISOTONIC;
        g_cmd.target_torque_ext = ext_nm;
        Serial.printf("[CLI] Mode 2: Extension Isotonic (%.1f N*m peak) started.\n", ext_nm);
    } else if (line.startsWith("isok")) {
        float vel = (line.length() > 4) ? constrain(line.substring(4).toFloat(), 15.0f, 120.0f) : 60.0f;
        g_cmd.estop_mode        = ESTOP_NORMAL;
        g_hardstop_locked       = false;
        g_spasm_latched         = false;
        g_cmd.op_mode           = OP_ISOKINETIC;
        g_cmd.target_vel_deg_s  = vel;
        g_cmd.target_torque_ext = 45.0f;
        Serial.printf("[CLI] Mode 3: Extension Isokinetic (%.1f deg/s ceiling, 45 N*m cap) started.\n", vel);
    } else if (line.startsWith("isom")) {
        float lock_deg = (line.length() > 4) ? constrain(line.substring(4).toFloat(), 15.0f, 105.0f) : 60.0f;
        g_cmd.estop_mode        = ESTOP_NORMAL;
        g_hardstop_locked       = false;
        g_spasm_latched         = false;
        g_cmd.op_mode           = OP_ISOMETRIC;
        g_cmd.rom_max_deg       = lock_deg;
        g_cmd.target_torque_ext = 30.0f;
        Serial.printf("[CLI] Mode 4: Extension Isometric Hold at %.1f deg (30 N*m yield) started.\n", lock_deg);
    } else if (line.startsWith("pwm") || line.startsWith("PWM")) {
        float pct = line.substring(3).toFloat();
        if (pct < 0.0f) {
            Serial.println("[WARN] Negative (FWD) PWM rejected: hardware DIR is REV-only. Use positive 0..100%.");
            pct = 0.0f;
        }
        pct = constrain(pct, 0.0f, 100.0f);
        if (pct < 0.5f) {
            g_cmd.op_mode     = OP_IDLE;
            g_cmd.estop_mode  = ESTOP_HARD_STOP_RELEASE;
            g_hardstop_locked = false;
            stopMotorCoastRelease();
            Serial.println("[CLI] PWM 0% -> IDLE.");
        } else {
            g_cmd.estop_mode       = ESTOP_NORMAL;
            g_hardstop_locked      = false;
            g_spasm_latched        = false;
            g_cmd.op_mode          = OP_DIRECT_PWM_TEST;
            g_cmd.target_vel_deg_s = pct;
            uint32_t duty          = (uint32_t)lroundf((pct * 0.01f) * PWM_MAX_DUTY);
            setMotorRevOutput(duty, false);
            Serial.printf("[CLI] Direct REV PWM: %.1f%% (%lu/1023 at %lu Hz)\n",
                          pct, (unsigned long)duty, (unsigned long)g_md_pwm_freq_hz);
        }
    } else if (line.startsWith("freq") || line.startsWith("FREQ")) {
        uint32_t new_freq = (uint32_t)line.substring(4).toInt();
        if (new_freq >= 100 && new_freq <= 25000) {
            g_md_pwm_freq_hz  = new_freq;
            int32_t prev_duty = s_last_motor_duty;
            initPWMHardware();
            if (prev_duty > 0) {
                setMotorRevOutput((uint32_t)prev_duty, false);
            }
            Serial.printf("[CLI] Motor PWM Frequency updated to %lu Hz.\n", (unsigned long)g_md_pwm_freq_hz);
        } else {
            Serial.println("[CLI] Usage: freq <100..25000>");
        }
    } else if (line.equalsIgnoreCase("adc")) {
        Serial.printf(
            "[ADC] VBUS(34): raw=%4u %6.1fmV (%s -> %.2fV) | ACS(35): raw=%4u %6.1fmV (%s -> %.2fA, z=%.1fmV) | ENC:%lld (%s -> %.1f deg) | PWM:%.1f%% @%luHz\n",
            g_vbus_raw_adc, g_vbus_raw_mv, g_vbus_valid ? "OK" : "NOM24V", g_vbus_volts,
            g_acs_raw_adc, g_acs_raw_mv, g_acs_valid ? "OK" : "OL", g_motor_current_a, g_cal.v_adc_zero_mv,
            (long long)g_enc_counts, g_enc_valid ? "OK" : "UNPLUG", g_theta_k_deg,
            g_active_pwm_pct, (unsigned long)g_md_pwm_freq_hz
        );
    } else if (line.equalsIgnoreCase("status")) {
        Serial.printf(
            "[STATUS] Mode:%u | REV PWM:%.1f%% | Coast:%d | Knee:%.1f deg (%.1f deg/s) | I_m:%.2f A | Vbus:%.2f V | Homed:%d\n",
            g_cmd.op_mode, g_active_pwm_pct, g_is_coasting ? 1 : 0,
            g_theta_k_deg, g_omega_k_deg_s, g_motor_current_a, g_vbus_volts, g_is_homed ? 1 : 0
        );
    } else {
        Serial.println("[CLI] Unknown command. Type 'help' for list.");
    }
}

static void pollSerialCLI() {
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        s_cli_last_rx_ms = millis();
        if (c == '\n' || c == '\r') {
            if (s_cli_len > 0) {
                s_cli_buf[s_cli_len] = '\0';
                s_cli_len = 0;
                executeCLICommand(String(s_cli_buf));
            }
        } else if (s_cli_len < sizeof(s_cli_buf) - 1) {
            s_cli_buf[s_cli_len++] = c;
        }
    }
    if (s_cli_len > 0 && (millis() - s_cli_last_rx_ms > 150)) {
        s_cli_buf[s_cli_len] = '\0';
        s_cli_len = 0;
        executeCLICommand(String(s_cli_buf));
    }
}

// ============================================================================
// 16. ARDUINO SETUP() & COOPERATIVE REAL-TIME LOOP()
// ============================================================================
void setup() {
    initPWMHardware();

    Serial.begin(115200);
    delay(100);
    Serial.println("\n================================================================");
    Serial.println(" PROJECT DOULA — U2 TOPHUB (EXTENSION-ONLY / REV-ONLY EDITION)");
    Serial.println("================================================================");

    pinMode(PIN_VBUS_SENSE, INPUT);
    pinMode(PIN_ACS712_SENSE, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBUS_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_ACS712_SENSE, ADC_11db);
    if (USE_OPTIONAL_LOADCELL) {
        pinMode(PIN_LOADCELL_ADC, INPUT);
        analogSetPinAttenuation(PIN_LOADCELL_ADC, ADC_11db);
    }

    initEncoderPCNT();
    latchEncoderAngleDeg(mapKneeToArmAngleDeg(90.0f, g_cmd.d_offset_m, g_cmd.L_leg_m));

    loadCalibrationFromNVS();
    sampleAnalogSensors500Hz();

    Serial.printf("[NVS] Calibrated:%d | V_zero:%.1f mV | Vbus:%.2f V | enc_pol:%+d | acs_pol:%+d\n",
                  g_cal.calibrated, g_cal.v_adc_zero_mv, g_cal.v_bus_idle_v,
                  g_cal.enc_polarity, g_cal.acs_polarity);

    if (ENABLE_ESPNOW_WIFI) {
        initEspNowChannel1();
        Serial.print("[WIFI] U2 MAC: ");
        Serial.println(WiFi.macAddress());
    } else {
        Serial.println("[WIFI] Disabled (ENABLE_ESPNOW_WIFI = false).");
    }
    Serial.printf("[PWM] Motor PWM default: %lu Hz (10-bit) on GPIO%d | DIR GPIO%d=LOW\n",
                  (unsigned long)g_md_pwm_freq_hz, PIN_MD_PWM, PIN_MD_DIR);
    printHelp();
}

void loop() {
    static uint32_t last_ctrl_us  = 0;
    static uint32_t last_telem_ms = 0;

    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_ctrl_us) >= CONTROL_PERIOD_US) {
        if ((uint32_t)(now_us - last_ctrl_us) > (CONTROL_PERIOD_US * 4)) {
            last_ctrl_us = now_us;
        } else {
            last_ctrl_us += CONTROL_PERIOD_US;
        }
        runControlStep500Hz();
    }

    uint32_t now_ms = millis();
    if (ENABLE_ESPNOW_WIFI && (uint32_t)(now_ms - last_telem_ms) >= 20) {
        last_telem_ms = now_ms;
        sendTelemetry50Hz();
    }

    pollSerialCLI();
}
