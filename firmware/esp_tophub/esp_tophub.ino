// ============================================================================
// PROJECT DOULA (VK26-IITI-HT-0240) — TOP-HUB KNEE REHAB CONTROLLER (U2: ESP_TOPHUB)
// Target MCU : ESP32-S NodeMCU V1.1 (240 MHz Dual-Core, Arduino-ESP32 v2.x / v3.x)
// Actuators  : U201 (Rhino MD20Amp) -> M1 (MY1016 24V 250W) + GBX1 (1:52, 85%, 60/100 N*m)
//              U202 (30A 2-Screw MOSFET) -> R1||R2 (1.0 Ohm 200W Series Brake Chopper)
// Sensors    : ENC1 (360 PPR NPN on M1 Shaft = 74,880 CPR on GPIO32/33, 80 Ohm + INPUT_PULLUP)
//              U203 (Inline ACS712-30A on GPIO35 via R13/R14 22k/22k -> 1.25V@0A, 33mV/A)
//              VBUS_SENSE (GPIO34 via R11=73.33k / R12=4.6k -> 1.417V@24V, mult 16.942)
// Network    : ESP-NOW on 2.4 GHz Wi-Fi Channel 1 (50 Hz Telemetry + <3ms Hard-Stop)
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <driver/pcnt.h>
#include <math.h>

// ============================================================================
// 0. EMBEDDED ESP-NOW PROTOCOL DEFINITIONS (No external .h tab required!)
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
    OP_DIRECT_PWM_TEST = 9,  // Direct open-loop PWM bench test ("pwm 40" / "pwm -40")
    OP_AUTO_BENCH_TEST = 10  // Automatic bidirectional bench test ("test")
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
// 1. HARDWARE PIN DEFINITIONS (Matches Sheet 2 & Wiring Guide Exactly)
// ============================================================================
static const int PIN_MD_PWM       = 25; // U201 Rhino MD20Amp PWM
static const int PIN_MD_DIR       = 26; // U201 Rhino MD20Amp DIR
static const int PIN_CHOP_PWM     = 27; // U202 30A MOSFET Control+ (5 kHz, R16=22k pull-down)
static const int PIN_ENC1_A       = 32; // ENC1 Out A (R3=80 Ohm series + INPUT_PULLUP)
static const int PIN_ENC1_B       = 33; // ENC1 Out B (R5=80 Ohm series + INPUT_PULLUP)
static const int PIN_VBUS_SENSE   = 34; // ADC1_CH6: R11(73.33k) / R12(4.6k) Bus Divider
static const int PIN_ACS712_SENSE = 35; // ADC1_CH7: R13(22k) / R14(22k) ACS712-30A Divider
static const int PIN_LOADCELL_ADC = 36; // Optional Expansion: Analog Shin-Pad Load Cell (VP)

static const bool USE_OPTIONAL_LOADCELL = false;

// ============================================================================
// 2. TIMING, PWM & PHYSICAL CONSTANTS
// ============================================================================
static const uint32_t CONTROL_PERIOD_US  = 2000;  // 500 Hz (2.0 ms) deterministic control loop
static const float    CONTROL_DT_SEC     = 0.002f;
static const float    CONTROL_FREQ_HZ    = 500.0f;

static uint32_t       g_md_pwm_freq_hz   = 1000;  // Default 1 kHz (matches Arduino analogWrite; adjustable via 'freq <hz>')
static const uint32_t CHOP_PWM_FREQ_HZ   = 5000;  // 5 kHz brake chopper PWM
static const uint8_t  PWM_RES_BITS       = 10;    // 10-bit resolution (0..1023)
static const int32_t  PWM_MAX_DUTY       = 1023;
static const int      LEDC_CH_MOTOR      = 0;     // Used on Arduino-ESP32 v2.x
static const int      LEDC_CH_CHOPPER    = 1;     // Used on Arduino-ESP32 v2.x

// Drivetrain & Sensor Constants
static const float GEAR_RATIO            = 52.0f;
static const float ENC_CPR_MOTOR         = 1440.0f;                     // 360 PPR * 4x quadrature
static const float ENC_CPR_OUTPUT        = ENC_CPR_MOTOR * GEAR_RATIO;  // 74,880 counts/output rev
static const float DEG_PER_COUNT         = 360.0f / ENC_CPR_OUTPUT;     // 0.00480769 deg/count
static const float RAD_PER_DEG           = 0.0174532925f;
static const float DEG_PER_RAD           = 57.2957795f;

// Voltage & Current Scaling Constants
static const float VBUS_DIV_MULT         = 16.942f; // (73.333k + 4.6k) / 4.6k -> 1.417V @ 24.0V
static const float ACS712_MV_PER_AMP     = 33.0f;   // 66 mV/A * 0.50 (22k/22k divider)

// Hardware Ratings & Safety Limits
static const float MOTOR_MAX_CURRENT_A   = 20.0f;   // Rhino MD20Amp continuous clamp (<100 N*m gbx)
static const float MOTOR_RATED_CURRENT_A = 13.5f;   // MY1016 continuous thermal rating (~60 N*m gbx)
static const float GBX_RATED_TORQUE_NM   = 60.0f;
static const float GBX_MAX_TORQUE_NM     = 100.0f;
static const float VBUS_OVERVOLT_FAULT_V = 28.5f;   // Absolute bus overvoltage release ceiling
static const float THETA_HARD_STOP_DEG   = 16.70f;  // Arm angle theta_m when knee is at 0 deg extension

// ============================================================================
// 3. CALIBRATION & NVS STRUCTURE (Identified in 6-Stage Setup Routine)
// ============================================================================
struct CalibrationData_t {
    bool    calibrated;
    int8_t  enc_polarity;       // +1 or -1 (auto-detected in Stage 2)
    int8_t  dir_polarity;       // HIGH/LOW mapping for extension (+theta_m or -theta_m)
    int8_t  acs_polarity;       // +1 or -1 (auto-detected in Stage 2/3 so PI never runs away)
    float   v_adc_zero_mv;      // Stage 1: ACS712 0A voltage [mV] (nominal 1250.0 mV)
    float   v_bus_idle_v;       // Stage 1: Idle 24V bus voltage [V] (nominal 24.0 V)
    float   pad_tare_mv;        // Stage 1: Optional load cell tare [mV]
    float   backlash_deg;       // Stage 2: Measured gearbox backlash [deg] (<= 0.833 deg)
    float   r_loop_ohm;         // Stage 3: Blocked-rotor armature loop resistance [Ohm] (nom 0.40)
    float   v_brush_v;          // Stage 3: Brush + H-bridge dead-zone drop [V] (nom 0.35 V)
    float   k_t_nm_per_a;       // Stage 4: Motor torque/EMF constant [N*m/A] (nom 0.0694)
    float   i_no_load_a;        // Stage 4: No-load / breakaway current I_0 [A] (nom 1.00 A)
    float   b_visc_a_per_rads;  // Stage 4: Viscous friction coeff [A/(rad/s)] (nom 0.0015)
    float   tau_fric_gbx_nm;    // Stage 4: Reflected gearbox breakaway friction [N*m] (nom 2.50)
    float   tau_arm_grav_nm;    // Stage 4: Metal orthosis arm peak gravity torque [N*m] (nom 3.50)
    float   tau_limb_grav_nm;   // Stage 5: Patient lower-leg peak gravity torque [N*m] (nom 8.50)
    float   eta_fwd;            // Stage 5: Gearbox forward efficiency (nom 0.85)
    float   eta_back;           // Stage 5: Gearbox back-driving efficiency (nom 0.85)
};

static CalibrationData_t g_cal = {
    false,      // calibrated
    +1,         // enc_polarity
    +1,         // dir_polarity
    +1,         // acs_polarity
    1250.0f,    // v_adc_zero_mv
    24.00f,     // v_bus_idle_v
    0.0f,       // pad_tare_mv
    0.65f,      // backlash_deg (<= 50 arcmin = 0.833 deg)
    0.40f,      // r_loop_ohm
    0.35f,      // v_brush_v
    0.0694f,    // k_t_nm_per_a
    1.00f,      // i_no_load_a
    0.0015f,    // b_visc_a_per_rads
    2.50f,      // tau_fric_gbx_nm
    3.50f,      // tau_arm_grav_nm
    8.50f,      // tau_limb_grav_nm
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
    0.0f,    // rom_min_deg = 0 deg (full extension)
    110.0f,  // rom_max_deg = 110 deg (flexion)
    2.0f,    // target_vel_deg_s (default 2 deg/s for CPM)
    35.0f,   // target_torque_ext [N*m]
    20.0f,   // target_torque_flex [N*m]
    0,
    false
};

// Lock-free staging buffer for incoming ESP-NOW command packets from Core 0 Wi-Fi callback
static RehabCommandPacket_t g_rx_staging_pkt;
static volatile bool        g_rx_pkt_pending = false;

// Live sensor validity flags (Auto-detects saturated 4095 / 3145mV or unplugged sensors)
static bool     g_vbus_valid         = false;
static bool     g_acs_valid          = false;
static bool     g_enc_valid          = false;

static uint16_t g_vbus_raw_adc       = 0;
static float    g_vbus_raw_mv        = 0.0f;
static uint16_t g_acs_raw_adc        = 0;
static float    g_acs_raw_mv         = 0.0f;

// Live control state
static bool     g_is_homed           = false;
static bool     g_spasm_latched      = false;
static bool     g_hardstop_locked    = false;
static float    g_freeze_theta_m_deg = 60.0f;

static int64_t  g_enc_counts         = 0;
static int64_t  g_enc_boot_counts    = 0;
static int16_t  g_prev_pcnt_raw      = 0;

static float    g_theta_m_deg        = 60.0f; // Motor/Gearbox arm angle [deg]
static float    g_omega_m_deg_s      = 0.0f;  // Arm angular velocity [deg/s]
static float    g_theta_k_deg        = 50.0f; // Anatomical knee angle [deg]
static float    g_omega_k_deg_s      = 0.0f;  // Anatomical knee velocity [deg/s]
static float    g_gamma_factor       = 1.23f; // Offset-pivot torque ratio Gamma(theta_k)

static float    g_vbus_volts         = 24.0f;
static float    g_chopper_duty_pct   = 0.0f;
static float    g_motor_current_a    = 0.0f;  // Signed armature current I_m [A]
static float    g_i_ref_a            = 0.0f;  // Commanded armature current [A]
static float    g_active_pwm_pct     = 0.0f;  // Signed applied PWM percentage [-100..+100%]
static float    g_gbx_torque_nm      = 0.0f;  // Gearbox shaft torque [N*m]
static float    g_knee_torque_nm     = 0.0f;  // Anatomical knee torque [N*m]
static float    g_pad_force_n        = 0.0f;  // Normal shin-pad force [N]
static float    g_target_curve_nm    = 0.0f;  // Instantaneous target knee torque [N*m]

static uint32_t g_bench_test_start_ms = 0;

// Repetition tracking & summary notification
static uint16_t g_rep_count          = 0;
static bool     g_rep_summary_ready  = false;
static TopHubRepSummaryPacket_t g_latest_rep_summary;

// Master MAC Address (Auto-learned on first ESP-NOW packet; Broadcast fallback)
static uint8_t g_master_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static bool    g_master_mac_known = false;

// ============================================================================
// 5. CHANGE-ONLY HARDWARE PWM & DIRECTION DRIVER (GPIO25 / GPIO26 / GPIO27)
// ============================================================================
static int32_t s_last_motor_duty   = -1;
static int8_t  s_last_motor_dir    = -1;
static int32_t s_last_chopper_duty = -1;

static inline void initPWMHardware() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
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

// Updates GPIO26 (DIR) and GPIO25 (PWM) ONLY when direction or duty cycle changes!
// Prevents 1 kHz LEDC timer re-latching glitches.
static inline void setMotorDrive(bool dir_high, uint32_t duty) {
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;

    int8_t target_dir = dir_high ? 1 : 0;
    if (target_dir != s_last_motor_dir) {
        digitalWrite(PIN_MD_DIR, dir_high ? HIGH : LOW);
        s_last_motor_dir = target_dir;
    }

    if ((int32_t)duty != s_last_motor_duty) {
        writeMotorPWMRaw(duty);
        s_last_motor_duty = (int32_t)duty;
    }

    float pct_mag = ((float)duty / (float)PWM_MAX_DUTY) * 100.0f;
    g_active_pwm_pct = dir_high ? pct_mag : -pct_mag;
}

static inline void stopMotorImmediate() {
    if (s_last_motor_duty != 0) {
        writeMotorPWMRaw(0);
        s_last_motor_duty = 0;
    }
    g_active_pwm_pct = 0.0f;
    g_i_ref_a        = 0.0f;
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

    pcnt_config_t pcnt_ch0 = {};
    pcnt_ch0.pulse_gpio_num = PIN_ENC1_A;
    pcnt_ch0.ctrl_gpio_num  = PIN_ENC1_B;
    pcnt_ch0.unit           = PCNT_UNIT_0;
    pcnt_ch0.channel        = PCNT_CHANNEL_0;
    pcnt_ch0.pos_mode       = PCNT_COUNT_INC;
    pcnt_ch0.neg_mode       = PCNT_COUNT_DEC;
    pcnt_ch0.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_ch0.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_ch0.counter_h_lim  = 30000;
    pcnt_ch0.counter_l_lim  = -30000;
    pcnt_unit_config(&pcnt_ch0);

    pcnt_config_t pcnt_ch1 = {};
    pcnt_ch1.pulse_gpio_num = PIN_ENC1_B;
    pcnt_ch1.ctrl_gpio_num  = PIN_ENC1_A;
    pcnt_ch1.unit           = PCNT_UNIT_0;
    pcnt_ch1.channel        = PCNT_CHANNEL_1;
    pcnt_ch1.pos_mode       = PCNT_COUNT_DEC;
    pcnt_ch1.neg_mode       = PCNT_COUNT_INC;
    pcnt_ch1.lctrl_mode     = PCNT_MODE_REVERSE;
    pcnt_ch1.hctrl_mode     = PCNT_MODE_KEEP;
    pcnt_ch1.counter_h_lim  = 30000;
    pcnt_ch1.counter_l_lim  = -30000;
    pcnt_unit_config(&pcnt_ch1);

    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);

    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
    g_prev_pcnt_raw = 0;
}

static inline void latchEncoderAngleDeg(float known_theta_m_deg) {
    pcnt_counter_clear(PCNT_UNIT_0);
    g_prev_pcnt_raw   = 0;
    g_enc_counts      = (int64_t)llroundf(known_theta_m_deg / DEG_PER_COUNT);
    g_enc_boot_counts = g_enc_counts;
    g_theta_m_deg     = known_theta_m_deg;
}

// ============================================================================
// 7. NVS FLASH STORAGE (WITH SANITY GUARD AGAINST SATURATED ADC VALUES)
// ============================================================================
static void loadCalibrationFromNVS() {
    g_prefs.begin("tophub_cal", true);
    if (g_prefs.getBool("valid", false)) {
        g_cal.calibrated        = true;
        g_cal.enc_polarity      = g_prefs.getChar("enc_pol", +1);
        g_cal.dir_polarity      = g_prefs.getChar("dir_pol", +1);
        g_cal.acs_polarity      = g_prefs.getChar("acs_pol", +1);
        g_cal.v_adc_zero_mv     = g_prefs.getFloat("v_zero", 1250.0f);
        g_cal.v_bus_idle_v      = g_prefs.getFloat("v_bus", 24.00f);
        g_cal.backlash_deg      = g_prefs.getFloat("lash", 0.65f);
        g_cal.r_loop_ohm        = g_prefs.getFloat("r_loop", 0.40f);
        g_cal.v_brush_v         = g_prefs.getFloat("v_brush", 0.35f);
        g_cal.k_t_nm_per_a      = g_prefs.getFloat("k_t", 0.0694f);
        g_cal.i_no_load_a       = g_prefs.getFloat("i_0", 1.00f);
        g_cal.b_visc_a_per_rads = g_prefs.getFloat("b_visc", 0.0015f);
        g_cal.tau_fric_gbx_nm   = g_prefs.getFloat("t_fric", 2.50f);
        g_cal.tau_arm_grav_nm   = g_prefs.getFloat("t_arm", 3.50f);
        g_cal.tau_limb_grav_nm  = g_prefs.getFloat("t_limb", 8.50f);
        g_cal.eta_fwd           = g_prefs.getFloat("eta_f", 0.85f);
        g_cal.eta_back          = g_prefs.getFloat("eta_b", 0.85f);

        // Sanity guard: reject any previously saved saturated ADC offset (e.g. 3145 mV)
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
    g_prefs.begin("tophub_cal", false);
    g_prefs.putBool("valid", g_cal.calibrated);
    g_prefs.putChar("enc_pol", g_cal.enc_polarity);
    g_prefs.putChar("dir_pol", g_cal.dir_polarity);
    g_prefs.putChar("acs_pol", g_cal.acs_polarity);
    g_prefs.putFloat("v_zero", g_cal.v_adc_zero_mv);
    g_prefs.putFloat("v_bus", g_cal.v_bus_idle_v);
    g_prefs.putFloat("lash", g_cal.backlash_deg);
    g_prefs.putFloat("r_loop", g_cal.r_loop_ohm);
    g_prefs.putFloat("v_brush", g_cal.v_brush_v);
    g_prefs.putFloat("k_t", g_cal.k_t_nm_per_a);
    g_prefs.putFloat("i_0", g_cal.i_no_load_a);
    g_prefs.putFloat("b_visc", g_cal.b_visc_a_per_rads);
    g_prefs.putFloat("t_fric", g_cal.tau_fric_gbx_nm);
    g_prefs.putFloat("t_arm", g_cal.tau_arm_grav_nm);
    g_prefs.putFloat("t_limb", g_cal.tau_limb_grav_nm);
    g_prefs.putFloat("eta_f", g_cal.eta_fwd);
    g_prefs.putFloat("eta_b", g_cal.eta_back);
    g_prefs.end();
}

// ============================================================================
// 8. NON-COLLINEAR OFFSET-PIVOT KINEMATICS & PHYSIOLOGICAL TORQUE CURVES
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

static float computePhysiologicalTorqueTarget(
    float theta_k_deg,
    bool  is_extension,
    float peak_ext_nm,
    float peak_flex_nm,
    uint8_t pathology_profile
) {
    float tk = constrain(theta_k_deg, 0.0f, 125.0f);
    if (is_extension) {
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
    } else {
        float mult = 1.0f;
        if (tk <= 30.0f) {
            mult = 0.80f + 0.20f * (tk / 30.0f);
        } else {
            float u = (tk - 30.0f) / 90.0f;
            mult = 1.0f - 0.70f * u;
        }
        return constrain(peak_flex_nm * constrain(mult, 0.25f, 1.0f), 0.0f, 40.0f);
    }
}

// ============================================================================
// 9. SMART SENSOR SAMPLING & AUTO-BYPASS CURRENT/VOLTAGE CONTROLLER
// ============================================================================
static void sampleAnalogSensors500Hz() {
    // 1. Sample VBUS_SENSE on GPIO34 with hardware sanity check
    g_vbus_raw_adc = analogRead(PIN_VBUS_SENSE);
    g_vbus_raw_mv  = (float)analogReadMilliVolts(PIN_VBUS_SENSE);

    // Valid divider range for 14V..32V bus is 820 mV .. 1900 mV (raw ADC ~900..2600).
    // If GPIO34 is saturated (e.g., 3145 mV / raw 4095) or unplugged (<600 mV), auto-bypass to nominal 24.0V!
    if (g_vbus_raw_adc >= 100 && g_vbus_raw_adc <= 3500 &&
        g_vbus_raw_mv >= 600.0f && g_vbus_raw_mv <= 2300.0f) {
        g_vbus_valid = true;
        float vbus_meas = (g_vbus_raw_mv * 0.001f) * VBUS_DIV_MULT;
        g_vbus_volts = 0.85f * g_vbus_volts + 0.15f * vbus_meas;
    } else {
        g_vbus_valid = false;
        g_vbus_volts = 24.0f; // Safe fallback so saturated ADC never kills PWM or halves duty cycle
    }

    // 2. Sample ACS712_SENSE on GPIO35 with hardware sanity check
    g_acs_raw_adc = analogRead(PIN_ACS712_SENSE);
    g_acs_raw_mv  = (float)analogReadMilliVolts(PIN_ACS712_SENSE);

    // Valid ACS712 range is 300 mV .. 2850 mV (raw ADC ~300..3850).
    // If saturated at 3145 mV (raw >= 3900) or shorted (< 250 mV), auto-bypass to open-loop voltage drive!
    if (g_acs_raw_adc >= 250 && g_acs_raw_adc <= 3900 &&
        g_acs_raw_mv >= 300.0f && g_acs_raw_mv <= 2850.0f) {
        g_acs_valid = true;
        float i_raw = ((g_acs_raw_mv - g_cal.v_adc_zero_mv) / ACS712_MV_PER_AMP) * (float)g_cal.acs_polarity;
        g_motor_current_a = 0.70f * g_motor_current_a + 0.30f * i_raw;
    } else {
        g_acs_valid = false;
        // Estimate current from commanded feedforward when ACS712 pin is saturated/bypassed
        g_motor_current_a = g_i_ref_a;
    }
}

static void updateBrakeChopper(float vbus_v) {
    if (!g_vbus_valid) {
        g_chopper_duty_pct = 0.0f;
        writeChopperPWM(0);
        return;
    }

    float v_on  = g_cal.v_bus_idle_v + 0.50f; // ~24.50 V
    float v_max = g_cal.v_bus_idle_v + 2.50f; // ~26.50 V

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

// Closed-Loop Armature Current Regulator with Automatic Open-Loop Voltage Fallback
static float g_pi_integral_v = 0.0f;

static void executeCurrentControl(float i_ref_a, float omega_m_deg_s, bool freewheel_release) {
    if (freewheel_release) {
        g_pi_integral_v = 0.0f;
        stopMotorImmediate();
        return;
    }

    i_ref_a = constrain(i_ref_a, -MOTOR_MAX_CURRENT_A, MOTOR_MAX_CURRENT_A);
    g_i_ref_a = i_ref_a;

    if (fabsf(omega_m_deg_s) > 330.0f) {
        i_ref_a = 0.0f;
    }

    if (fabsf(i_ref_a) < 0.08f) {
        g_pi_integral_v = 0.0f;
        stopMotorImmediate();
        return;
    }

    float omega_motor_rads = (omega_m_deg_s * RAD_PER_DEG) * GEAR_RATIO;
    float e_bemf_v         = g_cal.k_t_nm_per_a * omega_motor_rads;

    // Feedforward motor terminal voltage V_ff = I_ref * R_loop + E_bemf + V_brush * sign(I_ref)
    float sign_i = (i_ref_a >= 0.0f) ? 1.0f : -1.0f;
    float v_ff   = (i_ref_a * g_cal.r_loop_ohm) + e_bemf_v + (g_cal.v_brush_v * sign_i);

    float v_cmd = v_ff;
    if (g_acs_valid) {
        const float KP_I = 0.25f;
        const float KI_I = 8.0f;
        float i_err = i_ref_a - g_motor_current_a;
        g_pi_integral_v = constrain(g_pi_integral_v + (KI_I * CONTROL_DT_SEC * i_err), -6.0f, 6.0f);
        v_cmd += (KP_I * i_err) + g_pi_integral_v;
    } else {
        g_pi_integral_v = 0.0f;
    }

    bool dir_positive = (v_cmd >= 0.0f);
    if (g_cal.dir_polarity < 0) dir_positive = !dir_positive;

    float vbus = (g_vbus_valid && g_vbus_volts >= 15.0f && g_vbus_volts <= 30.0f) ? g_vbus_volts : 24.0f;
    float duty_frac = constrain(fabsf(v_cmd) / vbus, 0.0f, 0.98f);
    setMotorDrive(dir_positive, (uint32_t)(duty_frac * PWM_MAX_DUTY));
}

// ============================================================================
// 10. AUTOMATED 6-STAGE SETUP & PARAMETER IDENTIFICATION STATE MACHINE
// ============================================================================
enum SetupStage : uint8_t {
    SETUP_STAGE_IDLE = 0,
    SETUP_STAGE_1_STATIC_ZERO,
    SETUP_STAGE_2_HOMING_AND_LASH,
    SETUP_STAGE_3_BLOCKED_ROTOR_R,
    SETUP_STAGE_4_UNLOADED_SWEEP,
    SETUP_STAGE_5_LIMB_GRAV_SCAN,
    SETUP_STAGE_6_SAVE_AND_PARK
};

static SetupStage g_setup_stage       = SETUP_STAGE_IDLE;
static uint32_t   g_setup_stage_ms    = 0;
static uint8_t    g_setup_substep     = 0;
static double     g_cal_acc1          = 0.0;
static double     g_cal_acc2          = 0.0;
static uint32_t   g_cal_samples       = 0;
static float      g_cal_i1 = 0.0f, g_cal_v1 = 0.0f;
static float      g_cal_i_ext_sum = 0.0f, g_cal_i_flex_sum = 0.0f;
static uint32_t   g_cal_sweep_cnt = 0;

static void startSetupRoutine(bool full_calibration) {
    g_setup_stage     = SETUP_STAGE_1_STATIC_ZERO;
    g_setup_stage_ms  = millis();
    g_setup_substep   = full_calibration ? 1 : 0; // 1 = Full 6-stage, 0 = Quick 2-stage
    g_cal_acc1        = 0.0;
    g_cal_acc2        = 0.0;
    g_cal_samples     = 0;
    g_spasm_latched   = false;
    g_hardstop_locked = false;
}

static void runSetupStateMachine() {
    uint32_t elapsed = millis() - g_setup_stage_ms;

    switch (g_setup_stage) {
        // --------------------------------------------------------------------
        // STAGE 1: Static Sensor Zero-Offset & Idle Bus Voltage (~1.0 s)
        // --------------------------------------------------------------------
        case SETUP_STAGE_1_STATIC_ZERO: {
            stopMotorImmediate();
            if (elapsed > 100 && elapsed <= 900) {
                g_cal_acc1 += (double)g_acs_raw_mv;
                g_cal_acc2 += (double)g_vbus_raw_mv * 0.001 * VBUS_DIV_MULT;
                g_cal_samples++;
            } else if (elapsed > 900) {
                if (g_cal_samples > 50) {
                    float avg_acs_mv = (float)(g_cal_acc1 / g_cal_samples);
                    float avg_vbus_v = (float)(g_cal_acc2 / g_cal_samples);

                    // Only accept ACS712 zero offset if ADC is NOT saturated
                    if (avg_acs_mv >= 400.0f && avg_acs_mv <= 2750.0f) {
                        g_cal.v_adc_zero_mv = avg_acs_mv;
                    } else {
                        g_cal.v_adc_zero_mv = 1250.0f;
                        Serial.printf("[SETUP] Note: GPIO35 read %.1f mV (saturated/unplugged) -> using nominal 1250mV Open-Loop.\n", avg_acs_mv);
                    }

                    if (avg_vbus_v >= 16.0f && avg_vbus_v <= 28.0f) {
                        g_cal.v_bus_idle_v = avg_vbus_v;
                    } else {
                        g_cal.v_bus_idle_v = 24.00f;
                        Serial.printf("[SETUP] Note: GPIO34 read %.1f mV (saturated/unplugged) -> using nominal 24.00V Vbus.\n", g_vbus_raw_mv);
                    }
                }
                Serial.printf("[SETUP] Stage 1 Done: ACS_Zero=%.1f mV | Vbus_idle=%.2f V -> Starting Stage 2 Homing (30%% PWM for 4s)...\n",
                              g_cal.v_adc_zero_mv, g_cal.v_bus_idle_v);
                g_setup_stage    = SETUP_STAGE_2_HOMING_AND_LASH;
                g_setup_stage_ms = millis();
                g_cal_samples    = 0;
            }
            break;
        }

        // --------------------------------------------------------------------
        // STAGE 2: Current-Limited Hard-Stop Homing & Backlash ID (~4.8 s)
        // --------------------------------------------------------------------
        case SETUP_STAGE_2_HOMING_AND_LASH: {
            if (elapsed < 4000) {
                // Creep toward hard stop (extension) at 30% duty cycle
                bool dir_ext = (g_cal.dir_polarity > 0) ? false : true;
                float i_abs  = g_acs_valid ? fabsf(g_motor_current_a) : 0.0f;
                float duty_home = (g_acs_valid && i_abs > 4.0f) ? 0.12f : 0.30f;
                setMotorDrive(dir_ext, (uint32_t)(duty_home * PWM_MAX_DUTY));

                if (g_enc_valid && elapsed == 250 && fabsf(g_omega_m_deg_s) > 1.0f && g_omega_m_deg_s > 0.0f) {
                    g_cal.enc_polarity = -g_cal.enc_polarity;
                }
                if (g_acs_valid && elapsed == 300 && i_abs > 0.5f && g_motor_current_a > 0.0f) {
                    g_cal.acs_polarity = -g_cal.acs_polarity;
                }

                // Only trigger early stall stop if BOTH encoder & ACS712 are valid and confirm stall
                if (g_enc_valid && g_acs_valid && elapsed > 600 &&
                    fabsf(g_omega_m_deg_s) < 0.35f && i_abs > 3.0f && i_abs < 22.0f) {
                    g_cal_samples++;
                    if (g_cal_samples >= 150) { // 300 ms at 500 Hz
                        Serial.printf("[SETUP] Stage 2 Hard-Stop Contact Detected (I_m=%.2f A)!\n", g_motor_current_a);
                        g_setup_stage_ms = millis() - 4000;
                    }
                } else {
                    g_cal_samples = 0;
                }
            } else if (elapsed < 4350) {
                stopMotorImmediate();
            } else if (elapsed < 4400) {
                float theta_stop_m = mapKneeToArmAngleDeg(0.0f, g_cmd.d_offset_m, g_cmd.L_leg_m);
                latchEncoderAngleDeg(theta_stop_m);
                g_is_homed = true;
            } else if (elapsed < 4750) {
                // Reverse micro-pulse (25% duty in opposite direction) for 350 ms
                bool dir_flex = (g_cal.dir_polarity > 0) ? true : false;
                setMotorDrive(dir_flex, (uint32_t)(0.25f * PWM_MAX_DUTY));
            } else {
                stopMotorImmediate();
                float theta_stop_m  = mapKneeToArmAngleDeg(0.0f, g_cmd.d_offset_m, g_cmd.L_leg_m);
                float measured_lash = fabsf(g_theta_m_deg - theta_stop_m);
                g_cal.backlash_deg  = g_enc_valid ? constrain(measured_lash, 0.15f, 0.833f) : 0.65f;

                if (g_setup_substep == 0) {
                    Serial.printf("[SETUP] Quick Homing Complete! Homed=1 | Backlash=%.2f deg. Returning to IDLE.\n", g_cal.backlash_deg);
                    g_setup_stage    = SETUP_STAGE_6_SAVE_AND_PARK;
                    g_setup_stage_ms = millis();
                } else {
                    Serial.println("[SETUP] Stage 2 Done -> Starting Stage 3 (Blocked-Rotor R_loop ID)...");
                    g_setup_stage    = SETUP_STAGE_3_BLOCKED_ROTOR_R;
                    g_setup_stage_ms = millis();
                    g_cal_acc1 = 0.0; g_cal_acc2 = 0.0; g_cal_samples = 0;
                }
            }
            break;
        }

        // --------------------------------------------------------------------
        // STAGE 3: Blocked-Rotor R_loop & V_brush Identification (~0.8 s)
        // --------------------------------------------------------------------
        case SETUP_STAGE_3_BLOCKED_ROTOR_R: {
            bool dir_ext = (g_cal.dir_polarity > 0) ? false : true;
            if (elapsed < 300) {
                setMotorDrive(dir_ext, (uint32_t)(0.10f * PWM_MAX_DUTY));
                if (elapsed > 150 && g_acs_valid) {
                    g_cal_acc1 += fabsf(g_motor_current_a);
                    g_cal_acc2 += (0.10f * g_vbus_volts);
                    g_cal_samples++;
                }
            } else if (elapsed >= 300 && elapsed < 310 && g_cal_samples > 0) {
                g_cal_i1 = (float)(g_cal_acc1 / g_cal_samples);
                g_cal_v1 = (float)(g_cal_acc2 / g_cal_samples);
                g_cal_acc1 = 0.0; g_cal_acc2 = 0.0; g_cal_samples = 0;
            } else if (elapsed < 600) {
                setMotorDrive(dir_ext, (uint32_t)(0.18f * PWM_MAX_DUTY));
                if (elapsed > 450 && g_acs_valid) {
                    g_cal_acc1 += fabsf(g_motor_current_a);
                    g_cal_acc2 += (0.18f * g_vbus_volts);
                    g_cal_samples++;
                }
            } else {
                stopMotorImmediate();
                if (g_acs_valid && g_cal_samples > 0) {
                    float i2 = (float)(g_cal_acc1 / g_cal_samples);
                    float v2 = (float)(g_cal_acc2 / g_cal_samples);
                    if ((i2 - g_cal_i1) > 0.8f) {
                        float r_est = (v2 - g_cal_v1) / (i2 - g_cal_i1);
                        g_cal.r_loop_ohm = constrain(r_est, 0.25f, 0.75f);
                        g_cal.v_brush_v  = constrain(g_cal_v1 - g_cal.r_loop_ohm * g_cal_i1, 0.10f, 1.20f);
                    }
                }
                Serial.printf("[SETUP] Stage 3 Done (R_loop=%.3f Ohm) -> Starting Stage 4 (Unloaded Sweep)...\n", g_cal.r_loop_ohm);
                g_setup_stage    = SETUP_STAGE_4_UNLOADED_SWEEP;
                g_setup_stage_ms = millis();
                g_cal_i_ext_sum  = 0.0f;
                g_cal_i_flex_sum = 0.0f;
                g_cal_sweep_cnt  = 0;
            }
            break;
        }

        // --------------------------------------------------------------------
        // STAGE 4: Unloaded Arm Sweep (K_t, I_0, Arm Gravity) (~6.0 s)
        // --------------------------------------------------------------------
        case SETUP_STAGE_4_UNLOADED_SWEEP: {
            if (g_enc_valid) {
                float target_tk = (elapsed < 3000) ? 75.0f : 25.0f;
                float err_deg   = target_tk - g_theta_k_deg;
                float i_cmd     = constrain(0.35f * err_deg - 0.03f * g_omega_k_deg_s, -4.5f, 4.5f);
                executeCurrentControl(i_cmd, g_omega_m_deg_s, false);

                if (g_acs_valid && g_theta_k_deg > 35.0f && g_theta_k_deg < 65.0f && fabsf(g_omega_k_deg_s) > 5.0f) {
                    if (elapsed < 3000) g_cal_i_flex_sum += fabsf(g_motor_current_a);
                    else                g_cal_i_ext_sum  += fabsf(g_motor_current_a);
                    g_cal_sweep_cnt++;
                }
            } else {
                // Open-loop gentle bidirectional sweep when encoder is unplugged
                bool dir_flex = (elapsed < 3000);
                if (g_cal.dir_polarity < 0) dir_flex = !dir_flex;
                setMotorDrive(dir_flex, (uint32_t)(0.25f * PWM_MAX_DUTY));
            }

            if (elapsed >= 6000) {
                if (g_acs_valid && g_cal_sweep_cnt > 50) {
                    float i_avg_fric = 0.5f * ((g_cal_i_ext_sum + g_cal_i_flex_sum) / (float)g_cal_sweep_cnt);
                    g_cal.i_no_load_a     = constrain(i_avg_fric, 0.50f, 2.20f);
                    g_cal.tau_fric_gbx_nm = constrain(GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a * g_cal.i_no_load_a, 1.5f, 4.5f);
                }
                Serial.println("[SETUP] Stage 4 Done -> Starting Stage 5 (Limb Gravity Scan)...");
                g_setup_stage    = SETUP_STAGE_5_LIMB_GRAV_SCAN;
                g_setup_stage_ms = millis();
                g_cal_acc1 = 0.0; g_cal_samples = 0;
            }
            break;
        }

        // --------------------------------------------------------------------
        // STAGE 5: Passive Limb Gravity Scan (~4.0 s)
        // --------------------------------------------------------------------
        case SETUP_STAGE_5_LIMB_GRAV_SCAN: {
            if (g_enc_valid) {
                float target_tk = 45.0f;
                float err_deg   = target_tk - g_theta_k_deg;
                float i_cmd     = constrain(0.45f * err_deg - 0.04f * g_omega_k_deg_s, -6.0f, 6.0f);
                executeCurrentControl(i_cmd, g_omega_m_deg_s, false);

                if (g_acs_valid && elapsed > 2000 && elapsed < 3800 && fabsf(g_omega_k_deg_s) < 2.0f) {
                    g_cal_acc1 += fabsf(g_motor_current_a);
                    g_cal_samples++;
                }
            } else {
                stopMotorImmediate();
            }

            if (elapsed >= 4000) {
                if (g_acs_valid && g_cal_samples > 50) {
                    float i_hold    = (float)(g_cal_acc1 / g_cal_samples);
                    float tau_total = GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a * fmaxf(0.0f, i_hold - g_cal.i_no_load_a);
                    float limb_est  = (tau_total / (0.7071f * g_gamma_factor)) - g_cal.tau_arm_grav_nm;
                    if (limb_est > 2.0f && limb_est < 25.0f) {
                        g_cal.tau_limb_grav_nm = limb_est;
                    }
                }
                g_cal.calibrated = true;
                saveCalibrationToNVS();
                Serial.println("[SETUP] Full 6-Stage Calibration Complete & Saved to NVS! Returning to IDLE.");
                g_setup_stage    = SETUP_STAGE_6_SAVE_AND_PARK;
                g_setup_stage_ms = millis();
            }
            break;
        }

        // --------------------------------------------------------------------
        // STAGE 6: Park Safely & Return to IDLE
        // --------------------------------------------------------------------
        case SETUP_STAGE_6_SAVE_AND_PARK:
        default: {
            stopMotorImmediate();
            g_setup_stage = SETUP_STAGE_IDLE;
            g_cmd.op_mode = OP_IDLE;
            break;
        }
    }
}

// ============================================================================
// 11. TRI-MODAL SPASM, CLONUS & HARD-STOP SAFETY SUPERVISOR (500 Hz)
// Returns true if supervisor took over motor output for this control tick.
// ============================================================================
static float    g_prev_pad_force_n     = 0.0f;
static uint32_t g_residual_spike_ticks = 0;
static int8_t   g_prev_vel_sign        = 0;
static uint32_t g_clonus_window_start  = 0;
static uint8_t  g_clonus_reversals     = 0;
static bool     g_in_spasm_backoff     = false;
static float    g_backoff_target_deg   = 60.0f;

static bool evaluateSafetySupervisor(uint8_t op_mode) {
    // 1. Never interfere during Direct PWM Test, Auto Bench Test, or Setup Routines
    if (op_mode == OP_DIRECT_PWM_TEST || op_mode == OP_AUTO_BENCH_TEST ||
        op_mode == OP_SETUP_FULL_25S  || op_mode == OP_SETUP_QUICK_6S) {
        g_prev_pad_force_n = g_pad_force_n;
        return false;
    }

    // 2. Hardware Bus Overvoltage Check (ONLY if Vbus ADC divider is valid!)
    if (g_vbus_valid && g_vbus_volts > VBUS_OVERVOLT_FAULT_V) {
        writeChopperPWM(PWM_MAX_DUTY);
        stopMotorImmediate();
        return true;
    }

    // 3. Check Master ESP-NOW / CLI Commanded Release
    if (g_cmd.estop_mode == ESTOP_HARD_STOP_RELEASE) {
        g_hardstop_locked  = false;
        g_in_spasm_backoff = false;
        g_target_curve_nm  = 0.0f;
        stopMotorImmediate();
        return true;
    }

    // 4. Check Commanded Hard-Stop Lock
    if (g_cmd.estop_mode == ESTOP_HARD_STOP_LOCKED && !g_hardstop_locked) {
        g_hardstop_locked    = true;
        g_freeze_theta_m_deg = g_theta_m_deg;
    }

    // 5. ESP-NOW Heartbeat Watchdog (500 ms timeout when controlled by U1: ESP_MASTER)
    if (g_cmd.from_espnow && op_mode >= OP_CPM && (millis() - g_cmd.last_cmd_ms > 500)) {
        if (!g_hardstop_locked) {
            g_hardstop_locked    = true;
            g_freeze_theta_m_deg = g_theta_m_deg;
        }
    }

    // 6. Evaluate Spasm Detectors only during active therapy modes when sensors are valid
    if (op_mode >= OP_CPM && op_mode <= OP_ISOMETRIC && !g_hardstop_locked && !g_in_spasm_backoff) {
        bool force_spike            = false;
        bool cpm_catch              = false;
        bool current_residual_fault = false;
        bool clonus_fault           = false;

        if (g_acs_valid) {
            float df_dt = (g_pad_force_n - g_prev_pad_force_n) * CONTROL_FREQ_HZ;
            g_prev_pad_force_n = g_pad_force_n;

            force_spike = (fabsf(df_dt) > 450.0f && fabsf(g_pad_force_n) > 35.0f);
            cpm_catch   = (op_mode == OP_CPM && fabsf(g_pad_force_n) > 55.0f);

            if (fabsf(g_motor_current_a - g_i_ref_a) > 4.5f) {
                g_residual_spike_ticks++;
            } else if (g_residual_spike_ticks > 0) {
                g_residual_spike_ticks--;
            }
            current_residual_fault = (g_residual_spike_ticks > 20); // >40 ms at 500 Hz
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

        if (force_spike || cpm_catch || current_residual_fault || clonus_fault) {
            g_spasm_latched        = true;
            g_residual_spike_ticks = 0;
            g_clonus_reversals     = 0;

            if (op_mode == OP_CPM && g_enc_valid) {
                g_in_spasm_backoff   = true;
                float backoff_dir    = (g_omega_m_deg_s >= 0.0f) ? -4.0f : +4.0f;
                g_backoff_target_deg = constrain(g_theta_m_deg + backoff_dir, 18.0f, 115.0f);
            } else {
                g_hardstop_locked    = true;
                g_freeze_theta_m_deg = g_theta_m_deg;
            }
        }
    }

    // 7. Execute Spasm Backoff or Electronic Hard-Stop Lock if active
    if (g_in_spasm_backoff) {
        float err_m = g_backoff_target_deg - g_theta_m_deg;
        if (!g_enc_valid || fabsf(err_m) < 0.4f || fabsf(g_pad_force_n) < 10.0f) {
            g_in_spasm_backoff   = false;
            g_hardstop_locked    = true;
            g_freeze_theta_m_deg = g_theta_m_deg;
        } else {
            float i_backoff = constrain(0.8f * err_m - 0.05f * g_omega_m_deg_s, -2.8f, 2.8f);
            executeCurrentControl(i_backoff, g_omega_m_deg_s, false);
        }
        return true;
    }

    if (g_hardstop_locked) {
        if (g_enc_valid) {
            float grav_comp_nm = -(g_cal.tau_arm_grav_nm * cosf(g_theta_m_deg * RAD_PER_DEG) +
                                   g_gamma_factor * g_cal.tau_limb_grav_nm * cosf(g_theta_k_deg * RAD_PER_DEG));
            float tau_pd_gbx   = 6.0f * (g_freeze_theta_m_deg - g_theta_m_deg) - 0.35f * g_omega_m_deg_s + grav_comp_nm;
            float max_lock_gbx = 22.5f * g_gamma_factor;
            tau_pd_gbx         = constrain(tau_pd_gbx, -max_lock_gbx, max_lock_gbx);
            float i_lock       = tau_pd_gbx / (GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a);
            executeCurrentControl(i_lock, g_omega_m_deg_s, false);
        } else {
            stopMotorImmediate(); // Dynamic H-bridge low-side brake when encoder is unplugged
        }
        return true;
    }

    return false;
}

// ============================================================================
// 12. THERAPY MODE CONTROLLERS & REPETITION TRACKER (500 Hz)
// ============================================================================
static float    g_cpm_phase_deg       = 60.0f;
static int8_t   g_cpm_dir             = +1;
static uint32_t g_cpm_pause_ticks     = 0;
static float    g_last_rev_theta_m    = 60.0f;
static int8_t   g_last_torque_sign    = 0;

static bool     g_rep_going_flex      = true;
static float    g_rep_min_deg         = 120.0f;
static float    g_rep_max_deg         = 0.0f;
static float    g_rep_pk_ext_nm       = 0.0f;
static float    g_rep_ang_pk_ext      = 60.0f;
static float    g_rep_pk_flex_nm      = 0.0f;
static float    g_rep_ang_pk_flex     = 30.0f;
static float    g_rep_work_j          = 0.0f;
static double   g_rep_vel_sum         = 0.0;
static double   g_rep_vel_sq_sum      = 0.0;
static uint32_t g_rep_samples         = 0;

static void updateRepTracker(float tk_deg, float omega_k_degs, float tau_knee_nm) {
    if (!g_enc_valid) return;

    if (tk_deg < g_rep_min_deg) g_rep_min_deg = tk_deg;
    if (tk_deg > g_rep_max_deg) g_rep_max_deg = tk_deg;

    if (omega_k_degs < -2.0f && fabsf(tau_knee_nm) > g_rep_pk_ext_nm) {
        g_rep_pk_ext_nm  = fabsf(tau_knee_nm);
        g_rep_ang_pk_ext = tk_deg;
    } else if (omega_k_degs > 2.0f && fabsf(tau_knee_nm) > g_rep_pk_flex_nm) {
        g_rep_pk_flex_nm  = fabsf(tau_knee_nm);
        g_rep_ang_pk_flex = tk_deg;
    }

    float power_w = fabsf(tau_knee_nm * (omega_k_degs * RAD_PER_DEG));
    g_rep_work_j += power_w * CONTROL_DT_SEC;

    float v_abs = fabsf(omega_k_degs);
    if (v_abs > 2.0f) {
        g_rep_vel_sum    += v_abs;
        g_rep_vel_sq_sum += (v_abs * v_abs);
        g_rep_samples++;
    }

    float span = g_cmd.rom_max_deg - g_cmd.rom_min_deg;
    if (g_rep_going_flex && tk_deg >= (g_cmd.rom_max_deg - 0.20f * span)) {
        g_rep_going_flex = false;
    } else if (!g_rep_going_flex && tk_deg <= (g_cmd.rom_min_deg + 0.20f * span)) {
        g_rep_going_flex = true;
        g_rep_count++;

        g_latest_rep_summary.pkt_type            = PKT_TOPHUB_REP_SUMMARY;
        g_latest_rep_summary.rep_index           = g_rep_count;
        g_latest_rep_summary.achieved_rom_min    = g_rep_min_deg;
        g_latest_rep_summary.achieved_rom_max    = g_rep_max_deg;
        g_latest_rep_summary.peak_torque_ext_nm  = g_rep_pk_ext_nm;
        g_latest_rep_summary.angle_at_peak_ext   = g_rep_ang_pk_ext;
        g_latest_rep_summary.peak_torque_flex_nm = g_rep_pk_flex_nm;
        g_latest_rep_summary.angle_at_peak_flex  = g_rep_ang_pk_flex;
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
        g_rep_pk_ext_nm = 0.0f; g_rep_pk_flex_nm = 0.0f;
        g_rep_work_j = 0.0f; g_rep_vel_sum = 0.0; g_rep_vel_sq_sum = 0.0; g_rep_samples = 0;
    }
}

static inline float computeAntiBacklashFactor(float theta_m_deg, float tau_cmd) {
    int8_t sign_t = (tau_cmd >= 0.0f) ? +1 : -1;
    if (sign_t != g_last_torque_sign && fabsf(tau_cmd) > 0.5f) {
        g_last_rev_theta_m = theta_m_deg;
        g_last_torque_sign = sign_t;
    }
    float d_theta = fabsf(theta_m_deg - g_last_rev_theta_m);
    float lash    = fmaxf(0.20f, g_cal.backlash_deg);
    float ratio   = d_theta / lash;
    return 1.0f - 0.70f * expf(-(ratio * ratio));
}

// ============================================================================
// 13. UNIFIED 500 Hz REAL-TIME CONTROL STEP (COOPERATIVE, ZERO STARVATION)
// ============================================================================
static void runControlStep500Hz() {
    // ------------------------------------------------------------------------
    // STEP 0: Check for Pending ESP-NOW Command Packet
    // ------------------------------------------------------------------------
    if (g_rx_pkt_pending) {
        RehabCommandPacket_t pkt;
        portENTER_CRITICAL(&g_rx_mux);
        pkt = g_rx_staging_pkt;
        g_rx_pkt_pending = false;
        portEXIT_CRITICAL(&g_rx_mux);

        g_cmd.op_mode            = pkt.op_mode;
        g_cmd.pathology_profile  = pkt.pathology_profile;
        g_cmd.estop_mode         = pkt.estop_active;
        g_cmd.d_offset_m         = constrain(pkt.d_offset_m, 0.05f, 0.15f);
        g_cmd.L_leg_m            = constrain(pkt.L_leg_m, 0.20f, 0.40f);
        g_cmd.rom_min_deg        = constrain(pkt.rom_min_deg, 0.0f, 90.0f);
        g_cmd.rom_max_deg        = constrain(pkt.rom_max_deg, 15.0f, 125.0f);
        g_cmd.target_vel_deg_s   = pkt.target_vel_deg_s;
        g_cmd.target_torque_ext  = constrain(pkt.target_torque_ext, 0.0f, 68.0f);
        g_cmd.target_torque_flex = constrain(pkt.target_torque_flex, 0.0f, 40.0f);
        g_cmd.last_cmd_ms        = millis();
        g_cmd.from_espnow        = true;
        if (pkt.estop_active == ESTOP_NORMAL) {
            g_hardstop_locked = false;
            g_spasm_latched   = false;
        }
    }

    // ------------------------------------------------------------------------
    // STEP 1: Read Hardware Quadrature Encoder (PCNT_UNIT_0)
    // ------------------------------------------------------------------------
    int16_t raw_cnt = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw_cnt);
    int16_t delta_cnt = raw_cnt - g_prev_pcnt_raw;
    g_prev_pcnt_raw   = raw_cnt;
    if (abs(raw_cnt) > 20000) {
        pcnt_counter_clear(PCNT_UNIT_0);
        g_prev_pcnt_raw = 0;
    }
    g_enc_counts += (int64_t)delta_cnt * g_cal.enc_polarity;
    if (llabs(g_enc_counts - g_enc_boot_counts) > 8) {
        g_enc_valid = true;
    }

    float prev_theta_m = g_theta_m_deg;
    g_theta_m_deg      = (float)g_enc_counts * DEG_PER_COUNT;
    float raw_omega_m  = (g_theta_m_deg - prev_theta_m) * CONTROL_FREQ_HZ;
    g_omega_m_deg_s    = 0.75f * g_omega_m_deg_s + 0.25f * raw_omega_m;

    // ------------------------------------------------------------------------
    // STEP 2: Offset-Pivot Kinematic Transformation (theta_m -> theta_k)
    // ------------------------------------------------------------------------
    float d_off     = g_cmd.d_offset_m;
    float l_leg     = fmaxf(0.20f, g_cmd.L_leg_m);
    float prev_tk   = g_theta_k_deg;
    g_theta_k_deg   = mapArmToKneeAngleDeg(g_theta_m_deg, d_off, l_leg);
    float raw_wk    = (g_theta_k_deg - prev_tk) * CONTROL_FREQ_HZ;
    g_omega_k_deg_s = 0.75f * g_omega_k_deg_s + 0.25f * raw_wk;
    g_gamma_factor  = computeGammaFactor(g_theta_k_deg, d_off, l_leg);

    // ------------------------------------------------------------------------
    // STEP 3: Sample Analog Sensors (with Sanity Guard) & Update Brake Chopper
    // ------------------------------------------------------------------------
    sampleAnalogSensors500Hz();
    updateBrakeChopper(g_vbus_volts);

    float tau_em_gbx = GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a * g_motor_current_a;
    g_gbx_torque_nm  = constrain(tau_em_gbx, -GBX_MAX_TORQUE_NM, GBX_MAX_TORQUE_NM);
    g_knee_torque_nm = g_gbx_torque_nm / g_gamma_factor;
    g_pad_force_n    = g_knee_torque_nm / l_leg;

    // ------------------------------------------------------------------------
    // STEP 4: Evaluate Safety Supervisor (Single Authority Return)
    // ------------------------------------------------------------------------
    if (evaluateSafetySupervisor(g_cmd.op_mode)) {
        return; // Supervisor handled motor output for this tick; do not overwrite!
    }

    // ------------------------------------------------------------------------
    // STEP 5: Execute Selected Operating / Therapy Mode
    // ------------------------------------------------------------------------
    if (g_cmd.op_mode == OP_SETUP_FULL_25S || g_cmd.op_mode == OP_SETUP_QUICK_6S) {
        if (g_setup_stage == SETUP_STAGE_IDLE) {
            startSetupRoutine(g_cmd.op_mode == OP_SETUP_FULL_25S);
        }
        runSetupStateMachine();
        return;
    } else {
        g_setup_stage = SETUP_STAGE_IDLE;
    }

    float tau_grav_gbx = -(g_cal.tau_arm_grav_nm * cosf(g_theta_m_deg * RAD_PER_DEG) +
                           g_gamma_factor * g_cal.tau_limb_grav_nm * cosf(g_theta_k_deg * RAD_PER_DEG));
    float i_ff_fric    = (g_cal.i_no_load_a + g_cal.b_visc_a_per_rads * fabsf(g_omega_m_deg_s)) *
                         tanhf(g_omega_m_deg_s / 1.5f);

    switch (g_cmd.op_mode) {
        case OP_IDLE:
        default: {
            g_target_curve_nm = 0.0f;
            stopMotorImmediate();
            break;
        }

        case OP_CPM: {
            float cpm_speed = constrain(g_cmd.target_vel_deg_s, 0.8f, 6.0f);
            if (g_cpm_pause_ticks > 0) {
                g_cpm_pause_ticks--;
            } else {
                g_cpm_phase_deg += (float)g_cpm_dir * (cpm_speed * CONTROL_DT_SEC);
                if (g_cpm_phase_deg >= g_cmd.rom_max_deg) {
                    g_cpm_phase_deg   = g_cmd.rom_max_deg;
                    g_cpm_dir         = -1;
                    g_cpm_pause_ticks = 1000; // 2.0s pause at 500 Hz
                } else if (g_cpm_phase_deg <= g_cmd.rom_min_deg) {
                    g_cpm_phase_deg   = g_cmd.rom_min_deg;
                    g_cpm_dir         = +1;
                    g_cpm_pause_ticks = 1000;
                }
            }

            if (g_enc_valid) {
                float target_tm = mapKneeToArmAngleDeg(g_cpm_phase_deg, d_off, l_leg);
                float pos_err   = target_tm - g_theta_m_deg;
                float tau_cmd   = 4.5f * pos_err - 0.25f * g_omega_m_deg_s + tau_grav_gbx;
                g_target_curve_nm = tau_cmd / g_gamma_factor;

                float i_cmd = (tau_cmd / (GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a)) + i_ff_fric;
                i_cmd       = constrain(i_cmd, -10.0f, 10.0f);
                executeCurrentControl(i_cmd, g_omega_m_deg_s, false);
                updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            } else {
                // Open-loop CPM fallback when encoder is unplugged
                if (g_cpm_pause_ticks > 0) {
                    stopMotorImmediate();
                } else {
                    bool dir_high = (g_cpm_dir > 0);
                    if (g_cal.dir_polarity < 0) dir_high = !dir_high;
                    setMotorDrive(dir_high, (uint32_t)(0.25f * PWM_MAX_DUTY));
                }
            }
            break;
        }

        case OP_ISOTONIC: {
            bool is_ext = (g_omega_k_deg_s <= 0.0f);
            float tau_knee_ref = computePhysiologicalTorqueTarget(
                g_theta_k_deg, is_ext, g_cmd.target_torque_ext, g_cmd.target_torque_flex, g_cmd.pathology_profile
            );
            g_target_curve_nm = tau_knee_ref;

            float tau_gbx_req = constrain(g_gamma_factor * tau_knee_ref, 0.0f, 87.4f);
            float move_dir    = (g_omega_m_deg_s > 1.5f) ? +1.0f : ((g_omega_m_deg_s < -1.5f) ? -1.0f : 0.0f);
            float c_lash      = computeAntiBacklashFactor(g_theta_m_deg, -move_dir * tau_gbx_req);

            float i_mag = fmaxf(0.0f, (c_lash * tau_gbx_req - g_cal.tau_fric_gbx_nm) / 4.246f);
            float i_cmd = -move_dir * i_mag;

            if (g_enc_valid) {
                if (g_theta_k_deg < g_cmd.rom_min_deg) {
                    i_cmd = +3.5f * (g_cmd.rom_min_deg - g_theta_k_deg);
                } else if (g_theta_k_deg > g_cmd.rom_max_deg) {
                    i_cmd = -3.5f * (g_theta_k_deg - g_cmd.rom_max_deg);
                }
            }

            executeCurrentControl(i_cmd, g_omega_m_deg_s, false);
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            break;
        }

        case OP_ISOKINETIC: {
            float omega_iso = constrain(g_cmd.target_vel_deg_s, 15.0f, 120.0f);
            bool  is_ext    = (g_omega_k_deg_s <= 0.0f);
            float tau_cap_k = computePhysiologicalTorqueTarget(
                g_theta_k_deg, is_ext, g_cmd.target_torque_ext, g_cmd.target_torque_flex, g_cmd.pathology_profile
            );
            g_target_curve_nm = tau_cap_k;

            float abs_wk = fabsf(g_omega_k_deg_s);
            float i_cmd  = 0.0f;
            if (abs_wk > (0.85f * omega_iso)) {
                float vel_excess = abs_wk - (0.85f * omega_iso);
                float tau_iso_k  = constrain(3.2f * vel_excess, 0.0f, tau_cap_k);
                float tau_gbx    = constrain(g_gamma_factor * tau_iso_k, 0.0f, 87.4f);
                float i_mag      = fmaxf(0.0f, (tau_gbx - g_cal.tau_fric_gbx_nm) / 4.246f);
                float move_dir   = (g_omega_m_deg_s >= 0.0f) ? +1.0f : -1.0f;
                i_cmd            = -move_dir * i_mag;
            } else {
                i_cmd = 0.4f * i_ff_fric;
            }

            executeCurrentControl(i_cmd, g_omega_m_deg_s, false);
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            break;
        }

        case OP_ISOMETRIC: {
            float lock_tk   = constrain(g_cmd.rom_max_deg, 15.0f, 105.0f);
            float lock_tm   = mapKneeToArmAngleDeg(lock_tk, d_off, l_leg);
            float max_tau_k = constrain(g_cmd.target_torque_ext, 10.0f, 68.0f);
            g_target_curve_nm = max_tau_k;

            if (g_enc_valid) {
                float tau_pd_gbx = 8.0f * (lock_tm - g_theta_m_deg) - 0.40f * g_omega_m_deg_s + tau_grav_gbx;
                float max_gbx    = constrain(g_gamma_factor * max_tau_k, 10.0f, 87.4f);
                tau_pd_gbx       = constrain(tau_pd_gbx, -max_gbx, max_gbx);

                float i_cmd = tau_pd_gbx / (GEAR_RATIO * g_cal.eta_fwd * g_cal.k_t_nm_per_a);
                executeCurrentControl(i_cmd, g_omega_m_deg_s, false);
            } else {
                stopMotorImmediate();
            }
            break;
        }

        // --------------------------------------------------------------------
        // MODE 9: DIRECT OPEN-LOOP PWM BENCH TEST ("pwm 40" / "pwm -40")
        // --------------------------------------------------------------------
        case OP_DIRECT_PWM_TEST: {
            float pct = constrain(g_cmd.target_vel_deg_s, -100.0f, 100.0f);
            bool dir_high = (pct >= 0.0f);
            uint32_t duty = (uint32_t)lroundf((fabsf(pct) * 0.01f) * PWM_MAX_DUTY);
            setMotorDrive(dir_high, duty);
            break;
        }

        // --------------------------------------------------------------------
        // MODE 10: AUTOMATIC BIDIRECTIONAL BENCH TEST ("test")
        // +35% Forward (1.5s) -> Pause (0.5s) -> -35% Reverse (1.5s) -> Stop
        // --------------------------------------------------------------------
        case OP_AUTO_BENCH_TEST: {
            uint32_t elapsed = millis() - g_bench_test_start_ms;
            if (elapsed < 1500) {
                setMotorDrive(true, (uint32_t)(0.35f * PWM_MAX_DUTY));
            } else if (elapsed < 2000) {
                stopMotorImmediate();
            } else if (elapsed < 3500) {
                setMotorDrive(false, (uint32_t)(0.35f * PWM_MAX_DUTY));
            } else {
                stopMotorImmediate();
                g_cmd.op_mode = OP_IDLE;
                Serial.println("[TEST] Bidirectional 3.5s Motor Test Complete -> Returned to IDLE.");
            }
            break;
        }
    }
}

// ============================================================================
// 14. ESP-NOW WIRELESS CALLBACKS & 50 Hz TELEMETRY STREAM
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
// 15. NON-BLOCKING USB SERIAL CLI (ZERO CRITICAL-SECTION UART CORRUPTION)
// ============================================================================
static char     s_cli_buf[64];
static uint8_t  s_cli_len        = 0;
static uint32_t s_cli_last_rx_ms = 0;

static void executeCLICommand(String line) {
    line.trim();
    if (line.length() == 0) return;

    g_cmd.from_espnow = false;
    g_cmd.last_cmd_ms = millis();

    if (line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("lock")) {
        g_cmd.estop_mode     = ESTOP_HARD_STOP_LOCKED;
        g_hardstop_locked    = true;
        g_freeze_theta_m_deg = g_theta_m_deg;
        Serial.println("[CLI] HARD STOP LOCKED engaged.");
    } else if (line.equalsIgnoreCase("release") || line.equalsIgnoreCase("idle")) {
        g_cmd.op_mode     = OP_IDLE;
        g_cmd.estop_mode  = ESTOP_HARD_STOP_RELEASE;
        g_hardstop_locked = false;
        g_spasm_latched   = false;
        stopMotorImmediate();
        Serial.println("[CLI] Arm RELEASED (PWM = 0%, Freewheel IDLE).");
    } else if (line.equalsIgnoreCase("test")) {
        g_cmd.estop_mode      = ESTOP_NORMAL;
        g_hardstop_locked     = false;
        g_spasm_latched       = false;
        g_bench_test_start_ms = millis();
        g_cmd.op_mode         = OP_AUTO_BENCH_TEST;
        Serial.println("[CLI] Starting Automatic Bidirectional Motor Test: +35% (1.5s) -> Pause (0.5s) -> -35% (1.5s) -> Stop");
    } else if (line.equalsIgnoreCase("adc")) {
        Serial.printf(
            "[ADC] GPIO34 (VBUS): raw=%4u, %6.1f mV (%s -> Vbus=%.2f V)\n"
            "      GPIO35 (ACS) : raw=%4u, %6.1f mV (%s -> I_m =%.2f A, V_zero=%.1f mV)\n"
            "      GPIO32/33 ENC: counts=%lld (%s -> Knee=%.1f deg)\n"
            "      Motor Output : PWM=%.1f%% (Duty=%ld/1023 at %lu Hz on GPIO25, DIR GPIO26=%s)\n",
            g_vbus_raw_adc, g_vbus_raw_mv, g_vbus_valid ? "VALID" : "BYPASSED_NOM_24V", g_vbus_volts,
            g_acs_raw_adc,  g_acs_raw_mv,  g_acs_valid  ? "VALID" : "BYPASSED_OPEN_LOOP", g_motor_current_a, g_cal.v_adc_zero_mv,
            (long long)g_enc_counts, g_enc_valid ? "CONNECTED" : "UNPLUGGED", g_theta_k_deg,
            g_active_pwm_pct, (long)s_last_motor_duty, (unsigned long)g_md_pwm_freq_hz,
            (s_last_motor_dir > 0) ? "HIGH" : "LOW"
        );
    } else if (line.equalsIgnoreCase("cal")) {
        g_cmd.estop_mode  = ESTOP_NORMAL;
        g_hardstop_locked = false;
        g_spasm_latched   = false;
        g_setup_stage     = SETUP_STAGE_IDLE;
        g_cmd.op_mode     = OP_SETUP_FULL_25S;
        Serial.println("[CLI] Starting Full 6-Stage Setup & Parameter ID...");
    } else if (line.equalsIgnoreCase("home")) {
        g_cmd.estop_mode  = ESTOP_NORMAL;
        g_hardstop_locked = false;
        g_spasm_latched   = false;
        g_setup_stage     = SETUP_STAGE_IDLE;
        g_cmd.op_mode     = OP_SETUP_QUICK_6S;
        Serial.println("[CLI] Starting Quick 6s Homing...");
    } else if (line.equalsIgnoreCase("cpm")) {
        g_cmd.estop_mode       = ESTOP_NORMAL;
        g_hardstop_locked      = false;
        g_spasm_latched        = false;
        g_cmd.op_mode          = OP_CPM;
        g_cmd.target_vel_deg_s = 2.0f;
        Serial.printf("[CLI] Mode 1: Passive CPM started (%s).\n",
                      g_enc_valid ? "Closed-Loop Encoder" : "Open-Loop Sweep [ENC1 Unplugged]");
    } else if (line.equalsIgnoreCase("iso")) {
        g_cmd.estop_mode         = ESTOP_NORMAL;
        g_hardstop_locked        = false;
        g_spasm_latched          = false;
        g_cmd.op_mode            = OP_ISOTONIC;
        g_cmd.target_torque_ext  = 35.0f;
        g_cmd.target_torque_flex = 20.0f;
        Serial.println("[CLI] Mode 2: Physiological Isotonic (35/20 N*m) started.");
    } else if (line.equalsIgnoreCase("isok")) {
        g_cmd.estop_mode         = ESTOP_NORMAL;
        g_hardstop_locked        = false;
        g_spasm_latched          = false;
        g_cmd.op_mode            = OP_ISOKINETIC;
        g_cmd.target_vel_deg_s   = 60.0f;
        g_cmd.target_torque_ext  = 45.0f;
        g_cmd.target_torque_flex = 25.0f;
        Serial.println("[CLI] Mode 3: Accommodating Isokinetic (60 deg/s, 45 N*m cap) started.");
    } else if (line.equalsIgnoreCase("isom")) {
        g_cmd.estop_mode        = ESTOP_NORMAL;
        g_hardstop_locked       = false;
        g_spasm_latched         = false;
        g_cmd.op_mode           = OP_ISOMETRIC;
        g_cmd.rom_max_deg       = 60.0f;
        g_cmd.target_torque_ext = 30.0f;
        Serial.println("[CLI] Mode 4: Isometric Hold at 60 deg (30 N*m yield) started.");
    } else if (line.startsWith("pwm") || line.startsWith("PWM")) {
        float pct = line.substring(3).toFloat();
        pct = constrain(pct, -100.0f, 100.0f);
        if (fabsf(pct) < 0.5f) {
            g_cmd.op_mode     = OP_IDLE;
            g_cmd.estop_mode  = ESTOP_HARD_STOP_RELEASE;
            g_hardstop_locked = false;
            stopMotorImmediate();
            Serial.println("[CLI] Direct PWM set to 0.0% (Motor OFF / IDLE).");
        } else {
            g_cmd.estop_mode       = ESTOP_NORMAL;
            g_hardstop_locked      = false;
            g_spasm_latched        = false;
            g_cmd.op_mode          = OP_DIRECT_PWM_TEST;
            g_cmd.target_vel_deg_s = pct;
            uint32_t duty          = (uint32_t)lroundf((fabsf(pct) * 0.01f) * PWM_MAX_DUTY);
            bool dir_high          = (pct >= 0.0f);
            setMotorDrive(dir_high, duty);
            Serial.printf("[CLI] Direct PWM Active: %.1f%% (Duty=%lu/1023, ~%.2fV avg on GPIO%d, DIR GPIO%d=%s)\n",
                          pct, (unsigned long)duty, 3.30f * (fabsf(pct) * 0.01f),
                          PIN_MD_PWM, PIN_MD_DIR, dir_high ? "HIGH" : "LOW");
        }
    } else if (line.startsWith("freq") || line.startsWith("FREQ")) {
        uint32_t new_freq = (uint32_t)line.substring(4).toInt();
        if (new_freq >= 100 && new_freq <= 25000) {
            g_md_pwm_freq_hz  = new_freq;
            int32_t prev_duty = s_last_motor_duty;
            initPWMHardware();
            if (prev_duty > 0) {
                writeMotorPWMRaw((uint32_t)prev_duty);
                s_last_motor_duty = prev_duty;
            }
            Serial.printf("[CLI] Motor PWM Frequency updated to %lu Hz.\n", (unsigned long)g_md_pwm_freq_hz);
        } else {
            Serial.println("[CLI] Usage: freq <100..25000> (e.g. freq 1000 or freq 4000)");
        }
    } else if (line.equalsIgnoreCase("status")) {
        Serial.printf(
            "[STATUS] Mode:%u | PWM:%.1f%% (DIR:%s) | Knee:%.1f deg (%.1f deg/s) | I_m:%.2f A (%s) | Vbus:%.2f V (%s) | ENC:%s\n",
            g_cmd.op_mode, g_active_pwm_pct, (s_last_motor_dir > 0) ? "HIGH" : "LOW",
            g_theta_k_deg, g_omega_k_deg_s,
            g_motor_current_a, g_acs_valid ? "ADC_OK" : "OPEN_LOOP",
            g_vbus_volts, g_vbus_valid ? "ADC_OK" : "NOM_24V",
            g_enc_valid ? "OK" : "UNPLUGGED"
        );
    } else {
        Serial.println("[CLI] Unknown command. Use: pwm <pct> | test | adc | status | home | cal | cpm | iso | isok | isom | freq <hz> | stop | release");
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
    // Also flush command if user sent text without newline in Serial Monitor (>150 ms idle)
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
    // 1. Fail-Safe Boot Pin Initialization FIRST (Guarantees 0% PWM on U201 & U202)
    pinMode(PIN_MD_PWM, OUTPUT);
    digitalWrite(PIN_MD_PWM, LOW);
    pinMode(PIN_MD_DIR, OUTPUT);
    digitalWrite(PIN_MD_DIR, LOW);
    pinMode(PIN_CHOP_PWM, OUTPUT);
    digitalWrite(PIN_CHOP_PWM, LOW);

    Serial.begin(115200);
    delay(100);
    Serial.println("\n=======================================================");
    Serial.println(" PROJECT DOULA — U2: ESP_TOPHUB CONTROLLER BOOTING");
    Serial.println("=======================================================");

    // 2. Configure ADC1 (12-bit, 11dB attenuation)
    pinMode(PIN_VBUS_SENSE, INPUT);
    pinMode(PIN_ACS712_SENSE, INPUT);
    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBUS_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_ACS712_SENSE, ADC_11db);
    if (USE_OPTIONAL_LOADCELL) {
        pinMode(PIN_LOADCELL_ADC, INPUT);
        analogSetPinAttenuation(PIN_LOADCELL_ADC, ADC_11db);
    }

    // 3. Initialize PWM (1 kHz Motor on GPIO25, 5 kHz Chopper on GPIO27) & PCNT Encoder
    initPWMHardware();
    initEncoderPCNT();
    latchEncoderAngleDeg(mapKneeToArmAngleDeg(60.0f, g_cmd.d_offset_m, g_cmd.L_leg_m));

    // 4. Load Saved Calibration Factors from NVS Flash (with saturation sanity check)
    loadCalibrationFromNVS();
    sampleAnalogSensors500Hz();
    Serial.printf("[NVS] Calibrated:%d | V_zero:%.1f mV | R_loop:%.3f Ohm | K_t:%.4f | Lash:%.2f deg\n",
                  g_cal.calibrated, g_cal.v_adc_zero_mv, g_cal.r_loop_ohm, g_cal.k_t_nm_per_a, g_cal.backlash_deg);
    Serial.printf("[ADC] Boot Check -> GPIO34(VBUS): %.1f mV (%s) | GPIO35(ACS): %.1f mV (%s)\n",
                  g_vbus_raw_mv, g_vbus_valid ? "VALID" : "BYPASSED_NOM_24V",
                  g_acs_raw_mv,  g_acs_valid  ? "VALID" : "BYPASSED_OPEN_LOOP");

    // 5. Initialize ESP-NOW on Wi-Fi Channel 1
    initEspNowChannel1();
    Serial.print("[WIFI] U2: ESP_TOPHUB MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.println("[CLI] Ready! Commands: pwm <pct> | test | adc | status | home | cal | cpm | iso | isok | isom | freq <hz> | stop | release");
}

void loop() {
    static uint32_t last_ctrl_us  = 0;
    static uint32_t last_telem_ms = 0;

    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_ctrl_us) >= CONTROL_PERIOD_US) {
        // Prevent catch-up burst if a print ever took > 2 ms
        if ((uint32_t)(now_us - last_ctrl_us) > (CONTROL_PERIOD_US * 4)) {
            last_ctrl_us = now_us;
        } else {
            last_ctrl_us += CONTROL_PERIOD_US;
        }
        runControlStep500Hz();
    }

    uint32_t now_ms = millis();
    if ((uint32_t)(now_ms - last_telem_ms) >= 20) { // 50 Hz ESP-NOW Telemetry
        last_telem_ms = now_ms;
        sendTelemetry50Hz();
    }

    pollSerialCLI();
}
