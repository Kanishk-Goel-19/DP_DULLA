// ============================================================================
// PROJECT DOULA — U2 (ESP_TOPHUB) INCREMENTAL STAGED FIRMWARE
// Current Stage : STAGE 6 (Cytron MDD10A) — Self-Homing, CPM & Active Assist
// Target MCU    : ESP32-S NodeMCU V1.1 (Arduino-ESP32 v2.x / v3.x)
// Reference     : Built from esp_tophub_circuit_test.ino (kept untouched)
//
// Cytron MDD10A (Rev 2.0) Specs & Stage 6 Architecture:
//   - Single channel (M1) on GPIO25 (PWM) + GPIO26 (DIR), Sign-Magnitude mode.
//   - 3.3V-compatible logic (V_IOH >= 3.0V), 5-30V bus, 5 kHz default PWM (20 kHz max).
//   - 10 A continuous / 30 A peak (10s) rating: firmware enforces 10.0 A steady
//     limit with up to 15.0 A surge budget for <= 5.0 s before foldback, plus
//     25.0 A (500 ms) sustained overcurrent shutdown.
//   - Bidirectional closed-loop PI+FF current control ('iref <-15..15>'),
//     bidirectional 'dirtest', and NVS-persisted 'pol dir|enc|acs' & 'rom'.
//   - Stage 6 adds current/stall-guarded 'autohome' (STOP1), 'goto <deg>',
//     bilateral 'cpm', patient-triggered + yield-forward 'assist' (AAN), and
//     per-rep [REP] torque/ROM/work telemetry.
//   - Brake chopper defaults to CHOP_OFF (pending 5V NPN U202 driver) with
//     27.5 V overvoltage cutoff protection.
//
// Serial Monitor: 115200 baud (NL, CR, Both NL & CR, or No line ending)
// ============================================================================

#include <Arduino.h>
#include <Preferences.h>
#include <ctype.h>
#include <math.h>
#include <driver/pcnt.h>

// ----------------------------------------------------------------------------
// 0. STAGE & HARDWARE CAPABILITY FLAGS
// ----------------------------------------------------------------------------
static const int  FW_STAGE                    = 6;
static const bool MOTOR_BIDIRECTIONAL_ENABLED = true;  // Cytron MDD10A Sign-Magnitude FWD + REV

// ----------------------------------------------------------------------------
// 1. HARDWARE PINS (Matches U2 ESP_TOPHUB Wiring Guide)
// ----------------------------------------------------------------------------
static const int PIN_MD_PWM       = 25; // MDD10A M1 PWM (20 kHz, R15=220k pull-down)
static const int PIN_MD_DIR       = 26; // MDD10A M1 DIR (Sign-Magnitude bidirectional)
static const int PIN_CHOP_PWM     = 27; // U202 30A Brake Chopper MOSFET (R16=22k pull-down)
static const int PIN_ENC1_A       = 32; // ENC1 Out A (R3=80 Ohm + INPUT_PULLUP)
static const int PIN_ENC1_B       = 33; // ENC1 Out B (R5=80 Ohm + INPUT_PULLUP)
static const int PIN_VBUS_SENSE   = 34; // ADC1_CH6: R11/R12 Bus Voltage Divider
static const int PIN_ACS712_SENSE = 35; // ADC1_CH7: R13/R14 ACS712-30A Divider

// ----------------------------------------------------------------------------
// 2. TIMING, PWM & PHYSICAL CONSTANTS
// ----------------------------------------------------------------------------
static const uint32_t CONTROL_PERIOD_US  = 2000;   // 500 Hz deterministic tick
static const float    CONTROL_DT_SEC     = 0.002f;
static const uint32_t TELEM_PERIOD_MS    = 500;    // 2 Hz Serial live telemetry

static const int      MAX_MOTOR_PWM_PCT    = 100;    // Safety cap for motor PWM (0..100%)
static const int      MAX_CHOP_MANUAL_PCT  = 50;     // Safety cap for manual chopper bench cmds
static const int      MAX_CHOP_AUTO_PCT    = 100;    // Full auto-clamp range on overvoltage

static const float    VBUS_CHOP_DEFAULT_ON_V = 25.50f;
static const float    VBUS_CHOP_RAMP_SPAN_V  = 2.00f;
static const float    VBUS_CHOP_ABS_MAX_V    = 28.50f;

static uint32_t       g_md_pwm_freq_hz      = 5000;   // 5 kHz proven Stage 1-4 default ('freq 20000' for 20 kHz)
static const uint32_t MAX_MD_PWM_FREQ_HZ    = 20000;  // MDD10A rated max PWM frequency
static const uint32_t CHOP_PWM_FREQ_HZ      = 5000;   // 5 kHz brake chopper PWM
static const uint8_t  PWM_RES_BITS       = 10;     // 10-bit resolution (0..1023)
static const uint32_t PWM_MAX_DUTY       = (1U << PWM_RES_BITS) - 1U;

static const int      LEDC_CH_MOTOR      = 0;      // v2.x LEDC Timer 0
static const int      LEDC_CH_CHOPPER    = 2;      // v2.x LEDC Timer 1 (avoids sharing Timer 0)

// Verified Encoder & MY1016 + GBX1 Drivetrain Constants
static const float    ENC_CPR_MOTOR      = 1440.0f;                 // 360 PPR * 4x quadrature
static const float    ENC_CPR_OUTPUT     = ENC_CPR_MOTOR;           // Verified 1:1 on output shaft
static const float    DEG_PER_COUNT      = 360.0f / ENC_CPR_OUTPUT; // 0.2500 deg/count
static const int      VEL_WINDOW_TICKS   = 20;                      // 40 ms sliding window @ 500 Hz
static const float    RAD_PER_DEG        = 0.0174532925f;
static const float    DEG_PER_RAD        = 57.2957795f;

static const float    GEAR_RATIO         = 52.0f;
static const float    GEAR_EFFICIENCY    = 0.85f;
static const float    KT_MOTOR_NM_PER_A  = 0.0694f;
static const float    GBX_MAX_TORQUE_NM  = 100.0f;

static const float    DEFAULT_D_OFFSET_M    = 0.250f;  // 25.0 cm knee below arm pivot
static const float    DEFAULT_L_LEG_M       = 0.300f;  // 30.0 cm tibial pad lever arm

static const float    DEFAULT_R_LOOP_OHM    = 0.40f;   // Motor + harness + FET resistance
static const float    DEFAULT_V_BRUSH_V     = 0.35f;   // Brushed commutator drop
static const float    DEFAULT_KP_I          = 0.25f;   // Current loop P gain (V/A)
static const float    DEFAULT_KI_I          = 8.0f;    // Current loop I gain (V/(A*s))
static const float    CONT_CURRENT_LIMIT_A  = 10.0f;   // MDD10A continuous channel rating
static const float    PEAK_CURRENT_LIMIT_A  = 15.0f;   // Allowed <5s surge peak (30A/10s HW max)
static const float    PEAK_SURGE_MAX_S      = 5.0f;    // Max continuous surge window > 10A
static const float    HW_OVERCURRENT_CUT_A  = 25.0f;   // Overcurrent cutoff threshold (MDD10A peak 30A/10s)
static const uint16_t HW_OVERCURRENT_MS     = 500U;    // 500 ms persistence rides out inrush spikes
static const uint16_t HW_OVERCURRENT_TICKS  = (uint16_t)(HW_OVERCURRENT_MS / (CONTROL_PERIOD_US / 1000U));
static const float    MAX_SAFE_ARM_DEGS     = 330.0f;  // Overspeed cutoff (~55 RPM output)
static const float    VBUS_CHOP_OFF_OV_V    = 27.5f;   // CHOP_OFF OV shutdown ceiling

// Stage 6 ROM, Self-Homing, CPM & Active-Assist Defaults
static const float    ROM_ABS_MIN_DEG        = 0.0f;
static const float    ROM_ABS_MAX_DEG        = 120.0f;
static const float    ROM_MIN_SPAN_DEG       = 10.0f;
static const float    DEFAULT_ROM_MIN_DEG    = 5.0f;    // Safe extension limit off STOP1
static const float    DEFAULT_ROM_MAX_DEG    = 90.0f;   // Default flexion limit
static const float    DEFAULT_ASSIST_MAX_I_A = 6.0f;    // Outer-loop current cap (A)
static const float    DEFAULT_KP_POS_NM_DEG  = 0.55f;   // Knee position gain (Nm/deg)
static const float    DEFAULT_KD_VEL_NMS_DEG = 0.06f;   // Knee velocity gain (Nm/(deg/s))
static const float    DEFAULT_CPM_SPEED_DEGS = 18.0f;
static const float    DEFAULT_AST_SPEED_DEGS = 22.0f;
static const float    ASSIST_NUDGE_TRIG_DEG  = 1.25f;   // Patient nudge displacement trigger
static const float    ASSIST_VEL_TRIG_DEGS   = 2.5f;    // Intentional motion velocity trigger
static const float    ASSIST_RESIST_ERR_DEG  = 12.0f;   // Resistance/spasm pause threshold

static const float    VBUS_DIV_MULT         = 16.942f; // (73.333k + 4.6k) / 4.6k
static const float    ACS712_MV_PER_AMP     = 33.0f;   // 66 mV/A * 0.50 (22k/22k divider)

static const char    *NVS_NAMESPACE         = "tophub_stg";

// ----------------------------------------------------------------------------
// 3. STATE MACHINE, KINEMATICS & GLOBAL STATE
// ----------------------------------------------------------------------------
enum OperatingMode : uint8_t {
    MODE_IDLE = 0,
    MODE_DIRECT_PWM,
    MODE_BENCH_PULSE,
    MODE_CHOPPER_PULSE,
    MODE_DIRTEST,
    MODE_CAL_ZERO,
    MODE_CURRENT_REF,
    MODE_AUTOHOME,
    MODE_CPM,
    MODE_ACTIVE_ASSIST,
    MODE_GOTO
};

enum ChopperMode : uint8_t {
    CHOP_OFF = 0,
    CHOP_AUTO,
    CHOP_MANUAL
};

static Preferences   g_prefs;

static OperatingMode g_mode                 = MODE_IDLE;
static int           g_cmd_motor_pct        = 0;
static int           g_cmd_motor_dir        = 0;      // 0 = REV (LOW)
static float         g_cmd_iref_a           = 0.0f;
static float         g_active_iref_a        = 0.0f;   // Effective current ref (IREF/CPM/ASSIST/GOTO)
static ChopperMode   g_chop_mode            = CHOP_OFF; // Disabled until 5V NPN U202 driver is fitted
static int           g_cmd_chop_pct         = 0;
static int           g_ctest_pct            = 10;
static float         g_chopper_duty_pct     = 0.0f;
static bool          g_chop_thr_custom      = false;
static float         g_chop_v_on_custom     = VBUS_CHOP_DEFAULT_ON_V;
static float         g_chop_v_max_custom    = VBUS_CHOP_DEFAULT_ON_V + VBUS_CHOP_RAMP_SPAN_V;
static uint32_t      g_pulse_start_ms       = 0;
static bool          g_bench_stop_phase     = false;

static int           g_motor_pwm_pct        = 0;
static float         g_motor_pwm_pct_f      = 0.0f;
static int           g_motor_dir            = 0;
static int           g_chop_pwm_pct         = 0;
static bool          g_live_print           = true;

// Current Regulator Parameters & Runtime State
static float         g_r_loop_ohm           = DEFAULT_R_LOOP_OHM;
static float         g_v_brush_v            = DEFAULT_V_BRUSH_V;
static float         g_kp_i                 = DEFAULT_KP_I;
static float         g_ki_i                 = DEFAULT_KI_I;
static float         g_pi_integral_v        = 0.0f;
static float         g_ctrl_v_ff            = 0.0f;
static float         g_ctrl_v_cmd           = 0.0f;
static float         g_surge_budget_s       = 0.0f;
static bool          g_surge_foldback_act   = false;
static bool          g_surge_msg_pend       = false;
static uint16_t      g_oc_ticks             = 0;
static bool          g_oc_fault_msg_pend    = false;
static bool          g_regen_clamp_sent     = false;
static bool          g_regen_msg_pend       = false;
static uint16_t      g_acs_rev_neg_cnt      = 0;
static bool          g_acs_pol_warn_sent    = false;
static bool          g_acs_pol_warn_pending = false;
static bool          g_overspeed_cut_flag   = false;
static bool          g_overspeed_msg_pend   = false;

// Persistent Calibration & Geometry (stored in NVS)
static bool          g_cal_valid          = false;
static float         g_acs_zero_mv        = 1250.0f;
static float         g_vbus_idle_v        = 24.0f;
static float         g_d_offset_m         = DEFAULT_D_OFFSET_M;
static float         g_L_leg_m            = DEFAULT_L_LEG_M;
static int8_t        g_dir_polarity       = +1; // +1: DIR=HIGH moves +angle (Flexion)
static int8_t        g_enc_polarity       = +1;
static int8_t        g_acs_polarity       = +1;
static float         g_rom_min_deg        = DEFAULT_ROM_MIN_DEG;
static float         g_rom_max_deg        = DEFAULT_ROM_MAX_DEG;
static bool          g_nvs_save_pending   = false;

// 500 Hz Analog Sensor State
static uint16_t      g_vbus_raw_adc       = 0;
static float         g_vbus_raw_mv        = 0.0f;
static bool          g_vbus_valid         = false;
static float         g_vbus_volts         = 24.0f;

static uint16_t      g_acs_raw_adc        = 0;
static float         g_acs_raw_mv         = 0.0f;
static bool          g_acs_valid          = false;
static float         g_motor_current_a    = 0.0f;

// Encoder, Angle, Velocity & Passive Torque/Force State
static bool          g_is_homed           = false;
static int64_t       g_enc_counts         = 0;
static int16_t       g_prev_pcnt_raw      = 0;
static float         g_tm_zero_offset_deg = 0.0f;
static float         g_theta_m_deg        = 90.0f;
static float         g_omega_m_deg_s      = 0.0f;
static float         g_theta_k_deg        = 90.0f;
static float         g_omega_k_deg_s      = 0.0f;
static float         g_gamma_factor       = 1.83f;
static float         g_gbx_torque_nm      = 0.0f;
static float         g_knee_torque_nm     = 0.0f;
static float         g_pad_force_n        = 0.0f;

static float         g_vel_hist_tm[VEL_WINDOW_TICKS];
static float         g_vel_hist_tk[VEL_WINDOW_TICKS];
static int           g_vel_hist_idx       = 0;

// Non-blocking Test / Calibration Accumulators
static int64_t       g_dirtest_start_cnt  = 0;
static int64_t       g_dirtest_mid_cnt    = 0;
static uint8_t       g_dirtest_phase      = 0;
static double        g_dirtest_acs_sum    = 0.0;
static uint32_t      g_dirtest_samples    = 0;
static bool          g_dirtest_done_msg   = false;
static int64_t       g_dirtest_fwd_dcnt   = 0;
static int64_t       g_dirtest_rev_dcnt   = 0;
static float         g_dirtest_fwd_avg_i  = 0.0f;
static float         g_dirtest_rev_avg_i  = 0.0f;
static bool          g_dirtest_bidir_ok   = false;
static bool          g_dirtest_dir_flip   = false;
static bool          g_dirtest_acs_flip   = false;

static double        g_cal_acs_sum        = 0.0;
static double        g_cal_vbus_sum       = 0.0;
static uint32_t      g_cal_acs_cnt        = 0;
static uint32_t      g_cal_vbus_cnt       = 0;
static bool          g_cal_done_msg       = false;
static bool          g_cal_acs_ok         = false;
static bool          g_cal_vbus_ok        = false;

static uint32_t      g_loop_us_win_max    = 0;
static uint32_t      g_loop_us_peak       = 0;
static bool          g_ctest_done_msg     = false;

// Stage 6 Control Gains & Trajectory / Auto-Home / Active-Assist / Rep State
static float         g_kp_pos             = DEFAULT_KP_POS_NM_DEG;
static float         g_kd_vel             = DEFAULT_KD_VEL_NMS_DEG;
static float         g_assist_max_i_a     = DEFAULT_ASSIST_MAX_I_A;
static float         g_grav_ff_nm         = 0.0f;    // Optional -cos(Tk) gravity torque FF

static uint8_t       g_home_phase         = 0;       // 0=idle,1=seek,2=settle,3=backoff
static uint32_t      g_home_start_ms      = 0;
static uint32_t      g_home_phase_ms      = 0;
static int64_t       g_home_start_cnt     = 0;
static int64_t       g_home_window_cnt    = 0;
static uint8_t       g_home_stall_hits    = 0;
static float         g_home_park_deg      = DEFAULT_ROM_MIN_DEG;
static uint8_t       g_homing_msg_code    = 0;       // 1..6 non-blocking result for loop()

static float         g_traj_target_tk     = 0.0f;
static float         g_goto_goal_tk       = 0.0f;
static float         g_goto_speed_degs    = 20.0f;
static uint32_t      g_goto_settle_ms     = 0;
static bool          g_goto_reached_sent  = false;
static uint16_t      g_obst_ticks         = 0;
static uint8_t       g_motion_msg_code    = 0;       // 1=goto_done,2=goto_blocked,3=cpm_done,4=cpm_blocked,5=ov_chop_off
static float         g_motion_msg_val     = 0.0f;

static float         g_cpm_speed_degs     = DEFAULT_CPM_SPEED_DEGS;
static int           g_cpm_target_cycles  = 0;       // 0 = continuous until 's'
static int           g_cpm_done_cycles    = 0;
static uint8_t       g_cpm_phase          = 0;       // 0=flex,1=dwell_top,2=ext,3=dwell_bot
static uint32_t      g_cpm_dwell_ms       = 0;

static bool          g_assist_auto_cycle  = false;
static float         g_assist_speed_degs  = DEFAULT_AST_SPEED_DEGS;
static uint8_t       g_assist_phase       = 0;       // 0=wait_f,1=ast_f,2=hold_top,3=wait_e,4=ast_e,5=hold_bot
static float         g_assist_anchor_tk   = 0.0f;
static uint32_t      g_assist_hold_ms     = 0;
static uint8_t       g_assist_msg_code    = 0;       // 1..6 non-blocking transition notice
static float         g_assist_msg_val     = 0.0f;

static uint8_t       g_rep_phase          = 0;       // 0=wait_start,1=in_flex,2=in_ext
static uint32_t      g_rep_count          = 0;
static float         g_rep_min_tk         = 999.0f;
static float         g_rep_max_tk         = -999.0f;
static float         g_rep_peak_flex_nm   = 0.0f;
static float         g_rep_peak_ext_nm    = 0.0f;
static float         g_rep_work_j         = 0.0f;
static uint32_t      g_last_rep_num       = 0;
static float         g_last_rep_min_tk    = 0.0f;
static float         g_last_rep_max_tk    = 0.0f;
static float         g_last_rep_flex_nm   = 0.0f;
static float         g_last_rep_ext_nm    = 0.0f;
static float         g_last_rep_work_j    = 0.0f;
static bool          g_rep_done_msg       = false;

static int           g_enc_a_last         = -1;
static int           g_enc_b_last         = -1;
static uint32_t      g_enc_a_edges        = 0;
static uint32_t      g_enc_b_edges        = 0;

static char          g_cmd_buf[64];
static uint8_t       g_cmd_len            = 0;

// ----------------------------------------------------------------------------
// 4. CHANGE-ONLY PWM HELPERS (Arduino-ESP32 v2.x & v3.x compatible)
// ----------------------------------------------------------------------------
static int32_t s_last_motor_duty   = -1;
static int8_t  s_last_motor_dir    = -1;
static int32_t s_last_chopper_duty = -1;

static inline uint32_t pctToDuty(int pct) {
    return (uint32_t)(((uint32_t)pct * PWM_MAX_DUTY) / 100U);
}

static inline void writeLedcOutput(int pin, int ch, uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    (void)ch;
    ledcWrite(pin, duty);
#else
    (void)pin;
    ledcWrite(ch, duty);
#endif
}

static void configureMotorPWMChannel() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcDetach(PIN_MD_PWM);
    ledcAttach(PIN_MD_PWM, g_md_pwm_freq_hz, PWM_RES_BITS);
#else
    ledcSetup(LEDC_CH_MOTOR, g_md_pwm_freq_hz, PWM_RES_BITS);
    ledcAttachPin(PIN_MD_PWM, LEDC_CH_MOTOR);
#endif
    s_last_motor_duty = -1;
}

static void initPWM() {
    pinMode(PIN_MD_PWM, OUTPUT);
    digitalWrite(PIN_MD_PWM, LOW);
    pinMode(PIN_MD_DIR, OUTPUT);
    digitalWrite(PIN_MD_DIR, LOW);
    pinMode(PIN_CHOP_PWM, OUTPUT);
    digitalWrite(PIN_CHOP_PWM, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
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
    s_last_motor_dir    = 0;
    s_last_motor_duty   = 0;
    s_last_chopper_duty = 0;
}

static void applyMotorFloatPct(int dir, float pct) {
    float clamped = constrain(pct, 0.0f, (float)MAX_MOTOR_PWM_PCT);
    if (!MOTOR_BIDIRECTIONAL_ENABLED) {
        dir = 0; // Hardware DIR locked in REV (LOW)
    }
    int8_t target_dir = (dir != 0) ? 1 : 0;
    uint32_t target_duty = (uint32_t)lroundf((clamped * (float)PWM_MAX_DUTY) / 100.0f);
    if (target_duty > PWM_MAX_DUTY) target_duty = PWM_MAX_DUTY;

    if (target_dir != s_last_motor_dir) {
        digitalWrite(PIN_MD_DIR, target_dir ? HIGH : LOW);
        s_last_motor_dir = target_dir;
    }
    if ((int32_t)target_duty != s_last_motor_duty) {
        writeLedcOutput(PIN_MD_PWM, LEDC_CH_MOTOR, target_duty);
        s_last_motor_duty = (int32_t)target_duty;
    }

    g_motor_dir       = target_dir;
    g_motor_pwm_pct_f = clamped;
    g_motor_pwm_pct   = (int)lroundf(clamped);
}

static void applyMotorOutput(int dir, int pct) {
    applyMotorFloatPct(dir, (float)pct);
}

static void applyChopperFloatPct(float pct, float max_pct) {
    float clamped = constrain(pct, 0.0f, max_pct);
    uint32_t target_duty = (uint32_t)lroundf((clamped * (float)PWM_MAX_DUTY) / 100.0f);
    if (target_duty > PWM_MAX_DUTY) target_duty = PWM_MAX_DUTY;

    if ((int32_t)target_duty != s_last_chopper_duty) {
        writeLedcOutput(PIN_CHOP_PWM, LEDC_CH_CHOPPER, target_duty);
        s_last_chopper_duty = (int32_t)target_duty;
    }
    g_chopper_duty_pct = clamped;
    g_chop_pwm_pct     = (int)lroundf(clamped);
}

static void applyChopperOutput(int pct) {
    applyChopperFloatPct((float)pct, (float)MAX_CHOP_MANUAL_PCT);
}

static void stopAllImmediate() {
    g_mode               = MODE_IDLE;
    g_cmd_motor_pct      = 0;
    g_cmd_motor_dir      = 0;
    g_cmd_iref_a         = 0.0f;
    g_active_iref_a      = 0.0f;
    g_pi_integral_v      = 0.0f;
    g_ctrl_v_ff          = 0.0f;
    g_ctrl_v_cmd         = 0.0f;
    g_oc_ticks           = 0;
    g_obst_ticks         = 0;
    g_home_phase         = 0;
    g_regen_clamp_sent   = false;
    g_acs_rev_neg_cnt    = 0;
    g_acs_pol_warn_sent  = false;
    g_overspeed_cut_flag = false;
    g_cmd_chop_pct       = 0;
    if (g_chop_mode == CHOP_MANUAL) {
        g_chop_mode = CHOP_OFF;
    }
    applyMotorFloatPct(0, 0.0f);
    applyChopperFloatPct(0.0f, (float)MAX_CHOP_AUTO_PCT);
}

// ----------------------------------------------------------------------------
// 5. NON-COLLINEAR OFFSET-PIVOT KINEMATICS
// ----------------------------------------------------------------------------
static inline float mapKneeToArmAngleDeg(float theta_k_deg, float d_offset_m, float L_leg_m) {
    float tk_rad = theta_k_deg * RAD_PER_DEG;
    float y = L_leg_m * sinf(tk_rad) + d_offset_m;
    float x = L_leg_m * cosf(tk_rad);
    return atan2f(y, x) * DEG_PER_RAD;
}

static inline float mapArmToKneeAngleDeg(float theta_m_deg, float d_offset_m, float L_leg_m) {
    float tm_rad = theta_m_deg * RAD_PER_DEG;
    float ratio  = (L_leg_m > 0.05f) ? (d_offset_m / L_leg_m) : 0.8333f;
    float arg    = constrain(ratio * cosf(tm_rad), -0.99f, 0.99f);
    return theta_m_deg - (asinf(arg) * DEG_PER_RAD);
}

static inline float computeGammaFactor(float theta_k_deg, float d_offset_m, float L_leg_m) {
    float ratio = (L_leg_m > 0.05f) ? (d_offset_m / L_leg_m) : 0.8333f;
    float s     = sinf(constrain(theta_k_deg, 0.0f, 130.0f) * RAD_PER_DEG);
    return constrain(1.0f + ratio * s, 1.0f, 2.50f);
}

// ----------------------------------------------------------------------------
// 6. HARDWARE PCNT ENCODER, KINEMATICS & SLIDING-WINDOW VELOCITY
// ----------------------------------------------------------------------------
static void resetVelocityWindow(float theta_m_deg, float theta_k_deg) {
    for (int i = 0; i < VEL_WINDOW_TICKS; ++i) {
        g_vel_hist_tm[i] = theta_m_deg;
        g_vel_hist_tk[i] = theta_k_deg;
    }
    g_vel_hist_idx  = 0;
    g_omega_m_deg_s = 0.0f;
    g_omega_k_deg_s = 0.0f;
}

static void latchEncoderAngleDeg(float known_theta_m_deg) {
    pcnt_counter_clear(PCNT_UNIT_0);
    g_prev_pcnt_raw      = 0;
    g_enc_counts         = (int64_t)llroundf(known_theta_m_deg / DEG_PER_COUNT);
    g_tm_zero_offset_deg = known_theta_m_deg - ((float)g_enc_counts * DEG_PER_COUNT);
    g_theta_m_deg        = known_theta_m_deg;
    g_theta_k_deg        = mapArmToKneeAngleDeg(known_theta_m_deg, g_d_offset_m, g_L_leg_m);
    g_gamma_factor       = computeGammaFactor(g_theta_k_deg, g_d_offset_m, g_L_leg_m);
    resetVelocityWindow(g_theta_m_deg, g_theta_k_deg);
}

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

    // Boot default: nominal 90 deg vertical hang (not yet homed)
    latchEncoderAngleDeg(mapKneeToArmAngleDeg(90.0f, g_d_offset_m, g_L_leg_m));
    g_is_homed           = false;
    g_enc_a_last         = digitalRead(PIN_ENC1_A);
    g_enc_b_last         = digitalRead(PIN_ENC1_B);
    g_enc_a_edges        = 0;
    g_enc_b_edges        = 0;
}

static void updateEncoderKinematics500Hz() {
    int a_now = digitalRead(PIN_ENC1_A);
    int b_now = digitalRead(PIN_ENC1_B);
    if (g_enc_a_last >= 0 && a_now != g_enc_a_last) g_enc_a_edges++;
    if (g_enc_b_last >= 0 && b_now != g_enc_b_last) g_enc_b_edges++;
    g_enc_a_last = a_now;
    g_enc_b_last = b_now;

    int16_t raw = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &raw);
    int16_t delta = raw - g_prev_pcnt_raw;
    g_prev_pcnt_raw = raw;
    if (raw > 25000 || raw < -25000) {
        pcnt_counter_clear(PCNT_UNIT_0);
        g_prev_pcnt_raw = 0;
    }

    g_enc_counts += (int64_t)delta * (int64_t)g_enc_polarity;

    g_theta_m_deg  = (float)g_enc_counts * DEG_PER_COUNT + g_tm_zero_offset_deg;
    g_theta_k_deg  = mapArmToKneeAngleDeg(g_theta_m_deg, g_d_offset_m, g_L_leg_m);
    g_gamma_factor = computeGammaFactor(g_theta_k_deg, g_d_offset_m, g_L_leg_m);

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
}

static void zeroEncoder() {
    latchEncoderAngleDeg(0.0f);
    g_is_homed = false;
}

// ----------------------------------------------------------------------------
// 7. NVS PERSISTENCE & 500 Hz ANALOG SENSORS (VBUS & ACS712)
// ----------------------------------------------------------------------------
static float readAveragedMilliVolts(int pin, int samples = 64) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; ++i) {
        sum += analogReadMilliVolts(pin);
    }
    return (float)sum / (float)samples;
}

static void zeroACS712BootFallback() {
    float mv = readAveragedMilliVolts(PIN_ACS712_SENSE, 64);
    if (mv >= 300.0f && mv <= 2850.0f) {
        g_acs_zero_mv = mv;
    } else {
        g_acs_zero_mv = 1250.0f;
    }
}

static void loadCalibrationFromNVS() {
    if (!g_prefs.begin(NVS_NAMESPACE, true)) {
        g_cal_valid    = false;
        g_acs_zero_mv  = 1250.0f;
        g_vbus_idle_v  = 24.0f;
        g_d_offset_m   = DEFAULT_D_OFFSET_M;
        g_L_leg_m      = DEFAULT_L_LEG_M;
        g_dir_polarity = +1;
        g_enc_polarity = +1;
        g_acs_polarity = +1;
        return;
    }

    g_cal_valid = g_prefs.isKey("cal_ok") ? g_prefs.getBool("cal_ok", false) : false;
    float v_z   = g_prefs.isKey("v_zero") ? g_prefs.getFloat("v_zero", 1250.0f) : 1250.0f;
    float vb_0  = g_prefs.isKey("vbus_0") ? g_prefs.getFloat("vbus_0", 24.0f) : 24.0f;
    float d_off = g_prefs.isKey("d_off")  ? g_prefs.getFloat("d_off", DEFAULT_D_OFFSET_M) : DEFAULT_D_OFFSET_M;
    float l_leg = g_prefs.isKey("l_leg")  ? g_prefs.getFloat("l_leg", DEFAULT_L_LEG_M) : DEFAULT_L_LEG_M;
    int8_t dp   = g_prefs.isKey("dp")     ? (int8_t)g_prefs.getChar("dp", 1) : 1;
    int8_t ep   = g_prefs.isKey("ep")     ? (int8_t)g_prefs.getChar("ep", 1) : 1;
    int8_t ap   = g_prefs.isKey("ap")     ? (int8_t)g_prefs.getChar("ap", 1) : 1;
    float rmin  = g_prefs.isKey("rmin")   ? g_prefs.getFloat("rmin", DEFAULT_ROM_MIN_DEG) : DEFAULT_ROM_MIN_DEG;
    float rmax  = g_prefs.isKey("rmax")   ? g_prefs.getFloat("rmax", DEFAULT_ROM_MAX_DEG) : DEFAULT_ROM_MAX_DEG;
    g_prefs.end();

    g_acs_zero_mv  = (isfinite(v_z) && v_z >= 300.0f && v_z <= 2850.0f) ? v_z : 1250.0f;
    g_vbus_idle_v  = (isfinite(vb_0) && vb_0 >= 5.0f && vb_0 <= 35.0f) ? vb_0 : 24.0f;
    g_L_leg_m      = (isfinite(l_leg) && l_leg >= 0.15f && l_leg <= 0.60f) ? l_leg : DEFAULT_L_LEG_M;
    g_d_offset_m   = (isfinite(d_off) && d_off >= 0.0f && d_off <= 0.95f * g_L_leg_m) ? d_off : DEFAULT_D_OFFSET_M;
    g_dir_polarity = (dp == -1) ? -1 : +1;
    g_enc_polarity = (ep == -1) ? -1 : +1;
    g_acs_polarity = (ap == -1) ? -1 : +1;
    if (isfinite(rmin) && isfinite(rmax) &&
        rmin >= ROM_ABS_MIN_DEG && rmax <= ROM_ABS_MAX_DEG &&
        (rmax - rmin) >= ROM_MIN_SPAN_DEG) {
        g_rom_min_deg = rmin;
        g_rom_max_deg = rmax;
    } else {
        g_rom_min_deg = DEFAULT_ROM_MIN_DEG;
        g_rom_max_deg = DEFAULT_ROM_MAX_DEG;
    }
}

static void saveCalibrationToNVS() {
    if (!g_prefs.begin(NVS_NAMESPACE, false)) {
        Serial.println("[NVS] ERROR: failed to open NVS namespace for write.");
        return;
    }
    g_prefs.putBool("cal_ok", g_cal_valid);
    g_prefs.putFloat("v_zero", g_acs_zero_mv);
    g_prefs.putFloat("vbus_0", g_vbus_idle_v);
    g_prefs.putFloat("d_off", g_d_offset_m);
    g_prefs.putFloat("l_leg", g_L_leg_m);
    g_prefs.putChar("dp", g_dir_polarity);
    g_prefs.putChar("ep", g_enc_polarity);
    g_prefs.putChar("ap", g_acs_polarity);
    g_prefs.putFloat("rmin", g_rom_min_deg);
    g_prefs.putFloat("rmax", g_rom_max_deg);
    g_prefs.end();
}

static void clearCalibrationNVS() {
    if (g_prefs.begin(NVS_NAMESPACE, false)) {
        g_prefs.clear();
        g_prefs.end();
    }
    g_cal_valid    = false;
    g_d_offset_m   = DEFAULT_D_OFFSET_M;
    g_L_leg_m      = DEFAULT_L_LEG_M;
    g_dir_polarity = +1;
    g_enc_polarity = +1;
    g_acs_polarity = +1;
    g_rom_min_deg  = DEFAULT_ROM_MIN_DEG;
    g_rom_max_deg  = DEFAULT_ROM_MAX_DEG;
    g_vbus_idle_v  = 24.0f;
    g_is_homed     = false;
    zeroACS712BootFallback();
    latchEncoderAngleDeg(mapKneeToArmAngleDeg(90.0f, g_d_offset_m, g_L_leg_m));
    g_is_homed = false;
}

static void sampleAnalogSensors500Hz() {
    g_vbus_raw_adc = (uint16_t)analogRead(PIN_VBUS_SENSE);
    g_vbus_raw_mv  = (float)analogReadMilliVolts(PIN_VBUS_SENSE);
    if (g_vbus_raw_adc >= 100 && g_vbus_raw_adc <= 3500 &&
        g_vbus_raw_mv >= 600.0f && g_vbus_raw_mv <= 2300.0f) {
        float vbus_meas = (g_vbus_raw_mv * 0.001f) * VBUS_DIV_MULT;
        if (!g_vbus_valid) {
            g_vbus_volts = vbus_meas; // Seed on first valid sample
        } else {
            g_vbus_volts = 0.85f * g_vbus_volts + 0.15f * vbus_meas;
        }
        g_vbus_valid = true;
    } else {
        g_vbus_valid = false;
        g_vbus_volts = 24.0f;
    }

    g_acs_raw_adc = (uint16_t)analogRead(PIN_ACS712_SENSE);
    g_acs_raw_mv  = (float)analogReadMilliVolts(PIN_ACS712_SENSE);
    if (g_acs_raw_adc >= 250 && g_acs_raw_adc <= 3900 &&
        g_acs_raw_mv >= 300.0f && g_acs_raw_mv <= 2850.0f) {
        float i_raw = ((g_acs_raw_mv - g_acs_zero_mv) / ACS712_MV_PER_AMP) * (float)g_acs_polarity;
        if (!g_acs_valid) {
            g_motor_current_a = i_raw; // Seed on first valid sample
        } else {
            g_motor_current_a = 0.70f * g_motor_current_a + 0.30f * i_raw;
        }
        g_acs_valid = true;
    } else {
        g_acs_valid       = false;
        g_motor_current_a = 0.0f;
    }
}

static void printAdcDiagnostics() {
    Serial.printf("[ADC] VBUS: raw=%4u (%6.1f mV) [%s] -> %.2f V (NVS idle=%.2f V)\n",
                  g_vbus_raw_adc, g_vbus_raw_mv, g_vbus_valid ? "OK" : "NOM24V",
                  g_vbus_volts, g_vbus_idle_v);
    Serial.printf("[ADC] ACS : raw=%4u (%6.1f mV) [%s] -> %+.2f A (zero=%.1f mV, ap=%+d, cal=%d)\n",
                  g_acs_raw_adc, g_acs_raw_mv, g_acs_valid ? "OK" : "OL",
                  g_motor_current_a, g_acs_zero_mv, g_acs_polarity, g_cal_valid ? 1 : 0);
    Serial.printf("[PHYS] Tq_gbx=%+.2f Nm | Tq_knee=%+.2f Nm | F_pad=%+.1f N | G=%.2f (d=%.3fm, L=%.3fm)\n",
                  g_gbx_torque_nm, g_knee_torque_nm, g_pad_force_n,
                  g_gamma_factor, g_d_offset_m, g_L_leg_m);
}

static void printNvsStatus() {
    Serial.printf("[NVS] ns='%s' | cal=%d | v_zero=%.1f mV | vbus_idle=%.2f V | d_off=%.3f m | L_leg=%.3f m | dp=%+d ep=%+d ap=%+d | ROM=%.1f..%.1f deg\n",
                  NVS_NAMESPACE, g_cal_valid ? 1 : 0, g_acs_zero_mv, g_vbus_idle_v,
                  g_d_offset_m, g_L_leg_m, g_dir_polarity, g_enc_polarity, g_acs_polarity,
                  g_rom_min_deg, g_rom_max_deg);
}

static void printEncDiagnostics(bool show_hints = true) {
    int a_now = digitalRead(PIN_ENC1_A);
    int b_now = digitalRead(PIN_ENC1_B);
    int16_t pcnt_now = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &pcnt_now);
    Serial.printf("[ENC] GPIO%d(A)=%d (edges=%lu) | GPIO%d(B)=%d (edges=%lu) | pcnt_raw=%d | ENC=%lld (ep=%+d) | Tm=%.2f Tk=%.2f | VBUS=%.2fV(%s)\n",
                  PIN_ENC1_A, a_now, (unsigned long)g_enc_a_edges,
                  PIN_ENC1_B, b_now, (unsigned long)g_enc_b_edges,
                  (int)pcnt_now, (long long)g_enc_counts, g_enc_polarity,
                  g_theta_m_deg, g_theta_k_deg, g_vbus_volts, g_vbus_valid ? "OK" : "NOM24V");
    if (!show_hints) return;
    if (g_enc_a_edges == 0 && g_enc_b_edges == 0) {
        Serial.println("[ENC] Note: 0 edges seen at 500Hz. Turn shaft slowly and re-run 'enc'; if still 0 & A=B=1, check +5V_BUCK power to ENC1.");
    } else if ((g_enc_a_edges > 0 && g_enc_b_edges == 0) || (g_enc_a_edges == 0 && g_enc_b_edges > 0)) {
        Serial.printf("[ENC] FAULT HINT: Only %s channel is toggling! Quadrature PCNT nets 0 when one channel is open/shorted — check R3/R5 (80 Ohm) & A/B wires.\n",
                      (g_enc_a_edges > 0) ? "A (GPIO32)" : "B (GPIO33)");
    }
}

static const char *chopModeName(ChopperMode cm) {
    switch (cm) {
        case CHOP_OFF:    return "OFF ";
        case CHOP_AUTO:   return "AUTO";
        case CHOP_MANUAL: return "MAN ";
        default:          return "UNK ";
    }
}

static void getChopperThresholds(float *v_on_out, float *v_max_out) {
    float v_on = VBUS_CHOP_DEFAULT_ON_V;
    float v_max = v_on + VBUS_CHOP_RAMP_SPAN_V;
    if (g_chop_thr_custom) {
        v_on  = g_chop_v_on_custom;
        v_max = g_chop_v_max_custom;
    } else if (g_cal_valid && g_vbus_idle_v >= 15.0f && g_vbus_idle_v <= 27.0f) {
        v_on  = g_vbus_idle_v + 0.50f;
        v_max = fminf(VBUS_CHOP_ABS_MAX_V, v_on + VBUS_CHOP_RAMP_SPAN_V);
    }
    if (v_max < v_on + 0.25f) v_max = v_on + 0.25f;
    *v_on_out  = v_on;
    *v_max_out = v_max;
}

static float computeChopperDutyForVbus(float vbus_v, float v_on, float v_max) {
    if (vbus_v <= v_on) return 0.0f;
    if (vbus_v >= v_max) return 100.0f;
    return ((vbus_v - v_on) / (v_max - v_on)) * 100.0f;
}

static void updateBrakeChopper(float vbus_v) {
    if (g_chop_mode == CHOP_OFF) {
        applyChopperFloatPct(0.0f, (float)MAX_CHOP_AUTO_PCT);
        return;
    }
    if (g_chop_mode == CHOP_MANUAL) {
        applyChopperFloatPct((float)g_cmd_chop_pct, (float)MAX_CHOP_MANUAL_PCT);
        return;
    }
    if (!g_vbus_valid || vbus_v < 15.0f || vbus_v > 35.0f) {
        applyChopperFloatPct(0.0f, (float)MAX_CHOP_AUTO_PCT);
        return;
    }
    float v_on = 0.0f, v_max = 0.0f;
    getChopperThresholds(&v_on, &v_max);
    float duty_pct = computeChopperDutyForVbus(vbus_v, v_on, v_max);
    applyChopperFloatPct(duty_pct, (float)MAX_CHOP_AUTO_PCT);
}

static void printChopperStatus() {
    float v_on = 0.0f, v_max = 0.0f;
    getChopperThresholds(&v_on, &v_max);
    Serial.printf("[CHOP] Mode:%s | Duty:%5.1f%% @%luHz | VBUS:%.2fV (%s) | Thr:%s on=%.2fV full=%.2fV\n",
                  chopModeName(g_chop_mode), g_chopper_duty_pct, (unsigned long)CHOP_PWM_FREQ_HZ,
                  g_vbus_volts, g_vbus_valid ? "OK" : "NOM24V",
                  g_chop_thr_custom ? "CUSTOM" : (g_cal_valid ? "CAL+0.5V" : "DEF25.5V"),
                  v_on, v_max);
}

static float updateSurgeFoldbackTarget(float i_target_a) {
    float i_abs_ref  = fabsf(i_target_a);
    float i_abs_meas = g_acs_valid ? fabsf(g_motor_current_a) : 0.0f;
    // Allow minor +/-0.35A ADC ripple at 10A without blocking surge cooldown
    bool surging = (i_abs_ref > CONT_CURRENT_LIMIT_A) ||
                   (i_abs_meas > (CONT_CURRENT_LIMIT_A + 0.35f));

    if (surging) {
        g_surge_budget_s = fminf(PEAK_SURGE_MAX_S, g_surge_budget_s + CONTROL_DT_SEC);
        if (g_surge_budget_s >= PEAK_SURGE_MAX_S && !g_surge_foldback_act) {
            g_surge_foldback_act = true;
            g_surge_msg_pend     = true;
        }
    } else {
        g_surge_budget_s = fmaxf(0.0f, g_surge_budget_s - 0.5f * CONTROL_DT_SEC);
        if (g_surge_foldback_act && g_surge_budget_s <= (0.5f * PEAK_SURGE_MAX_S)) {
            g_surge_foldback_act = false;
        }
    }

    float cap = g_surge_foldback_act ? CONT_CURRENT_LIMIT_A : PEAK_CURRENT_LIMIT_A;
    return constrain(i_target_a, -cap, cap);
}

static inline bool isCurrentControlledMode(OperatingMode m) {
    return (m == MODE_CURRENT_REF || m == MODE_CPM || m == MODE_ACTIVE_ASSIST || m == MODE_GOTO);
}

static void executeBidirectionalCurrentControl(float i_target_a, float omega_m_deg_s) {
    float i_eff = updateSurgeFoldbackTarget(i_target_a);

    // Speed safety cutoff (> 330 deg/s output arm ~ 55 RPM) & near-zero demand cutoff
    if (fabsf(omega_m_deg_s) > MAX_SAFE_ARM_DEGS) {
        if (!g_overspeed_cut_flag) {
            g_overspeed_cut_flag = true;
            g_overspeed_msg_pend = true;
        }
        g_active_iref_a   = 0.0f;
        g_pi_integral_v   = 0.0f;
        g_ctrl_v_ff       = 0.0f;
        g_ctrl_v_cmd      = 0.0f;
        g_acs_rev_neg_cnt = 0;
        applyMotorFloatPct(0, 0.0f);
        return;
    }
    g_overspeed_cut_flag = false;

    if (fabsf(i_eff) < 0.08f) {
        g_active_iref_a   = 0.0f;
        g_pi_integral_v   = 0.0f;
        g_ctrl_v_ff       = 0.0f;
        g_ctrl_v_cmd      = 0.0f;
        g_acs_rev_neg_cnt = 0;
        applyMotorFloatPct(0, 0.0f);
        return;
    }
    g_active_iref_a = i_eff;

    float omega_motor_rads = (omega_m_deg_s * RAD_PER_DEG) * GEAR_RATIO;
    float e_bemf_v         = KT_MOTOR_NM_PER_A * omega_motor_rads;
    float sign_i           = (i_eff >= 0.0f) ? 1.0f : -1.0f;
    float v_ff             = (i_eff * g_r_loop_ohm) + e_bemf_v + (sign_i * g_v_brush_v);
    float v_cmd            = v_ff;

    float vbus_safe = (g_vbus_valid && g_vbus_volts >= 15.0f && g_vbus_volts <= 30.0f)
                      ? g_vbus_volts : 24.0f;
    float v_max_cmd = 0.98f * vbus_safe;

    // Polarity-reversal guard: during same-sign drive >25% PWM, detect sustained opposite ACS sign
    bool same_sign_drive = (v_ff * i_eff) > 0.0f;
    bool wrong_acs_sign  = (i_eff > 0.50f && g_motor_current_a < -0.50f) ||
                           (i_eff < -0.50f && g_motor_current_a > 0.50f);
    if (g_acs_valid && g_motor_pwm_pct_f > 25.0f && same_sign_drive && wrong_acs_sign) {
        if (g_acs_rev_neg_cnt < 200) g_acs_rev_neg_cnt++;
    } else {
        g_acs_rev_neg_cnt = 0;
    }

    if (g_acs_valid && g_acs_rev_neg_cnt < 75) {
        float i_err    = i_eff - g_motor_current_a;
        float p_term   = g_kp_i * i_err;
        float next_int = constrain(g_pi_integral_v + (g_ki_i * CONTROL_DT_SEC * i_err), -6.0f, 6.0f);
        float pre_cmd  = v_ff + p_term + g_pi_integral_v;

        // Symmetric anti-windup against +/- 98% VBUS ceilings
        if ((pre_cmd > -v_max_cmd || i_err > 0.0f) && (pre_cmd < v_max_cmd || i_err < 0.0f)) {
            g_pi_integral_v = next_int;
        }
        v_cmd += p_term + g_pi_integral_v;
    } else {
        g_pi_integral_v = 0.0f;
        if (g_acs_rev_neg_cnt >= 75 && !g_acs_pol_warn_sent) {
            g_acs_pol_warn_sent    = true;
            g_acs_pol_warn_pending = true;
        }
    }

    // CHOP_OFF overvoltage guard: prevent 4-quadrant plugging/regen pumping bus above 26.5V
    if (g_chop_mode == CHOP_OFF && g_vbus_valid && g_vbus_volts > 26.5f && (v_cmd * i_eff < 0.0f)) {
        v_cmd = 0.0f;
        if (!g_regen_clamp_sent) {
            g_regen_clamp_sent = true;
            g_regen_msg_pend   = true;
        }
    } else {
        g_regen_clamp_sent = false;
    }

    g_ctrl_v_ff  = v_ff;
    g_ctrl_v_cmd = v_cmd;

    bool logical_pos = (v_cmd >= 0.0f);
    int hw_dir = logical_pos ? (g_dir_polarity > 0 ? 1 : 0)
                             : (g_dir_polarity > 0 ? 0 : 1);
    float duty_pct = constrain((fabsf(v_cmd) / vbus_safe) * 100.0f, 0.0f, 98.0f);
    applyMotorFloatPct(hw_dir, duty_pct);
}

static const char *modeName(OperatingMode m) {
    switch (m) {
        case MODE_IDLE:          return "IDLE ";
        case MODE_DIRECT_PWM:    return "PWM  ";
        case MODE_BENCH_PULSE:   return "TEST ";
        case MODE_CHOPPER_PULSE: return "CTEST";
        case MODE_DIRTEST:       return "DIRT ";
        case MODE_CAL_ZERO:      return "CAL  ";
        case MODE_CURRENT_REF:   return "IREF ";
        case MODE_AUTOHOME:      return "HOME ";
        case MODE_CPM:           return "CPM  ";
        case MODE_ACTIVE_ASSIST: return "ASST ";
        case MODE_GOTO:          return "GOTO ";
        default:                 return "UNK  ";
    }
}

static void printTelemetry() {
    uint32_t win_max = g_loop_us_win_max;
    g_loop_us_win_max = 0;

    Serial.printf("[S%d|%s] Tk:%6.2f(Tm:%6.2f,wk:%+6.1f,G:%.2f,H:%d) ENC:%6lld(ep:%+d,dp:%+d) | V:%5.2fV(%s) I:%+6.2f/%+5.2fA(%s,ap:%+d) | Tqg:%+5.1f F:%+4.0fN | MOT:%s%5.1f%% @%luHz | CHOP:%s%3.0f%% | dt:%lu/%luus\n",
                  FW_STAGE,
                  modeName(g_mode),
                  g_theta_k_deg,
                  g_theta_m_deg,
                  g_omega_k_deg_s,
                  g_gamma_factor,
                  g_is_homed ? 1 : 0,
                  (long long)g_enc_counts,
                  g_enc_polarity,
                  g_dir_polarity,
                  g_vbus_volts,
                  g_vbus_valid ? "OK" : "NOM",
                  g_motor_current_a,
                  isCurrentControlledMode(g_mode) ? g_active_iref_a : 0.0f,
                  g_acs_valid ? "OK" : "OL",
                  g_acs_polarity,
                  g_gbx_torque_nm,
                  g_pad_force_n,
                  (g_motor_pwm_pct_f < 0.05f) ? "STOP" : (g_motor_dir ? "FWD " : "REV "),
                  g_motor_pwm_pct_f,
                  (unsigned long)g_md_pwm_freq_hz,
                  (g_mode == MODE_CHOPPER_PULSE) ? "TEST" : chopModeName(g_chop_mode),
                  g_chopper_duty_pct,
                  (unsigned long)win_max,
                  (unsigned long)g_loop_us_peak);
}

// ----------------------------------------------------------------------------
// 7B. STAGE 6 SELF-HOMING, POSITION-ASSIST & REP TRACKER HELPERS
// ----------------------------------------------------------------------------
static inline int64_t i64abs(int64_t v) {
    return (v < 0) ? -v : v;
}

static void resetRepAccumulators() {
    g_rep_phase        = 0;
    g_rep_min_tk       = g_theta_k_deg;
    g_rep_max_tk       = g_theta_k_deg;
    g_rep_peak_flex_nm = 0.0f;
    g_rep_peak_ext_nm  = 0.0f;
    g_rep_work_j       = 0.0f;
}

static void updateRepTracker(float tk_deg, float omega_k_degs, float tau_knee_nm) {
    float span = g_rom_max_deg - g_rom_min_deg;
    if (span < ROM_MIN_SPAN_DEG) return;

    if (g_rep_phase == 0) {
        if (tk_deg >= (g_rom_min_deg + 0.25f * span)) {
            g_rep_phase        = 1;
            g_rep_min_tk       = tk_deg;
            g_rep_max_tk       = tk_deg;
            g_rep_peak_flex_nm = 0.0f;
            g_rep_peak_ext_nm  = 0.0f;
            g_rep_work_j       = 0.0f;
        }
        return;
    }

    if (tk_deg < g_rep_min_tk) g_rep_min_tk = tk_deg;
    if (tk_deg > g_rep_max_tk) g_rep_max_tk = tk_deg;
    if (tau_knee_nm > g_rep_peak_flex_nm) g_rep_peak_flex_nm = tau_knee_nm;
    if (-tau_knee_nm > g_rep_peak_ext_nm) g_rep_peak_ext_nm = -tau_knee_nm;
    g_rep_work_j += fabsf(tau_knee_nm * (omega_k_degs * RAD_PER_DEG)) * CONTROL_DT_SEC;

    if (g_rep_phase == 1) {
        if (tk_deg >= (g_rom_max_deg - 4.0f)) {
            g_rep_phase = 2;
        }
    } else if (g_rep_phase == 2) {
        if (tk_deg <= (g_rom_min_deg + 4.0f)) {
            g_rep_count++;
            g_last_rep_num     = g_rep_count;
            g_last_rep_min_tk  = g_rep_min_tk;
            g_last_rep_max_tk  = g_rep_max_tk;
            g_last_rep_flex_nm = g_rep_peak_flex_nm;
            g_last_rep_ext_nm  = g_rep_peak_ext_nm;
            g_last_rep_work_j  = g_rep_work_j;
            g_rep_done_msg     = true;
            g_rep_phase        = 0;
        }
    }
}

static float computePositionAssistCurrent(float target_tk, float target_wk,
                                          int8_t dir_mask, float max_i_a) {
    float err_tk = target_tk - g_theta_k_deg;
    float err_wk = target_wk - g_omega_k_deg_s;
    // Knee=0 deg is horizontal; opposing downward (+Flexion) gravity needs -Extension torque.
    float tau_grav_nm     = -g_grav_ff_nm * cosf(g_theta_k_deg * RAD_PER_DEG);
    float tau_knee_cmd_nm = (g_kp_pos * err_tk) + (g_kd_vel * err_wk) + tau_grav_nm;
    float tau_gbx_cmd_nm  = tau_knee_cmd_nm * g_gamma_factor;
    float kt_gbx          = GEAR_RATIO * GEAR_EFFICIENCY * KT_MOTOR_NM_PER_A;
    float i_cmd           = tau_gbx_cmd_nm / kt_gbx;

    if (dir_mask > 0) {
        i_cmd = fmaxf(0.0f, i_cmd); // Flexion-only assist (yields freely to stronger patient)
    } else if (dir_mask < 0) {
        i_cmd = fminf(0.0f, i_cmd); // Extension-only assist
    }
    if (g_theta_k_deg >= (ROM_ABS_MAX_DEG - 1.0f) && i_cmd > 0.0f) {
        i_cmd = 0.0f;
    }
    if (g_theta_k_deg <= (ROM_ABS_MIN_DEG + 0.25f) && i_cmd < 0.0f) {
        i_cmd = 0.0f;
    }
    return constrain(i_cmd, -max_i_a, max_i_a);
}

// ----------------------------------------------------------------------------
// 8. 500 Hz DETERMINISTIC CONTROL STEP
// ----------------------------------------------------------------------------
static void runControlStep500Hz() {
    uint32_t t_start = micros();

    updateEncoderKinematics500Hz();
    sampleAnalogSensors500Hz();

    // Overcurrent fault protection: >25A sustained for 500ms (250 ticks @ 500Hz) shuts down
    // before stressing MDD10A (30A/10s peak), while riding out brief MY1016 inrush spikes.
    if (g_acs_valid && fabsf(g_motor_current_a) > HW_OVERCURRENT_CUT_A && g_mode != MODE_IDLE) {
        if (g_oc_ticks < HW_OVERCURRENT_TICKS) g_oc_ticks++;
        if (g_oc_ticks >= HW_OVERCURRENT_TICKS) {
            stopAllImmediate();
            g_oc_fault_msg_pend = true;
        }
    } else if (g_oc_ticks > 0U) {
        g_oc_ticks = (g_oc_ticks >= 2U) ? (uint16_t)(g_oc_ticks - 2U) : 0U;
    }

    // CHOP_OFF overvoltage cutoff (>27.5V) protects 24V supply/caps while chopper is disconnected
    if (g_chop_mode == CHOP_OFF && g_vbus_valid && g_vbus_volts > VBUS_CHOP_OFF_OV_V && g_mode != MODE_IDLE) {
        stopAllImmediate();
        g_motion_msg_code = 5;
        g_motion_msg_val  = g_vbus_volts;
    }

    // Cool surge timer during non-current-controlled modes so thermal memory stays accurate
    if (!isCurrentControlledMode(g_mode)) {
        (void)updateSurgeFoldbackTarget(0.0f);
    }

    // Passive MY1016 + 52:1 gearbox torque and pad-force estimation
    float tau_em_gbx = GEAR_RATIO * GEAR_EFFICIENCY * KT_MOTOR_NM_PER_A * g_motor_current_a;
    g_gbx_torque_nm  = constrain(tau_em_gbx, -GBX_MAX_TORQUE_NM, GBX_MAX_TORQUE_NM);
    g_knee_torque_nm = (g_gamma_factor > 0.1f) ? (g_gbx_torque_nm / g_gamma_factor) : g_gbx_torque_nm;
    g_pad_force_n    = (g_L_leg_m > 0.05f) ? (g_knee_torque_nm / g_L_leg_m) : 0.0f;

    switch (g_mode) {
        case MODE_IDLE:
        default: {
            applyMotorOutput(0, 0);
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_DIRECT_PWM: {
            applyMotorOutput(g_cmd_motor_dir, g_cmd_motor_pct);
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_BENCH_PULSE: {
            uint32_t elapsed = millis() - g_pulse_start_ms;
            if (elapsed < 1000) {
                applyMotorOutput(1, 20); // HW FWD 1.0s
            } else if (elapsed < 1500) {
                applyMotorOutput(0, 0);  // Pause 0.5s
            } else if (elapsed < 2500) {
                applyMotorOutput(0, 20); // HW REV 1.0s
            } else {
                stopAllImmediate();
            }
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_CHOPPER_PULSE: {
            uint32_t elapsed = millis() - g_pulse_start_ms;
            if (elapsed < 1000) {
                applyMotorOutput(0, 0);
                applyChopperOutput(g_ctest_pct);
            } else {
                stopAllImmediate();
                g_ctest_done_msg = true;
            }
            break;
        }

        case MODE_DIRTEST: {
            uint32_t elapsed = millis() - g_pulse_start_ms;
            if (elapsed < 250) {
                // Phase 1: HW FWD (DIR = HIGH) @ 18%
                applyMotorOutput(1, 18);
                updateBrakeChopper(g_vbus_volts);
                if (elapsed >= 100 && g_acs_valid) {
                    float i_raw = (g_acs_raw_mv - g_acs_zero_mv) / ACS712_MV_PER_AMP;
                    g_dirtest_acs_sum += (double)i_raw;
                    g_dirtest_samples++;
                }
            } else if (elapsed < 450) {
                // Phase 2: Settle 200ms and capture Phase 1 results once
                applyMotorOutput(0, 0);
                updateBrakeChopper(g_vbus_volts);
                if (g_dirtest_phase == 0) {
                    g_dirtest_phase     = 1;
                    g_dirtest_fwd_dcnt  = g_enc_counts - g_dirtest_start_cnt;
                    g_dirtest_fwd_avg_i = (g_dirtest_samples > 0)
                                          ? (float)(g_dirtest_acs_sum / (double)g_dirtest_samples)
                                          : 0.0f;
                    g_dirtest_acs_sum   = 0.0;
                    g_dirtest_samples   = 0;
                    g_dirtest_mid_cnt   = g_enc_counts;
                }
            } else if (elapsed < 700) {
                if (g_dirtest_phase == 1) {
                    g_dirtest_phase   = 2;
                    g_dirtest_mid_cnt = g_enc_counts; // Fresh baseline right before REV starts
                }
                // Phase 3: HW REV (DIR = LOW) @ 18%
                applyMotorOutput(0, 18);
                updateBrakeChopper(g_vbus_volts);
                if (elapsed >= 550 && g_acs_valid) {
                    float i_raw = (g_acs_raw_mv - g_acs_zero_mv) / ACS712_MV_PER_AMP;
                    g_dirtest_acs_sum += (double)i_raw;
                    g_dirtest_samples++;
                }
            } else {
                stopAllImmediate();
                g_dirtest_rev_dcnt  = g_enc_counts - g_dirtest_mid_cnt;
                g_dirtest_rev_avg_i = (g_dirtest_samples > 0)
                                      ? (float)(g_dirtest_acs_sum / (double)g_dirtest_samples)
                                      : 0.0f;
                g_dirtest_dir_flip  = false;
                g_dirtest_acs_flip  = false;

                int64_t fwd_logical = g_dirtest_fwd_dcnt * (int64_t)g_enc_polarity;
                int64_t rev_logical = g_dirtest_rev_dcnt * (int64_t)g_enc_polarity;
                bool opp_dirs = (fwd_logical > 10 && rev_logical < -10) ||
                                (fwd_logical < -10 && rev_logical > 10);
                g_dirtest_bidir_ok = opp_dirs;

                if (opp_dirs) {
                    int8_t new_dp = (fwd_logical > 10) ? +1 : -1;
                    if (new_dp != g_dir_polarity) {
                        g_dir_polarity     = new_dp;
                        g_dirtest_dir_flip = true;
                    }
                    // Align ACS712 so positive (+Flexion) drive reads +Amps
                    float pos_raw_i = (g_dir_polarity == +1) ? g_dirtest_fwd_avg_i : g_dirtest_rev_avg_i;
                    if (pos_raw_i < -0.15f) {
                        g_dirtest_acs_flip = (g_acs_polarity != -1);
                        g_acs_polarity     = -1;
                    } else if (pos_raw_i > 0.15f) {
                        g_dirtest_acs_flip = (g_acs_polarity != +1);
                        g_acs_polarity     = +1;
                    }
                    if (g_dirtest_dir_flip || g_dirtest_acs_flip) {
                        g_nvs_save_pending = true;
                    }
                }
                g_dirtest_done_msg = true;
            }
            break;
        }

        case MODE_CAL_ZERO: {
            applyMotorOutput(0, 0);
            applyChopperOutput(0);
            uint32_t elapsed = millis() - g_pulse_start_ms;
            if (elapsed >= 150 && elapsed < 1150) {
                if (g_acs_raw_mv >= 300.0f && g_acs_raw_mv <= 2850.0f) {
                    g_cal_acs_sum += (double)g_acs_raw_mv;
                    g_cal_acs_cnt++;
                }
                if (g_vbus_valid) {
                    g_cal_vbus_sum += (double)g_vbus_raw_mv * 0.001 * (double)VBUS_DIV_MULT;
                    g_cal_vbus_cnt++;
                }
            } else if (elapsed >= 1150) {
                stopAllImmediate();
                g_cal_acs_ok  = (g_cal_acs_cnt >= 100);
                g_cal_vbus_ok = (g_cal_vbus_cnt >= 100);
                if (g_cal_acs_ok) {
                    g_acs_zero_mv     = (float)(g_cal_acs_sum / (double)g_cal_acs_cnt);
                    g_motor_current_a = 0.0f;
                    g_cal_valid       = true;
                }
                if (g_cal_vbus_ok) {
                    g_vbus_idle_v = (float)(g_cal_vbus_sum / (double)g_cal_vbus_cnt);
                }
                if (g_cal_acs_ok) {
                    g_nvs_save_pending = true; // Save in loop(), outside 500Hz tick
                }
                g_cal_done_msg = true;
            }
            break;
        }

        case MODE_CURRENT_REF: {
            executeBidirectionalCurrentControl(g_cmd_iref_a, g_omega_m_deg_s);
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_AUTOHOME: {
            uint32_t now_ms     = millis();
            uint32_t elapsed_ms = now_ms - g_home_start_ms;
            int      ext_dir    = (g_dir_polarity > 0) ? 0 : 1;
            int      flex_dir   = (g_dir_polarity > 0) ? 1 : 0;

            if (g_vbus_valid && g_vbus_volts < 12.0f) {
                stopAllImmediate();
                g_homing_msg_code = 3;
                updateBrakeChopper(g_vbus_volts);
                break;
            }

            if (g_home_phase == 1) {
                if (g_acs_valid && fabsf(g_motor_current_a) > 10.0f) {
                    stopAllImmediate();
                    g_homing_msg_code = 4;
                } else if ((g_enc_counts - g_home_start_cnt) > 24) {
                    // Moved > +6 deg toward +Flexion instead of -Extension
                    stopAllImmediate();
                    g_homing_msg_code = 2;
                } else if (elapsed_ms > 15000U) {
                    stopAllImmediate();
                    g_homing_msg_code = 5;
                } else {
                    int creep_pct = (g_acs_valid && fabsf(g_motor_current_a) > 3.5f) ? 12 : 20;
                    applyMotorOutput(ext_dir, creep_pct);

                    if ((uint32_t)(now_ms - g_home_phase_ms) >= 200U) {
                        int64_t step_dcnt = g_enc_counts - g_home_window_cnt;
                        g_home_window_cnt = g_enc_counts;
                        g_home_phase_ms   = now_ms;

                        if (elapsed_ms >= 350U) {
                            if (i64abs(step_dcnt) <= 1) {
                                // Distinguish open motor circuit (~0A, zero motion) from true STOP1 stall
                                if (elapsed_ms >= 800U &&
                                    i64abs(g_enc_counts - g_home_start_cnt) <= 2 &&
                                    (!g_acs_valid || fabsf(g_motor_current_a) < 0.35f)) {
                                    stopAllImmediate();
                                    g_homing_msg_code = 3;
                                } else {
                                    g_home_stall_hits++;
                                    if (g_home_stall_hits >= 2) {
                                        applyMotorOutput(0, 0);
                                        g_home_phase    = 2;
                                        g_home_phase_ms = now_ms;
                                    }
                                }
                            } else {
                                g_home_stall_hits = 0;
                            }
                        }
                    }
                }
            } else if (g_home_phase == 2) {
                applyMotorOutput(0, 0);
                if ((uint32_t)(now_ms - g_home_phase_ms) >= 250U) {
                    latchEncoderAngleDeg(mapKneeToArmAngleDeg(0.0f, g_d_offset_m, g_L_leg_m));
                    g_theta_k_deg   = 0.0f;
                    g_is_homed      = true;
                    g_home_park_deg = constrain(g_rom_min_deg, 3.0f, 15.0f);
                    g_home_phase    = 3;
                    g_home_phase_ms = now_ms;
                }
            } else if (g_home_phase == 3) {
                if (g_theta_k_deg >= g_home_park_deg) {
                    stopAllImmediate();
                    g_homing_msg_code = 1;
                } else if ((uint32_t)(now_ms - g_home_phase_ms) >= 1200U) {
                    stopAllImmediate();
                    g_homing_msg_code = 6;
                } else {
                    applyMotorOutput(flex_dir, 18);
                }
            } else {
                stopAllImmediate();
            }
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_GOTO: {
            float err_goal = g_goto_goal_tk - g_traj_target_tk;
            float max_step = g_goto_speed_degs * CONTROL_DT_SEC;
            float ref_wk   = 0.0f;
            if (fabsf(err_goal) <= max_step) {
                g_traj_target_tk = g_goto_goal_tk;
                ref_wk           = 0.0f;
            } else if (err_goal > 0.0f) {
                g_traj_target_tk += max_step;
                ref_wk            = g_goto_speed_degs;
            } else {
                g_traj_target_tk -= max_step;
                ref_wk            = -g_goto_speed_degs;
            }

            if (fabsf(g_traj_target_tk - g_theta_k_deg) > 15.0f) {
                if (g_obst_ticks < 1000) g_obst_ticks++;
                if (g_obst_ticks >= 750) { // 1.5s blocked
                    stopAllImmediate();
                    g_motion_msg_code = 2;
                    g_motion_msg_val  = g_theta_k_deg;
                    updateBrakeChopper(g_vbus_volts);
                    break;
                }
            } else {
                g_obst_ticks = 0;
            }

            if (!g_goto_reached_sent &&
                fabsf(g_traj_target_tk - g_goto_goal_tk) < 0.05f &&
                fabsf(g_theta_k_deg - g_goto_goal_tk) <= 1.5f) {
                if ((uint32_t)(millis() - g_goto_settle_ms) >= 250U) {
                    g_goto_reached_sent = true;
                    g_motion_msg_code   = 1;
                    g_motion_msg_val    = g_theta_k_deg;
                }
            } else {
                g_goto_settle_ms = millis();
            }

            float i_cmd = computePositionAssistCurrent(g_traj_target_tk, ref_wk, 0, g_assist_max_i_a);
            executeBidirectionalCurrentControl(i_cmd, g_omega_m_deg_s);
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_CPM: {
            float ref_wk = 0.0f;
            float step   = g_cpm_speed_degs * CONTROL_DT_SEC;
            uint32_t now_ms = millis();

            if (g_cpm_phase == 0) { // Flexion toward g_rom_max_deg
                g_traj_target_tk += step;
                ref_wk            = g_cpm_speed_degs;
                if (g_traj_target_tk >= g_rom_max_deg) {
                    g_traj_target_tk = g_rom_max_deg;
                    ref_wk           = 0.0f;
                    g_cpm_phase      = 1;
                    g_cpm_dwell_ms   = now_ms;
                }
            } else if (g_cpm_phase == 1) { // Dwell top
                g_traj_target_tk = g_rom_max_deg;
                if ((uint32_t)(now_ms - g_cpm_dwell_ms) >= 400U) {
                    g_cpm_phase = 2;
                }
            } else if (g_cpm_phase == 2) { // Extension toward g_rom_min_deg
                g_traj_target_tk -= step;
                ref_wk            = -g_cpm_speed_degs;
                if (g_traj_target_tk <= g_rom_min_deg) {
                    g_traj_target_tk = g_rom_min_deg;
                    ref_wk           = 0.0f;
                    g_cpm_phase      = 3;
                    g_cpm_dwell_ms   = now_ms;
                }
            } else { // Dwell bottom
                g_traj_target_tk = g_rom_min_deg;
                if ((uint32_t)(now_ms - g_cpm_dwell_ms) >= 400U) {
                    g_cpm_done_cycles++;
                    if (g_cpm_target_cycles > 0 && g_cpm_done_cycles >= g_cpm_target_cycles) {
                        stopAllImmediate();
                        g_motion_msg_code = 3;
                        g_motion_msg_val  = (float)g_cpm_done_cycles;
                        updateBrakeChopper(g_vbus_volts);
                        break;
                    }
                    g_cpm_phase = 0;
                }
            }

            if (fabsf(g_traj_target_tk - g_theta_k_deg) > 18.0f) {
                if (g_obst_ticks < 1000) g_obst_ticks++;
                if (g_obst_ticks >= 500) { // 1.0s obstruction/spasm cutoff
                    stopAllImmediate();
                    g_motion_msg_code = 4;
                    g_motion_msg_val  = g_theta_k_deg;
                    updateBrakeChopper(g_vbus_volts);
                    break;
                }
            } else {
                g_obst_ticks = 0;
            }

            float i_cmd = computePositionAssistCurrent(g_traj_target_tk, ref_wk, 0, g_assist_max_i_a);
            executeBidirectionalCurrentControl(i_cmd, g_omega_m_deg_s);
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            updateBrakeChopper(g_vbus_volts);
            break;
        }

        case MODE_ACTIVE_ASSIST: {
            float    i_cmd  = 0.0f;
            float    step   = g_assist_speed_degs * CONTROL_DT_SEC;
            uint32_t now_ms = millis();

            if (g_assist_phase == 0) { // WAIT_FLEX (patient-triggered)
                if (g_theta_k_deg < g_assist_anchor_tk && g_theta_k_deg >= (g_rom_min_deg - 2.0f)) {
                    g_assist_anchor_tk = g_theta_k_deg;
                }
                float disp = g_theta_k_deg - g_assist_anchor_tk;
                bool  trig = (disp >= ASSIST_NUDGE_TRIG_DEG) ||
                             (g_omega_k_deg_s >= ASSIST_VEL_TRIG_DEGS && disp >= 0.40f);
                if (trig) {
                    g_traj_target_tk = constrain(g_theta_k_deg, g_rom_min_deg, g_rom_max_deg);
                    g_assist_phase   = 1;
                    g_obst_ticks     = 0;
                    g_assist_msg_code = 1;
                    g_assist_msg_val  = g_theta_k_deg;
                }
            } else if (g_assist_phase == 1) { // ASSIST_FLEX (yield-forward)
                g_traj_target_tk += step;
                // Yield forward immediately if patient moves faster than trajectory
                if (g_theta_k_deg > g_traj_target_tk) {
                    g_traj_target_tk = g_theta_k_deg;
                }
                if (g_traj_target_tk > g_rom_max_deg) g_traj_target_tk = g_rom_max_deg;

                if ((g_traj_target_tk - g_theta_k_deg) > ASSIST_RESIST_ERR_DEG) {
                    if (g_obst_ticks < 1000) g_obst_ticks++;
                    if (g_obst_ticks >= 200) { // 400 ms patient resistance -> yield & pause
                        g_assist_phase     = 0;
                        g_assist_anchor_tk = g_theta_k_deg;
                        g_obst_ticks       = 0;
                        g_assist_msg_code  = 3;
                        g_assist_msg_val   = g_theta_k_deg;
                    }
                } else {
                    g_obst_ticks = 0;
                }

                if (g_assist_phase == 1) {
                    if (g_theta_k_deg >= (g_rom_max_deg - 1.5f)) {
                        g_assist_phase    = 2;
                        g_assist_hold_ms  = now_ms;
                        g_obst_ticks      = 0;
                        g_assist_msg_code = 5;
                        g_assist_msg_val  = g_theta_k_deg;
                    } else {
                        i_cmd = computePositionAssistCurrent(g_traj_target_tk, g_assist_speed_degs,
                                                             +1, g_assist_max_i_a);
                    }
                }
            } else if (g_assist_phase == 2) { // HOLD_TOP
                if (g_assist_auto_cycle) {
                    i_cmd = computePositionAssistCurrent(g_rom_max_deg, 0.0f, 0, 0.5f * g_assist_max_i_a);
                }
                if ((uint32_t)(now_ms - g_assist_hold_ms) >= 400U) {
                    g_assist_anchor_tk = g_theta_k_deg;
                    if (g_assist_auto_cycle) {
                        g_traj_target_tk = g_rom_max_deg;
                        g_assist_phase   = 4;
                    } else {
                        g_assist_phase = 3;
                    }
                }
            } else if (g_assist_phase == 3) { // WAIT_EXT (patient-triggered)
                if (g_theta_k_deg > g_assist_anchor_tk && g_theta_k_deg <= (g_rom_max_deg + 2.0f)) {
                    g_assist_anchor_tk = g_theta_k_deg;
                }
                float disp = g_assist_anchor_tk - g_theta_k_deg;
                bool  trig = (disp >= ASSIST_NUDGE_TRIG_DEG) ||
                             (g_omega_k_deg_s <= -ASSIST_VEL_TRIG_DEGS && disp >= 0.40f);
                if (trig) {
                    g_traj_target_tk  = constrain(g_theta_k_deg, g_rom_min_deg, g_rom_max_deg);
                    g_assist_phase    = 4;
                    g_obst_ticks      = 0;
                    g_assist_msg_code = 2;
                    g_assist_msg_val  = g_theta_k_deg;
                }
            } else if (g_assist_phase == 4) { // ASSIST_EXT (yield-forward)
                g_traj_target_tk -= step;
                if (g_theta_k_deg < g_traj_target_tk) {
                    g_traj_target_tk = g_theta_k_deg;
                }
                if (g_traj_target_tk < g_rom_min_deg) g_traj_target_tk = g_rom_min_deg;

                if ((g_theta_k_deg - g_traj_target_tk) > ASSIST_RESIST_ERR_DEG) {
                    if (g_obst_ticks < 1000) g_obst_ticks++;
                    if (g_obst_ticks >= 200) {
                        g_assist_phase     = 3;
                        g_assist_anchor_tk = g_theta_k_deg;
                        g_obst_ticks       = 0;
                        g_assist_msg_code  = 4;
                        g_assist_msg_val   = g_theta_k_deg;
                    }
                } else {
                    g_obst_ticks = 0;
                }

                if (g_assist_phase == 4) {
                    if (g_theta_k_deg <= (g_rom_min_deg + 1.5f)) {
                        g_assist_phase    = 5;
                        g_assist_hold_ms  = now_ms;
                        g_obst_ticks      = 0;
                        g_assist_msg_code = 6;
                        g_assist_msg_val  = g_theta_k_deg;
                    } else {
                        i_cmd = computePositionAssistCurrent(g_traj_target_tk, -g_assist_speed_degs,
                                                             -1, g_assist_max_i_a);
                    }
                }
            } else { // HOLD_BOT
                if (g_assist_auto_cycle) {
                    i_cmd = computePositionAssistCurrent(g_rom_min_deg, 0.0f, 0, 0.5f * g_assist_max_i_a);
                }
                if ((uint32_t)(now_ms - g_assist_hold_ms) >= 400U) {
                    g_assist_anchor_tk = g_theta_k_deg;
                    if (g_assist_auto_cycle) {
                        g_traj_target_tk = g_rom_min_deg;
                        g_assist_phase   = 1;
                    } else {
                        g_assist_phase = 0;
                    }
                }
            }

            executeBidirectionalCurrentControl(i_cmd, g_omega_m_deg_s);
            updateRepTracker(g_theta_k_deg, g_omega_k_deg_s, g_knee_torque_nm);
            updateBrakeChopper(g_vbus_volts);
            break;
        }
    }

    uint32_t dt_us = micros() - t_start;
    if (dt_us > g_loop_us_win_max) g_loop_us_win_max = dt_us;
    if (dt_us > g_loop_us_peak)    g_loop_us_peak    = dt_us;
}

// ----------------------------------------------------------------------------
// 9. SERIAL COMMAND INTERFACE
// ----------------------------------------------------------------------------
static void printHelp() {
    Serial.printf("\n=== U2 (ESP_TOPHUB) STAGED FIRMWARE — STAGE %d (Cytron MDD10A) ===\n", FW_STAGE);
    Serial.printf("ENC1: %.0f CPR (%.4f deg/cnt) | Geom: d_off=%.3fm, L_leg=%.3fm | Pol: dir=%+d enc=%+d acs=%+d | Mode: %s @ %lu Hz\n",
                  ENC_CPR_OUTPUT, DEG_PER_COUNT, g_d_offset_m, g_L_leg_m,
                  g_dir_polarity, g_enc_polarity, g_acs_polarity,
                  MOTOR_BIDIRECTIONAL_ENABLED ? "BIDIR" : "REV-ONLY",
                  (unsigned long)g_md_pwm_freq_hz);
    Serial.println("Commands:");
    Serial.println("  autohome            : Self-home against STOP1 (0 deg) & park at ROM min -> H:1");
    Serial.println("  cpm [spd] [cycles]  : Continuous Passive Motion within ROM (default 18 deg/s)");
    Serial.println("  assist [auto] [spd] : Active Assist (wait patient nudge, yield-forward; or 'assist auto')");
    Serial.println("  goto <deg> [spd]    : Closed-loop move & hold at knee deg (0..120 deg)");
    Serial.println("  rom [min max|reset] : Show or set soft ROM limits (deg, default 5..90) -> NVS");
    Serial.println("  assistcfg [kp kd Imax [grav]] : Show/tune knee PD, Imax & gravity FF (or 'reset')");
    Serial.printf("  iref <%.0f..%.0f>     : Closed-loop signed current (A; >%.0fA <%.0fs surge); 'iref 0' -> IDLE\n",
                  -PEAK_CURRENT_LIMIT_A, PEAK_CURRENT_LIMIT_A, CONT_CURRENT_LIMIT_A, PEAK_SURGE_MAX_S);
    Serial.println("  rloop [ohm]         : Show or tune R_loop feedforward resistance (default 0.40)");
    Serial.println("  pi [kp ki]          : Show or tune current loop Kp/Ki gains (0.25 / 8.0)");
    Serial.println("  f/r <pct> | pwm <%> : Open-loop HW FWD/REV (0..100%) or signed pwm (-100..100%)");
    Serial.println("  s / stop            : STOP motor (IDLE / MDD10A low-side short brake)");
    Serial.println("  adc                 : Print raw ADC, mV, sanity flags, torque & pad force");
    Serial.println("  enc [reset]         : Show GPIO32(A)/GPIO33(B) levels, edge counts & raw PCNT");
    Serial.println("  cal                 : Non-blocking 1s static ACS712 & VBUS calibration -> NVS");
    Serial.println("  geom [d] [L]        : Show or set d_offset (m) and L_leg (m) -> NVS");
    Serial.println("  nvs / nvs_clear     : Inspect or erase saved NVS calibration");
    Serial.println("  home [deg]          : Latch current arm angle as knee deg (default 0 at STOP1)");
    Serial.println("  dirtest             : 2-phase FWD/REV 18% pulse to align MDD10A DIR, ENC1 & ACS712");
    Serial.println("  pol dir|enc|acs <1|-1> : Set DIR, encoder, or ACS712 polarity (+1 or -1) -> NVS");
    Serial.printf("  freq <100..%lu>   : Change motor PWM frequency live (MDD10A max 20 kHz)\n",
                  (unsigned long)MAX_MD_PWM_FREQ_HZ);
    Serial.println("  chop [auto|off|<%>] : Auto VBUS clamp, OFF, or manual 0..50% duty (alias 'c <%>)");
    Serial.println("  chop dry <volts>    : Compute auto-chopper duty at simulated VBUS (no output)");
    Serial.println("  chop thr [on full]  : Set/reset auto-chopper VBUS thresholds ('chop thr auto')");
    Serial.println("  ctest [5..50]       : 1s chopper pulse (default 10%) for U202 LED/load check");
    Serial.println("  test                : Non-blocking 1s HW FWD + 0.5s pause + 1s HW REV @ 20%");
    Serial.println("  z                   : Zero raw ENC1 count (Tm=0) & re-zero RAM ACS712");
    Serial.println("  p / status / h      : Toggle live print / single snapshot / help\n");
}

static bool parseOptionalInt(const char *arg, int default_val, int *out_val) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg == '\0') {
        *out_val = default_val;
        return true;
    }
    char *endptr = nullptr;
    long val = strtol(arg, &endptr, 10);
    if (endptr == arg) return false;
    *out_val = (int)val;
    return true;
}

static bool parseOptionalFloat(const char *arg, float default_val, float *out_val) {
    while (*arg == ' ' || *arg == '\t') arg++;
    if (*arg == '\0') {
        *out_val = default_val;
        return true;
    }
    char *endptr = nullptr;
    float val = strtof(arg, &endptr);
    if (endptr == arg) return false;
    *out_val = val;
    return true;
}

static void setDirectMotorCmd(int dir, int pct) {
    pct = constrain(pct, 0, MAX_MOTOR_PWM_PCT);
    if (!MOTOR_BIDIRECTIONAL_ENABLED) {
        dir = 0;
    }
    g_cmd_motor_dir = (dir != 0) ? 1 : 0;
    g_cmd_motor_pct = pct;
    g_mode          = (pct == 0) ? MODE_IDLE : MODE_DIRECT_PWM;
    applyMotorOutput(g_cmd_motor_dir, g_cmd_motor_pct);
    Serial.printf("[CMD] Motor %s PWM at %d%%\n",
                  g_cmd_motor_dir ? "HW FWD (DIR=HIGH)" : "HW REV (DIR=LOW)", g_cmd_motor_pct);
}

static void handleCommand(char *line) {
    while (*line == ' ' || *line == '\t') line++;
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == ' ' || line[len - 1] == '\t')) {
        line[--len] = '\0';
    }
    if (*line == '\0') return;

    if (strcmp(line, "s") == 0 || strcmp(line, "stop") == 0) {
        stopAllImmediate();
        Serial.println("[CMD] STOP -> MODE_IDLE (Motor 0%, Chopper 0%)");
    } else if (strcmp(line, "h") == 0 || strcmp(line, "?") == 0) {
        printHelp();
    } else if (strcmp(line, "p") == 0) {
        g_live_print = !g_live_print;
        Serial.printf("[CMD] Live telemetry print: %s\n", g_live_print ? "ON" : "OFF");
    } else if (strcmp(line, "status") == 0) {
        printTelemetry();
    } else if (strcmp(line, "adc") == 0) {
        printAdcDiagnostics();
    } else if (strcmp(line, "enc") == 0) {
        printEncDiagnostics();
    } else if (strcmp(line, "enc reset") == 0) {
        g_enc_a_edges = 0;
        g_enc_b_edges = 0;
        Serial.println("[ENC] Edge counters reset.");
        printEncDiagnostics();
    } else if (strcmp(line, "nvs") == 0) {
        printNvsStatus();
    } else if (strcmp(line, "nvs_clear") == 0) {
        stopAllImmediate();
        clearCalibrationNVS();
        Serial.println("[NVS] Cleared namespace and restored Stage 3 defaults (d_off=0.250m, L_leg=0.300m).");
        printNvsStatus();
    } else if (strcmp(line, "cal") == 0) {
        stopAllImmediate();
        g_cal_acs_sum    = 0.0;
        g_cal_vbus_sum   = 0.0;
        g_cal_acs_cnt    = 0;
        g_cal_vbus_cnt   = 0;
        g_pulse_start_ms = millis();
        g_mode           = MODE_CAL_ZERO;
        Serial.println("[CAL] Keep motor idle (150ms settle + 1.0s sampling; send any key to abort)...");
    } else if (strncmp(line, "geom", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        const char *arg = &line[4];
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            Serial.printf("[GEOM] d_off=%.3f m | L_leg=%.3f m | ratio=%.3f | Tm(Tk=0)=%.2f deg\n",
                          g_d_offset_m, g_L_leg_m, g_d_offset_m / g_L_leg_m,
                          mapKneeToArmAngleDeg(0.0f, g_d_offset_m, g_L_leg_m));
            return;
        }
        char *end1 = nullptr;
        float new_d = strtof(arg, &end1);
        if (end1 == arg) {
            Serial.println("[CMD] Usage: geom [d_off_m] [L_leg_m]  (e.g. 'geom 0.25 0.30')");
            return;
        }
        float new_L = g_L_leg_m;
        const char *arg2 = end1;
        while (*arg2 == ' ' || *arg2 == '\t') arg2++;
        if (*arg2 != '\0') {
            char *end2 = nullptr;
            new_L = strtof(arg2, &end2);
            if (end2 == arg2) {
                Serial.println("[CMD] Usage: geom [d_off_m] [L_leg_m]  (e.g. 'geom 0.25 0.30')");
                return;
            }
        }
        if (new_L < 0.15f || new_L > 0.60f || new_d < 0.0f || new_d > 0.95f * new_L) {
            Serial.printf("[GEOM] Out of safe range (need L in [0.15..0.60]m and 0 <= d <= 0.95*L; got d=%.3f, L=%.3f)\n",
                          new_d, new_L);
            return;
        }
        g_d_offset_m   = new_d;
        g_L_leg_m      = new_L;
        g_theta_k_deg  = mapArmToKneeAngleDeg(g_theta_m_deg, g_d_offset_m, g_L_leg_m);
        g_gamma_factor = computeGammaFactor(g_theta_k_deg, g_d_offset_m, g_L_leg_m);
        g_is_homed     = false;
        saveCalibrationToNVS();
        Serial.printf("[GEOM] Saved: d_off=%.3f m | L_leg=%.3f m | Tm(Tk=0)=%.2f deg (re-run 'home [deg]')\n",
                      g_d_offset_m, g_L_leg_m, mapKneeToArmAngleDeg(0.0f, g_d_offset_m, g_L_leg_m));
    } else if (strcmp(line, "z") == 0) {
        zeroEncoder();
        if (g_motor_pwm_pct == 0 && g_chop_pwm_pct == 0) {
            zeroACS712BootFallback();
            g_motor_current_a = 0.0f;
            Serial.printf("[CMD] Zeroed ENC1 (Tm=0, Tk=%.2f) and RAM ACS712 (0A=%.1f mV; run 'cal' to persist)\n",
                          g_theta_k_deg, g_acs_zero_mv);
        } else {
            Serial.println("[CMD] Zeroed ENC1 (stop motor & chopper first to re-zero ACS712)");
        }
    } else if (strncmp(line, "home", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        float known_tk = 0.0f;
        if (!parseOptionalFloat(&line[4], 0.0f, &known_tk)) {
            Serial.println("[CMD] Usage: home [0..125]");
            return;
        }
        known_tk = constrain(known_tk, 0.0f, 125.0f);
        float known_tm = mapKneeToArmAngleDeg(known_tk, g_d_offset_m, g_L_leg_m);
        latchEncoderAngleDeg(known_tm);
        g_theta_k_deg = known_tk;
        g_is_homed    = true;
        Serial.printf("[CMD] Homed: Knee=%.2f deg | Arm=%.2f deg | ENC=%lld (sub-cnt=%.3f deg) | Gamma=%.3f\n",
                      g_theta_k_deg, g_theta_m_deg, (long long)g_enc_counts, g_tm_zero_offset_deg, g_gamma_factor);
    } else if (strcmp(line, "dirtest") == 0) {
        stopAllImmediate();
        g_dirtest_start_cnt = g_enc_counts;
        g_dirtest_mid_cnt   = g_enc_counts;
        g_dirtest_phase     = 0;
        g_dirtest_acs_sum   = 0.0;
        g_dirtest_samples   = 0;
        g_dirtest_bidir_ok  = false;
        g_pulse_start_ms    = millis();
        g_mode              = MODE_DIRTEST;
        Serial.println("[DIRTEST] 2-phase FWD/REV 18% pulse (250ms FWD + 200ms settle + 250ms REV, send any key to abort)...");
    } else if (strncmp(line, "pol ", 4) == 0) {
        const char *sub = &line[4];
        while (*sub == ' ' || *sub == '\t') sub++;
        int p = 0;
        if (strncmp(sub, "dir", 3) == 0 && (sub[3] == ' ' || sub[3] == '\t') &&
            parseOptionalInt(&sub[3], 0, &p) && (p == 1 || p == -1)) {
            g_dir_polarity = (int8_t)p;
            saveCalibrationToNVS();
            Serial.printf("[CMD] dir_polarity set to %+d (DIR=HIGH -> %s angle) and saved to NVS\n",
                          g_dir_polarity, g_dir_polarity > 0 ? "+Flexion" : "-Extension");
        } else if (strncmp(sub, "enc", 3) == 0 && (sub[3] == ' ' || sub[3] == '\t') &&
                   parseOptionalInt(&sub[3], 0, &p) && (p == 1 || p == -1)) {
            g_enc_polarity = (int8_t)p;
            resetVelocityWindow(g_theta_m_deg, g_theta_k_deg);
            saveCalibrationToNVS();
            Serial.printf("[CMD] enc_polarity set to %+d and saved to NVS (re-run 'home [deg]' if needed)\n",
                          g_enc_polarity);
        } else if (strncmp(sub, "acs", 3) == 0 && (sub[3] == ' ' || sub[3] == '\t') &&
                   parseOptionalInt(&sub[3], 0, &p) && (p == 1 || p == -1)) {
            g_acs_polarity = (int8_t)p;
            saveCalibrationToNVS();
            Serial.printf("[CMD] acs_polarity set to %+d and saved to NVS\n", g_acs_polarity);
        } else {
            Serial.println("[CMD] Usage: pol dir|enc|acs <1|-1>");
        }
    } else if (strcmp(line, "test") == 0) {
        stopAllImmediate();
        g_mode             = MODE_BENCH_PULSE;
        g_pulse_start_ms   = millis();
        g_bench_stop_phase = false;
        Serial.println("[TEST] 1/2 HW FWD (DIR=HIGH) 20% 1.0s -> 0.5s pause -> 2/2 HW REV 20% 1.0s (send any key to abort)");
    } else if (strncmp(line, "ctest", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        int pct = 10;
        if (!parseOptionalInt(&line[5], 10, &pct) || pct < 1 || pct > MAX_CHOP_MANUAL_PCT) {
            Serial.printf("[CMD] Usage: ctest [1..%d] (default 10%%)\n", MAX_CHOP_MANUAL_PCT);
            return;
        }
        stopAllImmediate();
        g_ctest_pct = pct;
        Serial.printf("[CTEST] Baseline VBUS: %.2f V (%s) | Pulsing %d%% for 1.0s (send any key to abort)...\n",
                      g_vbus_volts, g_vbus_valid ? "OK" : "NOM24V", g_ctest_pct);
        g_mode           = MODE_CHOPPER_PULSE;
        g_pulse_start_ms = millis();
    } else if (strncmp(line, "chop", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        const char *arg = &line[4];
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            printChopperStatus();
        } else if (strcmp(arg, "auto") == 0) {
            g_chop_mode    = CHOP_AUTO;
            g_cmd_chop_pct = 0;
            updateBrakeChopper(g_vbus_volts);
            printChopperStatus();
        } else if (strcmp(arg, "off") == 0) {
            g_chop_mode    = CHOP_OFF;
            g_cmd_chop_pct = 0;
            updateBrakeChopper(g_vbus_volts);
            printChopperStatus();
        } else if (strncmp(arg, "dry ", 4) == 0) {
            float sim_v = 0.0f;
            if (!parseOptionalFloat(&arg[4], -1.0f, &sim_v) || sim_v < 0.0f || sim_v > 40.0f) {
                Serial.println("[CMD] Usage: chop dry <volts> (e.g. 'chop dry 26.0')");
                return;
            }
            float v_on = 0.0f, v_max = 0.0f;
            getChopperThresholds(&v_on, &v_max);
            float sim_duty = computeChopperDutyForVbus(sim_v, v_on, v_max);
            Serial.printf("[CHOP DRY] If VBUS=%.2f V (on=%.2f V, full=%.2f V) -> Auto Duty = %.1f%%\n",
                          sim_v, v_on, v_max, sim_duty);
        } else if (strncmp(arg, "thr", 3) == 0 && (arg[3] == ' ' || arg[3] == '\0')) {
            const char *targ = &arg[3];
            while (*targ == ' ' || *targ == '\t') targ++;
            if (*targ == '\0') {
                printChopperStatus();
            } else if (strcmp(targ, "auto") == 0) {
                g_chop_thr_custom = false;
                printChopperStatus();
            } else {
                char *end1 = nullptr;
                float new_on = strtof(targ, &end1);
                if (end1 == targ) {
                    Serial.println("[CMD] Usage: chop thr [auto | <v_on> [v_max]]");
                    return;
                }
                float new_full = new_on + VBUS_CHOP_RAMP_SPAN_V;
                const char *targ2 = end1;
                while (*targ2 == ' ' || *targ2 == '\t') targ2++;
                if (*targ2 != '\0') {
                    char *end2 = nullptr;
                    new_full = strtof(targ2, &end2);
                    if (end2 == targ2) {
                        Serial.println("[CMD] Usage: chop thr [auto | <v_on> [v_max]]");
                        return;
                    }
                }
                if (new_on < 20.0f || new_full > VBUS_CHOP_ABS_MAX_V || new_full < new_on + 0.50f) {
                    Serial.printf("[CHOP] Invalid thresholds (need 20.0 <= on, full <= %.2f, span >= 0.50 V)\n",
                                  VBUS_CHOP_ABS_MAX_V);
                    return;
                }
                g_chop_thr_custom   = true;
                g_chop_v_on_custom  = new_on;
                g_chop_v_max_custom = new_full;
                printChopperStatus();
            }
        } else {
            int pct = -1;
            if (!parseOptionalInt(arg, -1, &pct) || pct < 0 || pct > MAX_CHOP_MANUAL_PCT) {
                Serial.printf("[CMD] Usage: chop [auto | off | 0..%d | dry <V> | thr ...]\n",
                              MAX_CHOP_MANUAL_PCT);
                return;
            }
            g_chop_mode    = CHOP_MANUAL;
            g_cmd_chop_pct = pct;
            applyChopperOutput(g_cmd_chop_pct);
            Serial.printf("[CMD] Chopper MANUAL %d%% (use 'chop auto' for auto VBUS clamp)\n",
                          g_cmd_chop_pct);
        }
    } else if (strncmp(line, "freq", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        int new_freq = 0;
        if (!parseOptionalInt(&line[4], 0, &new_freq) || new_freq < 100 || (uint32_t)new_freq > MAX_MD_PWM_FREQ_HZ) {
            Serial.printf("[CMD] Usage: freq <100..%lu> (current=%lu Hz, MDD10A max 20 kHz)\n",
                          (unsigned long)MAX_MD_PWM_FREQ_HZ, (unsigned long)g_md_pwm_freq_hz);
            return;
        }
        g_md_pwm_freq_hz = (uint32_t)new_freq;
        configureMotorPWMChannel();
        if (g_mode == MODE_DIRECT_PWM) {
            applyMotorOutput(g_cmd_motor_dir, g_cmd_motor_pct);
        }
        Serial.printf("[CMD] Motor PWM frequency set to %lu Hz (Chopper remains %lu Hz)\n",
                      (unsigned long)g_md_pwm_freq_hz, (unsigned long)CHOP_PWM_FREQ_HZ);
    } else if (strncmp(line, "iref", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        float amps = 999.0f;
        if (!parseOptionalFloat(&line[4], 999.0f, &amps) ||
            amps < -PEAK_CURRENT_LIMIT_A || amps > PEAK_CURRENT_LIMIT_A) {
            Serial.printf("[CMD] Usage: iref <%.1f..%.1f>  (active=%s ref=%+.2f A, meas=%+.2f A, Vff=%+.2f V, Vcmd=%+.2f V, surge=%.1f/%.1fs)\n",
                          -PEAK_CURRENT_LIMIT_A, PEAK_CURRENT_LIMIT_A,
                          (g_mode == MODE_CURRENT_REF) ? "ON" : "OFF",
                          g_cmd_iref_a, g_motor_current_a, g_ctrl_v_ff, g_ctrl_v_cmd,
                          g_surge_budget_s, PEAK_SURGE_MAX_S);
            return;
        }
        if (fabsf(amps) < 0.08f) {
            stopAllImmediate();
            Serial.println("[CMD] iref -> 0 A (MODE_IDLE)");
            return;
        }
        if (!g_acs_valid) {
            Serial.println("[WARN] ACS712 not valid ('OL'): running open-loop feedforward only until ACS is valid.");
        }
        if (g_mode != MODE_CURRENT_REF || (amps * g_cmd_iref_a) < 0.0f) {
            g_pi_integral_v     = 0.0f;
            g_acs_rev_neg_cnt   = 0;
            g_acs_pol_warn_sent = false;
        }
        g_cmd_iref_a    = amps;
        g_cmd_motor_pct = 0;
        g_mode          = MODE_CURRENT_REF;
        Serial.printf("[CMD] Closed-loop %s current target: %+.2f A (tau_gbx~%+.1f Nm%s | 's' to stop)\n",
                      (g_cmd_iref_a >= 0.0f) ? "+FLEX" : "-EXT",
                      g_cmd_iref_a,
                      GEAR_RATIO * GEAR_EFFICIENCY * KT_MOTOR_NM_PER_A * g_cmd_iref_a,
                      (fabsf(g_cmd_iref_a) > CONT_CURRENT_LIMIT_A) ? ", >10A 5s surge window" : "");
    } else if (strncmp(line, "rloop", 5) == 0 && (line[5] == ' ' || line[5] == '\0')) {
        float r_new = -1.0f;
        if (!parseOptionalFloat(&line[5], -1.0f, &r_new)) {
            Serial.printf("[LOOP] R_loop=%.2f Ohm | V_brush=%.2f V | Vff=%.2f V | Vcmd=%.2f V | Iint=%.2f V\n",
                          g_r_loop_ohm, g_v_brush_v, g_ctrl_v_ff, g_ctrl_v_cmd, g_pi_integral_v);
            return;
        }
        if (r_new < 0.10f || r_new > 3.00f) {
            Serial.println("[CMD] Usage: rloop <0.10..3.00> Ohm");
            return;
        }
        g_r_loop_ohm = r_new;
        Serial.printf("[LOOP] R_loop set to %.2f Ohm\n", g_r_loop_ohm);
    } else if (strncmp(line, "pi", 2) == 0 && (line[2] == ' ' || line[2] == '\0')) {
        const char *arg = &line[2];
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            Serial.printf("[PI] Kp=%.3f V/A | Ki=%.3f V/(A*s) | Integral=%.2f V | Vff=%.2f V | Vcmd=%.2f V\n",
                          g_kp_i, g_ki_i, g_pi_integral_v, g_ctrl_v_ff, g_ctrl_v_cmd);
            return;
        }
        char *end1 = nullptr;
        float kp_new = strtof(arg, &end1);
        if (end1 == arg) {
            Serial.println("[CMD] Usage: pi [kp ki]");
            return;
        }
        float ki_new = g_ki_i;
        const char *arg2 = end1;
        while (*arg2 == ' ' || *arg2 == '\t') arg2++;
        if (*arg2 != '\0') {
            char *end2 = nullptr;
            ki_new = strtof(arg2, &end2);
            if (end2 == arg2) {
                Serial.println("[CMD] Usage: pi [kp ki]");
                return;
            }
        }
        if (kp_new < 0.0f || kp_new > 5.0f || ki_new < 0.0f || ki_new > 50.0f) {
            Serial.println("[CMD] Bounds: Kp in [0..5.0], Ki in [0..50.0]");
            return;
        }
        g_kp_i = kp_new;
        g_ki_i = ki_new;
        g_pi_integral_v = 0.0f;
        Serial.printf("[PI] Updated gains: Kp=%.3f, Ki=%.3f (integral reset)\n", g_kp_i, g_ki_i);
    } else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\0')) {
        if (!MOTOR_BIDIRECTIONAL_ENABLED) {
            Serial.println("[WARN] Hardware DIR (GPIO26) is locked in REV. 'f' is disabled; use 'r <pct>' or 'pwm <pct>'.");
        } else {
            int pct = 0;
            if (!parseOptionalInt(&line[1], 20, &pct)) {
                Serial.println("[CMD] Usage: f <0..100>");
                return;
            }
            if (pct < 0) pct = 0;
            setDirectMotorCmd(1, pct);
        }
    } else if (line[0] == 'r' && (line[1] == ' ' || line[1] == '\0')) {
        int pct = 0;
        if (!parseOptionalInt(&line[1], 20, &pct)) {
            Serial.println("[CMD] Usage: r <0..100>");
            return;
        }
        if (pct < 0) pct = 0;
        setDirectMotorCmd(0, pct);
    } else if (strncmp(line, "pwm", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        int pct = 0;
        if (!parseOptionalInt(&line[3], 0, &pct)) {
            Serial.printf("[CMD] Usage: pwm <%s>\n", MOTOR_BIDIRECTIONAL_ENABLED ? "-100..100" : "0..100");
            return;
        }
        if (!MOTOR_BIDIRECTIONAL_ENABLED) {
            if (pct < 0) {
                Serial.println("[WARN] Negative (FWD) PWM rejected: hardware DIR is REV-only. Use 0..100.");
                return;
            }
            setDirectMotorCmd(0, pct);
        } else if (pct > 0) {
            setDirectMotorCmd(1, pct);
        } else if (pct < 0) {
            setDirectMotorCmd(0, -pct);
        } else {
            setDirectMotorCmd(0, 0);
        }
    } else if (line[0] == 'c' && (line[1] == ' ' || line[1] == '\0')) {
        int pct = 0;
        if (!parseOptionalInt(&line[1], 0, &pct) || pct < 0 || pct > MAX_CHOP_MANUAL_PCT) {
            Serial.printf("[CMD] Usage: c <0..%d>\n", MAX_CHOP_MANUAL_PCT);
            return;
        }
        g_chop_mode    = CHOP_MANUAL;
        g_cmd_chop_pct = pct;
        applyChopperOutput(g_cmd_chop_pct);
        Serial.printf("[CMD] Chopper MANUAL %d%% (use 'chop auto' for auto VBUS clamp)\n",
                      g_cmd_chop_pct);
    } else if (strcmp(line, "autohome") == 0) {
        if (!MOTOR_BIDIRECTIONAL_ENABLED) {
            Serial.println("[AUTOHOME] Blocked: bidirectional driver required.");
            return;
        }
        if (g_vbus_valid && g_vbus_volts < 15.0f) {
            Serial.printf("[AUTOHOME] Aborted: VBUS=%.2f V (<15V) — power on 24V supply first.\n",
                          g_vbus_volts);
            return;
        }
        if (!g_acs_valid) {
            Serial.println("[AUTOHOME] Aborted: ACS712 invalid (OL) — cannot current-guard stop detection.");
            return;
        }
        stopAllImmediate();
        g_home_phase      = 1;
        g_home_start_ms   = millis();
        g_home_phase_ms   = g_home_start_ms;
        g_home_start_cnt  = g_enc_counts;
        g_home_window_cnt = g_enc_counts;
        g_home_stall_hits = 0;
        g_homing_msg_code = 0;
        g_mode            = MODE_AUTOHOME;
        Serial.println("[AUTOHOME] Seeking STOP1 (-Extension @ 20% PWM, stall & current guarded; send any key to abort)...");
    } else if (strncmp(line, "rom", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        const char *arg = &line[3];
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            Serial.printf("[ROM] Soft limits: min=%.1f deg | max=%.1f deg | current Tk=%.2f deg (homed=%d)\n",
                          g_rom_min_deg, g_rom_max_deg, g_theta_k_deg, g_is_homed ? 1 : 0);
        } else if (strcmp(arg, "reset") == 0) {
            g_rom_min_deg = DEFAULT_ROM_MIN_DEG;
            g_rom_max_deg = DEFAULT_ROM_MAX_DEG;
            saveCalibrationToNVS();
            Serial.printf("[ROM] Reset to defaults (%.1f .. %.1f deg) and saved to NVS.\n",
                          g_rom_min_deg, g_rom_max_deg);
        } else {
            char *p1 = nullptr;
            char *p2 = nullptr;
            float rmin = strtof(arg, &p1);
            float rmax = (p1 != arg) ? strtof(p1, &p2) : 0.0f;
            if (p1 == arg || p2 == p1 || !isfinite(rmin) || !isfinite(rmax) ||
                rmin < ROM_ABS_MIN_DEG || rmax > ROM_ABS_MAX_DEG || (rmax - rmin) < ROM_MIN_SPAN_DEG) {
                Serial.printf("[CMD] Usage: rom <min> <max> (%.0f..%.0f deg, span >= %.0f deg) or 'rom reset'\n",
                              ROM_ABS_MIN_DEG, ROM_ABS_MAX_DEG, ROM_MIN_SPAN_DEG);
                return;
            }
            g_rom_min_deg = rmin;
            g_rom_max_deg = rmax;
            saveCalibrationToNVS();
            Serial.printf("[ROM] Set %.1f .. %.1f deg and saved to NVS.\n",
                          g_rom_min_deg, g_rom_max_deg);
        }
    } else if (strncmp(line, "assistcfg", 9) == 0 && (line[9] == ' ' || line[9] == '\0')) {
        const char *arg = &line[9];
        while (*arg == ' ' || *arg == '\t') arg++;
        if (*arg == '\0') {
            Serial.printf("[ASSISTCFG] Kp=%.2f Nm/deg | Kd=%.3f Nm/(deg/s) | Imax=%.1f A | GravFF=%.2f Nm\n",
                          g_kp_pos, g_kd_vel, g_assist_max_i_a, g_grav_ff_nm);
        } else if (strcmp(arg, "reset") == 0) {
            g_kp_pos         = DEFAULT_KP_POS_NM_DEG;
            g_kd_vel         = DEFAULT_KD_VEL_NMS_DEG;
            g_assist_max_i_a = DEFAULT_ASSIST_MAX_I_A;
            g_grav_ff_nm     = 0.0f;
            Serial.printf("[ASSISTCFG] Reset: Kp=%.2f | Kd=%.3f | Imax=%.1f A | GravFF=%.2f Nm\n",
                          g_kp_pos, g_kd_vel, g_assist_max_i_a, g_grav_ff_nm);
        } else {
            char *p1 = nullptr, *p2 = nullptr, *p3 = nullptr, *p4 = nullptr;
            float kp   = strtof(arg, &p1);
            float kd   = (p1 != arg) ? strtof(p1, &p2) : 0.0f;
            float imax = (p2 != p1)  ? strtof(p2, &p3) : 0.0f;
            float grav = g_grav_ff_nm;
            bool  bad_grav = false;
            if (p3 != nullptr && p3 != p2) {
                const char *g_arg = p3;
                while (*g_arg == ' ' || *g_arg == '\t') g_arg++;
                if (*g_arg != '\0') {
                    grav = strtof(g_arg, &p4);
                    if (p4 == g_arg) bad_grav = true;
                }
            }
            if (p1 == arg || p2 == p1 || p3 == p2 || bad_grav ||
                !isfinite(kp) || !isfinite(kd) || !isfinite(imax) || !isfinite(grav) ||
                kp < 0.05f || kp > 3.0f || kd < 0.0f || kd > 0.30f ||
                imax < 1.0f || imax > PEAK_CURRENT_LIMIT_A || grav < 0.0f || grav > 10.0f) {
                Serial.printf("[CMD] Usage: assistcfg <kp 0.05..3.0> <kd 0..0.30> <imax 1..%.0f> [grav_nm 0..10]\n",
                              PEAK_CURRENT_LIMIT_A);
                return;
            }
            g_kp_pos         = kp;
            g_kd_vel         = kd;
            g_assist_max_i_a = imax;
            g_grav_ff_nm     = grav;
            Serial.printf("[ASSISTCFG] Set: Kp=%.2f Nm/deg | Kd=%.3f Nm/(deg/s) | Imax=%.1f A | GravFF=%.2f Nm\n",
                          g_kp_pos, g_kd_vel, g_assist_max_i_a, g_grav_ff_nm);
        }
    } else if (strncmp(line, "goto", 4) == 0 && (line[4] == ' ' || line[4] == '\0')) {
        if (!g_is_homed) {
            Serial.println("[GOTO] Blocked: device not homed. Run 'autohome' (or 'home <deg>') first.");
            return;
        }
        const char *arg = &line[4];
        while (*arg == ' ' || *arg == '\t') arg++;
        char *p1 = nullptr;
        char *p2 = nullptr;
        float goal_tk = (*arg != '\0') ? strtof(arg, &p1) : 0.0f;
        if (*arg == '\0' || p1 == arg || !isfinite(goal_tk) ||
            goal_tk < ROM_ABS_MIN_DEG || goal_tk > ROM_ABS_MAX_DEG) {
            Serial.printf("[CMD] Usage: goto <%.0f..%.0f deg> [speed 5..45 deg/s]\n",
                          ROM_ABS_MIN_DEG, ROM_ABS_MAX_DEG);
            return;
        }
        float spd = strtof(p1, &p2);
        if (p2 == p1 || !isfinite(spd)) spd = 20.0f;
        spd = constrain(spd, 5.0f, 45.0f);

        stopAllImmediate();
        g_traj_target_tk    = g_theta_k_deg;
        g_goto_goal_tk      = goal_tk;
        g_goto_speed_degs   = spd;
        g_goto_settle_ms    = millis();
        g_goto_reached_sent = false;
        g_motion_msg_code   = 0;
        g_mode              = MODE_GOTO;
        Serial.printf("[GOTO] Moving Tk=%.1f -> %.1f deg @ %.1f deg/s (Imax=%.1f A; 's' to stop)\n",
                      g_theta_k_deg, g_goto_goal_tk, g_goto_speed_degs, g_assist_max_i_a);
    } else if (strncmp(line, "cpm", 3) == 0 && (line[3] == ' ' || line[3] == '\0')) {
        if (!g_is_homed) {
            Serial.println("[CPM] Blocked: device not homed. Run 'autohome' (or 'home <deg>') first.");
            return;
        }
        const char *arg = &line[3];
        while (*arg == ' ' || *arg == '\t') arg++;
        float spd = DEFAULT_CPM_SPEED_DEGS;
        int cyc = 0;
        if (*arg != '\0') {
            char *p1 = nullptr;
            spd = strtof(arg, &p1);
            if (p1 == arg || !isfinite(spd)) {
                Serial.println("[CMD] Usage: cpm [speed 5..45 deg/s] [cycles 0..100]");
                return;
            }
            int tmp_cyc = 0;
            if (!parseOptionalInt(p1, 0, &tmp_cyc) || tmp_cyc < 0 || tmp_cyc > 100) {
                Serial.println("[CMD] Usage: cpm [speed 5..45 deg/s] [cycles 0..100]");
                return;
            }
            cyc = tmp_cyc;
        }
        spd = constrain(spd, 5.0f, 45.0f);

        stopAllImmediate();
        g_cpm_speed_degs    = spd;
        g_cpm_target_cycles = cyc;
        g_cpm_done_cycles   = 0;
        g_traj_target_tk    = constrain(g_theta_k_deg, g_rom_min_deg, g_rom_max_deg);
        g_cpm_phase         = (g_theta_k_deg >= (g_rom_max_deg - 2.0f)) ? 2 : 0;
        g_motion_msg_code   = 0;
        resetRepAccumulators();
        g_mode = MODE_CPM;
        Serial.printf("[CPM] Running ROM %.1f..%.1f deg @ %.1f deg/s (%d cycles, 0=cont; Imax=%.1f A; 's' to stop)\n",
                      g_rom_min_deg, g_rom_max_deg, g_cpm_speed_degs, g_cpm_target_cycles, g_assist_max_i_a);
    } else if (strncmp(line, "assist", 6) == 0 && (line[6] == ' ' || line[6] == '\0')) {
        if (!g_is_homed) {
            Serial.println("[ASSIST] Blocked: device not homed. Run 'autohome' (or 'home <deg>') first.");
            return;
        }
        const char *arg = &line[6];
        while (*arg == ' ' || *arg == '\t') arg++;
        bool auto_cyc = false;
        if (strncmp(arg, "auto", 4) == 0 && (arg[4] == ' ' || arg[4] == '\t' || arg[4] == '\0')) {
            auto_cyc = true;
            arg += 4;
        }
        float spd = DEFAULT_AST_SPEED_DEGS;
        if (!parseOptionalFloat(arg, DEFAULT_AST_SPEED_DEGS, &spd) || !isfinite(spd)) {
            Serial.println("[CMD] Usage: assist [auto] [speed 5..45 deg/s]");
            return;
        }
        spd = constrain(spd, 5.0f, 45.0f);

        stopAllImmediate();
        g_assist_auto_cycle = auto_cyc;
        g_assist_speed_degs = spd;
        g_assist_anchor_tk  = g_theta_k_deg;
        g_traj_target_tk    = constrain(g_theta_k_deg, g_rom_min_deg, g_rom_max_deg);
        g_assist_msg_code   = 0;
        g_motion_msg_code   = 0;
        resetRepAccumulators();

        float mid = 0.5f * (g_rom_min_deg + g_rom_max_deg);
        if (g_theta_k_deg <= mid) {
            g_assist_phase = auto_cyc ? 1 : 0;
        } else {
            g_assist_phase = auto_cyc ? 4 : 3;
        }
        g_mode = MODE_ACTIVE_ASSIST;
        if (auto_cyc) {
            Serial.printf("[ASSIST] Auto AAN cycling (yield-forward): ROM %.1f..%.1f deg @ %.1f deg/s, Imax=%.1f A ('s' to stop)\n",
                          g_rom_min_deg, g_rom_max_deg, g_assist_speed_degs, g_assist_max_i_a);
        } else {
            Serial.printf("[ASSIST] Patient-triggered AAN ready (yield-forward): ROM %.1f..%.1f deg @ %.1f deg/s, Imax=%.1f A.\n",
                          g_rom_min_deg, g_rom_max_deg, g_assist_speed_degs, g_assist_max_i_a);
            Serial.printf("[ASSIST] Nudge %s%.2f deg to trigger stroke (or run 'assist auto').\n",
                          (g_assist_phase == 0) ? "+" : "-", ASSIST_NUDGE_TRIG_DEG);
        }
    } else {
        Serial.printf("[CMD] Unknown command '%s'. Type 'h' for help.\n", line);
    }
}

static void pollSerial() {
    static uint32_t s_last_rx_ms       = 0;
    static uint32_t s_last_cmd_exec_ms = 0;

    while (Serial.available() > 0) {
        uint8_t raw = (uint8_t)Serial.read();
        char    ch  = (char)raw;

        // Ignore trailing CR/LF within 50 ms of a completed command ("Both NL & CR" at 115200 baud)
        if ((ch == '\n' || ch == '\r') && g_cmd_len == 0 &&
            (uint32_t)(millis() - s_last_cmd_exec_ms) < 50U) {
            continue;
        }

        // Instant single-keypress abort during non-blocking bench pulses / cal / autohome
        if (g_mode == MODE_BENCH_PULSE || g_mode == MODE_CHOPPER_PULSE ||
            g_mode == MODE_DIRTEST     || g_mode == MODE_CAL_ZERO ||
            g_mode == MODE_AUTOHOME) {
            // Only abort on printable ASCII or explicit Enter (ignore 0x80..0xFF EMI/glitch bytes)
            if (ch == '\n' || ch == '\r' || (raw >= 32U && raw <= 126U)) {
                bool was_homing = (g_mode == MODE_AUTOHOME);
                stopAllImmediate();
                while (Serial.available() > 0) Serial.read();
                g_cmd_len          = 0;
                s_last_cmd_exec_ms = millis();
                Serial.println(was_homing ? "[AUTOHOME] Aborted by Serial input -> ALL OUTPUTS STOPPED."
                                          : "[TEST] Aborted by Serial input -> ALL OUTPUTS STOPPED.");
                return;
            }
            continue;
        }

        if (ch == '\n' || ch == '\r') {
            if (g_cmd_len > 0) {
                g_cmd_buf[g_cmd_len] = '\0';
                g_cmd_len            = 0;
                s_last_cmd_exec_ms   = millis();
                Serial.printf("[RX] %s\n", g_cmd_buf);
                handleCommand(g_cmd_buf);
            }
        } else if (ch == '\b' || raw == 0x7FU) {
            if (g_cmd_len > 0) {
                g_cmd_len--;
                s_last_rx_ms = millis();
            }
        } else if (raw >= 32U && raw <= 126U) {
            // Strictly accept 7-bit printable ASCII only (rejects 0x80..0xFF unsigned char noise on ESP32)
            s_last_rx_ms = millis();
            if (g_cmd_len < sizeof(g_cmd_buf) - 1) {
                g_cmd_buf[g_cmd_len++] = (char)tolower(raw);
            } else {
                g_cmd_len = 0;
            }
        }
    }

    // Auto-dispatch after 60 ms of RX idle so "No line ending" in Arduino Serial Monitor works seamlessly
    if (g_cmd_len > 0 && (uint32_t)(millis() - s_last_rx_ms) >= 60U) {
        g_cmd_buf[g_cmd_len] = '\0';
        g_cmd_len            = 0;
        s_last_cmd_exec_ms   = millis();
        Serial.printf("[RX] %s\n", g_cmd_buf);
        handleCommand(g_cmd_buf);
    }
}

// ----------------------------------------------------------------------------
// 10. SETUP & COOPERATIVE 500 Hz LOOP
// ----------------------------------------------------------------------------
void setup() {
    initPWM();
    stopAllImmediate();

    Serial.setRxBufferSize(256);
    Serial.begin(115200);
    delay(300);
    while (Serial.available() > 0) {
        Serial.read();
    }
    g_cmd_len = 0;

    Serial.printf("\n[BOOT] U2 (ESP_TOPHUB) Stage %d starting...\n", FW_STAGE);

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_VBUS_SENSE, ADC_11db);
    analogSetPinAttenuation(PIN_ACS712_SENSE, ADC_11db);
    delay(50);

    loadCalibrationFromNVS();
    if (!g_cal_valid) {
        zeroACS712BootFallback();
    }

    initEncoderPCNT();
    sampleAnalogSensors500Hz();

    printHelp();
#ifdef ESP_ARDUINO_VERSION_MAJOR
    Serial.printf("[INFO] Arduino-ESP32 Core: v%d.%d.%d\n",
                  ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
#endif
    printNvsStatus();
    if (!g_cal_valid) {
        Serial.printf("[INFO] Uncalibrated in NVS — using boot ACS712 0A=%.1f mV (run 'cal' when ready to save).\n",
                      g_acs_zero_mv);
    }
    printAdcDiagnostics();
    printEncDiagnostics(false);
    Serial.printf("[INFO] Cytron MDD10A (GPIO%d PWM / GPIO%d DIR) | Sign-Magnitude @ %lu Hz | %.0fA cont / %.0fA (<%.0fs) surge / %.0fA (%ums) OC cut.\n",
                  PIN_MD_PWM, PIN_MD_DIR, (unsigned long)g_md_pwm_freq_hz,
                  CONT_CURRENT_LIMIT_A, PEAK_CURRENT_LIMIT_A, PEAK_SURGE_MAX_S,
                  HW_OVERCURRENT_CUT_A, (unsigned)HW_OVERCURRENT_MS);
    Serial.printf("[INFO] Stage %d Active Assist ready: ROM=%.1f..%.1f deg | Chopper OFF (OV cut >%.1fV).\n",
                  FW_STAGE, g_rom_min_deg, g_rom_max_deg, VBUS_CHOP_OFF_OV_V);
    Serial.println("[INFO] Run 'autohome' (or 'home <deg>') before 'assist', 'cpm', or 'goto'.");
    while (Serial.available() > 0) {
        Serial.read();
    }
    g_cmd_len = 0;
    Serial.println("[READY] Serial command parser active (115200 baud — NL, CR, Both NL & CR, or No line ending).");
}

void loop() {
    static uint32_t last_ctrl_us  = 0;
    static uint32_t last_print_ms = 0;

    uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_ctrl_us) >= CONTROL_PERIOD_US) {
        if ((uint32_t)(now_us - last_ctrl_us) > (CONTROL_PERIOD_US * 4U)) {
            last_ctrl_us = now_us;
        } else {
            last_ctrl_us += CONTROL_PERIOD_US;
        }
        runControlStep500Hz();
    }

    pollSerial();

    if (g_nvs_save_pending) {
        g_nvs_save_pending = false;
        saveCalibrationToNVS();
    }

    if (g_acs_pol_warn_pending) {
        g_acs_pol_warn_pending = false;
        Serial.println("[WARN] ACS712 sign opposes drive current — PI paused (using FF). Run 'pol acs -1' or 'dirtest' to align.");
    }

    if (g_overspeed_msg_pend) {
        g_overspeed_msg_pend = false;
        Serial.printf("[SAFE] Arm speed exceeded %.0f deg/s — current output clamped to 0%%.\n", MAX_SAFE_ARM_DEGS);
    }

    if (g_oc_fault_msg_pend) {
        g_oc_fault_msg_pend = false;
        Serial.printf("[FAULT] Overcurrent (>%.0f A for %u ms) — motor stopped to MODE_IDLE.\n",
                      HW_OVERCURRENT_CUT_A, (unsigned)HW_OVERCURRENT_MS);
    }

    if (g_surge_msg_pend) {
        g_surge_msg_pend = false;
        Serial.printf("[LIMIT] %.0fs surge budget exhausted — folding current limit back to %.0f A continuous.\n",
                      PEAK_SURGE_MAX_S, CONT_CURRENT_LIMIT_A);
    }

    if (g_regen_msg_pend) {
        g_regen_msg_pend = false;
        Serial.println("[SAFE] CHOP_OFF regen guard triggered (VBUS > 26.5 V during plugging) — braking output clamped to 0%.");
    }

    if (g_homing_msg_code != 0) {
        uint8_t code = g_homing_msg_code;
        g_homing_msg_code = 0;
        if (code == 1) {
            Serial.printf("[AUTOHOME] Complete! STOP1 latched as 0.00 deg and parked at Tk=%.2f deg (H:1). Ready for 'assist', 'cpm', or 'goto'.\n",
                          g_theta_k_deg);
        } else if (code == 2) {
            Serial.println("[AUTOHOME] ABORT: Arm moved +Flexion instead of -Extension! Run 'dirtest' (or 'pol dir -1') and retry.");
        } else if (code == 3) {
            Serial.println("[AUTOHOME] ABORT: No motion and low current (<0.35 A) / low VBUS — check motor leads & 24V power.");
        } else if (code == 4) {
            Serial.println("[AUTOHOME] ABORT: High current (>10 A) during seek — check mechanical jam at STOP1.");
        } else if (code == 5) {
            Serial.println("[AUTOHOME] ABORT: 15 s timeout before STOP1 stall detected.");
        } else if (code == 6) {
            Serial.printf("[AUTOHOME] WARN: STOP1 latched (H:1), backoff timed out at Tk=%.2f deg (target %.1f deg).\n",
                          g_theta_k_deg, g_home_park_deg);
        }
    }

    if (g_motion_msg_code != 0) {
        uint8_t code = g_motion_msg_code;
        float   val  = g_motion_msg_val;
        g_motion_msg_code = 0;
        if (code == 1) {
            Serial.printf("[GOTO] Reached Tk=%.2f deg (holding target; send 's' to release).\n", val);
        } else if (code == 2) {
            Serial.printf("[GOTO] Stopped: obstruction/stall (>15 deg error at Tk=%.2f deg) -> IDLE.\n", val);
        } else if (code == 3) {
            Serial.printf("[CPM] Completed %.0f cycle(s) -> IDLE.\n", val);
        } else if (code == 4) {
            Serial.printf("[CPM] Stopped: obstruction/spasm (>18 deg error at Tk=%.2f deg) -> IDLE.\n", val);
        } else if (code == 5) {
            Serial.printf("[SAFE] CHOP_OFF overvoltage cutoff (VBUS=%.2f V > %.1f V) -> IDLE.\n",
                          val, VBUS_CHOP_OFF_OV_V);
        }
    }

    if (g_assist_msg_code != 0) {
        uint8_t code = g_assist_msg_code;
        float   val  = g_assist_msg_val;
        g_assist_msg_code = 0;
        if (code == 1) {
            Serial.printf("[ASSIST] Flexion triggered at Tk=%.2f deg -> assisting to %.1f deg (yields if you push faster).\n",
                          val, g_rom_max_deg);
        } else if (code == 2) {
            Serial.printf("[ASSIST] Extension triggered at Tk=%.2f deg -> assisting to %.1f deg.\n",
                          val, g_rom_min_deg);
        } else if (code == 3) {
            Serial.printf("[ASSIST] Flexion paused (resistance at Tk=%.2f deg) — nudge +%.2f deg when ready.\n",
                          val, ASSIST_NUDGE_TRIG_DEG);
        } else if (code == 4) {
            Serial.printf("[ASSIST] Extension paused (resistance at Tk=%.2f deg) — nudge -%.2f deg when ready.\n",
                          val, ASSIST_NUDGE_TRIG_DEG);
        } else if (code == 5) {
            Serial.printf("[ASSIST] Reached Flexion apex (Tk=%.2f deg)%s\n",
                          val, g_assist_auto_cycle ? " — returning." : " — nudge toward Extension when ready.");
        } else if (code == 6) {
            Serial.printf("[ASSIST] Reached Extension limit (Tk=%.2f deg)%s\n",
                          val, g_assist_auto_cycle ? " — cycling." : " — nudge toward Flexion when ready.");
        }
    }

    if (g_rep_done_msg) {
        g_rep_done_msg = false;
        Serial.printf("[REP #%lu] ROM: %.1f..%.1f deg (span %.1f deg) | Peak Knee Tq: +%.2f / -%.2f Nm | Work: %.2f J\n",
                      (unsigned long)g_last_rep_num,
                      g_last_rep_min_tk,
                      g_last_rep_max_tk,
                      g_last_rep_max_tk - g_last_rep_min_tk,
                      g_last_rep_flex_nm,
                      g_last_rep_ext_nm,
                      g_last_rep_work_j);
    }

    if (g_ctest_done_msg) {
        g_ctest_done_msg = false;
        Serial.printf("[CTEST] Done (%d%%)! VBUS After=%.2f V (%s) | Mode:%s (Duty:%.0f%%)\n",
                      g_ctest_pct, g_vbus_volts, g_vbus_valid ? "OK" : "NOM24V",
                      chopModeName(g_chop_mode), g_chopper_duty_pct);
        Serial.println("[CTEST] Check: if U202 LED stays off & resistors stay cold even at 'ctest 50', U202 optocoupler needs 5V + NPN driver on Control+/-.");
    }

    if (g_cal_done_msg) {
        g_cal_done_msg = false;
        if (g_cal_acs_ok) {
            Serial.printf("[CAL] Saved to NVS: ACS_Zero=%.1f mV (%lu samples) | Vbus_idle=%.2f V (%s, %lu samples)\n",
                          g_acs_zero_mv, (unsigned long)g_cal_acs_cnt,
                          g_vbus_idle_v, g_cal_vbus_ok ? "OK" : "NOM", (unsigned long)g_cal_vbus_cnt);
        } else {
            Serial.printf("[CAL] FAILED: ACS712 out of valid range (got %lu samples, raw=%.1f mV).\n",
                          (unsigned long)g_cal_acs_cnt, g_acs_raw_mv);
        }
    }

    if (g_dirtest_done_msg) {
        g_dirtest_done_msg = false;
        Serial.printf("[DIRTEST] Complete: FWD(DIR=HIGH)=%+lld cnt (%+.2f deg, raw_I=%+.2fA) | REV(DIR=LOW)=%+lld cnt (%+.2f deg, raw_I=%+.2fA)\n",
                      (long long)g_dirtest_fwd_dcnt,
                      (float)g_dirtest_fwd_dcnt * DEG_PER_COUNT,
                      g_dirtest_fwd_avg_i,
                      (long long)g_dirtest_rev_dcnt,
                      (float)g_dirtest_rev_dcnt * DEG_PER_COUNT,
                      g_dirtest_rev_avg_i);
        if (!g_dirtest_bidir_ok) {
            Serial.println("[DIRTEST] Bi-dir check incomplete (need >10 counts in opposite directions). Check stops, power, DIR wiring, or ENC1.");
        } else {
            if (g_dirtest_dir_flip) {
                Serial.printf("[DIRTEST] Auto-flipped dir_polarity to %+d (DIR=%s drives +angle/+Flexion; saved to NVS).\n",
                              g_dir_polarity, (g_dir_polarity > 0) ? "HIGH" : "LOW");
            } else {
                Serial.printf("[DIRTEST] MDD10A DIR polarity OK (dp=%+d, DIR=%s -> +angle; ep=%+d).\n",
                              g_dir_polarity, (g_dir_polarity > 0) ? "HIGH" : "LOW", g_enc_polarity);
            }
            if (g_dirtest_samples > 10) {
                float pos_raw_i = (g_dir_polarity > 0) ? g_dirtest_fwd_avg_i : g_dirtest_rev_avg_i;
                if (g_dirtest_acs_flip) {
                    Serial.printf("[DIRTEST] Auto-flipped acs_polarity to %+d so +angle drive reads +Amps (saved to NVS).\n",
                                  g_acs_polarity);
                } else {
                    Serial.printf("[DIRTEST] ACS712 polarity=%+d (signed +angle avg %+.2f A).\n",
                                  g_acs_polarity, pos_raw_i * (float)g_acs_polarity);
                }
            }
        }
    }

    if (g_live_print && (uint32_t)(millis() - last_print_ms) >= TELEM_PERIOD_MS) {
        last_print_ms = millis();
        printTelemetry();
    }
}
