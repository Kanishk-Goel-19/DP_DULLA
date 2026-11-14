#ifndef REHAB_ESPNOW_PROTOCOL_H
#define REHAB_ESPNOW_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

// ============================================================================
// SHARED ESP-NOW PROTOCOL HEADER (U1: MASTER, U2: TOPHUB, U3: CYCLING)
// Locked to 2.4 GHz Wi-Fi Channel 1 for Wi-Fi AP + ESP-NOW Coexistence
// ============================================================================

#define REHAB_WIFI_CHANNEL 1

enum NodeID : uint8_t {
    NODE_MASTER  = 0x01, // U1: Web Server & Clinical Coordinator
    NODE_TOPHUB  = 0x02, // U2: Top Hub Knee Flexion/Extension
    NODE_CYCLING = 0x03  // U3: Bottom Tier Cycling & Eddy Brake
};

enum PacketType : uint8_t {
    PKT_HEARTBEAT_CMD      = 0x10, // Master -> Worker: Mode, Targets, Geometry, Hard-Stop, Setup
    PKT_TOPHUB_TELEMETRY   = 0x20, // U2 -> Master (50 Hz real-time stream)
    PKT_TOPHUB_REP_SUMMARY = 0x21, // U2 -> Master (Sent at completion of each rep)
    PKT_CYCLING_TELEMETRY  = 0x30  // U3 -> Master (50 Hz real-time stream)
};

enum OperatingMode : uint8_t {
    OP_IDLE            = 0,
    OP_SETUP_FULL_25S  = 1, // Full 6-Stage Calibration & Parameter ID
    OP_SETUP_QUICK_6S  = 2, // Quick Static Zero + Hard-Stop Homing
    OP_CPM             = 3, // Mode 1: Continuous Passive Motion + Spasm Backoff
    OP_ISOTONIC        = 4, // Mode 2: Physiological Curve-Shaped Isotonic
    OP_ISOKINETIC      = 5, // Mode 3: Accommodating Isokinetic
    OP_ISOMETRIC       = 6, // Mode 4: Multi-Angle Isometric Hold
    OP_CYCLE_TORQUE    = 7, // Bottom Tier: Constant Torque Cycling
    OP_CYCLE_POWER     = 8, // Bottom Tier: Constant Power Ergometer
    OP_DIRECT_PWM_TEST = 9, // Direct open-loop PWM bench test ("pwm 40" / "pwm -40")
    OP_AUTO_BENCH_TEST = 10 // Automatic bidirectional bench test ("test")
};

enum PathologyProfile : uint8_t {
    PATH_STANDARD      = 0,
    PATH_ACL_PROTECTED = 1, // Enforces 40 deg -> 0 deg terminal extension shear taper
    PATH_TKA           = 2,
    PATH_PFPS          = 3
};

enum HardStopMode : uint8_t {
    ESTOP_NORMAL            = 0, // Normal therapy operation
    ESTOP_HARD_STOP_LOCKED  = 1, // <3ms Electronic Brake Hold with 75N Compliant Yield Ceiling
    ESTOP_HARD_STOP_RELEASE = 2  // Zero-Torque Freewheel / Arm Pressure Release
};

// State Flag Bits for TopHubTelemetryPacket_t::state_flags
#define STATE_FLAG_HOMED            (1 << 0)
#define STATE_FLAG_CALIBRATED       (1 << 1)
#define STATE_FLAG_CHOPPER_ACTIVE   (1 << 2)
#define STATE_FLAG_SPASM_STOP       (1 << 3)
#define STATE_FLAG_HARDSTOP_LOCKED  (1 << 4)

// 1. Master -> Worker Command Packet (40 Bytes)
struct __attribute__((packed)) RehabCommandPacket_t {
    uint8_t  pkt_type;           // PKT_HEARTBEAT_CMD (0x10)
    uint8_t  target_node;        // NODE_TOPHUB or NODE_CYCLING
    uint16_t seq_num;            // Rolling sequence counter
    uint8_t  op_mode;            // OperatingMode enum (0..8)
    uint8_t  pathology_profile;  // PathologyProfile enum (0..3)
    uint8_t  estop_active;       // HardStopMode enum (0=NORMAL, 1=LOCKED, 2=RELEASE)
    float    d_offset_m;         // Top-Hub vertical pivot offset above knee [m] (default 0.090m)
    float    L_leg_m;            // Tibial pad distance from knee pivot [m] (default 0.300m)
    float    rom_min_deg;        // Extension limit [deg] (e.g., 0.0 deg)
    float    rom_max_deg;        // Flexion limit [deg] (e.g., 110.0 deg)
    float    target_vel_deg_s;   // Target speed [deg/s] (Top Hub) or Target Cadence [RPM]
    float    target_torque_ext;  // Peak Knee Extension Torque [N*m] (<=68 N*m)
    float    target_torque_flex; // Peak Knee Flexion Torque [N*m] (<=40 N*m)
    uint16_t crc16;              // CRC-16-CCITT checksum
};

// 2. Top Hub (U2) -> Master Real-Time Telemetry Packet (48 Bytes @ 50 Hz)
struct __attribute__((packed)) TopHubTelemetryPacket_t {
    uint8_t  pkt_type;           // PKT_TOPHUB_TELEMETRY (0x20)
    uint8_t  node_id;            // NODE_TOPHUB (0x02)
    uint16_t seq_num;
    uint8_t  state_flags;        // Bit0: Homed, Bit1: Calibrated, Bit2: Chopper, Bit3: Spasm, Bit4: HardStop
    uint16_t current_rep;        // Completed repetitions in current set
    float    knee_angle_deg;     // True anatomical knee angle theta_k [deg]
    float    knee_vel_deg_s;     // Instantaneous anatomical angular velocity [deg/s]
    float    knee_torque_nm;     // Anatomical interaction torque at knee tau_knee [N*m]
    float    gearbox_torque_nm;  // Gearbox shaft torque tau_gbx = Gamma(theta_k)*tau_knee [N*m]
    float    pad_force_newtons;  // Normal patient leg force at shin pad F_pad = tau_knee / L_leg [N]
    float    motor_current_amps; // Measured ACS712-30A bidirectional armature current I_m [A]
    float    target_curve_nm;    // Instantaneous physiological target knee torque tau_ref(theta_k) [N*m]
    float    vbus_volts;         // Measured 24V bus voltage [V]
    float    chopper_duty_pct;   // Brake chopper PWM duty cycle [0-100%]
    uint16_t crc16;
};

// 3. Top Hub (U2) -> Master Per-Repetition Summary Packet (40 Bytes, Event-Driven)
struct __attribute__((packed)) TopHubRepSummaryPacket_t {
    uint8_t  pkt_type;           // PKT_TOPHUB_REP_SUMMARY (0x21)
    uint16_t rep_index;          // 1..N
    float    achieved_rom_min;   // Peak extension reached in rep [deg]
    float    achieved_rom_max;   // Peak flexion reached in rep [deg]
    float    peak_torque_ext_nm; // Peak quadriceps extension torque [N*m]
    float    angle_at_peak_ext;  // Angle where peak quad torque occurred [deg]
    float    peak_torque_flex_nm;// Peak hamstring flexion torque [N*m]
    float    angle_at_peak_flex; // Angle where peak hamstring torque occurred [deg]
    float    rep_work_joules;    // Mechanical work integral(tau * d_theta) during rep [J]
    float    vel_cov_pct;        // Velocity coefficient of variation (smoothness) [%]
    uint16_t crc16;
};

// CRC-16-CCITT (Polynomial 0x1021, Initial 0xFFFF)
static inline uint16_t computeCRC16(const void *data, size_t len) {
    const uint8_t *bytes = (const uint8_t *)data;
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (uint8_t b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

#endif // REHAB_ESPNOW_PROTOCOL_H
