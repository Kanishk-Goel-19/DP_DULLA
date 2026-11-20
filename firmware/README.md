# DOULA ESP32 Firmware & Integrated System Guide

Ready-to-flash Arduino sketches for the three-node DOULA rehabilitation architecture communicating over Wi-Fi WebSockets (`U1 <-> Laptop/Website`) and unified ESP-NOW (`U1 <-> U2, U3`):

- **`U1` (`ESP_MASTER`):** [doula_esp32s.ino](doula_esp32s/doula_esp32s.ino) — Wi-Fi STA + WebSocket server (`port 81`) + ESP-NOW master coordinator, auto-live/simulation arbitration, and Serial monitor interface.
- **`U2` (`TOP_HUB`):** [esp_tophub_staged.ino](esp_tophub_staged/esp_tophub_staged.ino) — Flexion/extension motor drive, encoder & torque sensing, `PCNT_UNIT_0`, safety/hardstop/spasm supervision, and 50 Hz ESP-NOW telemetry + rep summaries.
- **`U3` (`CYCLING_STEPPER`):** [doula_stepper_esp32.ino](doula_stepper_esp32/doula_stepper_esp32.ino) — Cycling load-adjusting stepper controller (`GPIO18 -> PUL+`, `GPIO19 -> DIR+`) with optional flywheel `ENC2` (`PCNT_UNIT_0` on `GPIO32/33`) + 100 Hz Momentum Disturbance Observer hook.
- **Shared Protocol Header:** Byte-identical copies of `rehab_espnow_protocol.h` reside inside each sketch folder ([doula_esp32s/rehab_espnow_protocol.h](doula_esp32s/rehab_espnow_protocol.h), [esp_tophub_staged/rehab_espnow_protocol.h](esp_tophub_staged/rehab_espnow_protocol.h), [doula_stepper_esp32/rehab_espnow_protocol.h](doula_stepper_esp32/rehab_espnow_protocol.h)) so each sketch compiles standalone in Arduino IDE.

## 1. Flash Instructions (Windows Laptop + Arduino IDE)

1. Install board support (**esp32 by Espressif Systems**) via **Boards Manager** and select **NodeMCU-32S** (or **ESP32 Dev Module**).
2. Open **Library Manager** and install **`WebSockets`** by **Markus Sattler (Links2004)** (required for `U1`).
3. Edit `WIFI_SSID` and `WIFI_PASSWORD` in [doula_esp32s.ino](doula_esp32s/doula_esp32s.ino) (lines 33–34) to match the Wi-Fi network or phone hotspot your laptop is connected to:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```
4. Review compile-time integration flags as needed:
   - In `doula_esp32s.ino`:
     - `BENCH_SIMULATION_FALLBACK` (`true` default): allows per-node simulation fallback when `U2` or `U3` is offline; automatically switches to live node data as soon as the node links.
     - `CYCLING_RPM_FALLBACK_NO_ENC2` (`true` default): estimates cycling RPM/speed on `U1` when `U3` has no `ENC2` flywheel sensor (`CYCLE_FLAG_ENC2_ACTIVE` clear).
     - `AUTO_HOME_U2_ON_START` (`true` default): if `U2` is live and unhomed when starting a flexion session, `U1` automatically runs `OP_SETUP_QUICK_6S` first and transitions into the requested therapy mode once `STATE_FLAG_HOMED && !STATE_FLAG_BUSY_HOMING`.
   - In `doula_stepper_esp32.ino`:
     - `ENABLE_FLYWHEEL_ENCODER` (`false` default): set `true` once `ENC2` is wired to `GPIO32`/`GPIO33` to enable `PCNT_UNIT_0` quadrature sensing and the 100 Hz Momentum Disturbance Observer.
5. Disconnect any external `+3V3_LDO` power rail before plugging USB into the laptop (per [WIRING_AND_COMMISSIONING_GUIDE.md](../docs/WIRING_AND_COMMISSIONING_GUIDE.md)).
6. Flash each sketch to its respective ESP32 (`doula_esp32s.ino`, `esp_tophub_staged.ino`, `doula_stepper_esp32.ino`), open `U1`'s **Serial Monitor (`115200` baud)**, and note the printed IP address (for example `192.168.1.42` or `doula-esp32.local`).

## 2. Connect to the Website

1. Start the Node server on your laptop (`npm --prefix software_new start`, or `HOST=0.0.0.0 npm --prefix software_new start` if opening the site from your phone too).
2. Log in and on **Dashboard -> Device Connection Gate**:
   - Paste your `U1` ESP32 IP (e.g. `192.168.1.42` or `ws://192.168.1.42:81/` or `doula-esp32.local`) into the **ESP32 Address** box.
   - Click **Connect ESP32**.
   - Click **Start Session** -> choose **Flexion & Extension** or **Cycling**.
3. While active:
   - Live **torque**, **reps / cycles**, **RPM**, and **speed** stream from `U1` at `20 Hz` via the Node WebSocket bridge (`/ws/device`).
   - Clicking **− / + Torque** on either **Flexion & Extension** or **Cycling** immediately sends `set_torque` JSON commands to `U1`, which forwards unified ESP-NOW commands to `U2`/`U3` and streams updated telemetry back to the website.
   - Ending a verified ESP32 session saves it with **`ESP32 Verified`** in **Track Progress**.

## 3. Unified ESP-NOW Protocol & Pairing

- **Channel Sync & Auto-Discovery (No Hardcoded MACs):**
  - `U1` remains in Wi-Fi STA mode on the router's channel and broadcasts/unicasts `RehabCommandPacket_t` (`45 B`, CRC-16-CCITT) at `100 ms` (`10 Hz`) plus immediate command bursts on state changes.
  - Both `U2` (`esp_tophub_staged`) and `U3` (`doula_stepper_esp32`) scan channels `1–13` (`180 ms/ch` on `U2` when idle/unlinked), lock onto `pkt.wifi_channel` and `U1`'s MAC automatically, and ignore sibling node telemetry frames cleanly.
- **Packet Types (`rehab_espnow_protocol.h`):**
  - `RehabCommandPacket_t` (`45 B`): unified command & heartbeat packet from `U1` carrying `wifi_channel`, `boot_id`, `op_mode` (including `OP_ACTIVE_ASSIST = 11` and `OP_SETUP_QUICK_6S`), therapy parameters, `stepper_action`, and `action_seq`.
  - `TopHubTelemetryPacket_t` (`45 B`): `50 Hz` telemetry from `U2` when linked (`STATE_FLAG_HOMED`, `STATE_FLAG_BUSY_HOMING`, `STATE_FLAG_FAULT_LOCKOUT`, angle, torque, velocity, reps, safety status).
  - `TopHubRepSummaryPacket_t` (`37 B`): per-rep work and peak flexion/extension torque summary from `U2`.
  - `CyclingTelemetryPacket_t` (`47 B`): cycling status & optional encoder/observer telemetry from `U3` (`20 Hz` active / `5 Hz` idle).
- **Watchdog Safety:**
  - `U2` enforces a `500 ms` ESP-NOW watchdog (`ESPNOW_WATCHDOG_MS = 500U`).
  - `U3` enforces a `1500 ms` active-session watchdog (`ACTIVE_WATCHDOG_MS = 1500`), retracting smoothly back to `0°` on link loss.
- **Cycling Stepper Kinematics (`doula_stepper_esp32.ino`):**
  - With `GEAR_RATIO = 6.0f` and `1.8°` full steps (`DEG_PER_STEP = 0.30°/step`), starting a Cycling session moves to `22.50°` (`DEFAULT_START_STEPS = 75` steps).
  - Each `+` / `−` step command is deduplicated via `(boot_id, action_seq)` so `100 ms` heartbeats do not re-trigger single steps; stopping or E-Stop retracts smoothly to `0.00°`.

## 4. JSON WebSocket Protocol (Port `81`)

### Website -> `U1` Commands
- `{"type":"start_session","mode":"isotonic|passive|isokinetic|active_assist|isometric|cycling","op_mode":4,"role":"resistance|support","torque_nm":22,"target_torque_ext":22,"target_reps":15,"rom_min_deg":0,"rom_max_deg":110,"target_vel_deg_s":45}` (supports optional extended fields: `pathology_profile`, `d_offset_m`, `L_leg_m`, `target_torque_flex`)
- `{"type":"set_torque","mode":"isotonic","op_mode":4,"role":"resistance","torque_nm":24,"target_torque_ext":24,"rom_min_deg":0,"rom_max_deg":110,"target_vel_deg_s":45}`
- `{"type":"set_params","mode":"passive","op_mode":3,"role":"support","torque_nm":12,"rom_min_deg":0,"rom_max_deg":110,"target_vel_deg_s":18}`
- `{"type":"pause_session"}` / `{"type":"resume_session"}` / `{"type":"stop_session"}` / `{"type":"estop","estop_active":1}`
- Top-Hub homing & lock recovery commands:
  - `{"type":"setup_home"}` -> `OP_SETUP_QUICK_6S` (instant `0.0°` full-extension home latch + ACS712 RAM zero on `U2`)
  - `{"type":"setup_full"}` -> `OP_SETUP_FULL_25S` (motorized `STOP1` extension hard-stop seek + backoff on `U2`)
  - `{"type":"hardstop_release"}` -> `ESTOP_HARD_STOP_RELEASE` (releases electronic hard-stop / spasm lockout on `U2` & `U3`)

### `U1` -> Website Telemetry (`20 Hz` Active / `4 Hz` Idle)
- **Flexion & Extension:** `{"type":"telemetry","mode":"isotonic","op_mode":4,"active":1,"support_torque":0.0,"resistance_torque":22.4,"torque":22.4,"target_torque":22.0,"knee_angle_deg":64.5,"rom_min_deg":0.0,"rom_max_deg":110.0,"target_vel_deg_s":45.0,"completed_reps":5,"reps_remaining":10,"estop":0,"paused":0,"u2_linked":1,"homed":1,"busy_homing":0,"hardstop_locked":0,"spasm":0,"knee_vel_deg_s":31.2,"motor_current_a":2.15,"vbus_v":24.10,"last_rep_work_j":14.20}`
- **Cycling:** `{"type":"telemetry","mode":"cycling","op_mode":7,"active":1,"rpm":68.2,"speed":21.1,"torque":18.1,"target_torque":18.0,"stepper_angle_deg":22.50,"stepper_steps":75,"u2_linked":1,"homed":1,"knee_angle_deg":0.0,"reps":9,"estop":0,"paused":0}`

## 5. Serial Monitor Commands (`115200` baud)

<<<<<<< HEAD
Flash [doula_stepper_esp32.ino](doula_stepper_esp32/doula_stepper_esp32.ino) to the second ESP32 (`U3` wired to `GPIO18 -> PUL+`, `GPIO19 -> DIR+`, `PUL-/DIR- -> GND`):

- **Automatic ESP-NOW Pairing & Channel Sync:** `U1` (`doula_esp32s.ino`) broadcasts a `500 ms` sync heartbeat carrying its active Wi-Fi channel; `U3` scans channels `1–13`, locks onto `U1`, and pairs automatically without hardcoding MAC addresses.
- **Session Lifecycle (`1.8°/step` full-step default):**
  - **Start Cycling Session** (from website or `START` on Master/Stepper Serial) → moves to `22.5°` (`13` steps = `23.40°` at `1.8°/step`; adjust `MICROSTEPS` or `GEAR_RATIO` at the top of `doula_stepper_esp32.ino` if using microstepping or `1:6` gearing).
  - **`+` / `−`** (from website torque controls, Master Serial `+` / `-`, or Stepper Serial `+` / `-`) → moves `+1` / `-1` step and updates tracked `currentSteps` & `angleDeg`.
  - **End Session / E-Stop** (from website or `END` on Master/Stepper Serial) → returns smoothly back to `0.00°` (step `0`).
- **Stepper Serial Monitor Commands (`115200` baud):** `+`, `-`, `START` (`s`), `END` (`e`), `HOME` (`0`), `ZERO`, `STATUS` (`?`).

## 5. Standalone Multi-Mode Cycling Controller (`doula_cycling_modes/doula_cycling_modes.ino`)

Flash [doula_cycling_modes.ino](doula_cycling_modes/doula_cycling_modes.ino) to `U3` (`ESP_CYCLING`) for standalone Serial-Monitor operation (`115200` baud) with `ENC2` quadrature encoder feedback:

- **Wiring:** Stepper `DIR+` → `GPIO18`, `PUL+` → `GPIO19`, `PUL-/DIR-` → `GND` (`1.8°/step`, `1:6` gear `GBX2` = `0.30°/step`, `0..42°`); Encoder `ENC2` `OUT_A` → `GPIO32`, `OUT_B` → `GPIO33` (via `80 Ω` series damping + internal `INPUT_PULLUP`, `1440 CPR` flywheel / `7200` counts per pedal rev).
- **4 Selectable Modes:**
  - `manual` / `m0`: Base manual step mode (`start` → `22.5°`, `+`/`-` → `±1` step, `angle <deg>`, `end` → `0°`).
  - `torque [Nm]` / `m1 [Nm]`: Constant Pedal Torque (`2..85 N·m`, `+`/`-` adjusts `±2 N·m`).
  - `power [W]` / `m2 [W]`: Constant Power Ergometer (`15..250 W`, `+`/`-` adjusts `±10 W`).
  - `isok [RPM]` / `m3 [RPM]`: Isokinetic Cadence-Clamping (`15..100 RPM`, `+`/`-` adjusts `±5 RPM`).
- **Noise Immunity & Anti-Stall:** Hardware `PCNT` 4× decoding with `5.0 µs` glitch filter, per-tick delta spike rejection (`> 220` counts/`10 ms`), EMA velocity low-pass filter (`lpf`), Luenberger Momentum Disturbance Observer (`dob on|off`), `1.5 s` soft-start ramp, `5°` coasting unload standby, `30 °/s` slew limit, and NVS persistence (`save` / `resetcal`).
- **Diagnostics & Tuning:** `status` (`?`), `enc`, `log [on|off]`, `plot`, `lpf <alpha>`, `dob on|off`, `encpol [1|-1]`, `keddy <val>`, `fric <c0> <bvisc>`, `pi <kp> <ki>`, `help`.
=======
- **`U1` (`doula_esp32s.ino`):**
  - Diagnostics & Live Stream: `HELP` (`?`), `STATUS` (`IP`), `NODES`, `STREAM` (`P` — toggles `5 Hz` live `U2` telemetry stream on USB Serial).
  - Top-Hub Homing & Modes: `HOME` / `ZERO` (`OP_SETUP_QUICK_6S` instant `0.0°` home), `AUTOHOME` / `CAL` / `SETUP` (`OP_SETUP_FULL_25S` motorized `STOP1` seek), `CPM [deg_s] [Nm]`, `ASSIST [deg_s] [Nm]`, `ISO [ext_Nm] [flex_Nm]`, `ISOK [deg_s] [cap_Nm]`, `ISOM [hold_deg] [cap_Nm]`, `ROM <min> <max>`, `SPEED <deg_s>`, `TORQUE <ext_Nm> [flex_Nm]`, `PWM <signed_pct>`, `BENCH`.
  - Lifecycle & Safety: `PAUSE`, `RESUME`, `STOP` (`END`), `ESTOP` (`LOCK`), `RELEASE` (`CLEAR`), `START`, `+`, `-`, `REP`.
- **`U3` (`doula_stepper_esp32.ino`):** `+`, `-`, `START` (`s`), `END` (`e`), `HOME` (`0`), `ZERO`, `STATUS` (`?`).
>>>>>>> 27ab21cfb38aebea385d87cf9a3a87dbd4fe94ba
