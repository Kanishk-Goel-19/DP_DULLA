# DOULA ESP32-S Firmware & Laptop Testing Guide

Ready-to-flash Arduino sketch for **ESP32-S NodeMCU V1.1** (`U1: ESP_MASTER`).

## 1. Flash Instructions (Windows Laptop + Arduino IDE)

1. Open [doula_esp32s.ino](doula_esp32s/doula_esp32s.ino) in **Arduino IDE**.
2. Install board support (**esp32 by Espressif Systems**) via **Boards Manager** and select **NodeMCU-32S** (or **ESP32 Dev Module**).
3. Open **Library Manager** and install **`WebSockets`** by **Markus Sattler (Links2004)**.
4. Edit lines 23–24 in `doula_esp32s.ino` with the same Wi-Fi (or phone hotspot) your laptop is on:
   ```cpp
   const char* WIFI_SSID     = "YOUR_WIFI_SSID";
   const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
   ```
5. Disconnect any external `+3V3_LDO` power rail before plugging USB into the laptop (per [WIRING_AND_COMMISSIONING_GUIDE.md](../docs/WIRING_AND_COMMISSIONING_GUIDE.md)).
6. Click **Upload**, open **Serial Monitor (`115200` baud)**, and note the printed IP address (for example `192.168.1.42` or `doula-esp32.local`).

## 2. Connect to the Website

1. Start the Node server on your laptop (`npm --prefix software_new start`, or `HOST=0.0.0.0 npm --prefix software_new start` if opening the site from your phone too).
2. Log in and on **Dashboard -> Device Connection Gate**:
   - Paste your ESP32 IP (e.g. `192.168.1.42` or `ws://192.168.1.42:81/` or `doula-esp32.local`) into the **ESP32 Address** box.
   - Click **Connect ESP32**.
   - Click **Start Session** -> choose **Flexion & Extension** or **Cycling**.
3. While active:
   - Live **torque**, **reps / cycles**, **RPM**, and **speed** stream from the ESP32 at `20 Hz` via the Node WebSocket bridge (`/ws/device`).
   - Clicking **− / + Torque** on either **Flexion & Extension** or **Cycling** immediately sends `set_torque` JSON commands to the ESP32 and updates the live streamed telemetry.
   - Ending a verified ESP32 session saves it with **`ESP32 Verified`** in **Track Progress**.

## 3. JSON WebSocket Protocol (Port `81`)

### Website -> ESP32 Commands
- `{"type":"start_session","mode":"isotonic|passive|isokinetic|cycling","op_mode":4,"role":"resistance|support","torque_nm":22,"target_torque_ext":22,"target_reps":15,"rom_min_deg":0,"rom_max_deg":110}`
- `{"type":"set_torque","mode":"cycling","op_mode":7,"role":"resistance","torque_nm":18,"target_torque_ext":18,"delta":2}`
- `{"type":"pause_session"}` / `{"type":"resume_session"}` / `{"type":"stop_session"}` / `{"type":"estop","estop_active":1}`

### ESP32 -> Website Telemetry (`20 Hz`)
- **Flexion & Extension:** `{"type":"telemetry","mode":"isotonic","op_mode":4,"support_torque":0.0,"resistance_torque":22.4,"torque":22.4,"target_torque":22.0,"knee_angle_deg":64.5,"completed_reps":5,"reps_remaining":10,"estop":0}`
- **Cycling:** `{"type":"telemetry","mode":"cycling","op_mode":7,"rpm":68.2,"speed":21.1,"torque":18.1,"target_torque":18.0,"stepper_angle_deg":23.40,"stepper_steps":13,"reps":9,"estop":0}`

## 4. Cycling Stepper Node (`doula_stepper_esp32/doula_stepper_esp32.ino`)

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
