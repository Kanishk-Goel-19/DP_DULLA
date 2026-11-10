# Complete Hardware Wiring, Potentiometer Tuning, Steel-Chassis Isolation & Pre-Power Commissioning Guide

> **Repository Cross-Reference Index (Project Doula — `VK26-IITI-HT-0240`):**
> * **Master Top-Hub Architecture (`U2: ESP_TOPHUB`):** [`README.md`](./README.md)
> * **Bottom Cycling & Eddy-Current Brake (`U3: ESP_CYCLING`):** [`README_CYCLING_EDDY_BRAKE.md`](./README_CYCLING_EDDY_BRAKE.md)
> * **Integrated 3-ESP32 Network & Web GUI (`U1: ESP_MASTER`):** [`README_INTEGRATED_SYSTEM_AND_WEB_APP.md`](./README_INTEGRATED_SYSTEM_AND_WEB_APP.md)
> * **Electrical Schematics & BOM:** [`System_Schematics_3Page.pdf`](./System_Schematics_3Page.pdf) | [`Sheet 1`](./schematic_page1_master_power_network.svg) | [`Sheet 2`](./schematic_page2_top_hub_control.svg) | [`Sheet 3`](./schematic_page3_bottom_cycling_ecb.svg) | [`Sheet 4 (BOM)`](./schematic_page4_bom.svg)
> * **3D CAD & SKF Bearings:** [`Doula_DP_parts_only/`](./Doula_DP_parts_only/Doula_DP_parts_only/)

This document provides the **exact pin-by-pin wiring instructions** for the entire Dual-Tier Knee Rehabilitation System using three **ESP32-S NodeMCU V1.1** boards (`U1: ESP_MASTER`, `U2: ESP_TOPHUB`, `U3: ESP_CYCLING`), along with exact potentiometer tuning procedures for the CC/CV Buck Converter (`U4`), signal resistor networks (`R1–R16`), steel chassis earthing/isolation rules, and a mandatory pre-power-on multimeter checklist.

---

## 1. Master Power Distribution & Regulator Tuning (`PSU1`, `U4` Buck, `U5` LDO)

### 1.1 24V 30A SMPS (`PSU1`) Connections
Your $24\,\text{V}\;30\,\text{A}$ ($720\,\text{W}$) SMPS has a 9-screw terminal strip. Wire it as follows:

| SMPS Terminal Label | Wire Gauge & Color | Destination / Connection | Critical Notes |
| :--- | :--- | :--- | :--- |
| **`L` (Line / Live)** | $1.5\,\text{mm}^2$ ($16\,\text{AWG}$) Red/Brown | $230\,\text{V}$ AC Mains Live (via $10\,\text{A}$ DPST Mains Switch + $6\,\text{A}$ Fuse) | Never leave `L` or `N` terminals exposed; use the plastic terminal cover |
| **`N` (Neutral)** | $1.5\,\text{mm}^2$ ($16\,\text{AWG}$) Black/Blue | $230\,\text{V}$ AC Mains Neutral | — |
| **`FG` / `PE` (`⏚`)** | $1.5\,\text{mm}^2$ ($16\,\text{AWG}$) Green-Yellow | Mains Earth **AND** bolted to the **Steel Chassis Earth Stud** | **Mandatory for patient safety on a steel frame!** Scrape paint to bare steel under a star washer |
| **`+V` (3 terminals)** | $12\,\text{AWG}$ ($3.3\text{–}4.0\,\text{mm}^2$) Red | **Insulated `+24V_BUS` Distribution Block** | Use separate $12\text{–}14\,\text{AWG}$ runs from the block to Rhino MD20Amp, Heatbed MOSFET, and Stepper Driver |
| **`-V` / `COM` (3 terminals)** | $12\,\text{AWG}$ ($3.3\text{–}4.0\,\text{mm}^2$) Black | **Insulated `STAR_GND` Busbar** | **Do NOT bolt `-V` (`DC GND`) to the steel chassis to carry current!** All returns must use insulated copper wire back to `STAR_GND` |
| **`+V ADJ` Trimpot** | Small Phillips Screwdriver | Fine-tune unloaded SMPS output to **$24.00\,\text{V}$ DC** | Measure with multimeter before connecting any downstream boards |

---

### 1.2 Tuning the 24V $\rightarrow$ 5V CC/CV Buck Converter (`U4`)
Your adjustable DC-DC Buck Converter has **four connections** (`IN+`, `IN-`, `OUT+`, `OUT-`) and **two blue multi-turn brass-screw potentiometers**:
1. **CV Trimpot (Constant Voltage):** Located closest to the output capacitor / `OUT+` side.
2. **CC Trimpot (Constant Current Limit):** Located closest to the current-sense shunt resistor / op-amp IC.

> [!CAUTION]
> **DO NOT CONNECT THE BUCK CONVERTER OUTPUT TO THE LDO, ESP32s, ENCODERS, OR ACS712 BEFORE TUNING!**
> Factory buck converters often ship set to $12\,\text{V}\text{–}20\,\text{V}$. Connecting an untuned buck converter will permanently destroy the $5\,\text{V}$ optical encoders, the ACS712 sensor, and the LDO/ESP32s in milliseconds.

#### Step-by-Step Potentiometer Tuning Procedure (With Output Disconnected)
1. **Wire Input Only:** Connect `IN+` to `+24V_BUS` ($18\text{–}20\,\text{AWG}$ Red) and `IN-` to `STAR_GND` ($18\text{–}20\,\text{AWG}$ Black). Leave `OUT+` and `OUT-` **completely disconnected**.
2. **Tune Voltage (CV Potentiometer) to $\mathbf{5.05\,\text{V}}$:**
   * Set your digital multimeter (DMM) to **DC Volts ($20\,\text{V}$ range)** and clip the probes to `OUT+` and `OUT-`.
   * Power on the $24\,\text{V}$ SMPS.
   * Turn the **CV (Voltage) potentiometer** counter-clockwise to lower the voltage (if the reading does not change initially, turn up to $10\text{–}15$ full turns until the internal worm gear catches).
   * Adjust slowly until the multimeter reads **exactly $5.05\,\text{V}$** (acceptable window: $5.00\,\text{V}\text{–}5.10\,\text{V}$).
3. **Tune Current Limit (CC Potentiometer) to $\mathbf{1.8\,\text{A}\text{–}2.0\,\text{A}}$:**
   * *Why $1.8\text{–}2.0\,\text{A}$?* Three ESP32s transmitting Wi-Fi/ESP-NOW draw $\sim 0.6\text{–}0.9\,\text{A}$ average with $1.4\,\text{A}$ RF peaks, two optical encoders draw $\sim 0.08\,\text{A}$, and the ACS712 draws $\sim 0.015\,\text{A}$ ($\text{Total} \approx 0.7\text{–}1.5\,\text{A}$). Setting CC too low ($< 1.0\,\text{A}$) will cause the $5\,\text{V}$ rail to drop out when Wi-Fi transmits; setting it to **$1.8\text{–}2.0\,\text{A}$** gives full headroom while protecting against accidental short circuits.
   * **How to Measure & Set CC Exactly:**
     1. Turn the **CC (Current) potentiometer** $15$ turns counter-clockwise (minimum current).
     2. Move your multimeter red probe into the **`10A` Fused Current Jack** and select **DC Amps ($10\,\text{A}$)**.
     3. Touch the multimeter probes across `OUT+` and `OUT-` of the buck converter (because CV is already $5.05\,\text{V}$ and CC is at minimum, the buck converter safely enters Constant-Current mode and its red `CC` LED lights up).
     4. Slowly turn the **CC potentiometer clockwise** until your multimeter reads **$1.90\,\text{A}$** ($1.80\text{–}2.00\,\text{A}$).
     5. Remove the probes, turn off power, and **immediately move your multimeter red lead back from the `10A` jack to the `V/Ω` jack**!
4. **Mark the Screws:** Put a small dot of nail polish or marker on both brass screws so chassis vibrations cannot untune them.

---

### 1.3 Wiring the 5V $\rightarrow$ 3.3V LDO Regulator (`U5`) & Powering the ESP32-S NodeMCU V1.1 Boards
1. **Buck to `+5V_BUCK` Rail:**
   * `U4` Buck `OUT+` ($5.05\,\text{V}$) $\rightarrow$ **`+5V_BUCK` Terminal Strip** (feeds `ENC1`, `ENC2`, `U203: ACS712-30A`, `U5` LDO `VIN`, and optional `HX711/INA125` expansion if installed).
   * `U4` Buck `OUT-` $\rightarrow$ Connected internally to `IN-` (`STAR_GND`).
2. **LDO (`U5`: $5\,\text{V} \rightarrow 3.3\,\text{V}$, rated $\ge 1.5\,\text{A}$ e.g. `LD1117V33` / `AMS1117-3.3` module or high-current $3.3\,\text{V}$ LDO):**
   * `U5` **`VIN` (`IN+`)** $\rightarrow$ Connect to **`+5V_BUCK`** ($5.05\,\text{V}$).
   * `U5` **`GND` (`IN-`/`OUT-`)** $\rightarrow$ Connect to **`STAR_GND`**.
   * `U5` **`VOUT` (`3.3V`)** $\rightarrow$ Verify with multimeter that it reads **$3.30\,\text{V} \pm 0.05\,\text{V}$**, then connect to the **`+3V3_LDO` Rail** feeding the **`3V3`** pin of `U1`, `U2`, and `U3`.
   * *Practical Capacitance Tip:* Place a $100\,\mu\text{F}$ electrolytic capacitor + $100\,\text{nF}$ ceramic capacitor between `3V3` and `GND` right next to each ESP32-S NodeMCU V1.1 board to absorb Wi-Fi/ESP-NOW current spikes.

> [!WARNING]
> **USB Flashing Precaution on ESP32-S NodeMCU V1.1:**
> Because each NodeMCU V1.1 has its own onboard AMS1117-3.3 regulator fed from the micro-USB port, **disconnect the external `+3V3_LDO` wire from the ESP32's `3V3` pin whenever you plug that ESP32 into your laptop's USB port for flashing** (or use a modified USB cable with the red $+5\,\text{V}$ wire cut, keeping only `D+`, `D-`, and `GND`). Alternatively, you may feed `+5V_BUCK` ($5.05\,\text{V}$) into the **`VIN` (`5V`)** pin of each NodeMCU V1.1 and let each board's onboard 3.3V LDO regulate its own $3.3\,\text{V}$ rail independently!

---

## 2. Exact Pinout & Wiring for All Three ESP32-S NodeMCU V1.1 Boards

On the **ESP32-S NodeMCU V1.1** development board, pins are labeled on the bottom/top silkscreen as `3V3`, `GND`, `VIN`, and `IOxx` / `Pxx` / `Dxx`.

> [!IMPORTANT]
> **Critical ESP32 Hardware Rule (`ADC1` vs. `ADC2`):**
> Because Wi-Fi and ESP-NOW are active on all three ESP32s, **`ADC2` pins (`GPIO0, 2, 4, 12, 13, 14, 15, 25, 26, 27`) CANNOT be used for `analogRead()`**! Every analog sensor (`VBUS_SENSE`, `ACS712`, Analog Strain Gauge) **must** be wired to **`ADC1` pins (`GPIO34`, `GPIO35`, `GPIO36/VP`, `GPIO39/VN`)**. Notice also that `GPIO34`, `GPIO35`, `GPIO36`, and `GPIO39` are **input-only** pins and have no internal pull-up/pull-down resistors.

---

### 2.1 `U1`: Master ESP32 (`ESP_MASTER` — Web GUI Server & ESP-NOW Coordinator)
`U1` runs the Wi-Fi SoftAP (`192.168.4.1`), serves the Clinical Web GUI, and coordinates `U2` and `U3` over ESP-NOW.

| `U1` NodeMCU V1.1 Pin Silkscreen | GPIO # | Connected Net / Wire | Destination |
| :--- | :---: | :--- | :--- |
| **`3V3`** (or **`VIN`** if using $5\,\text{V}$) | Power | `+3V3_LDO` ($3.30\,\text{V}$) | Output of `U5` $3.3\,\text{V}$ LDO |
| **`GND`** (Wire at least 2 `GND` pins) | Ground | `STAR_GND` | Central DC Ground Busbar |
| **`IO4`** *(Optional Physical E-Stop Button)* | `GPIO4` | `HW_ESTOP_BTN` | Optional physical mushroom E-Stop switch to `GND` (`INPUT_PULLUP`) |

---

### 2.2 `U2`: Top-Hub ESP32 (`ESP_TOPHUB` — Knee Motor, Chopper, ACS712, Encoder 1, Spasm Sensor)

| `U2` NodeMCU V1.1 Pin Silkscreen | GPIO # | Signal Direction | Net Label | Exact Destination |
| :--- | :---: | :---: | :--- | :--- |
| **`3V3`** | Power | Power In | `+3V3_LDO` | Output of `U5` $3.3\,\text{V}$ LDO |
| **`GND`** (Use 2 pins) | Ground | Return | `STAR_GND` | Central DC Ground Busbar |
| **`IO34` / `P34`** (`ADC1_CH6`) | **`GPIO34`** | Analog In ($0\text{–}3.3\,\text{V}$) | `VBUS_SENSE` | Midpoint of **`R11` ($3\times 220\,\text{k}\Omega$ in parallel $= 73.33\,\text{k}\Omega$) & `R12` ($1\times 4.6\,\text{k}\Omega$)** Bus Voltage Divider |
| **`IO35` / `P35`** (`ADC1_CH7`) | **`GPIO35`** | Analog In ($0\text{–}3.3\,\text{V}$) | `ACS712_SENSE` | Midpoint of **`R13` ($1\times 22\,\text{k}\Omega$) & `R14` ($1\times 22\,\text{k}\Omega$)** from `ACS712` `OUT` pin |
| **`IO32` / `P32`** (`PCNT0`) | **`GPIO32`** | Digital In ($3.3\,\text{V}$ `INPUT_PULLUP`) | `ENC1_CHA_3V3` | **NPN Open-Collector:** `ENC1` `OUT_A` (Green) via **`R3` ($80\,\Omega$ series damping)** to `GPIO32` *(NO resistor to `GND`! Enable `INPUT_PULLUP` in code)* |
| **`IO33` / `P33`** (`PCNT0`) | **`GPIO33`** | Digital In ($3.3\,\text{V}$ `INPUT_PULLUP`) | `ENC1_CHB_3V3` | **NPN Open-Collector:** `ENC1` `OUT_B` (White) via **`R5` ($80\,\Omega$ series damping)** to `GPIO33` *(NO resistor to `GND`! Enable `INPUT_PULLUP` in code)* |
| **`IO25` / `P25`** (`MCPWM`) | **`GPIO25`** | PWM Out ($20\,\text{kHz}$) | `MD_PWM` | **Rhino MD20Amp** `PWM` pin (via **`R15` = $1\times 220\,\text{k}\Omega$** pull-down to `GND`) |
| **`IO26` / `P26`** | **`GPIO26`** | Digital Out | `MD_DIR` | **Rhino MD20Amp** `DIR` pin |
| **`IO27` / `P27`** (`LEDC` $5\,\text{kHz}$) | **`GPIO27`** | PWM Out ($5\,\text{kHz}$) | `CHOP_PWM` | **30A Heatbed MOSFET** `Control +` pin (with **`R16` = $1\times 22\,\text{k}\Omega$** pull-down to `GND`) |

*(Note: Spasm Detection and Setup Calibration on `U2` operate using the **ACS712-30A Current Sensor (`GPIO35`)** + **Motor-Shaft Optical Encoder (`GPIO32/33`, $74,880\,\text{counts/output rev}$)**; `GPIO36` or `GPIO21/22` remain free if you ever add an optional shin-pad load cell later).*

---

### 2.3 `U3`: Bottom-Cycling ESP32 (`ESP_CYCLING` — TB6600/DM542 Stepper Eddy Brake & Flywheel Encoder 2)

| `U3` NodeMCU V1.1 Pin Silkscreen | GPIO # | Signal Direction | Net Label | Exact Destination |
| :--- | :---: | :---: | :--- | :--- |
| **`3V3`** | Power | Power In | `+3V3_LDO` | Output of `U5` $3.3\,\text{V}$ LDO |
| **`GND`** (Use 2 pins) | Ground | Return | `STAR_GND` | Central DC Ground Busbar (and `TB6600/DM542` `PUL-` & `DIR-`) |
| **`IO32` / `P32`** (`PCNT0`) | **`GPIO32`** | Digital In ($3.3\,\text{V}$ `INPUT_PULLUP`) | `ENC2_CHA_3V3` | **NPN Open-Collector:** `ENC2` `OUT_A` (Green) via **`R7` ($80\,\Omega$ series damping)** to `GPIO32` *(NO resistor to `GND`!)* |
| **`IO33` / `P33`** (`PCNT0`) | **`GPIO33`** | Digital In ($3.3\,\text{V}$ `INPUT_PULLUP`) | `ENC2_CHB_3V3` | **NPN Open-Collector:** `ENC2` `OUT_B` (White) via **`R9` ($80\,\Omega$ series damping)** to `GPIO33` *(NO resistor to `GND`!)* |
| **`IO18` / `P18`** | **`GPIO18`** | Pulse Out | `STEP_PUL` | **TB6600 / DM542 Stepper Driver (`U301`)** **`PUL+`** input |
| **`IO19` / `P19`** | **`GPIO19`** | Digital Out | `STEP_DIR` | **TB6600 / DM542 Stepper Driver (`U301`)** **`DIR+`** input |

---

## 3. Component-by-Component Detailed Wiring Instructions

### 3.1 Top-Hub Motor Driver (`U201`: Rhino MD20Amp) & `ACS712 30A` Current Sensor Module
The **Rhino MD20Amp** has 4 large power screw terminals (`+VCC`, `GND`, `M+`, `M-`) and a 3-pin logic header (`PWM`, `DIR`, `GND`).
The **ACS712 30A Module** has 2 large green screw terminals (`IP+`, `IP-`) for the high-current path and a 3-pin male header (`VCC`, `OUT`, `GND`).

```mermaid
flowchart LR
    BUS["+24V_BUS (SMPS +V)"] ==>|"12 AWG Red"| VCC["Rhino MD20Amp: +VCC"]
    SGND["STAR_GND (SMPS -V)"] ==>|"12 AWG Black"| DGND["Rhino MD20Amp: Power GND"]
    
    MPLUS["Rhino MD20Amp: M+"] ==>|"12 AWG Red"| IPP["ACS712-30A: IP+ Screw"]
    IPM["ACS712-30A: IP- Screw"] ==>|"12 AWG Red"| MOTP["MY1016 Motor: (+) Red Wire"]
    MMINUS["Rhino MD20Amp: M-"] ==>|"12 AWG Black"| MOTM["MY1016 Motor: (-) Black Wire"]
```

1. **High-Current Power Side ($12\,\text{AWG}$ stranded copper wire with crimped ferrules/spades):**
   * **Rhino `+VCC`** $\rightarrow$ Connect directly to **`+24V_BUS`**.
   * **Rhino Power `GND`** $\rightarrow$ Connect directly to **`STAR_GND`**.
   * **Rhino `M+`** $\rightarrow$ Connect to **`IP+` screw terminal of the `ACS712 30A` module**.
   * **`ACS712 30A` `IP-` screw terminal** $\rightarrow$ Connect to **MY1016 Motor `+` (Red) lead**.
   * **Rhino `M-`** $\rightarrow$ Connect directly to **MY1016 Motor `-` (Black) lead**.
   * *(Why wire the ACS712 between `M+` and the Motor instead of on `+VCC`? Because during Mode 2 dynamic braking, the motor current circulates through the motor and the Rhino's lower MOSFETs while `+VCC` current is zero. Placing the ACS712 inline with `M+` measures true motor current in all 4 modes!)*
2. **Rhino Logic Side:**
   * **Rhino `PWM`** $\rightarrow$ Connect to `U2` **`GPIO25`** (`IO25`). **Add your 4th $220\,\text{k}\Omega$ pull-down resistor (`R15`) between `GPIO25` and `GND`** so the motor stays at $0\%$ PWM during the $200\,\text{ms}$ ESP32 bootup before GPIOs initialize!
   * **Rhino `DIR`** $\rightarrow$ Connect to `U2` **`GPIO26`** (`IO26`).
   * **Rhino Logic `GND`** $\rightarrow$ Connect to `U2` **`GND`**.
3. **ACS712 30A Low-Voltage Side & Level-Shifting Resistors (`R13`, `R14`):**
   * **ACS712 `VCC`** $\rightarrow$ Connect to **`+5V_BUCK`** ($5.05\,\text{V}$).
   * **ACS712 `GND`** $\rightarrow$ Connect to `U2` **`GND`**.
   * **ACS712 `OUT`** $\rightarrow$ Do **not** wire directly to the ESP32! Wire through your **$22\,\text{k}\Omega / 22\,\text{k}\Omega$ ($1:1$, $0.50\times$) divider**:
     $$\text{ACS712 }\texttt{OUT} \xrightarrow{\quad R_{13} = 22\,\text{k}\Omega \quad} \bullet \xrightarrow{\quad \text{to } U_2\;\texttt{GPIO35} \quad} \qquad \text{and from } \bullet \xrightarrow{\quad R_{14} = 22\,\text{k}\Omega \quad} \texttt{GND}$$
     *(Optional Noise Filter: Place a $10\,\text{nF}$ ($0.01\,\mu\text{F}$) ceramic capacitor in parallel with $R_{14}$ from `GPIO35` to `GND` to filter $20\,\text{kHz}$ PWM switching spikes).*

---

### 3.2 30A Heatbed MOSFET Brake Chopper (`U202`, 2-Screw-Terminal Variant) & Braking Resistors (`R1 || R2`)
Your 30A Heatbed MOSFET Module has **two high-current screw terminals** (`Power +` [Drain] and `Power -` [Source]) and **two small control pins** (`Control +` and `Control -`).

> [!CAUTION]
> **CRITICAL WIRING RULE FOR YOUR 2-SCREW-TERMINAL MOSFET MODULE:**
> Because your MOSFET module only has **one `Power +` and one `Power -` screw terminal**, it acts as a low-side switch between `Power +` (Drain) and `Power -` (Source). **NEVER wire `Power +` directly to `+24V_BUS` and `Power -` to `GND`**, or turning on the MOSFET will dead-short the $24\,\text{V}$ power supply! Instead, the **$1.0\,\Omega$ Braking Resistor Bank (`R1 || R2`) MUST be wired IN SERIES between `+24V_BUS` and `Power +`**!

```mermaid
flowchart LR
    BUS["+24V_BUS (SMPS +V)"] ==>|"12 AWG Red"| RA["Braking Resistors (R1 || R2 = 1.0Ω)\nLead A"]
    RB["Braking Resistors (R1 || R2 = 1.0Ω)\nLead B"] ==>|"12 AWG Orange/Red"| MP["30A MOSFET Module:\nPower + Screw (Drain)"]
    MM["30A MOSFET Module:\nPower - Screw (Source)"] ==>|"12 AWG Black"| SGND["STAR_GND (SMPS -V)"]
```

#### 1. High-Current Series Loop Wiring ($12\text{–}14\,\text{AWG}$ Wire)
* **Step 1 — Wire the two $2\,\Omega, 100\,\text{W}$ Aluminum-Housed Resistors (`R1` and `R2`) in PARALLEL:**
  * Tie the left terminal of `R1` and the left terminal of `R2` together $\rightarrow$ **Resistor Bank Lead A**.
  * Tie the right terminal of `R1` and the right terminal of `R2` together $\rightarrow$ **Resistor Bank Lead B**.
  * **Multimeter Check:** Measure resistance between **Lead A** and **Lead B**—it must read **$1.0\,\Omega \pm 0.15\,\Omega$**.
* **Step 2 — Wire the Series Chopper Path:**
  * Connect **`+24V_BUS`** directly to **Resistor Bank Lead A** ($12\text{–}14\,\text{AWG}$ Red).
  * Connect **Resistor Bank Lead B** to the MOSFET Module's **`Power +` (Drain)** screw terminal ($12\text{–}14\,\text{AWG}$ Orange/Red).
  * Connect the MOSFET Module's **`Power -` (Source)** screw terminal directly to **`STAR_GND`** ($12\text{–}14\,\text{AWG}$ Black).

#### 2. The Two Control Pins (`Control +` and `Control -`)
* **`Control +` (Red wire / `+` mark on 2-pin JST header):** Connect directly to `U2` **`GPIO27`** (`IO27`). *(Do NOT add the $330\,\Omega$ resistor in series here, because 3D-printer heatbed MOSFET modules already have an internal $1\text{–}2.2\,\text{k}\Omega$ optocoupler resistor on the board and need the full $3.30\,\text{V}$ headroom from `GPIO27`!)*
* **`Control -` (Black wire / `-` mark on 2-pin JST header):** Connect to `U2` **`GND`**.
* **Mandatory Pull-Down Resistor (`R16`):** Solder your **3rd $22\,\text{k}\Omega$ resistor between `GPIO27` and `GND`**. This guarantees that while the ESP32 is resetting or turned off, the MOSFET gate stays strictly OFF so the $1.0\,\Omega$ resistor bank does not continuously dump $576\,\text{W}$ from the $24\,\text{V}$ supply!
* **Important Practical Check on the Heatbed Module's Optocoupler Voltage:**
  Some 3D printer heatbed MOSFET modules have an onboard series resistor ($R_{\text{opto}} = 1\,\text{k}\Omega\text{–}4.7\,\text{k}\Omega$) on the `Control +/-` pins designed for $12\text{–}24\,\text{V}$ 3D printer mainboards.
  * **Bench Test:** With $24\,\text{V}$ connected to the series loop, touch `Control +` to $+3.3\,\text{V}$ briefly (< $0.5\,\text{s}$).
  * If the small red indicator LED on the MOSFET module lights up brightly and `Power +` pulls down to $0\,\text{V}$, your module triggers directly from $3.3\,\text{V}$!
  * If $3.3\,\text{V}$ is too low to turn the internal optocoupler fully ON, drive the `Control -` pin with a simple `2N2222` / `BC547` NPN transistor whose collector goes to `Control -`, emitter to `GND`, base via your spare $330\,\Omega$ resistor to `GPIO27`, and tie `Control +` to **`+5V_BUCK`** (or `+24V_BUS`).

---

### 3.3 Opto-Isolated Stepper Motor Driver (`U301`: TB6600 / DM542) & $1.2\,\text{N}\cdot\text{m}$ Stepper Motor (`M2`)

#### 1. Power & Motor Coils ($16\text{–}18\,\text{AWG}$)
* **`VCC` / `+V`** $\rightarrow$ Connect to **`+24V_BUS`**.
* **`GND`** $\rightarrow$ Connect to **`STAR_GND`**.
* **`A+` and `A-`** $\rightarrow$ Connect to **Coil A** of the $1.2\,\text{N}\cdot\text{m}$ Stepper Motor *(Verify Coil A pair with a multimeter: the two motor wires that have $\sim 1.5\text{–}3.5\,\Omega$ continuity between them belong to Coil A)*.
* **`B+` and `B-`** $\rightarrow$ Connect to **Coil B** of the Stepper Motor *(The other pair of motor wires with $\sim 1.5\text{–}3.5\,\Omega$ continuity)*.
* **Current DIP Switch Setting on Driver:** Set the driver's peak/RMS current DIP switches (`SW1–SW3`) to match your $1.2\,\text{N}\cdot\text{m}$ stepper's rated current (typically **$2.0\,\text{A}\text{–}2.8\,\text{A}$ peak**), and set microstepping (`SW4–SW6`) to **$8\times$ ($1600\,\text{steps/rev}$)** or **$4\times$ ($800\,\text{steps/rev}$)**.

#### 2. Opto-Isolated Control Signals from `U3` (`ESP_CYCLING`) to `TB6600 / DM542`
* Jumper **`PUL-`** and **`DIR-`** together and connect them to `U3` **`GND`**.
* Connect **`PUL+`** to `U3` **`GPIO18`** (`IO18`).
* Connect **`DIR+`** to `U3` **`GPIO19`** (`IO19`).
* Leave **`ENA+`** and **`ENA-`** **completely unconnected** (on TB6600/DM542, leaving `ENA` unconnected enables the driver by default; driving `ENA` disables holding torque).

---

### 3.4 Both 360 PPR NPN Open-Collector Optical Encoders (`ENC1` on Motor Shaft & `ENC2` on Flywheel)

Both of your $360\,\text{PPR}$ incremental rotary encoders are **NPN Open-Collector** (`LPD3806-360BM-G5-24C` type) with a shielded 5-wire cable:
* **Red Wire:** $+5\,\text{V}$ DC Power $\rightarrow$ Connect to **`+5V_BUCK`** ($5.05\,\text{V}$).
* **Black Wire:** $0\,\text{V}$ Common $\rightarrow$ Connect to **`STAR_GND`** (at the respective ESP32).
* **Green Wire:** **Phase A Open-Collector Output (`OUT_A`)**
* **White Wire:** **Phase B Open-Collector Output (`OUT_B`)**
* **Bare Braided Shield Wire:** Connect to **`GND` at the ESP32 end ONLY** (leave the encoder-body end of the shield unconnected so chassis currents cannot flow through the cable shield!).

#### Exact Wiring for High-Speed Motor-Shaft `ENC1` & Flywheel `ENC2`
Because your encoders are **NPN Open-Collector**, they **never output $+5\,\text{V}$** on `OUT_A` or `OUT_B`—they only pull down to `GND` ($0\,\text{V}$) when active and float when inactive.
* **CRITICAL RULE 1:** **Do NOT wire any resistor from `GPIO32` or `GPIO33` to `GND`!**
* **CRITICAL RULE 2 (Why You Should Use `80Ω` Series Damping + `INPUT_PULLUP` and NOT Low-Ohm $117\,\Omega\text{–}330\,\Omega$ Pull-Ups):**
  Many industrial $360\,\text{PPR}$ NPN encoders (`LPD3806-360BM-G5-24C`) contain an internal `LM393` comparator ($I_{\text{sink,min}} = 6\,\text{mA}$) or an internal $50\text{–}100\,\Omega$ collector protection resistor. If you used a low-ohm pull-up like $234\,\Omega$ ($2\times 117\,\Omega$) or $330\,\Omega$ to $+3.3\,\text{V}$, it would demand $10\text{–}14\,\text{mA}$ of sink current and form a voltage divider with the internal protection resistor + external $80\,\Omega$ resistor ($V_{\text{low}} \approx 3.3\,\text{V} \times \frac{180}{414} = 1.43\,\text{V}$), which is **above the ESP32's maximum logic-LOW threshold ($V_{IL,\max} = 0.825\,\text{V}$)** and would prevent the ESP32 from registering pulses!
  By using only your **four $80\,\Omega$ resistors (`R3, R5, R7, R9`) in series** and enabling the ESP32's internal $45\,\text{k}\Omega$ pull-up (`pinMode(pin, INPUT_PULLUP)`) to $+3.30\,\text{V}$, the low voltage pulls cleanly down to **$0.013\,\text{V}$** while the RC rise time ($\sim 4.5\text{–}9\,\mu\text{s}$) is more than $8\times$ faster than the $80\,\mu\text{s}$ half-period at maximum clinical knee speed ($120^\circ/\text{s} \implies 1040\,\text{RPM}_{\text{motor}} = 6.24\,\text{kHz}$)!
  1. **On `U2` (`ENC1` — High-Speed Motor Shaft):**
     * Wire `ENC1` `OUT_A` (Green) $\xrightarrow{\;R_3 = 80\,\Omega\;}$ **`U2` `GPIO32`** (with `pinMode(32, INPUT_PULLUP)` enabled in `U2` code; **no resistor to `GND`**).
     * Wire `ENC1` `OUT_B` (White) $\xrightarrow{\;R_5 = 80\,\Omega\;}$ **`U2` `GPIO33`** (with `pinMode(33, INPUT_PULLUP)` enabled in `U2` code; **no resistor to `GND`**).
  2. **On `U3` (`ENC2` — Flywheel Shaft, $\le 375\,\text{RPM} = 2.25\,\text{kHz}$):**
     * Wire `ENC2` `OUT_A` (Green) $\xrightarrow{\;R_7 = 80\,\Omega\;}$ **`U3` `GPIO32`** (with `pinMode(32, INPUT_PULLUP)` enabled in `U3` code; **no resistor to `GND`**).
     * Wire `ENC2` `OUT_B` (White) $\xrightarrow{\;R_9 = 80\,\Omega\;}$ **`U3` `GPIO33`** (with `pinMode(33, INPUT_PULLUP)` enabled in `U3` code; **no resistor to `GND`**).

---

### 3.5 Complete Summary Table of All Signal Resistors (Verified for Your Exact Hardware)

Your exact resistor inventory on hand: **$4 \times 220\,\text{k}\Omega$**, **$3 \times 22\,\text{k}\Omega$**, **$1 \times 4.6\,\text{k}\Omega$**, **$4 \times 80\,\Omega$**, **$4 \times 117\,\Omega$**, and **$1 \times 330\,\Omega$**.

| Resistor ID | Your Exact Resistor(s) Used | Pin 1 Connection | Pin 2 Connection | Exact Voltage / Function |
| :---: | :--- | :--- | :--- | :--- |
| **`R1`, `R2`** | **$2\times 2.0\,\Omega, 100\,\text{W}$** Aluminum-Shell | **`+24V_BUS`** (Lead A) | **MOSFET `Power +` [Drain]** (Lead B) | Wired **in parallel** ($R_{\text{eq}} = 1.0\,\Omega, 200\,\text{W}$), placed **in series** with the 2-screw MOSFET |
| **`R11`** | **$3 \times 220\,\text{k}\Omega$ in Parallel** ($= \mathbf{73.33\,\text{k}\Omega}$) | `+24V_BUS` | `U2` `GPIO34` (`VBUS_SENSE`) | Upper leg of Bus Voltage Divider: $24.0\,\text{V} \rightarrow \mathbf{1.417\,\text{V}}$ (`VBUS_MULT = 16.942f`) |
| **`R12`** | **$1 \times 4.6\,\text{k}\Omega$** | `U2` `GPIO34` (`VBUS_SENSE`) | `STAR_GND` | Lower leg of Bus Voltage Divider (**Solder `R12` securely—never leave loose!**) |
| **`R13`** | **$1 \times 22\,\text{k}\Omega$** | `ACS712-30A` `OUT` pin | `U2` `GPIO35` (`ACS712_SENSE`) | Upper leg of $1:1$ Current Sensor Divider ($0.50\times$ scaling) |
| **`R14`** | **$1 \times 22\,\text{k}\Omega$** | `U2` `GPIO35` (`ACS712_SENSE`) | `STAR_GND` | Lower leg of Current Sensor Divider: **$1.250\,\text{V}$ at $0\,\text{A}$**, **$33.0\,\text{mV/A}$** sensitivity |
| **`R15`** *(Safety)* | **$1 \times 220\,\text{k}\Omega$** *(4th $220\,\text{k}\Omega$ resistor)* | `U2` `GPIO25` (`MD_PWM`) | `STAR_GND` | Pull-down resistor keeping Rhino MD20Amp OFF during ESP32 boot/reset |
| **`R16`** *(Safety)* | **$1 \times 22\,\text{k}\Omega$** *(3rd $22\,\text{k}\Omega$ resistor)* | `U2` `GPIO27` (`CHOP_PWM`) | `STAR_GND` | Pull-down resistor keeping 30A Brake Chopper MOSFET OFF during ESP32 boot/reset |
| **`R3, R5, R7, R9`** | **$4 \times 80\,\Omega$** *(1 in series per encoder wire)* | `ENC1/ENC2` `OUT_A` & `OUT_B` | `U2/U3` `GPIO32` & `GPIO33` | Series line-damping & protection for all 4 NPN Open-Collector encoder channels (`INPUT_PULLUP`) |
| *Spares* | **$4 \times 117\,\Omega$ & $1 \times 330\,\Omega$** | *Kept in parts bin* | *Do NOT place on encoder lines* | Kept as spares (low-ohm pull-ups would prevent NPN encoders from pulling $< 0.8\,\text{V}$) |

---

## 4. Practical Wiring Guidelines for a Conductive Steel Chassis

Because your entire mechanical frame is fabricated from **mild steel**, following strict industrial earthing, isolation, and EMI practices will prevent electrical shocks, blown ESP32s, and erratic encoder counts:

1. **Bond AC Protective Earth (`PE` / `FG`) to the Steel Frame—Never `DC GND` as a Current Path:**
   * Drill a dedicated **M5 or M6 Earth Stud** on the main steel chassis. Grind away any paint, primer, or rust in a $15\,\text{mm}$ circle around the hole down to shiny bare steel.
   * Crimp ring terminals onto two Green-Yellow $16\,\text{AWG}$ wires: run one from the 3-pin AC Mains inlet `PE` pin to the SMPS `FG` (`⏚`) terminal, and the second from `FG` to the Steel Chassis Earth Stud with a serrated star lock-washer.
   * **Keep `DC GND` (`SMPS -V`) on Insulated Wires:** Do **not** bolt motor negative wires to the steel frame like a car chassis. Every component's `GND` must travel through insulated black copper wire back to a single **Insulated Copper `STAR_GND` Busbar** right next to the SMPS `-V` terminals.
2. **Bolt the $100\,\text{W}$ Aluminum Braking Resistors (`R1`, `R2`) to the Steel Frame as a Heatsink—With Insulated Lugs:**
   * The gold aluminum shells of `R1` and `R2` are electrically isolated from their internal resistance wire ($\text{dielectric rating} > 1000\,\text{V}$), so bolting their flat bases directly to a thick steel chassis member (with a thin layer of thermal paste) turns your steel frame into a massive heatsink!
   * **Precaution:** The exposed solder tabs sticking out the ends of `R1` and `R2` are only $4\,\text{mm}$ above the steel surface. Slip **heat-shrink tubing** over the solder joints so a bumped wire can never short `+24V_BUS` to the steel chassis!
3. **Separate High-Current PWM Wires from Encoder & Sensor Wires:**
   * The wires between the **Rhino MD20Amp (`M+`, `M-`) and MY1016 Motor** carry $20\,\text{A}$ chopped at $20\,\text{kHz}$. **Twist the Red (`M+`) and Black (`M-`) motor wires tightly together** (3–4 twists per $10\,\text{cm}$) along their entire length to cancel radiated magnetic fields.
   * Route the **Encoder (`ENC1`, `ENC2`), `ACS712`, and Strain Gauge** cables along the **opposite side** of the steel tube (at least $10\,\text{cm}$ away from the motor power wires). If a sensor wire must cross a power wire, cross them at a **$90^\circ$ right angle**—never bundle them parallel in the same zip-tie!
4. **Do NOT Enclose the ESP32 Antennas Inside a Closed Steel Box:**
   * Steel acts as a Faraday cage for $2.4\,\text{GHz}$ Wi-Fi and ESP-NOW signals. Mount `U1`, `U2`, and `U3` inside **plastic (ABS / Acrylic / 3D-printed PLA/PETG) enclosures** mounted on Nylon standoffs, with the black PCB antenna end of each ESP32-S NodeMCU V1.1 sticking clear of any metal plates by at least $3\,\text{cm}$.
5. **Protect Wires Passing Through Steel Holes:**
   * Sharp drilled or laser-cut steel edges will vibrate through PVC wire insulation and cause a dead short. Always insert **rubber grommets** or wrap **spiral cable wrap / corrugated conduit** wherever wires pass around steel corners or near the moving $1:52$ top-hub arm and $1:5$ bottom cycling gears.

---

## 5. Mandatory Pre-Power-On Multimeter Checklist (Staged Bring-Up Protocol)

**Never power on the entire system at once on the first try.** Follow this 5-stage bring-up checklist in exact order using a digital multimeter:

### Stage A: Unpowered Continuity & Resistance Checks (Mains Cord Unplugged!)
- [ ] **1. Steel Chassis Isolation Check:** Set DMM to Continuity / Resistance ($\Omega$). Measure between the **Steel Chassis** and:
  * **`+24V_BUS`:** Must read **Open Circuit ($\text{OL}$ / $> 1\,\text{M}\Omega$)**.
  * **`+5V_BUCK`:** Must read **Open Circuit ($\text{OL}$)**.
  * **`+3V3_LDO`:** Must read **Open Circuit ($\text{OL}$)**.
  * **Resistor Bank Leads (`Lead A` and `Lead B`):** Must read **Open Circuit ($\text{OL}$)** to the steel frame.
  * **AC Mains Plug Earth Pin (`PE`):** Must read **$< 0.5\,\Omega$ (Beep!)** to the steel frame.
- [ ] **2. Power Rail Short-Circuit Check:** Measure resistance across each power rail before applying power:
  * **`+24V_BUS` to `STAR_GND`:** Must read high resistance ($> 5\,\text{k}\Omega$; it may slowly climb as SMPS capacitors charge from the DMM).
  * **`+5V_BUCK` to `STAR_GND`:** Must read $> 1\,\text{k}\Omega$ (not $0\,\Omega$!).
  * **`+3V3_LDO` to `STAR_GND`:** Must read $> 500\,\Omega$ (not $0\,\Omega$!).
- [ ] **3. Voltage Divider & Pull-Down Ground Integrity Check (`R12`, `R14`, `R15`, `R16`):**
  * Measure resistance from `U2` **`GPIO34`** (`VBUS_SENSE`) to `GND`: **Must read $4.6\,\text{k}\Omega \pm 5\%$** (with `R11` disconnected, or $\approx 4.33\,\text{k}\Omega$ in-circuit). *(If this reads $\text{OL}$, `R12` is not connected to `GND` and $24\,\text{V}$ would destroy `GPIO34`!)*
  * Measure resistance from `U2` **`GPIO34`** to `+24V_BUS`: **Must read $73.3\,\text{k}\Omega \pm 5\%$** (`R11` = $3\times 220\,\text{k}\Omega$ in parallel).
  * Measure resistance from `U2` **`GPIO35`** (`ACS712_SENSE`) to `GND`: **Must read $22.0\,\text{k}\Omega \pm 5\%$** (`R14`, or $\approx 11.0\,\text{k}\Omega$ when `ACS712` is powered/low-impedance).
  * Measure resistance from `U2` **`GPIO25`** (`MD_PWM`) to `GND`: **Must read $220\,\text{k}\Omega \pm 5\%$** (`R15` boot pull-down).
  * Measure resistance from `U2` **`GPIO27`** (`CHOP_PWM`) to `GND`: **Must read $22.0\,\text{k}\Omega \pm 5\%$** (`R16` boot pull-down).
- [ ] **4. Braking Resistor Bank Check:**
  * Measure across **`Lead A` and `Lead B`** of the parallel braking resistors (`R1 || R2`): **Must read $1.0\,\Omega\text{–}1.2\,\Omega$**.

---

### Stage B: Power Stage Bring-Up (With ESP32s, Encoders & Motors Unplugged)
- [ ] **1. Disconnect** the ESP32s from their sockets/headers, unplug the motor leads (`M+`, `M-`), and disconnect the Buck Converter output wire.
- [ ] **2. Plug in AC Mains & Switch On SMPS:**
  * Measure `+24V_BUS` to `STAR_GND`: Adjust SMPS trimpot to **$24.00\,\text{V}$ DC**.
  * Measure voltage at the `VBUS_SENSE` wire (`GPIO34` node) to `GND`: **Must read $\mathbf{1.42\,\text{V}}$ DC** (with `R11` = $3\times 220\,\text{k}\Omega$ in parallel $= 73.33\,\text{k}\Omega$ and `R12` = $4.6\,\text{k}\Omega$). This confirms `GPIO34` is $100\%$ safe!
- [ ] **3. Tune Buck Converter (`U4`) & Verify LDO (`U5`):**
  * Tune `U4` **CV pot** to **$5.05\,\text{V}$** and **CC pot** to **$1.9\,\text{A}$** (per Section 1.2).
  * Turn off power, connect `U4` `OUT+` to `+5V_BUCK` (feeding `U5` LDO and `ACS712`), and power back on.
  * Measure `+3V3_LDO` output of `U5`: **Must read $\mathbf{3.30\,\text{V} \pm 0.05\,\text{V}}$**.
  * Measure `ACS712` raw `OUT` pin ($I_m = 0\,\text{A}$): **Must read $\mathbf{2.52\,\text{V}}$** ($0.5 \times 5.05\,\text{V}$).
  * Measure `ACS712_SENSE` (`GPIO35` node after the $22\,\text{k}\Omega / 22\,\text{k}\Omega$ divider): **Must read $\mathbf{1.25\text{–}1.26\,\text{V}}$ DC**—right in the sweet spot for `U2`!

---

### Stage C: Sensor & Encoder Bring-Up (Motors Still Disconnected)
- [ ] **1. Power off**, install `U1`, `U2`, and `U3` (ESP32-S NodeMCU V1.1 boards), and connect `ENC1` and `ENC2`.
- [ ] **2. Power on** and rotate the Top-Hub shaft (`ENC1`) and Flywheel shaft (`ENC2`) slowly by hand while measuring `GPIO32` and `GPIO33` on `U2` and `U3`:
  * Both pins must toggle cleanly between **$\approx 0.0\,\text{V}$ and $\approx 3.0\text{–}3.3\,\text{V}$** (never above $3.35\,\text{V}$!).
- [ ] **3. Check Brake Chopper Idle State:**
  * Verify that the red indicator LED on the **30A Heatbed MOSFET (`U202`)** is **OFF** and the aluminum braking resistors (`R1`, `R2`) remain completely cold at room temperature.

---

### Stage D: Motor & Direction Verification (With Orthosis Arm Detached from Hard Stop / Patient)
- [ ] **1. Power off** and connect the MY1016 motor (through the ACS712-30A) and the $1.2\,\text{N}\cdot\text{m}$ Stepper Motor.
- [ ] **2. Keep the Top-Hub Orthosis Arm in the middle of its travel ($\sim 45^\circ\text{–}60^\circ$)**, completely clear of the mechanical hard stop and with no limb attached.
- [ ] **3. Power on** and connect a phone/laptop to the Master ESP32 Wi-Fi network (`KneeRehab_Clinical_Hub` at `http://192.168.4.1`).
- [ ] **4. Run Stage 1 & Stage 2 of the Automated Setup Routine** on the Web GUI while keeping one hand on the $24\,\text{V}$ mains switch:
  * Confirm the arm creeps slowly ($\sim 3^\circ/\text{s}$) toward the hard stop, detects stall current ($\le 2.5\,\text{A}$) on the ACS712, backs off $5^\circ$, and latches the encoder angle cleanly.
