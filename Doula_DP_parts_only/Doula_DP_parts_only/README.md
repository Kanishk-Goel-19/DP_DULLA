# SolidWorks 3D CAD Assemblies, STEP Models, DXF Profiles & SKF Bearing Index

> **Repository Cross-Reference Index (Project Doula — `VK26-IITI-HT-0240`):**
> * **Master Top-Hub Architecture (`U2: ESP_TOPHUB`):** [`../../README.md`](../../README.md)
> * **Bottom Cycling & Eddy-Current Brake (`U3: ESP_CYCLING`):** [`../../README_CYCLING_EDDY_BRAKE.md`](../../README_CYCLING_EDDY_BRAKE.md)
> * **Integrated 3-ESP32 Network & Web GUI (`U1: ESP_MASTER`):** [`../../README_INTEGRATED_SYSTEM_AND_WEB_APP.md`](../../README_INTEGRATED_SYSTEM_AND_WEB_APP.md)
> * **Hardware Wiring & Commissioning Guide:** [`../../WIRING_AND_COMMISSIONING_GUIDE.md`](../../WIRING_AND_COMMISSIONING_GUIDE.md)
> * **Electrical Schematics & Master BOM:** [`../../System_Schematics_3Page.pdf`](../../System_Schematics_3Page.pdf) | [`../../schematic_page4_bom.svg`](../../schematic_page4_bom.svg)

This directory contains all **SolidWorks 2024/2025 3D CAD exports (`.STEP`, `.SLDASM`, `.SLDPRT`), 2D laser/waterjet cutting drawings (`.DXF`, `.SLDDRW`), and SKF Deep-Groove Ball Bearing models** for the **Dual-Tier Knee Rehabilitation System (Project Doula)**.

---

## 1. Production CAD Files vs. Baseline Revisions

### 1.1 Active Production CAD Files (Use for Manufacturing & Assembly)

| File Name | Subsystem & Schematic ID | Material / Process | Mechanical Function & Key Specifications |
| :--- | :--- | :--- | :--- |
| [`Whole_cycling _assembly_part-1.STEP`](./Whole_cycling%20_assembly_part-1.STEP) | **Integrated Dual-Tier Assembly** (`Sheet 2` + `Sheet 3`) | Mild Steel Frame + Al-6061 + PLA/Steel Gears | Complete 3D assembly integrating the Top-Hub knee orthosis (`M1`, `GBX1`, `STOP1`) and Bottom Cycling Eddy-Current Brake (`GR1`, `GP1`, `FLY1`, `STAT1`, `STAT2`, `M2`, `GBX2`). |
| [`arm_link-1_new.STEP`](./arm_link-1_new.STEP) | **Top-Hub Proximal Orthosis Arm Link 1** (`GBX1` Output / `STOP1`) | Mild Steel / Al-6061 Structural Arm | Mounts to the $1:52$ Top-Hub gearbox (`GBX1`) output shaft; interfaces with the mechanical hard stop (`STOP1`) at $\theta_{\text{stop}} = 0.0^\circ$ and carries the non-collinear offset pivot ($d_{\text{offset}} \approx 6\text{–}9\,\text{cm}$). |
| [`arm_link-2.STEP`](./arm_link-2.STEP) | **Top-Hub Distal Tibial/Calf Cuff Link 2** (`Sheet 2` Patient Interface) | Mild Steel / Al-6061 + Padded Calf Cuff | Adjustable distal link ($L_{\text{leg}} \approx 0.25\text{–}0.40\,\text{m}$ from hub to shin strap) that transmits assistive/resistive torque ($\tau_{\text{knee}} = F_{\text{pad}}\cdot L_{\text{leg}}$) to the lower leg. |
| [`NEW_gear_pedal_wheel.STEP`](./NEW_gear_pedal_wheel.STEP) | **Bottom-Unit $15\,\text{cm}$ Ring Gear** (`GR1`) | Steel / Heavy-Duty Spur Ring Gear | $D_1 = 150\,\text{mm}$ ($R_1 = 75\,\text{mm}$) primary pedal-driven ring gear driving the $1:5$ step-up stage (`GR1` $\rightarrow$ `GP1`). |
| [`pedal_gear.STEP`](./pedal_gear.STEP) | **Pedal Crank & Hub Gear Mount** (`GR1` Crank Interface) | Mild Steel | Couples the patient pedal crank ($L_{\text{crank}} = 0.15\text{–}0.17\,\text{m}$) to the $15\,\text{cm}$ ring gear (`GR1`). |
| [`pedal_attachment.STEP`](./pedal_attachment.STEP) | **Patient Foot Pedal & Orthotic Boot** (`GR1` Foot Interface) | Steel / Polymer Footplate | Footplate and heel-cup attachment with Velcro strap retention for post-operative cycling (`60–75 RPM` target cadence). |
| [`NEW_flywheel_and_gear.STEP`](./NEW_flywheel_and_gear.STEP) | **High-Speed $3\,\text{cm}$ Pinion (`GP1`) + $12\,\text{cm}$ Aluminum Eddy Disc (`FLY1`)** | Steel Pinion + Al-6061-T6 Disc ($b = 5\text{–}6\,\text{mm}$) | $D_2 = 30\,\text{mm}$ ($R_2 = 15\,\text{mm}$) driven pinion ($N_{\text{pedal}} = R_1/R_2 = 5.0$) coaxially coupled to the $D_{\text{Al}} = 120\,\text{mm}$ ($R_{\text{Al}} = 60\,\text{mm}$) non-ferromagnetic aluminum eddy-current rotor and `ENC2` ($360\,\text{PPR}$). |
| [`NEW_static_magneticplate&stand.STEP`](./NEW_static_magneticplate&stand.STEP) | **Fixed Stator Plate (`STAT1`) & Support Stand** | Mild Steel Back-Iron + Structural Stand | Holds 6 axially magnetized **N42/N52 NdFeB** magnets ($\varnothing 20\text{–}25\,\text{mm}\times 5\text{–}10\,\text{mm}$) in alternating `N–S–N–S–N–S` polarity facing `FLY1` across air gap $g \approx 1.5\text{–}2.0\,\text{mm}$, plus mounts for `M2` + `GBX2`. |
| [`rotating_magnetic_plate.STEP`](./rotating_magnetic_plate.STEP) | **Rotatable Stator Plate (`STAT2`) with Gear Sector** | Mild Steel Back-Iron + Gear Sector | Coaxial 6-magnet NdFeB array driven through $\phi \in [0^\circ, 60^\circ]$ by the $1.2\,\text{N}\cdot\text{m}$ stepper motor (`M2`) via the $1:3.6$ gearbox (`GBX2`) to modulate $\tau_{\text{pedal}}(\phi) = \tau_{\text{pedal,max}}\cos^2(3\phi/2)$. |
| [`spur gear_iso.DXF`](./spur%20gear_iso.DXF) & [`spur gear_iso.SLDDRW`](./spur%20gear_iso.SLDDRW) | **2D Involute Spur Gear Cutting Profile** (`GR1`, `GP1`, `GBX2`) | 2D DXF / SolidWorks Drawing | 2D profile for laser/waterjet/wire-EDM fabrication of the $1:5$ step-up gear pair (`GR1`, `GP1`) and $1:3.6$ stepper reduction stage (`GBX2`). |

---

### 1.2 Baseline / Superseded CAD Iterations (Retained for Design History)

| Legacy File Name | Superseded By (Production File) | Revision Notes |
| :--- | :--- | :--- |
| [`flywheel_and_gear.STEP`](./flywheel_and_gear.STEP) | [`NEW_flywheel_and_gear.STEP`](./NEW_flywheel_and_gear.STEP) | Initial flywheel/pinion hub geometry prior to bearing seat (`SKF 6205-Z`) and air-gap boss refinement. |
| [`gear_wheel.STEP`](./gear_wheel.STEP) | [`NEW_gear_pedal_wheel.STEP`](./NEW_gear_pedal_wheel.STEP) | Initial $15\,\text{cm}$ pedal ring gear prior to crank-hub reinforcement (`SKF 6004-2RSL`). |
| [`static_magneticplate&stand.STEP`](./static_magneticplate&stand.STEP) | [`NEW_static_magneticplate&stand.STEP`](./NEW_static_magneticplate&stand.STEP) | Initial stator stand prior to adding rigid mounting bosses for the $1:3.6$ stepper gearbox (`GBX2`) and guide rollers (`SKF 619/6-2Z`). |

---

## 2. SKF Deep-Groove Ball Bearing Subdirectories (`Sheet 4` BOM Row 17)

All rotating shafts and stator guide mechanisms use standard metric **SKF Explorer single-row deep-groove ball bearings**. Each subdirectory contains the full SolidWorks assembly (`.SLDASM`), individual component parts (inner ring, outer ring, ball cage, balls, shield/seal `.SLDPRT`), and the official SKF dimension table (`.txt`):

| Subdirectory | SKF Designation | Bore $d$ ($\text{mm}$) | OD $D$ ($\text{mm}$) | Width $B$ ($\text{mm}$) | Dynamic Load $C$ ($\text{kN}$) | Static Load $C_0$ ($\text{kN}$) | Limiting Speed ($\text{r/min}$) | Application in Project Doula |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| [`bearings_flywheel/`](./bearings_flywheel/) | **`SKF 6205-Z`** ([`6205-z.txt`](./bearings_flywheel/6205-z.txt)) | $25.0$ | $52.0$ | $15.0$ | $14.8\,\text{kN}$ | $7.8\,\text{kN}$ | $18,000$ | **High-Speed Flywheel & $3\,\text{cm}$ Pinion Shaft (`GP1` + `FLY1`)** — supports $5\times$ pedal speed ($300\text{–}375\,\text{RPM}$) and axial magnetic attraction forces across the dual-stator air gap. |
| [`pedal_wheels_bearings/`](./pedal_wheels_bearings/) | **`SKF 6004-2RSL`** ([`6004-2rsl.txt`](./pedal_wheels_bearings/6004-2rsl.txt)) | $20.0$ | $42.0$ | $12.0$ | $9.95\,\text{kN}$ | $5.0\,\text{kN}$ | $19,000$ | **Primary Pedal Crank & $15\,\text{cm}$ Ring Gear Shaft (`GR1`)** — low-friction dual-seal bearing supporting patient foot-pedal crank loads and $1:5$ spur-gear radial tooth forces. |
| [`roller_bearings/`](./roller_bearings/) | **`SKF 619/6-2Z`** ([`619_6-2z.txt`](./roller_bearings/619_6-2z.txt)) | $6.0$ | $15.0$ | $5.0$ | $1.74\,\text{kN}$ | $0.71\,\text{kN}$ | $56,000$ | **Rotatable Stator Plate (`STAT2`) Guide Rollers & Idlers** — maintains a uniform $1.5\text{–}2.0\,\text{mm}$ axial air gap ($g$) against normal magnetic attraction while `M2` + `GBX2` rotates `STAT2` across $\phi \in [0^\circ, 60^\circ]$. |
