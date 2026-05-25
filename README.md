# AURA 7" Heavy-Lift Rocket Tracking & Interception Mission Control

Welcome to the official deployment repository for the **AURA 7-inch Heavy-Lift Avionics & Rocket Tracking Platform**. This repository hosts the standalone, production-ready binary distribution of the ground station telemetry dashboard and multi-physics trajectory estimation core.

---

## 🎯 Mission Objectives

The AURA 7" platform is engineered as a high-performance tactical observation and tracking system designed to interface with supersonic target vectors. Its primary mission constraints include:

1. **High-Dynamic Rocket Trajectory Tracking**: Real-time localization, cinematic recording, and vector estimation of sounding rockets operating under extreme conditions (up to Max Q and supersonic flight regimes).
2. **Dual-GNSS Telemetry Fusion**: Utilizing heterogeneous dual-GPS hardware arrays (Align HEGAPS11 + TBS M10Q) to provide redundant positioning, anti-jamming resilience, and synchronized 10-channel telemetry streaming.
3. **Heavy-Lift Stabilized Payload Operations**: Sustaining stable 4K target acquisition and gimbal orientation under high vibration profiles and aerodynamic stress during aggressive intercept vectors.
4. **Supply Chain Integrity**: Built exclusively on professional-grade, strictly non-PRC (de-China) component architectures (TBS, Align, APD, Xnova) to meet rigorous aerospace research and deployment standards.

---

## 📊 UAV System Specifications

| Parameter | Specification |
| :--- | :--- |
| **All-Up Weight (AUW)** | 1.90 kg (Dual-battery configuration) |
| **Thrust-to-Weight Ratio (TWR)** | 6.02 : 1 |
| **Estimated Hover Endurance** | 13.1 min |
| **Telemetry Array** | 10-Channel Synchronized Real-Time Data Stream |
| **Propulsion Architecture** | High-Torque 6S Power Loop |

---

## 📋 Bill of Materials (BOM)

| Category | Component | Model / Specification | Qty | Brand |
| :--- | :--- | :--- | :---: | :--- |
| **Structure** | Frame | Aura 7-inch Long Range | 1 | Rotorbuilds |
| **Avionics** | GNSS 01 | HEGAPS11 High-Precision Module | 1 | Align |
| | GNSS 02 | TBS M10Q GPS Glonass Array | 1 | TBS |
| | Telemetry TX | TBS Crossfire Nano TX | 1 | TBS |
| | Telemetry RX | Nano Rx - FPV Long Range Receiver | 1 | TBS |
| | Video TX | TBS Unify Pro32 HV (MMCX) | 1 | TBS |
| | Video RX | TBS Fusion Module | 1 | TBS |
| | Flight Controller | Align AP5 Core Unit | 1 | Align |
| **Propulsion**| Battery | MaxAmps 3250mAh 6S LiPo | 2 | MaxAmps |
| | Power Distribution| APD PDB 360 (High-Current) | 1 | APD |
| | ESC | APD 80F3 (80A Single Industrial ESC) | 4 | APD |
| | Propellers | Gemfan 7050 Hurricane 3-Blade | 4 | Gemfan |
| | Motors | Xnova 2812 Heavy Lift - 1300KV | 4 | Xnova |
| **Payload** | Gimbal Controller | G3P 4K DV Stabilization Board | 1 | Align |
| | Camera | G3P Pro 4K Optical Core | 1 | Align |

---

## ⚡ Ground Station Core Features

* **Parameterized Component Architect**: Hardcoded mathematical scaling models aligned directly with the physical 1.90 kg payload inertia matrix.
* **Deterministic Trajectory Intercept Solver**: Real-time multi-physics dynamics computation engine evaluating tracking matrices via pre-loaded CSV data.
* **Synchronized Telemetry HUD**: Interactive 3-axis attitude display, dual-battery voltage diagnostics, and multi-channel responsive kinematic graphs.

---

## 📦 How to Run (Zero-Dependency Package)

This repository is delivered as a compiled standalone package. No local Python environment or external ANSYS runtimes are required.

1. **Clone or Download the Package**: Download this repo as a ZIP file and extract it to your local machine.
2. **Execute the Control Core**: Double-click **`Aura7_MissionControl.exe`**. Do not close the black terminal window, as it runs the backend server.
3. **Open the HUD UI**: Open any browser (Chrome/Edge/Brave) and go to:
   👉 **`http://localhost:8080`**

---

## 📁 Repository Structure

```text
├── _internal/                  # Compiled binary runtimes and system libraries
├── Aura7_MissionControl.exe    # Standalone execution file (Dashboard Backend)
├── rocket_simulation.csv       # Pre-loaded high-fidelity aerodynamic trajectory matrix
└── README.md                   # System documentation (This file)
