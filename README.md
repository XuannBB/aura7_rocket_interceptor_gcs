# AURA 7" Interception Mission Control & Avionics Ground Station

Welcome to the official deployment repository for the **AURA 7-inch Heavy-Lift Interception Mission Control System**. This repository hosts the standalone, production-ready binary distribution of the ground station telemetry dashboard and multi-physics flight trajectory simulation engine.

---

## 🚀 Platform Specifications & Capabilities

The ground station architecture is pre-configured and optimized for the high-performance **AURA 7" UAV platform** (utilizing high-torque propulsion configurations and fully optimized de-China/non-PRC component integration).

| Parameter | Specification |
| :--- | :--- |
| **Total Takeoff Mass (AUW)** | 1.90 kg |
| **Thrust-to-Weight Ratio (TWR)** | 6.02 : 1 |
| **Estimated Hover Endurance** | 13.1 min |
| **Avionics Telemetry Stream** | 10-Channel Synchronized Real-Time Data |

---

## ⚡ Key Features

* **UAV Component Architect:** Parameterized hardware loading module matching custom high-payload aerodynamic configurations.
* **Intercept Trajectory Generation:** Real-time multi-physics dynamics solver calculating deterministic aerodynamic intercept vectors using pre-loaded high-fidelity simulation matrices.
* **Interactive Telemetry Dashboard:** Integrated 3-axis attitude visualizer, live power consumption diagnostics, and synchronized kinematic plots with full panning and scaling capabilities.

---

## 📦 How to Download and Run (Zero-Dependency)

This repository is distributed as a zero-dependency pre-compiled package. You **do not** need to install Python, ANSYS runtimes, or any external toolchains to execute the system.

### 1. Download the Repository
Click the green **`Code`** button at the top right of this page and select **`Download ZIP`**, or clone this repository directly using Git Bash:
```bash
git clone [https://github.com/XuannBB/FDV_drone.git](https://github.com/XuannBB/FDV_drone.git)
