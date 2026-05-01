# 🔄 Furuta Pendulum — Hybrid Control System

> **MCT411: Hybrid Control | Fall 2025 | Team 14**  
> Ain Shams University — Faculty of Engineering, Mechatronics Department

---

## 📖 Overview

This repository contains the MATLAB/Simulink implementation of a **hybrid control system** for the **Furuta (Rotary Inverted) Pendulum** — a nonlinear, underactuated mechatronic benchmark system. The control architecture combines two distinct strategies:

1. **Energy-Based Swing-Up Controller** — pumps energy into the pendulum to drive it from the natural downward-stable position to the upright equilibrium.
2. **Linear Quadratic Regulator (LQR)** — takes over once the pendulum is near upright, maintaining stability against small disturbances.

The full system is validated through both **MATLAB/Simulink simulation** and **Hardware-in-the-Loop (HIL)** testing on a physical prototype.

---

## 🧑‍💻 Team

| Name | ID | Contribution |
|---|---|---|
| Abdelghaffar Essam Abdelghaffar | 2100428 | Mechanical Design, LQR Implementation |
| Ingy Ahmed Sherif | 2100648 | LQR Implementation, Simulation Setup |
| Kareem Saleh Hassan | 2100909 | Mechanical Design, Swing-Up Implementation |
| Mariam Mohamed El Sebaey | 2100260 | Swing-Up Implementation, Simulation Setup |

---

## ⚙️ System Description

The Furuta Pendulum consists of two arms whose rotary axes are perpendicular to each other:

- **Arm 1 (Table/Rotor):** Horizontal arm driven by a DC motor. Angle: θ₁
- **Arm 2 (Pendulum):** Vertical free-swinging arm attached to the end of Arm 1. Angle: θ₂

The system is **underactuated** — only one torque input (the motor) controls two degrees of freedom.

### Hardware Components

| Component | Specification |
|---|---|
| Microcontroller | ESP32 |
| Motor Driver | H-Bridge (bidirectional) |
| Motor Encoder | 600 PPR (horizontal arm θ₁) |
| Pendulum Encoder | High-resolution (pendulum angle θ₂) |
| Power Supply | 12 V DC |
| Host Interface | USB/Serial → MATLAB/Simulink HIL |

### Physical Parameters

| Parameter | Symbol | Value |
|---|---|---|
| Pendulum mass | mᵣ | 0.018 kg |
| Pendulum COM to rotation axis | lᵣ | 0.0638 m |
| Pendulum moment of inertia | Jᵣ | 9.83 × 10⁻⁵ kg·m² |
| Table rotation axis to pendulum pivot | lₜ | 0.07 m |
| Rotor moment of inertia | J_motor | 0.0024 kg·m² |
| Motor resistance | Rₐ | 5.8466 Ω |
| Motor constants | kₜ = k_b | 0.048 V/rpm |
| Armature inductance | Lₐ | 9.15 × 10⁻⁴ H |

> Friction coefficients (motor, pendulum, table) were identified via **parameter estimation** rather than theoretical derivation.

---

## 🎮 Control Architecture

### State Vector

```
x = [θ₁  θ̇₁  θ₂  θ̇₂]ᵀ
```

### 1. Swing-Up Controller (Energy-Based)

The controller injects energy by oscillating the horizontal arm, guided by the difference between the current pendulum energy and the target energy at the upright equilibrium:

```
E  = ½ Jᵣ θ̇₁² + mᵣ g lᵣ (1 + cos θ₁)
Eᵣ = 2 mᵣ g lᵣ

u  = sat( k_swing · (Eᵣ − E) · θ̇₁ · cos θ₁ )
```

### 2. Balancing Controller (LQR)

The nonlinear Simscape Multibody model is linearized at the upright equilibrium using **Simulink Model Linearizer**, yielding the state-space matrices used for LQR design:

```
ẋ = Ax + Bu
```

**LQR Cost Function:**
```
J = ∫₀^∞ (xᵀQx + uᵀRu) dt
```

**Tuned Weights:**
```
Q = diag([1, 0.1, 100, 10])
R = 100
```

**Resulting Gain:**
```
K = [−0.0100  −0.0857  1.5880  0.1478]
u(t) = −K x(t)
```

### 3. Supervisory Logic (Stateflow)

A Stateflow chart manages transitions between control modes:

```
Swing_Up ──(|θ₂| ≤ 15°, |θ̇₂| ≤ 7)──► LQR
   ▲                                      │
   │                                      │ (|θ̇₂| ≥ 25 or timeout)
   │                                      ▼
   └────────(safe)────────────────── Recovery
```

| State | Condition |
|---|---|
| **Swing_Up** | Default; energy injection active |
| **LQR** | `\|θ₂\| ≤ 15°` and `\|θ̇₂\| ≤ 7 rad/s` |
| **Recovery** | `\|θ₂\| ≥ 110°` or speed exceeded; motor halted |

---

## 📁 Repository Structure

```
furuta-pendulum/
├── Simulink/
│   ├── SwingUp_Model.slx          # Energy-based swing-up subsystem
│   ├── LQR_Model.slx              # LQR balancing subsystem
│   ├── FullSystem_Model.slx       # Complete hybrid model with Stateflow
│   └── Pendulum_SimMechanics.slx  # Simscape Multibody physical model
├── MATLAB/
│   ├── parameters.m               # System physical parameters
│   ├── lqr_design.m               # LQR gain computation (K, Q, R)
│   └── linearize_model.m          # Linearization script
├── ESP32/
│   └── firmware/                  # Microcontroller code for HIL
├── Results/
│   ├── simulation_results.png
│   └── real_life_results.png
├── Report/
│   └── T14_Hybrid_Project_Report.pdf
└── README.md
```

---

## 🚀 Getting Started

### Requirements

- MATLAB R2022b or later
- Simulink
- Simscape & Simscape Multibody toolboxes
- Control System Toolbox (for `lqr()`)
- Stateflow

### Running the Simulation

1. Clone the repository:
   ```bash
   git clone https://github.com/<your-org>/furuta-pendulum.git
   cd furuta-pendulum
   ```

2. Open MATLAB and run the parameter initialization script:
   ```matlab
   run('MATLAB/parameters.m')
   ```

3. Open and run the full Simulink model:
   ```matlab
   open('Simulink/FullSystem_Model.slx')
   ```

4. Press **Run** in Simulink. Observe the four scopes: `PendAngle`, `PendVel`, `MotorAngle`, `MotorVel`.

### HIL Deployment (ESP32)

1. Flash the ESP32 firmware from `ESP32/firmware/`.
2. Connect via USB to the host PC.
3. In Simulink, configure the serial port in the ESP32 communication block.
4. Run the model in **External Mode** for real-time HIL control.

---

## 📊 Results Summary

| Metric | Simulation | Real-Life |
|---|---|---|
| Swing-up success | ✅ | ✅ |
| LQR stabilization | ✅ | ✅ |
| Settling time | ~7 s | ~12 s |
| Post-stabilization oscillations | Low | Slightly higher |

The discrepancies between simulation and real-life results are attributed to unmodeled joint backlash, high-frequency friction, and minor actuation delays.

---

## 📐 Controller Requirements

| Requirement | Specification |
|---|---|
| Swing-Up | Move pendulum to upright |
| Stabilization | Maintain upright via LQR |
| Sampling rate | ≥ 0.1 kHz |
| Torque limits | Motor saturation constraints |
| Robustness | Against small disturbances |

---

## 📚 References

- B. Yao, *"Modeling of Rotary Inverted Pendulum,"* 2022.
- K. J. Åström & K. Furuta, *"Swinging up a pendulum by energy control,"* Automatica, 2000.

---

## 📄 License

This project is submitted as academic coursework for MCT411 at Ain Shams University. All rights reserved by the authors.
