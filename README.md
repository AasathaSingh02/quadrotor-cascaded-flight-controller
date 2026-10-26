# Hybrid Cascaded Flight Controller for 6-DOF Quadrotor

<p align="center">
  <img src="https://img.shields.io/badge/Platform-Crazyflie%202.X-orange?style=flat-square" />
  <img src="https://img.shields.io/badge/Simulation-MATLAB%20%2F%20Python-blue?style=flat-square" />
  <img src="https://img.shields.io/badge/Control-Cascaded%20PID%20%2B%20Adaptive-green?style=flat-square" />
  <img src="https://img.shields.io/badge/License-MIT-brightgreen?style=flat-square" />
</p>

A high-performance **cascaded PID flight control architecture** for a 6-DOF nano-quadrotor (Crazyflie 2.X platform). Features an outer-loop translational position controller feeding an inner-loop attitude/angular-rate controller with motor thrust allocation, gyroscopic precession decoupling, and anti-windup clamping. Validated across 50 continuous laps of high-speed circular trajectory tracking with **4.2 cm RMS tracking error** and tilt angle strictly bounded below **15°**.

---

## Control Architecture

```
 Desired Trajectory [x_d, y_d, z_d, ψ_d]
                    │
                    ▼
     ┌─────────────────────────────┐
     │  Outer Loop (Position PID)  │ ◄─── Position Feedback [x, y, z]
     └──────────────┬──────────────┘
                    │ Desired Thrust & Attitudes [F_total, φ_d, θ_d]
                    ▼
     ┌─────────────────────────────┐
     │  Inner Loop (Attitude PID)  │ ◄─── IMU Attitude Feedback [φ, θ, ψ, p, q, r]
     └──────────────┬──────────────┘
                    │ Control Torques [τ_x, τ_y, τ_z]
                    ▼
     ┌─────────────────────────────┐
     │  Thrust & Torque Mixer      │
     └──────────────┬──────────────┘
                    │ Individual Rotor Speeds [ω1, ω2, ω3, ω4]
                    ▼
     ┌─────────────────────────────┐
     │  6-DOF Quadrotor Plant      │
     └─────────────────────────────┘
```

## Performance Metrics

| Metric | Target Specification | Experimental / Sim Result |
|--------|:--------------------:|:-------------------------:|
| Tracking Error (RMS) | < 6.0 cm | **4.2 cm** |
| Max Overshoot (Step Response) | < 8.0% | **4.8%** |
| Max Tilt Angle (Fast Lap) | < 20° | **14.6°** |
| Motor Saturation Overhead | > 20% margin | **29% margin** (58–71% rated) |
| Convergence Time (Disturbance) | < 1.2 s | **0.82 s** |

---

## Directory Structure

```
quadrotor-cascaded-flight-controller/
├── src/
│   ├── pid_controller.py        # Cascaded outer/inner PID with anti-windup
│   ├── quadrotor_dynamics.py    # Non-linear 6-DOF equations of motion
│   └── trajectory_generator.py # Smooth minimum-snap trajectory generator
├── sim/
│   └── run_simulation.py        # 50-lap circular simulation & plotting
├── docs/
│   └── mathematical_formulation.md
├── LICENSE
└── README.md
```

## Usage

```bash
python sim/run_simulation.py --laps 50 --plot
```
