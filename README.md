# Optimal Control Tests

This repository contains minimal but structured experiments for **robot control and simulation**, with a focus on joint-level control, dynamics, and ROS 2–based system integration.

This repository is part of a larger ongoing effort aimed at understanding **robot dynamics, feedback control, and optimal control methods**, and will be updated incrementally as development progresses.

---

## Contents

- `src/` – Source code (ROS 2 nodes, MuJoCo simulation, control experiments)
- MuJoCo-based simulation of a 6-DOF industrial robot (UR5e)
- Joint-level torque control experiments
- PD controller implementation with gravity compensation (estimated/learned)
- Joint state feedback (positions & velocities)
- Minimal ROS 2 publisher/subscriber examples for robot control

---

## Current Status (Completed)

The following components are **implemented and functional**:

- Robot simulation using **MuJoCo**
- Joint torque control interface
- PD controller for joint-space regulation
- Joint state feedback (positions and velocities)
- Joint torque command publishing
- ROS 2 integration between simulation and controller nodes
- Torque saturation using joint limits

These components together form a **closed-loop joint-level control pipeline** using ROS 2.

---

## Upcoming

Planned additions include advanced control strategies:

- PID control
- Inverse dynamics control
- Feedback linearization and Computed torque control
- Further experiments with optimal control formulations
- Improved gravity and dynamics estimation
- Controller tuning and stability analysis

---

## Requirements

- Ubuntu 22.04
- ROS 2 (Humble or later)
- Python 3.10+
- MuJoCo

---

## Usage

This repository is intended for **learning, experimentation, and research** in:

- Robot simulation
- Joint-space control design
- Robot dynamics and gravity compensation
- ROS 2 interfaces for robot control
- Understanding the interaction between physics simulation and control laws

---

## Resources

- **Joint Torque Limits of UR5e Robot**  
  https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/

---

## Notes

- Build artifacts (`build/`, `install/`, `log/`) are intentionally ignored.
- This repository is a **work-in-progress**, focused on clarity, minimal examples, and progressive experimentation.
- Code structure may evolve as more advanced controllers and dynamics models are introduced.
