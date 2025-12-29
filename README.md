# Optimal Control Tests

This repository contains hands-on experiments for learning robot dynamics and control using simulation as a substitute for real robot hardware.

It is developed alongside a control systems / robotics course, where theoretical concepts (dynamics, controllers, stability) are implemented and validated in simulation to gain practical experience equivalent to working with real robots.

The project follows a joint-level control approach using MuJoCo and ROS 2, closely mimicking real industrial robot interfaces.

---

## Motivation

Access to real robot hardware is often limited.  
This repository treats simulation as hardware, enabling:

- Safe experimentation with control laws
- Direct torque-level control
- Realistic joint state feedback
- ROS 2–based modular controller design

The goal is learning by implementation, not building a polished framework.

---

## Current Features 

- **MuJoCo-based robot simulation**
  - UR5e robot model
  - Physics-based joint dynamics
- **ROS 2 integration**
  - Joint position & velocity publishers
  - Joint torque command subscribers
- **Joint-level PD controller**
  - Torque control interface
  - Velocity damping
- **Gravity compensation (estimated / learned)**
  - Online gravity term estimation
- **Joint limits & torque saturation**
  - Realistic actuator constraints
- **Simulation ↔ controller decoupling**
  - Separate ROS 2 nodes for plant and controller
- **Simulation used as real hardware**
  - No direct access to model internals in controller

---

## Controllers Implemented

- PD joint torque controller
- Gravity compensation (estimated online)
- Torque saturation & joint safety limits

---

## Upcoming Controllers

Planned implementations as part of the course progression:

- PID control
- Inverse dynamics control
- Feedback linearization
- Computed torque control
- Optimal control experiments
- Trajectory tracking (joint space)

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble (or later)
- Python 3.10+
- MuJoCo

---

## Usage

This repository is intended for:

- Learning robot dynamics
- Implementing classical control laws
- Understanding joint-level robot interfaces
- Practicing ROS 2 communication for robotics
- Gaining practical experience without physical robots

---

## Resources

- **UR5e Joint Torque Limits**  
  https://www.universal-robots.com/articles/ur/robot-care-maintenance/max-joint-torques-cb3-and-e-series/

- MuJoCo Documentation  
- ROS 2 Control Concepts
- Classical Robotics & Control textbooks

---

## Notes

- Build artifacts (`build/`, `install/`, `log/`) are intentionally ignored
- This is a learning and research repository
- Code clarity and experimentation are prioritized over abstraction
- The repository will evolve as the course progresses

---

---

