# Go2 Custom Controller

A custom locomotion controller for the **Unitree Go2** focused on building a complete torque-controlled locomotion stack from scratch.

Current development progresses from:

- Position Control
- Pure PD Torque Tracking
- Dynamic Trot Generation
- Model Predictive Control (Work in Progress)

---


## Demo

▶ **Watch Demo on YouTube:** https://youtu.be/iW85iWN7zKw
---

## Features

- 250 Hz Pure PD Torque Controller
- Dynamic Trot Gait Generator
- Hardware-safe Reachability Checks
- Inverse & Forward Kinematics
- Swing / Stance Foot Trajectory Planning
- Contact State Estimation
- EKF-based State Estimation
- Velocity Estimation
- ROS2 Humble Compatible

---

## Development Roadmap

### Stage 1 ✅ Position Control

- Trot gait generation
- Position-based locomotion
- EKF state estimation
- Velocity estimation

---

### Stage 2 ✅ Pure PD Torque Tracking

- Effort controller
- 250 Hz control loop
- Hardware-safe torque tracking
- Cartesian foot tracking
- Reachability validation
- Stable standing controller

---

### Stage 3 🚧 Model Predictive Control

Currently under development.

Planned additions include

- Whole-body MPC
- Ground Reaction Force Optimization
- Jacobian Torque Mapping

```
τ = JᵀF
```

- Terrain adaptation
- Dynamic disturbance rejection
- Uneven terrain locomotion

---

## Project Structure

```
go2_custom_controller/
│
├── gait/
│     ├── trot_gait.cpp
│     ├── standing_controller.cpp
│
├── tracker/
│     ├── pure_pd_tracker.cpp
│     ├── dynamic_tracker.cpp
│
├── state_estimation/
│
├── launch/
│
├── config/
│
└── docs/
      ├── images/
      └── videos/
```

---

## Controller Pipeline

```
cmd_vel
   │
   ▼
Trot Gait Planner
   │
   ▼
Inverse Kinematics
   │
   ▼
Desired Joint Angles
   │
   ▼
Pure PD Tracker (250 Hz)
   │
   ▼
Joint Torques
   │
   ▼
Go2 Robot
```

---

## Future Work

- MPC Controller
- Whole Body Control
- Terrain Adaptation
- Online Footstep Planning
- Disturbance Recovery
- Dynamic Running

---

## License

MIT