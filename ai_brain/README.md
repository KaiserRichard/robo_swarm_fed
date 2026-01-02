# Phase 2 — AI Brain (Offline Learning Contract)

This module defines the **frozen AI contract** for Robo-Swarm-Fed.
It converts recorded robot sensor data into a trained navigation policy.

---

## Purpose
- Offline supervised learning
- Deterministic preprocessing
- Reproducible model training

---

## AI Contract

### Input
- LiDAR scan
- Shape: `(360,)`
- Range: `[0.0, 1.0]` (normalized)

### Output
- Continuous control
- Shape: `(2,)`
  - `linear_x` (m/s)
  - `angular_z` (rad/s)

---

## Design Principles
- Single source of truth for preprocessing
- No ROS or Gazebo dependency
- Federated-learning ready

---

## Status
This module is **frozen after Phase 2**.
All later phases must comply with this contract.
