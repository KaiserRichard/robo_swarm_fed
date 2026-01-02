# Phase 1 — Robot Control & Data Generation

This module implements the **baseline robot behavior** and **data collection pipeline**
used to generate training data for later AI and Federated Learning phases.

---

## Scope
- ROS 2 control nodes (patrol, obstacle avoidance)
- LiDAR-based reflex controller (safety-first)
- CSV logging of `(LiDAR → cmd_vel)` pairs

---

## Design Intent
- Acts as a **teacher system** for imitation learning
- Prioritizes **stability, interpretability, and safety**
- Used only for **data generation**, not final autonomy

---

## Guarantees
- Fixed LiDAR dimensionality: **360 beams**
- Deterministic control logic
- Reproducible CSV schema

---

## Non-Goals
- No machine learning
- No federated or online learning
- Not a production controller
