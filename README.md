# Robo-Swarm-Fed

**Robo-Swarm-Fed** is a robotics research project focused on **AI-driven navigation**
and **federated learning for robot swarms** using **ROS 2**.

The project is developed in structured phases, with each phase building on a
stable and well-defined contract from the previous one.

---

## Project Phases

### Phase 1 — Robot Control & Data Generation
Baseline ROS 2 navigation stack, robot controllers, and data collection pipelines.

- Obstacle avoidance
- Patrol logic
- Sensor-based data logging
- ROS 2-native execution

---

### Phase 2 — AI Brain (Offline Learning)
Deterministic, reproducible offline learning pipeline.

- Clean dataset / model / training separation
- PyTorch-based MLP for navigation
- Frozen AI contract (used by later phases)
- Emphasis on code clarity and reproducibility

---

### Phase 3 — Federated Learning (Flower Next-Gen)
Distributed training across multiple robot clients using **Flower 1.25 (Next-Gen API)**.

**Purpose of this phase** is system orchestration — not federated learning research.

**Key characteristics:**
- Server implemented via `ServerApp(server_fn=...)`
- Clients implemented via `ClientApp`
- Passive aggregation strategy (`FedAdagrad`)
- Configuration driven by `pyproject.toml`
- Local simulation backend (Ray, implicit)

**Important design decisions:**
- Clients intentionally start from fresh local models each round
- No global weight synchronization yet (added in Phase 4)
- Dataset is shared across clients (non-IID planned later)

---

### Phase 4 — AI Deployment (Planned)
Deployment of trained models back into ROS 2 runtime.

- Model inference inside ROS 2 nodes
- Closed-loop navigation
- System-level validation on robot swarms

---

## Code Structure

### AI Brain (`ai_brain/`)
Core learning logic shared across phases.

- Offline training (Phase 2)
- Federated learning infrastructure (Phase 3)
- Models, datasets, and evaluation utilities

### Robot Controller (`robo_swarm_controller/`)
ROS 2 nodes responsible for:

- Navigation
- Obstacle avoidance
- Data generation
- Runtime control

---

## Project Philosophy

> **Robotics first, learning second, infrastructure last.**

Complexity is added only when justified by real robotic system requirements,
not by algorithmic novelty alone.
