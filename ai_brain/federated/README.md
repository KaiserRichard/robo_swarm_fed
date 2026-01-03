# Phase 3 — Federated Learning (Flower Next-Gen)

This module implements **Phase 3 of Robo-Swarm-Fed**, introducing federated learning
as a coordination mechanism between multiple robots.

Federated learning is treated as **supporting infrastructure**, not the primary
research focus. The goal of this phase is to validate **clean orchestration,
correct aggregation, and system-level integration**.

---

## Scope

- Flower **1.25 (Next-Gen API)**–based federated learning
- Server–client orchestration using `ServerApp` / `ClientApp`
- Passive aggregation via `FedAdagrad`
- Local simulation backend (Ray, implicit)
- Reuse of frozen Phase-2 AI model and dataset contract

---

## Design Intent

- **Robotics first, learning second**  
  Federated learning supports multi-robot coordination; it is not studied in isolation.

- **Minimal federated complexity**  
  No custom execution loops, no experimental FL features.

- **Phase isolation**  
  Phase-2 AI logic is reused unchanged. Phase-3 introduces no new model semantics.

---

## Key Design Decisions

### No Global Weight Synchronization (Intentional)

Clients **start from fresh local models each round**.

This is deliberate:
- Keeps Phase-3 focused on orchestration correctness
- Avoids mixing infrastructure validation with learning behavior
- Global weight synchronization is introduced in **Phase 4**

---

### Shared Dataset Across Clients (Temporary)

- All clients currently load the same dataset
- Non-IID partitioning is deferred to Phase 4
- Simplifies validation of federated execution flow

---

## Module Responsibilities

### `client_app.py`
- Defines Flower `ClientApp`
- Performs local training and evaluation
- Reports scalar metrics only (loss)

### `server_app.py`
- Defines Flower `ServerApp` using `server_fn`
- Initializes global model and strategy
- Delegates execution entirely to Flower runtime

### `strategy.py`
- Custom wrapper around `FedAdagrad`
- Provides hooks for future robustness logic
- Currently behaves exactly like base strategy

### `task.py`
- Shared ML logic (training, evaluation, data loading)
- Ray-safe dataset resolution
- No networking or federated logic

---

## Guarantees

- Deterministic orchestration given fixed configuration
- Clean separation between:
  - Robotics (Phase 1)
  - Centralized learning (Phase 2)
  - Federated orchestration (Phase 3)
- Forward compatibility with Phase-4 deployment

---

## Non-Goals (Phase 3)

- Federated learning algorithm research
- Performance optimization
- Robustness heuristics
- ROS ↔ AI runtime integration

These are addressed in later phases.

