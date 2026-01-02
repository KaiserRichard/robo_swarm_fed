# Robo-Swarm-Fed

Robo-Swarm-Fed is a robotics research project focused on **AI-driven navigation**
and **federated learning for robot swarms** using ROS 2.

---

## Project Phases

1. **Phase 1 — Robot Control & Data Generation**  
   Baseline ROS 2 controllers and data collection.

2. **Phase 2 — AI Brain (Offline Learning)**  
   Deterministic AI training pipeline (frozen contract).

3. **Phase 3 — Federated Learning**  
   Distributed training across multiple robot clients.

4. **Phase 4 — AI Deployment**  
   Running trained models inside ROS 2.

---

## Code Structure

- [AI Brain (Phase 2)](./ai_brain)  
  Offline training, datasets, and model definitions.

- [Robot Controller (Phase 1)](./robo_swarm_controller)  
  ROS 2 nodes for navigation and data generation.
