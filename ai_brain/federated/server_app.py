"""
@file server_app.py
@description
Federated Learning server entry point for Robo-Swarm-Fed (Phase 3).

This file follows the Flower 1.25 Next-Gen API.

Key architectural rules:
- Uses ServerApp(server_fn=...)
- Does NOT use @app.main
- Strategy is passive (aggregation only)
- Execution is owned entirely by Flower runtime

Phase 3 Scope:
- Correct orchestration
- Correct aggregation
- No custom execution loops
"""

from datetime import datetime
from pathlib import Path

from flwr.app import Context
from flwr.server import ServerAppComponents
from flwr.serverapp import ServerApp
from flwr.common import ndarrays_to_parameters

from ai_brain.model import SimpleMLP
from ai_brain.federated.strategy import CustomFedAdagrad


def server_fn(context: Context) -> ServerAppComponents:
    """
    Server factory function (Flower 1.25 compatible).

    This function is called exactly once by the Flower runtime.
    It must return ServerAppComponents describing server behavior.
    """

    # --------------------------------------------------------------
    # Read configuration injected from pyproject.toml
    # --------------------------------------------------------------
    fraction_fit: float = context.run_config["fraction-train"]
    fraction_evaluate: float = context.run_config["fraction-evaluate"]

    # --------------------------------------------------------------
    # Prepare output directory (future use)
    # --------------------------------------------------------------
    # NOTE:
    # This directory is intentionally created here so Phase 3.1 / Phase 4
    # can save the final global model without changing orchestration logic.
    run_dir = datetime.now().strftime("%Y-%m-%d/%H-%M-%S")
    output_dir = Path.cwd() / "outputs" / run_dir
    output_dir.mkdir(parents=True, exist_ok=True)

    # --------------------------------------------------------------
    # Initialize global model (Phase 2 contract)
    # --------------------------------------------------------------
    global_model = SimpleMLP()
    initial_parameters = ndarrays_to_parameters(
        [v.cpu().numpy() for v in global_model.state_dict().values()]
    )

    # --------------------------------------------------------------
    # Configure federated learning strategy
    # --------------------------------------------------------------
    strategy = CustomFedAdagrad(
        initial_parameters=initial_parameters,
        fraction_fit=fraction_fit,
        fraction_evaluate=fraction_evaluate,
        min_fit_clients=2,
        min_available_clients=2,
    )

    # --------------------------------------------------------------
    # IMPORTANT (Flower 1.25 Rule)
    # --------------------------------------------------------------
    # - Do NOT pass server_config here
    # - Number of rounds is injected via Context.run_config
    return ServerAppComponents(
        strategy=strategy,
    )


# ------------------------------------------------------------------
# Create ServerApp (NO @app.main allowed here)
# ------------------------------------------------------------------
app = ServerApp(server_fn=server_fn)
