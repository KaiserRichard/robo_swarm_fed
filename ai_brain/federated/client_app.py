"""
@file client_app.py
@description
Federated Learning client for Robo-Swarm-Fed (Phase 3).

Each client:
- Loads hyperparameters from pyproject.toml via Context.run_config
- Trains locally for a fixed number of epochs
- Reports scalar metrics (loss) to the server

IMPORTANT (Phase 3 Design Decision):
- Clients intentionally start from a fresh local model each round.
- Global weight synchronization is NOT implemented yet.
- This is deliberate and will be introduced in Phase 4.
"""

from flwr.client import ClientApp
from flwr.app import Context
from flwr.common import Message

from ai_brain.federated.task import (
    SimpleMLP,
    load_data,
    train,
    test,
    get_device,
)

# ------------------------------------------------------------------
# Create ClientApp
# ------------------------------------------------------------------
app = ClientApp()

# Device is selected once per client process
device = get_device()


@app.train()
def train_fn(msg: Message, context: Context):
    """
    Local training callback executed on each federated client.

    Notes:
    - Hyperparameters are injected from pyproject.toml
    - context.node_id is provided by Flower runtime
    - No networking or aggregation logic belongs here
    """

    # --------------------------------------------------------------
    # Read hyperparameters from Flower Context
    # --------------------------------------------------------------
    local_epochs: int = context.run_config["local-epochs"]
    lr: float = context.run_config["lr"]

    # --------------------------------------------------------------
    # Phase 3 Intentional Design Choice
    # --------------------------------------------------------------
    # Each client starts from a fresh local model.
    # This keeps Phase 3 focused on orchestration correctness.
    # Global model synchronization will be added in Phase 4.
    model = SimpleMLP()

    # --------------------------------------------------------------
    # Load local dataset
    # --------------------------------------------------------------
    train_loader, _ = load_data(
        partition_id=int(context.node_id),
    )

    # --------------------------------------------------------------
    # Perform local training
    # --------------------------------------------------------------
    loss = train(
        model=model,
        loader=train_loader,
        epochs=local_epochs,
        lr=lr,
        device=device,
    )

    # --------------------------------------------------------------
    # Return scalar metrics only (Flower requirement)
    # --------------------------------------------------------------
    return {"loss": loss}


@app.evaluate()
def evaluate_fn(msg: Message, context: Context):
    """
    Local evaluation callback.

    Notes:
    - Uses the same dataset loader as training
    - Evaluation metrics are optional in Phase 3
    """

    model = SimpleMLP()
    _, val_loader = load_data()

    loss, _ = test(model, val_loader, device)

    return {"loss": loss}
