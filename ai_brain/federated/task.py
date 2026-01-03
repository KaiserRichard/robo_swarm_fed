"""
@file task.py
@description
Shared machine learning logic for Phase 3 (Federated Learning).

Responsibilities:
- Device selection
- Dataset loading
- Local training
- Local evaluation

IMPORTANT:
- No Flower, Ray, or networking logic here
- Reuses Phase 2 AI contract unchanged
"""

from dataclasses import dataclass
from pathlib import Path

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

from ai_brain.model import SimpleMLP
from ai_brain.dataset import load_dataset


# ------------------------------------------------------------------
# Dataset path (package-relative, Ray-safe)
# ------------------------------------------------------------------
# Resolves to: ai_brain/data/robot_data.csv
DATASET_PATH = (
    Path(__file__)
    .resolve()
    .parents[1]   # ai_brain/
    / "data"
    / "robot_data.csv"
)


def get_device():
    """
    Select the best available compute device.

    Priority:
    1. CUDA (NVIDIA GPU)
    2. MPS (Apple Silicon)
    3. CPU
    """
    if torch.cuda.is_available():
        return torch.device("cuda")
    if torch.backends.mps.is_available():
        return torch.device("mps")
    return torch.device("cpu")


def load_data(partition_id: int = 0, num_partitions: int = 1):
    """
    Load local dataset for a federated client.

    Notes:
    - partition_id is reserved for future non-IID splits
    - Phase 3 currently uses the same dataset on all clients
    """

    train_set, val_set = load_dataset(str(DATASET_PATH))

    return (
        DataLoader(train_set, batch_size=32, shuffle=True),
        DataLoader(val_set, batch_size=32, shuffle=False),
    )


def train(
    model: nn.Module,
    loader: DataLoader,
    epochs: int,
    lr: float,
    device: torch.device,
):
    """
    Perform local training for one federated round.
    """

    model.to(device)
    model.train()

    optimizer = optim.Adam(model.parameters(), lr=lr)
    criterion = nn.MSELoss()

    for _ in range(epochs):
        for x, y in loader:
            x, y = x.to(device), y.to(device)

            optimizer.zero_grad()
            loss = criterion(model(x), y)
            loss.backward()
            optimizer.step()

    return loss.item()


def test(
    model: nn.Module,
    loader: DataLoader,
    device: torch.device,
):
    """
    Evaluate model on validation data.
    """

    model.to(device)
    model.eval()

    criterion = nn.MSELoss()
    total_loss = 0.0

    with torch.no_grad():
        for x, y in loader:
            x, y = x.to(device), y.to(device)
            total_loss += criterion(model(x), y).item()

    return total_loss / len(loader), 0.0


@dataclass
class TrainProcessMetaData:
    """
    Reserved for future client-side metadata reporting.
    """
    training_time: float
    converged: bool
