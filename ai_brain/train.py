"""
@file train.py
@description
Offline supervised training loop for Phase 2.

Trains a SimpleMLP model to imitate robot navigation behavior
from LiDAR → velocity data.
"""

import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader

from dataset import load_dataset
from model import SimpleMLP

# -----------------------------
# Configuration
# -----------------------------
DATA_PATH = "data/robot_data.csv"
# A file that stores the learned weights of your neural network after training.
MODEL_PATH = "data/local_model.pth"


def train():
    """
    Main training routine.
    """

    # 1. Load dataset (already preprocessed & validated)
    train_set, val_set = load_dataset(DATA_PATH)

    # 2. DataLoaders handle batching & shuffling
    train_loader = DataLoader(train_set, batch_size=32, shuffle=True)
    val_loader = DataLoader(val_set, batch_size=32, shuffle=False)

    # 3. Model + loss + optimizer
    model = SimpleMLP()
    criterion = nn.MSELoss()      # regression loss
    optimizer = optim.Adam(
        model.parameters(),
        lr=1e-3
    )

    epochs = 50

    for epoch in range(epochs):

        # ---- TRAINING ----
        model.train()
        train_loss = 0.0

        for X, y in train_loader:
            optimizer.zero_grad()
            predictions = model(X)
            loss = criterion(predictions, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()

        avg_train_loss = train_loss / len(train_loader)

        # ---- VALIDATION ----
        model.eval()
        val_loss = 0.0

        with torch.no_grad():
            for X, y in val_loader:
                val_loss += criterion(model(X), y).item()

        avg_val_loss = val_loss / len(val_loader)

        print(
            f"[Epoch {epoch:03d}] " 
            # 0: pad with zeros on the left, 3: minimum width, d: decimal int
            f"Train Loss: {avg_train_loss:.4f} | "
            f"Val Loss: {avg_val_loss:.4f}"
        )

    # 4. Save model weights only (architecture is fixed)
    torch.save(model.state_dict(), MODEL_PATH)
    print(f"✅ Model saved to {MODEL_PATH}")


if __name__ == "__main__":
    train()
