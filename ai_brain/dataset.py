"""
@file dataset.py
@description
Canonical dataset loader and preprocessor for Robo-Swarm-Fed.

This file is the SINGLE SOURCE OF TRUTH for:
- LiDAR preprocessing
- Data normalization
- Tensor shape guarantees

Any change here affects ALL later phases.
"""

import os
import numpy as np
import pandas as pd
import torch
from torch.utils.data import TensorDataset, random_split

# -----------------------------
# Constants (AI contract)
# -----------------------------
LIDAR_DIM = 360
LIDAR_MAX_RANGE = 10.0


def preprocess_lidar(ranges: np.ndarray) -> np.ndarray:
    """
    Normalize and validate a single LiDAR scan.

    Steps:
    1. Assert correct dimensionality
    2. Replace NaN / Inf values
    3. Clip to sensor max range
    4. Normalize to [0.0, 1.0]

    This function MUST remain deterministic.
    """

    # Safety check: shape must be exactly (360,)
    assert ranges.shape == (LIDAR_DIM,), "LiDAR scan must have 360 values"

    # Replace invalid values
    ranges = np.nan_to_num(
	    ranges,              # input array (raw LiDAR scan)
	    nan=0.0,             # replace NaN (not a number) values with 0.0
	    posinf=LIDAR_MAX_RANGE,  # replace +infinity with max sensor range
	    neginf=0.0           # replace -infinity with 0.0
		)


    # Clip to physical sensor limits
    ranges = np.clip(ranges, 0.0, LIDAR_MAX_RANGE)

    # Normalize to [0, 1]
    return ranges / LIDAR_MAX_RANGE


def load_dataset(csv_path: str, train_ratio: float = 0.8):
    """
    Load CSV dataset and return PyTorch train/validation datasets.

    CSV format (Phase 1 guarantee):
    - Columns 0–1   : control commands (linear_x, angular_z)
    - Columns 2–361 : LiDAR ranges (360 values)

    Returns:
    - train_dataset
    - val_dataset
    """

    # Ensure dataset exists
    assert os.path.exists(csv_path), f"Dataset not found: {csv_path}"

    # Load CSV and drop corrupted rows
    df = pd.read_csv(csv_path).dropna()

    # Extract raw arrays
    X_raw = df.iloc[:, 2:].to_numpy()
    y = df.iloc[:, 0:2].to_numpy()

    # Apply preprocessing scan-by-scan
    X = np.array([preprocess_lidar(scan) for scan in X_raw])

    # Convert to PyTorch tensors
    X_tensor = torch.tensor(X, dtype=torch.float32)
    y_tensor = torch.tensor(y, dtype=torch.float32)

    # Final contract assertions
    assert X_tensor.shape[1] == LIDAR_DIM
    assert y_tensor.shape[1] == 2

    # Wrap as dataset
    dataset = TensorDataset(X_tensor, y_tensor)

    # Split into train / validation
    train_size = int(train_ratio * len(dataset))
    val_size = len(dataset) - train_size

    return random_split(dataset, [train_size, val_size])
