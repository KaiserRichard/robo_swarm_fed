"""
@file model.py
@description
Neural network policy used for LiDAR-based navigation.

This file defines ONLY the model architecture.
No training logic belongs here.
"""

import torch.nn as nn


class SimpleMLP(nn.Module):
    """
    Simple feed-forward neural network.

    Architecture:
    - Input : 360 LiDAR values
    - Hidden: 64 neurons (ReLU)
    - Output: 2 control commands
    """

    def __init__(self):
        super().__init__()

        self.net = nn.Sequential(
            nn.Linear(360, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x):
        return self.net(x)
