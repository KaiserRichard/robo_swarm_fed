"""
    Defines a simple Multi-Layer Perceptron (MLP) for behavioral cloning.
    Input: 360 LiDAR range values.
    Output: 2 Control commands (Linear Velocity, Angular Velocity).
"""

import torch.nn as nn

class SimpleMLP(nn.Module):
    def __init__(self):
        super().__init__()
        # --- Network Architecture ---
        # Layer 1: Input (360 LiDAR points) -> Hidden Layer (64 Neurons)
        # We compress 360 points into 64 features to abstract the environment.
        self.layer1 = nn.Linear(360, 64)
        
        # Activation: ReLU
        # Adds non-linearity, allowing the robot to learn complex decision boundaries.
        self.relu = nn.ReLU()
        
        # Layer 2: Hidden (64) -> Output (2)
        # Output 0: Linear Velocity (x)
        # Output 1: Angular Velocity (z)
        self.layer2 = nn.Linear(64, 2)

    def forward(self, x):
        """Forward pass: computes the output given a LiDAR input x."""
        x = self.layer1(x)
        x = self.relu(x)
        x = self.layer2(x)
        return x

if __name__ == "__main__":
    # Quick test to verify architecture dimensions
    model = SimpleMLP()
    print("🧠 Model Architecture Created:")
    print(model)