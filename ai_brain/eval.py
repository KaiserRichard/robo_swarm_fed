"""
@file eval.py
@description
Sanity check for trained Phase 2 model.
"""

import torch
import numpy as np
from model import SimpleMLP

MODEL_PATH = "data/local_model.pth"


def load_model():
    model = SimpleMLP()
    # load_state_dict : copies the weights into the model
    # The model remembers what it learned in phase 2
    model.load_state_dict(
        torch.load(MODEL_PATH, map_location="cpu")
    )
    model.eval() #Dropout OFF, BatchNorm frozen, Deterministic output
    return model
    
def test_single_sample():
    """
    Run inference on a fake LiDAR scan to perform a sanity check.

    This function verifies that:
    - The trained model can be loaded correctly
    - The input tensor shape is valid
    - The model produces finite, reasonable outputs
    """

    # Load the trained model (architecture + learned weights)
    model = load_model()

    # Create a fake LiDAR scan where all distances are 0.5 (normalized).
    # Interpretation:
    #   The robot sees free space in all directions.
    fake_scan = np.ones(360, dtype=np.float32) * 0.5

    # Convert NumPy array to PyTorch tensor and add batch dimension.
    #
    # Before unsqueeze:
    #   shape = (360,)  -> one LiDAR scan, no batch information
    #
    # After unsqueeze(0):
    #   shape = (1, 360)
    #
    # Meaning:
    #   batch_size = 1 (one robot observation)
    #   number_of_features = 360 (LiDAR beams)
    #
    # PyTorch models ALWAYS expect a batch dimension.
    x = torch.tensor(fake_scan).unsqueeze(0)

    # Disable gradient computation because this is inference, not training.
    # This makes execution faster and prevents unnecessary memory usage.
    with torch.no_grad():
        output = model(x)

    # Print the raw model output.
    #
    # Expected properties:
    # - Shape: (1, 2)
    # - Finite values (no NaN or ±inf)
    # - Reasonable magnitudes for robot control
    #
    # Examples:
    #   Good:  [[0.21, -0.05]]
    #   Bad:   [[nan, nan]] or [[inf, -inf]]  -> model is broken
    print("Model output:", output.numpy())



if __name__ == "__main__":
    test_single_sample()
