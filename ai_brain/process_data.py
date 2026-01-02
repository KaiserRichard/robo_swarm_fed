"""
    Loads raw CSV data collected from the robot, cleans it, 
    normalizes LiDAR readings, and converts it into PyTorch tensors (.pt).
    It saves the processed tensors to the 'data/' directory for training.
"""

import torch
import pandas as pd
import numpy as np
import os

def load_and_process_data(csv_path):
    """
    Reads CSV, removes NaNs, and splits into Inputs (X) and Targets (y).
    """
    print(f"🔄 Loading data from {csv_path}...") 
    
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"❌ File not found: {csv_path}")
    
    # Load into DataFrame
    df = pd.read_csv(csv_path)
    df = df.dropna()  # Remove incomplete rows to prevent errors

    # 1. Separate Inputs (X) and Outputs (y)
    # y = Target: [Linear Velocity, Angular Velocity] (Columns 0-1)
    y = df.iloc[:, 0:2].values
    
    # X = Features: [LiDAR ranges 0-359] (Columns 2 to end)
    X = df.iloc[:, 2:].values

    # 2. Normalize LiDAR Data
    # LiDAR sensors often return 'inf' for far distances. 
    # We clip values to a max of 10.0 meters and scale to 0.0-1.0 range.
    # This helps the Neural Network learn faster and more visibly.
    X = np.clip(X, 0, 10.0) / 10.0

    # 3. Convert to PyTorch Tensors
    X_tensor = torch.tensor(X, dtype=torch.float32)
    y_tensor = torch.tensor(y, dtype=torch.float32)
    
    return X_tensor, y_tensor

if __name__ == "__main__":
    # --- Robust Path Handling ---
    # Finds the absolute path of THIS script to locate the 'data' folder safely.
    # This prevents "File Not Found" errors when running from different directories.
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(current_dir, 'data')
    csv_path = os.path.join(data_dir, 'robot_data.csv')

    try: 
        X_train, y_train = load_and_process_data(csv_path)

        # Save processed tensors to 'data/' folder to keep repo clean
        torch.save(X_train, os.path.join(data_dir, 'X_train.pt'))
        torch.save(y_train, os.path.join(data_dir, 'y_train.pt')) 
        
        print(f"💾 Success: Processed {len(X_train)} samples.")
        print(f"   Saved .pt files to: {data_dir}")
        
    except Exception as e:
        print(f"❌ Error: {e}")