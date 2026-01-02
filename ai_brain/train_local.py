"""
    Loads processed .pt data and trains the SimpleMLP model using 
    Mean Squared Error (MSE) loss and the Adam optimizer.
    Saves the trained weights to 'local_model.pth'.
"""

import torch
import torch.nn as nn
import torch.optim as optim
import os
from simple_mlp import SimpleMLP

def train():
    print("🚀 Starting Training...")
    
    # --- Path Setup ---
    # Automatically locate the 'data' folder relative to this script
    current_dir = os.path.dirname(os.path.abspath(__file__))
    data_dir = os.path.join(current_dir, 'data')
    
    X_path = os.path.join(data_dir, 'X_train.pt')
    y_path = os.path.join(data_dir, 'y_train.pt')
    model_save_path = os.path.join(data_dir, 'local_model.pth')

    # --- Load Data ---
    try: 
        print(f"📂 Loading data from: {data_dir}")
        X_train = torch.load(X_path)
        y_train = torch.load(y_path)
    except FileNotFoundError:
        print(f"❌ Error: Could not find .pt files in {data_dir}")
        print("   Run process_data.py first!")
        return
    
    # --- Model & Hyperparameters ---
    model = SimpleMLP()
    criterion = nn.MSELoss()  # Mean Squared Error: Good for regression (predicting continuous values)
    optimizer = optim.Adam(model.parameters(), lr=0.001) # Adam is efficient for generic MLPs

    epochs = 100
    print(f"🧠 Training for {epochs} epochs...")

    # --- Training Loop ---
    for epoch in range(epochs):
        # 1. Forward Pass: Make a prediction
        predictions = model(X_train)
        
        # 2. Compute Loss: How bad was the guess?
        loss = criterion(predictions, y_train)
        
        # 3. Backward Pass: Learn from mistakes
        optimizer.zero_grad()   # Clear previous gradients (essential!)
        loss.backward()         # Calculate new gradients
        optimizer.step()        # Update weights
        
        # Log progress every 10 epochs
        if (epoch+1) % 10 == 0:
            print(f"   Epoch [{epoch+1}/{epochs}], Loss: {loss.item():.4f}")

    # --- Save Result ---
    torch.save(model.state_dict(), model_save_path)
    print(f"✅ Success! Trained model saved to: {model_save_path}")

if __name__ == "__main__":
    train()