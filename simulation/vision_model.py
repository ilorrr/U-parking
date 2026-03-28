# ============================================================
# vision_model.py
# Lightweight CNN for parking spot occupancy classification.
#
# Architecture: 4 conv layers + 2 FC layers
# Input:  128x128 RGB patch (center crop from downward camera)
# Output: P(occupied)
#
# Usage:
#   python vision_model.py                     # train from vision_data/
#   python vision_model.py --data path/to/data # custom data dir
#   python vision_model.py --evaluate          # eval only (no training)
# ============================================================

import os
import sys
import json
import argparse
import numpy as np

# ---------- Check for PyTorch ----------
try:
    import torch
    import torch.nn as nn
    import torch.optim as optim
    from torch.utils.data import Dataset, DataLoader, random_split
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False

import cv2


# ============================================================
# DATASET
# ============================================================

class ParkingSpotDataset(Dataset):
    """
    Loads patches from vision_data/occupied/ and vision_data/empty/.
    Returns (image_tensor, label) where label is 1=occupied, 0=empty.
    """

    def __init__(self, data_dir="vision_data", img_size=128, augment=False):
        self.img_size = img_size
        self.augment  = augment
        self.samples  = []  # (filepath, label)

        occ_dir   = os.path.join(data_dir, "occupied")
        empty_dir = os.path.join(data_dir, "empty")

        if os.path.isdir(occ_dir):
            for f in sorted(os.listdir(occ_dir)):
                if f.lower().endswith((".jpg", ".png")):
                    self.samples.append((os.path.join(occ_dir, f), 1))

        if os.path.isdir(empty_dir):
            for f in sorted(os.listdir(empty_dir)):
                if f.lower().endswith((".jpg", ".png")):
                    self.samples.append((os.path.join(empty_dir, f), 0))

        print(f"  [Model] Dataset: {len(self.samples)} samples "
              f"({sum(1 for _,l in self.samples if l==1)} occupied, "
              f"{sum(1 for _,l in self.samples if l==0)} empty)")

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        path, label = self.samples[idx]
        img = cv2.imread(path)
        if img is None:
            # Return a blank image if file is corrupted
            img = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)

        img = cv2.resize(img, (self.img_size, self.img_size))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Data augmentation (training only)
        if self.augment:
            # Random horizontal flip
            if np.random.random() > 0.5:
                img = np.fliplr(img).copy()
            # Random vertical flip
            if np.random.random() > 0.5:
                img = np.flipud(img).copy()
            # Random brightness adjustment
            delta = np.random.randint(-30, 30)
            img = np.clip(img.astype(np.int16) + delta, 0, 255).astype(np.uint8)
            # Random rotation (0, 90, 180, 270)
            k = np.random.randint(0, 4)
            img = np.rot90(img, k).copy()

        # Normalize to [0, 1] and convert to CHW tensor
        tensor = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        return tensor, torch.tensor(label, dtype=torch.float32)


# ============================================================
# MODEL ARCHITECTURE
# ============================================================

class ParkingSpotCNN(nn.Module):
    """
    Lightweight CNN for binary classification of parking spot patches.

    Architecture:
      Conv2d(3→16) → ReLU → MaxPool
      Conv2d(16→32) → ReLU → MaxPool
      Conv2d(32→64) → ReLU → MaxPool
      Conv2d(64→128) → ReLU → AdaptiveAvgPool
      FC(128→64) → ReLU → Dropout
      FC(64→1) → Sigmoid
    """

    def __init__(self):
        super().__init__()
        self.features = nn.Sequential(
            nn.Conv2d(3, 16, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(16, 32, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.MaxPool2d(2),

            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(inplace=True),
            nn.AdaptiveAvgPool2d(1),
        )
        self.classifier = nn.Sequential(
            nn.Flatten(),
            nn.Linear(128, 64),
            nn.ReLU(inplace=True),
            nn.Dropout(0.3),
            nn.Linear(64, 1),
            nn.Sigmoid(),
        )

    def forward(self, x):
        x = self.features(x)
        x = self.classifier(x)
        return x.squeeze(-1)


# ============================================================
# TRAINING
# ============================================================

def train_model(data_dir="vision_data", epochs=30, batch_size=16,
                lr=0.001, val_split=0.2, save_path="vision_model.pth"):
    """
    Train the CNN on collected parking spot patches.

    Args:
        data_dir:   path to vision_data/ with occupied/ and empty/ subdirs
        epochs:     number of training epochs
        batch_size: batch size
        lr:         learning rate
        val_split:  fraction of data used for validation
        save_path:  where to save the trained model weights

    Returns:
        dict with training metrics (loss, accuracy per epoch)
    """
    if not HAS_TORCH:
        print("ERROR: PyTorch is required. Install with:")
        print("  pip install torch torchvision")
        return None

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"  [Model] Device: {device}")

    # Load dataset
    full_dataset = ParkingSpotDataset(data_dir, augment=True)
    if len(full_dataset) < 10:
        print("  [Model] ERROR: Need at least 10 samples to train. "
              "Run more scans with data collection enabled.")
        return None

    # Split into train/val
    val_size   = max(2, int(len(full_dataset) * val_split))
    train_size = len(full_dataset) - val_size
    train_ds, val_ds = random_split(full_dataset, [train_size, val_size])

    # Disable augmentation for validation subset
    val_dataset_no_aug = ParkingSpotDataset(data_dir, augment=False)
    val_indices = val_ds.indices
    val_ds_clean = torch.utils.data.Subset(val_dataset_no_aug, val_indices)

    train_loader = DataLoader(train_ds, batch_size=batch_size, shuffle=True,
                              num_workers=0)
    val_loader   = DataLoader(val_ds_clean, batch_size=batch_size, shuffle=False,
                              num_workers=0)

    print(f"  [Model] Train: {train_size}, Val: {val_size}")

    # Model, loss, optimizer
    model     = ParkingSpotCNN().to(device)
    criterion = nn.BCELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(
        optimizer, mode='min', factor=0.5, patience=5)

    # Count parameters
    n_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    print(f"  [Model] Parameters: {n_params:,}")

    history = {"train_loss": [], "val_loss": [], "val_acc": []}
    best_acc = 0.0

    for epoch in range(1, epochs + 1):
        # --- Train ---
        model.train()
        train_loss = 0.0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(device), labels.to(device)
            optimizer.zero_grad()
            preds = model(imgs)
            loss  = criterion(preds, labels)
            loss.backward()
            optimizer.step()
            train_loss += loss.item() * imgs.size(0)
        train_loss /= train_size

        # --- Validate ---
        model.eval()
        val_loss = 0.0
        correct  = 0
        total    = 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(device), labels.to(device)
                preds = model(imgs)
                loss  = criterion(preds, labels)
                val_loss += loss.item() * imgs.size(0)
                predicted = (preds > 0.5).float()
                correct  += (predicted == labels).sum().item()
                total    += labels.size(0)
        val_loss /= val_size
        val_acc   = correct / total if total > 0 else 0.0

        scheduler.step(val_loss)
        history["train_loss"].append(train_loss)
        history["val_loss"].append(val_loss)
        history["val_acc"].append(val_acc)

        if epoch % 5 == 0 or epoch == 1:
            print(f"  Epoch {epoch:3d}/{epochs}  |  "
                  f"Train loss: {train_loss:.4f}  |  "
                  f"Val loss: {val_loss:.4f}  |  "
                  f"Val acc: {val_acc:.1%}")

        # Save best model
        if val_acc > best_acc:
            best_acc = val_acc
            torch.save({
                "model_state_dict": model.state_dict(),
                "epoch": epoch,
                "val_acc": val_acc,
                "n_params": n_params,
            }, save_path)

    print(f"\n  [Model] Training complete. Best val accuracy: {best_acc:.1%}")
    print(f"  [Model] Saved to: {save_path}")

    # Save training history
    hist_path = save_path.replace(".pth", "_history.json")
    with open(hist_path, "w") as f:
        json.dump(history, f, indent=2)

    return history


# ============================================================
# EVALUATION
# ============================================================

def evaluate_model(data_dir="vision_data", model_path="vision_model.pth"):
    """Run evaluation on the full dataset and print confusion matrix."""
    if not HAS_TORCH:
        print("ERROR: PyTorch required.")
        return

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    # Load model
    model = ParkingSpotCNN().to(device)
    checkpoint = torch.load(model_path, map_location=device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()
    print(f"  [Model] Loaded: {model_path} "
          f"(epoch {checkpoint['epoch']}, acc {checkpoint['val_acc']:.1%})")

    # Load data (no augmentation)
    dataset = ParkingSpotDataset(data_dir, augment=False)
    loader  = DataLoader(dataset, batch_size=32, shuffle=False, num_workers=0)

    # Predict
    tp = fp = tn = fn = 0
    with torch.no_grad():
        for imgs, labels in loader:
            imgs, labels = imgs.to(device), labels.to(device)
            preds = (model(imgs) > 0.5).float()
            tp += ((preds == 1) & (labels == 1)).sum().item()
            fp += ((preds == 1) & (labels == 0)).sum().item()
            tn += ((preds == 0) & (labels == 0)).sum().item()
            fn += ((preds == 0) & (labels == 1)).sum().item()

    total     = tp + fp + tn + fn
    accuracy  = (tp + tn) / total if total > 0 else 0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1        = (2 * precision * recall / (precision + recall)
                 if (precision + recall) > 0 else 0)

    print(f"\n  {'='*50}")
    print(f"  EVALUATION RESULTS")
    print(f"  {'='*50}")
    print(f"  Accuracy:  {accuracy:.1%}  ({tp+tn}/{total})")
    print(f"  Precision: {precision:.1%}")
    print(f"  Recall:    {recall:.1%}")
    print(f"  F1 Score:  {f1:.3f}")
    print(f"\n  Confusion matrix:")
    print(f"                 Predicted")
    print(f"                 Occ    Empty")
    print(f"  Actual Occ  [{tp:5d}  {fn:5d}]")
    print(f"  Actual Empty[{fp:5d}  {tn:5d}]")
    print(f"  {'='*50}")

    return {"accuracy": accuracy, "precision": precision,
            "recall": recall, "f1": f1}


# ============================================================
# CLI
# ============================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train parking spot classifier")
    parser.add_argument("--data", default="vision_data",
                        help="Path to training data directory")
    parser.add_argument("--epochs", type=int, default=30)
    parser.add_argument("--batch-size", type=int, default=16)
    parser.add_argument("--lr", type=float, default=0.001)
    parser.add_argument("--save", default="vision_model.pth",
                        help="Output model path")
    parser.add_argument("--evaluate", action="store_true",
                        help="Evaluate existing model instead of training")
    args = parser.parse_args()

    if args.evaluate:
        evaluate_model(data_dir=args.data, model_path=args.save)
    else:
        train_model(data_dir=args.data, epochs=args.epochs,
                    batch_size=args.batch_size, lr=args.lr,
                    save_path=args.save)
        print("\nRunning evaluation on trained model...")
        evaluate_model(data_dir=args.data, model_path=args.save)
