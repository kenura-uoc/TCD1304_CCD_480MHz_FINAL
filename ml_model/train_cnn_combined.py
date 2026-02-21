#!/usr/bin/env python3
"""
train_cnn_combined.py
======================
Phase 2: Dual-Analyte Combined Model using 1D CNN.

Trains a single 1D Convolutional Neural Network that simultaneously predicts 
BOTH Chl-a and Chl-b concentrations from a two-channel spectral input:
  - Channel 1: spectrum captured with 405 nm laser excitation
  - Channel 2: spectrum captured with 450 nm laser excitation

This script adapts the architecture from train_cnn_v1.py to handle the 
concatenated real-world dataset from train_combined_model.py.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.model_selection import KFold
from sklearn.metrics import r2_score, mean_squared_error
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR     = Path(__file__).parent.resolve()
DATA_DIR_A     = SCRIPT_DIR / "data" / "chl_a"
DATA_DIR_B     = SCRIPT_DIR / "data" / "chl_b"
BG_DIR         = SCRIPT_DIR / "data" / "background_data"
CSV_A          = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B          = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"
PLOT_DIR       = SCRIPT_DIR / "cnn_results"
PLOT_DIR.mkdir(exist_ok=True)

ROI_START      = 1300
ROI_END        = 3500
SG_WIN         = 11
SG_POLY        = 2
SG_DWIN        = 11
SG_DPOLY       = 3
N_SPLITS       = 5  # Use 5-fold for CNN to keep training time reasonable

BG_MAP = {
    240: "background-240.csv", 250: "background-250.csv",
    300: "background-300.csv", 500: "background-500.csv",
    1000: "background-1000.csv"
}
BG_AVAIL = sorted(BG_MAP.keys())

# ─────────────────────────────────────────────────────────────
# PREPROCESSING HELPERS
# ─────────────────────────────────────────────────────────────
def load_spectrum(path):
    df = pd.read_csv(path)
    return np.mean(df.iloc[:, 1:].values.astype(float), axis=0)

def load_bg(int_time):
    nearest = min(BG_AVAIL, key=lambda t: abs(t - int_time))
    path = BG_DIR / BG_MAP[nearest]
    return load_spectrum(path) if path.exists() else None

def snv(x):
    m, s = x.mean(), x.std()
    return (x - m) / s if s > 1e-12 else x - m

def preprocess(raw, bg, int_time):
    if bg is not None and len(bg) == len(raw):
        raw = raw - bg
    roi = raw[ROI_START:ROI_END]
    roi = roi / int_time
    sm  = savgol_filter(roi, SG_WIN, SG_POLY)
    drv = savgol_filter(sm, SG_DWIN, SG_DPOLY, deriv=1)
    return snv(drv)

def load_dataset(csv_path, data_dir):
    meta = pd.read_csv(csv_path).dropna(subset=["Sample Con", "integration time(mS)"])
    spectra = []
    for i, row in meta.iterrows():
        fpath = data_dir / f"{i+1}.csv"
        if not fpath.exists(): continue
        raw = load_spectrum(fpath)
        if len(raw) < ROI_END: continue
        bg = load_bg(int(row["integration time(mS)"]))
        feat = preprocess(raw, bg, row["integration time(mS)"])
        spectra.append(feat)
    return np.array(spectra)

# ─────────────────────────────────────────────────────────────
# CNN ARCHITECTURE (Dual Output)
# ─────────────────────────────────────────────────────────────
class CNN1D_Dual(nn.Module):
    def __init__(self, input_len, kernel_sizes=(51, 25, 11), filters=(16, 32, 64)):
        super().__init__()
        k1, k2, k3 = kernel_sizes
        f1, f2, f3 = filters

        self.features = nn.Sequential(
            nn.Conv1d(1, f1, kernel_size=k1, padding=k1//2),
            nn.BatchNorm1d(f1),
            nn.ReLU(),
            nn.MaxPool1d(4),
            nn.Dropout(0.1),

            nn.Conv1d(f1, f2, kernel_size=k2, padding=k2//2),
            nn.BatchNorm1d(f2),
            nn.ReLU(),
            nn.MaxPool1d(4),
            nn.Dropout(0.1),

            nn.Conv1d(f2, f3, kernel_size=k3, padding=k3//2),
            nn.BatchNorm1d(f3),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(f3 * 8, 64),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(64, 2),  # Dual output: [Chl-a, Chl-b]
        )

    def forward(self, x):
        return self.regressor(self.features(x))

# ─────────────────────────────────────────────────────────────
# TRAINING LOGIC
# ─────────────────────────────────────────────────────────────
def train_and_eval_cnn(X, Y, epochs=150, lr=1e-3):
    kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
    y_pred_all = np.zeros_like(Y, dtype=float)

    input_len = X.shape[1]
    
    for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
        print(f"  Training Fold {fold+1}/{N_SPLITS}...")
        
        X_train = torch.FloatTensor(X[train_idx]).unsqueeze(1)
        y_train = torch.FloatTensor(Y[train_idx])
        X_val   = torch.FloatTensor(X[val_idx]).unsqueeze(1)
        
        ds = TensorDataset(X_train, y_train)
        dl = DataLoader(ds, batch_size=16, shuffle=True)
        
        model = CNN1D_Dual(input_len)
        optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
        criterion = nn.MSELoss()
        
        for epoch in range(epochs):
            model.train()
            for xb, yb in dl:
                optimizer.zero_grad()
                loss = criterion(model(xb), yb)
                loss.backward()
                optimizer.step()
            scheduler.step()
            
        model.eval()
        with torch.no_grad():
            pred = model(X_val).numpy()
        y_pred_all[val_idx] = pred

    return y_pred_all

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("  Loading real data...")
    X_a = load_dataset(CSV_A, DATA_DIR_A)
    X_b = load_dataset(CSV_B, DATA_DIR_B)
    
    meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])["Sample Con"].values
    meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])["Sample Con"].values
    
    min_n = min(len(X_a), len(X_b))
    X_combined = np.hstack([X_a[:min_n], X_b[:min_n]])
    Y_combined = np.column_stack([meta_a[:min_n], meta_b[:min_n]])
    
    print(f"  Dataset size: {min_n} samples")
    print(f"  Input features: {X_combined.shape[1]}")
    
    print("\n  Starting 1D CNN training (K-Fold CV)...")
    y_pred = train_and_eval_cnn(X_combined, Y_combined)
    
    # ── Evaluation ──
    for i, target in enumerate(["Chl-a", "Chl-b"]):
        r2 = r2_score(Y_combined[:, i], y_pred[:, i])
        rmse = np.sqrt(mean_squared_error(Y_combined[:, i], y_pred[:, i]))
        print(f"  {target}: R²={r2:.4f}, RMSE={rmse:.4f} mg/L")
        
    # ── Plotting ──
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    targets = ["Chl-a", "Chl-b"]
    colors = ["royalblue", "forestgreen"]
    
    for i in range(2):
        ax = axes[i]
        y_true = Y_combined[:, i]
        y_p = y_pred[:, i]
        r2 = r2_score(y_true, y_p)
        rmse = np.sqrt(mean_squared_error(y_true, y_p))
        
        lims = [min(y_true.min(), y_p.min())*0.9, max(y_true.max(), y_p.max())*1.1]
        ax.scatter(y_true, y_p, color=colors[i], alpha=0.6, edgecolors='k')
        ax.plot(lims, lims, 'r--', lw=1.5)
        ax.set_xlim(lims); ax.set_ylim(lims)
        ax.set_title(f"{targets[i]} (CNN)\n$R^2$={r2:.4f}, RMSE={rmse:.3f}")
        ax.set_xlabel("Actual (mg/L)")
        ax.set_ylabel("Predicted (mg/L)")
        ax.grid(True, alpha=0.3)
        
    plt.suptitle("1D CNN Dual-Output Evaluation on Real-World Collected Data", fontsize=14, fontweight='bold')
    plt.tight_layout()
    plot_path = PLOT_DIR / "real_data_cnn_evaluation.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\n  Plot saved: {plot_path.name}")
    print("  Done.")
