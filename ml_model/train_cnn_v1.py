#!/usr/bin/env python3
"""
train_cnn_v1.py — 1D CNN for Chlorophyll Concentration from CCD Spectra
========================================================================
Uses a 1D Convolutional Neural Network with hyperparameter grid search
to find optimal kernel sizes, filter counts, and learning rate.

Requirements:  pip install torch

Usage:
    python train_cnn_v1.py              # full grid search (12 combos)
    python train_cnn_v1.py --quick      # quick run with best defaults
"""

import numpy as np
import sys
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.model_selection import KFold
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import cross_val_predict

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

print("✓ PyTorch found — using 1D CNN")

OUT_DIR = Path(__file__).parent / "cnn_results"
OUT_DIR.mkdir(exist_ok=True)

# ── Sensor / pipeline constants ──────────────────────────────
TOTAL_PIXELS = 3694
ROI_START, ROI_END = 1300, 3200
ROI_LEN = ROI_END - ROI_START  # 1900
SG_SMOOTH_W, SG_SMOOTH_P = 11, 2
SG_DERIV_W,  SG_DERIV_P  = 11, 3


# ══════════════════════════════════════════════════════════════
#  Synthetic data generator
# ══════════════════════════════════════════════════════════════
def generate_spectrum(concentration, int_time_ms=300, noise_frames=32):
    x = np.arange(TOTAL_PIXELS, dtype=float)
    peak_pos = 1950 + concentration * 12.0
    sigma = (180 + concentration * 6.0) / 2.355
    max_signal = 3200
    signal = (max_signal * (1.0 - np.exp(-0.12 * concentration)) / 300.0) * int_time_ms
    spectrum_clean = signal * np.exp(-0.5 * ((x - peak_pos) / sigma)**2)
    frames = []
    for _ in range(noise_frames):
        frame = spectrum_clean + 200
        frame += np.random.normal(0, np.sqrt(np.maximum(frame, 1)))
        frame += np.random.normal(0, 15, TOTAL_PIXELS)
        frames.append(frame)
    return np.mean(frames, axis=0)


def preprocess(spectrum, int_time_ms, background):
    s = (spectrum - background) / (int_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    roi = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi = savgol_filter(roi, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std = roi.mean(), roi.std()
    return (roi - mean) / (std + 1e-8)


def build_synthetic_data(n=300, frames=64):
    np.random.seed(42)
    concs = np.linspace(0.1, 20.0, n)
    bg = generate_spectrum(0.0, 300, frames)
    X = np.array([preprocess(generate_spectrum(c, 300, frames), 300, bg) for c in concs])
    return X, concs


# ══════════════════════════════════════════════════════════════
#  1D CNN Model — Configurable architecture
# ══════════════════════════════════════════════════════════════
class CNN1D(nn.Module):
    """1D CNN with configurable kernel sizes and filter counts."""
    def __init__(self, input_len, kernel_sizes=(21, 11, 5), filters=(16, 32, 64)):
        super().__init__()
        k1, k2, k3 = kernel_sizes
        f1, f2, f3 = filters

        self.features = nn.Sequential(
            # Block 1: broad spectral features
            nn.Conv1d(1, f1, kernel_size=k1, padding=k1//2),
            nn.BatchNorm1d(f1),
            nn.ReLU(),
            nn.MaxPool1d(4),
            nn.Dropout(0.1),

            # Block 2: medium features
            nn.Conv1d(f1, f2, kernel_size=k2, padding=k2//2),
            nn.BatchNorm1d(f2),
            nn.ReLU(),
            nn.MaxPool1d(4),
            nn.Dropout(0.1),

            # Block 3: fine features
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
            nn.Linear(64, 1),
        )

    def forward(self, x):
        return self.regressor(self.features(x)).squeeze(-1)


# ══════════════════════════════════════════════════════════════
#  Training function (K-Fold CV)
# ══════════════════════════════════════════════════════════════
def train_cnn_cv(X, y, kernel_sizes=(21, 11, 5), filters=(16, 32, 64),
                 epochs=80, lr=1e-3, n_splits=5, verbose=True):
    """Train with K-Fold CV. Returns overall R², RMSE, predictions."""
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=42)
    y_pred_all = np.zeros_like(y, dtype=float)

    for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
        X_train = torch.FloatTensor(X[train_idx]).unsqueeze(1)
        y_train = torch.FloatTensor(y[train_idx])
        X_val   = torch.FloatTensor(X[val_idx]).unsqueeze(1)

        ds = TensorDataset(X_train, y_train)
        dl = DataLoader(ds, batch_size=32, shuffle=True)

        model = CNN1D(X.shape[1], kernel_sizes, filters)
        optimizer = torch.optim.Adam(model.parameters(), lr=lr, weight_decay=1e-4)
        scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=epochs)
        criterion = nn.MSELoss()

        model.train()
        for epoch in range(epochs):
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

        if verbose:
            r2 = r2_score(y[val_idx], pred)
            rmse = np.sqrt(mean_squared_error(y[val_idx], pred))
            print(f"    Fold {fold+1}/{n_splits}: R²={r2:.5f}  RMSE={rmse:.4f}")

    overall_r2 = r2_score(y, y_pred_all)
    overall_rmse = np.sqrt(mean_squared_error(y, y_pred_all))
    return overall_r2, overall_rmse, y_pred_all


# ══════════════════════════════════════════════════════════════
#  FOCUSED GRID SEARCH (12 combos — fast)
# ══════════════════════════════════════════════════════════════
def grid_search_cnn(X, y):
    """Search over the most impactful hyperparameters."""
    print("\n" + "="*65)
    print("  GRID SEARCH — Finding optimal 1D CNN hyperparameters")
    print("="*65)

    # Focused search: kernel size matters most, then lr
    configs = [
        # (kernels,            filters,         lr)
        ((7,  5,  3),  (16, 32, 64),   1e-3),
        ((7,  5,  3),  (16, 32, 64),   3e-3),
        ((11, 7,  3),  (16, 32, 64),   1e-3),
        ((11, 7,  3),  (16, 32, 64),   3e-3),
        ((21, 11, 5),  (16, 32, 64),   1e-3),
        ((21, 11, 5),  (16, 32, 64),   3e-3),
        ((31, 15, 7),  (16, 32, 64),   1e-3),
        ((31, 15, 7),  (16, 32, 64),   3e-3),
        ((41, 21, 11), (16, 32, 64),   1e-3),
        ((41, 21, 11), (16, 32, 64),   3e-3),
        ((51, 25, 11), (16, 32, 64),   1e-3),
        ((51, 25, 11), (16, 32, 64),   3e-3),
    ]

    results = []
    for i, (kernels, filters, lr) in enumerate(configs):
        print(f"\n[{i+1}/{len(configs)}] kernels={kernels} filters={filters} lr={lr}")
        try:
            r2, rmse, _ = train_cnn_cv(X, y, kernels, filters,
                                        epochs=80, lr=lr,
                                        n_splits=5, verbose=False)
            results.append({
                "kernels": str(kernels), "filters": str(filters),
                "lr": lr, "r2": r2, "rmse": rmse,
            })
            print(f"    → R²={r2:.6f}  RMSE={rmse:.5f}")
        except Exception as e:
            print(f"    → FAILED: {e}")
            results.append({
                "kernels": str(kernels), "filters": str(filters),
                "lr": lr, "r2": -1, "rmse": 999,
            })

    # Sort by RMSE
    results.sort(key=lambda x: x["rmse"])

    print("\n" + "="*65)
    print("  TOP 5 CONFIGURATIONS (sorted by RMSE)")
    print("="*65)
    for j, r in enumerate(results[:5]):
        print(f"  #{j+1}  kernels={r['kernels']:20s}  lr={r['lr']:.4f}  "
              f"R²={r['r2']:.6f}  RMSE={r['rmse']:.5f}")

    # Save
    import pandas as pd
    df = pd.DataFrame(results)
    df.to_csv(OUT_DIR / "grid_search_results.csv", index=False)
    print(f"\n  Results saved to: {OUT_DIR / 'grid_search_results.csv'}")

    # ── Kernel size vs RMSE plot ──
    fig, ax = plt.subplots(figsize=(10, 5))
    # Group by kernel (average over lr)
    kernel_groups = {}
    for r in results:
        k = r["kernels"]
        if k not in kernel_groups:
            kernel_groups[k] = []
        kernel_groups[k].append(r["rmse"])

    kernel_labels = list(kernel_groups.keys())
    kernel_means  = [np.mean(v) for v in kernel_groups.values()]
    kernel_mins   = [np.min(v) for v in kernel_groups.values()]
    x_pos = range(len(kernel_labels))
    ax.bar(x_pos, kernel_means, alpha=0.5, color="steelblue", label="Mean RMSE")
    ax.scatter(x_pos, kernel_mins, c="red", s=80, zorder=5, label="Best RMSE")
    ax.set_xticks(list(x_pos))
    ax.set_xticklabels(kernel_labels, rotation=30, ha="right", fontsize=9)
    ax.set(xlabel="Kernel Sizes (Layer1, Layer2, Layer3)", ylabel="RMSE (ppm)",
           title="Kernel Size Impact on CNN Performance")
    ax.axhline(0.01, ls="--", c="red", alpha=0.3, label="0.01 ppm target")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "kernel_size_search.png", dpi=150)
    plt.close(fig)

    return results


# ══════════════════════════════════════════════════════════════
#  FINAL COMPARISON (Best CNN vs PLS)
# ══════════════════════════════════════════════════════════════
def run_final_comparison(X, y, best_kernels, best_filters, best_lr):
    print("\n" + "="*65)
    print("  FINAL COMPARISON — Best 1D CNN vs PLS")
    print("="*65)

    # PLS
    print("\n── PLS (5 components) ──")
    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    pls = PLSRegression(n_components=5, scale=True)
    y_pls = cross_val_predict(pls, X, y, cv=kf)
    pls_r2 = r2_score(y, y_pls)
    pls_rmse = np.sqrt(mean_squared_error(y, y_pls))
    pls_max = np.max(np.abs(y - y_pls))
    print(f"  R²={pls_r2:.6f}  RMSE={pls_rmse:.5f}  MaxErr={pls_max:.5f}")

    # Best CNN (200 epochs for final comparison)
    print(f"\n── 1D CNN (k={best_kernels}, lr={best_lr}) ──")
    cnn_r2, cnn_rmse, y_cnn = train_cnn_cv(
        X, y, best_kernels, best_filters, epochs=200, lr=best_lr
    )
    cnn_max = np.max(np.abs(y - y_cnn))
    print(f"\n  Overall: R²={cnn_r2:.6f}  RMSE={cnn_rmse:.5f}  MaxErr={cnn_max:.5f}")

    # ── 2x2 comparison plot ──
    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    axes[0, 0].scatter(y, y_pls, s=10, alpha=0.7, color="steelblue")
    axes[0, 0].plot([0, 20], [0, 20], "k--", alpha=0.4)
    axes[0, 0].set(xlabel="Actual (ppm)", ylabel="Predicted (ppm)",
                    title=f"PLS  |  R²={pls_r2:.6f}  RMSE={pls_rmse:.4f}")
    axes[0, 0].grid(True, alpha=0.3)

    axes[0, 1].scatter(y, y_cnn, s=10, alpha=0.7, color="coral")
    axes[0, 1].plot([0, 20], [0, 20], "k--", alpha=0.4)
    axes[0, 1].set(xlabel="Actual (ppm)", ylabel="Predicted (ppm)",
                    title=f"1D CNN  |  R²={cnn_r2:.6f}  RMSE={cnn_rmse:.4f}")
    axes[0, 1].grid(True, alpha=0.3)

    res_pls = y_pls - y
    axes[1, 0].scatter(y, res_pls, s=10, alpha=0.7, color="steelblue")
    axes[1, 0].axhline(0, c="k", ls="--", alpha=0.4)
    axes[1, 0].axhline(0.01, c="red", ls=":", alpha=0.3)
    axes[1, 0].axhline(-0.01, c="red", ls=":", alpha=0.3)
    axes[1, 0].set(xlabel="Actual (ppm)", ylabel="Residual (ppm)",
                    title=f"PLS Residuals  |  MaxErr={pls_max:.4f}")
    axes[1, 0].grid(True, alpha=0.3)

    res_cnn = y_cnn - y
    axes[1, 1].scatter(y, res_cnn, s=10, alpha=0.7, color="coral")
    axes[1, 1].axhline(0, c="k", ls="--", alpha=0.4)
    axes[1, 1].axhline(0.01, c="red", ls=":", alpha=0.3)
    axes[1, 1].axhline(-0.01, c="red", ls=":", alpha=0.3)
    axes[1, 1].set(xlabel="Actual (ppm)", ylabel="Residual (ppm)",
                    title=f"1D CNN Residuals  |  MaxErr={cnn_max:.4f}")
    axes[1, 1].grid(True, alpha=0.3)

    fig.suptitle(f"PLS vs 1D CNN (k={best_kernels}) — 300 Synthetic Spectra",
                 fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "pls_vs_cnn_comparison.png", dpi=150)
    plt.close(fig)

    # ── Summary ──
    print("\n" + "="*65)
    print("  COMPARISON SUMMARY")
    print("="*65)
    print(f"  {'Metric':<15} {'PLS':>12}  {'1D CNN':>12}")
    print(f"  {'R²':<15} {pls_r2:>12.6f}  {cnn_r2:>12.6f}")
    print(f"  {'RMSE (ppm)':<15} {pls_rmse:>12.5f}  {cnn_rmse:>12.5f}")
    print(f"  {'Max Err (ppm)':<15} {pls_max:>12.5f}  {cnn_max:>12.5f}")
    print("="*65)


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("="*65)
    print("  1D CNN — Hyperparameter Search + PLS Comparison")
    print("="*65)

    quick = "--quick" in sys.argv

    # Build dataset
    print("\n→ Generating 300 synthetic spectra (64-frame averaged)...")
    X, y = build_synthetic_data(n=300, frames=64)
    print(f"  Dataset shape: X={X.shape}, y={y.shape}")

    if quick:
        best_k = (21, 11, 5)
        best_f = (16, 32, 64)
        best_lr = 1e-3
    else:
        results = grid_search_cnn(X, y)
        best = results[0]
        best_k = eval(best["kernels"])
        best_f = eval(best["filters"])
        best_lr = best["lr"]

    # Final comparison with best config (more epochs)
    run_final_comparison(X, y, best_k, best_f, best_lr)
    print(f"\nAll plots saved to: {OUT_DIR.resolve()}")
