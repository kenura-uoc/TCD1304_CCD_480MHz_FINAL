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
# TRAINING LOGIC (Repeated K-Fold CV)
# ─────────────────────────────────────────────────────────────
def train_and_eval_cnn(X, Y, epochs=150, lr=1e-3, n_repeats=3):
    """Run Repeated K-Fold CV to produce mean/std predictions per sample."""
    from sklearn.metrics import mean_absolute_error
    input_len = X.shape[1]
    n = len(Y)

    all_preds_a = [[] for _ in range(n)]
    all_preds_b = [[] for _ in range(n)]

    repeat_metrics = {"r2_a": [], "r2_b": [], "rmse_a": [], "rmse_b": [], "mae_a": [], "mae_b": []}

    for rep in range(n_repeats):
        print(f"\n  ── Repeat {rep+1}/{n_repeats} ──")
        kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42 + rep)
        fold_preds = np.zeros_like(Y)

        for fold, (train_idx, val_idx) in enumerate(kf.split(X)):
            print(f"    Fold {fold+1}/{N_SPLITS}...")

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
            fold_preds[val_idx] = pred

        # Accumulate per-sample predictions
        for idx in range(n):
            all_preds_a[idx].append(fold_preds[idx, 0])
            all_preds_b[idx].append(fold_preds[idx, 1])

        # Metrics for this repeat
        repeat_metrics["r2_a"].append(r2_score(Y[:, 0], fold_preds[:, 0]))
        repeat_metrics["r2_b"].append(r2_score(Y[:, 1], fold_preds[:, 1]))
        repeat_metrics["rmse_a"].append(np.sqrt(mean_squared_error(Y[:, 0], fold_preds[:, 0])))
        repeat_metrics["rmse_b"].append(np.sqrt(mean_squared_error(Y[:, 1], fold_preds[:, 1])))
        repeat_metrics["mae_a"].append(mean_absolute_error(Y[:, 0], fold_preds[:, 0]))
        repeat_metrics["mae_b"].append(mean_absolute_error(Y[:, 1], fold_preds[:, 1]))

    # Compute mean and std across repeats
    mean_preds = np.zeros_like(Y)
    std_preds  = np.zeros_like(Y)
    for idx in range(n):
        mean_preds[idx, 0] = np.mean(all_preds_a[idx])
        mean_preds[idx, 1] = np.mean(all_preds_b[idx])
        std_preds[idx, 0]  = np.std(all_preds_a[idx])
        std_preds[idx, 1]  = np.std(all_preds_b[idx])

    return mean_preds, std_preds, repeat_metrics


def plot_premium_results(Y, mean_preds, std_preds, metrics, out_path):
    """Premium plot with error bars and 95% CI boxes, matching PLS style."""
    from sklearn.metrics import mean_absolute_error
    fig = plt.figure(figsize=(16, 7))
    gs = gridspec.GridSpec(1, 2)
    targets = ["Chl-a", "Chl-b"]

    for i in range(2):
        ax = fig.add_subplot(gs[0, i])
        y_true = Y[:, i]
        y_mean = mean_preds[:, i]
        y_std  = std_preds[:, i]

        r2   = r2_score(y_true, y_mean)
        rmse = np.sqrt(mean_squared_error(y_true, y_mean))
        mae  = mean_absolute_error(y_true, y_mean)

        # 95% CI from repeated CV
        def get_ci(key):
            m_list = metrics[key]
            return [np.min(m_list), np.max(m_list)]

        suffix = 'a' if i == 0 else 'b'
        r2_ci   = get_ci(f"r2_{suffix}")
        rmse_ci = get_ci(f"rmse_{suffix}")
        mae_ci  = get_ci(f"mae_{suffix}")

        # Plot limits
        lim_min = min(y_true.min(), y_mean.min()) * 0.95
        lim_max = max(y_true.max(), y_mean.max()) * 1.05
        ax.plot([lim_min, lim_max], [lim_min, lim_max], 'r--', lw=1.5, label="Ideal (y=x)")

        # Error bars + Red Squares
        ax.errorbar(y_true, y_mean, yerr=y_std, fmt='rs', mfc='salmon', mec='darkred',
                    ms=6, alpha=0.7, capsize=3, elinewidth=1, label="Test samples (CV)")

        # Summary box
        meta_text = (f"Repeated CV (n=3):\n"
                     f" R2  [{r2_ci[0]:.3f}, {r2_ci[1]:.3f}]\n"
                     f" RMSE [{rmse_ci[0]:.3f}, {rmse_ci[1]:.3f}]\n"
                     f" MAE  [{mae_ci[0]:.3f}, {mae_ci[1]:.3f}]")

        props = dict(boxstyle='round', facecolor='lightyellow', alpha=0.8)
        ax.text(0.05, 0.95, meta_text, transform=ax.transAxes, fontsize=10,
                verticalalignment='top', bbox=props, fontfamily='monospace')

        ax.set_xlim(lim_min, lim_max)
        ax.set_ylim(lim_min, lim_max)
        ax.set_xlabel("Actual Concentration (mg/L)", fontsize=11)
        ax.set_ylabel("Predicted Concentration (mg/L)", fontsize=11)
        ax.set_title(f"Chlorophyll-{suffix} | TEST (R2={r2:.3f}, RMSE={rmse:.3f}, MAE={mae:.3f})",
                     fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower right')

    fig.suptitle("1D CNN Dual-Output (Repeated 5-Fold CV, n_repeats=3)",
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"  Saved premium plot: {out_path.name}")


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

    print("\n  Starting 1D CNN training (Repeated K-Fold CV, n_repeats=3)...")
    mean_preds, std_preds, metrics = train_and_eval_cnn(X_combined, Y_combined)

    # ── Evaluation ──
    for i, target in enumerate(["Chl-a", "Chl-b"]):
        r2 = r2_score(Y_combined[:, i], mean_preds[:, i])
        rmse = np.sqrt(mean_squared_error(Y_combined[:, i], mean_preds[:, i]))
        print(f"  {target}: R²={r2:.4f}, RMSE={rmse:.4f} mg/L")
        print(f"    R2 CI:   [{min(metrics[f'r2_{chr(97+i)}']):.3f}, {max(metrics[f'r2_{chr(97+i)}']):.3f}]")
        print(f"    RMSE CI: [{min(metrics[f'rmse_{chr(97+i)}']):.3f}, {max(metrics[f'rmse_{chr(97+i)}']):.3f}]")

    # ── Premium Plot ──
    plot_path = PLOT_DIR / "real_data_cnn_evaluation.png"
    plot_premium_results(Y_combined, mean_preds, std_preds, metrics, plot_path)
    print(f"\n  Plot saved: {plot_path.name}")
    print("  Done.")

