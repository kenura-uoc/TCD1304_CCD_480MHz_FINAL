#!/usr/bin/env python3
"""
gridsearch_dual_ideal.py — Hyperparameter Grid Search for Dual-Output Models
=============================================================================
Finds optimal hyperparameters for BOTH the dual-output PLS and 1D CNN models
on ideal simulation data (300 samples, 128× averaging, two-channel input).

Outputs:
  - cnn_results/dual_pls_gridsearch.csv
  - cnn_results/dual_cnn_gridsearch.csv
  - cnn_results/dual_gridsearch_comparison.png
"""

import time
import itertools
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from sklearn.cross_decomposition import PLSRegression
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import KFold
from sklearn.metrics import r2_score, mean_squared_error, mean_absolute_error
import warnings
warnings.filterwarnings("ignore")

# PyTorch
import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

# ── Import simulation helpers from simulation_combined.py ────
from simulation_combined import (
    generate_dataset, N_SAMPLES, ROI_LEN,
)

np.random.seed(42)
torch.manual_seed(42)

# ─────────────────────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
OUT_DIR    = SCRIPT_DIR / "cnn_results"
OUT_DIR.mkdir(exist_ok=True)

N_SPLITS = 5


# ═════════════════════════════════════════════════════════════
# SECTION A: PLS GRID SEARCH
# ═════════════════════════════════════════════════════════════
def pls_grid_search(X, Y):
    """Sweep n_components (1–15) with and without StandardScaler."""
    results = []
    max_comp = min(16, X.shape[0] // N_SPLITS)

    for scale in [False, True]:
        for n in range(1, max_comp):
            kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
            preds = np.zeros_like(Y, dtype=float)

            for tr, te in kf.split(X):
                X_tr, X_te = X[tr], X[te]
                if scale:
                    scaler = StandardScaler()
                    X_tr = scaler.fit_transform(X_tr)
                    X_te = scaler.transform(X_te)
                m = PLSRegression(n_components=n)
                m.fit(X_tr, Y[tr])
                preds[te] = m.predict(X_te)

            r2_a  = r2_score(Y[:, 0], preds[:, 0])
            r2_b  = r2_score(Y[:, 1], preds[:, 1])
            rmse_a = np.sqrt(mean_squared_error(Y[:, 0], preds[:, 0]))
            rmse_b = np.sqrt(mean_squared_error(Y[:, 1], preds[:, 1]))
            mae_a  = mean_absolute_error(Y[:, 0], preds[:, 0])
            mae_b  = mean_absolute_error(Y[:, 1], preds[:, 1])
            total_rmse = rmse_a + rmse_b

            results.append({
                "n_components": n,
                "scale": scale,
                "r2_a": r2_a, "r2_b": r2_b,
                "rmse_a": rmse_a, "rmse_b": rmse_b,
                "mae_a": mae_a, "mae_b": mae_b,
                "total_rmse": total_rmse,
            })
            tag = "scaled" if scale else "raw"
            print(f"  PLS [{tag}] n={n:2d}  "
                  f"Chl-a R²={r2_a:.4f} RMSE={rmse_a:.3f}  |  "
                  f"Chl-b R²={r2_b:.4f} RMSE={rmse_b:.3f}  |  "
                  f"ΣRMSe={total_rmse:.3f}")

    df = pd.DataFrame(results).sort_values("total_rmse")
    csv_path = OUT_DIR / "dual_pls_gridsearch.csv"
    df.to_csv(csv_path, index=False)
    print(f"\n  PLS results saved: {csv_path.name}")
    print(f"  Best: n_components={df.iloc[0]['n_components']:.0f}, "
          f"scale={df.iloc[0]['scale']}, "
          f"total_RMSE={df.iloc[0]['total_rmse']:.4f}")
    return df


# ═════════════════════════════════════════════════════════════
# SECTION B: CNN ARCHITECTURE (Configurable)
# ═════════════════════════════════════════════════════════════
class DualCNN1D(nn.Module):
    """1D CNN with configurable architecture and dual output."""
    def __init__(self, input_len,
                 kernel_sizes=(51, 25, 11),
                 filters=(16, 32, 64),
                 dropout_conv=0.1,
                 dropout_fc=0.2):
        super().__init__()
        k1, k2, k3 = kernel_sizes
        f1, f2, f3 = filters
        self.features = nn.Sequential(
            nn.Conv1d(1, f1, kernel_size=k1, padding=k1 // 2),
            nn.BatchNorm1d(f1), nn.ReLU(), nn.MaxPool1d(4),
            nn.Dropout(dropout_conv),

            nn.Conv1d(f1, f2, kernel_size=k2, padding=k2 // 2),
            nn.BatchNorm1d(f2), nn.ReLU(), nn.MaxPool1d(4),
            nn.Dropout(dropout_conv),

            nn.Conv1d(f2, f3, kernel_size=k3, padding=k3 // 2),
            nn.BatchNorm1d(f3), nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(f3 * 8, 128), nn.ReLU(), nn.Dropout(dropout_fc),
            nn.Linear(128, 32),     nn.ReLU(),
            nn.Linear(32, 2),
        )

    def forward(self, x):
        return self.regressor(self.features(x))


def train_one_fold(X_tr, Y_tr, X_te, input_len, config):
    """Train one fold with the given config dict."""
    model = DualCNN1D(
        input_len,
        kernel_sizes=config["kernels"],
        filters=config["filters"],
        dropout_conv=config["dropout_conv"],
        dropout_fc=config["dropout_fc"],
    )
    opt = torch.optim.Adam(model.parameters(), lr=config["lr"], weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=config["epochs"])
    loss_fn = nn.MSELoss()

    Xt = torch.FloatTensor(X_tr).unsqueeze(1)
    Yt = torch.FloatTensor(Y_tr)
    ds = TensorDataset(Xt, Yt)
    dl = DataLoader(ds, batch_size=config["batch_size"], shuffle=True)

    model.train()
    for _ in range(config["epochs"]):
        for xb, yb in dl:
            opt.zero_grad()
            loss_fn(model(xb), yb).backward()
            opt.step()
        scheduler.step()

    model.eval()
    with torch.no_grad():
        preds = model(torch.FloatTensor(X_te).unsqueeze(1)).numpy()
    return preds


# ═════════════════════════════════════════════════════════════
# SECTION C: CNN GRID SEARCH
# ═════════════════════════════════════════════════════════════
def cnn_grid_search(X, Y):
    """Sweep CNN hyperparameters with 5-fold CV on ideal data."""
    input_len = X.shape[1]

    # Define search space
    kernel_options  = [(51, 25, 11), (41, 21, 11), (31, 15, 7), (21, 11, 5), (11, 7, 3)]
    filter_options  = [(16, 32, 64), (32, 64, 128)]
    lr_options      = [1e-3, 3e-3]
    epoch_options   = [80, 120]
    batch_options   = [16, 32]
    dropout_conv_options = [0.1, 0.2]
    dropout_fc_options   = [0.2, 0.3]

    combos = list(itertools.product(
        kernel_options, filter_options, lr_options,
        epoch_options, batch_options,
        dropout_conv_options, dropout_fc_options,
    ))
    total = len(combos)
    print(f"\n  CNN Grid Search: {total} configurations × {N_SPLITS}-fold CV")
    print(f"  Estimated: {total * N_SPLITS} training runs\n")

    results = []
    t_start = time.time()

    for idx, (kernels, filters, lr, epochs, bs, dc, df_) in enumerate(combos):
        config = {
            "kernels": kernels, "filters": filters, "lr": lr,
            "epochs": epochs, "batch_size": bs,
            "dropout_conv": dc, "dropout_fc": df_,
        }

        kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
        preds = np.zeros_like(Y, dtype=float)

        for tr, te in kf.split(X):
            preds[te] = train_one_fold(X[tr], Y[tr], X[te], input_len, config)

        r2_a  = r2_score(Y[:, 0], preds[:, 0])
        r2_b  = r2_score(Y[:, 1], preds[:, 1])
        rmse_a = np.sqrt(mean_squared_error(Y[:, 0], preds[:, 0]))
        rmse_b = np.sqrt(mean_squared_error(Y[:, 1], preds[:, 1]))
        mae_a  = mean_absolute_error(Y[:, 0], preds[:, 0])
        mae_b  = mean_absolute_error(Y[:, 1], preds[:, 1])
        total_rmse = rmse_a + rmse_b

        results.append({
            "kernels": str(kernels), "filters": str(filters),
            "lr": lr, "epochs": epochs, "batch_size": bs,
            "dropout_conv": dc, "dropout_fc": df_,
            "r2_a": r2_a, "r2_b": r2_b,
            "rmse_a": rmse_a, "rmse_b": rmse_b,
            "mae_a": mae_a, "mae_b": mae_b,
            "total_rmse": total_rmse,
        })

        elapsed = time.time() - t_start
        avg_per = elapsed / (idx + 1)
        eta = avg_per * (total - idx - 1)

        print(f"  [{idx+1:3d}/{total}] k={str(kernels):>16s} f={str(filters):>14s} "
              f"lr={lr:.0e} ep={epochs:3d} bs={bs:2d} dc={dc:.1f} df={df_:.1f}  |  "
              f"R²_a={r2_a:.4f} R²_b={r2_b:.4f} ΣRMSE={total_rmse:.3f}  "
              f"[ETA: {eta/60:.1f} min]")

    df = pd.DataFrame(results).sort_values("total_rmse")
    csv_path = OUT_DIR / "dual_cnn_gridsearch.csv"
    df.to_csv(csv_path, index=False)
    total_time = time.time() - t_start
    print(f"\n  CNN results saved: {csv_path.name}")
    print(f"  Total time: {total_time/60:.1f} min")
    print(f"  Best: kernels={df.iloc[0]['kernels']}, "
          f"filters={df.iloc[0]['filters']}, "
          f"lr={df.iloc[0]['lr']}, epochs={df.iloc[0]['epochs']:.0f}, "
          f"bs={df.iloc[0]['batch_size']:.0f}, "
          f"dc={df.iloc[0]['dropout_conv']}, df={df.iloc[0]['dropout_fc']}, "
          f"total_RMSE={df.iloc[0]['total_rmse']:.4f}")
    return df


# ═════════════════════════════════════════════════════════════
# SECTION D: COMPARISON PLOT
# ═════════════════════════════════════════════════════════════
def plot_comparison(pls_df, cnn_df):
    """Summary plot: PLS n_components curve + CNN top-15 bar chart."""
    fig = plt.figure(figsize=(18, 7))
    gs = gridspec.GridSpec(1, 2, width_ratios=[1, 1.5])

    # ── Left: PLS n_components curve ──
    ax1 = fig.add_subplot(gs[0, 0])
    for scale_val, marker, label in [(False, 'o', 'Raw'), (True, 's', 'Scaled')]:
        sub = pls_df[pls_df["scale"] == scale_val].sort_values("n_components")
        ax1.plot(sub["n_components"], sub["total_rmse"], marker=marker,
                 label=f"{label}", lw=2, ms=6, alpha=0.8)
        ax1.plot(sub["n_components"], sub["rmse_a"], marker=marker,
                 label=f"{label} (Chl-a)", lw=1, ls='--', ms=4, alpha=0.5)
        ax1.plot(sub["n_components"], sub["rmse_b"], marker=marker,
                 label=f"{label} (Chl-b)", lw=1, ls=':', ms=4, alpha=0.5)

    best = pls_df.iloc[0]
    ax1.axhline(best["total_rmse"], ls='--', color='red', alpha=0.5, lw=1)
    ax1.annotate(f"Best: n={best['n_components']:.0f}, ΣRMSE={best['total_rmse']:.3f}",
                 xy=(best["n_components"], best["total_rmse"]),
                 xytext=(best["n_components"] + 1, best["total_rmse"] + 0.05),
                 fontsize=9, color='red',
                 arrowprops=dict(arrowstyle='->', color='red', lw=1.2))
    ax1.set_xlabel("PLS n_components", fontsize=11)
    ax1.set_ylabel("RMSE (mg/L)", fontsize=11)
    ax1.set_title("PLS Grid Search (Dual-Output, Ideal)", fontsize=12, fontweight='bold')
    ax1.legend(fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)

    # ── Right: CNN top-15 horizontal bar chart ──
    ax2 = fig.add_subplot(gs[0, 1])
    top = cnn_df.head(15).iloc[::-1]  # reverse for bottom-up bar chart
    labels = [f"k={r['kernels']}\nf={r['filters']}\nlr={r['lr']:.0e} ep={r['epochs']:.0f} "
              f"bs={r['batch_size']:.0f}\ndc={r['dropout_conv']} df={r['dropout_fc']}"
              for _, r in top.iterrows()]
    colors = plt.cm.viridis(np.linspace(0.3, 0.9, len(top)))

    bars = ax2.barh(range(len(top)), top["total_rmse"], color=colors, edgecolor='k', lw=0.5)
    ax2.set_yticks(range(len(top)))
    ax2.set_yticklabels(labels, fontsize=6)
    ax2.set_xlabel("Total RMSE (Chl-a + Chl-b) [mg/L]", fontsize=11)
    ax2.set_title("CNN Grid Search — Top 15 (Dual-Output, Ideal)", fontsize=12, fontweight='bold')

    # Add RMSE values on bars
    for bar, val in zip(bars, top["total_rmse"]):
        ax2.text(bar.get_width() + 0.005, bar.get_y() + bar.get_height()/2,
                 f"{val:.3f}", va='center', fontsize=7)
    ax2.grid(True, axis='x', alpha=0.3)

    plt.tight_layout()
    out_path = OUT_DIR / "dual_gridsearch_comparison.png"
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"\n  Comparison plot saved: {out_path.name}")


# ═════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 70)
    print("  DUAL-OUTPUT HYPERPARAMETER GRID SEARCH (IDEAL SIMULATION)")
    print("=" * 70)

    # ── Generate ideal data ──────────────────────────────────
    print(f"\n[1/4] Generating {N_SAMPLES} ideal mixed-solution spectra...")
    _, _, X_concat, Y, Ca, Cb = generate_dataset(N_SAMPLES)
    print(f"  X shape: {X_concat.shape},  Y shape: {Y.shape}")
    print(f"  Chl-a range: {Ca.min():.2f}–{Ca.max():.2f} mg/L")
    print(f"  Chl-b range: {Cb.min():.2f}–{Cb.max():.2f} mg/L")

    # ── PLS Grid Search ──────────────────────────────────────
    print(f"\n[2/4] PLS Grid Search...")
    pls_df = pls_grid_search(X_concat, Y)

    # ── CNN Grid Search ──────────────────────────────────────
    print(f"\n[3/4] CNN Grid Search...")
    cnn_df = cnn_grid_search(X_concat, Y)

    # ── Comparison Plot ──────────────────────────────────────
    print(f"\n[4/4] Generating comparison plot...")
    plot_comparison(pls_df, cnn_df)

    # ── Final Summary ────────────────────────────────────────
    print("\n" + "=" * 70)
    print("  FINAL RESULTS")
    print("=" * 70)

    pls_best = pls_df.iloc[0]
    cnn_best = cnn_df.iloc[0]

    print(f"\n  Best PLS Config:")
    print(f"    n_components = {pls_best['n_components']:.0f}")
    print(f"    scale        = {pls_best['scale']}")
    print(f"    Chl-a: R²={pls_best['r2_a']:.4f}  RMSE={pls_best['rmse_a']:.3f}  MAE={pls_best['mae_a']:.3f}")
    print(f"    Chl-b: R²={pls_best['r2_b']:.4f}  RMSE={pls_best['rmse_b']:.3f}  MAE={pls_best['mae_b']:.3f}")
    print(f"    Total RMSE = {pls_best['total_rmse']:.4f}")

    print(f"\n  Best CNN Config:")
    print(f"    kernels      = {cnn_best['kernels']}")
    print(f"    filters      = {cnn_best['filters']}")
    print(f"    lr           = {cnn_best['lr']}")
    print(f"    epochs       = {cnn_best['epochs']:.0f}")
    print(f"    batch_size   = {cnn_best['batch_size']:.0f}")
    print(f"    dropout_conv = {cnn_best['dropout_conv']}")
    print(f"    dropout_fc   = {cnn_best['dropout_fc']}")
    print(f"    Chl-a: R²={cnn_best['r2_a']:.4f}  RMSE={cnn_best['rmse_a']:.3f}  MAE={cnn_best['mae_a']:.3f}")
    print(f"    Chl-b: R²={cnn_best['r2_b']:.4f}  RMSE={cnn_best['rmse_b']:.3f}  MAE={cnn_best['mae_b']:.3f}")
    print(f"    Total RMSE = {cnn_best['total_rmse']:.4f}")

    print(f"\n  Winner: {'CNN' if cnn_best['total_rmse'] < pls_best['total_rmse'] else 'PLS'}")
    print(f"\nAll results saved to: {OUT_DIR.resolve()}")
