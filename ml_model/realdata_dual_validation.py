#!/usr/bin/env python3
"""
realdata_dual_validation.py — Real Data Validation with Grid-Search-Optimised Hyperparameters
=============================================================================================
Runs the top-5 CNN configs (from the Colab grid search) + best PLS on real
experimental data. Produces a single multi-panel comparison chart.

Usage:
    python realdata_dual_validation.py

Outputs (saved to training_plots/):
    - realdata_dual_validation_all.png  (6-panel comparison: PLS + top-5 CNN)
    - realdata_dual_summary.csv         (metrics for all configs)
"""

import time
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import KFold, RepeatedKFold
from sklearn.metrics import r2_score, mean_squared_error, mean_absolute_error
import warnings
warnings.filterwarnings("ignore")

import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset

np.random.seed(42)
torch.manual_seed(42)

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"  Device: {DEVICE}")

# ─────────────────────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
DATA_DIR_A = SCRIPT_DIR / "data" / "chl_a"
DATA_DIR_B = SCRIPT_DIR / "data" / "chl_b"
BG_DIR     = SCRIPT_DIR / "data" / "background_data"
CSV_A      = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B      = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"
OUT_DIR    = SCRIPT_DIR / "training_plots"
OUT_DIR.mkdir(exist_ok=True)
IMG_DIR    = SCRIPT_DIR.parent / "kenura_lab_report" / "images"
IMG_DIR.mkdir(exist_ok=True)

ROI_START = 1300
ROI_END   = 3500
ROI_LEN   = ROI_END - ROI_START
SG_WIN, SG_POLY = 11, 2
SG_DWIN, SG_DPOLY = 11, 3

# ─────────────────────────────────────────────────────────────
# GRID SEARCH WINNERS (from Colab run)
# ─────────────────────────────────────────────────────────────
BEST_PLS = {"n_components": 15, "label": "PLS (n=15)"}

TOP_CNN_CONFIGS = [
    {"kernels": (51,25,11), "filters": (16,32,64), "lr": 3e-3, "epochs": 80,
     "batch_size": 16, "dropout_conv": 0.1, "dropout_fc": 0.2,
     "label": "CNN #1: k(51,25,11) lr=3e-3"},
    {"kernels": (31,15,7), "filters": (16,32,64), "lr": 3e-3, "epochs": 120,
     "batch_size": 16, "dropout_conv": 0.1, "dropout_fc": 0.2,
     "label": "CNN #2: k(31,15,7) lr=3e-3 ep120"},
    {"kernels": (31,15,7), "filters": (16,32,64), "lr": 3e-3, "epochs": 80,
     "batch_size": 16, "dropout_conv": 0.1, "dropout_fc": 0.2,
     "label": "CNN #3: k(31,15,7) lr=3e-3 ep80"},
    {"kernels": (41,21,11), "filters": (32,64,128), "lr": 1e-3, "epochs": 120,
     "batch_size": 16, "dropout_conv": 0.1, "dropout_fc": 0.2,
     "label": "CNN #4: k(41,21,11) f(32,64,128)"},
    {"kernels": (21,11,5), "filters": (32,64,128), "lr": 1e-3, "epochs": 120,
     "batch_size": 16, "dropout_conv": 0.2, "dropout_fc": 0.2,
     "label": "CNN #5: k(21,11,5) f(32,64,128)"},
]

# ─────────────────────────────────────────────────────────────
# DATA LOADING (from train_combined_model.py)
# ─────────────────────────────────────────────────────────────
BG_MAP = {
    240: "background-240.csv", 250: "background-250.csv",
    300: "background-300.csv", 500: "background-500.csv",
    1000: "background-1000.csv",
}
BG_AVAIL = sorted(BG_MAP.keys())


def load_spectrum(path):
    df = pd.read_csv(path)
    return np.mean(df.iloc[:, 1:].values.astype(float), axis=0)


def load_bg(int_time):
    nearest = min(BG_AVAIL, key=lambda t: abs(t - int_time))
    path = BG_DIR / BG_MAP[nearest]
    return load_spectrum(path) if path.exists() else None


def als_baseline(y, lam=1e5, p=0.01, niter=10):
    L = len(y)
    D = sparse.diags([1, -2, 1], [0, 1, 2], shape=(L-2, L), format='csc')
    D = D.T @ D
    w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w, 0, format='csc')
        z = spsolve(W + lam * D, w * y)
        w = p * (y > z) + (1 - p) * (y <= z)
    return z


def preprocess(raw, bg, int_time, use_als=False):
    if bg is not None and len(bg) == len(raw):
        raw = raw - bg
    roi = raw[ROI_START:ROI_END]
    if use_als:
        try:
            roi = roi - als_baseline(roi)
        except Exception:
            pass
    roi = roi / int_time
    sm  = savgol_filter(roi, SG_WIN, SG_POLY)
    drv = savgol_filter(sm, SG_DWIN, SG_DPOLY, deriv=1)
    mu, s = drv.mean(), drv.std()
    return (drv - mu) / (s + 1e-12)


def load_dataset(csv_path, data_dir, use_als=False):
    meta = pd.read_csv(csv_path).dropna(subset=["Sample Con", "integration time(mS)"])
    spectra, row_indices = [], []
    for i, row in meta.iterrows():
        fpath = data_dir / f"{i+1}.csv"
        if not fpath.exists():
            continue
        raw = load_spectrum(fpath)
        if len(raw) < ROI_END:
            continue
        bg = load_bg(int(row["integration time(mS)"]))
        feat = preprocess(raw, bg, row["integration time(mS)"], use_als)
        spectra.append(feat)
        row_indices.append(i)
    return np.array(spectra), row_indices


# ─────────────────────────────────────────────────────────────
# PLS EVALUATION (Repeated CV with error bars)
# ─────────────────────────────────────────────────────────────
def evaluate_pls_repeated(X, Y, n_components, n_repeats=3, n_splits=20):
    """Repeated K-fold CV → mean predictions, error bars, metrics."""
    all_preds = np.zeros((n_repeats, len(Y), 2))

    for rep in range(n_repeats):
        kf = KFold(n_splits=n_splits, shuffle=True, random_state=42 + rep)
        fold_preds = np.zeros_like(Y)
        for train_idx, test_idx in kf.split(X):
            m = PLSRegression(n_components=n_components)
            m.fit(X[train_idx], Y[train_idx])
            fold_preds[test_idx] = m.predict(X[test_idx])
        all_preds[rep] = fold_preds

    mean_preds = all_preds.mean(axis=0)
    std_preds  = all_preds.std(axis=0)

    # Metrics per repeat
    metrics = {}
    for key, col in [("a", 0), ("b", 1)]:
        metrics[f"r2_{key}"]   = [r2_score(Y[:, col], all_preds[r, :, col]) for r in range(n_repeats)]
        metrics[f"rmse_{key}"] = [np.sqrt(mean_squared_error(Y[:, col], all_preds[r, :, col])) for r in range(n_repeats)]
        metrics[f"mae_{key}"]  = [mean_absolute_error(Y[:, col], all_preds[r, :, col]) for r in range(n_repeats)]

    return mean_preds, std_preds, metrics


# ─────────────────────────────────────────────────────────────
# CNN ARCHITECTURE
# ─────────────────────────────────────────────────────────────
class DualCNN1D(nn.Module):
    def __init__(self, input_len, kernel_sizes=(51,25,11), filters=(16,32,64),
                 dropout_conv=0.1, dropout_fc=0.2):
        super().__init__()
        k1, k2, k3 = kernel_sizes
        f1, f2, f3 = filters
        self.features = nn.Sequential(
            nn.Conv1d(1, f1, kernel_size=k1, padding=k1//2),
            nn.BatchNorm1d(f1), nn.ReLU(), nn.MaxPool1d(4),
            nn.Dropout(dropout_conv),
            nn.Conv1d(f1, f2, kernel_size=k2, padding=k2//2),
            nn.BatchNorm1d(f2), nn.ReLU(), nn.MaxPool1d(4),
            nn.Dropout(dropout_conv),
            nn.Conv1d(f2, f3, kernel_size=k3, padding=k3//2),
            nn.BatchNorm1d(f3), nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(f3*8, 128), nn.ReLU(), nn.Dropout(dropout_fc),
            nn.Linear(128, 32), nn.ReLU(),
            nn.Linear(32, 2),
        )

    def forward(self, x):
        return self.regressor(self.features(x))


def train_cnn_fold(X_tr, Y_tr, X_te, config):
    model = DualCNN1D(
        X_tr.shape[1],
        kernel_sizes=config["kernels"], filters=config["filters"],
        dropout_conv=config["dropout_conv"], dropout_fc=config["dropout_fc"],
    ).to(DEVICE)

    opt = torch.optim.Adam(model.parameters(), lr=config["lr"], weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=config["epochs"])
    loss_fn = nn.MSELoss()

    Xt = torch.FloatTensor(X_tr).unsqueeze(1).to(DEVICE)
    Yt = torch.FloatTensor(Y_tr).to(DEVICE)
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
        preds = model(torch.FloatTensor(X_te).unsqueeze(1).to(DEVICE)).cpu().numpy()
    return preds


def evaluate_cnn_repeated(X, Y, config, n_repeats=3, n_splits=5):
    """Repeated K-fold CV for CNN → mean preds, std, metrics."""
    all_preds = np.zeros((n_repeats, len(Y), 2))

    for rep in range(n_repeats):
        kf = KFold(n_splits=n_splits, shuffle=True, random_state=42 + rep)
        fold_preds = np.zeros_like(Y)
        for fold, (tr, te) in enumerate(kf.split(X)):
            fold_preds[te] = train_cnn_fold(X[tr], Y[tr], X[te], config)
        all_preds[rep] = fold_preds
        print(f"    Rep {rep+1}/{n_repeats} done — "
              f"R²_a={r2_score(Y[:, 0], fold_preds[:, 0]):.4f}  "
              f"R²_b={r2_score(Y[:, 1], fold_preds[:, 1]):.4f}")

    mean_preds = all_preds.mean(axis=0)
    std_preds  = all_preds.std(axis=0)

    metrics = {}
    for key, col in [("a", 0), ("b", 1)]:
        metrics[f"r2_{key}"]   = [r2_score(Y[:, col], all_preds[r, :, col]) for r in range(n_repeats)]
        metrics[f"rmse_{key}"] = [np.sqrt(mean_squared_error(Y[:, col], all_preds[r, :, col])) for r in range(n_repeats)]
        metrics[f"mae_{key}"]  = [mean_absolute_error(Y[:, col], all_preds[r, :, col]) for r in range(n_repeats)]

    return mean_preds, std_preds, metrics


# ═════════════════════════════════════════════════════════════
# COMBINED COMPARISON CHART
# ═════════════════════════════════════════════════════════════
def plot_all_results(results_list, Y, out_path):
    """
    Single multi-panel figure: one row per model (PLS + 5 CNNs),
    with Chl-a (left) and Chl-b (right) scatter + error bars.
    """
    n_models = len(results_list)
    fig, axes = plt.subplots(n_models, 2, figsize=(14, 4.5 * n_models))
    if n_models == 1:
        axes = axes.reshape(1, -1)

    for row, res in enumerate(results_list):
        label = res["label"]
        mean_p = res["mean_preds"]
        std_p  = res["std_preds"]
        metrics = res["metrics"]
        color = res.get("color", "royalblue")

        for col, (name, key) in enumerate([("Chl-a", "a"), ("Chl-b", "b")]):
            ax = axes[row, col]
            y_true = Y[:, col]
            y_pred = mean_p[:, col]
            y_std  = std_p[:, col]

            r2   = np.mean(metrics[f"r2_{key}"])
            rmse = np.mean(metrics[f"rmse_{key}"])
            mae  = np.mean(metrics[f"mae_{key}"])

            r2_ci   = [np.min(metrics[f"r2_{key}"]),   np.max(metrics[f"r2_{key}"])]
            rmse_ci = [np.min(metrics[f"rmse_{key}"]), np.max(metrics[f"rmse_{key}"])]
            mae_ci  = [np.min(metrics[f"mae_{key}"]),  np.max(metrics[f"mae_{key}"])]

            lim_min = min(y_true.min(), y_pred.min()) * 0.9
            lim_max = max(y_true.max(), y_pred.max()) * 1.08

            # Ideal line
            ax.plot([lim_min, lim_max], [lim_min, lim_max], 'k--', lw=1, alpha=0.5)

            # Error bars + data points
            ax.errorbar(y_true, y_pred, yerr=y_std, fmt='o', color=color,
                        mfc=color, mec='k', ms=5, alpha=0.7, capsize=2,
                        elinewidth=0.8, label=label)

            # Metric box
            box_text = (f"R²   = {r2:.3f} [{r2_ci[0]:.3f}, {r2_ci[1]:.3f}]\n"
                        f"RMSE = {rmse:.3f} [{rmse_ci[0]:.3f}, {rmse_ci[1]:.3f}]\n"
                        f"MAE  = {mae:.3f} [{mae_ci[0]:.3f}, {mae_ci[1]:.3f}]")
            ax.text(0.03, 0.97, box_text, transform=ax.transAxes, fontsize=8,
                    va='top', fontfamily='monospace',
                    bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.85))

            ax.set_xlim(lim_min, lim_max)
            ax.set_ylim(lim_min, lim_max)
            ax.set_xlabel(f"Actual [{name}] mg/L", fontsize=9)
            ax.set_ylabel(f"Predicted [{name}] mg/L", fontsize=9)
            ax.set_title(f"{label} — {name}", fontsize=10, fontweight='bold')
            ax.grid(True, alpha=0.2)
            ax.set_aspect('equal', adjustable='box')

    fig.suptitle("Real Data Validation: PLS vs Top-5 CNN Configs\n"
                 "(Grid-Search-Optimised on Ideal Simulation → Applied to Real Experimental Data)",
                 fontsize=13, fontweight='bold', y=1.01)
    plt.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"\n  ✓ Combined chart saved: {out_path.name}")


def plot_summary_bars(summary_df, out_path):
    """Horizontal bar chart comparing all models on total RMSE."""
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))

    df = summary_df.sort_values("total_rmse")
    labels = df["model"].values
    colors = ['#2196F3' if 'PLS' in l else '#FF9800' for l in labels]

    for ax, metric, title in zip(axes,
                                  ["total_rmse", "mean_r2_a", "mean_r2_b"],
                                  ["Total RMSE (↓ better)", "Mean R² Chl-a (↑ better)", "Mean R² Chl-b (↑ better)"]):
        bars = ax.barh(range(len(df)), df[metric], color=colors, edgecolor='k', lw=0.5)
        ax.set_yticks(range(len(df)))
        ax.set_yticklabels(labels, fontsize=8)
        ax.set_xlabel(metric.replace("_", " ").title(), fontsize=10)
        ax.set_title(title, fontsize=11, fontweight='bold')
        for bar, val in zip(bars, df[metric]):
            ax.text(bar.get_width() + 0.01, bar.get_y() + bar.get_height()/2,
                    f"{val:.3f}", va='center', fontsize=8)
        ax.grid(True, axis='x', alpha=0.3)

    plt.tight_layout()
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"  ✓ Summary bars saved: {out_path.name}")


# ═════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 70)
    print("  REAL DATA VALIDATION — Grid-Search-Optimised Models")
    print("=" * 70)

    # ── Load real data ───────────────────────────────────────
    print("\n[1/3] Loading real experimental data...")
    X_a, idx_a = load_dataset(CSV_A, DATA_DIR_A, use_als=False)
    X_b, idx_b = load_dataset(CSV_B, DATA_DIR_B, use_als=True)
    print(f"  Chl-a spectra: {X_a.shape}")
    print(f"  Chl-b spectra: {X_b.shape}")

    meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])
    meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])

    min_n = min(len(X_a), len(X_b))
    X = np.hstack([X_a[:min_n], X_b[:min_n]])
    Y = np.column_stack([
        meta_a["Sample Con"].values[:min_n],
        meta_b["Sample Con"].values[:min_n],
    ])

    print(f"  Combined X: {X.shape}, Y: {Y.shape}")
    print(f"  Chl-a: {Y[:, 0].min():.2f}–{Y[:, 0].max():.2f} mg/L")
    print(f"  Chl-b: {Y[:, 1].min():.2f}–{Y[:, 1].max():.2f} mg/L")

    results_list = []
    summary_rows = []

    # ── PLS (best from grid search) ──────────────────────────
    print(f"\n[2/3] Evaluating PLS (n_components={BEST_PLS['n_components']})...")
    t0 = time.time()
    pls_mean, pls_std, pls_metrics = evaluate_pls_repeated(X, Y, BEST_PLS["n_components"])
    pls_time = time.time() - t0

    r2a  = np.mean(pls_metrics["r2_a"])
    r2b  = np.mean(pls_metrics["r2_b"])
    rmsea = np.mean(pls_metrics["rmse_a"])
    rmseb = np.mean(pls_metrics["rmse_b"])
    maea  = np.mean(pls_metrics["mae_a"])
    maeb  = np.mean(pls_metrics["mae_b"])

    print(f"  PLS: Chl-a R²={r2a:.4f} RMSE={rmsea:.3f}  |  "
          f"Chl-b R²={r2b:.4f} RMSE={rmseb:.3f}  [{pls_time:.1f}s]")

    results_list.append({
        "label": BEST_PLS["label"], "mean_preds": pls_mean,
        "std_preds": pls_std, "metrics": pls_metrics, "color": "#2196F3",
    })
    summary_rows.append({
        "model": BEST_PLS["label"],
        "mean_r2_a": r2a, "mean_r2_b": r2b,
        "mean_rmse_a": rmsea, "mean_rmse_b": rmseb,
        "mean_mae_a": maea, "mean_mae_b": maeb,
        "total_rmse": rmsea + rmseb,
        "time_s": pls_time,
    })

    # ── Top-5 CNN configs ────────────────────────────────────
    print(f"\n[3/3] Evaluating top-5 CNN configs on real data...")
    cnn_colors = ["#FF9800", "#E91E63", "#4CAF50", "#9C27B0", "#795548"]

    for i, config in enumerate(TOP_CNN_CONFIGS):
        print(f"\n  ── CNN #{i+1}: {config['label']} ──")
        t0 = time.time()
        cnn_mean, cnn_std, cnn_metrics = evaluate_cnn_repeated(
            X, Y, config, n_repeats=3, n_splits=5)
        cnn_time = time.time() - t0

        r2a  = np.mean(cnn_metrics["r2_a"])
        r2b  = np.mean(cnn_metrics["r2_b"])
        rmsea = np.mean(cnn_metrics["rmse_a"])
        rmseb = np.mean(cnn_metrics["rmse_b"])
        maea  = np.mean(cnn_metrics["mae_a"])
        maeb  = np.mean(cnn_metrics["mae_b"])

        print(f"  CNN #{i+1}: Chl-a R²={r2a:.4f} RMSE={rmsea:.3f}  |  "
              f"Chl-b R²={r2b:.4f} RMSE={rmseb:.3f}  [{cnn_time:.1f}s]")

        results_list.append({
            "label": config["label"], "mean_preds": cnn_mean,
            "std_preds": cnn_std, "metrics": cnn_metrics, "color": cnn_colors[i],
        })
        summary_rows.append({
            "model": config["label"],
            "mean_r2_a": r2a, "mean_r2_b": r2b,
            "mean_rmse_a": rmsea, "mean_rmse_b": rmseb,
            "mean_mae_a": maea, "mean_mae_b": maeb,
            "total_rmse": rmsea + rmseb,
            "time_s": cnn_time,
        })

    # ── Generate charts ──────────────────────────────────────
    print("\n  Generating comparison charts...")
    chart_path = OUT_DIR / "realdata_dual_validation_all.png"
    plot_all_results(results_list, Y, chart_path)

    # Copy to report images
    import shutil
    shutil.copy(chart_path, IMG_DIR / chart_path.name)
    print(f"  ✓ Copied to report images: {chart_path.name}")

    # Summary bar chart
    summary_df = pd.DataFrame(summary_rows)
    bars_path = OUT_DIR / "realdata_dual_summary_bars.png"
    plot_summary_bars(summary_df, bars_path)
    shutil.copy(bars_path, IMG_DIR / bars_path.name)

    # Save CSV
    csv_path = OUT_DIR / "realdata_dual_summary.csv"
    summary_df.to_csv(csv_path, index=False)
    print(f"  ✓ Summary CSV saved: {csv_path.name}")

    # ── Final Summary ────────────────────────────────────────
    print("\n" + "=" * 70)
    print("  REAL DATA VALIDATION RESULTS")
    print("=" * 70)
    summary_df = summary_df.sort_values("total_rmse")
    print(f"\n  {'Model':<38}  {'R²_a':>6}  {'R²_b':>6}  {'RMSE_a':>7}  {'RMSE_b':>7}  {'ΣRMSE':>7}  {'Time':>6}")
    print(f"  {'-'*85}")
    for _, r in summary_df.iterrows():
        marker = " 🏆" if r.name == summary_df.index[0] else ""
        print(f"  {r['model']:<38}  {r['mean_r2_a']:>6.4f}  {r['mean_r2_b']:>6.4f}  "
              f"{r['mean_rmse_a']:>7.3f}  {r['mean_rmse_b']:>7.3f}  "
              f"{r['total_rmse']:>7.3f}  {r['time_s']:>5.1f}s{marker}")

    winner = summary_df.iloc[0]
    print(f"\n  🏆 Winner on real data: {winner['model']}")
    print(f"\nAll outputs saved to: {OUT_DIR.resolve()}")
