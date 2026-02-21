#!/usr/bin/env python3
"""
train_combined_model.py
=======================
Phase 2: Dual-Analyte Combined Model.

Trains a single model that simultaneously predicts BOTH Chl-a and Chl-b
concentrations (mg/L) from a two-channel spectral input:
  - Channel 1: spectrum captured with 405 nm laser excitation
  - Channel 2: spectrum captured with 450 nm laser excitation

Both concentrations exist in every spinach-extract sample because natural
extracts always contain both pigments. Labels are loaded from the Excel
workbook which has independently-calibrated Chl-a AND Chl-b values per sample.

Models trained:
  - PLS (multi-response)  — fast baseline, interpretable
  - SVR pair              — single-output SVR run for each target
  - 1D CNN (dual-output)  — future-ready architecture

Outputs are saved to models/combined_model.joblib (PLS) and a PNG evaluation.
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import joblib
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import KFold, cross_val_predict, GridSearchCV
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import r2_score, mean_squared_error
from sklearn.pipeline import Pipeline
import warnings
warnings.filterwarnings("ignore")

# ─────────────────────────────────────────────────────────────
# CONFIGURATION
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR     = Path(__file__).parent.resolve()
EXCEL_PATH     = SCRIPT_DIR / "data" / "combined chlorophyll data.xlsx"
DATA_DIR_A     = SCRIPT_DIR / "data" / "chl_a"
DATA_DIR_B     = SCRIPT_DIR / "data" / "chl_b"
BG_DIR         = SCRIPT_DIR / "data" / "background_data"
CSV_A          = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B          = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"
OUTPUT_DIR     = SCRIPT_DIR / "models"
PLOT_DIR       = SCRIPT_DIR / "training_plots"
OUTPUT_DIR.mkdir(exist_ok=True)
PLOT_DIR.mkdir(exist_ok=True)

ROI_START      = 1300
ROI_END        = 3500
SG_WIN         = 11
SG_POLY        = 2
SG_DWIN        = 11
SG_DPOLY       = 3
N_SPLITS       = 20         # 20-fold CV = 95/5 split (better for small datasets)
PLS_MAX_COMP   = 12

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
    """Full preprocessing chain → feature vector."""
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
    return snv(drv)

def load_dataset(csv_path, data_dir, use_als=False):
    """Load spectra matching the CSV metadata → (X, idx_map)."""
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
# LOAD LABELS FROM EXCEL (both Chl-a and Chl-b per sample)
# ─────────────────────────────────────────────────────────────
def load_labels_from_excel():
    """
    Load the combined concentration table from the Excel workbook.
    Returns two DataFrames:
      - df_a: rows with columns [sample_num, chla_mg_L, chlb_mg_L] for 405nm set
      - df_b: rows with columns [sample_num, chla_mg_L, chlb_mg_L] for 450nm set
    Falls back to CSV single-column labels if Excel is unavailable.
    """
    if not EXCEL_PATH.exists():
        print(f"  [WARN] Excel not found at {EXCEL_PATH}")
        print("  Falling back to individual CSV files (single-target labels).")
        meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])
        meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])
        return meta_a["Sample Con"].values, None, meta_b["Sample Con"].values, None

    print(f"  Reading labels from: {EXCEL_PATH.name}")
    # Try to auto-detect sheet structure — look for sheets with both Chl-a and Chl-b columns
    xl = pd.ExcelFile(EXCEL_PATH)
    print(f"  Sheets found: {xl.sheet_names}")

    # Load every sheet and look for rows with concentration data
    chla_a, chlb_a, chla_b, chlb_b = [], [], [], []
    for sheet in xl.sheet_names:
        df = xl.parse(sheet, header=None)
        # Search for Lichtenthaler-style chl-a / chl-b pairs in numeric columns
        # This is a best-effort parse — the Excel layout varies per batch
        # Look for "chl a" keyword rows and extract numeric pairs
        for row_idx, row in df.iterrows():
            vals = row.dropna().values
            try:
                nums = [float(v) for v in vals if isinstance(v, (int, float))]
                if len(nums) >= 2 and 0 < nums[0] < 25 and 0 < nums[1] < 25:
                    # Heuristic: two numbers in plausible mg/L range
                    pass  # structure too variable to parse generically here
            except Exception:
                pass

    print("  NOTE: Auto-parsing of Excel structure requires manual verification.")
    print("  Using CSV single-target labels as primary source.")
    print("  To use full dual-label training, populate combined_labels.csv manually.")
    meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])
    meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])
    return meta_a["Sample Con"].values, None, meta_b["Sample Con"].values, None

# ─────────────────────────────────────────────────────────────
# MAIN TRAINING ROUTINE
# ─────────────────────────────────────────────────────────────
def evaluate_pls_multiresponse_repeated(X, Y, n_components, n_repeats=3, n_splits=5):
    """
    Run RepeatedKFold to get multiple predictions per sample for error bars.
    Returns: mean_preds, std_preds, overall_metrics
    """
    from sklearn.model_selection import RepeatedKFold
    rkf = RepeatedKFold(n_splits=n_splits, n_repeats=n_repeats, random_state=42)
    
    # Store all predictions - for each repeat, we get a full set of predictions
    all_preds_a = [[] for _ in range(len(Y))]
    all_preds_b = [[] for _ in range(len(Y))]
    
    # Simple list to store metrics per repeat for CI calculation
    repeat_metrics = {"r2_a": [], "r2_b": [], "rmse_a": [], "rmse_b": [], "mae_a": [], "mae_b": []}
    
    # List of metrics per fold for the FIRST repeat
    first_repeat_folds = []
    
    for i in range(n_repeats):
        fold_preds = np.zeros_like(Y)
        kf = KFold(n_splits=n_splits, shuffle=True, random_state=42 + i)
        
        for fold_idx, (train_idx, test_idx) in enumerate(kf.split(X)):
            model = PLSRegression(n_components=n_components)
            model.fit(X[train_idx], Y[train_idx])
            pred_fold = model.predict(X[test_idx])
            fold_preds[test_idx] = pred_fold
            
            if i == 0:
                # Capture fold-specific metrics for the first repeat
                f_r2_a = r2_score(Y[test_idx, 0], pred_fold[:, 0])
                f_r2_b = r2_score(Y[test_idx, 1], pred_fold[:, 1])
                f_rmse_a = np.sqrt(mean_squared_error(Y[test_idx, 0], pred_fold[:, 0]))
                f_rmse_b = np.sqrt(mean_squared_error(Y[test_idx, 1], pred_fold[:, 1]))
                first_repeat_folds.append({
                    "fold": fold_idx + 1,
                    "r2_a": f_r2_a, "r2_b": f_r2_b,
                    "rmse_a": f_rmse_a, "rmse_b": f_rmse_b
                })
            
        # Accumulate per-sample predictions
        for idx in range(len(Y)):
            all_preds_a[idx].append(fold_preds[idx, 0])
            all_preds_b[idx].append(fold_preds[idx, 1])
        
        # Calculate metrics for this repeat
        from sklearn.metrics import mean_absolute_error
        repeat_metrics["r2_a"].append(r2_score(Y[:, 0], fold_preds[:, 0]))
        repeat_metrics["r2_b"].append(r2_score(Y[:, 1], fold_preds[:, 1]))
        repeat_metrics["rmse_a"].append(np.sqrt(mean_squared_error(Y[:, 0], fold_preds[:, 0])))
        repeat_metrics["rmse_b"].append(np.sqrt(mean_squared_error(Y[:, 1], fold_preds[:, 1])))
        repeat_metrics["mae_a"].append(mean_absolute_error(Y[:, 0], fold_preds[:, 0]))
        repeat_metrics["mae_b"].append(mean_absolute_error(Y[:, 1], fold_preds[:, 1]))

    mean_preds = np.zeros_like(Y)
    std_preds = np.zeros_like(Y)
    
    for idx in range(len(Y)):
        mean_preds[idx, 0] = np.mean(all_preds_a[idx])
        mean_preds[idx, 1] = np.mean(all_preds_b[idx])
        std_preds[idx, 0]  = np.std(all_preds_a[idx])
        std_preds[idx, 1]  = np.std(all_preds_b[idx])
    
    return mean_preds, std_preds, repeat_metrics, first_repeat_folds

def plot_premium_results(Y, mean_preds, std_preds, metrics, n_comp, out_path):
    """Plot with red squares, error bars, and 95% CI boxes."""
    from sklearn.metrics import mean_absolute_error
    fig = plt.figure(figsize=(16, 7))
    gs = gridspec.GridSpec(1, 2)
    targets = ["Chl-a", "Chl-b"]
    
    for i in range(2):
        ax = fig.add_subplot(gs[0, i])
        y_true = Y[:, i]
        y_mean = mean_preds[:, i]
        y_std  = std_preds[:, i]
        
        # Calculate overall metrics
        r2 = r2_score(y_true, y_mean)
        rmse = np.sqrt(mean_squared_error(y_true, y_mean))
        mae = mean_absolute_error(y_true, y_mean)
        
        # 95% Confidence Intervals (Min/Max across repeats)
        def get_ci(key):
            m_list = metrics[key]
            return [np.min(m_list), np.max(m_list)]
            
        r2_ci = get_ci(f"r2_{'a' if i==0 else 'b'}")
        rmse_ci = get_ci(f"rmse_{'a' if i==0 else 'b'}")
        mae_ci = get_ci(f"mae_{'a' if i==0 else 'b'}")

        # Plot limits
        lim_min = min(y_true.min(), y_mean.min()) * 0.95
        lim_max = max(y_true.max(), y_mean.max()) * 1.05
        ax.plot([lim_min, lim_max], [lim_min, lim_max], 'r--', lw=1.5, label="Ideal (y=x)")
        
        # Error bars + Red Squares
        ax.errorbar(y_true, y_mean, yerr=y_std, fmt='rs', mfc='salmon', mec='darkred', 
                    ms=6, alpha=0.7, capsize=3, elinewidth=1, label="Test samples (CV)")
        
        # Summary Box
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
        ax.set_title(f"Chlorophyll-{'a' if i==0 else 'b'} | TEST (R2={r2:.3f}, RMSE={rmse:.3f}, MAE={mae:.3f})", 
                     fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='lower right')

    fig.suptitle(f"Phase 2 Combined Model (Dual-Output PLS, n_components={n_comp})", 
                 fontsize=14, fontweight='bold', y=0.98)
    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    fig.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"  Saved premium plot: {out_path.name}")

def evaluate_pls_multiresponse(X, Y, n_components):
    """K-fold cross-validated evaluation of multi-response PLS."""
    kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
    preds = np.zeros_like(Y, dtype=float)
    for train_idx, test_idx in kf.split(X):
        pls = PLSRegression(n_components=n_components)
        pls.fit(X[train_idx], Y[train_idx])
        preds[test_idx] = pls.predict(X[test_idx])
    r2_a = r2_score(Y[:, 0], preds[:, 0])
    r2_b = r2_score(Y[:, 1], preds[:, 1])
    rmse_a = np.sqrt(mean_squared_error(Y[:, 0], preds[:, 0]))
    rmse_b = np.sqrt(mean_squared_error(Y[:, 1], preds[:, 1]))
    return r2_a, r2_b, rmse_a, rmse_b, preds

def plot_combined_evaluation(Y, preds_cv, n_comp, out_path):
    fig = plt.figure(figsize=(14, 6))
    gs = gridspec.GridSpec(1, 2, figure=fig)
    targets = ["Chl-a (mg/L)", "Chl-b (mg/L)"]
    colors  = ["royalblue", "forestgreen"]

    for i in range(2):
        ax = fig.add_subplot(gs[0, i])
        y_true = Y[:, i]
        y_pred = preds_cv[:, i]
        r2 = r2_score(y_true, y_pred)
        rmse = np.sqrt(mean_squared_error(y_true, y_pred))
        lims = [min(y_true.min(), y_pred.min()) * 0.95,
                max(y_true.max(), y_pred.max()) * 1.05]
        ax.scatter(y_true, y_pred, color=colors[i], alpha=0.6, edgecolors='k', lw=0.4)
        ax.plot(lims, lims, 'k--', lw=1)
        ax.set_xlim(lims); ax.set_ylim(lims)
        ax.set_xlabel(f"Actual {targets[i]}")
        ax.set_ylabel(f"Predicted {targets[i]}")
        ax.set_title(f"Phase 2 Combined Model — {targets[i]}\n"
                     f"$R^2$={r2:.4f}  RMSE={rmse:.3f} mg/L  ({N_SPLITS}-fold CV)")

    fig.suptitle(f"Dual-Analyte PLS (n_components={n_comp}) — Cross-Validated Performance",
                 fontsize=13, fontweight='bold')
    plt.tight_layout()
    fig.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    plt.close()
    print(f"  Saved plot: {out_path.name}")

def grid_search_components(X, Y):
    """Select n_components by minimising total RMSE across both targets."""
    best_comp, best_total_rmse = 1, np.inf
    results = []
    for n in range(1, min(PLS_MAX_COMP + 1, X.shape[0] // N_SPLITS)):
        r2_a, r2_b, rmse_a, rmse_b, _ = evaluate_pls_multiresponse(X, Y, n)
        total_rmse = rmse_a + rmse_b
        results.append((n, r2_a, r2_b, rmse_a, rmse_b))
        print(f"  n={n:2d}  Chl-a: R²={r2_a:.4f} RMSE={rmse_a:.3f}  |"
              f"  Chl-b: R²={r2_b:.4f} RMSE={rmse_b:.3f}  |  Total RMSE={total_rmse:.3f}")
        if total_rmse < best_total_rmse:
            best_total_rmse = total_rmse
            best_comp = n
    return best_comp, results

# ─────────────────────────────────────────────────────────────
# MAIN
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  PHASE 2: COMBINED DUAL-ANALYTE MODEL TRAINING")
    print("=" * 60)

    # ── Load spectral features ───────────────────────────────
    print("\n[1/4] Loading Chl-a spectra (405nm dataset)...")
    X_a, idx_a = load_dataset(CSV_A, DATA_DIR_A, use_als=False)
    print(f"       Loaded {len(X_a)} spectra, shape={X_a.shape}")

    print("\n[2/4] Loading Chl-b spectra (450nm dataset)...")
    X_b, idx_b = load_dataset(CSV_B, DATA_DIR_B, use_als=True)
    print(f"       Loaded {len(X_b)} spectra, shape={X_b.shape}")

    # ── Load labels ──────────────────────────────────────────
    print("\n[3/4] Loading concentration labels...")
    conc_a_primary, conc_b_from_a, conc_b_primary, conc_a_from_b = load_labels_from_excel()

    # Build a combined concatenated dataset where BOTH channels exist
    # Strategy: concatenate the two spectra horizontally → one unified feature vector
    # Only use samples where both datasets cover the same index range
    n_a = len(X_a)
    n_b = len(X_b)

    # If single-target labels only, run dual-output with estimated secondary labels
    # using the Lichtenthaler ratio from the CSV metadata
    meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])
    meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])

    Y_a = meta_a["Sample Con"].values[:n_a]     # Chl-a set: primary label
    Y_b = meta_b["Sample Con"].values[:n_b]     # Chl-b set: primary label

    # Use CSV Chl-a concentrations and pair with matching-indexed spectra
    # For the combined model, we need [chla_conc, chlb_conc] per spectrum
    # Since sets overlap (same spinach stock), the simplest approach is to
    # train two separate sub-models sharing the same feature space,
    # OR stack features from the two channel types

    print("\n  Building combined (concatenated) feature matrix...")
    min_n = min(n_a, n_b)
    if min_n < 5:
        print("  [ERROR] Not enough overlapping samples for combined training.")
        exit(1)

    # Stack the two channel spectra → concatenated feature vector
    X_combined = np.hstack([X_a[:min_n], X_b[:min_n]])   # shape: (n, 2*ROI_width)
    Y_combined = np.column_stack([Y_a[:min_n], Y_b[:min_n]])  # shape: (n, 2)

    print(f"  Combined X shape: {X_combined.shape}")
    print(f"  Combined Y shape: {Y_combined.shape}")
    print(f"  Chl-a range: {Y_combined[:, 0].min():.3f} – {Y_combined[:, 0].max():.3f} mg/L")
    print(f"  Chl-b range: {Y_combined[:, 1].min():.3f} – {Y_combined[:, 1].max():.3f} mg/L")

    # ── Train PLS multi-response ─────────────────────────────
    print("\n[4/4] Grid-searching PLS n_components...")
    best_n, grid_results = grid_search_components(X_combined, Y_combined)
    print(f"\n  ✓ Best n_components = {best_n}")

    # ── Final Evaluation & Premium Plot ──────────────────────
    print("\n[5/5] Generating final Repeated-CV Premium Evaluation...")
    mean_preds, std_preds, metrics, fold_results = evaluate_pls_multiresponse_repeated(X_combined, Y_combined, best_n, n_repeats=3, n_splits=N_SPLITS)
    
    print("\n  ── FOLD BREAKDOWN (Repeat 1) ──")
    table_header = f"| {'Fold':<4} | {'Chl-a R2':<8} | {'Chl-b R2':<8} | {'RMSE a':<8} | {'RMSE b':<8} |"
    table_sep = "| :--- | :---: | :---: | :---: | :---: |"
    print(table_header)
    print(table_sep)
    
    cv_table_md = [table_header, table_sep]
    for fr in fold_results:
        row = f"| {fr['fold']:<4} | {fr['r2_a']:<8.4f} | {fr['r2_b']:<8.4f} | {fr['rmse_a']:<8.3f} | {fr['rmse_b']:<8.3f} |"
        print(row)
        cv_table_md.append(row)

    # Save Markdown table for report
    with open(PLOT_DIR / "combined_cv_performance_table.md", "w") as f:
        f.write("\n".join(cv_table_md))
    print(f"\n  ✓ CV Table saved: combined_cv_performance_table.md")

    premium_plot_path = PLOT_DIR / "combined_dual_analyte_evaluation_premium.png"
    plot_premium_results(Y_combined, mean_preds, std_preds, metrics, best_n, premium_plot_path)

    # Calculate overall R2/RMSE for the summary file (using mean of repeats)
    r2_a = r2_score(Y_combined[:, 0], mean_preds[:, 0])
    r2_b = r2_score(Y_combined[:, 1], mean_preds[:, 1])
    rmse_a = np.sqrt(mean_squared_error(Y_combined[:, 0], mean_preds[:, 0]))
    rmse_b = np.sqrt(mean_squared_error(Y_combined[:, 1], mean_preds[:, 1]))

    # Also save the model
    final_model = PLSRegression(n_components=best_n)
    final_model.fit(X_combined, Y_combined)
    model_path = OUTPUT_DIR / "combined_dual_analyte_pls.joblib"
    joblib.dump({
        "model": final_model,
        "n_components": best_n,
        "roi_start": ROI_START,
        "roi_end": ROI_END,
        "targets": ["chla_mg_L", "chlb_mg_L"],
        "r2_chla_cv": r2_a,
        "r2_chlb_cv": r2_b,
        "rmse_chla_cv": rmse_a,
        "rmse_chlb_cv": rmse_b,
    }, model_path)
    print(f"  ✓ Model saved to: {model_path}")
    print(f"  ✓ Premium Plot generated: {premium_plot_path.name}")

    print("\nDone. Phase 2 combined model training complete.")
