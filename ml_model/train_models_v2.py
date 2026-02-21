import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import stats as sp_stats
from sklearn.cross_decomposition import PLSRegression
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.model_selection import KFold, GridSearchCV, cross_val_score, cross_val_predict
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.pipeline import Pipeline
from sklearn.base import BaseEstimator, RegressorMixin

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR      = Path(__file__).parent.resolve()
DATA_DIR_A      = SCRIPT_DIR / "data/chl_a"
DATA_DIR_B      = SCRIPT_DIR / "data/chl_b"
BACKGROUND_DIR  = SCRIPT_DIR / "data/background_data"
CSV_A           = SCRIPT_DIR / "data/real_data/chla_data.csv"
CSV_B           = SCRIPT_DIR / "data/real_data/chlb_data.csv"
OUTPUT_MODEL_A  = SCRIPT_DIR / "models/chla_model.pkl"
OUTPUT_MODEL_B  = SCRIPT_DIR / "models/chlb_model.pkl"
PLOTS_DIR       = SCRIPT_DIR / "training_plots"
PLOTS_DIR.mkdir(exist_ok=True)
(SCRIPT_DIR / "models").mkdir(exist_ok=True)

# Preprocessing
ROI_START        = 1300
ROI_END          = 3200
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY   = 2
SG_DERIV_WINDOW  = 11
SG_DERIV_POLY    = 3
SG_DERIV_ORDER   = 1

# Background maps
BACKGROUND_MAP_A = {240: "background-240.csv", 250: "background-250.csv",
                    300: "background-300.csv",  500: "background-500.csv",
                    1000: "background-1000.csv"}
CLOSEST_BG_A     = {580: 500, 700: 500}

BACKGROUND_MAP_B = {1000: "background-1000.csv", 1200: "background-1200.csv",
                    1500: "background-1500.csv",  3000: "background-3000.csv",
                    5000: "background-5000.csv"}

# ============================================================
# SKLEARN WRAPPER FOR PLSRegression (needed for GridSearchCV)
# ============================================================
class PLSRWrapper(BaseEstimator, RegressorMixin):
    """Thin sklearn-compatible wrapper so GridSearchCV can tune n_components."""
    def __init__(self, n_components=5):
        self.n_components = n_components

    def fit(self, X, y):
        self.pls_ = PLSRegression(n_components=self.n_components, scale=True)
        self.pls_.fit(X, y)
        return self

    def predict(self, X):
        return self.pls_.predict(X).ravel()

    def get_params(self, deep=True):
        return {"n_components": self.n_components}

    def set_params(self, **params):
        for k, v in params.items():
            setattr(self, k, v)
        return self

# ============================================================
# DATA LOADING
# ============================================================
def load_spectrum(path):
    df   = pd.read_csv(path)
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def load_bg(int_time, bg_map, closest_map, bg_dir):
    time_key = closest_map.get(int_time, int_time)
    fname    = bg_map.get(time_key)
    if not fname: return None
    path = bg_dir / fname
    if not path.exists(): return None
    return load_spectrum(path)

# ============================================================
# BASELINE CORRECTION
# ============================================================
def asymmetric_least_squares_baseline(y, lam=1e5, p=0.01, n_iter=10):
    """
    Asymmetric Least Squares (ALS) baseline estimator.
    lam  : smoothness of baseline (higher = smoother)
    p    : asymmetry (low value = baseline hugs the bottom)
    Returns the estimated baseline vector.
    """
    from scipy.sparse import diags, eye as speye
    from scipy.sparse.linalg import spsolve

    L  = len(y)
    D  = np.diff(np.eye(L), 2)          # second-difference matrix
    D  = diags([1, -2, 1], [0, 1, 2], shape=(L - 2, L)).toarray()
    D  = D.T @ D
    w  = np.ones(L)
    for _ in range(n_iter):
        W    = diags(w)
        Z    = W + lam * D
        # solve via numpy (small enough for our spectra)
        z    = np.linalg.solve(Z.toarray() if hasattr(Z, 'toarray') else Z, w * y)
        w    = p * (y > z) + (1 - p) * (y <= z)
    return z

def correct_baseline(spectrum):
    """
    Correct the flat/elevated pre-peak baseline using ALS.
    Subtracts the estimated smooth baseline so the spectrum sits at ~zero.
    Clips negative values to 0 after correction.
    """
    baseline = asymmetric_least_squares_baseline(spectrum, lam=1e6, p=0.005, n_iter=15)
    corrected = spectrum - baseline
    corrected = np.clip(corrected, 0, None)
    return corrected

# ============================================================
# SPECTRUM CLEANING (MAD + Hotelling T²)
# ============================================================
def clean_spectra(spectra, concs, name=""):
    n = len(spectra)

    # Stage 1 — MAD on peak position
    peak_pixels = np.array([np.argmax(s) for s in spectra])
    med         = np.median(peak_pixels)
    mad         = np.median(np.abs(peak_pixels - med))
    s1_mask     = np.abs(peak_pixels - med) <= 3.0 * (mad + 1e-8)
    surv        = np.where(s1_mask)[0]

    if len(surv) < 5:
        print(f"  [{name}] Only {len(surv)} survivors after MAD — skipping T²")
        return surv

    # Stage 2 — Hotelling T²
    min_len      = min(len(spectra[i]) for i in surv)
    surv_arr     = np.array([spectra[i][:min_len] for i in surv])
    n_comp       = min(5, surv_arr.shape[0] - 1)
    pca          = PCA(n_components=n_comp)
    scores       = pca.fit_transform(surv_arr)
    sc_std       = scores.std(axis=0) + 1e-8
    t2           = np.sum((scores / sc_std) ** 2, axis=1)
    t2_thresh    = sp_stats.chi2.ppf(0.99, df=n_comp)
    good         = np.where(t2 <= t2_thresh)[0]
    clean_idx    = surv[good]

    removed = set(range(n)) - set(clean_idx)
    print(f"  [{name}] {n} → {len(clean_idx)} clean  |  removed {len(removed)}: {sorted(removed)}")
    return clean_idx

# ============================================================
# PREPROCESSING PIPELINE
# ============================================================
def preprocess_spectrum(spectrum, roi_start=ROI_START, roi_end=ROI_END,
                        apply_baseline=False):
    """
    Full preprocessing pipeline:
      1. Slice ROI
      2. (Optional) ALS baseline correction  ← new for Chl-B
      3. Savitzky-Golay smoothing
      4. Savitzky-Golay first derivative
      5. SNV normalization
    """
    sliced = spectrum[roi_start:roi_end]

    if apply_baseline:
        sliced = correct_baseline(sliced)

    smoothed  = savgol_filter(sliced,  SG_SMOOTH_WINDOW, SG_SMOOTH_POLY)
    deriv     = savgol_filter(smoothed, SG_DERIV_WINDOW,  SG_DERIV_POLY,
                              deriv=SG_DERIV_ORDER)
    mean, std = np.mean(deriv), np.std(deriv)
    snv       = (deriv - mean) / (std + 1e-8)
    return snv

# ============================================================
# DATASET BUILDER
# ============================================================
def build_dataset(csv_path, data_dir, bg_map, closest_map, apply_baseline=False):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return np.array([]), np.array([])

    df.columns = [c.lower().strip() for c in df.columns]
    if "solution num"        in df.columns: df.rename(columns={"solution num": "sample"}, inplace=True)
    if "sample con"          in df.columns: df.rename(columns={"sample con": "concentration"}, inplace=True)
    if "integration time(ms)" in df.columns: df.rename(columns={"integration time(ms)": "integration_time"}, inplace=True)
    if "sample" in df.columns:
        df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)

    df["concentration"]  = pd.to_numeric(df["concentration"],  errors="coerce")
    df["integration_time"] = pd.to_numeric(df.get("integration_time", pd.Series(dtype=float)), errors="coerce").fillna(1000)
    df = df.dropna(subset=["concentration", "integration_time"])

    X, y, bg_cache = [], [], {}
    for _, row in df.iterrows():
        s_num = int(row["sample_num"])
        conc  = row["concentration"]
        it    = int(row["integration_time"])
        fpath = data_dir / f"{s_num}.csv"
        if not fpath.exists(): continue

        spec = load_spectrum(fpath)
        if it not in bg_cache:
            bg_cache[it] = load_bg(it, bg_map, closest_map, BACKGROUND_DIR)
        bg = bg_cache[it]
        if bg is not None:
            min_len = min(len(spec), len(bg))
            spec = spec[:min_len] - bg[:min_len]

        spec = spec / (it + 1e-8)
        processed = preprocess_spectrum(spec, ROI_START, ROI_END,
                                        apply_baseline=apply_baseline)
        X.append(processed)
        y.append(conc)

    if not X: return np.array([]), np.array([])
    return np.array(X), np.array(y)

# ============================================================
# GRID SEARCH + EVALUATION
# ============================================================
def grid_search_plsr(X, y, name, max_components=15):
    """
    GridSearchCV over n_components for PLSRWrapper.
    Uses 5-fold CV with neg_root_mean_squared_error scoring.
    Returns best model and prints a full report.
    """
    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    param_grid = {"n_components": list(range(1, min(max_components + 1, X.shape[0] // 5)))}

    gs = GridSearchCV(
        PLSRWrapper(),
        param_grid,
        scoring="neg_root_mean_squared_error",
        cv=kf,
        n_jobs=-1,
        verbose=0,
        return_train_score=True
    )
    gs.fit(X, y)

    best_n   = gs.best_params_["n_components"]
    best_rmse = -gs.best_score_

    print(f"\n  [{name}] GridSearchCV Results:")
    print(f"    Best n_components : {best_n}")
    print(f"    Best CV RMSE      : {best_rmse:.4f}")

    # Plot CV RMSE vs n_components
    results  = gs.cv_results_
    n_range  = param_grid["n_components"]
    cv_rmse  = -results["mean_test_score"]
    cv_std   = results["std_test_score"]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(n_range, cv_rmse, 'o-', color='steelblue', label='CV RMSE (mean)')
    ax.fill_between(n_range, cv_rmse - cv_std, cv_rmse + cv_std,
                    alpha=0.2, color='steelblue', label='±1 std')
    ax.axvline(best_n, color='red', linestyle='--', label=f'Best n={best_n}')
    ax.set_xlabel("n_components")
    ax.set_ylabel("CV RMSE (ppm)")
    ax.set_title(f"{name} — GridSearchCV: RMSE vs n_components")
    ax.legend()
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_gridsearch.png", dpi=150)
    plt.close(fig)

    return gs.best_estimator_

def full_evaluation(model, X, y, name):
    """
    Fit on all data, then run 5-fold CV.
    Produces: metrics printout + predicted vs actual plot.
    """
    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    # Training metrics
    model.fit(X, y)
    y_train_pred = model.predict(X)
    train_r2   = r2_score(y, y_train_pred)
    train_rmse = np.sqrt(mean_squared_error(y, y_train_pred))

    # CV predictions (leave out each fold)
    y_cv_pred = cross_val_predict(model, X, y, cv=kf)
    cv_r2     = r2_score(y, y_cv_pred)
    cv_rmse   = np.sqrt(mean_squared_error(y, y_cv_pred))

    print(f"\n  [{name}] Final Evaluation:")
    print(f"    Training  R²   : {train_r2:.4f}   RMSE: {train_rmse:.4f} ppm")
    print(f"    CV (5-fold) R² : {cv_r2:.4f}   RMSE: {cv_rmse:.4f} ppm")
    gap = train_r2 - cv_r2
    if gap > 0.15:
        print(f"    [WARNING] R² gap = {gap:.3f} → likely overfitting. Collect more data.")
    elif gap > 0.05:
        print(f"    [OK-ish]  R² gap = {gap:.3f} → mild overfit, acceptable.")
    else:
        print(f"    [GOOD]    R² gap = {gap:.3f} → generalizes well.")

    # Predicted vs Actual plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(f"{name} — Model Evaluation", fontsize=13, fontweight='bold')

    # CV predicted vs actual
    ax = axes[0]
    sc = ax.scatter(y, y_cv_pred, c=y, cmap='viridis', s=60, edgecolors='k', lw=0.4, zorder=5)
    lims = [min(y.min(), y_cv_pred.min()) * 0.9, max(y.max(), y_cv_pred.max()) * 1.05]
    ax.plot(lims, lims, 'r--', lw=1.5, label='Perfect prediction')
    ax.set_xlim(lims); ax.set_ylim(lims)
    ax.set_xlabel("Actual Concentration (ppm)")
    ax.set_ylabel("CV Predicted (ppm)")
    ax.set_title(f"CV Predicted vs Actual\nR²={cv_r2:.3f}  RMSE={cv_rmse:.3f} ppm")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=ax, label="Concentration (ppm)")

    # Residuals
    ax2 = axes[1]
    residuals = y_cv_pred - y
    ax2.scatter(y, residuals, c=y, cmap='viridis', s=60, edgecolors='k', lw=0.4, zorder=5)
    ax2.axhline(0, color='red', linestyle='--', lw=1.5)
    ax2.axhline(cv_rmse,  color='orange', linestyle=':', lw=1, label=f'+RMSE ({cv_rmse:.2f})')
    ax2.axhline(-cv_rmse, color='orange', linestyle=':', lw=1, label=f'-RMSE ({cv_rmse:.2f})')
    ax2.set_xlabel("Actual Concentration (ppm)")
    ax2.set_ylabel("Residual (ppm)")
    ax2.set_title("CV Residuals\n(systematic pattern → model issue, random → noise)")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_evaluation.png", dpi=150)
    plt.close(fig)

    return model

# ============================================================
# BASELINE CORRECTION QC PLOT
# ============================================================
def plot_baseline_qc(X_raw_spectra, X_corrected_spectra, concs, name):
    """Show before/after baseline correction for a few example spectra."""
    n = min(6, len(X_raw_spectra))
    cmap = plt.colormaps['viridis']
    colors = [cmap(i / max(n - 1, 1)) for i in range(n)]

    fig, axes = plt.subplots(1, 2, figsize=(16, 5))
    fig.suptitle(f"{name} — Baseline Correction QC", fontsize=13, fontweight='bold')

    for i in range(n):
        axes[0].plot(X_raw_spectra[i], color=colors[i], lw=1, alpha=0.8,
                     label=f"{concs[i]:.1f} ppm")
        axes[1].plot(X_corrected_spectra[i], color=colors[i], lw=1, alpha=0.8,
                     label=f"{concs[i]:.1f} ppm")

    for ax, title in zip(axes, ["Before Baseline Correction", "After ALS Baseline Correction"]):
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.set_xlabel("ROI Pixel"); ax.set_ylabel("Intensity / ms")
        ax.grid(True, alpha=0.25); ax.legend(fontsize=7, loc='upper right')
        ax.spines['top'].set_visible(False); ax.spines['right'].set_visible(False)

    plt.tight_layout()
    fig.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_baseline_qc.png", dpi=150)
    plt.close(fig)
    print(f"  Saved baseline QC plot.")

# ============================================================
# MAIN
# ============================================================
def train_and_save():

    # ---- CHL-A (PLSR, no baseline correction needed) ----
    print("=" * 55)
    print("TRAINING Chl-A  (PLSR)")
    print("=" * 55)
    Xa, ya = build_dataset(CSV_A, DATA_DIR_A, BACKGROUND_MAP_A, CLOSEST_BG_A,
                           apply_baseline=False)

    # Append NPZ data if present
    npz_path = SCRIPT_DIR.parent / "dataset.npz"
    if npz_path.exists():
        try:
            data   = np.load(npz_path, allow_pickle=True)
            concs  = data['concentrations']
            laser_a = data['laser_a_spectra']
            dark   = data['dark_spectra']
            meta   = data['metadata']
            new_X, new_y = [], []
            for i in range(len(concs)):
                it   = meta[i].get('integration_time', 1000) if i < len(meta) else 1000
                net  = (laser_a[i].astype(float) - dark[i].astype(float)) / (it + 1e-8)
                new_X.append(preprocess_spectrum(net, ROI_START, ROI_END, apply_baseline=False))
                new_y.append(concs[i][0])
            if new_X:
                Xa = np.vstack([Xa, new_X]) if len(Xa) > 0 else np.array(new_X)
                ya = np.concatenate([ya, new_y]) if len(ya) > 0 else np.array(new_y)
                print(f"  Added {len(new_X)} NPZ samples to Chl-A.")
        except Exception as e:
            print(f"  NPZ load failed: {e}")

    if len(Xa) == 0:
        print("  No Chl-A data found.")
    else:
        # Clean
        clean_idx = clean_spectra(Xa, ya, "Chl-A")
        Xa, ya = Xa[clean_idx], ya[clean_idx]

        # Grid search for best n_components
        best_model_a = grid_search_plsr(Xa, ya, "Chl-A", max_components=15)

        # Final evaluation
        best_model_a = full_evaluation(best_model_a, Xa, ya, "Chl-A")

        # Refit final PLS on all clean data with best n_components
        n_best_a = best_model_a.n_components
        final_pls_a = PLSRegression(n_components=n_best_a, scale=True)
        final_pls_a.fit(Xa, ya)
        joblib.dump(final_pls_a, OUTPUT_MODEL_A)
        print(f"  Saved Chl-A model → {OUTPUT_MODEL_A}")

    # ---- CHL-B (PLSR + ALS baseline correction) ----
    print("\n" + "=" * 55)
    print("TRAINING Chl-B  (PLSR + ALS Baseline Correction)")
    print("=" * 55)

    # Build raw (pre-baseline) for QC plot
    Xb_raw, yb_raw = build_dataset(CSV_B, DATA_DIR_B, BACKGROUND_MAP_B, {},
                                   apply_baseline=False)
    # Build with baseline correction for training
    Xb, yb = build_dataset(CSV_B, DATA_DIR_B, BACKGROUND_MAP_B, {},
                            apply_baseline=True)

    # NPZ data for Chl-B
    if npz_path.exists():
        try:
            data    = np.load(npz_path, allow_pickle=True)
            concs   = data['concentrations']
            laser_b = data['laser_b_spectra']
            dark    = data['dark_spectra']
            meta    = data['metadata']
            new_X, new_y = [], []
            for i in range(len(concs)):
                it  = meta[i].get('integration_time', 1000) if i < len(meta) else 1000
                net = (laser_b[i].astype(float) - dark[i].astype(float)) / (it + 1e-8)
                new_X.append(preprocess_spectrum(net, ROI_START, ROI_END, apply_baseline=True))
                new_y.append(concs[i][1])
            if new_X:
                Xb = np.vstack([Xb, new_X]) if len(Xb) > 0 else np.array(new_X)
                yb = np.concatenate([yb, new_y]) if len(yb) > 0 else np.array(new_y)
                print(f"  Added {len(new_X)} NPZ samples to Chl-B.")
        except Exception as e:
            print(f"  NPZ load failed: {e}")

    if len(Xb) == 0:
        print("  No Chl-B data found.")
    else:
        # Baseline QC plot (raw ROI slices before full preprocessing)
        if len(Xb_raw) > 0:
            plot_baseline_qc(Xb_raw[:6], Xb[:6], yb[:6], "Chl-B")

        # Clean
        clean_idx = clean_spectra(Xb, yb, "Chl-B")
        Xb, yb = Xb[clean_idx], yb[clean_idx]

        # Grid search
        best_model_b = grid_search_plsr(Xb, yb, "Chl-B", max_components=15)

        # Final evaluation
        best_model_b = full_evaluation(best_model_b, Xb, yb, "Chl-B")

        # Save final model
        n_best_b = best_model_b.n_components
        final_pls_b = PLSRegression(n_components=n_best_b, scale=True)
        final_pls_b.fit(Xb, yb)
        joblib.dump(final_pls_b, OUTPUT_MODEL_B)
        print(f"  Saved Chl-B model → {OUTPUT_MODEL_B}")

    print("\n✓ Training complete. Plots saved to:", PLOTS_DIR)


if __name__ == "__main__":
    train_and_save()
