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
PLOTS_DIR       = SCRIPT_DIR / "training_plots_v4.1"
PLOTS_DIR.mkdir(exist_ok=True)
(SCRIPT_DIR / "models").mkdir(exist_ok=True)

# ── Preprocessing ─────────────────────────────────────────────────────────────
# X-AXIS ALIGNMENT: instead of a fixed pixel crop, we align the peak first
# then crop a fixed window AROUND the aligned peak center.
# This prevents asymmetric cropping caused by mirror/tube placement variation.
#
# ROI_HALF_WIDTH: pixels to keep on each side of the aligned peak center
# Total features = 2 * ROI_HALF_WIDTH
# Old approach: ROI_START=1300, ROI_END=3200 → 1900 features (fixed window)
# New approach: align then crop ±950 → 1900 features (peak-centered window)
ROI_HALF_WIDTH   = 2500  # ← adjust if you want wider/narrower window

SG_SMOOTH_WINDOW = 31
SG_SMOOTH_POLY   = 2
SG_DERIV_WINDOW  = 31
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
# Y-TRANSFORM CONFIGURATION
# 'log'  → log(1+y), back-transform: exp(y_pred)-1
# 'sqrt' → sqrt(y),  back-transform: y_pred²
# 'none' → no transform
# ============================================================
Y_TRANSFORM = 'log'

def y_transform(y):
    if Y_TRANSFORM == 'log':   return np.log1p(y)
    elif Y_TRANSFORM == 'sqrt': return np.sqrt(np.clip(y, 0, None))
    return y.copy()

def y_inverse(y_t):
    if Y_TRANSFORM == 'log':   return np.expm1(y_t)
    elif Y_TRANSFORM == 'sqrt': return np.clip(y_t, 0, None) ** 2
    return y_t.copy()

# ============================================================
# X-AXIS PEAK ALIGNMENT
# ============================================================
def align_and_crop(spectrum, half_width=ROI_HALF_WIDTH):
    """
    Align spectrum peak to center BEFORE cropping.

    Why this matters:
      Old approach: fixed pixel crop [1300:3200]
        → if peak shifts right due to placement variation,
          the right tail gets cut off asymmetrically
      New approach: find peak on FULL spectrum → shift → crop ±half_width
        → every spectrum always has its peak at the center of the crop window

    Steps:
      1. Find peak index on full raw spectrum
      2. Circular-shift so peak lands at center (n//2)
      3. Crop ±half_width around center

    Uses np.roll (circular shift) — safe because spectrum edges
    are near-zero baseline so any wrap-around is negligible.
    """
    n        = len(spectrum)
    peak_idx = np.argmax(spectrum)
    center   = n // 2
    shift    = center - peak_idx
    aligned  = np.roll(spectrum, shift)
    start    = max(0, center - half_width)
    end      = min(n, center + half_width)
    return aligned[start:end]

# ============================================================
# SKLEARN WRAPPER FOR PLSRegression
# ============================================================
class PLSRWrapper(BaseEstimator, RegressorMixin):
    """
    Sklearn-compatible PLSR wrapper for GridSearchCV.
    Trains in TRANSFORMED y-space (log/sqrt/none).
    predict() returns back-transformed values in original ppm space.
    """
    def __init__(self, n_components=5):
        self.n_components = n_components

    def fit(self, X, y):
        self.pls_ = PLSRegression(n_components=self.n_components, scale=True)
        self.pls_.fit(X, y_transform(y))
        return self

    def predict(self, X):
        return y_inverse(self.pls_.predict(X).ravel())

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
# BASELINE CORRECTION (ALS) — sparse solver for speed
# ============================================================
def correct_baseline(spectrum):
    """
    Asymmetric Least Squares (ALS) baseline correction.
    Uses sparse solver — much faster than dense np.linalg.solve.
    Applied AFTER x-alignment and crop.
    """
    from scipy.sparse import diags
    from scipy.sparse.linalg import spsolve

    y    = spectrum
    L    = len(y)
    D    = diags([1, -2, 1], [0, 1, 2], shape=(L-2, L))
    DDT  = D.T.dot(D)
    w    = np.ones(L)
    lam, p = 1e6, 0.005
    for _ in range(10):
        W = diags(w)
        z = spsolve(W + lam * DDT, w * y)
        w = p * (y > z) + (1 - p) * (y <= z)
    return np.clip(y - z, 0, None)

# ============================================================
# SPECTRUM CLEANING (MAD + Hotelling T²)
# Loosened thresholds: 5 MAD + 99.9% T² to keep more samples
# ============================================================
def clean_spectra(spectra, concs, name=""):
    n = len(spectra)

    # Stage 1 — MAD on peak position (5 MAD = loose, keeps borderline samples)
    peak_pixels = np.array([np.argmax(s) for s in spectra])
    med         = np.median(peak_pixels)
    mad         = np.median(np.abs(peak_pixels - med))
    s1_mask     = np.abs(peak_pixels - med) <= 5.0 * (mad + 1e-8)
    surv        = np.where(s1_mask)[0]

    if len(surv) < 5:
        print(f"  [{name}] Only {len(surv)} survivors after MAD — skipping T²")
        return surv

    # Stage 2 — Hotelling T² at 99.9% (very conservative — only egregious outliers)
    min_len   = min(len(spectra[i]) for i in surv)
    surv_arr  = np.array([spectra[i][:min_len] for i in surv])
    n_comp    = min(5, surv_arr.shape[0] - 1)
    pca       = PCA(n_components=n_comp)
    scores    = pca.fit_transform(surv_arr)
    sc_std    = scores.std(axis=0) + 1e-8
    t2        = np.sum((scores / sc_std) ** 2, axis=1)
    t2_thresh = sp_stats.chi2.ppf(0.999, df=n_comp)
    good      = np.where(t2 <= t2_thresh)[0]
    clean_idx = surv[good]

    removed = set(range(n)) - set(clean_idx)
    print(f"  [{name}] {n} → {len(clean_idx)} clean  |  removed {len(removed)}: {sorted(removed)}")
    return clean_idx

# ============================================================
# PREPROCESSING PIPELINE
# ============================================================
# ORDER (critical — alignment must happen before crop):
#   1. Background subtract          (in build_dataset)
#   2. / integration_time           (y-normalization, in build_dataset)
#   3. align_and_crop()             ← x-normalization: align peak THEN crop
#   4. correct_baseline() if Chl-B  ← ALS on the already-cropped region
#   5. Savitzky-Golay smooth
#   6. Savitzky-Golay first derivative
#   7. SNV
# ============================================================
def preprocess_spectrum(spectrum, apply_baseline=False):
    """
    Full preprocessing. spectrum is the full raw array after bg subtract + /int_time.
    Peak alignment happens here on the full spectrum before any crop.
    """
    # Step 1: align peak then crop (x-normalization)
    cropped = align_and_crop(spectrum)

    # Step 2: ALS baseline correction (Chl-B only)
    if apply_baseline:
        cropped = correct_baseline(cropped)

    # Step 3: Savitzky-Golay smooth
    smoothed = savgol_filter(cropped, SG_SMOOTH_WINDOW, SG_SMOOTH_POLY)

    # Step 4: First derivative
    deriv    = savgol_filter(smoothed, SG_DERIV_WINDOW, SG_DERIV_POLY,
                             deriv=SG_DERIV_ORDER)

    # Step 5: SNV
    mean, std = np.mean(deriv), np.std(deriv)
    return (deriv - mean) / (std + 1e-8)

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
    if "solution num"          in df.columns: df.rename(columns={"solution num": "sample"}, inplace=True)
    if "sample con"            in df.columns: df.rename(columns={"sample con": "concentration"}, inplace=True)
    if "integration time(ms)"  in df.columns: df.rename(columns={"integration time(ms)": "integration_time"}, inplace=True)
    if "sample" in df.columns:
        df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)

    df["concentration"]    = pd.to_numeric(df["concentration"],  errors="coerce")
    df["integration_time"] = pd.to_numeric(
        df.get("integration_time", pd.Series(dtype=float)), errors="coerce").fillna(1000)
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
            spec    = spec[:min_len] - bg[:min_len]

        # y-normalize BEFORE preprocess (alignment uses intensity-scale peak)
        spec      = spec / (it + 1e-8)
        processed = preprocess_spectrum(spec, apply_baseline=apply_baseline)
        X.append(processed)
        y.append(conc)

    if not X: return np.array([]), np.array([])
    return np.array(X), np.array(y)

# ============================================================
# GRID SEARCH
# ============================================================
def grid_search_plsr(X, y, name, max_components=15):
    kf         = KFold(n_splits=5, shuffle=True, random_state=42)
    param_grid = {"n_components": list(range(1, min(max_components + 1, X.shape[0] // 5)))}

    gs = GridSearchCV(PLSRWrapper(), param_grid,
                      scoring="neg_root_mean_squared_error",
                      cv=kf, n_jobs=-1, verbose=0, return_train_score=True)
    gs.fit(X, y)

    best_n    = gs.best_params_["n_components"]
    best_rmse = -gs.best_score_

    print(f"\n  [{name}] GridSearchCV Results:")
    print(f"    Best n_components : {best_n}")
    print(f"    Best CV RMSE      : {best_rmse:.4f}")

    results = gs.cv_results_
    n_range = param_grid["n_components"]
    cv_rmse = -results["mean_test_score"]
    cv_std  = results["std_test_score"]

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(n_range, cv_rmse, 'o-', color='steelblue', label='CV RMSE (mean)')
    ax.fill_between(n_range, cv_rmse - cv_std, cv_rmse + cv_std,
                    alpha=0.2, color='steelblue', label='±1 std')
    ax.axvline(best_n, color='red', linestyle='--', label=f'Best n={best_n}')
    ax.set_xlabel("n_components"); ax.set_ylabel("CV RMSE (ppm)")
    ax.set_title(f"{name} — GridSearchCV: RMSE vs n_components")
    ax.legend(); ax.grid(True, alpha=0.3)
    plt.tight_layout()
    fig.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_gridsearch.png", dpi=150)
    plt.close(fig)

    return gs.best_estimator_

# ============================================================
# EVALUATION + PLOTS
# ============================================================
def full_evaluation(model, X, y, name):
    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    model.fit(X, y)
    y_train_pred = model.predict(X)
    train_r2     = r2_score(y, y_train_pred)
    train_rmse   = np.sqrt(mean_squared_error(y, y_train_pred))

    y_cv_pred   = cross_val_predict(model, X, y, cv=kf)
    cv_r2       = r2_score(y, y_cv_pred)
    cv_rmse     = np.sqrt(mean_squared_error(y, y_cv_pred))
    residuals   = y_cv_pred - y
    y_t         = y_transform(y)
    y_cv_t      = y_transform(np.clip(y_cv_pred, 1e-6, None))
    residuals_t = y_cv_t - y_t

    print(f"\n  [{name}] Final Evaluation  [x_align=True | Y_TRANSFORM='{Y_TRANSFORM}']:")
    print(f"    Training  R²   : {train_r2:.4f}   RMSE: {train_rmse:.4f} ppm")
    print(f"    CV (5-fold) R² : {cv_r2:.4f}   RMSE: {cv_rmse:.4f} ppm")
    gap = train_r2 - cv_r2
    if   gap > 0.15: print(f"    [WARNING] R² gap={gap:.3f} → overfitting. Collect more data.")
    elif gap > 0.05: print(f"    [OK-ish]  R² gap={gap:.3f} → mild overfit, acceptable.")
    else:            print(f"    [GOOD]    R² gap={gap:.3f} → generalizes well.")

    # ── 4-panel evaluation plot ───────────────────────────────────────────────
    fig, axes = plt.subplots(1, 4, figsize=(26, 5))
    fig.suptitle(f"{name} — Model Evaluation  [x_align=True | transform='{Y_TRANSFORM}']",
                 fontsize=13, fontweight='bold')

    # P1: predicted vs actual
    ax = axes[0]
    sc = ax.scatter(y, y_cv_pred, c=y, cmap='viridis', s=60, edgecolors='k', lw=0.4, zorder=5)
    lims = [min(y.min(), y_cv_pred.min()) * 0.9, max(y.max(), y_cv_pred.max()) * 1.05]
    ax.plot(lims, lims, 'r--', lw=1.5, label='Perfect prediction')
    ax.set_xlim(lims); ax.set_ylim(lims)
    ax.set_xlabel("Actual Concentration (ppm)"); ax.set_ylabel("CV Predicted (ppm)")
    ax.set_title(f"CV Predicted vs Actual\nR²={cv_r2:.3f}  RMSE={cv_rmse:.3f} ppm")
    ax.legend(); ax.grid(True, alpha=0.3)
    fig.colorbar(sc, ax=ax, label="Concentration (ppm)")

    # P2: residuals ppm
    ax2 = axes[1]
    ax2.scatter(y, residuals, c=y, cmap='viridis', s=60, edgecolors='k', lw=0.4, zorder=5)
    ax2.axhline(0,        color='red',    linestyle='--', lw=1.5)
    ax2.axhline( cv_rmse, color='orange', linestyle=':',  lw=1, label=f'+RMSE ({cv_rmse:.2f})')
    ax2.axhline(-cv_rmse, color='orange', linestyle=':',  lw=1, label=f'-RMSE ({cv_rmse:.2f})')
    ax2.set_xlabel("Actual Concentration (ppm)"); ax2.set_ylabel("Residual (ppm)")
    ax2.set_title("Residuals in ppm space\n(funnel shape = heteroscedasticity)")
    ax2.legend(); ax2.grid(True, alpha=0.3)

    # P3: residuals log-space
    ax3 = axes[2]
    rmse_t = np.sqrt(mean_squared_error(y_t, y_cv_t))
    ax3.scatter(y, residuals_t, c=y, cmap='plasma', s=60, edgecolors='k', lw=0.4, zorder=5)
    ax3.axhline(0,       color='red',   linestyle='--', lw=1.5)
    ax3.axhline( rmse_t, color='green', linestyle=':',  lw=1, label=f'+RMSE_t ({rmse_t:.3f})')
    ax3.axhline(-rmse_t, color='green', linestyle=':',  lw=1, label=f'-RMSE_t ({rmse_t:.3f})')
    ax3.set_xlabel("Actual Concentration (ppm)")
    ax3.set_ylabel(f"Residual in {Y_TRANSFORM}-space")
    ax3.set_title(f"Residuals in {Y_TRANSFORM}-space\n(flat/random = transform worked)")
    ax3.legend(); ax3.grid(True, alpha=0.3)

    # P4: variance profile (rolling std of residuals across concentration range)
    ax4 = axes[3]
    sort_v   = np.argsort(y)
    y_sv     = y[sort_v]
    res_sv   = residuals[sort_v]
    win      = max(5, len(y) // 8)
    roll_std = np.array([res_sv[max(0, i-win):i+win].std() for i in range(len(res_sv))])
    ax4.fill_between(y_sv, -roll_std, roll_std, alpha=0.3, color='steelblue', label='±1 local std')
    ax4.plot(y_sv,  roll_std, color='steelblue', lw=1.5)
    ax4.plot(y_sv, -roll_std, color='steelblue', lw=1.5)
    ax4.scatter(y, residuals, color='gray', s=20, alpha=0.5, zorder=3)
    ax4.axhline(0, color='red', linestyle='--', lw=1.5)
    ax4.set_xlabel("Actual Concentration (ppm)"); ax4.set_ylabel("Residual (ppm)")
    ax4.set_title("Variance Profile\n(wider band = less reliable at that concentration)")
    ax4.legend(fontsize=8); ax4.grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_evaluation.png", dpi=150)
    plt.close(fig)

    # ── Concentration output plot ─────────────────────────────────────────────
    sort_idx    = np.argsort(y)
    y_sorted    = y[sort_idx]
    pred_sorted = y_cv_pred[sort_idx]
    err_sorted  = pred_sorted - y_sorted
    abs_err     = np.abs(err_sorted)
    pct_err     = np.where(y_sorted > 0, 100 * abs_err / y_sorted, np.nan)
    n_samples   = len(y_sorted)
    x           = np.arange(n_samples)
    cmap_bar    = plt.colormaps['viridis']
    bar_colors  = [cmap_bar(v / y_sorted.max()) for v in y_sorted]
    width       = 0.38

    fig2, axes2 = plt.subplots(3, 1, figsize=(14, 12))
    fig2.suptitle(f"{name} — CV Concentration Outputs  [transform='{Y_TRANSFORM}']\n"
                  f"(sorted by actual concentration)", fontsize=13, fontweight='bold')

    axes2[0].bar(x - width/2, y_sorted,    width, label='Actual',
                 color='steelblue',  alpha=0.85, edgecolor='k', lw=0.3)
    axes2[0].bar(x + width/2, pred_sorted, width, label='CV Predicted',
                 color='darkorange', alpha=0.85, edgecolor='k', lw=0.3)
    axes2[0].set_ylabel("Concentration (ppm)")
    axes2[0].set_title("Actual vs CV Predicted Concentration")
    axes2[0].legend(); axes2[0].grid(True, alpha=0.25, axis='y')
    axes2[0].spines['top'].set_visible(False); axes2[0].spines['right'].set_visible(False)

    bar_cols = ['tomato' if e > 0 else 'steelblue' for e in err_sorted]
    axes2[1].bar(x, err_sorted, color=bar_cols, edgecolor='k', lw=0.3, alpha=0.85)
    axes2[1].axhline(0,        color='black', lw=1.0)
    axes2[1].axhline( cv_rmse, color='red',   lw=1.2, linestyle='--', label=f'+RMSE ({cv_rmse:.3f})')
    axes2[1].axhline(-cv_rmse, color='red',   lw=1.2, linestyle='--', label=f'-RMSE ({cv_rmse:.3f})')
    axes2[1].set_ylabel("Error (ppm)  [pred − actual]")
    axes2[1].set_title("Prediction Error per Sample  (red=overestimate, blue=underestimate)")
    axes2[1].legend(fontsize=9); axes2[1].grid(True, alpha=0.25, axis='y')
    axes2[1].spines['top'].set_visible(False); axes2[1].spines['right'].set_visible(False)

    axes2[2].bar(x, pct_err, color=bar_colors, edgecolor='k', lw=0.3, alpha=0.9)
    axes2[2].axhline(10, color='red',    lw=1.2, linestyle='--', label='10% threshold')
    axes2[2].axhline(5,  color='orange', lw=1.0, linestyle=':',  label='5% threshold')
    axes2[2].set_ylabel("Absolute % Error")
    axes2[2].set_title("Absolute % Error per Sample")
    axes2[2].set_xticks(x)
    axes2[2].set_xticklabels([f"{v:.2f}" for v in y_sorted], rotation=70, fontsize=7)
    axes2[2].set_xlabel("Actual Concentration (ppm)")
    axes2[2].legend(fontsize=9); axes2[2].grid(True, alpha=0.25, axis='y')
    axes2[2].spines['top'].set_visible(False); axes2[2].spines['right'].set_visible(False)

    plt.tight_layout()
    fig2.savefig(PLOTS_DIR / f"{name.lower().replace(' ','_')}_concentration_outputs.png", dpi=150)
    plt.close(fig2)

    # ── CSV ───────────────────────────────────────────────────────────────────
    pd.DataFrame({
        "sample_rank"      : np.arange(1, n_samples + 1),
        "actual_ppm"       : y_sorted,
        "cv_predicted_ppm" : np.round(pred_sorted, 4),
        "error_ppm"        : np.round(err_sorted,  4),
        "abs_error_ppm"    : np.round(abs_err,      4),
        "pct_error"        : np.round(pct_err,      2),
    }).to_csv(PLOTS_DIR / f"{name.lower().replace(' ','_')}_concentration_outputs.csv", index=False)

    print(f"\n  [{name}] Concentration Output Summary:")
    print(f"    Mean abs error           : {abs_err.mean():.4f} ppm")
    print(f"    Mean % error             : {np.nanmean(pct_err):.2f}%")
    print(f"    Max % error              : {np.nanmax(pct_err):.2f}%")
    print(f"    Samples within 10% error : {(pct_err <= 10).sum()}/{n_samples}")

    return model

# ============================================================
# BASELINE QC PLOT
# ============================================================
def plot_baseline_qc(X_raw_spectra, X_corrected_spectra, concs, name):
    n      = min(6, len(X_raw_spectra))
    cmap   = plt.colormaps['viridis']
    colors = [cmap(i / max(n - 1, 1)) for i in range(n)]

    fig, axes = plt.subplots(1, 2, figsize=(16, 5))
    fig.suptitle(f"{name} — Baseline Correction QC  [after x-alignment]",
                 fontsize=13, fontweight='bold')

    for i in range(n):
        axes[0].plot(X_raw_spectra[i],  color=colors[i], lw=1, alpha=0.8, label=f"{concs[i]:.1f} ppm")
        axes[1].plot(X_corrected_spectra[i], color=colors[i], lw=1, alpha=0.8, label=f"{concs[i]:.1f} ppm")

    for ax, title in zip(axes, ["Before ALS Correction", "After ALS Correction"]):
        ax.set_title(title, fontsize=11, fontweight='bold')
        ax.set_xlabel("Aligned ROI Pixel"); ax.set_ylabel("Intensity / ms")
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

    # ---- CHL-A ---------------------------------------------------------------
    print("=" * 60)
    print("TRAINING Chl-A  (PLSR + x-alignment + log transform)")
    print("=" * 60)
    Xa, ya = build_dataset(CSV_A, DATA_DIR_A, BACKGROUND_MAP_A, CLOSEST_BG_A,
                           apply_baseline=False)

    npz_path = SCRIPT_DIR.parent / "dataset.npz"
    if npz_path.exists():
        try:
            data    = np.load(npz_path, allow_pickle=True)
            concs   = data['concentrations']
            laser_a = data['laser_a_spectra']
            dark    = data['dark_spectra']
            meta    = data['metadata']
            new_X, new_y = [], []
            for i in range(len(concs)):
                it  = meta[i].get('integration_time', 1000) if i < len(meta) else 1000
                net = (laser_a[i].astype(float) - dark[i].astype(float)) / (it + 1e-8)
                new_X.append(preprocess_spectrum(net, apply_baseline=False))
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
        clean_idx    = clean_spectra(Xa, ya, "Chl-A")
        Xa, ya       = Xa[clean_idx], ya[clean_idx]
        best_model_a = grid_search_plsr(Xa, ya, "Chl-A", max_components=15)
        best_model_a = full_evaluation(best_model_a, Xa, ya, "Chl-A")

        n_best_a    = best_model_a.n_components
        final_pls_a = PLSRegression(n_components=n_best_a, scale=True)
        final_pls_a.fit(Xa, y_transform(ya))
        joblib.dump({
            'model'      : final_pls_a,
            'transform'  : Y_TRANSFORM,
            'x_aligned'  : True,
            'half_width' : ROI_HALF_WIDTH,
        }, OUTPUT_MODEL_A)
        print(f"  Saved Chl-A model → {OUTPUT_MODEL_A}  (transform='{Y_TRANSFORM}', x_align=True)")

    # ---- CHL-B ---------------------------------------------------------------
    print("\n" + "=" * 60)
    print("TRAINING Chl-B  (PLSR + x-alignment + ALS baseline + log transform)")
    print("=" * 60)

    # Build raw spectra for QC plot (aligned but no ALS)
    Xb_raw, yb_raw = build_dataset(CSV_B, DATA_DIR_B, BACKGROUND_MAP_B, {},
                                   apply_baseline=False)
    # Build with ALS for training
    Xb, yb = build_dataset(CSV_B, DATA_DIR_B, BACKGROUND_MAP_B, {},
                           apply_baseline=True)

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
                new_X.append(preprocess_spectrum(net, apply_baseline=True))
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
        if len(Xb_raw) > 0:
            plot_baseline_qc(Xb_raw[:6], Xb[:6], yb[:6], "Chl-B")

        clean_idx    = clean_spectra(Xb, yb, "Chl-B")
        Xb, yb       = Xb[clean_idx], yb[clean_idx]
        best_model_b = grid_search_plsr(Xb, yb, "Chl-B", max_components=15)
        best_model_b = full_evaluation(best_model_b, Xb, yb, "Chl-B")

        n_best_b    = best_model_b.n_components
        final_pls_b = PLSRegression(n_components=n_best_b, scale=True)
        final_pls_b.fit(Xb, y_transform(yb))
        joblib.dump({
            'model'      : final_pls_b,
            'transform'  : Y_TRANSFORM,
            'x_aligned'  : True,
            'half_width' : ROI_HALF_WIDTH,
        }, OUTPUT_MODEL_B)
        print(f"  Saved Chl-B model → {OUTPUT_MODEL_B}  (transform='{Y_TRANSFORM}', x_align=True)")

    print(f"\n✓ Training complete. Plots saved to: {PLOTS_DIR}")


if __name__ == "__main__":
    train_and_save()