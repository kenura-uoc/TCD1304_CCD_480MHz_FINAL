#!/usr/bin/env python3
"""
simulation_combined.py — Physics-Complete Mixed Chl-a + Chl-b Simulation
=========================================================================
Sections
--------
A. IDEAL SIMULATION (synthetic data)
   - Models 5 physical effects in a mixed-pigment solution
   - Trains dual-output PLS   (benchmark)
   - Trains dual-output 1D CNN (upper bound)

B. REAL DATA TESTING
   - Loads actual Chl-a and Chl-b measurements from CSV/data files
   - Applies the same preprocessing + cross-channel concatenation
   - Evaluates a dual-output PLS on the real data (cross-validated)

C. CROSS-EXCITATION VARIABILITY STUDY
   - Shows how the cross-excitation coefficient changes with:
     solvent composition, pigment degradation, concentration regime

Physical Effects Modelled in Synthetic Spectra:
───────────────────────────────────────────────
1. SPECTRAL SUPERPOSITION — both analytes emit simultaneously
2. CROSS-EXCITATION      — 405nm partially excites Chl-b (15%);
                           450nm partially excites Chl-a (25%)
3. MUTUAL IFE            — each analyte absorbs the other's emission
4. TOTAL QUENCHING       — combined density suppresses fluorescence yield
5. INDEPENDENT RED SHIFTS— each analyte's peak shifts by its own [C]

References:
  - Lakowicz (2006) Principles of Fluorescence Spectroscopy, 3rd ed.
  - Maxwell & Johnson (2000) J Exp Bot 51(345):659-668 [cross-excitation]
  - Murchie & Lawson (2013) J Exp Bot 64(13):3983-3998 [IFE discussion]
  - Porra et al. (2002) BBA Bioenergetics [Chl extinction coefficients]
  - Lichtenthaler & Buschmann (2001) Curr Prot Food Anal Chem [calibration]
  - Lakowicz (2006) Ch.2 for quenching mechanisms
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import shutil
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import KFold
from sklearn.metrics import r2_score, mean_squared_error
import warnings
warnings.filterwarnings("ignore")

# PyTorch: optional — CNN section skipped gracefully if not available
try:
    import torch
    import torch.nn as nn
    from torch.utils.data import DataLoader, TensorDataset
    TORCH_AVAILABLE = True
except ImportError:
    TORCH_AVAILABLE = False
    print("  [WARN] PyTorch not found — CNN section will be skipped.")

np.random.seed(42)

# ─────────────────────────────────────────────────────────────
# PATHS
# ─────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
OUT_DIR    = SCRIPT_DIR / "simulation_results"
OUT_DIR.mkdir(exist_ok=True)
IMG_DIR    = SCRIPT_DIR.parent / "kenura_lab_report" / "images"
IMG_DIR.mkdir(exist_ok=True)

# Real-data paths
DATA_DIR_A = SCRIPT_DIR / "data" / "chl_a"
DATA_DIR_B = SCRIPT_DIR / "data" / "chl_b"
BG_DIR     = SCRIPT_DIR / "data" / "background_data"
CSV_A      = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B      = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"

# ─────────────────────────────────────────────────────────────
# SENSOR / PIPELINE CONSTANTS
# ─────────────────────────────────────────────────────────────
TOTAL_PIXELS  = 3694
ROI_START     = 1300
ROI_END       = 3500
ROI_LEN       = ROI_END - ROI_START
N_SAMPLES     = 300
N_SPLITS      = 5
PLS_MAX_COMP  = 12
INTEGRATION_MS   = 300
NOISE_FRAMES     = 128
DARK_LEVEL       = 200
READ_NOISE_STD   = 15

# ─────────────────────────────────────────────────────────────
# ANALYTE SPECTRAL PARAMETERS  (calibrated to match single-analyte sim)
# ─────────────────────────────────────────────────────────────
PARAMS = {
    "chla": dict(base_peak=1950, red_shift=12.0, fwhm_base=180,
                 fwhm_growth=6.0, alpha=0.12, max_signal=3200),
    "chlb": dict(base_peak=2200, red_shift=40.0, fwhm_base=220,
                 fwhm_growth=8.0, alpha=0.18, max_signal=3200),
}

# Cross-excitation efficiency matrix
# [Lakowicz 2006; Maxwell & Johnson 2000 — chlorophyll absorption at 405/450nm]
EXCITATION = {
    "laser_405": {"chla": 1.00, "chlb": 0.15},   # 405nm: 100% Chl-a, 15% Chl-b
    "laser_450": {"chlb": 1.00, "chla": 0.25},   # 450nm: 100% Chl-b, 25% Chl-a
}

# Mutual IFE absorption coefficients per analyte emission band
# [Porra et al. 2002; Lichtenthaler & Buschmann 2001]
CROSS_IFE = {
    "chlb_on_chla_emission": 0.018,   # Chl-b absorbs Chl-a emission (~680nm)
    "chla_on_chlb_emission": 0.008,   # Chl-a absorbs Chl-b emission (~665nm)
}

# Total-concentration quenching [Lakowicz 2006, Ch.2 — collisional quenching]
QUENCH_COEFF = 0.004    # fractional loss per mg/L total pigment


# ═════════════════════════════════════════════════════════════
# SECTION A: SYNTHETIC SPECTRUM GENERATOR
# ═════════════════════════════════════════════════════════════
def _clean_gaussian(x, conc, analyte, excitation_efficiency=1.0):
    p        = PARAMS[analyte]
    peak_pos = p["base_peak"] + conc * p["red_shift"]
    sigma    = (p["fwhm_base"] + conc * p["fwhm_growth"]) / 2.355
    fluor_I  = p["max_signal"] * (1.0 - np.exp(-p["alpha"] * conc))
    signal   = fluor_I / 300.0 * INTEGRATION_MS * excitation_efficiency
    return signal * np.exp(-0.5 * ((x - peak_pos) / sigma) ** 2)


def generate_combined_spectrum(ca, cb, laser="laser_405"):
    """
    Generate one CCD frame for a MIXED Chl-a + Chl-b solution.
    All 5 physical effects applied (see module docstring).
    """
    x   = np.arange(TOTAL_PIXELS, dtype=float)
    eff = EXCITATION[laser]

    # Effect 1+2: Superposition + Cross-excitation
    clean_a = _clean_gaussian(x, ca, "chla", eff["chla"])
    clean_b = _clean_gaussian(x, cb, "chlb", eff["chlb"])

    # Effect 3: Mutual IFE (Beer-Lambert attenuation at emission bands)
    clean_a *= np.exp(-CROSS_IFE["chlb_on_chla_emission"] * cb)
    clean_b *= np.exp(-CROSS_IFE["chla_on_chlb_emission"] * ca)

    # Effect 4: Total-concentration quenching
    quench = max(0.0, 1.0 - QUENCH_COEFF * (ca + cb))
    spectrum_clean = (clean_a + clean_b) * quench

    # Effect 6: CCD noise (shot + read + dark, averaged over N frames)
    frames = []
    for _ in range(NOISE_FRAMES):
        frame = spectrum_clean + DARK_LEVEL
        frame += np.random.normal(0, np.sqrt(np.maximum(frame, 1)))
        frame += np.random.normal(0, READ_NOISE_STD, TOTAL_PIXELS)
        frames.append(frame)
    return np.mean(frames, axis=0)


def preprocess_sim(raw):
    """SNV 1st-derivative preprocessing for simulated spectra."""
    roi = raw[ROI_START:ROI_END] / INTEGRATION_MS
    sm  = savgol_filter(roi, 11, 2)
    drv = savgol_filter(sm, 11, 3, deriv=1)
    mu, s = drv.mean(), drv.std()
    return (drv - mu) / (s + 1e-12)


def generate_dataset(n=N_SAMPLES):
    Ca = np.sqrt(np.random.uniform(0.1**2, 10.0**2, n))
    Cb = np.sqrt(np.random.uniform(0.1**2,  4.0**2, n))
    X_405, X_450 = [], []
    for i in range(n):
        X_405.append(preprocess_sim(generate_combined_spectrum(Ca[i], Cb[i], "laser_405")))
        X_450.append(preprocess_sim(generate_combined_spectrum(Ca[i], Cb[i], "laser_450")))
        if (i + 1) % 50 == 0:
            print(f"  Generated {i+1}/{n} samples...")
    X_405 = np.array(X_405)
    X_450 = np.array(X_450)
    X_concat = np.hstack([X_405, X_450])
    Y = np.column_stack([Ca, Cb])
    return X_405, X_450, X_concat, Y, Ca, Cb


# ═════════════════════════════════════════════════════════════
# SECTION B: DUAL-OUTPUT PLS
# ═════════════════════════════════════════════════════════════
def grid_search_pls(X, Y, tag=""):
    best_n, best_rmse = 1, np.inf
    for n in range(1, min(PLS_MAX_COMP + 1, X.shape[0] // N_SPLITS)):
        kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
        preds = np.zeros_like(Y, dtype=float)
        for tr, te in kf.split(X):
            m = PLSRegression(n_components=n)
            m.fit(X[tr], Y[tr])
            preds[te] = m.predict(X[te])
        ea = np.sqrt(mean_squared_error(Y[:, 0], preds[:, 0]))
        eb = np.sqrt(mean_squared_error(Y[:, 1], preds[:, 1]))
        tot = ea + eb
        ra = r2_score(Y[:, 0], preds[:, 0])
        rb = r2_score(Y[:, 1], preds[:, 1])
        print(f"  {tag} n={n:2d}  Chl-a R²={ra:.4f} RMSE={ea:.3f}  |  "
              f"Chl-b R²={rb:.4f} RMSE={eb:.3f}  | ΣRMSe={tot:.3f}")
        if tot < best_rmse:
            best_rmse, best_n = tot, n
    return best_n


def cv_pls(X, Y, n):
    kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
    preds = np.zeros_like(Y, dtype=float)
    for tr, te in kf.split(X):
        m = PLSRegression(n_components=n)
        m.fit(X[tr], Y[tr])
        preds[te] = m.predict(X[te])
    return preds


# ═════════════════════════════════════════════════════════════
# SECTION C: DUAL-OUTPUT 1D CNN
# ═════════════════════════════════════════════════════════════
class DualCNN1D(nn.Module):
    """
    1D CNN with two output neurons — one per analyte.
    Architecture mirrors train_cnn_v1.py but with 2-output head.
    Input: concatenated two-channel spectrum (ROI_LEN × 2 = 4400 features)
    Output: [Chl-a concentration, Chl-b concentration] in mg/L
    """
    def __init__(self, input_len, kernel_sizes=(51, 25, 11), filters=(16, 32, 64)):
        super().__init__()
        k1, k2, k3 = kernel_sizes
        f1, f2, f3 = filters
        self.features = nn.Sequential(
            # Block 1: broad spectral features (~100px window)
            nn.Conv1d(1, f1, kernel_size=k1, padding=k1 // 2),
            nn.BatchNorm1d(f1), nn.ReLU(), nn.MaxPool1d(4), nn.Dropout(0.1),
            # Block 2: medium features (~25px window)
            nn.Conv1d(f1, f2, kernel_size=k2, padding=k2 // 2),
            nn.BatchNorm1d(f2), nn.ReLU(), nn.MaxPool1d(4), nn.Dropout(0.1),
            # Block 3: fine features (~11px window)
            nn.Conv1d(f2, f3, kernel_size=k3, padding=k3 // 2),
            nn.BatchNorm1d(f3), nn.ReLU(),
            nn.AdaptiveAvgPool1d(8),
        )
        self.regressor = nn.Sequential(
            nn.Flatten(),
            nn.Linear(f3 * 8, 128), nn.ReLU(), nn.Dropout(0.2),
            nn.Linear(128, 32),     nn.ReLU(),
            nn.Linear(32, 2),       # ← dual output: [Chl-a, Chl-b]
        )

    def forward(self, x):
        return self.regressor(self.features(x))


def train_cnn_fold(X_tr, Y_tr, X_te, input_len, epochs=80, lr=1e-3):
    model = DualCNN1D(input_len)
    opt   = torch.optim.Adam(model.parameters(), lr=lr)
    loss_fn = nn.MSELoss()
    Xt = torch.FloatTensor(X_tr).unsqueeze(1)
    Yt = torch.FloatTensor(Y_tr)
    ds = TensorDataset(Xt, Yt)
    dl = DataLoader(ds, batch_size=32, shuffle=True)
    model.train()
    for _ in range(epochs):
        for xb, yb in dl:
            opt.zero_grad()
            loss_fn(model(xb), yb).backward()
            opt.step()
    model.eval()
    with torch.no_grad():
        preds = model(torch.FloatTensor(X_te).unsqueeze(1)).numpy()
    return preds


def cv_cnn(X, Y, epochs=80):
    kf = KFold(n_splits=N_SPLITS, shuffle=True, random_state=42)
    preds = np.zeros_like(Y, dtype=float)
    for fold, (tr, te) in enumerate(kf.split(X)):
        print(f"    CNN fold {fold+1}/{N_SPLITS}...", end=" ", flush=True)
        preds[te] = train_cnn_fold(X[tr], Y[tr], X[te], X.shape[1], epochs=epochs)
        ra = r2_score(Y[te, 0], preds[te, 0])
        rb = r2_score(Y[te, 1], preds[te, 1])
        print(f"Chl-a R²={ra:.4f}  Chl-b R²={rb:.4f}")
    return preds


# ═════════════════════════════════════════════════════════════
# SECTION D: REAL DATA LOADING + PREPROCESSING
# ═════════════════════════════════════════════════════════════
BG_MAP = {
    240:  "background-240.csv",  250:  "background-250.csv",
    300:  "background-300.csv",  500:  "background-500.csv",
    1000: "background-1000.csv",
}
BG_AVAIL = sorted(BG_MAP.keys())


def _load_spectrum(path):
    df = pd.read_csv(path)
    return np.mean(df.iloc[:, 1:].values.astype(float), axis=0)


def _load_bg(int_time):
    nearest = min(BG_AVAIL, key=lambda t: abs(t - int_time))
    path = BG_DIR / BG_MAP[nearest]
    return _load_spectrum(path) if path.exists() else None


def als_baseline(y, lam=1e5, p=0.01, niter=10):
    L = len(y)
    D = sparse.diags([1, -2, 1], [0, 1, 2], shape=(L-2, L), format='csc').T
    D = D.T @ D
    w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w, format='csc')
        z = spsolve(W + lam * D, w * y)
        w = p * (y > z) + (1 - p) * (y <= z)
    return z


def preprocess_real(raw, bg, int_time, use_als=False):
    if bg is not None and len(bg) == len(raw):
        raw = raw - bg
    roi = raw[ROI_START:ROI_END]
    if use_als:
        try:
            roi = roi - als_baseline(roi)
        except Exception:
            pass
    roi = roi / int_time
    sm  = savgol_filter(roi, 11, 2)
    drv = savgol_filter(sm, 11, 3, deriv=1)
    mu, s = drv.mean(), drv.std()
    return (drv - mu) / (s + 1e-12)


def load_real_dataset(csv_path, data_dir, use_als=False):
    meta = pd.read_csv(csv_path).dropna(subset=["Sample Con", "integration time(mS)"])
    spectra, concs, idxs = [], [], []
    for i, row in meta.iterrows():
        fpath = data_dir / f"{i+1}.csv"
        if not fpath.exists():
            continue
        raw = _load_spectrum(fpath)
        if len(raw) < ROI_END:
            continue
        bg   = _load_bg(int(row["integration time(mS)"]))
        feat = preprocess_real(raw, bg, row["integration time(mS)"], use_als)
        spectra.append(feat)
        concs.append(float(row["Sample Con"]))
        idxs.append(i)
    return np.array(spectra), np.array(concs), idxs


# ═════════════════════════════════════════════════════════════
# SECTION E: PLOTS
# ═════════════════════════════════════════════════════════════
def _save(fig, name):
    out = OUT_DIR / name
    fig.savefig(out, dpi=150, bbox_inches='tight')
    plt.close(fig)
    shutil.copy(out, IMG_DIR / name)
    print(f"  Saved: {name}")


def plot_pipeline(X_405, X_450, Ca, Cb):
    px = np.arange(ROI_LEN)
    fig, axes = plt.subplots(2, 3, figsize=(16, 8))
    fig.suptitle("Combined Chl-a + Chl-b Mixed Solution — Preprocessing Pipeline",
                 fontsize=12, fontweight='bold')
    c_total = Ca + Cb
    for row, (X, title, cmap) in enumerate([
        (X_405, "405 nm (Mixed)", "Blues"),
        (X_450, "450 nm (Mixed)", "Greens"),
    ]):
        cm   = plt.get_cmap(cmap)
        norm = plt.Normalize(c_total.min(), c_total.max())
        for i in range(len(X)):
            axes[row, 0].plot(px, X[i], color=cm(norm(c_total[i])), alpha=0.3, lw=0.5)
        axes[row, 0].set_title(f"{title} — All Samples (colored by [Ca+Cb])")
        axes[row, 0].set_xlabel("Pixel (ROI)")
        axes[row, 0].set_ylabel("SNV 1st Derivative")

    # Superposition panel
    concs = [(1.0, 0.5), (3.0, 1.0), (6.0, 2.0), (9.0, 3.5)]
    for ca, cb in concs:
        raw = generate_combined_spectrum(ca, cb, "laser_405")
        axes[0, 1].plot(px, raw[ROI_START:ROI_END] / INTEGRATION_MS, label=f"Ca={ca}, Cb={cb}")
    axes[0, 1].axvline(1950-ROI_START, ls='--', color='blue',  alpha=0.4, label='Chl-a peak')
    axes[0, 1].axvline(2200-ROI_START, ls='--', color='green', alpha=0.4, label='Chl-b peak')
    axes[0, 1].set_title("405 nm ROI — Superposed Emission Bands")
    axes[0, 1].legend(fontsize=7)

    # Cross-excitation panel
    for cb in [0.1, 0.5, 1.0, 2.0, 3.5]:
        raw = generate_combined_spectrum(3.0, cb, "laser_405")
        axes[1, 1].plot(px, raw[ROI_START:ROI_END] / INTEGRATION_MS, label=f"Cb={cb}")
    axes[1, 1].set_title("Cross-Excitation (405nm, Ca=3 fixed, Cb varies)")
    axes[1, 1].legend(fontsize=7)

    # Mutual IFE panel
    for cb in [0.1, 1.0, 2.0, 3.5]:
        raw = generate_combined_spectrum(3.0, cb, "laser_450")
        axes[0, 2].plot(px, raw[ROI_START:ROI_END] / INTEGRATION_MS, label=f"Cb={cb}")
    axes[0, 2].set_title("Mutual IFE + Quenching (450nm, Ca=3 fixed, Cb varies)")
    axes[0, 2].legend(fontsize=7)

    # Quenching curve
    totals = np.linspace(0, 14, 100)
    axes[1, 2].plot(totals, np.maximum(0, 1 - QUENCH_COEFF * totals), 'r-', lw=2)
    axes[1, 2].set_title("Total-Conc. Quenching Factor")
    axes[1, 2].set_xlabel("Total [Ca + Cb] mg/L")
    axes[1, 2].set_ylabel("Fluorescence Yield Factor")
    plt.tight_layout()
    _save(fig, "combined_sim_pipeline.png")


def plot_heatmaps(X_405, X_450, Ca, Cb):
    fig, axes = plt.subplots(1, 2, figsize=(15, 7))
    sort_idx = np.argsort(Ca)
    for ax, X, laser, cmap in [
        (axes[0], X_405, "405 nm", "Blues"),
        (axes[1], X_450, "450 nm", "Greens"),
    ]:
        im = ax.imshow(X[sort_idx], aspect="auto", origin="lower", cmap=cmap,
                       extent=[0, ROI_LEN, Ca[sort_idx[0]], Ca[sort_idx[-1]]])
        ax.set_title(f"Mixed Solution Heatmap — {laser} Excitation\n"
                     f"Sorted by [Chl-a] (Chl-b varying independently)")
        ax.set_xlabel("Pixel (ROI)")
        ax.set_ylabel("[Chl-a] mg/L")
        plt.colorbar(im, ax=ax, label="SNV 1st Derivative")
    fig.suptitle("Ideal Simulation: Two-Channel Mixed-Solution Dataset", fontweight='bold')
    plt.tight_layout()
    _save(fig, "combined_sim_heatmap.png")


def plot_eval(Y, preds, label, filename, note=""):
    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    for i, (name, color) in enumerate(zip(["Chl-a", "Chl-b"], ["royalblue", "forestgreen"])):
        y_true, y_pred = Y[:, i], preds[:, i]
        r2   = r2_score(y_true, y_pred)
        rmse = np.sqrt(mean_squared_error(y_true, y_pred))
        lims = [min(y_true.min(), y_pred.min()) * 0.9,
                max(y_true.max(), y_pred.max()) * 1.05]
        axes[i].scatter(y_true, y_pred, alpha=0.5, color=color, edgecolors='k', lw=0.3, s=30)
        axes[i].plot(lims, lims, 'k--', lw=1)
        axes[i].set_xlim(lims); axes[i].set_ylim(lims)
        axes[i].set_xlabel(f"Actual [{name}] mg/L")
        axes[i].set_ylabel(f"Predicted [{name}] mg/L")
        axes[i].set_title(f"{label} — {name}\n$R^2$={r2:.4f}  RMSE={rmse:.3f} mg/L")
    fig.suptitle(f"{label} ({N_SPLITS}-fold CV){note}", fontsize=12, fontweight='bold')
    plt.tight_layout()
    _save(fig, filename)
    return (r2_score(Y[:, 0], preds[:, 0]),
            r2_score(Y[:, 1], preds[:, 1]),
            np.sqrt(mean_squared_error(Y[:, 0], preds[:, 0])),
            np.sqrt(mean_squared_error(Y[:, 1], preds[:, 1])))


def plot_cross_excitation_variability():
    """
    Shows how cross-excitation coefficient changes with key experimental variables.
    Cross-excitation at a given laser wavelength is fixed by the molar absorption
    spectrum of each pigment. However it appears variable in practice due to:
      1. Solvent (acetone vs. DMSO shifts Soret peaks ±5nm)
      2. Pigment degradation (pheophytin has red-shifted Soret)
      3. Concentration-dependent aggregate formation
      4. Temperature broadening of absorption peaks
    This plot quantifies the sensitivity of the PREDICTED spectrum to
    plausible variation in ε_cross.
    """
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    ca_fixed, cb_fixed = 3.0, 1.0
    px = np.arange(ROI_LEN)

    # 1. Vary ε_cross (405nm→Chl-b) over plausible solvent range [0.08, 0.25]
    cross_vals = np.linspace(0.05, 0.30, 6)
    for eps in cross_vals:
        eff_old = EXCITATION["laser_405"]["chlb"]
        EXCITATION["laser_405"]["chlb"] = eps
        raw = generate_combined_spectrum(ca_fixed, cb_fixed, "laser_405")
        axes[0].plot(px, raw[ROI_START:ROI_END] / INTEGRATION_MS,
                     label=f"ε={eps:.2f}", alpha=0.8)
        EXCITATION["laser_405"]["chlb"] = eff_old
    axes[0].set_title("Sensitivity to ε_cross (405nm→Chl-b)\n"
                       "Actual value fixed by absorption spectrum at 405nm\n"
                       "[Lakowicz 2006; Maxwell & Johnson 2000]")
    axes[0].set_xlabel("Pixel (ROI)"); axes[0].set_ylabel("Counts / ms")
    axes[0].legend(fontsize=7)

    # 2. Degradation scenario: pheophytin formation shifts Soret ~15nm red
    #    Approximate by shifting Chl-b base_peak by +30 pixels
    shifts = [0, 15, 30, 50]
    for shift in shifts:
        orig = PARAMS["chlb"]["base_peak"]
        PARAMS["chlb"]["base_peak"] = orig + shift
        raw = generate_combined_spectrum(ca_fixed, cb_fixed, "laser_405")
        axes[1].plot(px, raw[ROI_START:ROI_END] / INTEGRATION_MS,
                     label=f"Δpeak={shift}px", alpha=0.8)
        PARAMS["chlb"]["base_peak"] = orig
    axes[1].set_title("Effect of Pigment Degradation\n"
                       "(Chl-b peak shift → pheophytin)\n"
                       "[Murchie & Lawson 2013]")
    axes[1].set_xlabel("Pixel (ROI)")
    axes[1].legend(fontsize=7)

    # 3. How to MEASURE cross-excitation experimentally:
    #    Pure Chl-a solution → illuminate with 450nm → measure Chl-a contribution
    #    Ratio of 450nm/405nm signals = ε_cross for Chl-a
    ca_range = np.linspace(0.5, 8.0, 20)
    ratio_405_450 = []
    for ca in ca_range:
        raw_405 = generate_combined_spectrum(ca, 0, "laser_405")
        raw_450 = generate_combined_spectrum(ca, 0, "laser_450")
        sig_405 = np.max(raw_405[ROI_START:ROI_END])
        sig_450 = np.max(raw_450[ROI_START:ROI_END])
        ratio_405_450.append(sig_450 / (sig_405 + 1e-9))
    axes[2].plot(ca_range, ratio_405_450, 'royalblue', lw=2, marker='o', ms=4)
    axes[2].axhline(EXCITATION["laser_450"]["chla"], ls='--', color='red',
                    label=f"Nominal ε = {EXCITATION['laser_450']['chla']}")
    axes[2].set_title("Experimental Measurement of ε_cross\n"
                       "Pure Chl-a: ratio of 450nm/405nm peak signals\n"
                       "→ gives ε_cross for Chl-a under 450nm excitation")
    axes[2].set_xlabel("[Chl-a] mg/L")
    axes[2].set_ylabel("Signal ratio I_450 / I_405")
    axes[2].legend(fontsize=8)

    fig.suptitle("Cross-Excitation: Physical Origin, Variability, and Measurement Method",
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    _save(fig, "combined_cross_excitation_variability.png")


def plot_real_data_comparison(Y_real, preds_real, Y_sim, preds_sim, best_n_real):
    """Side-by-side: ideal simulation vs real data performance."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    datasets = [
        ("Ideal Simulation", Y_sim, preds_sim, "steelblue"),
        ("Real Data",        Y_real, preds_real, "coral"),
    ]
    for col, (label, Y, preds, color) in enumerate(datasets):
        for row, name in enumerate(["Chl-a", "Chl-b"]):
            ax = axes[row, col]
            y_true, y_pred = Y[:, row], preds[:, row]
            r2   = r2_score(y_true, y_pred)
            rmse = np.sqrt(mean_squared_error(y_true, y_pred))
            lims = [min(y_true.min(), y_pred.min()) * 0.85,
                    max(y_true.max(), y_pred.max()) * 1.08]
            ax.scatter(y_true, y_pred, alpha=0.6, color=color, edgecolors='k', lw=0.3, s=35)
            ax.plot(lims, lims, 'k--', lw=1)
            ax.set_xlim(lims); ax.set_ylim(lims)
            ax.set_xlabel(f"Actual [{name}] mg/L")
            ax.set_ylabel(f"Predicted [{name}] mg/L")
            ax.set_title(f"{label} — {name}\n$R^2$={r2:.4f}  RMSE={rmse:.3f} mg/L")
    fig.suptitle(f"Phase 2 Combined Model: Ideal Simulation vs Real Data\n"
                 f"Real data: n={Y_real.shape[0]} samples, PLS n_components={best_n_real}, "
                 f"{N_SPLITS}-fold CV",
                 fontsize=12, fontweight='bold')
    plt.tight_layout()
    _save(fig, "combined_sim_vs_real_comparison.png")


# ═════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("=" * 70)
    print("  PHASE 2: MIXED Chl-a + Chl-b — IDEAL SIMULATION + REAL DATA TEST")
    print("=" * 70)

    # ── A: Generate synthetic dataset ────────────────────────
    print(f"\n[A] Generating {N_SAMPLES} synthetic mixed-solution spectra...")
    X_405, X_450, X_concat, Y_sim, Ca, Cb = generate_dataset(N_SAMPLES)
    print(f"  Synthetic X shape: {X_concat.shape},  Y shape: {Y_sim.shape}")

    # ── A: Plots ─────────────────────────────────────────────
    print("\n[A] Diagnostic plots...")
    plot_pipeline(X_405, X_450, Ca, Cb)
    plot_heatmaps(X_405, X_450, Ca, Cb)

    # ── B: PLS ───────────────────────────────────────────────
    print("\n[B] Dual-output PLS on synthetic data...")
    best_n_sim = grid_search_pls(X_concat, Y_sim, tag="[SIM]")
    preds_pls_sim = cv_pls(X_concat, Y_sim, best_n_sim)
    r2a_p, r2b_p, rmse_a_p, rmse_b_p = plot_eval(
        Y_sim, preds_pls_sim, "Dual-Output PLS (Ideal Simulation)",
        "combined_dual_pls_evaluation.png",
        note=f"  n_components={best_n_sim}"
    )
    print(f"  PLS SIM: Chl-a R²={r2a_p:.4f} RMSE={rmse_a_p:.3f}  |"
          f"  Chl-b R²={r2b_p:.4f} RMSE={rmse_b_p:.3f}")

    # ── C: 1D CNN ────────────────────────────────────────────
    r2a_c = r2b_c = rmse_a_c = rmse_b_c = None
    preds_cnn_sim = None
    if TORCH_AVAILABLE:
        print("\n[C] Dual-output 1D CNN on synthetic data...")
        preds_cnn_sim = cv_cnn(X_concat, Y_sim, epochs=100)
        r2a_c, r2b_c, rmse_a_c, rmse_b_c = plot_eval(
            Y_sim, preds_cnn_sim,
            "Dual-Output 1D CNN (Ideal Simulation)",
            "combined_dual_cnn_evaluation.png",
        )
        print(f"  CNN SIM: Chl-a R²={r2a_c:.4f} RMSE={rmse_a_c:.3f}  |"
              f"  Chl-b R²={r2b_c:.4f} RMSE={rmse_b_c:.3f}")
    else:
        print("\n[C] Skipping CNN — PyTorch not installed.")
        print("    Install with: pip install torch")

    # ── D: Cross-excitation variability ─────────────────────
    print("\n[D] Cross-excitation variability study...")
    plot_cross_excitation_variability()

    # ── E: Real data ─────────────────────────────────────────
    print("\n[E] Loading real data for combined pipeline test...")
    X_real_a, y_real_a, _ = load_real_dataset(CSV_A, DATA_DIR_A, use_als=False)
    X_real_b, y_real_b, _ = load_real_dataset(CSV_B, DATA_DIR_B, use_als=True)
    min_n = min(len(X_real_a), len(X_real_b))

    if min_n < N_SPLITS + 1:
        print(f"  [WARN] Only {min_n} overlapping real samples — skipping real-data evaluation.")
    else:
        X_real_concat = np.hstack([X_real_a[:min_n], X_real_b[:min_n]])
        Y_real = np.column_stack([y_real_a[:min_n], y_real_b[:min_n]])
        print(f"  Real data: {min_n} samples, X shape={X_real_concat.shape}")
        print(f"  Chl-a: {Y_real[:, 0].min():.2f}–{Y_real[:, 0].max():.2f} mg/L")
        print(f"  Chl-b: {Y_real[:, 1].min():.2f}–{Y_real[:, 1].max():.2f} mg/L")

        print("\n  Grid-searching PLS on real data...")
        best_n_real = grid_search_pls(X_real_concat, Y_real, tag="[REAL]")
        preds_pls_real = cv_pls(X_real_concat, Y_real, best_n_real)
        r2a_r, r2b_r, rmse_a_r, rmse_b_r = plot_eval(
            Y_real, preds_pls_real,
            "Dual-Output PLS (Real Data)",
            "combined_real_pls_evaluation.png",
            note=f"  n_components={best_n_real}, n={min_n} samples"
        )
        print(f"  PLS REAL: Chl-a R²={r2a_r:.4f} RMSE={rmse_a_r:.3f}  |"
              f"  Chl-b R²={r2b_r:.4f} RMSE={rmse_b_r:.3f}")

        # Side-by-side comparison plot
        plot_real_data_comparison(Y_real, preds_pls_real, Y_sim, preds_pls_sim, best_n_real)

    # ── Summary ──────────────────────────────────────────────
    print("\n" + "=" * 70)
    print("  RESULTS SUMMARY")
    print("=" * 70)
    print(f"  {'Model':<35}  {'Chl-a R²':>8}  {'RMSE_a':>8}  {'Chl-b R²':>8}  {'RMSE_b':>8}")
    print(f"  {'-'*70}")
    print(f"  {'PLS — Ideal Simulation':<35}  {r2a_p:>8.4f}  {rmse_a_p:>8.3f}"
          f"  {r2b_p:>8.4f}  {rmse_b_p:>8.3f}")
    if r2a_c is not None:
        print(f"  {'1D CNN — Ideal Simulation':<35}  {r2a_c:>8.4f}  {rmse_a_c:>8.3f}"
              f"  {r2b_c:>8.4f}  {rmse_b_c:>8.3f}")
    if min_n >= N_SPLITS + 1:
        print(f"  {'PLS — Real Data':<35}  {r2a_r:>8.4f}  {rmse_a_r:>8.3f}"
              f"  {r2b_r:>8.4f}  {rmse_b_r:>8.3f}")
    print("\nAll outputs saved to simulation_results/ and kenura_lab_report/images/")
