#!/usr/bin/env python3
"""
Generate real-data visualizations matching the simulation ideal case format.
Produces for each analyte (chla, chlb):
  1. real_spectra_pipeline_{analyte}.png  - 4-panel: raw, ROI+BG-sub, 1st deriv, SNV
  2. real_spectra_heatmap_{analyte}.png   - 3-panel heatmap sorted by concentration
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve

# ============================================================
# CONFIGURATION (match train_models_v2.py)
# ============================================================
SCRIPT_DIR      = Path(__file__).parent.resolve()
BACKGROUND_DIR  = SCRIPT_DIR / "data/background_data"
OUTPUT_DIR      = SCRIPT_DIR.parent / "kenura_lab_report" / "images"
OUTPUT_DIR.mkdir(exist_ok=True)

ROI_START        = 1300
ROI_END          = 3500
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY   = 2
SG_DERIV_WINDOW  = 11
SG_DERIV_POLY    = 3
SG_DERIV_ORDER   = 1

BACKGROUND_MAP = {
    240: "background-240.csv", 250: "background-250.csv",
    300: "background-300.csv", 500: "background-500.csv",
    1000: "background-1000.csv"
}
# For bg times not in map, use the nearest available
BG_TIMES_AVAILABLE = [240, 250, 300, 500, 1000]

ANALYTE_CONFIG = {
    "chla": {
        "data_dir": SCRIPT_DIR / "data/chl_a",
        "csv":      SCRIPT_DIR / "data/real_data/chla_data.csv",
        "label":    "Chl-a",
        "use_als":  False,
    },
    "chlb": {
        "data_dir": SCRIPT_DIR / "data/chl_b",
        "csv":      SCRIPT_DIR / "data/real_data/chlb_data.csv",
        "label":    "Chl-b",
        "use_als":  True,   # Chl-b training uses ALS baseline correction
    },
}

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def load_spectrum(path):
    df = pd.read_csv(path)
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def load_bg(int_time):
    """Load closest background file for a given integration time."""
    nearest = min(BG_TIMES_AVAILABLE, key=lambda t: abs(t - int_time))
    fname = BACKGROUND_MAP.get(nearest)
    if not fname:
        return None
    path = BACKGROUND_DIR / fname
    if not path.exists():
        return None
    return load_spectrum(path)

def snv_normalize(spectrum):
    mean, std = np.mean(spectrum), np.std(spectrum)
    if std < 1e-12:
        return spectrum - mean
    return (spectrum - mean) / std

def als_baseline(y, lam=1e5, p=0.01, niter=10):
    """Asymmetric Least Squares baseline correction."""
    L = len(y)
    D = sparse.diags([1, -2, 1], [0, 1, 2], shape=(L - 2, L), format='csc')
    D = D.T @ D
    w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w, 0, shape=(L, L), format='csc')
        Z = W + lam * D
        z = spsolve(Z, w * y)
        w = p * (y > z) + (1 - p) * (y <= z)
    return z

# ============================================================
# MAIN VISUALIZATION FUNCTION
# ============================================================
def generate_real_data_plots(analyte="chla"):
    cfg = ANALYTE_CONFIG[analyte]
    data_dir = cfg["data_dir"]
    csv_path = cfg["csv"]
    label    = cfg["label"]
    use_als  = cfg["use_als"]

    print(f"\n=== Processing {label} ===")

    if not csv_path.exists():
        print(f"  [WARN] Metadata CSV not found: {csv_path}")
        return

    meta = pd.read_csv(csv_path)
    # Only use rows with valid concentration and integration time
    meta = meta.dropna(subset=["Sample Con", "integration time(mS)"])

    concentrations = meta["Sample Con"].values
    int_times      = meta["integration time(mS)"].values

    raw_spectra, roi_spectra, deriv_spectra, snv_spectra = [], [], [], []
    valid_concs, valid_int_times = [], []

    for i, (conc, it) in enumerate(zip(concentrations, int_times)):
        fpath = data_dir / f"{i+1}.csv"
        if not fpath.exists():
            continue

        raw = load_spectrum(fpath)
        if len(raw) < ROI_END:
            continue

        # Background subtraction using nearest bg
        bg = load_bg(int(it))
        if bg is not None and len(bg) == len(raw):
            raw_diff = raw - bg
        else:
            raw_diff = raw.copy()

        # ROI crop
        roi = raw_diff[ROI_START:ROI_END]

        # Optional ALS baseline correction (for Chl-b)
        if use_als:
            try:
                baseline = als_baseline(roi)
                roi = roi - baseline
            except Exception:
                pass  # Skip if ALS fails

        # Normalize by integration time
        roi_norm = roi / it

        # Smooth + 1st derivative
        smoothed = savgol_filter(roi_norm, SG_SMOOTH_WINDOW, SG_SMOOTH_POLY)
        deriv    = savgol_filter(smoothed, SG_DERIV_WINDOW, SG_DERIV_POLY, deriv=SG_DERIV_ORDER)

        # SNV
        snv = snv_normalize(deriv)

        raw_spectra.append(raw)
        roi_spectra.append(roi_norm)
        deriv_spectra.append(deriv)
        snv_spectra.append(snv)
        valid_concs.append(conc)
        valid_int_times.append(it)

    if not raw_spectra:
        print(f"  [WARN] No valid spectra loaded for {label}")
        return

    raw_spectra   = np.array(raw_spectra)
    roi_spectra   = np.array(roi_spectra)
    deriv_spectra = np.array(deriv_spectra)
    snv_spectra   = np.array(snv_spectra)
    valid_concs   = np.array(valid_concs)

    # Sort by concentration
    sort_idx   = np.argsort(valid_concs)
    raw_sorted  = raw_spectra[sort_idx]
    roi_sorted  = roi_spectra[sort_idx]
    deriv_sorted = deriv_spectra[sort_idx]
    snv_sorted  = snv_spectra[sort_idx]
    conc_sorted = valid_concs[sort_idx]

    print(f"  Loaded {len(valid_concs)} spectra, conc range: "
          f"{valid_concs.min():.2f} – {valid_concs.max():.2f} ppm")

    roi_pixels = np.arange(ROI_START, ROI_END)
    norm = plt.Normalize(conc_sorted.min(), conc_sorted.max())
    cmap = cm.plasma

    # ── Pipeline Plot ───────────────────────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"Preprocessing Pipeline — Real CCD Data ({label})",
                 fontsize=14, fontweight='bold')

    ax = axes[0, 0]
    for j in range(len(raw_sorted)):
        ax.plot(raw_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.4, lw=0.5)
    ax.axvline(ROI_START, color='lime', ls='--', alpha=0.7, label='ROI start')
    ax.axvline(ROI_END,   color='lime', ls='--', alpha=0.7, label='ROI end')
    ax.axvspan(ROI_START, ROI_END, alpha=0.05, color='lime')
    ax.set_xlabel("Pixel"); ax.set_ylabel("ADC Counts")
    ax.set_title(f"(a) Raw CCD Spectra ({len(raw_sorted)} samples, 3694 px)")
    ax.legend(fontsize=7)

    ax = axes[0, 1]
    for j in range(len(roi_sorted)):
        ax.plot(roi_pixels, roi_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.4, lw=0.5)
    ax.set_xlabel("Pixel"); ax.set_ylabel("Norm. Intensity")
    als_note = " + ALS Baseline" if use_als else ""
    ax.set_title(f"(b) ROI [1300–3500] — BG Subtracted + Norm{als_note}")

    ax = axes[1, 0]
    for j in range(len(deriv_sorted)):
        ax.plot(roi_pixels, deriv_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.4, lw=0.5)
    ax.set_xlabel("Pixel"); ax.set_ylabel("1st Derivative")
    ax.set_title("(c) Savitzky-Golay 1st Derivative")

    ax = axes[1, 1]
    for j in range(len(snv_sorted)):
        ax.plot(roi_pixels, snv_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.4, lw=0.5)
    ax.set_xlabel("Pixel"); ax.set_ylabel("SNV")
    ax.set_title("(d) SNV Normalized (Model Input)")

    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=axes.ravel().tolist(), shrink=0.6, pad=0.02)
    cbar.set_label("Concentration (ppm)", fontsize=10)

    plt.tight_layout(rect=[0, 0, 0.92, 0.95])
    out_pipeline = OUTPUT_DIR / f"real_spectra_pipeline_{analyte}.png"
    fig.savefig(out_pipeline, dpi=150, bbox_inches='tight')
    print(f"  Saved: {out_pipeline.name}")
    plt.close()

    # ── Heatmap Plot ────────────────────────────────────────────
    n = len(roi_sorted)
    tick_positions = np.linspace(0, n - 1, min(8, n)).astype(int)

    fig, axes = plt.subplots(1, 3, figsize=(16, 6))
    fig.suptitle(f"Dataset Heatmap — Real CCD Data ({n} {label} Samples, sorted by conc.)",
                 fontsize=13, fontweight='bold')

    # ROI spectra
    ax = axes[0]
    im1 = ax.imshow(roi_sorted, aspect='auto', cmap='viridis',
                    extent=[ROI_START, ROI_END, n, 0])
    ax.set_xlabel("Pixel"); ax.set_ylabel("Record # (sorted by conc.)")
    ax.set_title("ROI Spectra (BG-sub + norm)")
    fig.colorbar(im1, ax=ax, shrink=0.7).set_label("Intensity")
    ax2 = ax.twinx()
    ax2.set_ylim(n, 0); ax2.set_yticks(tick_positions)
    ax2.set_yticklabels([f"{conc_sorted[p]:.2f}" for p in tick_positions], fontsize=7)
    ax2.set_ylabel("Concentration (ppm)", fontsize=8)

    # 1st derivative
    ax = axes[1]
    vmax_d = np.percentile(np.abs(deriv_sorted), 98)
    im2 = ax.imshow(deriv_sorted, aspect='auto', cmap='RdBu_r',
                    vmin=-vmax_d, vmax=vmax_d, extent=[ROI_START, ROI_END, n, 0])
    ax.set_xlabel("Pixel"); ax.set_ylabel("Record #")
    ax.set_title("1st Derivative")
    fig.colorbar(im2, ax=ax, shrink=0.7).set_label("d/dx")
    ax3 = ax.twinx()
    ax3.set_ylim(n, 0); ax3.set_yticks(tick_positions)
    ax3.set_yticklabels([f"{conc_sorted[p]:.2f}" for p in tick_positions], fontsize=7)
    ax3.set_ylabel("Concentration (ppm)", fontsize=8)

    # SNV
    ax = axes[2]
    vmax_s = np.percentile(np.abs(snv_sorted), 98)
    im3 = ax.imshow(snv_sorted, aspect='auto', cmap='RdBu_r',
                    vmin=-vmax_s, vmax=vmax_s, extent=[ROI_START, ROI_END, n, 0])
    ax.set_xlabel("Pixel"); ax.set_ylabel("Record #")
    ax.set_title("SNV (Model Input)")
    fig.colorbar(im3, ax=ax, shrink=0.7).set_label("SNV")
    ax4 = ax.twinx()
    ax4.set_ylim(n, 0); ax4.set_yticks(tick_positions)
    ax4.set_yticklabels([f"{conc_sorted[p]:.2f}" for p in tick_positions], fontsize=7)
    ax4.set_ylabel("Concentration (ppm)", fontsize=8)

    plt.tight_layout(rect=[0, 0, 1, 0.93])
    out_heatmap = OUTPUT_DIR / f"real_spectra_heatmap_{analyte}.png"
    fig.savefig(out_heatmap, dpi=150, bbox_inches='tight')
    print(f"  Saved: {out_heatmap.name}")
    plt.close()


# ============================================================
# MAIN
# ============================================================
if __name__ == "__main__":
    generate_real_data_plots("chla")
    generate_real_data_plots("chlb")

    # Keep backward-compatible copies for the original filenames
    import shutil
    src_p = OUTPUT_DIR / "real_spectra_pipeline_chla.png"
    src_h = OUTPUT_DIR / "real_spectra_heatmap_chla.png"
    if src_p.exists():
        shutil.copy(src_p, OUTPUT_DIR / "real_spectra_pipeline.png")
    if src_h.exists():
        shutil.copy(src_h, OUTPUT_DIR / "real_spectra_heatmap.png")
    print("\nDone  — all Real Data plots written to kenura_lab_report/images/")
