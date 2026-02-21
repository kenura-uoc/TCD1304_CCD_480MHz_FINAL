#!/usr/bin/env python3
"""
Generate real-data visualizations matching the simulation ideal case format.
Produces:
  1. real_spectra_pipeline.png  — 4-panel: raw, ROI+BG-sub, 1st deriv, SNV
  2. real_spectra_heatmap.png   — 3-panel heatmap sorted by concentration
For side-by-side comparison with the simulation ideal case plots.
"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path
from scipy.signal import savgol_filter

# ============================================================
# CONFIGURATION (match train_models_v2.py)
# ============================================================
SCRIPT_DIR      = Path(__file__).parent.resolve()
DATA_DIR_A      = SCRIPT_DIR / "data/chl_a"
BACKGROUND_DIR  = SCRIPT_DIR / "data/background_data"
CSV_A           = SCRIPT_DIR / "data/real_data/chla_data.csv"
OUTPUT_DIR      = SCRIPT_DIR.parent / "kenura_lab_report" / "images"
OUTPUT_DIR.mkdir(exist_ok=True)

ROI_START        = 1300
ROI_END          = 3200
SG_DERIV_WINDOW  = 11
SG_DERIV_POLY    = 3
SG_DERIV_ORDER   = 1

BACKGROUND_MAP = {
    240: "background-240.csv", 250: "background-250.csv",
    300: "background-300.csv", 500: "background-500.csv",
    1000: "background-1000.csv"
}
CLOSEST_BG = {580: 500, 700: 500}

# ============================================================
# DATA LOADING (match train_models_v2.py)
# ============================================================
def load_spectrum(path):
    df = pd.read_csv(path)
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def load_bg(int_time):
    time_key = CLOSEST_BG.get(int_time, int_time)
    fname = BACKGROUND_MAP.get(time_key)
    if not fname:
        return None
    path = BACKGROUND_DIR / fname
    if not path.exists():
        return None
    return load_spectrum(path)

def snv_normalize(spectrum):
    """Standard Normal Variate"""
    mean = np.mean(spectrum)
    std = np.std(spectrum)
    if std < 1e-12:
        return spectrum - mean
    return (spectrum - mean) / std

# ============================================================
# LOAD AND PREPROCESS ALL CHL-A DATA
# ============================================================
meta = pd.read_csv(CSV_A)
concentrations = meta["Sample Con"].values
int_times = meta["integration time(mS)"].values

raw_spectra = []
roi_spectra = []
deriv_spectra = []
snv_spectra = []
valid_concs = []
valid_int_times = []

for i, (conc, it) in enumerate(zip(concentrations, int_times)):
    fpath = DATA_DIR_A / f"{i+1}.csv"
    if not fpath.exists():
        continue
    
    # Raw spectrum (full 3694 pixels)
    raw = load_spectrum(fpath)
    if len(raw) < ROI_END:
        continue
    
    # Background subtraction
    bg = load_bg(int(it))
    if bg is not None and len(bg) == len(raw):
        raw_bg = raw - bg
    else:
        raw_bg = raw.copy()
    
    # ROI crop
    roi = raw_bg[ROI_START:ROI_END]
    
    # Normalize by integration time for comparability
    roi_norm = roi / it
    
    # 1st derivative (Savitzky-Golay)
    deriv = savgol_filter(roi_norm, SG_DERIV_WINDOW, SG_DERIV_POLY, deriv=SG_DERIV_ORDER)
    
    # SNV normalization
    snv = snv_normalize(deriv)
    
    raw_spectra.append(raw)
    roi_spectra.append(roi_norm)
    deriv_spectra.append(deriv)
    snv_spectra.append(snv)
    valid_concs.append(conc)
    valid_int_times.append(it)

raw_spectra = np.array(raw_spectra)
roi_spectra = np.array(roi_spectra)
deriv_spectra = np.array(deriv_spectra)
snv_spectra = np.array(snv_spectra)
valid_concs = np.array(valid_concs)
valid_int_times = np.array(valid_int_times)

# Sort by concentration for visualization
sort_idx = np.argsort(valid_concs)
raw_sorted = raw_spectra[sort_idx]
roi_sorted = roi_spectra[sort_idx]
deriv_sorted = deriv_spectra[sort_idx]
snv_sorted = snv_spectra[sort_idx]
conc_sorted = valid_concs[sort_idx]
it_sorted = valid_int_times[sort_idx]

print(f"Loaded {len(valid_concs)} spectra, concentrations: {valid_concs.min():.1f} - {valid_concs.max():.1f} ppm")
print(f"Integration times: {np.unique(valid_int_times)} ms")

# ============================================================
# PLOT 1: Preprocessing Pipeline (4-panel, matching simulation)
# ============================================================
fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle("Preprocessing Pipeline — Real CCD Data (Chl-a)", fontsize=14, fontweight='bold')

norm = plt.Normalize(conc_sorted.min(), conc_sorted.max())
cmap = cm.plasma

# (a) Raw CCD spectra
ax = axes[0, 0]
for j in range(len(raw_sorted)):
    ax.plot(raw_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.5, lw=0.5)
ax.axvline(ROI_START, color='green', ls='--', alpha=0.5, label='ROI')
ax.axvline(ROI_END, color='green', ls='--', alpha=0.5)
ax.axvspan(ROI_START, ROI_END, alpha=0.05, color='green')
ax.set_xlabel("Pixel")
ax.set_ylabel("ADC Counts")
ax.set_title(f"(a) Raw CCD Spectra ({len(raw_sorted)} samples, 3694 px)")

# (b) ROI + BG-subtracted + normalized
ax = axes[0, 1]
roi_pixels = np.arange(ROI_START, ROI_END)
for j in range(len(roi_sorted)):
    ax.plot(roi_pixels, roi_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.5, lw=0.5)
ax.set_xlabel("Pixel")
ax.set_ylabel("Norm. Intensity")
ax.set_title("(b) ROI [1300-3200] — BG Subtracted + Normalized")

# (c) 1st derivative
ax = axes[1, 0]
for j in range(len(deriv_sorted)):
    ax.plot(roi_pixels, deriv_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.5, lw=0.5)
ax.set_xlabel("Pixel")
ax.set_ylabel("1st Derivative")
ax.set_title("(c) Savitzky-Golay 1st Derivative")

# (d) SNV normalized
ax = axes[1, 1]
for j in range(len(snv_sorted)):
    ax.plot(roi_pixels, snv_sorted[j], color=cmap(norm(conc_sorted[j])), alpha=0.5, lw=0.5)
ax.set_xlabel("Pixel")
ax.set_ylabel("SNV")
ax.set_title("(d) SNV Normalized (Model Input)")

# Add colorbar
sm = cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=axes.ravel().tolist(), shrink=0.6, pad=0.02)
cbar.set_label("Concentration (ppm)", fontsize=10)

plt.tight_layout(rect=[0, 0, 0.92, 0.95])
fig.savefig(OUTPUT_DIR / "real_spectra_pipeline.png", dpi=150, bbox_inches='tight')
print(f"Saved: real_spectra_pipeline.png")
plt.close()

# ============================================================
# PLOT 2: Heatmap (3-panel, matching simulation)  
# ============================================================
fig, axes = plt.subplots(1, 3, figsize=(16, 6))
fig.suptitle(f"Dataset Heatmap — Real CCD Data ({len(roi_sorted)} Chl-a Samples, sorted by conc.)",
             fontsize=13, fontweight='bold')

# Heatmap 1: ROI spectra
ax = axes[0]
im1 = ax.imshow(roi_sorted, aspect='auto', cmap='viridis',
                extent=[ROI_START, ROI_END, len(roi_sorted), 0])
ax.set_xlabel("Pixel")
ax.set_ylabel("Record # (sorted by conc.)")
ax.set_title("ROI Spectra (BG-sub + norm)")
cb1 = fig.colorbar(im1, ax=ax, shrink=0.7)
cb1.set_label("Intensity")

# Add concentration ticks on right
ax2 = ax.twinx()
ax2.set_ylim(len(roi_sorted), 0)
tick_positions = np.linspace(0, len(roi_sorted)-1, 8).astype(int)
ax2.set_yticks(tick_positions)
ax2.set_yticklabels([f"{conc_sorted[p]:.1f}" for p in tick_positions], fontsize=7)
ax2.set_ylabel("Concentration (ppm)", fontsize=8)

# Heatmap 2: 1st derivative
ax = axes[1]
vmax_d = np.percentile(np.abs(deriv_sorted), 98)
im2 = ax.imshow(deriv_sorted, aspect='auto', cmap='RdBu_r',
                vmin=-vmax_d, vmax=vmax_d,
                extent=[ROI_START, ROI_END, len(deriv_sorted), 0])
ax.set_xlabel("Pixel")
ax.set_ylabel("Record #")
ax.set_title("1st Derivative")
cb2 = fig.colorbar(im2, ax=ax, shrink=0.7)
cb2.set_label("d/dx")

ax3 = ax.twinx()
ax3.set_ylim(len(deriv_sorted), 0)
ax3.set_yticks(tick_positions)
ax3.set_yticklabels([f"{conc_sorted[p]:.1f}" for p in tick_positions], fontsize=7)
ax3.set_ylabel("Concentration (ppm)", fontsize=8)

# Heatmap 3: SNV
ax = axes[2]
vmax_s = np.percentile(np.abs(snv_sorted), 98)
im3 = ax.imshow(snv_sorted, aspect='auto', cmap='RdBu_r',
                vmin=-vmax_s, vmax=vmax_s,
                extent=[ROI_START, ROI_END, len(snv_sorted), 0])
ax.set_xlabel("Pixel")
ax.set_ylabel("Record #")
ax.set_title("SNV (PLS Input)")
cb3 = fig.colorbar(im3, ax=ax, shrink=0.7)
cb3.set_label("SNV")

ax4 = ax.twinx()
ax4.set_ylim(len(snv_sorted), 0)
ax4.set_yticks(tick_positions)
ax4.set_yticklabels([f"{conc_sorted[p]:.1f}" for p in tick_positions], fontsize=7)
ax4.set_ylabel("Concentration (ppm)", fontsize=8)

plt.tight_layout(rect=[0, 0, 1, 0.93])
fig.savefig(OUTPUT_DIR / "real_spectra_heatmap.png", dpi=150, bbox_inches='tight')
print(f"Saved: real_spectra_heatmap.png")
plt.close()

print("Done!")
