import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from pathlib import Path
import re
from scipy.signal import savgol_filter

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent
DATA_DIR_A = SCRIPT_DIR / "data/chl_a"
DATA_DIR_B = SCRIPT_DIR / "data/chl_b"
BG_DIR = SCRIPT_DIR / "data/background_data"
META_DIR = SCRIPT_DIR / "data/real_data"
OUTPUT_DIR = SCRIPT_DIR / "solution_analysis"
OUTPUT_DIR.mkdir(exist_ok=True)

(OUTPUT_DIR / "chl_a").mkdir(exist_ok=True)
(OUTPUT_DIR / "chl_b").mkdir(exist_ok=True)

# ROI / Dummy Pixel Configuration
ROI_START = 32
ROI_END = 3650

# ============================================================
# NORMALIZATION HELPERS
# ============================================================

def normalize_x_axis(spectrum, x_pixels):
    """
    X-axis normalization: shift spectrum so its peak aligns to pixel 0.
    Returns the spectrum reindexed on a relative pixel axis centered at the peak.
    This compensates for physical placement differences causing peak shifts.
    """
    peak_idx = np.argmax(spectrum)
    # Relative pixel axis: 0 = peak position
    x_relative = x_pixels - x_pixels[peak_idx]
    return x_relative

def normalize_y_snv(spectrum):
    """
    Y-axis SNV (Standard Normal Variate) normalization.
    Centers to zero mean and scales to unit variance.
    Removes multiplicative scatter / path-length effects.
    NOTE: This removes absolute intensity (concentration info),
    so it's used only for the shape-comparison plot, NOT for ML input.
    """
    mean = np.mean(spectrum)
    std = np.std(spectrum)
    if std < 1e-8:
        return spectrum - mean
    return (spectrum - mean) / std

def normalize_y_integration(spectrum, int_time):
    """
    Y-axis physical normalization: divide by integration time.
    Preserves concentration information (amplitude still encodes conc).
    """
    return spectrum / (int_time + 1e-8)

# ============================================================
# DATA LOADING
# ============================================================

def load_metadata(csv_path):
    try:
        df = pd.read_csv(csv_path)
        df.columns = [c.lower().strip() for c in df.columns]

        if "solution num" in df.columns:
            df.rename(columns={"solution num": "sample"}, inplace=True)
        if "sample con" in df.columns:
            df.rename(columns={"sample con": "concentration"}, inplace=True)
        if "integration time(ms)" in df.columns:
            df.rename(columns={"integration time(ms)": "integration_time"}, inplace=True)

        if "sample" in df.columns:
            df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)

        df["concentration"] = pd.to_numeric(df["concentration"], errors="coerce")
        return df.dropna(subset=["concentration"]).set_index("sample_num")
    except Exception as e:
        print(f"Error loading metadata {csv_path}: {e}")
        return pd.DataFrame()

def load_spectrum_data(filepath):
    try:
        df = pd.read_csv(filepath)
        data = df.iloc[:, 1:].values.astype(float)
        return data
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None

def load_backgrounds():
    bg_map = {}
    if not BG_DIR.exists():
        print("Background directory not found!")
        return bg_map
    for f in BG_DIR.glob("*.csv"):
        match = re.search(r"(\d+)", f.name)
        if match:
            int_time = int(match.group(1))
            data = load_spectrum_data(f)
            if data is not None:
                bg_map[int_time] = np.mean(data, axis=0)
    return bg_map

def get_closest_background(bg_map, target_int_time):
    if not bg_map:
        return None
    if target_int_time in bg_map:
        return bg_map[target_int_time]
    closest_time = min(bg_map.keys(), key=lambda x: abs(x - target_int_time))
    return bg_map[closest_time]

def preprocess_spectra(raw_data, bg_spectrum):
    if bg_spectrum is not None:
        min_len = min(raw_data.shape[1], len(bg_spectrum))
        data_sub = raw_data[:, :min_len] - bg_spectrum[:min_len]
    else:
        data_sub = raw_data.copy()

    start = max(0, ROI_START)
    end = min(data_sub.shape[1], ROI_END)
    data_roi = data_sub[:, start:end]

    window = 51
    if data_roi.shape[1] < window:
        window = data_roi.shape[1] // 2 * 2 + 1

    if window > 3:
        data_smooth = savgol_filter(data_roi, window, 3, axis=1)
    else:
        data_smooth = data_roi

    return data_smooth, np.arange(start, end)

# ============================================================
# PLOTTING HELPERS
# ============================================================

def style_ax(ax, title, xlabel, ylabel):
    ax.set_title(title, fontsize=13, fontweight='bold', pad=10)
    ax.set_xlabel(xlabel, fontsize=11)
    ax.set_ylabel(ylabel, fontsize=11)
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

def save_fig(fig, path, title_suffix=""):
    plt.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"  Saved: {path}")

# ============================================================
# MAIN PROCESSING
# ============================================================

def process_dataset(name, data_dir, meta_file, out_dir, bg_map):
    print(f"\nProcessing {name}...")
    meta = load_metadata(meta_file)
    if meta.empty:
        print(f"  No metadata found for {name}")
        return

    meta_sorted = meta.sort_values("concentration")
    n = len(meta_sorted)
    cmap = cm.get_cmap('viridis', n)

    # Storage for all processed spectra
    all_x_orig = []       # original pixel axis
    all_x_aligned = []    # x-aligned (peak at 0)
    all_y_inttime = []    # y normalized by integration time only
    all_y_snv = []        # y SNV normalized (shape only)
    all_y_both = []       # both x-aligned + SNV
    all_concs = []
    all_labels = []

    for i, (sample_num, row) in enumerate(meta_sorted.iterrows()):
        file_path = data_dir / f"{sample_num}.csv"
        if not file_path.exists():
            continue

        raw_data = load_spectrum_data(file_path)
        if raw_data is None:
            continue

        conc = row['concentration']
        int_time = int(row.get('integration_time', 1000))

        bg = get_closest_background(bg_map, int_time)
        proc_data, x_proc = preprocess_spectra(raw_data, bg)
        proc_mean = np.mean(proc_data, axis=0)

        # --- Y: Integration time normalization (preserves concentration info) ---
        y_inttime = normalize_y_integration(proc_mean, int_time)

        # --- X: Peak alignment (shift so peak is at relative pixel 0) ---
        x_aligned = normalize_x_axis(y_inttime, x_proc)

        # --- Y: SNV on top of integration-time normalization (shape only) ---
        y_snv = normalize_y_snv(y_inttime)

        # --- Both: X-aligned + SNV ---
        y_both = normalize_y_snv(y_inttime)  # same y, different x labeling

        all_x_orig.append(x_proc)
        all_x_aligned.append(x_aligned)
        all_y_inttime.append(y_inttime)
        all_y_snv.append(y_snv)
        all_y_both.append(y_both)
        all_concs.append(conc)
        all_labels.append(f"{conc:.2f} ppm ({int_time}ms)")

    if not all_x_orig:
        print("  No spectra loaded.")
        return

    n_loaded = len(all_x_orig)
    colors = [cmap(i / max(n_loaded - 1, 1)) for i in range(n_loaded)]

    # ================================================================
    # FIGURE 1: Original (Integration Time Normalized) — baseline
    # ================================================================
    fig1, ax1 = plt.subplots(figsize=(14, 6))
    for i in range(n_loaded):
        ax1.plot(all_x_orig[i], all_y_inttime[i], color=colors[i], linewidth=1.2, alpha=0.85)
    style_ax(ax1,
             f"{name} — Y Normalized by Integration Time (Baseline)",
             "Pixel Index",
             "Intensity / ms")
    # Colorbar for concentration
    sm = cm.ScalarMappable(cmap='viridis',
                           norm=plt.Normalize(vmin=min(all_concs), vmax=max(all_concs)))
    sm.set_array([])
    fig1.colorbar(sm, ax=ax1, label="Concentration (ppm)", pad=0.01)
    save_fig(fig1, out_dir.parent / f"{name.lower().replace('-','')}_1_y_inttime.png")

    # ================================================================
    # FIGURE 2: X-axis aligned (peak centered at 0) + Y int-time norm
    # ================================================================
    fig2, ax2 = plt.subplots(figsize=(14, 6))
    for i in range(n_loaded):
        ax2.plot(all_x_aligned[i], all_y_inttime[i], color=colors[i], linewidth=1.2, alpha=0.85)
    style_ax(ax2,
             f"{name} — X Peak-Aligned (Peak=0) + Y Normalized by Integration Time\n"
             f"Compensates for physical placement shifts",
             "Relative Pixel Index (0 = Peak)",
             "Intensity / ms")
    fig2.colorbar(sm, ax=ax2, label="Concentration (ppm)", pad=0.01)
    save_fig(fig2, out_dir.parent / f"{name.lower().replace('-','')}_2_x_aligned_y_inttime.png")

    # ================================================================
    # FIGURE 3: Y SNV normalized only (shape comparison, no amplitude)
    # ================================================================
    fig3, ax3 = plt.subplots(figsize=(14, 6))
    for i in range(n_loaded):
        ax3.plot(all_x_orig[i], all_y_snv[i], color=colors[i], linewidth=1.2, alpha=0.85)
    style_ax(ax3,
             f"{name} — Y SNV Normalized (Shape Only, Concentration Info Removed)\n"
             f"Good for peak shape comparison, NOT for concentration prediction",
             "Pixel Index",
             "SNV Units (zero mean, unit variance)")
    fig3.colorbar(sm, ax=ax3, label="Concentration (ppm)", pad=0.01)
    save_fig(fig3, out_dir.parent / f"{name.lower().replace('-','')}_3_y_snv.png")

    # ================================================================
    # FIGURE 4: Both X-aligned + Y SNV (fully normalized shape)
    # ================================================================
    fig4, ax4 = plt.subplots(figsize=(14, 6))
    for i in range(n_loaded):
        ax4.plot(all_x_aligned[i], all_y_both[i], color=colors[i], linewidth=1.2, alpha=0.85)
    style_ax(ax4,
             f"{name} — X Peak-Aligned + Y SNV (Fully Normalized)\n"
             f"Removes placement shifts AND intensity scaling — pure shape comparison",
             "Relative Pixel Index (0 = Peak)",
             "SNV Units (zero mean, unit variance)")
    fig4.colorbar(sm, ax=ax4, label="Concentration (ppm)", pad=0.01)
    save_fig(fig4, out_dir.parent / f"{name.lower().replace('-','')}_4_both_normalized.png")

    # ================================================================
    # FIGURE 5: 2x2 summary grid
    # ================================================================
    fig5, axes = plt.subplots(2, 2, figsize=(18, 10))
    fig5.suptitle(f"{name} — Normalization Comparison", fontsize=15, fontweight='bold', y=1.01)

    plot_configs = [
        (axes[0, 0], all_x_orig,     all_y_inttime, "1. Y: Integration Time Norm\nX: Raw Pixel",         "Pixel Index",                    "Intensity / ms"),
        (axes[0, 1], all_x_aligned,  all_y_inttime, "2. Y: Integration Time Norm\nX: Peak-Aligned (0=Peak)", "Relative Pixel (0 = Peak)",  "Intensity / ms"),
        (axes[1, 0], all_x_orig,     all_y_snv,     "3. Y: SNV Norm (Shape Only)\nX: Raw Pixel",          "Pixel Index",                    "SNV Units"),
        (axes[1, 1], all_x_aligned,  all_y_both,    "4. Y: SNV Norm\nX: Peak-Aligned  ← Best for alignment QC", "Relative Pixel (0 = Peak)", "SNV Units"),
    ]

    for ax, xs, ys, title, xlabel, ylabel in plot_configs:
        for i in range(n_loaded):
            ax.plot(xs[i], ys[i], color=colors[i], linewidth=1.0, alpha=0.8)
        style_ax(ax, title, xlabel, ylabel)

    # Shared colorbar
    fig5.subplots_adjust(right=0.88, hspace=0.4, wspace=0.3)
    cbar_ax = fig5.add_axes([0.91, 0.15, 0.02, 0.7])
    fig5.colorbar(sm, cax=cbar_ax, label="Concentration (ppm)")

    summary_path = out_dir.parent / f"{name.lower().replace('-','')}_5_summary_grid.png"
    fig5.savefig(summary_path, dpi=150, bbox_inches='tight')
    plt.close(fig5)
    print(f"  Saved: {summary_path}")

    # ================================================================
    # Feature stats printout
    # ================================================================
    peak_positions = [x[np.argmax(y)] for x, y in zip(all_x_orig, all_y_inttime)]
    print(f"\n--- {name} Peak Position Stats (raw pixel) ---")
    print(f"  Mean : {np.mean(peak_positions):.1f}")
    print(f"  Std  : {np.std(peak_positions):.1f}  ← spread due to placement variation")
    print(f"  Range: {np.min(peak_positions):.0f} – {np.max(peak_positions):.0f}")
    print(f"  After X-alignment all peaks sit at relative pixel 0.")


def main():
    print("Loading backgrounds...")
    bg_map = load_backgrounds()
    print(f"  Loaded {len(bg_map)} background spectra.")

    process_dataset("Chl-A",
                    DATA_DIR_A,
                    META_DIR / "chla_data.csv",
                    OUTPUT_DIR / "chl_a",
                    bg_map)

    process_dataset("Chl-B",
                    DATA_DIR_B,
                    META_DIR / "chlb_data.csv",
                    OUTPUT_DIR / "chl_b",
                    bg_map)

    print("\nDone. Output images saved to solution_analysis/")


if __name__ == "__main__":
    main()
