import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
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

def load_metadata(csv_path):
    try:
        df = pd.read_csv(csv_path)
        # Normalize columns
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

def plot_single(ax, x, mean, std, color, label, alpha_fill=0.2, linewidth=1.5):
    ax.plot(x, mean, label=label, color=color, linewidth=linewidth)
    if std is not None:
        ax.fill_between(x, mean - std, mean + std, color=color, alpha=alpha_fill)

def process_dataset(name, data_dir, meta_file, out_dir, bg_map):
    print(f"\nProcessing {name}...")
    meta = load_metadata(meta_file)
    if meta.empty:
        print(f"  No metadata found for {name}")
        return

    # Combined Plot Setup
    fig_comb, ax_comb = plt.subplots(figsize=(14, 8))
    cmap = plt.get_cmap('viridis')
    
    meta_sorted = meta.sort_values("concentration")
    stats_list = []
    
    # --- Combined Plot (Normalized by Integration Time) ---
    fig_norm, ax_norm = plt.subplots(figsize=(14, 8))

    for i, (sample_num, row) in enumerate(meta_sorted.iterrows()):
        file_path = data_dir / f"{sample_num}.csv"
        if not file_path.exists():
            continue
            
        raw_data = load_spectrum_data(file_path)
        if raw_data is None: continue
        
        conc = row['concentration']
        int_time = int(row.get('integration_time', 0))
        
        bg = get_closest_background(bg_map, int_time)
        proc_data, x_proc = preprocess_spectra(raw_data, bg)
        
        # Calculate stats
        raw_mean = np.mean(raw_data, axis=0)
        proc_mean = np.mean(proc_data, axis=0)
        proc_std = np.std(proc_data, axis=0)
        
        # Feature Extraction
        peak_idx = np.argmax(proc_mean)
        peak_x = x_proc[peak_idx]
        peak_h = proc_mean[peak_idx]
        
        # FWHM
        half_max = peak_h / 2
        above_half = np.where(proc_mean > half_max)[0]
        if len(above_half) > 1:
            fwhm = x_proc[above_half[-1]] - x_proc[above_half[0]]
        else:
            fwhm = 0
            
        # Centroid
        centroid = np.sum(x_proc * proc_mean) / np.sum(proc_mean) if np.sum(proc_mean) > 0 else peak_x
        
        stats_list.append({
            'sample': sample_num,
            'conc': conc,
            'int_time': int_time,
            'peak_x': peak_x,
            'peak_h': peak_h,
            'fwhm': fwhm,
            'centroid': centroid
        })

        label = f"{conc} ppm"
        
        # --- Individual Plot ---
        fig_ind, ax_ind = plt.subplots(figsize=(10, 6))
        ax_ind.plot(np.arange(len(raw_mean)), raw_mean, color='gray', alpha=0.3, linewidth=0.8, label='Raw Input (Inverted)')
        plot_single(ax_ind, x_proc, proc_mean, proc_std, 'blue', "Processed (Dark Sub + Smooth)")
        ax_ind.set_title(f"{name} Sample {sample_num}\nConc: {conc} ppm | Int: {int_time}ms | Peak: {peak_x}")
        ax_ind.set_xlabel("Pixel Index")
        ax_ind.set_ylabel("Intensity")
        ax_ind.legend()
        ax_ind.grid(True, alpha=0.3)
        ind_out = out_dir / f"sample_{sample_num}.png"
        fig_ind.savefig(ind_out)
        plt.close(fig_ind)
        
        # --- Add to Combined Plot ---
        color = cmap(i / len(meta_sorted))
        plot_single(ax_comb, x_proc, proc_mean, None, color, label, linewidth=1.5)
        
        # --- Add to Normalized Plot ---
        if int_time > 0:
            norm_mean = proc_mean / int_time
            norm_label = f"{conc} ppm ({int_time}ms)"
            plot_single(ax_norm, x_proc, norm_mean, None, color, norm_label, linewidth=1.5)

    # Save Combined (Regular)
    ax_comb.set_title(f"Combined {name} Spectra (Processed: Dark Subtracted + Smoothed)")
    ax_comb.set_xlabel("Pixel Index")
    ax_comb.set_ylabel("Intensity (ADU)")
    ax_comb.legend(title="Concentration", bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
    ax_comb.grid(True, alpha=0.3)
    plt.tight_layout()
    comb_out = out_dir.parent / f"combined_{name.lower().replace('-', '')}.png"
    plt.savefig(comb_out)
    print(f"  Saved combined plot to {comb_out}")

    # Save Combined (Normalized)
    ax_norm.set_title(f"Combined {name} Spectra (Normalized by Integration Time)")
    ax_norm.set_xlabel("Pixel Index")
    ax_norm.set_ylabel("Intensity / ms")
    ax_norm.legend(title="Concentration", bbox_to_anchor=(1.05, 1), loc='upper left', fontsize='small')
    ax_norm.grid(True, alpha=0.3)
    plt.tight_layout()
    norm_out = out_dir.parent / f"combined_{name.lower().replace('-', '')}_normalized.png"
    plt.savefig(norm_out)
    print(f"  Saved normalized plot to {norm_out}")
    
    # --- Feature Analysis ---
    if not stats_list:
        return

    df_stats = pd.DataFrame(stats_list)
    
    # print summary
    print(f"\n--- {name} Statistical Summary ---")
    print(f"Mean Peak Position: {df_stats['peak_x'].mean():.2f} ± {df_stats['peak_x'].std():.2f}")
    print(f"Mean FWHM:          {df_stats['fwhm'].mean():.2f} ± {df_stats['fwhm'].std():.2f}")
    
    # Plot Features
    fig_feat, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig_feat.suptitle(f"{name} Spectral Features Analysis")
    
    # Peak Pos vs Conc
    axes[0,0].scatter(df_stats['conc'], df_stats['peak_x'], c=df_stats['conc'], cmap='viridis')
    axes[0,0].set_title("Peak Position vs Concentration")
    axes[0,0].set_xlabel("Concentration")
    axes[0,0].set_ylabel("Peak Index")
    axes[0,0].grid(True, alpha=0.3)
    
    # FWHM vs Conc
    axes[0,1].scatter(df_stats['conc'], df_stats['fwhm'], c=df_stats['conc'], cmap='viridis')
    axes[0,1].set_title("FWHM vs Concentration")
    axes[0,1].set_xlabel("Concentration")
    axes[0,1].set_ylabel("FWHM (pixels)")
    axes[0,1].grid(True, alpha=0.3)
    
    # Centroid vs Conc
    axes[1,0].scatter(df_stats['conc'], df_stats['centroid'], c=df_stats['conc'], cmap='viridis')
    axes[1,0].set_title("Centroid vs Concentration")
    axes[1,0].set_xlabel("Concentration")
    axes[1,0].set_ylabel("Centroid Index")
    axes[1,0].grid(True, alpha=0.3)
    
    # Peak Pos Histogram
    axes[1,1].hist(df_stats['peak_x'], bins=10, color='skyblue', edgecolor='black')
    axes[1,1].set_title(f"Peak Position Distribution\nMean: {df_stats['peak_x'].mean():.1f}")
    axes[1,1].set_xlabel("Peak Index")
    axes[1,1].set_ylabel("Count")
    axes[1,1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    feat_out = out_dir.parent / f"features_{name.lower().replace('-', '')}.png"
    plt.savefig(feat_out)
    print(f"  Saved feature plot to {feat_out}")

def main():
    print("Loading backgrounds...")
    bg_map = load_backgrounds()
    print(f"  Loaded {len(bg_map)} background spectra.")

    # Process Chl-A
    process_dataset("Chl-A", 
                   DATA_DIR_A, 
                   META_DIR / "chla_data.csv", 
                   OUTPUT_DIR / "chl_a",
                   bg_map)
                   
    # Process Chl-B
    process_dataset("Chl-B", 
                   DATA_DIR_B, 
                   META_DIR / "chlb_data.csv", 
                   OUTPUT_DIR / "chl_b",
                   bg_map)

if __name__ == "__main__":
    main()
