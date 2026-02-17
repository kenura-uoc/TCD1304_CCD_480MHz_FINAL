import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import scipy.stats as stats

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent
DATA_DIR = SCRIPT_DIR / "data/background_data"
OUTPUT_DIR = SCRIPT_DIR / "analysis_results"
OUTPUT_DIR.mkdir(exist_ok=True)

def load_data(filepath):
    try:
        df = pd.read_csv(filepath)
        # Assuming first column is label/index, rest are pixel data
        data = df.iloc[:, 1:].values.astype(float)
        return data, df.shape[0] # Return data and n_replicates
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        return None, 0

def plot_individual_with_error(filename, data, ax=None):
    if ax is None:
        fig, ax = plt.subplots(figsize=(10, 6))
        return_fig = True
    else:
        return_fig = False

    mean_spectrum = np.mean(data, axis=0)
    std_spectrum = np.std(data, axis=0)
    
    pixels = np.arange(len(mean_spectrum))
    
    ax.plot(pixels, mean_spectrum, label='Mean', color='blue', linewidth=1)
    ax.fill_between(pixels, 
                   mean_spectrum - std_spectrum, 
                   mean_spectrum + std_spectrum, 
                   color='blue', alpha=0.3, label='±1 Std Dev (Error)')
    
    ax.set_title(f"Spectrum: {filename} (n={data.shape[0]})")
    ax.set_xlabel("Pixel Index")
    ax.set_ylabel("Intensity")
    ax.legend(loc='upper right')
    ax.grid(True, alpha=0.3)
    
    if return_fig:
        return fig

def perform_statistical_checks(data_map):
    print("\n" + "="*50)
    print("ST2004 STATISTICAL ADEQUACY CHECKS")
    print("="*50)

    # 1. Replication Check
    print("\n1. REPLICATION CHECK")
    print("-" * 30)
    for name, data in data_map.items():
        print(f"  {name}: {data.shape[0]} replicates")
        if data.shape[0] < 3:
            print("    [WARNING] Low replication count (n < 3). Power of tests will be low.")

    # 2. Homoscedasticity (Constant Variance) - Levene's Test
    print("\n2. HOMOSCEDASTICITY (Constant Variance)")
    print("   Testing if noise levels (variance) differ across integration times.")
    print("-" * 30)
    
    # We'll take the variance of the *middle* pixel (approx pixel 1500) as a sample
    # to avoid testing 3600 pixels individually.
    test_pixel = 1500
    samples = [data[:, test_pixel] for data in data_map.values()]
    names = list(data_map.keys())
    
    try:
        stat, p_value = stats.levene(*samples)
        print(f"  Levene's Test on Pixel {test_pixel}:")
        print(f"    Statistic: {stat:.4f}")
        print(f"    p-value:   {p_value:.4e}")
        
        if p_value < 0.05:
            print("    [RESULT] Significant difference in variances (p < 0.05). Assumption Violates.")
            print("    > The noise level (variance) is NOT constant across different integration times.")
            print("    > This suggests Heteroscedasticity.")
        else:
            print("    [RESULT] No significant difference in variances. Assumption Holds.")
    except Exception as e:
        print(f"  Could not perform Levene's test: {e}")

    # 3. Normality Assumption (Residual Analysis)
    print("\n3. NORMALITY OF ERROR TERMS")
    print("   Checking if replicates at a random pixel follow a Normal distribution.")
    print("-" * 30)
    
    for name, data in data_map.items():
        if data.shape[0] < 3: 
            continue # Comparison requires replicates

        pixel_data = data[:, test_pixel]
        stat, p = stats.shapiro(pixel_data)
        
        normality_status = "Normal" if p > 0.05 else "Non-Normal"
        print(f"  {name:<25}: p={p:.4f} ({normality_status})")

def main():
    csv_files = sorted(list(DATA_DIR.glob("*.csv")))
    if not csv_files:
        print("No CSV files found.")
        return

    data_map = {}
    
    print("Generating Individual Plots...")
    for filepath in csv_files:
        data, n = load_data(filepath)
        if data is None: continue
        
        data_map[filepath.name] = data
        
        # Determine plot bounds for better visualization of error
        mean = np.mean(data, axis=0)
        std = np.std(data, axis=0)
        
        # Individual Plot
        fig = plot_individual_with_error(filepath.name, data)
        # Zoom in Y to see the error band clearly if std is small vs intensity
        # plt.ylim(mean.min() - 5*std.max(), mean.max() + 5*std.max()) 
        
        out_name = OUTPUT_DIR / f"plot_{filepath.stem}.png"
        fig.savefig(out_name)
        plt.close(fig)
        print(f"  Saved {out_name}")

    # Combined Plot with Error Bands (Alpha blending)
    print("Generating Combined Plot...")
    fig, ax = plt.subplots(figsize=(12, 8))
    
    cmap = plt.get_cmap('viridis')
    indices = np.linspace(0, 1, len(data_map))
    
    for i, (name, data) in enumerate(data_map.items()):
        mean = np.mean(data, axis=0)
        std = np.std(data, axis=0)
        pixels = np.arange(len(mean))
        color = cmap(indices[i])
        
        ax.plot(pixels, mean, label=name, color=color, linewidth=1)
        ax.fill_between(pixels, mean - std, mean + std, color=color, alpha=0.1)
        
    ax.set_title("Combined Background Spectra with Error Boundaries (±1σ)")
    ax.set_xlabel("Pixel Index")
    ax.set_ylabel("Intensity")
    ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    
    combined_out = OUTPUT_DIR / "combined_backgrounds_with_error.png"
    plt.savefig(combined_out)
    print(f"  Saved {combined_out}")
    
    # Statistical Analysis
    perform_statistical_checks(data_map)

if __name__ == "__main__":
    main()
