import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.signal import savgol_filter

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent
DATA_DIR_A = SCRIPT_DIR / "data/chl_a"
DATA_DIR_B = SCRIPT_DIR / "data/chl_b"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

# Params from train_and_save.py
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY = 2
SG_DERIV_WINDOW = 11
SG_DERIV_POLY = 3
SG_DERIV_ORDER = 1
ROI_START = 1300
ROI_END = 3500

# ============================================================
# PREPROCESSING FUNCTIONS
# ============================================================
def load_and_average_spectrum(csv_path):
    if not csv_path.exists():
        print(f"File not found: {csv_path}")
        return None
    df = pd.read_csv(csv_path)
    # Assuming first column is frame_number/index and rest are data
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def moving_average_3pt(spectrum):
    return np.convolve(spectrum, np.ones(3)/3, mode='same')

def preprocess_spectrum_existing(spectrum):
    # 1. Slice ROI FIRST (Remove dummy pixels)
    sliced = spectrum[ROI_START:ROI_END]

    # 2. Savitzky-Golay Smoothing
    smoothed = savgol_filter(sliced, window_length=SG_SMOOTH_WINDOW, polyorder=SG_SMOOTH_POLY)
    
    # 3. Savitzky-Golay Derivative
    deriv = savgol_filter(smoothed, window_length=SG_DERIV_WINDOW, polyorder=SG_DERIV_POLY, deriv=SG_DERIV_ORDER)
    
    # 4. SNV (Standard Normal Variate) on ROI
    mean = np.mean(deriv)
    std = np.std(deriv)
    snv = (deriv - mean) / (std + 1e-8)
    return snv

def get_processed_data(sample_file, bg_file, integration_time):
    raw = load_and_average_spectrum(sample_file)
    bg = load_and_average_spectrum(bg_file)
    
    if raw is None or bg is None:
        return None, None, None, None

    min_len = min(len(raw), len(bg))
    raw = raw[:min_len]
    bg = bg[:min_len]

    # Dark Subtraction
    dark_sub = raw - bg
    
    # Normalize by integration time
    normalized = dark_sub / (integration_time + 1e-8)

    # 1. 3-Point Moving Average (on normalized data) - Full Spectrum
    ma_3pt = moving_average_3pt(normalized)

    # 2. Existing Pipeline (on normalized data) - Returns ROI only
    existing_processed = preprocess_spectrum_existing(normalized)

    return normalized, ma_3pt, existing_processed

# ============================================================
# MAIN VISUALIZATION
# ============================================================
def main():
    # Setup samples
    # Chl-A: Sample 1 (300ms) -> Background 300
    file_a = DATA_DIR_A / "1.csv"
    bg_a = BACKGROUND_DIR / "background-300.csv"
    int_time_a = 300

    # Chl-B: Sample 1 (1000ms) -> Background 1000
    file_b = DATA_DIR_B / "1.csv"
    bg_b = BACKGROUND_DIR / "background-1000.csv" # Using the renamed good file
    int_time_b = 1000

    # Process
    norm_a, ma_a, pipe_a = get_processed_data(file_a, bg_a, int_time_a)
    norm_b, ma_b, pipe_b = get_processed_data(file_b, bg_b, int_time_b)

    if norm_a is None or norm_b is None:
        print("Error loading data.")
        return

    # Plot
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), sharex='col')
    
    # Titles
    axes[0, 0].set_title("Chl-A (300ms) - Raw / Dark Subtracted")
    axes[0, 1].set_title("Chl-B (1000ms) - Raw / Dark Subtracted")
    
    # Row 1: Normalized (Dark Subtracted / Int Time)
    axes[0, 0].plot(norm_a, label="Dark Subtracted + Norm", color='blue', alpha=0.7)
    axes[0, 0].legend()
    axes[0, 1].plot(norm_b, label="Dark Subtracted + Norm", color='green', alpha=0.7)
    axes[0, 1].legend()

    # Row 2: 3-Point Moving Average
    axes[1, 0].plot(ma_a, label="3-Point Mean Smooth", color='orange')
    axes[1, 0].set_title("3-Point Mean Smoothing")
    axes[1, 0].legend()
    axes[1, 1].plot(ma_b, label="3-Point Mean Smooth", color='orange')
    axes[1, 1].set_title("3-Point Mean Smoothing")
    axes[1, 1].legend()

    # Row 3: Existing Pipeline (SG Smooth + Deriv + SNV)
    axes[2, 0].plot(pipe_a, label="Existing Pipeline (SG+Deriv+SNV)", color='red')
    axes[2, 0].set_title("Existing Preprocessing Pipeline")
    axes[2, 0].legend()
    axes[2, 1].plot(pipe_b, label="Existing Pipeline (SG+Deriv+SNV)", color='red')
    axes[2, 1].set_title("Existing Preprocessing Pipeline")
    axes[2, 1].legend()

    # Layout
    plt.tight_layout()
    output_path = SCRIPT_DIR / "pipeline_visualization.png"
    plt.savefig(output_path, dpi=100)
    print(f"Pipeline visualization saved to {output_path}")

if __name__ == "__main__":
    main()
