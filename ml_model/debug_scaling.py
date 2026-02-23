
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Paths
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_DIR = SCRIPT_DIR.parent / "ccd_monitor/projects/16-2-2026"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

# Data
Rec4 = {"405": "rec_20260216_133810.npz", "450": "rec_20260216_133914.npz", "int": 300}
Rec5 = {"405": "rec_20260216_134320.npz", "450": "rec_20260216_134352.npz", "int": 1000}


def get_peak_raw(fname, int_time):
    path = PROJECT_DIR / fname
    data = np.load(path)
    pixels = np.mean(data['pixels'], axis=0)
    if np.mean(pixels) < 30000: pixels = 65535 - pixels
    
    # Raw Peak (after inversion)
    raw_peak = np.max(pixels)
    # Baseline (dummy pixels)
    baseline = np.mean(pixels[10:100])
    # Peak above baseline
    peak_above_bl = raw_peak - baseline
    
    # Background
    bg_path = BACKGROUND_DIR / f"background-{int_time}.csv"
    bg = np.mean(pd.read_csv(bg_path).iloc[:, 1:].values, axis=0) if bg_path.exists() else np.zeros_like(pixels)
    bg_peak = np.max(bg)
    bg_baseline = np.mean(bg[10:100])
    
    # Net (following validate_sol9 logic)
    offset = baseline - bg_baseline
    net = (pixels - offset) - bg
    net_peak = np.max(net)
    
    return {
        "raw_peak": raw_peak,
        "baseline": baseline,
        "peak_above_bl": peak_above_bl,
        "bg_peak": bg_peak,
        "net_peak": net_peak,
        "norm_net": net_peak / int_time
    }


def main():
    r4_405 = get_peak_raw(Rec4["405"], 300)
    r5_405 = get_peak_raw(Rec5["405"], 1000)
    
    # Reload for plotting
    d4 = np.load(PROJECT_DIR / Rec4["405"])
    s4 = np.mean(d4['pixels'], axis=0)
    if np.mean(s4) < 30000: s4 = 65535 - s4
    
    d5 = np.load(PROJECT_DIR / Rec5["405"])
    s5 = np.mean(d5['pixels'], axis=0)
    if np.mean(s5) < 30000: s5 = 65535 - s5
    
    print(f"--- 405nm Scaling Analysis ---")
    print(f"300ms:  Raw={r4_405['raw_peak']:.1f}, Base={r4_405['baseline']:.1f}, Peak-Above={r4_405['peak_above_bl']:.1f}, Net={r4_405['net_peak']:.1f}")
    print(f"1000ms: Raw={r5_405['raw_peak']:.1f}, Base={r5_405['baseline']:.1f}, Peak-Above={r5_405['peak_above_bl']:.1f}, Net={r5_405['net_peak']:.1f}")
    
    ratio_time = 1000/300
    ratio_above = r5_405['peak_above_bl'] / r4_405['peak_above_bl']
    ratio_net = r5_405['net_peak'] / r4_405['net_peak']
    
    print(f"\nTime Ratio:   {ratio_time:.3f}")
    print(f"Above-Base Ratio: {ratio_above:.3f} ({(ratio_above/ratio_time):.2f}x deviation)")
    print(f"Net-Signal Ratio: {ratio_net:.3f} ({(ratio_net/ratio_time):.2f}x deviation)")

    # Check Background
    print(f"\n--- Background Analysis ---")
    print(f"BG 300ms:  Peak={r4_405['bg_peak']:.1f}")
    print(f"BG 1000ms: Peak={r5_405['bg_peak']:.1f}")
    print(f"BG Ratio:  {r5_405['bg_peak'] / r4_405['bg_peak']:.3f} vs expected {ratio_time:.3f}")
    
    # Plot Comparison
    plt.figure(figsize=(12, 6))
    plt.plot(s4[1300:3500]/300, label="300ms (405nm) Normed", alpha=0.7)
    plt.plot(s5[1300:3500]/1000, label="1000ms (405nm) Normed", alpha=0.7)
    plt.title("Normalized RAW Signal (Signal / IntTime) Comparison")
    plt.ylabel("Intensity / ms")
    plt.legend()
    plt.savefig(SCRIPT_DIR / "training_plots" / "scaling_investigation.png")
    print(f"\nPlot saved to training_plots/scaling_investigation.png")

if __name__ == "__main__":
    main()
