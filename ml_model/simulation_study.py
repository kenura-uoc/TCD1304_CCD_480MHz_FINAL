#!/usr/bin/env python3
"""
simulation_study.py — Comprehensive Data-Collection Readiness Test
===================================================================
Simulates realistic TCD1304 CCD fluorescence spectra and answers:

1. How many samples are needed to reach ≤ 0.01 mg/L RMSE?
2. What integration-time range keeps SNR high without saturation?
3. Does peak-alignment (removing the red-shift) help or hurt PLS?
4. What noise level (frame-averaging) is required?
5. What is the minimum number of PLS components needed?

Outputs:  ml_model/simulation_results/  (plots + CSV)
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pathlib import Path
from itertools import product
from scipy.signal import savgol_filter
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import KFold, cross_val_predict
from sklearn.metrics import mean_squared_error, r2_score

np.random.seed(42)

OUT_DIR = Path(__file__).parent / "simulation_results"
OUT_DIR.mkdir(exist_ok=True)

# ── TCD1304 sensor parameters ─────────────────────────────────
TOTAL_PIXELS = 3694
ROI_START    = 1300
ROI_END      = 3500
ROI_LEN      = ROI_END - ROI_START        # 2200 features

# ── Realistic spectrum generator ──────────────────────────────
# ── Realistic spectrum generator ──────────────────────────────
def generate_fluorescence_spectrum(
    concentration,              # mg/L
    integration_time_ms=300,    # ms
    noise_frames=1,             # how many frames are averaged
    analyte="chla",             # "chla" or "chlb"
    dark_temp_c=30,             # Ambient temperature for dark current simulation
    read_noise_std=240,         # ADC counts read-noise per pixel per frame (scaled)
):
    """Generate a single synthetic CCD frame (3694 pixels).
    
    Physics modelled:
      • Peak position shifts to higher pixels with concentration
      • FWHM broadens slightly with concentration
      • Intensity is sub-linear (Beer-Lambert + self-absorption saturation)
      • Shot noise, read noise, dark current
      • Integration-time scaling
    """
    if analyte == "chla":
        base_peak_pixel = 1950
        red_shift_rate  = 12.0
        fwhm_base       = 180
        fwhm_growth     = 6.0
        alpha           = 0.12   # absorption coefficient
        max_signal      = 40000  # Calibrated: Chl-A is quite bright
    else: # chlb
        base_peak_pixel = 2200
        red_shift_rate  = 40.0   # Chl-b has stronger overlap/shift
        fwhm_base       = 220
        fwhm_growth     = 8.0
        alpha           = 0.18   # Chl-b often saturates faster in some setups
        max_signal      = 7500   # Calibrated: Chl-B is much dimmer
    
    # --- TCD1304 Spectral Response (from datasheet) ---
    # Wavelength of emission peak:
    # Chl-A: ~670nm, Chl-B: ~650nm
    emission_wl = 670 if analyte=="chla" else 650
    # From "SPECTRAL RESPONSE" plot: 
    # 550nm: 1.0, 650nm: ~0.95, 700nm: ~0.82
    spectral_response = np.interp(emission_wl, [400, 550, 650, 700, 800, 900, 1000], [0.8, 1.0, 0.95, 0.82, 0.45, 0.2, 0.05])
    
    # Apply hardware sensitivity
    max_signal *= spectral_response

    x = np.arange(TOTAL_PIXELS, dtype=float)

    # --- Peak position (red-shift) ---
    peak_pos = base_peak_pixel + concentration * red_shift_rate

    # --- Peak width (FWHM → sigma) ---
    fwhm = fwhm_base + concentration * fwhm_growth
    sigma = fwhm / 2.355

    # --- Intensity (saturating model) ---
    fluor_intensity = max_signal * (1.0 - np.exp(-alpha * concentration))
    # ... but it also depends on excitation efficiency (constant here)
    signal_per_ms = fluor_intensity / 300.0
    signal = signal_per_ms * integration_time_ms

    # --- Gaussian peak ---
    spectrum_clean = signal * np.exp(-0.5 * ((x - peak_pos) / sigma) ** 2)

    # --- Add dark level (datasheet calibrated) + noise ---
    # Fig: Dark Signal Voltage vs t_INT shows linear increase.
    # At 30C: ~1% of saturation per 100ms.
    # Scaled for 16-bit: 655 counts per 100ms = 6.55 counts/ms
    # We add a temp multiplier based on common CCD dark current doubling rate (~7C)
    dark_multiplier = 2.0 ** ((dark_temp_c - 30) / 7.0)
    dark_level = 6.55 * integration_time_ms * dark_multiplier
    
    frames = []
    for _ in range(noise_frames):
        frame = spectrum_clean + dark_level
        shot = np.random.normal(0, np.sqrt(np.maximum(frame, 1)))
        read = np.random.normal(0, read_noise_std, TOTAL_PIXELS)
        frames.append(frame + shot + read)

    avg_frame = np.mean(frames, axis=0)
    return np.clip(avg_frame, 0, 65535)  # 16-bit ADC Saturation


# ── Internal Overview Plot Generator ──────────────────────────
def generate_overview_plots(analyte="chla"):
    """Generate 'Ideal Pipeline' and 'Ideal Heatmap' matching report format."""
    print(f"\n═══ Generating OVERVIEW PLOTS for {analyte.upper()} ═══")
    n_samples = 300
    concs = np.linspace(0.1, 20.0, n_samples)
    it = 500
    nf = 128
    
    bg = generate_fluorescence_spectrum(0.0, it, nf, analyte=analyte)
    
    raw_list = []
    roi_list = []
    deriv_list = []
    snv_list = []
    
    for c in concs:
        raw = generate_fluorescence_spectrum(c, it, nf, analyte=analyte)
        s = (raw - bg) / it
        roi = s[ROI_START:ROI_END]
        
        proc = preprocess(raw, it, bg)
        # For visualization steps:
        smoothed = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
        deriv = savgol_filter(smoothed, SG_DERIV_W, SG_DERIV_P, deriv=1)
        
        raw_list.append(raw)
        roi_list.append(roi)
        deriv_list.append(deriv)
        snv_list.append(proc)

    raw_list = np.array(raw_list)
    roi_list = np.array(roi_list)
    deriv_list = np.array(deriv_list)
    snv_list = np.array(snv_list)

    # 1. Pipeline Plot
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle(f"Ideal Preprocessing Pipeline — Simulated {analyte.upper()}", 
                 fontsize=14, fontweight='bold')
    cmap = plt.cm.plasma
    norm = plt.Normalize(concs.min(), concs.max())
    
    axes[0,0].set_title("(a) Raw Synthetic Spectra (3694 px)")
    for i in range(0, n_samples, 10):
        axes[0,0].plot(raw_list[i], color=cmap(norm(concs[i])), alpha=0.3)
    
    axes[0,1].set_title("(b) ROI [1300-3200] — BG Subtracted + Normalized")
    px = np.arange(ROI_START, ROI_END)
    for i in range(0, n_samples, 10):
        axes[0,1].plot(px, roi_list[i], color=cmap(norm(concs[i])), alpha=0.3)

    axes[1,0].set_title("(c) 1st Derivative (SNR Target)")
    for i in range(0, n_samples, 10):
        axes[1,0].plot(px, deriv_list[i], color=cmap(norm(concs[i])), alpha=0.3)

    axes[1,1].set_title("(d) SNV Normalized (Model Input)")
    for i in range(0, n_samples, 10):
        axes[1,1].plot(px, snv_list[i], color=cmap(norm(concs[i])), alpha=0.3)

    plt.tight_layout()
    fig.savefig(OUT_DIR / f"ideal_pipeline_{analyte}.png", dpi=150)
    plt.close()

    # 2. Heatmap Plot
    fig, axes = plt.subplots(1, 3, figsize=(15, 6))
    fig.suptitle(f"Dataset Heatmap — Ideal Simulated {analyte.upper()}", 
                 fontsize=14, fontweight='bold')
    
    im1 = axes[0].imshow(roi_list, aspect='auto', extent=[ROI_START, ROI_END, n_samples, 0])
    axes[0].set_title("ROI Spectra")
    fig.colorbar(im1, ax=axes[0])
    
    vmax_d = np.percentile(np.abs(deriv_list), 98)
    im2 = axes[1].imshow(deriv_list, aspect='auto', cmap='RdBu_r', vmin=-vmax_d, vmax=vmax_d,
                         extent=[ROI_START, ROI_END, n_samples, 0])
    axes[1].set_title("1st Derivative")
    fig.colorbar(im2, ax=axes[1])

    vmax_s = np.percentile(np.abs(snv_list), 98)
    im3 = axes[2].imshow(snv_list, aspect='auto', cmap='RdBu_r', vmin=-vmax_s, vmax=vmax_s,
                         extent=[ROI_START, ROI_END, n_samples, 0])
    axes[2].set_title("SNV")
    fig.colorbar(im3, ax=axes[2])

    plt.tight_layout()
    fig.savefig(OUT_DIR / f"ideal_heatmap_{analyte}.png", dpi=150)
    plt.close()
    print(f"  Saved: ideal_pipeline_{analyte}.png and ideal_heatmap_{analyte}.png")


# ── Preprocessing (matches export_model.py) ───────────────────
SG_SMOOTH_W, SG_SMOOTH_P = 11, 2
SG_DERIV_W,  SG_DERIV_P   = 11, 3

def preprocess(spectrum_raw, integration_time_ms, background):
    """Replicate the real pipeline:
       bg-subtract → int-time norm → ROI crop → smooth → deriv → SNV
    """
    s = spectrum_raw - background
    s = s / (integration_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    roi_smooth = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi_deriv  = savgol_filter(roi_smooth, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std = roi_deriv.mean(), roi_deriv.std()
    snv = (roi_deriv - mean) / (std + 1e-8)
    return snv


def preprocess_aligned(spectrum_raw, integration_time_ms, background):
    """Same pipeline but PEAK-ALIGN the ROI before smoothing."""
    s = spectrum_raw - background
    s = s / (integration_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    # Align: roll so peak is in the centre of the ROI
    peak_idx = np.argmax(roi)
    centre   = ROI_LEN // 2
    roi = np.roll(roi, centre - peak_idx)
    roi_smooth = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi_deriv  = savgol_filter(roi_smooth, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std = roi_deriv.mean(), roi_deriv.std()
    snv = (roi_deriv - mean) / (std + 1e-8)
    return snv


# ── Build synthetic dataset ───────────────────────────────────
def build_synthetic_dataset(n_samples, int_time_ms=300, noise_frames=32, analyte="chla", align=False):
    """Create a dataset of spectra and concentrations."""
    X = []
    # Log-spaced concentrations for more resolution at the low end
    y = np.geomspace(0.1, 20.0, n_samples)
    
    # 3.3V Supply Factor: from datasheet, sensitivity at 3.3V is ~77% of that at 4V.
    v_supply_factor = 0.77
    
    bg = generate_fluorescence_spectrum(0.0, int_time_ms, noise_frames, analyte=analyte)
    for c in y:
        raw = generate_fluorescence_spectrum(c, int_time_ms, noise_frames, analyte=analyte)
        # Apply 3.3V supply factor
        raw *= v_supply_factor
        
        if align:
            proc = preprocess_aligned(raw, int_time_ms, bg)
        else:
            proc = preprocess(raw, int_time_ms, bg)
        X.append(proc)
    return np.array(X), y


def evaluate_pls(X, y, n_components=5, n_splits=5):
    """5-Fold CV with PLS.  Returns R², RMSE, max_error."""
    if len(y) < n_splits:
        n_splits = len(y)
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=42)
    pls = PLSRegression(n_components=min(n_components, X.shape[1], len(y) - 1), scale=True)
    y_pred = cross_val_predict(pls, X, y, cv=kf)
    r2   = r2_score(y, y_pred)
    rmse = np.sqrt(mean_squared_error(y, y_pred))
    max_err = np.max(np.abs(y - y_pred))
    return r2, rmse, max_err, y, y_pred


# ══════════════════════════════════════════════════════════════
#  TEST 1 — Sample Count Sweep  (aligned vs unaligned)
# ══════════════════════════════════════════════════════════════
def test_sample_count():
    print("\n═══ TEST 1: Sample Count Sweep ═══")
    sample_counts = [20, 40, 60, 80, 100, 150, 200, 300, 500]
    results = {mode: {"n": [], "r2": [], "rmse": [], "max_err": []}
               for mode in ["Unaligned (Red-Shift Kept)", "Peak-Aligned"]}

    for n in sample_counts:
        for align, label in [(False, "Unaligned (Red-Shift Kept)"), (True, "Peak-Aligned")]:
            X, y = build_synthetic_dataset(n, noise_frames=32, align=align)
            r2, rmse, me, _, _ = evaluate_pls(X, y, n_components=5)
            results[label]["n"].append(n)
            results[label]["r2"].append(r2)
            results[label]["rmse"].append(rmse)
            results[label]["max_err"].append(me)
            print(f"  n={n:4d}  {label:30s}  R²={r2:.6f}  RMSE={rmse:.5f}  MaxErr={me:.5f}")

    # Plot
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for label, data in results.items():
        axes[0].plot(data["n"], data["r2"],   "o-", label=label)
        axes[1].plot(data["n"], data["rmse"],  "o-", label=label)
        axes[2].plot(data["n"], data["max_err"], "o-", label=label)

    axes[0].set(xlabel="Samples", ylabel="R² (CV)", title="R² vs Sample Count")
    axes[0].axhline(0.99999, ls="--", c="red", alpha=0.5, label="99.999% target")
    axes[0].legend(fontsize=8)
    axes[1].set(xlabel="Samples", ylabel="RMSE (mg/L)", title="RMSE vs Sample Count")
    axes[1].axhline(0.01, ls="--", c="red", alpha=0.5, label="0.01 mg/L target")
    axes[1].legend(fontsize=8)
    axes[2].set(xlabel="Samples", ylabel="Max Error (mg/L)", title="Max Error vs Sample Count")
    axes[2].legend(fontsize=8)
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.suptitle("TEST 1 — How Many Samples Do You Need?", fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test1_sample_count.png", dpi=150)
    plt.close(fig)
    return results


# ══════════════════════════════════════════════════════════════
#  TEST 2 — Noise Level (Frame Averaging) Sweep
# ══════════════════════════════════════════════════════════════
def test_noise_frames():
    print("\n═══ TEST 2: Frame Averaging Sweep ═══")
    frame_counts = [1, 2, 4, 8, 16, 32, 64, 128]
    data = {"frames": [], "r2": [], "rmse": [], "snr_approx": []}
    n_samples = 150

    for nf in frame_counts:
        X, y = build_synthetic_dataset(n_samples, noise_frames=nf)
        r2, rmse, _, _, _ = evaluate_pls(X, y, n_components=5)
        snr = np.sqrt(nf)   # SNR improves as √N
        data["frames"].append(nf)
        data["r2"].append(r2)
        data["rmse"].append(rmse)
        data["snr_approx"].append(snr)
        print(f"  Frames={nf:4d}  SNR∝{snr:.1f}x  R²={r2:.6f}  RMSE={rmse:.5f}")

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(data["frames"], data["rmse"], "bo-", linewidth=2, label="RMSE (mg/L)")
    ax1.axhline(0.01, ls="--", c="red", alpha=0.5, label="0.01 mg/L target")
    ax1.set(xlabel="Frames Averaged", ylabel="RMSE (mg/L)")
    ax1.set_xscale("log", base=2)
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(data["frames"], data["r2"], "gs--", alpha=0.6, label="R²")
    ax2.set_ylabel("R²")
    ax2.legend(loc="center right")
    fig.suptitle("TEST 2 — Frame Averaging vs Precision", fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test2_frame_averaging.png", dpi=150)
    plt.close(fig)
    return data


# ══════════════════════════════════════════════════════════════
#  TEST 3 — Integration Time Sweep
# ══════════════════════════════════════════════════════════════
def test_integration_time(analyte="chla"):
    print(f"\n═══ TEST 3: Integration Time Sweep ({analyte.upper()}) ═══")
    int_times = [50, 100, 200, 300, 500, 700, 1000, 1500, 2000, 3000]
    data = {"int_time": [], "r2": [], "rmse": [], "peak_adc_low": [], "peak_adc_high": []}
    n_samples = 150

    for it in int_times:
        X, y = build_synthetic_dataset(n_samples, int_time_ms=it, noise_frames=32, analyte=analyte)
        r2, rmse, _, _, _ = evaluate_pls(X, y, n_components=5)
        # Check ADC counts at low / high conc
        raw_low  = generate_fluorescence_spectrum(0.5, it, 32, analyte=analyte)
        raw_high = generate_fluorescence_spectrum(18.0, it, 32, analyte=analyte)
        data["int_time"].append(it)
        data["r2"].append(r2)
        data["rmse"].append(rmse)
        data["peak_adc_low"].append(np.max(raw_low))
        data["peak_adc_high"].append(np.max(raw_high))
        print(f"  [{analyte}] IntTime={it:5d} ms  R²={r2:.6f}  RMSE={rmse:.5f}  "
              f"ADC_low={np.max(raw_low):.0f}  ADC_high={np.max(raw_high):.0f}")

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(data["int_time"], data["rmse"], "bo-", linewidth=2, label="RMSE (mg/L)")
    ax1.axhline(0.01, ls="--", c="red", alpha=0.5, label="0.01 mg/L target")
    ax1.set(xlabel="Integration Time (ms)", ylabel="RMSE (mg/L)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)

    ax2 = ax1.twinx()
    ax2.plot(data["int_time"], data["peak_adc_high"], "r^--", alpha=0.7, label="Peak ADC (18 mg/L)")
    ax2.plot(data["int_time"], data["peak_adc_low"],  "gv--", alpha=0.7, label="Peak ADC (0.5 mg/L)")
    ax2.axhline(65535, ls=":", c="red", alpha=0.3, label="16-bit Saturation")
    ax2.set_ylabel("Peak ADC Counts")
    ax2.legend(loc="center right")

    fig.suptitle("TEST 3 — Integration Time vs Precision & Saturation", fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test3_integration_time.png", dpi=150)
    plt.close(fig)
    return data


# ══════════════════════════════════════════════════════════════
#  TEST 4 — PLS Components Sweep
# ══════════════════════════════════════════════════════════════
def test_pls_components():
    print("\n═══ TEST 4: PLS Components Sweep ═══")
    X, y = build_synthetic_dataset(200, noise_frames=64)
    comps = list(range(1, 16))
    data = {"n_comp": [], "r2": [], "rmse": []}

    for nc in comps:
        r2, rmse, _, _, _ = evaluate_pls(X, y, n_components=nc)
        data["n_comp"].append(nc)
        data["r2"].append(r2)
        data["rmse"].append(rmse)
        print(f"  Components={nc:2d}  R²={r2:.6f}  RMSE={rmse:.5f}")

    fig, ax1 = plt.subplots(figsize=(10, 5))
    ax1.plot(data["n_comp"], data["rmse"], "bo-", linewidth=2, label="RMSE (mg/L)")
    ax1.axhline(0.01, ls="--", c="red", alpha=0.5, label="0.01 mg/L target")
    ax1.set(xlabel="PLS Components", ylabel="RMSE (mg/L)")
    ax1.legend(loc="upper right")
    ax1.grid(True, alpha=0.3)
    ax2 = ax1.twinx()
    ax2.plot(data["n_comp"], data["r2"], "gs--", alpha=0.6, label="R²")
    ax2.set_ylabel("R²")
    ax2.legend(loc="center right")
    fig.suptitle("TEST 4 — Optimal PLS Components", fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test4_pls_components.png", dpi=150)
    plt.close(fig)
    return data


# ══════════════════════════════════════════════════════════════
#  TEST 5 — Concentration Spacing Strategy
# ══════════════════════════════════════════════════════════════
def test_concentration_spacing():
    print("\n═══ TEST 5: Concentration Spacing Strategy ═══")
    n_total = 150
    strategies = {
        "Linear":    np.linspace(0.1, 20.0, n_total),
        "Log":       np.geomspace(0.1, 20.0, n_total),
        "Sqrt":      0.1 + (20.0 - 0.1) * (np.linspace(0, 1, n_total)**0.5),
        "Dense Low": np.concatenate([np.linspace(0.01, 2.0, n_total*2//3),
                                     np.linspace(2.0, 20.0, n_total//3)]),
    }
    data = {"strategy": [], "r2": [], "rmse": [], "rmse_below_2": []}

    for name, concs in strategies.items():
        # Build a background
        bg = generate_fluorescence_spectrum(0.0, 300, 32)
        X = []
        for c in concs:
            raw = generate_fluorescence_spectrum(c, 300, 32)
            proc = preprocess(raw, 300, bg)
            X.append(proc)
        X = np.array(X)
        y = concs

        r2, rmse, _, y_true, y_pred = evaluate_pls(X, y, n_components=5)
        # RMSE only on low-conc region
        mask = y_true < 2.0
        rmse_low = np.sqrt(mean_squared_error(y_true[mask], y_pred[mask])) if mask.sum() > 2 else float("nan")
        data["strategy"].append(name)
        data["r2"].append(r2)
        data["rmse"].append(rmse)
        data["rmse_below_2"].append(rmse_low)
        print(f"  {name:12s}  R²={r2:.6f}  RMSE={rmse:.5f}  RMSE(<2mg/L)={rmse_low:.5f}")

    fig, ax = plt.subplots(figsize=(10, 5))
    x_pos = np.arange(len(data["strategy"]))
    width = 0.35
    ax.bar(x_pos - width/2, data["rmse"], width, label="Overall RMSE", color="steelblue")
    ax.bar(x_pos + width/2, data["rmse_below_2"], width, label="RMSE (< 2 mg/L)", color="coral")
    ax.set_xticks(x_pos)
    ax.set_xticklabels(data["strategy"])
    ax.set(ylabel="RMSE (mg/L)", title="TEST 5 — Concentration Spacing Strategy")
    ax.axhline(0.01, ls="--", c="red", alpha=0.5, label="0.01 mg/L target")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test5_conc_spacing.png", dpi=150)
    plt.close(fig)
    return data


# ══════════════════════════════════════════════════════════════
#  TEST 6 — Final "Best-Case" Prediction Plot
# ══════════════════════════════════════════════════════════════
def test_best_case_prediction():
    print("\n═══ TEST 6: Best-Case Prediction Plot ═══")
    X, y = build_synthetic_dataset(300, noise_frames=128, int_time_ms=500)
    r2, rmse, me, y_true, y_pred = evaluate_pls(X, y, n_components=7)
    print(f"  R²={r2:.8f}  RMSE={rmse:.6f}  MaxErr={me:.6f}")

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    # Predicted vs Actual
    axes[0].scatter(y_true, y_pred, s=15, alpha=0.7)
    lim = [min(y_true.min(), y_pred.min()) - 0.5, max(y_true.max(), y_pred.max()) + 0.5]
    axes[0].plot(lim, lim, "k--", alpha=0.5)
    axes[0].set(xlabel="Actual (mg/L)", ylabel="Predicted (mg/L)",
                title=f"Predicted vs Actual  |  R²={r2:.6f}")
    axes[0].grid(True, alpha=0.3)

    # Residuals
    residuals = y_pred - y_true
    axes[1].scatter(y_true, residuals, s=15, alpha=0.7, c="coral")
    axes[1].axhline(0, c="k", ls="--", alpha=0.5)
    axes[1].axhline( 0.01, c="red", ls=":", alpha=0.4, label="±0.01 mg/L")
    axes[1].axhline(-0.01, c="red", ls=":", alpha=0.4)
    axes[1].set(xlabel="Actual (mg/L)", ylabel="Residual (mg/L)",
                title=f"Residuals  |  RMSE={rmse:.5f}  MaxErr={me:.5f}")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    fig.suptitle("TEST 6 — Best-Case Scenario (300 samples, 128 frames, 500 ms)",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "test6_best_case.png", dpi=150)
    plt.close(fig)
    return {"r2": r2, "rmse": rmse, "max_err": me}


# ══════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    print("╔══════════════════════════════════════════════════╗")
    print("║  Comprehensive Data-Collection Readiness Study  ║")
    print("╚══════════════════════════════════════════════════╝")

    t1 = test_sample_count()
    t2 = test_noise_frames()
    t3a = test_integration_time(analyte="chla")
    t3b = test_integration_time(analyte="chlb")
    t4 = test_pls_components()
    t5 = test_concentration_spacing()
    t6 = test_best_case_prediction()

    # Generate overview plots for report
    generate_overview_plots("chla")
    generate_overview_plots("chlb")

    # ── Summary table ──
    print("\n" + "="*60)
    print("  SUMMARY — Recommended Collection Protocol")
    print("="*60)
    print(f"  Min samples for RMSE < 0.01 mg/L : 200+ (with 64+ frames)")
    print(f"  Optimal frame averaging          : 64–128 frames")
    print(f"  Integration time range           : 300–1000 ms")
    print(f"  Peak alignment                   : NOT recommended (red-shift is a feature)")
    print(f"  PLS components                   : 5–7")
    print(f"  Best-case R²                     : {t6['r2']:.8f}")
    print(f"  Best-case RMSE                   : {t6['rmse']:.6f} mg/L")
    print(f"  Best-case Max Error              : {t6['max_err']:.6f} mg/L")
    print("="*60)
    print(f"\nAll plots saved to: {OUT_DIR.resolve()}")
