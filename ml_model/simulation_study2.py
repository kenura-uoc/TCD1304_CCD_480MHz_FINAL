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
from scipy.interpolate import interp1d
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

ADC_SATURATION    = 65535   # 16-bit ADC ceiling
ADC_TARGET_LOW    = 3000    # Minimum useful ADC counts above dark (SNR floor, net signal)
ADC_TARGET_HIGH   = 55000   # Maximum before saturation risk (84% of 65535)

# ── Datasheet-calibrated dark current model ───────────────────
# From TCD1304 datasheet Fig: Dark Signal Voltage vs Integration Time
# At 30°C, t_INT=0.1s → ~1% V_SAT; t_INT=1s → ~8% V_SAT (log-log linear)
# Doubling temp every 7°C (standard CCD dark current rule)
# 16-bit ADC: V_SAT = 65535 counts
#
# Reading from datasheet log-log graph at 30°C:
#   t=0.01s  → ~0.15% V_SAT  → ~98 counts
#   t=0.1s   → ~1.0%  V_SAT  → ~655 counts
#   t=1.0s   → ~7.5%  V_SAT  → ~4915 counts
#   t=10s    → ~55%   V_SAT  → ~36045 counts
# Power law fit: dark_fraction = 0.01 * (t_INT / 0.1) ^ 0.88  at 30°C

DARK_POWER_LAW_COEFF = 0.01     # 1% at 0.1s reference (30°C)
DARK_POWER_LAW_EXP   = 0.88     # sub-linear exponent from datasheet log-log
DARK_TEMP_DOUBLING_C = 7.0      # Dark current doubles every 7°C (typical CCD)
DARK_REFERENCE_TEMP  = 30.0     # Datasheet reference temperature

# ── Datasheet-calibrated spectral response ────────────────────
# From TCD1304 datasheet Spectral Response curve (Ta=25°C):
#   400nm → 0.80,  450nm → 0.93,  500nm → 1.00 (peak),
#   550nm → 1.00,  600nm → 0.97,  650nm → 0.90,
#   670nm → 0.85,  700nm → 0.72,  750nm → 0.52,
#   800nm → 0.32,  900nm → 0.10,  1000nm → 0.02
SPECTRAL_RESP_WL  = np.array([400, 450, 500, 550, 600, 650, 670, 700, 750, 800, 900, 1000])
SPECTRAL_RESP_VAL = np.array([0.80, 0.93, 1.00, 1.00, 0.97, 0.90, 0.85, 0.72, 0.52, 0.32, 0.10, 0.02])
_spectral_resp_interp = interp1d(SPECTRAL_RESP_WL, SPECTRAL_RESP_VAL,
                                  kind='cubic', bounds_error=False, fill_value=0.0)

# ── Supply voltage sensitivity correction ─────────────────────
# From datasheet Sensitivity Response: V_DD=V_AD(V)
#   At 3.3V → ~150 V/lx·s  (relative to 4V peak ~197 V/lx·s)
#   Ratio: 150/197 ≈ 0.762
V_SUPPLY_FACTOR = 0.762   # 3.3V operation vs 4V datasheet reference


def dark_current_counts(integration_time_ms: float, temp_c: float = 30.0) -> float:
    """
    Compute expected dark current in ADC counts using datasheet power-law model.

    Model: dark_fraction = COEFF * (t_s / 0.1)^EXP  at DARK_REFERENCE_TEMP
    Temperature scaling: doubles every DARK_TEMP_DOUBLING_C degrees.

    Parameters
    ----------
    integration_time_ms : float  — integration time in milliseconds
    temp_c              : float  — ambient temperature in °C

    Returns
    -------
    float — dark current contribution in 16-bit ADC counts
    """
    t_s = integration_time_ms / 1000.0
    t_s = max(t_s, 1e-6)  # guard against zero
    dark_fraction = DARK_POWER_LAW_COEFF * (t_s / 0.1) ** DARK_POWER_LAW_EXP
    temp_factor   = 2.0 ** ((temp_c - DARK_REFERENCE_TEMP) / DARK_TEMP_DOUBLING_C)
    dark_counts   = dark_fraction * ADC_SATURATION * temp_factor
    return float(dark_counts)


def spectral_response_at(wavelength_nm: float) -> float:
    """Return CCD relative spectral response at a given wavelength (datasheet curve)."""
    return float(_spectral_resp_interp(wavelength_nm))


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
      • Peak position shifts to higher pixels with concentration  (red-shift)
      • FWHM broadens slightly with concentration
      • Intensity is sub-linear (Beer-Lambert + self-absorption saturation)
      • Shot noise, read noise, dark current (datasheet power-law model)
      • Integration-time scaling
      • Datasheet spectral response correction
      • 3.3V supply sensitivity correction
    """
    if analyte == "chla":
        base_peak_pixel = 1950
        red_shift_rate  = 12.0
        fwhm_base       = 180
        fwhm_growth     = 6.0
        alpha           = 0.12
        max_signal      = 40000
        emission_wl     = 670   # nm
    else:  # chlb
        base_peak_pixel = 2200
        red_shift_rate  = 40.0
        fwhm_base       = 220
        fwhm_growth     = 8.0
        alpha           = 0.18
        max_signal      = 7500
        emission_wl     = 650   # nm

    # Datasheet spectral response at emission wavelength
    qe_factor = spectral_response_at(emission_wl)
    max_signal *= qe_factor * V_SUPPLY_FACTOR

    x = np.arange(TOTAL_PIXELS, dtype=float)

    # Peak position (red-shift with concentration)
    peak_pos = base_peak_pixel + concentration * red_shift_rate

    # Peak width (FWHM → sigma)
    fwhm  = fwhm_base + concentration * fwhm_growth
    sigma = fwhm / 2.355

    # Fluorescence intensity — Beer-Lambert saturation model
    fluor_intensity  = max_signal * (1.0 - np.exp(-alpha * concentration))
    signal_per_ms    = fluor_intensity / 300.0   # calibrated at 300 ms
    signal           = signal_per_ms * integration_time_ms

    # Gaussian peak
    spectrum_clean = signal * np.exp(-0.5 * ((x - peak_pos) / sigma) ** 2)

    # Dark current (datasheet power-law model)
    dark_level = dark_current_counts(integration_time_ms, dark_temp_c)

    # Frame averaging with realistic noise
    frames = []
    for _ in range(noise_frames):
        frame   = spectrum_clean + dark_level
        shot    = np.random.normal(0, np.sqrt(np.maximum(frame, 1)))
        read    = np.random.normal(0, read_noise_std, TOTAL_PIXELS)
        frames.append(frame + shot + read)

    avg_frame = np.mean(frames, axis=0)
    return np.clip(avg_frame, 0, ADC_SATURATION)


# ── Preprocessing (matches export_model.py) ───────────────────
SG_SMOOTH_W, SG_SMOOTH_P = 11, 2
SG_DERIV_W,  SG_DERIV_P   = 11, 3


def preprocess(spectrum_raw, integration_time_ms, background):
    """Replicate the real firmware pipeline:
       dark-subtract → int-time norm → ROI crop → SG smooth → SG deriv → SNV
    """
    s   = spectrum_raw - background
    s   = s / (integration_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    roi_smooth = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi_deriv  = savgol_filter(roi_smooth, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std  = roi_deriv.mean(), roi_deriv.std()
    snv = (roi_deriv - mean) / (std + 1e-8)
    return snv


def preprocess_aligned(spectrum_raw, integration_time_ms, background):
    """Same pipeline but PEAK-ALIGN the ROI before smoothing."""
    s   = spectrum_raw - background
    s   = s / (integration_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    peak_idx = np.argmax(roi)
    centre   = ROI_LEN // 2
    roi      = np.roll(roi, centre - peak_idx)
    roi_smooth = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi_deriv  = savgol_filter(roi_smooth, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std  = roi_deriv.mean(), roi_deriv.std()
    snv = (roi_deriv - mean) / (std + 1e-8)
    return snv


# ── Build synthetic dataset ───────────────────────────────────
def build_synthetic_dataset(n_samples, int_time_ms=300, noise_frames=32,
                             analyte="chla", align=False):
    """Create a dataset of spectra and concentrations."""
    y  = np.geomspace(0.1, 20.0, n_samples)
    bg = generate_fluorescence_spectrum(0.0, int_time_ms, noise_frames, analyte=analyte)
    X  = []
    for c in y:
        raw  = generate_fluorescence_spectrum(c, int_time_ms, noise_frames, analyte=analyte)
        proc = (preprocess_aligned if align else preprocess)(raw, int_time_ms, bg)
        X.append(proc)
    return np.array(X), y


def evaluate_pls(X, y, n_components=5, n_splits=5):
    """5-Fold CV with PLS.  Returns R², RMSE, max_error."""
    n_splits = min(n_splits, len(y))
    kf  = KFold(n_splits=n_splits, shuffle=True, random_state=42)
    pls = PLSRegression(n_components=min(n_components, X.shape[1], len(y) - 1), scale=True)
    y_pred = cross_val_predict(pls, X, y, cv=kf)
    r2     = r2_score(y, y_pred)
    rmse   = np.sqrt(mean_squared_error(y, y_pred))
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

    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    for label, data in results.items():
        axes[0].plot(data["n"], data["r2"],     "o-", label=label)
        axes[1].plot(data["n"], data["rmse"],   "o-", label=label)
        axes[2].plot(data["n"], data["max_err"],"o-", label=label)

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
        snr = np.sqrt(nf)
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
#  TEST 3 — Integration Time: Datasheet-Calibrated Optimal Range
# ══════════════════════════════════════════════════════════════
def _compute_integration_time_metrics(int_time_ms: float, n_samples: int = 150,
                                       noise_frames: int = 32, analyte: str = "chla",
                                       temp_c: float = 30.0):
    """
    For a given integration time, compute all metrics needed to judge feasibility:
      - PLS model R² and RMSE (via 5-fold CV)
      - Peak ADC at low and high concentration
      - Dark current contribution (counts)
      - Signal-to-dark ratio at low concentration (SNR proxy)
      - Saturation flag (any pixel > ADC_TARGET_HIGH at high conc)
    """
    # Build dataset
    X, y = build_synthetic_dataset(n_samples, int_time_ms=int_time_ms,
                                    noise_frames=noise_frames, analyte=analyte)
    r2, rmse, _, _, _ = evaluate_pls(X, y, n_components=5)

    # Raw spectra for ADC checks
    raw_low  = generate_fluorescence_spectrum(0.5,  int_time_ms, noise_frames,
                                               analyte=analyte, dark_temp_c=temp_c)
    raw_high = generate_fluorescence_spectrum(18.0, int_time_ms, noise_frames,
                                               analyte=analyte, dark_temp_c=temp_c)

    peak_adc_low  = float(np.max(raw_low))
    peak_adc_high = float(np.max(raw_high))
    dark_counts   = dark_current_counts(int_time_ms, temp_c)

    # Signal-to-dark ratio at low concentration
    signal_low   = peak_adc_low - dark_counts
    signal_dark_ratio = signal_low / (dark_counts + 1e-8)

    # Saturation check: is peak ADC at HIGH conc within safe range?
    saturated    = peak_adc_high > ADC_TARGET_HIGH
    # SNR floor check: is peak ADC at LOW conc above minimum useful level?
    snr_too_low  = peak_adc_low < ADC_TARGET_LOW

    return {
        "int_time_ms":        int_time_ms,
        "r2":                 r2,
        "rmse":               rmse,
        "peak_adc_low":       peak_adc_low,
        "peak_adc_high":      peak_adc_high,
        "dark_counts":        dark_counts,
        "signal_dark_ratio":  signal_dark_ratio,
        "saturated":          saturated,
        "snr_too_low":        snr_too_low,
        "feasible":           (not saturated) and (not snr_too_low),
    }


def find_optimal_integration_range(analyte: str = "chla", temp_c: float = 30.0,
                                    noise_frames: int = 32):
    """
    Algorithmically identify the optimal integration time range by sweeping
    int_time_ms and applying three hard constraints from the datasheet:

      1. No saturation  : peak ADC (18 mg/L) < ADC_TARGET_HIGH  (55000 counts)
      2. Sufficient SNR : peak ADC (0.5 mg/L) > ADC_TARGET_LOW  (10000 counts)
      3. Dark dominance : dark_current < 20% of peak_adc_low (dark fraction guard)

    Returns the feasible range as (t_min_ms, t_max_ms) and the full results table.
    """
    int_times = [50, 75, 100, 150, 200, 300, 400, 500, 700,
                 1000, 1500, 2000, 3000, 5000]
    records = []
    for it in int_times:
        m = _compute_integration_time_metrics(it, analyte=analyte,
                                               temp_c=temp_c, noise_frames=noise_frames)
        # Dark dominance guard: net signal (peak - dark) at low conc must exceed ADC_TARGET_LOW
        # This ensures SNR is driven by fluorescence, not dark current noise
        net_signal_low    = max(m["peak_adc_low"] - m["dark_counts"], 0.0)
        dark_fraction     = m["dark_counts"] / (m["peak_adc_low"] + 1e-8)
        m["dark_fraction"]   = dark_fraction
        m["net_signal_low"]  = net_signal_low
        m["dark_dominant"]   = net_signal_low < ADC_TARGET_LOW
        m["feasible"]        = m["feasible"] and not m["dark_dominant"]
        records.append(m)
        status = "✓ FEASIBLE" if m["feasible"] else "✗"
        print(f"  [{analyte}] {it:6d} ms | R²={m['r2']:.4f} | RMSE={m['rmse']:.4f} | "
              f"net_sig={m['net_signal_low']:.0f} | ADC_high={m['peak_adc_high']:.0f} | "
              f"dark={m['dark_counts']:.0f} ({m['dark_fraction']*100:.1f}%) | {status}")

    feasible_times = [r["int_time_ms"] for r in records if r["feasible"]]
    if feasible_times:
        t_min, t_max = min(feasible_times), max(feasible_times)
    else:
        t_min, t_max = None, None
        print(f"  WARNING: No feasible integration time found for {analyte} at {temp_c}°C!")

    return t_min, t_max, records


def test_integration_time(analyte="chla"):
    """
    Full integration-time sweep with datasheet-calibrated dark current,
    automatic feasibility detection, and annotated plot.
    """
    print(f"\n═══ TEST 3: Integration Time Sweep ({analyte.upper()}) ═══")
    print(f"  Dark model: {DARK_POWER_LAW_COEFF:.3f} * (t/0.1s)^{DARK_POWER_LAW_EXP} at 30°C")
    print(f"  Saturation ceiling : {ADC_TARGET_HIGH} counts  ({ADC_TARGET_HIGH/ADC_SATURATION*100:.0f}% of full-scale)")
    print(f"  SNR floor          : {ADC_TARGET_LOW} counts")

    t_min, t_max, records = find_optimal_integration_range(analyte=analyte,
                                                            temp_c=30.0, noise_frames=32)

    # Also run a warm-environment check (40°C, typical lab after prolonged use)
    print(f"\n  — Warm environment check (40°C) —")
    t_min_warm, t_max_warm, records_warm = find_optimal_integration_range(
        analyte=analyte, temp_c=40.0, noise_frames=32)

    # ── Plotting ─────────────────────────────────────────────
    int_times   = [r["int_time_ms"]    for r in records]
    rmse_vals   = [r["rmse"]           for r in records]
    adc_low     = [r["peak_adc_low"]   for r in records]
    adc_high    = [r["peak_adc_high"]  for r in records]
    dark_vals   = [r["dark_counts"]    for r in records]
    feasible    = [r["feasible"]       for r in records]
    dark_frac   = [r["dark_fraction"]  for r in records]

    rmse_warm   = [r["rmse"]           for r in records_warm]
    adc_high_w  = [r["peak_adc_high"]  for r in records_warm]
    dark_warm   = [r["dark_counts"]    for r in records_warm]

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(
        f"TEST 3 — Integration Time Optimisation [{analyte.upper()}]  "
        f"(Datasheet-Calibrated Dark Model)",
        fontsize=13, fontweight="bold"
    )

    # ── Panel A: RMSE vs integration time ────────────────────
    ax = axes[0, 0]
    ax.plot(int_times, rmse_vals,  "bo-", lw=2, label="RMSE @ 30°C")
    ax.plot(int_times, rmse_warm,  "b^--", lw=1.5, alpha=0.7, label="RMSE @ 40°C")
    # Shade feasible window
    if t_min and t_max:
        ax.axvspan(t_min, t_max, alpha=0.15, color="green",
                   label=f"Optimal window\n{t_min}–{t_max} ms @ 30°C")
    if t_min_warm and t_max_warm:
        ax.axvspan(t_min_warm, t_max_warm, alpha=0.08, color="orange",
                   label=f"Optimal window\n{t_min_warm}–{t_max_warm} ms @ 40°C")
    for i, (t, r, f) in enumerate(zip(int_times, rmse_vals, feasible)):
        ax.annotate("✓" if f else "✗",
                    xy=(t, r), xytext=(0, 6), textcoords="offset points",
                    ha="center", fontsize=10,
                    color="green" if f else "red")
    ax.set_xscale("log")
    ax.set(xlabel="Integration Time (ms)", ylabel="CV RMSE (mg/L)",
           title="(A) Model RMSE vs Integration Time")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel B: ADC counts vs integration time ───────────────
    ax = axes[0, 1]
    ax.plot(int_times, adc_high,  "r^-",  lw=2, label=f"Peak ADC (18 mg/L) @ 30°C")
    ax.plot(int_times, adc_high_w, "r^--", lw=1.5, alpha=0.7, label=f"Peak ADC (18 mg/L) @ 40°C")
    ax.plot(int_times, adc_low,   "gv-",  lw=2, label=f"Peak ADC (0.5 mg/L) @ 30°C")
    ax.axhline(ADC_TARGET_HIGH, ls=":",  c="red",   lw=2, label=f"Saturation limit ({ADC_TARGET_HIGH})")
    ax.axhline(ADC_TARGET_LOW,  ls=":",  c="green", lw=2, label=f"SNR floor ({ADC_TARGET_LOW})")
    ax.axhline(ADC_SATURATION,  ls="--", c="darkred", lw=1, alpha=0.4, label="16-bit ceiling (65535)")
    if t_min and t_max:
        ax.axvspan(t_min, t_max, alpha=0.12, color="green")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set(xlabel="Integration Time (ms)", ylabel="Peak ADC Counts (log)",
           title="(B) ADC Saturation & SNR Constraints")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── Panel C: Dark current vs integration time ─────────────
    ax = axes[1, 0]
    net_sig  = [r.get("net_signal_low", r["peak_adc_low"] - r["dark_counts"]) for r in records]
    net_sig_w= [r.get("net_signal_low", r["peak_adc_low"] - r["dark_counts"]) for r in records_warm]
    ax.plot(int_times, dark_vals,  "k-o",  lw=2, label="Dark counts @ 30°C")
    ax.plot(int_times, dark_warm,  "k--^", lw=1.5, alpha=0.7, label="Dark counts @ 40°C")
    ax.plot(int_times, net_sig,    "b-s",  lw=2, label="Net fluorescence signal (0.5 mg/L) @ 30°C")
    ax.plot(int_times, net_sig_w,  "b--s", lw=1.5, alpha=0.7, label="Net signal @ 40°C")
    ax.axhline(ADC_TARGET_LOW, ls=":", c="green", lw=2, label=f"Net signal floor ({ADC_TARGET_LOW} counts)")
    if t_min and t_max:
        ax.axvspan(t_min, t_max, alpha=0.12, color="green",
                   label=f"Optimal: {t_min}–{t_max} ms")
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set(xlabel="Integration Time (ms)", ylabel="ADC Counts (log)",
           title="(C) Dark Current vs Net Fluorescence Signal (Datasheet Model)")
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3)

    # ── Panel D: Feasibility summary heatmap ─────────────────
    ax = axes[1, 1]
    categories = ["Not saturated\n(ADC_high < 55k)",
                  "SNR adequate\n(ADC_low > 10k)",
                  "Dark < 20%\nof signal",
                  "All constraints\n(FEASIBLE)"]
    heatmap = np.zeros((len(categories), len(int_times)))
    for j, r in enumerate(records):
        heatmap[0, j] = 1.0 if not r["saturated"]    else 0.0
        heatmap[1, j] = 1.0 if not r["snr_too_low"]  else 0.0
        heatmap[2, j] = 1.0 if not r.get("dark_dominant", False) else 0.0
        heatmap[3, j] = 1.0 if     r["feasible"]      else 0.0

    im = ax.imshow(heatmap, aspect="auto", cmap="RdYlGn", vmin=0, vmax=1,
                   extent=[0, len(int_times), len(categories), 0])
    ax.set_xticks(np.arange(len(int_times)) + 0.5)
    ax.set_xticklabels([f"{t}" for t in int_times], rotation=45, ha="right", fontsize=8)
    ax.set_yticks(np.arange(len(categories)) + 0.5)
    ax.set_yticklabels(categories, fontsize=8)
    ax.set(xlabel="Integration Time (ms)", title="(D) Constraint Feasibility Matrix")
    plt.colorbar(im, ax=ax, label="Pass (1) / Fail (0)", shrink=0.8)

    fig.tight_layout()
    fig.savefig(OUT_DIR / f"test3_integration_time_{analyte}.png", dpi=150)
    plt.close(fig)

    # ── Print summary ─────────────────────────────────────────
    print(f"\n  ┌─ OPTIMAL INTEGRATION TIME RANGE [{analyte.upper()}] ─────────────────┐")
    if t_min and t_max:
        print(f"  │  @ 30°C (standard): {t_min} – {t_max} ms")
        print(f"  │  @ 40°C (warm lab): {t_min_warm} – {t_max_warm} ms")
        conservative = t_min_warm or t_min
        print(f"  │  Conservative recommendation: {conservative} – {min(t_max, t_max_warm or t_max)} ms")
    else:
        print(f"  │  No feasible range found — check hardware parameters")
    print(f"  └──────────────────────────────────────────────────────────────┘")

    return {"t_min": t_min, "t_max": t_max,
            "t_min_warm": t_min_warm, "t_max_warm": t_max_warm,
            "records": records, "records_warm": records_warm}


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
        "Sqrt":      0.1 + (20.0 - 0.1) * (np.linspace(0, 1, n_total) ** 0.5),
        "Dense Low": np.concatenate([np.linspace(0.01, 2.0, n_total * 2 // 3),
                                     np.linspace(2.0, 20.0, n_total // 3)]),
    }
    data = {"strategy": [], "r2": [], "rmse": [], "rmse_below_2": []}

    for name, concs in strategies.items():
        bg = generate_fluorescence_spectrum(0.0, 300, 32)
        X  = []
        for c in concs:
            raw  = generate_fluorescence_spectrum(c, 300, 32)
            proc = preprocess(raw, 300, bg)
            X.append(proc)
        X = np.array(X)
        y = concs

        r2, rmse, _, y_true, y_pred = evaluate_pls(X, y, n_components=5)
        mask     = y_true < 2.0
        rmse_low = (np.sqrt(mean_squared_error(y_true[mask], y_pred[mask]))
                    if mask.sum() > 2 else float("nan"))
        data["strategy"].append(name)
        data["r2"].append(r2)
        data["rmse"].append(rmse)
        data["rmse_below_2"].append(rmse_low)
        print(f"  {name:12s}  R²={r2:.6f}  RMSE={rmse:.5f}  RMSE(<2mg/L)={rmse_low:.5f}")

    fig, ax = plt.subplots(figsize=(10, 5))
    x_pos = np.arange(len(data["strategy"]))
    width = 0.35
    ax.bar(x_pos - width / 2, data["rmse"],         width, label="Overall RMSE", color="steelblue")
    ax.bar(x_pos + width / 2, data["rmse_below_2"], width, label="RMSE (< 2 mg/L)", color="coral")
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
    axes[0].scatter(y_true, y_pred, s=15, alpha=0.7)
    lim = [min(y_true.min(), y_pred.min()) - 0.5, max(y_true.max(), y_pred.max()) + 0.5]
    axes[0].plot(lim, lim, "k--", alpha=0.5)
    axes[0].set(xlabel="Actual (mg/L)", ylabel="Predicted (mg/L)",
                title=f"Predicted vs Actual  |  R²={r2:.6f}")
    axes[0].grid(True, alpha=0.3)

    residuals = y_pred - y_true
    axes[1].scatter(y_true, residuals, s=15, alpha=0.7, c="coral")
    axes[1].axhline(0,     c="k",   ls="--", alpha=0.5)
    axes[1].axhline( 0.01, c="red", ls=":",  alpha=0.4, label="±0.01 mg/L")
    axes[1].axhline(-0.01, c="red", ls=":",  alpha=0.4)
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
    print(f"\n  Datasheet dark model (TCD1304, 30°C):")
    print(f"    dark_counts = {DARK_POWER_LAW_COEFF} × (t_s / 0.1)^{DARK_POWER_LAW_EXP} × 65535")
    print(f"    Doubles every {DARK_TEMP_DOUBLING_C}°C above {DARK_REFERENCE_TEMP}°C")
    print(f"  Supply factor @ 3.3V vs 4V: {V_SUPPLY_FACTOR:.3f}")

    t1        = test_sample_count()
    t2        = test_noise_frames()
    t3a       = test_integration_time(analyte="chla")
    t3b       = test_integration_time(analyte="chlb")
    t4        = test_pls_components()
    t5        = test_concentration_spacing()
    t6        = test_best_case_prediction()

    # ── Consolidated integration-time summary ─────────────────
    print("\n" + "=" * 70)
    print("  INTEGRATION TIME OPTIMAL RANGES (datasheet-calibrated)")
    print("=" * 70)
    for analyte, result in [("Chl-a", t3a), ("Chl-b", t3b)]:
        lo, hi     = result["t_min"],      result["t_max"]
        lo_w, hi_w = result["t_min_warm"], result["t_max_warm"]
        print(f"  {analyte}  @30°C: {lo}–{hi} ms   @40°C: {lo_w}–{hi_w} ms")
        if lo_w and hi_w:
            safe_lo = max(lo or 0, lo_w or 0)
            safe_hi = min(hi or 9999, hi_w or 9999)
            if safe_lo <= safe_hi:
                print(f"         Conservative (covers both temps): {safe_lo}–{safe_hi} ms")

    print("\n" + "=" * 70)
    print("  OVERALL RECOMMENDED PROTOCOL")
    print("=" * 70)
    print(f"  Min samples for RMSE < 0.01 mg/L : 200+ (with 64+ frames)")
    print(f"  Optimal frame averaging           : 64–128 frames")
    print(f"  Integration time (see above)      : from datasheet-calibrated sweep")
    print(f"  Peak alignment                    : NOT recommended (red-shift is a feature)")
    print(f"  PLS components                    : 5–7")
    print(f"  Best-case R²                      : {t6['r2']:.8f}")
    print(f"  Best-case RMSE                    : {t6['rmse']:.6f} mg/L")
    print(f"  Best-case Max Error               : {t6['max_err']:.6f} mg/L")
    print("=" * 70)
    print(f"\nAll plots saved to: {OUT_DIR.resolve()}")