#!/usr/bin/env python3
"""
visualize_spectra.py — Generate visual overviews of synthetic spectra
=====================================================================
Shows how spectra vary with concentration, preprocessed features,
and a dataset-wide heatmap for pattern checking.
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import cm
from pathlib import Path
from scipy.signal import savgol_filter

OUT_DIR = Path(__file__).parent / "cnn_results"
OUT_DIR.mkdir(exist_ok=True)

TOTAL_PIXELS = 3694
ROI_START, ROI_END = 1300, 3200
ROI_LEN = ROI_END - ROI_START
SG_SMOOTH_W, SG_SMOOTH_P = 11, 2
SG_DERIV_W,  SG_DERIV_P  = 11, 3


def generate_spectrum(concentration, int_time_ms=300, noise_frames=64):
    x = np.arange(TOTAL_PIXELS, dtype=float)
    peak_pos = 1950 + concentration * 12.0
    sigma = (180 + concentration * 6.0) / 2.355
    max_signal = 3200
    signal = (max_signal * (1.0 - np.exp(-0.12 * concentration)) / 300.0) * int_time_ms
    spectrum_clean = signal * np.exp(-0.5 * ((x - peak_pos) / sigma)**2)
    frames = []
    for _ in range(noise_frames):
        frame = spectrum_clean + 200
        frame += np.random.normal(0, np.sqrt(np.maximum(frame, 1)))
        frame += np.random.normal(0, 15, TOTAL_PIXELS)
        frames.append(frame)
    return np.mean(frames, axis=0)


def preprocess(spectrum, int_time_ms, background):
    s = (spectrum - background) / (int_time_ms + 1e-8)
    roi = s[ROI_START:ROI_END]
    roi_smooth = savgol_filter(roi, SG_SMOOTH_W, SG_SMOOTH_P)
    roi_deriv = savgol_filter(roi_smooth, SG_DERIV_W, SG_DERIV_P, deriv=1)
    mean, std = roi_deriv.mean(), roi_deriv.std()
    snv = (roi_deriv - mean) / (std + 1e-8)
    return roi, roi_smooth, roi_deriv, snv


if __name__ == "__main__":
    np.random.seed(42)
    N = 300
    concs = np.linspace(0.1, 20.0, N)
    bg = generate_spectrum(0.0, 300, 64)

    # Collect all stages
    raw_spectra = []
    roi_spectra = []
    smoothed_spectra = []
    deriv_spectra = []
    snv_spectra = []

    print("Generating spectra...")
    for c in concs:
        spec = generate_spectrum(c, 300, 64)
        raw_spectra.append(spec)
        roi, smooth, deriv, snv = preprocess(spec, 300, bg)
        roi_spectra.append(roi)
        smoothed_spectra.append(smooth)
        deriv_spectra.append(deriv)
        snv_spectra.append(snv)

    raw_spectra = np.array(raw_spectra)
    roi_spectra = np.array(roi_spectra)
    smoothed_spectra = np.array(smoothed_spectra)
    deriv_spectra = np.array(deriv_spectra)
    snv_spectra = np.array(snv_spectra)

    pixels_full = np.arange(TOTAL_PIXELS)
    pixels_roi = np.arange(ROI_START, ROI_END)

    # ── Color map for concentration ──
    cmap = cm.plasma
    norm = plt.Normalize(concs.min(), concs.max())

    # ═══════════════════════════════════════════════════════
    # PLOT 1: All raw spectra overlaid, colored by concentration
    # ═══════════════════════════════════════════════════════
    print("Plot 1: Raw spectra overview...")
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))

    # Top-left: Raw full spectra (every 10th for clarity)
    ax = axes[0, 0]
    for i in range(0, N, 5):
        ax.plot(pixels_full, raw_spectra[i], color=cmap(norm(concs[i])),
                alpha=0.5, linewidth=0.5)
    ax.axvspan(ROI_START, ROI_END, alpha=0.05, color="green", label="ROI")
    ax.axvline(ROI_START, c="green", ls="--", alpha=0.3)
    ax.axvline(ROI_END, c="green", ls="--", alpha=0.3)
    ax.set(xlabel="Pixel", ylabel="ADC Counts", title="(a) Raw CCD Spectra (3694 px)")
    ax.set_xlim(0, TOTAL_PIXELS)
    ax.grid(True, alpha=0.2)

    # Top-right: ROI only (bg-subtracted, int-time normalized)
    ax = axes[0, 1]
    for i in range(0, N, 5):
        ax.plot(pixels_roi, roi_spectra[i], color=cmap(norm(concs[i])),
                alpha=0.5, linewidth=0.5)
    ax.set(xlabel="Pixel", ylabel="Norm. Intensity",
           title="(b) ROI [1300-3200] — BG Subtracted + Normalized")
    ax.grid(True, alpha=0.2)

    # Bottom-left: 1st Derivative
    ax = axes[1, 0]
    for i in range(0, N, 5):
        ax.plot(pixels_roi, deriv_spectra[i], color=cmap(norm(concs[i])),
                alpha=0.5, linewidth=0.5)
    ax.set(xlabel="Pixel", ylabel="1st Derivative",
           title="(c) Savitzky-Golay 1st Derivative")
    ax.grid(True, alpha=0.2)

    # Bottom-right: SNV normalized
    ax = axes[1, 1]
    for i in range(0, N, 5):
        ax.plot(pixels_roi, snv_spectra[i], color=cmap(norm(concs[i])),
                alpha=0.5, linewidth=0.5)
    ax.set(xlabel="Pixel", ylabel="SNV",
           title="(d) SNV Normalized (Model Input)")
    ax.grid(True, alpha=0.2)

    # Colorbar
    sm = cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=axes.ravel().tolist(), shrink=0.6, pad=0.02)
    cbar.set_label("Concentration (ppm)", fontsize=11)

    fig.suptitle("Preprocessing Pipeline — How Spectra Change with Concentration",
                 fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 0.92, 0.95])
    fig.savefig(OUT_DIR / "spectra_overview_pipeline.png", dpi=150)
    plt.close(fig)
    print(f"  Saved: {OUT_DIR / 'spectra_overview_pipeline.png'}")

    # ═══════════════════════════════════════════════════════
    # PLOT 2: Heatmap — Record Index (y) vs Pixel (x)
    # ═══════════════════════════════════════════════════════
    print("Plot 2: Dataset heatmap (record vs pixel)...")
    fig, axes = plt.subplots(1, 3, figsize=(18, 8))

    # ROI spectra heatmap
    ax = axes[0]
    im = ax.imshow(roi_spectra, aspect="auto", cmap="viridis",
                   extent=[ROI_START, ROI_END, N, 0])
    ax.set(xlabel="Pixel", ylabel="Record # (sorted by conc.)",
           title="ROI Spectra (BG-sub + norm)")
    fig.colorbar(im, ax=ax, shrink=0.7, label="Intensity")

    # Derivative heatmap
    ax = axes[1]
    im = ax.imshow(deriv_spectra, aspect="auto", cmap="RdBu_r",
                   extent=[ROI_START, ROI_END, N, 0],
                   vmin=-np.percentile(np.abs(deriv_spectra), 95),
                   vmax=np.percentile(np.abs(deriv_spectra), 95))
    ax.set(xlabel="Pixel", ylabel="Record #",
           title="1st Derivative")
    fig.colorbar(im, ax=ax, shrink=0.7, label="d/dx")

    # SNV heatmap
    ax = axes[2]
    im = ax.imshow(snv_spectra, aspect="auto", cmap="RdBu_r",
                   extent=[ROI_START, ROI_END, N, 0],
                   vmin=-3, vmax=3)
    ax.set(xlabel="Pixel", ylabel="Record #",
           title="SNV (CNN Input)")
    fig.colorbar(im, ax=ax, shrink=0.7, label="SNV")

    # Add concentration scale on right side
    for ax in axes:
        ax2 = ax.twinx()
        ax2.set_ylim(concs[-1], concs[0])
        ax2.set_ylabel("Concentration (ppm)")

    fig.suptitle("Dataset Heatmap — Each Row is One Spectrum (sorted by concentration)",
                 fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "spectra_heatmap.png", dpi=150)
    plt.close(fig)
    print(f"  Saved: {OUT_DIR / 'spectra_heatmap.png'}")

    # ═══════════════════════════════════════════════════════
    # PLOT 3: Key spectral features vs concentration
    # ═══════════════════════════════════════════════════════
    print("Plot 3: Feature trends vs concentration...")
    peak_positions = []
    peak_intensities = []
    fwhm_values = []

    for i in range(N):
        roi = roi_spectra[i]
        peak_idx = np.argmax(roi)
        peak_positions.append(ROI_START + peak_idx)
        peak_intensities.append(roi[peak_idx])
        # FWHM: find half-max crossings
        half_max = roi[peak_idx] / 2
        above = np.where(roi > half_max)[0]
        if len(above) > 1:
            fwhm_values.append(above[-1] - above[0])
        else:
            fwhm_values.append(0)

    fig, axes = plt.subplots(1, 3, figsize=(16, 5))

    axes[0].scatter(concs, peak_positions, s=8, alpha=0.7, c="steelblue")
    axes[0].set(xlabel="Concentration (ppm)", ylabel="Peak Position (pixel)",
                title="(a) Red Shift — Peak moves right")
    axes[0].grid(True, alpha=0.3)

    axes[1].scatter(concs, peak_intensities, s=8, alpha=0.7, c="coral")
    axes[1].set(xlabel="Concentration (ppm)", ylabel="Peak Intensity (norm.)",
                title="(b) Beer-Lambert Saturation")
    axes[1].grid(True, alpha=0.3)

    axes[2].scatter(concs, fwhm_values, s=8, alpha=0.7, c="green")
    axes[2].set(xlabel="Concentration (ppm)", ylabel="FWHM (pixels)",
                title="(c) Peak Broadening")
    axes[2].grid(True, alpha=0.3)

    fig.suptitle("Three Key Spectral Features That Change with Concentration",
                 fontsize=14, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "feature_trends.png", dpi=150)
    plt.close(fig)
    print(f"  Saved: {OUT_DIR / 'feature_trends.png'}")

    # ═══════════════════════════════════════════════════════
    # PLOT 4: Cross-validation visualization
    # ═══════════════════════════════════════════════════════
    print("Plot 4: Cross-validation fold visualization...")
    from sklearn.model_selection import KFold
    kf = KFold(n_splits=5, shuffle=True, random_state=42)

    fig, axes = plt.subplots(5, 1, figsize=(14, 8), sharex=True)
    colors_train = "steelblue"
    colors_val = "coral"

    for fold, (train_idx, val_idx) in enumerate(kf.split(concs)):
        ax = axes[fold]
        ax.scatter(train_idx, concs[train_idx], s=4, alpha=0.5, c=colors_train, label="Train")
        ax.scatter(val_idx, concs[val_idx], s=15, alpha=0.9, c=colors_val,
                   marker="x", label="Validation")
        ax.set_ylabel(f"Fold {fold+1}", fontsize=9)
        ax.set_ylim(-1, 21)
        ax.grid(True, alpha=0.2)
        if fold == 0:
            ax.legend(loc="upper right", fontsize=8)

    axes[-1].set_xlabel("Record Index (sorted by concentration)")
    fig.suptitle("5-Fold Cross Validation — Validation samples span the full range",
                 fontsize=13, fontweight="bold")
    fig.tight_layout()
    fig.savefig(OUT_DIR / "cv_folds.png", dpi=150)
    plt.close(fig)
    print(f"  Saved: {OUT_DIR / 'cv_folds.png'}")

    print("\n✓ All 4 visualizations saved!")
