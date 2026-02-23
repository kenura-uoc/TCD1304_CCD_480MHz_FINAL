"""
Generate integration time normalization validation plots for the physics report.
Replicates the analysis from 'Integration Analysis Report' (16/02/2026).

Outputs:
  kenura_lab_report/images/integ_norm_405nm.png
  kenura_lab_report/images/integ_norm_450nm.png
"""

import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import os

# ── Paths ────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA = os.path.join(BASE, "ccd_monitor", "projects", "16-2-2026")
OUT  = os.path.join(BASE, "kenura_lab_report", "images")
os.makedirs(OUT, exist_ok=True)

FILES = {
    "405nm": {
        "300ms":  ("rec_20260216_133810.npz", 300),
        "1000ms": ("rec_20260216_134320.npz", 1000),
    },
    "450nm": {
        "300ms":  ("rec_20260216_133914.npz", 300),
        "1000ms": ("rec_20260216_134352.npz", 1000),
    },
}

# TCD1304 readout layout: 3694 total
# [0:32]    = front dummy pixels
# [32:3680] = 3648 active pixels
# [3680:]   = rear dummy pixels
DUMMY_FRONT = slice(0, 32)
DUMMY_REAR  = slice(3680, 3694)
ACTIVE      = slice(32, 3680)         # 3648 active pixels
PIXELS      = 3694


def load_and_normalise(path, t_int):
    """Load NPZ, dark-subtract using dummy pixels, divide by t_int, return active pixels only."""
    data   = np.load(path)
    frames = data["pixels"].astype(float)          # shape: (N_frames, 3694)
    normed = []
    for frame in frames:
        dark      = np.mean(np.concatenate([frame[DUMMY_FRONT], frame[DUMMY_REAR]]))
        corrected = frame[ACTIVE] - dark            # only active pixels
        normed.append(corrected / t_int)
    normed = np.array(normed)
    return normed.mean(axis=0), normed.std(axis=0)


def make_plot(laser, output_filename):
    records = FILES[laser]
    fig = plt.figure(figsize=(10, 6))
    fig.patch.set_facecolor('white')
    gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1], hspace=0.15)

    ax_main = fig.add_subplot(gs[0])
    ax_diff = fig.add_subplot(gs[1], sharex=ax_main)

    pixels_x = np.arange(3648)   # active pixel indices
    colour = {"300ms": "#1f77b4", "1000ms": "#d62728"}
    means, stds, labels = {}, {}, list(records.keys())

    for label, (fname, t_int) in records.items():
        fpath = os.path.join(DATA, fname)
        mu, sigma = load_and_normalise(fpath, t_int)
        mu_sg = savgol_filter(mu, window_length=11, polyorder=2)
        means[label] = mu_sg
        stds[label]  = sigma
        ax_main.plot(pixels_x, mu_sg, color=colour[label], lw=1.2, label=f"{label}")
        ax_main.fill_between(pixels_x, mu_sg - sigma, mu_sg + sigma,
                              color=colour[label], alpha=0.1)

    # Difference plot
    diff = means[labels[0]] - means[labels[1]]
    diff_noise = np.sqrt(stds[labels[0]]**2 + stds[labels[1]]**2)
    mean_diff  = np.mean(diff)
    rel_err    = abs(mean_diff) / (np.mean(np.abs(means[labels[0]])) + 1e-9) * 100

    ax_diff.plot(pixels_x, diff, color="#2ca02c", lw=1.0, label="Difference (300ms − 1000ms)")
    ax_diff.fill_between(pixels_x, -diff_noise, diff_noise, color="gray", alpha=0.15,
                         label=r"$\pm 1\sigma$ noise floor")
    ax_diff.axhline(0, color="black", lw=0.6, ls="--", alpha=0.3)
    ax_diff.axhline(mean_diff, color="#ff7f0e", lw=1.0, ls=":",
                    label=f"Mean diff = {mean_diff:.4f} cts/ms (rel. err ≈ {rel_err:.1f}%)")

    # Styling
    for ax in [ax_main, ax_diff]:
        ax.set_facecolor('white')
        ax.tick_params(colors='#333333', labelsize=8)
        ax.spines[:].set_color('#888888')
        ax.grid(True, color='#eeeeee', lw=0.5)

    ax_main.set_ylabel("Intensity Rate (Counts/ms)", color="#333333", fontsize=10)
    ax_main.legend(framealpha=0.5, fontsize=9)
    ax_main.set_title(
        f"Integration Time Linearity Validation — {laser} Laser\n"
        r"Normalised spectra ($S_{corr}/t_{int}$) at 300 ms vs 1000 ms",
        color="black", fontsize=11, pad=10
    )
    plt.setp(ax_main.get_xticklabels(), visible=False)

    ax_diff.set_xlabel("Pixel Index", color="#333333", fontsize=10)
    ax_diff.set_ylabel("Difference\n(Cts/ms)", color="#333333", fontsize=8)
    ax_diff.legend(framealpha=0.5, fontsize=7, loc='upper right')

    out_path = os.path.join(OUT, output_filename)
    fig.savefig(out_path, dpi=200, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)
    print(f"Saved: {out_path}")
    return mean_diff, rel_err


print("Generating integration time normalization validation plots...")
d405, e405 = make_plot("405nm", "integ_norm_405nm.png")
d450, e450 = make_plot("450nm", "integ_norm_450nm.png")
print(f"\n405nm → Mean Δ = {d405:.4f} cts/ms, Rel. err ≈ {e405:.1f}%")
print(f"450nm → Mean Δ = {d450:.4f} cts/ms, Rel. err ≈ {e450:.1f}%")
print("\nDone. Images saved to kenura_lab_report/images/")
