#!/usr/bin/env python3
"""
validate_solution9.py — Solution 9, Record 4 (300ms) validation
================================================================
Focus: 300ms recording only. Tests two preprocessing variants:
  A) Dark (dummy px) + Lab background subtraction (training pipeline)
  B) Dark (dummy px) only, no lab background

Ground truth:  Chl-a = 3.0465 mg/L,  Chl-b = 1.865 mg/L
"""

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import shutil
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve
from sklearn.cross_decomposition import PLSRegression
import warnings
warnings.filterwarnings("ignore")

SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_DIR = SCRIPT_DIR.parent / "ccd_monitor" / "projects" / "16-2-2026"
BG_DIR = SCRIPT_DIR / "data" / "background_data"
PLOT_DIR = SCRIPT_DIR / "training_plots"
IMG_DIR = SCRIPT_DIR.parent / "kenura_lab_report" / "images"
PLOT_DIR.mkdir(exist_ok=True)

GT_A, GT_B = 3.0465, 1.865
DUMMY_END = 32
INT_TIME = 300
ROI_S, ROI_E = 1300, 3500

FILE_405 = PROJECT_DIR / "rec_20260216_133810.npz"
FILE_450 = PROJECT_DIR / "rec_20260216_133914.npz"
STM32_A, STM32_B = 26.00, 3.42

BG_MAP = {240:"background-240.csv", 250:"background-250.csv",
           300:"background-300.csv", 500:"background-500.csv",
           1000:"background-1000.csv"}

# ─────────────────────────────────────────────────────────────
def load_bg(it):
    nearest = min(sorted(BG_MAP), key=lambda t: abs(t-it))
    p = BG_DIR / BG_MAP[nearest]
    if p.exists():
        return np.mean(pd.read_csv(p).iloc[:,1:].values.astype(float), axis=0)
    return None

def als_baseline(y, lam=1e5, p=0.01, niter=10):
    L = len(y)
    D = sparse.diags([1,-2,1],[0,1,2], shape=(L-2,L), format='csc')
    D = D.T @ D; w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w,0,format='csc')
        z = spsolve(W + lam*D, w*y)
        w = p*(y>z) + (1-p)*(y<=z)
    return z

def preprocess(raw, bg, it, use_als=False):
    if bg is not None and len(bg)==len(raw):
        raw = raw - bg
    roi = raw[ROI_S:ROI_E]
    if use_als:
        try: roi = roi - als_baseline(roi)
        except: pass
    roi = roi / it
    sm = savgol_filter(roi, 11, 2)
    drv = savgol_filter(sm, 11, 3, deriv=1)
    mu, s = drv.mean(), drv.std()
    return (drv - mu) / (s + 1e-12)

def load_npz(path):
    px = np.load(path)["pixels"].astype(float)
    dark = np.mean(px[:, :DUMMY_END])
    return np.mean(px, axis=0) - dark, px.shape[0], dark

# ═══════════════════════════════════════════════════════════
print("="*65)
print(" Solution 9 — Record 4 (300ms) Validation")
print("="*65)
print(f" GT: Chl-a = {GT_A:.4f},  Chl-b = {GT_B:.4f} mg/L\n")

# Load pre-processed training data & fit PLS(n=15)
d = np.load(SCRIPT_DIR / "real_data_preprocessed.npz")
X_tr, Y_tr = d["X"], d["Y"]
pls = PLSRegression(n_components=15)
pls.fit(X_tr, Y_tr)
print(f" PLS(n=15) fitted on {X_tr.shape[0]} training samples\n")

# Load NPZ with dark subtraction
raw_405, nf_405, dk_405 = load_npz(FILE_405)
raw_450, nf_450, dk_450 = load_npz(FILE_450)
bg = load_bg(INT_TIME)

print(f" 405nm: {nf_405} frames, dark={dk_405:.0f}")
print(f" 450nm: {nf_450} frames, dark={dk_450:.0f}")
print(f" Background 300ms: {'loaded' if bg is not None else 'NONE'}")

# ── Variant A: Dark + BG subtraction ────────────────────────
feat_405_a = preprocess(raw_405, bg, INT_TIME, use_als=False)
feat_450_a = preprocess(raw_450, bg, INT_TIME, use_als=True)
X_a = np.hstack([feat_405_a, feat_450_a]).reshape(1,-1)
pred_a = pls.predict(X_a)[0]

# ── Variant B: Dark only (no lab BG) ────────────────────────
feat_405_b = preprocess(raw_405, None, INT_TIME, use_als=False)
feat_450_b = preprocess(raw_450, None, INT_TIME, use_als=True)
X_b = np.hstack([feat_405_b, feat_450_b]).reshape(1,-1)
pred_b = pls.predict(X_b)[0]

print(f"\n{'─'*65}")
print(f" Variant A (dark + lab BG):")
print(f"   Chl-a = {pred_a[0]:.3f}  (GT {GT_A:.3f}, err {pred_a[0]-GT_A:+.3f}, {(pred_a[0]-GT_A)/GT_A*100:+.1f}%)")
print(f"   Chl-b = {pred_a[1]:.3f}  (GT {GT_B:.3f}, err {pred_a[1]-GT_B:+.3f}, {(pred_a[1]-GT_B)/GT_B*100:+.1f}%)")
print(f"\n Variant B (dark only, no lab BG):")
print(f"   Chl-a = {pred_b[0]:.3f}  (GT {GT_A:.3f}, err {pred_b[0]-GT_A:+.3f}, {(pred_b[0]-GT_A)/GT_A*100:+.1f}%)")
print(f"   Chl-b = {pred_b[1]:.3f}  (GT {GT_B:.3f}, err {pred_b[1]-GT_B:+.3f}, {(pred_b[1]-GT_B)/GT_B*100:+.1f}%)")
print(f"\n STM32 Lite:")
print(f"   Chl-a = {STM32_A:.2f}  (err {STM32_A-GT_A:+.2f}, {(STM32_A-GT_A)/GT_A*100:+.0f}%)")
print(f"   Chl-b = {STM32_B:.2f}  (err {STM32_B-GT_B:+.2f}, {(STM32_B-GT_B)/GT_B*100:+.1f}%)")

# ═══════════════════════════════════════════════════════════
# CHART
# ═══════════════════════════════════════════════════════════
print("\n Generating chart...")
fig = plt.figure(figsize=(18, 12))
gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.35, wspace=0.3)

# ── P1: All predictions comparison (Chl-a) ──────────────────
ax = fig.add_subplot(gs[0, 0])
models = ["Ground Truth", "PLS\n(dark+BG)", "PLS\n(dark only)", "STM32\nLite"]
va = [GT_A, pred_a[0], pred_b[0], STM32_A]
ca = ["#4CAF50", "#2196F3", "#7B1FA2", "#FF9800"]
bars = ax.barh(range(4), va, color=ca, edgecolor='k', lw=0.5, height=0.6)
ax.axvline(GT_A, color='green', ls='--', lw=1.5, alpha=0.7)
ax.set_yticks(range(4)); ax.set_yticklabels(models, fontsize=9)
ax.set_xlabel("Chl-a (mg/L)")
ax.set_title("Chl-a Predictions", fontweight='bold')
for b,v in zip(bars,va):
    ax.text(max(b.get_width(),0)+0.15, b.get_y()+b.get_height()/2, f"{v:.2f}",
            va='center', fontsize=9, fontweight='bold')
ax.grid(True, axis='x', alpha=0.3)

# ── P2: Chl-b ───────────────────────────────────────────────
ax = fig.add_subplot(gs[0, 1])
vb = [GT_B, pred_a[1], pred_b[1], STM32_B]
bars = ax.barh(range(4), vb, color=ca, edgecolor='k', lw=0.5, height=0.6)
ax.axvline(GT_B, color='green', ls='--', lw=1.5, alpha=0.7)
ax.set_yticks(range(4)); ax.set_yticklabels(models, fontsize=9)
ax.set_xlabel("Chl-b (mg/L)")
ax.set_title("Chl-b Predictions", fontweight='bold')
for b,v in zip(bars,vb):
    ax.text(max(b.get_width(),0)+0.03, b.get_y()+b.get_height()/2, f"{v:.2f}",
            va='center', fontsize=9, fontweight='bold')
ax.grid(True, axis='x', alpha=0.3)

# ── P3: Summary table ───────────────────────────────────────
ax = fig.add_subplot(gs[0, 2]); ax.axis('off')
td = [["Model", "Chl-a", "Err(%)", "Chl-b", "Err(%)"]]
td.append(["UV-Vis (GT)", f"{GT_A:.3f}", "—", f"{GT_B:.3f}", "—"])
td.append(["PLS (dark+BG)", f"{pred_a[0]:.3f}", f"{(pred_a[0]-GT_A)/GT_A*100:+.1f}",
           f"{pred_a[1]:.3f}", f"{(pred_a[1]-GT_B)/GT_B*100:+.1f}"])
td.append(["PLS (dark only)", f"{pred_b[0]:.3f}", f"{(pred_b[0]-GT_A)/GT_A*100:+.1f}",
           f"{pred_b[1]:.3f}", f"{(pred_b[1]-GT_B)/GT_B*100:+.1f}"])
td.append(["STM32 Lite", f"{STM32_A:.1f}", f"{(STM32_A-GT_A)/GT_A*100:+.0f}",
           f"{STM32_B:.2f}", f"{(STM32_B-GT_B)/GT_B*100:+.1f}"])
tbl = ax.table(cellText=td, loc='center', cellLoc='center')
tbl.auto_set_font_size(False); tbl.set_fontsize(9); tbl.scale(1.1, 1.6)
for j in range(5):
    tbl[0,j].set_facecolor('#333'); tbl[0,j].set_text_props(color='white', fontweight='bold')
    tbl[1,j].set_facecolor('#E8F5E9')
ax.set_title("Record 4 (300ms) — Summary", fontweight='bold', pad=20)

# ── P4: Raw spectra (both variants) ─────────────────────────
ax = fig.add_subplot(gs[1, 0])
roi = slice(ROI_S, ROI_E)
# With BG
r405_bg = (raw_405 - bg[:len(raw_405)]) if bg is not None else raw_405
r450_bg = (raw_450 - bg[:len(raw_450)]) if bg is not None else raw_450
ax.plot(r405_bg[roi]/INT_TIME, label="405nm (dark+BG)", color='#2196F3', alpha=0.6, lw=0.8)
ax.plot(r450_bg[roi]/INT_TIME, label="450nm (dark+BG)", color='#FF9800', alpha=0.6, lw=0.8)
# Without BG
ax.plot(raw_405[roi]/INT_TIME, label="405nm (dark only)", color='#7B1FA2', alpha=0.6, lw=0.8, ls='--')
ax.plot(raw_450[roi]/INT_TIME, label="450nm (dark only)", color='#E91E63', alpha=0.6, lw=0.8, ls='--')
ax.axhline(0, color='k', lw=0.5, alpha=0.3)
ax.set_xlabel("Pixel (ROI)"); ax.set_ylabel("Intensity / ms")
ax.set_title("Raw Spectra Comparison\n(dark+BG vs dark only)", fontweight='bold', fontsize=10)
ax.legend(fontsize=7, ncol=2); ax.grid(True, alpha=0.2)

# ── P5: Preprocessed features comparison ────────────────────
ax = fig.add_subplot(gs[1, 1])
ax.plot(feat_405_a, label="405nm (dark+BG)", color='#2196F3', alpha=0.7, lw=0.8)
ax.plot(feat_405_b, label="405nm (dark only)", color='#7B1FA2', alpha=0.7, lw=0.8, ls='--')
ax.set_xlabel("Feature Index"); ax.set_ylabel("SNV 1st Derivative")
ax.set_title("Preprocessed 405nm Features\n(variant comparison)", fontweight='bold', fontsize=10)
ax.legend(fontsize=8); ax.grid(True, alpha=0.2)

# ── P6: Error breakdown ─────────────────────────────────────
ax = fig.add_subplot(gs[1, 2])
x = np.arange(3); w = 0.3
errs_a = [pred_a[0]-GT_A, pred_b[0]-GT_A, STM32_A-GT_A]
errs_b = [pred_a[1]-GT_B, pred_b[1]-GT_B, STM32_B-GT_B]
ax.bar(x-w/2, errs_a, w, label="Chl-a error", color='#2196F3', alpha=0.8)
ax.bar(x+w/2, errs_b, w, label="Chl-b error", color='#FF9800', alpha=0.8)
ax.axhline(0, color='k', lw=0.8)
ax.set_xticks(x); ax.set_xticklabels(["PLS\n(dark+BG)", "PLS\n(dark only)", "STM32\nLite"], fontsize=9)
ax.set_ylabel("Error (mg/L)")
ax.set_title("Absolute Errors", fontweight='bold')
ax.legend(fontsize=9); ax.grid(True, axis='y', alpha=0.3)
# Annotate values
for i, (ea, eb) in enumerate(zip(errs_a, errs_b)):
    ax.text(i-w/2, ea + (0.3 if ea>0 else -0.5), f"{ea:+.2f}", ha='center', fontsize=8, fontweight='bold')
    ax.text(i+w/2, eb + (0.3 if eb>0 else -0.5), f"{eb:+.2f}", ha='center', fontsize=8, fontweight='bold')

fig.suptitle(f"Solution 9 Validation — Record 4 (300ms, {nf_405}+{nf_450} frames)\n"
             f"PLS(n=15) with Dark Subtraction (dummy px 0–31)  |  "
             f"GT: Chl-a={GT_A:.4f}, Chl-b={GT_B:.4f} mg/L",
             fontsize=13, fontweight='bold', y=1.01)
plt.tight_layout()
out = PLOT_DIR / "solution9_validation.png"
fig.savefig(out, dpi=200, bbox_inches='tight'); plt.close()
print(f"\n ✅ {out.name}")
shutil.copy(out, IMG_DIR / out.name)
print(" ✅ Copied to report images")
print(" Done.")
