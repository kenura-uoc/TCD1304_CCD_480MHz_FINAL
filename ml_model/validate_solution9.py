#!/usr/bin/env python3
"""
validate_solution9.py — Solution 9 Record 4 (300ms): Single vs Dual PLS
========================================================================
Compares single-output PLS (one channel each) against dual-output PLS
(both channels concatenated) on field data with dark current subtraction.

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

FILE_405 = PROJECT_DIR / "rec_20260216_133810.npz"
FILE_450 = PROJECT_DIR / "rec_20260216_133914.npz"

CSV_A = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"
DATA_A = SCRIPT_DIR / "data" / "chl_a"
DATA_B = SCRIPT_DIR / "data" / "chl_b"

BG_MAP = {240:"background-240.csv", 250:"background-250.csv",
           300:"background-300.csv", 500:"background-500.csv",
           1000:"background-1000.csv"}

# ─────────────────────────────────────────────────────────────
def load_bg(it):
    nearest = min(sorted(BG_MAP), key=lambda t: abs(t-it))
    p = BG_DIR / BG_MAP[nearest]
    return np.mean(pd.read_csv(p).iloc[:,1:].values.astype(float), axis=0) if p.exists() else None

def als_baseline(y, lam=1e5, p=0.01, niter=10):
    L = len(y)
    D = sparse.diags([1,-2,1],[0,1,2], shape=(L-2,L), format='csc')
    D = D.T @ D; w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w,0,format='csc')
        z = spsolve(W + lam*D, w*y)
        w = p*(y>z) + (1-p)*(y<=z)
    return z

def preprocess(raw, bg_spec, it, roi_s, roi_e, use_als=False):
    if bg_spec is not None and len(bg_spec)==len(raw):
        raw = raw - bg_spec
    roi = raw[roi_s:roi_e]
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

def load_training(csv_path, data_dir, roi_s, roi_e, use_als):
    meta = pd.read_csv(csv_path).dropna(subset=["Sample Con", "integration time(mS)"])
    X, Y = [], []
    for i, row in meta.iterrows():
        fp = data_dir / f"{i+1}.csv"
        if not fp.exists(): continue
        df = pd.read_csv(fp)
        raw_spec = np.mean(df.iloc[:,1:].values.astype(float), axis=0)
        if len(raw_spec) < roi_e: continue
        bg_spec = load_bg(int(row["integration time(mS)"]))
        feat = preprocess(raw_spec, bg_spec, row["integration time(mS)"], roi_s, roi_e, use_als)
        X.append(feat)
        Y.append(row["Sample Con"])
    return np.array(X), np.array(Y)


# ═══════════════════════════════════════════════════════════
print("="*65)
print(" Solution 9 — Single vs Dual PLS (Record 4, 300ms)")
print("="*65)

# ── Load field data ──────────────────────────────────────────
raw_405, nf_405, dk_405 = load_npz(FILE_405)
raw_450, nf_450, dk_450 = load_npz(FILE_450)
bg = load_bg(INT_TIME)
print(f" 405nm: {nf_405} frames, dark={dk_405:.0f}")
print(f" 450nm: {nf_450} frames, dark={dk_450:.0f}")

# ── Train single-output models ──────────────────────────────
ROI_SINGLE = (1300, 3200)
ROI_DUAL   = (1300, 3500)

print("\n Training single Chl-a model (405nm, ROI 1300-3200)...")
X_sa, Y_sa = load_training(CSV_A, DATA_A, *ROI_SINGLE, False)
n_sa = min(15, X_sa.shape[0] // 5)
pls_sa = PLSRegression(n_components=n_sa)
pls_sa.fit(X_sa, Y_sa)
print(f"  {X_sa.shape[0]} samples, n_comp={n_sa}")

print(" Training single Chl-b model (450nm, ROI 1300-3200)...")
X_sb, Y_sb = load_training(CSV_B, DATA_B, *ROI_SINGLE, True)
n_sb = min(15, X_sb.shape[0] // 5)
pls_sb = PLSRegression(n_components=n_sb)
pls_sb.fit(X_sb, Y_sb)
print(f"  {X_sb.shape[0]} samples, n_comp={n_sb}")

# ── Train dual-output model ─────────────────────────────────
print(" Loading dual model from pre-exported data...")
d = np.load(SCRIPT_DIR / "real_data_preprocessed.npz")
pls_dual = PLSRegression(n_components=15)
pls_dual.fit(d["X"], d["Y"])
print(f"  {d['X'].shape[0]} samples, n_comp=15")

# ── Predict ──────────────────────────────────────────────────
# Single models
feat_405_s = preprocess(raw_405, bg, INT_TIME, *ROI_SINGLE, False).reshape(1,-1)
feat_450_s = preprocess(raw_450, bg, INT_TIME, *ROI_SINGLE, True).reshape(1,-1)
pred_sa = float(pls_sa.predict(feat_405_s)[0])
pred_sb = float(pls_sb.predict(feat_450_s)[0])

# Dual model
feat_405_d = preprocess(raw_405, bg, INT_TIME, *ROI_DUAL, False)
feat_450_d = preprocess(raw_450, bg, INT_TIME, *ROI_DUAL, True)
X_dual = np.hstack([feat_405_d, feat_450_d]).reshape(1,-1)
pred_dual = pls_dual.predict(X_dual)[0]

print(f"\n{'─'*65}")
print(f" GT: Chl-a = {GT_A:.4f}, Chl-b = {GT_B:.4f}")
print(f"\n Single Chl-a (405nm): {pred_sa:.3f}  ({(pred_sa-GT_A)/GT_A*100:+.1f}%)")
print(f" Single Chl-b (450nm): {pred_sb:.3f}  ({(pred_sb-GT_B)/GT_B*100:+.1f}%)")
print(f"\n Dual   Chl-a:         {pred_dual[0]:.3f}  ({(pred_dual[0]-GT_A)/GT_A*100:+.1f}%)")
print(f" Dual   Chl-b:         {pred_dual[1]:.3f}  ({(pred_dual[1]-GT_B)/GT_B*100:+.1f}%)")


# ═══════════════════════════════════════════════════════════
# CHART
# ═══════════════════════════════════════════════════════════
print("\n Generating chart...")
fig = plt.figure(figsize=(16, 10))
gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.35, wspace=0.3)

# ── P1: Chl-a comparison ────────────────────────────────────
ax = fig.add_subplot(gs[0, 0])
models = ["Ground\nTruth", "Single PLS\n(405nm only)", "Dual PLS\n(405+450nm)"]
va = [GT_A, pred_sa, pred_dual[0]]
ca = ["#4CAF50", "#FF9800", "#2196F3"]
bars = ax.barh(range(3), va, color=ca, edgecolor='k', lw=0.5, height=0.5)
ax.axvline(GT_A, color='green', ls='--', lw=1.5, alpha=0.7)
ax.set_yticks(range(3)); ax.set_yticklabels(models, fontsize=10)
ax.set_xlabel("Chl-a (mg/L)", fontsize=10)
ax.set_title("Chl-a Predictions", fontweight='bold', fontsize=12)
for b, v in zip(bars, va):
    pos = max(b.get_width(), 0) + 0.15 if v > 0 else 0.15
    ax.text(pos, b.get_y()+b.get_height()/2, f"{v:.2f}", va='center', fontsize=10, fontweight='bold')
ax.grid(True, axis='x', alpha=0.3)

# ── P2: Chl-b comparison ────────────────────────────────────
ax = fig.add_subplot(gs[0, 1])
vb = [GT_B, pred_sb, pred_dual[1]]
bars = ax.barh(range(3), vb, color=ca, edgecolor='k', lw=0.5, height=0.5)
ax.axvline(GT_B, color='green', ls='--', lw=1.5, alpha=0.7)
ax.set_yticks(range(3)); ax.set_yticklabels(models, fontsize=10)
ax.set_xlabel("Chl-b (mg/L)", fontsize=10)
ax.set_title("Chl-b Predictions", fontweight='bold', fontsize=12)
for b, v in zip(bars, vb):
    pos = max(b.get_width(), 0) + 0.05 if v > 0 else 0.05
    ax.text(pos, b.get_y()+b.get_height()/2, f"{v:.2f}", va='center', fontsize=10, fontweight='bold')
ax.grid(True, axis='x', alpha=0.3)

# ── P3: Summary table ───────────────────────────────────────
ax = fig.add_subplot(gs[0, 2]); ax.axis('off')
td = [["Model", "Chl-a", "Err(%)", "Chl-b", "Err(%)"]]
td.append(["UV-Vis (GT)", f"{GT_A:.3f}", "—", f"{GT_B:.3f}", "—"])
td.append(["Single PLS", f"{pred_sa:.3f}", f"{(pred_sa-GT_A)/GT_A*100:+.1f}",
           f"{pred_sb:.3f}", f"{(pred_sb-GT_B)/GT_B*100:+.1f}"])
td.append(["Dual PLS", f"{pred_dual[0]:.3f}", f"{(pred_dual[0]-GT_A)/GT_A*100:+.1f}",
           f"{pred_dual[1]:.3f}", f"{(pred_dual[1]-GT_B)/GT_B*100:+.1f}"])

tbl = ax.table(cellText=td, loc='center', cellLoc='center')
tbl.auto_set_font_size(False); tbl.set_fontsize(10); tbl.scale(1.1, 1.8)
for j in range(5):
    tbl[0,j].set_facecolor('#333'); tbl[0,j].set_text_props(color='white', fontweight='bold')
    tbl[1,j].set_facecolor('#E8F5E9')
# Highlight best
tbl[3,3].set_facecolor('#C8E6C9')  # Dual Chl-b
tbl[3,4].set_facecolor('#C8E6C9')
ax.set_title("Record 4 (300ms) — Summary", fontweight='bold', pad=20, fontsize=12)

# ── P4: Error breakdown ─────────────────────────────────────
ax = fig.add_subplot(gs[1, 0])
x = np.arange(2); w = 0.3
errs_single = [pred_sa - GT_A, pred_sb - GT_B]
errs_dual   = [pred_dual[0] - GT_A, pred_dual[1] - GT_B]
bars1 = ax.bar(x - w/2, errs_single, w, label="Single PLS", color='#FF9800', alpha=0.85, edgecolor='k', lw=0.5)
bars2 = ax.bar(x + w/2, errs_dual, w, label="Dual PLS", color='#2196F3', alpha=0.85, edgecolor='k', lw=0.5)
ax.axhline(0, color='k', lw=0.8)
ax.set_xticks(x); ax.set_xticklabels(["Chl-a", "Chl-b"], fontsize=11)
ax.set_ylabel("Prediction Error (mg/L)", fontsize=10)
ax.set_title("Absolute Errors", fontweight='bold', fontsize=12)
ax.legend(fontsize=10)
ax.grid(True, axis='y', alpha=0.3)
for b, v in zip(bars1, errs_single):
    ax.text(b.get_x()+b.get_width()/2, v + (0.3 if v>0 else -0.8),
            f"{v:+.2f}", ha='center', fontsize=9, fontweight='bold')
for b, v in zip(bars2, errs_dual):
    ax.text(b.get_x()+b.get_width()/2, v + (0.3 if v>0 else -0.8),
            f"{v:+.2f}", ha='center', fontsize=9, fontweight='bold')

# ── P5: Raw spectra ──────────────────────────────────────────
ax = fig.add_subplot(gs[1, 1])
roi = slice(ROI_DUAL[0], ROI_DUAL[1])
r405 = raw_405.copy()
r450 = raw_450.copy()
if bg is not None:
    r405_bg = r405 - bg[:len(r405)]
    r450_bg = r450 - bg[:len(r450)]
else:
    r405_bg = r405; r450_bg = r450
# Dark-only (no BG)
ax.plot(r405[roi]/INT_TIME, label="405nm (dark sub)", color='#2196F3', alpha=0.7, lw=0.8)
ax.plot(r450[roi]/INT_TIME, label="450nm (dark sub)", color='#FF9800', alpha=0.7, lw=0.8)
ax.axhline(0, color='k', lw=0.5, alpha=0.3)
ax.set_xlabel("Pixel (ROI 1300–3500)", fontsize=10)
ax.set_ylabel("Intensity / ms", fontsize=10)
ax.set_title("Dark-Subtracted Spectra\n(before BG subtraction)", fontweight='bold', fontsize=11)
ax.legend(fontsize=9); ax.grid(True, alpha=0.2)

# ── P6: Preprocessed features comparison ────────────────────
ax = fig.add_subplot(gs[1, 2])
ax.plot(feat_405_d, label="405nm (dual preproc)", color='#2196F3', alpha=0.7, lw=0.8)
ax.plot(feat_450_d, label="450nm (dual preproc)", color='#FF9800', alpha=0.7, lw=0.8)
ax.set_xlabel("Feature Index", fontsize=10)
ax.set_ylabel("SNV 1st Derivative", fontsize=10)
ax.set_title("Preprocessed Features\n(dual-channel input)", fontweight='bold', fontsize=11)
ax.legend(fontsize=9); ax.grid(True, alpha=0.2)

fig.suptitle(f"Solution 9 — Single vs Dual PLS Validation (300ms, {nf_405}+{nf_450} frames)\n"
             f"Dark subtracted (dummy px 0–31)  |  GT: Chl-a={GT_A:.4f}, Chl-b={GT_B:.4f} mg/L",
             fontsize=13, fontweight='bold', y=1.01)
plt.tight_layout()
out = PLOT_DIR / "solution9_validation.png"
fig.savefig(out, dpi=200, bbox_inches='tight'); plt.close()
print(f"\n ✅ {out.name}")
shutil.copy(out, IMG_DIR / out.name)
print(" ✅ Copied to report images")
print(" Done.")
