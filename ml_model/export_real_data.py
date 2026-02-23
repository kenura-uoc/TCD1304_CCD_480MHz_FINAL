#!/usr/bin/env python3
"""
export_real_data.py — Export preprocessed real data to .npz for Colab upload.
Runs the same preprocessing as train_combined_model.py, then saves X, Y arrays.
"""
import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter
from scipy import sparse
from scipy.sparse.linalg import spsolve

SCRIPT_DIR = Path(__file__).parent.resolve()
DATA_DIR_A = SCRIPT_DIR / "data" / "chl_a"
DATA_DIR_B = SCRIPT_DIR / "data" / "chl_b"
BG_DIR     = SCRIPT_DIR / "data" / "background_data"
CSV_A      = SCRIPT_DIR / "data" / "real_data" / "chla_data.csv"
CSV_B      = SCRIPT_DIR / "data" / "real_data" / "chlb_data.csv"

ROI_START, ROI_END = 1300, 3500
SG_WIN, SG_POLY = 11, 2
SG_DWIN, SG_DPOLY = 11, 3

BG_MAP = {240: "background-240.csv", 250: "background-250.csv",
           300: "background-300.csv", 500: "background-500.csv",
           1000: "background-1000.csv"}
BG_AVAIL = sorted(BG_MAP.keys())

def load_spectrum(path):
    df = pd.read_csv(path)
    return np.mean(df.iloc[:, 1:].values.astype(float), axis=0)

def load_bg(int_time):
    nearest = min(BG_AVAIL, key=lambda t: abs(t - int_time))
    path = BG_DIR / BG_MAP[nearest]
    return load_spectrum(path) if path.exists() else None

def als_baseline(y, lam=1e5, p=0.01, niter=10):
    L = len(y)
    D = sparse.diags([1, -2, 1], [0, 1, 2], shape=(L-2, L), format='csc')
    D = D.T @ D
    w = np.ones(L)
    for _ in range(niter):
        W = sparse.diags(w, 0, format='csc')
        z = spsolve(W + lam * D, w * y)
        w = p * (y > z) + (1 - p) * (y <= z)
    return z

def preprocess(raw, bg, int_time, use_als=False):
    if bg is not None and len(bg) == len(raw):
        raw = raw - bg
    roi = raw[ROI_START:ROI_END]
    if use_als:
        try: roi = roi - als_baseline(roi)
        except: pass
    roi = roi / int_time
    sm  = savgol_filter(roi, SG_WIN, SG_POLY)
    drv = savgol_filter(sm, SG_DWIN, SG_DPOLY, deriv=1)
    mu, s = drv.mean(), drv.std()
    return (drv - mu) / (s + 1e-12)

def load_dataset(csv_path, data_dir, use_als=False):
    meta = pd.read_csv(csv_path).dropna(subset=["Sample Con", "integration time(mS)"])
    spectra, row_indices = [], []
    for i, row in meta.iterrows():
        fpath = data_dir / f"{i+1}.csv"
        if not fpath.exists(): continue
        raw = load_spectrum(fpath)
        if len(raw) < ROI_END: continue
        bg = load_bg(int(row["integration time(mS)"]))
        feat = preprocess(raw, bg, row["integration time(mS)"], use_als)
        spectra.append(feat)
        row_indices.append(i)
    return np.array(spectra), row_indices

if __name__ == "__main__":
    print("Loading Chl-a spectra (405nm)...")
    X_a, idx_a = load_dataset(CSV_A, DATA_DIR_A, use_als=False)
    print(f"  {X_a.shape}")
    print("Loading Chl-b spectra (450nm)...")
    X_b, idx_b = load_dataset(CSV_B, DATA_DIR_B, use_als=True)
    print(f"  {X_b.shape}")

    meta_a = pd.read_csv(CSV_A).dropna(subset=["Sample Con"])
    meta_b = pd.read_csv(CSV_B).dropna(subset=["Sample Con"])

    min_n = min(len(X_a), len(X_b))
    X = np.hstack([X_a[:min_n], X_b[:min_n]])
    Y = np.column_stack([
        meta_a["Sample Con"].values[:min_n],
        meta_b["Sample Con"].values[:min_n],
    ])

    out = SCRIPT_DIR / "real_data_preprocessed.npz"
    np.savez_compressed(out, X=X, Y=Y)
    print(f"\n✅ Saved: {out}")
    print(f"   X shape: {X.shape}, Y shape: {Y.shape}")
    print(f"   Chl-a range: {Y[:, 0].min():.2f}–{Y[:, 0].max():.2f} mg/L")
    print(f"   Chl-b range: {Y[:, 1].min():.2f}–{Y[:, 1].max():.2f} mg/L")
    print(f"   File size: {out.stat().st_size / 1024:.1f} KB")
    print(f"\nUpload this .npz to Colab and run the validation script.")
