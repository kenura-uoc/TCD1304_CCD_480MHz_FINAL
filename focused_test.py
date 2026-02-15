#!/usr/bin/env python3
"""Focused test: What does the device actually compute?"""
import numpy as np
import pandas as pd
import sys

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
p405 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004637.npz'
p450 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004702.npz'
bg_csv = BASE + r'\background_data\background-1000.csv'
model_h = BASE + r'\Core\Inc\chl_model_data.h'

FULL = 3694; ROI_S = 1300; ROI_E = 3200; NF = 1900

SG_S = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
                  1.9580419580e-01, 2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
                  1.0256410256e-01, 2.0979020979e-02, -8.3916083916e-02])
SG_D = np.array([1.1655011655e-01, 9.3240093240e-02, 6.9930069930e-02, 4.6620046620e-02,
                  2.3310023310e-02, 0.0, -2.3310023310e-02, -4.6620046620e-02,
                  -6.9930069930e-02, -9.3240093240e-02, -1.1655011655e-01])

def sg(d, c):
    h = len(c)//2; o = np.copy(d)
    for i in range(h, len(d)-h): o[i] = np.sum(d[i-h:i+h+1]*c)
    return o

def ext(content, name, sz):
    mk = f"static const float {name}[{sz}] = {{"
    s = content.find(mk)+len(mk); e = content.find("};", s)
    return np.array([float(x.strip()) for x in content[s:e].replace('f','').replace('\n','').split(',') if x.strip()])

def predict_full(spectrum, bg, int_ms, content):
    """Full C-style pipeline, returns (raw_pred, clamped_pred, display_val)"""
    sub = np.zeros(FULL)
    for i in range(FULL):
        bv = bg[i] if bg is not None else 0.0
        sub[i] = (bv - spectrum[i]) / int_ms
    
    sm = sg(sub, SG_S)
    dr = sg(sm, SG_D)
    roi = dr[ROI_S:ROI_E]
    m = np.mean(roi); s = np.std(roi)
    roi = (roi - m) / (s if s > 1e-8 else 1.0)
    
    coef = ext(content, "PLS_COEF", NF)
    xm = ext(content, "PLS_X_MEAN", NF)
    xs = ext(content, "PLS_X_STD", NF)
    
    pred = 4.2750242620 + np.sum(((roi - xm) / np.maximum(xs, 1e-8)) * coef)
    clamped = max(0.0, pred)
    display = clamped if clamped != 0.0 else -3.0
    return pred, clamped, display

with open(model_h, 'r') as f: content = f.read()
bg_raw = np.mean(pd.read_csv(bg_csv).iloc[:, 1:].values.astype(float), axis=0)

# Load data - these are INVERTED by DMA (65535 - raw)
avg405 = np.mean(np.load(p405)['pixels'][:8], axis=0)  # 8 frames like device
avg450 = np.mean(np.load(p450)['pixels'][:8], axis=0)

# Dark frame estimates
dark_edge = np.full(FULL, np.mean(avg405[:200]))  # uniform from edges
dark_csv_inv = 65535 - bg_raw  # old CSV as if device captured it

results = []
sys.stdout.write("RESULTS TABLE\n")
sys.stdout.write("="*80 + "\n")
sys.stdout.write(f"{'Scenario':<45} {'Raw PLS':>10} {'Clamped':>10} {'Display':>10}\n")
sys.stdout.write("-"*80 + "\n")

for label, sig, bgdata in [
    ("405nm + uniform dark (edges)", avg405, dark_edge),
    ("405nm + CSV dark (old setup)", avg405, dark_csv_inv),
    ("405nm + NO background", avg405, None),
    ("450nm + uniform dark (edges)", avg450, dark_edge),
    ("450nm + CSV dark (old setup)", avg450, dark_csv_inv),
    ("450nm + NO background", avg450, None),
]:
    raw, clamp, disp = predict_full(sig, bgdata, 1000.0, content)
    sys.stdout.write(f"{label:<45} {raw:>10.4f} {clamp:>10.4f} {disp:>10.4f}\n")
    results.append((label, raw, clamp, disp))

sys.stdout.write("\n")
sys.stdout.write("Real lab values: A=2.84, B=1.60\n")
sys.stdout.write("Device reported: A=-3.00 (error), B=1.80\n")
sys.stdout.write("\n")

# KEY: what the device ACTUALLY does
sys.stdout.write("KEY ANALYSIS:\n")
sys.stdout.write("-"*80 + "\n")
sys.stdout.write("The device captures its OWN dark frame (lasers OFF, CCD running).\n")
sys.stdout.write("Edge pixels give an estimate of what that looks like.\n")
sys.stdout.write(f"Dark level estimate (inverted): {np.mean(avg405[:200]):.1f}\n")
sys.stdout.write(f"Dark level raw:                 {65535 - np.mean(avg405[:200]):.1f}\n")
sys.stdout.write(f"CSV background raw:             {np.mean(bg_raw):.1f}\n")
sys.stdout.write(f"Difference:                     {np.mean(bg_raw) - (65535 - np.mean(avg405[:200])):.1f}\n")
sys.stdout.write("\n")

# Check if the 405nm with edge-dark gives a POSITIVE prediction
sys.stdout.write("CRITICAL: With edge-estimated dark, 405nm Chl-a PLS gives POSITIVE ~9.0\n")
sys.stdout.write("But device shows -3.00. This means the device dark frame is DIFFERENT\n")
sys.stdout.write("from the edge-pixel estimate, OR the integration_time_ms value is wrong.\n")
sys.stdout.write("\n")

# What integration time would cause -3.00?
sys.stdout.write("INTEGRATION TIME SENSITIVITY:\n")
for it in [100, 200, 500, 1000, 2000, 5000]:
    r, c, d = predict_full(avg405, dark_edge, float(it), content)
    sys.stdout.write(f"  int_time={it}ms: raw={r:.4f} display={d:.4f}\n")

sys.stdout.flush()
