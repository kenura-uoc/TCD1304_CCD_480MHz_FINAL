#!/usr/bin/env python3
"""Compare different background subtraction scenarios."""
import numpy as np
import pandas as pd
import sys

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
p405 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004637.npz'
p450 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004702.npz'
bg_path = BASE + r'\background_data\background-1000.csv'

SG_SMOOTH = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
                      1.9580419580e-01,  2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
                      1.0256410256e-01,  2.0979020979e-02, -8.3916083916e-02])
SG_DERIV = np.array([1.1655011655e-01,  9.3240093240e-02,  6.9930069930e-02,
                     4.6620046620e-02,  2.3310023310e-02,  0.0,
                    -2.3310023310e-02, -4.6620046620e-02, -6.9930069930e-02,
                    -9.3240093240e-02, -1.1655011655e-01])

def apply_sg(data, coeffs):
    half = len(coeffs) // 2
    out = np.copy(data)
    for i in range(half, len(data) - half):
        out[i] = np.sum(data[i-half:i+half+1] * coeffs)
    return out

def predict_pls(spectrum, int_ms, content):
    buf = spectrum / int_ms
    buf = apply_sg(buf, SG_SMOOTH)
    buf = apply_sg(buf, SG_DERIV)
    roi = buf[1300:3200]
    m, s = np.mean(roi), np.std(roi)
    roi = (roi - m) / (s if s > 1e-8 else 1.0)
    def ext(name, sz):
        st = content.find(f'static const float {name}[{sz}] = {{') + len(f'static const float {name}[{sz}] = {{')
        en = content.find('};', st)
        return np.array([float(x.strip()) for x in content[st:en].replace('f','').replace('\n','').split(',') if x.strip()])
    coef = ext('PLS_COEF', 1900)
    xm = ext('PLS_X_MEAN', 1900)
    xs = ext('PLS_X_STD', 1900)
    scaled = (roi - xm) / np.maximum(xs, 1e-8)
    return 4.2750242620 + np.sum(scaled * coef)

with open(BASE + r'\Core\Inc\chl_model_data.h', 'r') as f:
    content = f.read()

bg_raw = np.mean(pd.read_csv(bg_path).iloc[:, 1:].values.astype(float), axis=0)
d405_inv = np.mean(np.load(p405)['pixels'], axis=0)
d450_inv = np.mean(np.load(p450)['pixels'], axis=0)
raw405 = 65535 - d405_inv
raw450 = 65535 - d450_inv

print("Real lab: A=2.84, B=1.60")
print("Device:   A=-3.00, B=1.80")
print()

# Scenario 1: Training polarity (Raw - RawBG) using stored background CSV
print("=== S1: Training polarity (RawSig - RawBG_csv) ===")
r1a = predict_pls(raw405 - bg_raw, 1000, content)
r1b = predict_pls(raw450 - bg_raw, 1000, content)
print(f"  405nm → PLS: {r1a:.4f}")
print(f"  450nm → PLS: {r1b:.4f}")

# Scenario 2: Firmware formula with stored BG
print("\n=== S2: Firmware formula (InvBG_csv - InvSig) ===")
inv_bg = 65535 - bg_raw
r2a = predict_pls(inv_bg - d405_inv, 1000, content)
r2b = predict_pls(inv_bg - d450_inv, 1000, content)
print(f"  405nm → PLS: {r2a:.4f}  (same as S1)")
print(f"  450nm → PLS: {r2b:.4f}  (same as S1)")

# Scenario 3: Device self-captured dark frame (simulate)
# Dark frame = CCD with no laser = baseline inverted signal
# Use edge pixels (0-200) as proxy for dark level
print("\n=== S3: Device self-dark (edges as dark estimate) ===")
dark_est_inv = np.mean(d405_inv[:200])  # ~7500 inverted = ~58000 raw
dark_frame = np.full_like(d405_inv, dark_est_inv)
r3a = predict_pls(dark_frame - d405_inv, 1000, content)
r3b = predict_pls(dark_frame - d450_inv, 1000, content)
print(f"  Dark level (inverted): {dark_est_inv:.0f}")
print(f"  405nm → PLS: {r3a:.4f}")
print(f"  450nm → PLS: {r3b:.4f}")

# Scenario 4: Inverted signal, no BG subtraction
print("\n=== S4: Inverted signal, NO BG ===")
r4a = predict_pls(d405_inv, 1000, content)
r4b = predict_pls(d450_inv, 1000, content)
print(f"  405nm → PLS: {r4a:.4f}")
print(f"  450nm → PLS: {r4b:.4f}")

# Scenario 5: Raw signal (no inversion, no BG)
print("\n=== S5: Raw signal (no BG, no inversion) ===")
r5a = predict_pls(raw405, 1000, content)
r5b = predict_pls(raw450, 1000, content)
print(f"  405nm → PLS: {r5a:.4f}")
print(f"  450nm → PLS: {r5b:.4f}")

print("\n" + "="*50)
print("SUMMARY: Which is closest to real A=2.84?")
for name, val in [("S1/S2 (RawSig-RawBG_csv)", r1a), ("S3 (self-dark)", r3a), ("S4 (inv, no BG)", r4a), ("S5 (raw, no BG)", r5a)]:
    err = val - 2.84
    print(f"  {name}: {val:.4f} (err={err:+.4f})")
