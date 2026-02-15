#!/usr/bin/env python3
"""
Full analysis: simulate EXACTLY what the STM32 firmware does step-by-step,
compare Python PLS vs C-style PLS, and check all error conditions.

Real lab values: A=2.84, B=1.60
Device output:   A=-3.00 (error), B=1.80
"""
import numpy as np
import pandas as pd
import struct
import sys

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
p405 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004637.npz'
p450 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004702.npz'
bg_csv = BASE + r'\background_data\background-1000.csv'
model_h = BASE + r'\Core\Inc\chl_model_data.h'

FULL_SPECTRUM_LEN = 3694
ROI_START = 1300
ROI_END = 3200
NUM_FEATURES = 1900
AUTO_FRAMES = 8  # AUTO_FRAMES_PER_LASER

# SG coefficients (from header)
SG_SMOOTH = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
                      1.9580419580e-01,  2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
                      1.0256410256e-01,  2.0979020979e-02, -8.3916083916e-02])
SG_DERIV = np.array([1.1655011655e-01,  9.3240093240e-02,  6.9930069930e-02,
                     4.6620046620e-02,  2.3310023310e-02,  0.0,
                    -2.3310023310e-02, -4.6620046620e-02, -6.9930069930e-02,
                    -9.3240093240e-02, -1.1655011655e-01])

# ---- C-style functions (matching firmware exactly) ----

def apply_sg_c(inp, coeffs):
    """Exact C-style SG filter: edges = copy, interior = convolution"""
    half = len(coeffs) // 2
    out = np.copy(inp).astype(np.float32)
    for i in range(half, len(inp) - half):
        s = np.float32(0.0)
        for j in range(len(coeffs)):
            s += np.float32(coeffs[j]) * np.float32(inp[i - half + j])
        out[i] = s
    return out

def common_preprocess_c(avg_spectrum, background, integration_ms):
    """
    Exact replica of chl_predictor.c common_preprocess()
    Returns (status, roi_features)
    """
    avg = np.array(avg_spectrum, dtype=np.float32)
    bg = np.array(background, dtype=np.float32) if background is not None else None
    int_ms = np.float32(integration_ms)
    
    # Check low signal
    max_val = np.float32(0.0)
    for i in range(FULL_SPECTRUM_LEN):
        if avg[i] > max_val:
            max_val = avg[i]
    
    if max_val < np.float32(500.0):
        return ("CHL_ERR_LOW_SIGNAL", None)
    
    # Step 0: Background subtraction & polarity correction
    buf_a = np.zeros(FULL_SPECTRUM_LEN, dtype=np.float32)
    for i in range(FULL_SPECTRUM_LEN):
        bg_val = np.float32(bg[i]) if bg is not None else np.float32(0.0)
        buf_a[i] = (bg_val - avg[i]) / int_ms
    
    # Step 1: SG smooth
    buf_b = apply_sg_c(buf_a, SG_SMOOTH)
    
    # Step 2: SG derivative
    buf_a = apply_sg_c(buf_b, SG_DERIV)
    
    # Step 3: ROI crop
    roi = np.zeros(NUM_FEATURES, dtype=np.float32)
    for i in range(NUM_FEATURES):
        roi[i] = buf_a[ROI_START + i]
    
    # Step 4: SNV on ROI
    mean = np.float32(0.0)
    for i in range(NUM_FEATURES):
        mean += roi[i]
    mean /= np.float32(NUM_FEATURES)
    
    std_val = np.float32(0.0)
    for i in range(NUM_FEATURES):
        d = roi[i] - mean
        std_val += d * d
    std_val = np.sqrt(np.float32(std_val / np.float32(NUM_FEATURES)))
    inv_std = np.float32(1.0 / std_val) if std_val > np.float32(1e-8) else np.float32(1.0)
    
    for i in range(NUM_FEATURES):
        roi[i] = (roi[i] - mean) * inv_std
    
    return ("CHL_OK", roi)

def pls_predict_c(features, content):
    """Exact replica of chl_predictor.c pls_predict()"""
    coef = extract_array(content, "PLS_COEF", NUM_FEATURES)
    x_mean = extract_array(content, "PLS_X_MEAN", NUM_FEATURES)
    x_std = extract_array(content, "PLS_X_STD", NUM_FEATURES)
    
    prediction = np.float32(4.2750242620)
    for i in range(NUM_FEATURES):
        xs = np.float32(x_std[i])
        if xs > np.float32(1e-8):
            scaled = (np.float32(features[i]) - np.float32(x_mean[i])) / xs
        else:
            scaled = np.float32(0.0)
        prediction += scaled * np.float32(coef[i])
    
    # Clamp
    if prediction < np.float32(0.0):
        prediction = np.float32(0.0)
    
    return float(prediction)

def extract_array(content, name, size):
    marker = f"static const float {name}[{size}] = {{"
    start = content.find(marker) + len(marker)
    if start < len(marker):
        return np.zeros(size, dtype=np.float32)
    end = content.find("};", start)
    vals = content[start:end].replace('f', '').replace('\n', '').split(',')
    return np.array([float(x.strip()) for x in vals if x.strip()], dtype=np.float32)

# ---- Load data ----

print("Loading model data...")
with open(model_h, 'r') as f:
    content = f.read()

bg_raw = np.mean(pd.read_csv(bg_csv).iloc[:, 1:].values.astype(float), axis=0)

d405_frames = np.load(p405)['pixels']  # Shape: (N, 3694), INVERTED by DMA
d450_frames = np.load(p450)['pixels']

print(f"405nm: {d405_frames.shape[0]} frames")
print(f"450nm: {d450_frames.shape[0]} frames")

# ================================================================
# STEP 1: Simulate EXACTLY what the device firmware does
# ================================================================
print("\n" + "=" * 60)
print("STEP 1: EXACT FIRMWARE SIMULATION")
print("=" * 60)

def simulate_auto_measure(frames_405, frames_450, int_time_ms):
    """
    Simulate the full Auto Measure sequence as the firmware does it.
    The device:
    1. Captures AUTO_FRAMES_PER_LASER dark frames (no laser) -> dark_accum
    2. Moves servo, turns on laser 1 (405nm)
    3. Captures AUTO_FRAMES_PER_LASER laser1 frames -> auto_accum
    4. Averages and predicts Chl-a
    5. Moves servo, turns on laser 2 (450nm)
    6. Captures AUTO_FRAMES_PER_LASER laser2 frames -> auto_accum
    7. Averages and predicts Chl-b
    
    BUT: we don't have a separate dark recording!
    The device captured dark frames with NO LASER on.
    We'll estimate dark from the edge pixels of the 405nm recording.
    """
    n_frames = min(AUTO_FRAMES, frames_405.shape[0], frames_450.shape[0])
    
    # Average signal frames (already inverted by DMA on device)
    avg_405 = np.mean(frames_405[:n_frames], axis=0).astype(np.float32)
    avg_450 = np.mean(frames_450[:n_frames], axis=0).astype(np.float32)
    
    return avg_405, avg_450

avg_405, avg_450 = simulate_auto_measure(d405_frames, d450_frames, 1000)

# The dark frame: we DON'T have the actual device dark capture
# But we know the dark level from the non-illuminated edge pixels
dark_est_val = np.mean(avg_405[:200])  # Dark region average
print(f"\nEstimated dark level from edges (inverted): {dark_est_val:.1f}")
print(f"  This corresponds to raw: {65535 - dark_est_val:.1f}")

# Uniform dark frame (what edges suggest)
dark_uniform = np.full(FULL_SPECTRUM_LEN, dark_est_val, dtype=np.float32)

# Also try: what if device dark frame = background CSV inverted
dark_csv = (65535 - bg_raw).astype(np.float32)

print("\n--- Chl-a Prediction (405nm data) ---")
for dark_name, dark_data in [("Uniform dark (edges)", dark_uniform), 
                              ("CSV background (inverted)", dark_csv),
                              ("No background", None)]:
    status, roi = common_preprocess_c(avg_405, dark_data, 1000.0)
    if status != "CHL_OK":
        print(f"  {dark_name}: ERROR = {status}")
        continue
    pred = pls_predict_c(roi, content)
    # Apply menu.c logic
    display = pred
    if pred == 0.0:
        display = -3.0
    print(f"  {dark_name}: PLS={pred:.4f}, Display={display:.4f}")

print("\n--- What value does the PLS predict BEFORE clamping? ---")
for dark_name, dark_data in [("Uniform dark (edges)", dark_uniform), 
                              ("CSV background (inverted)", dark_csv)]:
    status, roi = common_preprocess_c(avg_405, dark_data, 1000.0)
    if status != "CHL_OK":
        print(f"  {dark_name}: ERROR = {status}")
        continue
    # Unclamped prediction
    coef = extract_array(content, "PLS_COEF", NUM_FEATURES)
    x_mean = extract_array(content, "PLS_X_MEAN", NUM_FEATURES)
    x_std = extract_array(content, "PLS_X_STD", NUM_FEATURES)
    pred_unclamped = np.float32(4.2750242620)
    for i in range(NUM_FEATURES):
        xs = np.float32(x_std[i])
        if xs > np.float32(1e-8):
            scaled = (np.float32(roi[i]) - np.float32(x_mean[i])) / xs
        else:
            scaled = np.float32(0.0)
        pred_unclamped += scaled * np.float32(coef[i])
    print(f"  {dark_name}: UNCLAMPED PLS = {float(pred_unclamped):.4f}")

print("\n--- Chl-b Prediction (450nm data, using Chl-a PLS model for comparison) ---")
for dark_name, dark_data in [("Uniform dark (edges)", dark_uniform), 
                              ("CSV background (inverted)", dark_csv)]:
    status, roi = common_preprocess_c(avg_450, dark_data, 1000.0)
    if status != "CHL_OK":
        print(f"  {dark_name}: ERROR = {status}")
        continue
    pred = pls_predict_c(roi, content)
    display = pred
    if pred == 0.0:
        display = -3.0
    print(f"  {dark_name}: PLS(Chl-a model)={pred:.4f}, Display={display:.4f}")

# ================================================================
# STEP 2: Compare with Python-style (float64) prediction 
# ================================================================
print("\n" + "=" * 60)
print("STEP 2: PYTHON (float64) vs C-STYLE (float32) COMPARISON")
print("=" * 60)

def preprocess_py(spectrum, integration_ms):
    """Python-style preprocessing (float64)"""
    buf = spectrum / integration_ms
    half = len(SG_SMOOTH) // 2
    out = np.copy(buf)
    for i in range(half, len(buf) - half):
        out[i] = np.sum(buf[i-half:i+half+1] * SG_SMOOTH)
    buf = out
    out = np.copy(buf)
    for i in range(half, len(buf) - half):
        out[i] = np.sum(buf[i-half:i+half+1] * SG_DERIV)
    roi = out[ROI_START:ROI_END]
    m, s = np.mean(roi), np.std(roi)
    roi = (roi - m) / (s if s > 1e-8 else 1.0)
    return roi

def pls_predict_py(roi, content):
    """Python-style PLS (float64)"""
    coef = extract_array(content, "PLS_COEF", NUM_FEATURES).astype(np.float64)
    xm = extract_array(content, "PLS_X_MEAN", NUM_FEATURES).astype(np.float64)
    xs = extract_array(content, "PLS_X_STD", NUM_FEATURES).astype(np.float64)
    scaled = (roi - xm) / np.maximum(xs, 1e-8)
    return 4.2750242620 + np.sum(scaled * coef)

# Using CSV background for comparison
sub_405 = (65535.0 - bg_raw) - avg_405.astype(np.float64)
sub_450 = (65535.0 - bg_raw) - avg_450.astype(np.float64)

roi_py_405 = preprocess_py(sub_405, 1000.0)
roi_py_450 = preprocess_py(sub_450, 1000.0)

pred_py_405 = pls_predict_py(roi_py_405, content)
pred_py_450 = pls_predict_py(roi_py_450, content)

status_c_405, roi_c_405 = common_preprocess_c(avg_405, dark_csv, 1000.0)
status_c_450, roi_c_450 = common_preprocess_c(avg_450, dark_csv, 1000.0)
pred_c_405 = pls_predict_c(roi_c_405, content) if status_c_405 == "CHL_OK" else "ERROR"
pred_c_450 = pls_predict_c(roi_c_450, content) if status_c_450 == "CHL_OK" else "ERROR"

print(f"405nm: Python(f64)={pred_py_405:.6f}  C-style(f32)={pred_c_405}")
print(f"450nm: Python(f64)={pred_py_450:.6f}  C-style(f32)={pred_c_450}")

# ================================================================
# STEP 3: Signal level analysis
# ================================================================
print("\n" + "=" * 60)
print("STEP 3: SIGNAL LEVEL ANALYSIS")
print("=" * 60)
print(f"405nm inverted: min={avg_405.min():.0f} max={avg_405.max():.0f} mean={avg_405.mean():.0f}")
print(f"450nm inverted: min={avg_450.min():.0f} max={avg_450.max():.0f} mean={avg_450.mean():.0f}")
print(f"Low signal threshold: 500")
print(f"405nm passes: {avg_405.max() >= 500}")
print(f"450nm passes: {avg_450.max() >= 500}")

# Check what the peak looks like in the fluorescence region
print(f"\nFluorescence region [1800:2800]:")
print(f"  405nm inverted: min={avg_405[1800:2800].min():.0f} max={avg_405[1800:2800].max():.0f}")
print(f"  450nm inverted: min={avg_450[1800:2800].min():.0f} max={avg_450[1800:2800].max():.0f}")

# Previous working data comparison
old_405 = np.mean(np.load(BASE + r'\ccd_monitor\projects\Default\rec_20260215_235828_405nm.npz')['pixels'], axis=0)
print(f"\nOld 405nm (gave 10.44): max={old_405.max():.0f}, fluorescence peak max={old_405[1800:2800].max():.0f}")
print(f"New 405nm:               max={avg_405.max():.0f}, fluorescence peak max={avg_405[1800:2800].max():.0f}")
print(f"Signal ratio (new/old):  {avg_405[1800:2800].max() / old_405[1800:2800].max():.3f}")
