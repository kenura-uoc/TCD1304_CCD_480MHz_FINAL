#!/usr/bin/env python3
"""
Analyze the test_bug NPZ recordings to find why Chl-a gives -3.00 on device.
Real lab values: A=2.84, B=1.60
Device output:   A=-3.00 (error), B=1.80
"""
import numpy as np
import pandas as pd
import os

# Paths
BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
p405 = os.path.join(BASE, 'ccd_monitor', 'projects', 'test_bug', 'rec_20260216_004637.npz')
p450 = os.path.join(BASE, 'ccd_monitor', 'projects', 'test_bug', 'rec_20260216_004702.npz')
bg_path = os.path.join(BASE, 'background_data', 'background-1000.csv')
model_header = os.path.join(BASE, 'Core', 'Inc', 'chl_model_data.h')

# ROI
ROI_START = 1300
ROI_END = 3200
FULL_SPECTRUM_LEN = 3694

# SG Coefficients from header
SG_SMOOTH = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
                      1.9580419580e-01,  2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
                      1.0256410256e-01,  2.0979020979e-02, -8.3916083916e-02])

SG_DERIV = np.array([1.1655011655e-01,  9.3240093240e-02,  6.9930069930e-02,
                     4.6620046620e-02,  2.3310023310e-02,  0.0,
                    -2.3310023310e-02, -4.6620046620e-02, -6.9930069930e-02,
                    -9.3240093240e-02, -1.1655011655e-01])

def apply_sg_c_style(input_data, coeffs):
    half = len(coeffs) // 2
    output = np.copy(input_data)
    for i in range(half, len(input_data) - half):
        window = input_data[i-half : i+half+1]
        output[i] = np.sum(window * coeffs)
    return output

def preprocess(spectrum, integration_ms):
    buf_a = spectrum / integration_ms
    buf_b = apply_sg_c_style(buf_a, SG_SMOOTH)
    buf_a = apply_sg_c_style(buf_b, SG_DERIV)
    # ROI crop FIRST, then SNV
    roi = buf_a[ROI_START:ROI_END]
    mean = np.mean(roi)
    std = np.std(roi)
    roi = (roi - mean) / (std if std > 1e-8 else 1.0)
    return roi

def extract_array(content, name, size):
    start_marker = f"static const float {name}[{size}] = {{"
    start_idx = content.find(start_marker) + len(start_marker)
    if start_idx < len(start_marker): return np.zeros(size)
    end_idx = content.find("};", start_idx)
    raw_vals = content[start_idx:end_idx].replace('f', '').replace('\n', '').split(',')
    return np.array([float(x.strip()) for x in raw_vals if x.strip()])

def pls_predict(roi_features, content):
    pls_coef = extract_array(content, "PLS_COEF", 1900)
    pls_x_mean = extract_array(content, "PLS_X_MEAN", 1900)
    pls_x_std = extract_array(content, "PLS_X_STD", 1900)
    pls_intercept = 4.2750242620
    scaled = (roi_features - pls_x_mean) / np.maximum(pls_x_std, 1e-8)
    return pls_intercept + np.sum(scaled * pls_coef)

# Load model data
with open(model_header, 'r') as f:
    content = f.read()

# Load background
bg_df = pd.read_csv(bg_path)
bg_df = bg_df.iloc[:, 1:]
bg_raw = np.mean(bg_df.values.astype(float), axis=0)

print("=" * 60)
print("SIGNAL ANALYSIS")
print("=" * 60)

for label, path in [("405nm (Chl-a)", p405), ("450nm (Chl-b)", p450)]:
    data = np.load(path)
    pixels = data['pixels']
    avg = np.mean(pixels, axis=0)
    
    print(f"\n--- {label}: {os.path.basename(path)} ---")
    print(f"  Frames: {pixels.shape[0]}, Pixels: {pixels.shape[1]}")
    print(f"  Inverted signal (from device):")
    print(f"    Min: {avg.min():.1f}, Max: {avg.max():.1f}, Mean: {avg.mean():.1f}")
    
    # The .npz data from the Python monitor is INVERTED (65535 - raw)
    # because the DMA callback inverts it before USB send
    raw_signal = 65535 - avg  # Convert back to raw
    print(f"  Raw signal (un-inverted):")
    print(f"    Min: {raw_signal.min():.1f}, Max: {raw_signal.max():.1f}, Mean: {raw_signal.mean():.1f}")
    
    # Check: would the firmware LOW_SIGNAL check trigger?
    # The firmware checks max(avg_spectrum) < 500
    # avg_spectrum is the INVERTED signal
    max_inverted = avg.max()
    print(f"  Low signal check: max(inverted) = {max_inverted:.1f} {'< 500 → WOULD TRIGGER!' if max_inverted < 500 else '>= 500 → OK'}")
    
    # Background (raw)
    print(f"  Background (raw): Min: {bg_raw.min():.1f}, Max: {bg_raw.max():.1f}, Mean: {bg_raw.mean():.1f}")
    
    # What the firmware would compute: (dark_inverted - signal_inverted) = RawSig - RawBG
    dark_inverted = 65535 - bg_raw  # Firmware inverts dark frames too
    firmware_sub = dark_inverted - avg  # bg_val - avg_spectrum
    print(f"  Firmware subtraction (InvBG - InvSig = RawSig - RawBG):")
    print(f"    Min: {firmware_sub.min():.1f}, Max: {firmware_sub.max():.1f}, Mean: {firmware_sub.mean():.1f}")
    
    # Python model prediction (matching training: spectrum - bg, both raw)
    spectrum_for_model = raw_signal - bg_raw  # RawSig - RawBG (training polarity)
    
    roi = preprocess(spectrum_for_model, 1000.0)
    prediction = pls_predict(roi, content)
    
    print(f"  Python PLS Prediction (Chl-a model): {prediction:.4f}")
    print(f"  Clamped (if < 0 → 0): {max(0, prediction):.4f}")

print("\n" + "=" * 60)
print("COMPARISON WITH PREVIOUS WORKING DATA")
print("=" * 60)

# Also test with the previous 405nm that gave 10.44
p405_old = os.path.join(BASE, 'ccd_monitor', 'projects', 'Default', 'rec_20260215_235828_405nm.npz')
if os.path.exists(p405_old):
    data_old = np.load(p405_old)
    avg_old = np.mean(data_old['pixels'], axis=0)
    print(f"\nOld 405nm data: Min={avg_old.min():.1f}, Max={avg_old.max():.1f}, Mean={avg_old.mean():.1f}")
    data_new = np.load(p405)
    avg_new = np.mean(data_new['pixels'], axis=0)
    print(f"New 405nm data: Min={avg_new.min():.1f}, Max={avg_new.max():.1f}, Mean={avg_new.mean():.1f}")
    
    # Compare signal ranges
    print(f"\nSignal difference (new - old):")
    print(f"  Mean diff: {(avg_new - avg_old).mean():.1f}")
    print(f"  Max absolute diff: {np.abs(avg_new - avg_old).max():.1f}")
