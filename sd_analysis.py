#!/usr/bin/env python3
"""
Analyze SD card data: load binary frames, plot spectra, run model.
CRITICAL FINDING: integration_time_ms=100 on device!
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
SD = BASE + r'\sd_card\measurements'
bg_csv = BASE + r'\background_data\background-1000.csv'
model_h = BASE + r'\Core\Inc\chl_model_data.h'
OUT = BASE + r'\sd_analysis'

import os
os.makedirs(OUT, exist_ok=True)

FULL=3694; RS=1300; RE=3200; NF=1900; NFRAMES=5

SG_S=np.array([-8.3916083916e-02,2.0979020979e-02,1.0256410256e-01,1.6083916084e-01,
               1.9580419580e-01,2.0745920746e-01,1.9580419580e-01,1.6083916084e-01,
               1.0256410256e-01,2.0979020979e-02,-8.3916083916e-02])
SG_D=np.array([1.1655011655e-01,9.3240093240e-02,6.9930069930e-02,4.6620046620e-02,
               2.3310023310e-02,0.0,-2.3310023310e-02,-4.6620046620e-02,
               -6.9930069930e-02,-9.3240093240e-02,-1.1655011655e-01])

def sg(d,c):
    h=len(c)//2; o=np.copy(d)
    for i in range(h,len(d)-h): o[i]=np.sum(d[i-h:i+h+1]*c)
    return o

def ext(content,name,sz):
    mk=f"static const float {name}[{sz}] = {{"; s=content.find(mk)+len(mk); e=content.find("};",s)
    return np.array([float(x.strip()) for x in content[s:e].replace('f','').replace('\n','').split(',') if x.strip()])

def predict_pls(spectrum, bg, int_ms, content):
    sub=np.zeros(FULL)
    for i in range(FULL):
        bv=bg[i] if bg is not None else 0.0
        sub[i]=(bv-spectrum[i])/int_ms
    sm=sg(sub,SG_S); dr=sg(sm,SG_D); roi=dr[RS:RE]
    m=np.mean(roi); s=np.std(roi)
    roi=(roi-m)/(s if s>1e-8 else 1.0)
    coef=ext(content,"PLS_COEF",NF); xm=ext(content,"PLS_X_MEAN",NF); xs=ext(content,"PLS_X_STD",NF)
    return 4.2750242620+np.sum(((roi-xm)/np.maximum(xs,1e-8))*coef)

def load_bin(path):
    """Load binary frame data: uint16 raw pixels, NFRAMES x FULL"""
    data = np.fromfile(path, dtype=np.uint16)
    return data.reshape(NFRAMES, FULL)

# Load model and background
with open(model_h,'r') as f: content=f.read()
bg_raw = np.mean(pd.read_csv(bg_csv).iloc[:,1:].values.astype(float), axis=0)

# Also load Python monitor NPZ data for comparison
npz405 = np.mean(np.load(BASE+r'\ccd_monitor\projects\test_bug\rec_20260216_004637.npz')['pixels'], axis=0)
npz450 = np.mean(np.load(BASE+r'\ccd_monitor\projects\test_bug\rec_20260216_004702.npz')['pixels'], axis=0)

# Analyze last 3 measurements
lines = []
lines.append("SD CARD ANALYSIS")
lines.append("="*80)

for meas_idx in [8, 9, 10]:
    meas_dir = f"{SD}\\MEAS_{meas_idx:03d}"
    
    # Load frames
    l1_frames = load_bin(f"{meas_dir}\\laser1_data.bin")
    l2_frames = load_bin(f"{meas_dir}\\laser2_data.bin")
    
    # These frames are what the DMA callback gives (already inverted: 65535 - raw)
    l1_avg = np.mean(l1_frames.astype(float), axis=0)
    l2_avg = np.mean(l2_frames.astype(float), axis=0)
    
    lines.append(f"\n--- MEAS_{meas_idx:03d} (integration_time=100ms) ---")
    lines.append(f"Laser1 (405nm): min={l1_avg.min():.0f} max={l1_avg.max():.0f} mean={l1_avg.mean():.0f}")
    lines.append(f"Laser2 (450nm): min={l2_avg.min():.0f} max={l2_avg.max():.0f} mean={l2_avg.mean():.0f}")
    
    # Dark frame: the device captures dark before laser1
    # We don't have the dark frame saved to SD, but we can estimate from edges
    dark_est = np.full(FULL, np.mean(l1_avg[:200]))
    lines.append(f"Est dark level (from edges): {np.mean(l1_avg[:200]):.0f}")
    
    # Run predictions with different scenarios
    # IMPORTANT: device used int_time=100ms, but model was trained with 240-1000ms!
    for int_ms_label, int_ms in [("Device: 100ms", 100), ("If 1000ms", 1000)]:
        p1 = predict_pls(l1_avg, dark_est, float(int_ms), content)
        p2 = predict_pls(l2_avg, dark_est, float(int_ms), content)
        clamped1 = max(0.0, p1)
        disp1 = clamped1 if clamped1 != 0 else -3.0
        lines.append(f"  {int_ms_label}: Chl-a(405)={p1:.4f} display={disp1:.2f} | Chl-a(450)={p2:.4f}")
    
    # Plot spectra
    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f'MEAS_{meas_idx:03d} - SD Card Data Analysis', fontsize=14, fontweight='bold')
    
    # Plot 1: Raw inverted spectra (as device sees them)
    ax = axes[0,0]
    ax.plot(l1_avg, 'b-', alpha=0.7, label='Laser1 (405nm)', linewidth=0.5)
    ax.plot(l2_avg, 'r-', alpha=0.7, label='Laser2 (450nm)', linewidth=0.5)
    ax.axhline(y=dark_est[0], color='gray', linestyle='--', alpha=0.5, label=f'Dark est: {dark_est[0]:.0f}')
    ax.set_title('Inverted Signal (as device sees it)')
    ax.set_xlabel('Pixel')
    ax.set_ylabel('Inverted ADC Value')
    ax.legend()
    ax.set_xlim(0, 3694)
    
    # Plot 2: Background-subtracted (dark - signal = RawSig - RawBG)
    ax = axes[0,1]
    sub1 = dark_est - l1_avg
    sub2 = dark_est - l2_avg
    ax.plot(sub1, 'b-', alpha=0.7, label='405nm (dark-sig)', linewidth=0.5)
    ax.plot(sub2, 'r-', alpha=0.7, label='450nm (dark-sig)', linewidth=0.5)
    ax.axhline(y=0, color='gray', linestyle='--', alpha=0.3)
    ax.set_title('After Dark Subtraction (dark_accum - signal)')
    ax.set_xlabel('Pixel')
    ax.set_ylabel('Value')
    ax.legend()
    ax.set_xlim(0, 3694)
    
    # Plot 3: ROI region close-up
    ax = axes[1,0]
    ax.plot(range(RS, RE), l1_avg[RS:RE], 'b-', alpha=0.7, label='405nm', linewidth=0.5)
    ax.plot(range(RS, RE), l2_avg[RS:RE], 'r-', alpha=0.7, label='450nm', linewidth=0.5)
    ax.axhline(y=dark_est[0], color='gray', linestyle='--', alpha=0.5, label='Dark level')
    ax.set_title(f'ROI Region (pixels {RS}-{RE})')
    ax.set_xlabel('Pixel')
    ax.set_ylabel('Inverted ADC Value')
    ax.legend()
    
    # Plot 4: Compare SD vs NPZ (Python monitor)
    ax = axes[1,1]
    ax.plot(l1_avg, 'b-', alpha=0.5, label='SD Laser1 (405nm)', linewidth=0.5)
    ax.plot(npz405, 'c--', alpha=0.5, label='NPZ 405nm (monitor)', linewidth=0.5)
    ax.set_title('SD Card vs Python Monitor Data')
    ax.set_xlabel('Pixel')
    ax.set_ylabel('Inverted ADC Value')
    ax.legend()
    ax.set_xlim(0, 3694)
    
    plt.tight_layout()
    plt.savefig(f"{OUT}\\meas_{meas_idx:03d}_analysis.png", dpi=150)
    plt.close()
    lines.append(f"  Plot saved: sd_analysis/meas_{meas_idx:03d}_analysis.png")

# Compare SD data vs NPZ data
lines.append("\n" + "="*80)
lines.append("SD vs PYTHON MONITOR COMPARISON")
lines.append("="*80)

# MEAS_010 laser1 vs NPZ 405nm
l1_010 = np.mean(load_bin(f"{SD}\\MEAS_010\\laser1_data.bin").astype(float), axis=0)
lines.append(f"MEAS_010 Laser1 max: {l1_010.max():.0f}")
lines.append(f"NPZ 405nm max:       {npz405.max():.0f}")
lines.append(f"Correlation:         {np.corrcoef(l1_010, npz405)[0,1]:.6f}")
lines.append(f"Max abs diff:        {np.abs(l1_010 - npz405).max():.0f}")

# Training data comparison plot
fig, axes = plt.subplots(1, 2, figsize=(16, 6))

# Load a training sample for comparison
train_dir = BASE + r'\model2\chla2'
if os.path.exists(train_dir):
    train_files = sorted([f for f in os.listdir(train_dir) if f.endswith('.csv')])
    if train_files:
        train_sample = np.mean(pd.read_csv(f"{train_dir}\\{train_files[0]}").iloc[:,1:].values.astype(float), axis=0)
        # Training data is RAW (not inverted)
        train_sub = train_sample - bg_raw[:len(train_sample)]
        
        ax = axes[0]
        ax.plot(train_sub / 1000.0, 'g-', alpha=0.7, label=f'Training sample: {train_files[0]}', linewidth=0.5)
        
        # SD data: (dark_est - l1) = RawSig - RawBG
        sd_sub = (np.full(FULL, np.mean(l1_010[:200])) - l1_010)
        ax.plot(sd_sub / 100.0, 'b-', alpha=0.7, label='SD MEAS_010 (405nm, /100ms)', linewidth=0.5)
        ax.set_title('Training Data vs SD Data (after BG sub + int_time norm)')
        ax.set_xlabel('Pixel')
        ax.set_ylabel('Normalized Value')
        ax.legend()
        ax.set_xlim(0, 3694)
        
        lines.append(f"\nTraining sample: {train_files[0]}")
        lines.append(f"  Raw range: [{train_sample.min():.0f}, {train_sample.max():.0f}]")

ax = axes[1]
# Show what the preprocessed features look like
l1_010_sub = np.full(FULL, np.mean(l1_010[:200])) - l1_010
preprocessed = l1_010_sub / 100.0  # int_time=100
preprocessed = sg(preprocessed, SG_S)
preprocessed = sg(preprocessed, SG_D)
roi = preprocessed[RS:RE]
m = np.mean(roi); s = np.std(roi)
roi_snv = (roi - m) / (s if s > 1e-8 else 1.0)

pls_xm = ext(content, "PLS_X_MEAN", NF)
ax.plot(roi_snv, 'b-', alpha=0.7, label='SD data (preprocessed)', linewidth=0.5)
ax.plot(pls_xm, 'r-', alpha=0.3, label='PLS X Mean (training mean)', linewidth=0.5)
ax.set_title('Preprocessed ROI Features vs Training Mean')
ax.set_xlabel('Feature Index')
ax.set_ylabel('SNV Value')
ax.legend()

plt.tight_layout()
plt.savefig(f"{OUT}\\training_comparison.png", dpi=150)
plt.close()
lines.append(f"Training comparison plot saved: sd_analysis/training_comparison.png")

print("\n".join(lines))
