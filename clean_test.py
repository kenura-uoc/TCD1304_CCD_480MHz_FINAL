#!/usr/bin/env python3
"""Clean analysis - collect results first, then print."""
import numpy as np
import pandas as pd

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
p405 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004637.npz'
p450 = BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_004702.npz'
bg_csv = BASE + r'\background_data\background-1000.csv'
model_h = BASE + r'\Core\Inc\chl_model_data.h'

FULL=3694; RS=1300; RE=3200; NF=1900

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

def predict(spectrum, bg, int_ms, content):
    sub=np.zeros(FULL)
    for i in range(FULL):
        bv=bg[i] if bg is not None else 0.0
        sub[i]=(bv-spectrum[i])/int_ms
    sm=sg(sub,SG_S); dr=sg(sm,SG_D); roi=dr[RS:RE]
    m=np.mean(roi); s=np.std(roi)
    roi=(roi-m)/(s if s>1e-8 else 1.0)
    coef=ext(content,"PLS_COEF",NF); xm=ext(content,"PLS_X_MEAN",NF); xs=ext(content,"PLS_X_STD",NF)
    return 4.2750242620+np.sum(((roi-xm)/np.maximum(xs,1e-8))*coef)

# Load everything first, no printing
with open(model_h,'r') as f: content=f.read()
bg_raw=np.mean(pd.read_csv(bg_csv).iloc[:,1:].values.astype(float),axis=0)
avg405_all=np.mean(np.load(p405)['pixels'],axis=0)
avg450_all=np.mean(np.load(p450)['pixels'],axis=0)
avg405_8=np.mean(np.load(p405)['pixels'][:8],axis=0)
avg450_8=np.mean(np.load(p450)['pixels'][:8],axis=0)
n405=np.load(p405)['pixels'].shape[0]
n450=np.load(p450)['pixels'].shape[0]

dark_edge=np.full(FULL, np.mean(avg405_all[:200]))
dark_csv=65535-bg_raw

# Collect all results
R = []

for label,sig,bgd in [
    ("405nm+edge_dark",avg405_all,dark_edge),
    ("405nm+csv_dark",avg405_all,dark_csv),
    ("405nm+no_bg",avg405_all,None),
    ("405nm(8fr)+edge_dark",avg405_8,dark_edge),
    ("405nm(8fr)+csv_dark",avg405_8,dark_csv),
    ("450nm+edge_dark",avg450_all,dark_edge),
    ("450nm+csv_dark",avg450_all,dark_csv),
    ("450nm+no_bg",avg450_all,None),
]:
    raw_pred=predict(sig,bgd,1000.0,content)
    clamped=max(0.0,raw_pred)
    display=clamped if clamped!=0.0 else -3.0
    R.append((label,raw_pred,clamped,display))

# Also test old data
old405=np.mean(np.load(BASE+r'\ccd_monitor\projects\Default\rec_20260215_235828_405nm.npz')['pixels'],axis=0)
for label,sig,bgd in [
    ("OLD_405nm+edge_dark",old405,np.full(FULL,np.mean(old405[:200]))),
    ("OLD_405nm+csv_dark",old405,dark_csv),
]:
    raw_pred=predict(sig,bgd,1000.0,content)
    R.append((label,raw_pred,max(0.0,raw_pred),max(0.0,raw_pred) if max(0.0,raw_pred)!=0 else -3.0))

# Now print everything at once
lines = []
lines.append("RESULTS TABLE")
lines.append("="*80)
lines.append(f"{'Scenario':<30} {'Unclamped':>12} {'Clamped':>12} {'Display':>12}")
lines.append("-"*80)
for label,raw,clamp,disp in R:
    lines.append(f"{label:<30} {raw:>12.4f} {clamp:>12.4f} {disp:>12.4f}")

lines.append("")
lines.append(f"Real lab values: A=2.84, B=1.60")
lines.append(f"Device reported: A=-3.00 (error), B=1.80")
lines.append("")
lines.append(f"405nm frames: {n405}, 450nm frames: {n450}")
lines.append(f"Edge dark level (inverted): {np.mean(avg405_all[:200]):.1f}")
lines.append(f"CSV BG mean (raw): {np.mean(bg_raw):.1f}")
lines.append(f"CSV BG inverted mean: {65535-np.mean(bg_raw):.1f}")
lines.append(f"Old 405nm max inverted: {old405.max():.1f}")
lines.append(f"New 405nm max inverted: {avg405_all.max():.1f}")

print("\n".join(lines))
