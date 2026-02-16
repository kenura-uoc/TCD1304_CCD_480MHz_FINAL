import numpy as np, pandas as pd

BASE = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL'
FULL=3694; RS=1300; RE=3200; NF=1900
SG_S = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
    1.9580419580e-01, 2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
    1.0256410256e-01, 2.0979020979e-02, -8.3916083916e-02])
SG_D = np.array([1.1655011655e-01, 9.3240093240e-02, 6.9930069930e-02, 4.6620046620e-02,
    2.3310023310e-02, 0.0, -2.3310023310e-02, -4.6620046620e-02,
    -6.9930069930e-02, -9.3240093240e-02, -1.1655011655e-01])

def sg(d, c):
    h = len(c) // 2
    o = np.copy(d)
    for i in range(h, len(d) - h):
        o[i] = np.sum(d[i - h:i + h + 1] * c)
    return o

with open(BASE + r'\Core\Inc\chl_model_data.h', 'r') as f:
    ct = f.read()

def ext(n, sz):
    mk = f'static const float {n}[{sz}] = {{'
    s = ct.find(mk) + len(mk)
    e = ct.find('};', s)
    return np.array([float(x.strip()) for x in ct[s:e].replace('f', '').replace('\n', '').split(',') if x.strip()])

co = ext('PLS_COEF', NF)
xm = ext('PLS_X_MEAN', NF)
xs = ext('PLS_X_STD', NF)

def pred_correct(sp, it):
    """CORRECT: SNV on full spectrum THEN crop (matches training)"""
    dk = np.full(FULL, np.mean(sp[:200]))
    sub = (dk - sp) / it
    sm = sg(sub, SG_S)
    dr = sg(sm, SG_D)
    m = np.mean(dr)
    s = np.std(dr)
    dr = (dr - m) / (s if s > 1e-8 else 1.0)
    roi = dr[RS:RE]
    return 4.2750242620 + np.sum(((roi - xm) / np.maximum(xs, 1e-8)) * co)

def pred_wrong(sp, it):
    """WRONG (current firmware): crop THEN SNV on ROI only"""
    dk = np.full(FULL, np.mean(sp[:200]))
    sub = (dk - sp) / it
    sm = sg(sub, SG_S)
    dr = sg(sm, SG_D)
    roi = dr[RS:RE]
    m = np.mean(roi)
    s = np.std(roi)
    roi = (roi - m) / (s if s > 1e-8 else 1.0)
    return 4.2750242620 + np.sum(((roi - xm) / np.maximum(xs, 1e-8)) * co)

R = []
R.append('CORRECT ORDER (SNV full->crop) vs WRONG (crop->SNV roi)')
R.append('=' * 70)

for fn, it, lbl in [
    ('rec_20260216_020230.npz', 300, '300ms-405'),
    ('rec_20260216_020258.npz', 300, '300ms-450'),
    ('rec_20260216_020341.npz', 1000, '1000ms-405'),
    ('rec_20260216_020358.npz', 1000, '1000ms-450'),
]:
    avg = np.mean(np.load(BASE + r'\ccd_monitor\projects\test_bug\\' + fn)['pixels'], axis=0)
    pc = pred_correct(avg, it)
    pw = pred_wrong(avg, it)
    R.append(f'{lbl}: CORRECT={pc:.4f}  WRONG={pw:.4f}')

R.append('')
R.append('300ms vs 1000ms consistency check (same sample, CORRECT SNV):')
a300 = np.mean(np.load(BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_020230.npz')['pixels'], axis=0)
a1000 = np.mean(np.load(BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_020341.npz')['pixels'], axis=0)
R.append(f'  405nm@300ms  = {pred_correct(a300, 300):.4f}')
R.append(f'  405nm@1000ms = {pred_correct(a1000, 1000):.4f}')

b300 = np.mean(np.load(BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_020258.npz')['pixels'], axis=0)
b1000 = np.mean(np.load(BASE + r'\ccd_monitor\projects\test_bug\rec_20260216_020358.npz')['pixels'], axis=0)
R.append(f'  450nm@300ms  = {pred_correct(b300, 300):.4f}')
R.append(f'  450nm@1000ms = {pred_correct(b1000, 1000):.4f}')

R.append('')
R.append(f'Real values: A=2.84, B=1.60')
R.append(f'Device @1000ms: A=4.00, B=3.01')
R.append(f'Device @300ms:  A=-3.00, B=2.59')

print('\n'.join(R))
