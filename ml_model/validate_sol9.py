
import numpy as np
import pandas as pd
import joblib
import matplotlib.pyplot as plt
import scipy.stats as stats
from pathlib import Path
from scipy.signal import savgol_filter

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent.resolve()
MODELS_DIR = SCRIPT_DIR / "models"
PROJECT_DIR = SCRIPT_DIR.parent / "ccd_monitor/projects/16-2-2026"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

# Ground Truth for Sol 9 (from Excel)
ACTUAL_A = 3.0465
ACTUAL_B = 1.865

# Records to Validate
RECORDS = [
    {
        "name": "Record 4 (300ms)",
        "file_405": "rec_20260216_133810.npz",
        "file_450": "rec_20260216_133914.npz",
        "int_time": 300
    },
    {
        "name": "Record 5 (1000ms)",
        "file_405": "rec_20260216_134320.npz",
        "file_450": "rec_20260216_134352.npz",
        "int_time": 1000
    }
]

# Preprocessing Constants
ROI_START = 1300
ROI_END = 3500
SG_WIN = 11
SG_POLY = 2
SG_DWIN = 11
SG_DPOLY = 3

# ============================================================
# HELPERS
# ============================================================
def load_npz(fname):
    path = PROJECT_DIR / fname
    if not path.exists(): return None
    data = np.load(path)
    return np.mean(data['pixels'], axis=0) if 'pixels' in data else None

def load_bg(int_time):
    path = BACKGROUND_DIR / f"background-{int_time}.csv"
    if not path.exists(): return None
    df = pd.read_csv(path)
    return np.mean(df.iloc[:, 1:].values.astype(float), axis=0)

def preprocess(raw, bg, int_time):
    # Inversion Check
    if np.mean(raw) < 30000: raw = 65535 - raw
    if bg is not None:
        # Align offset
        offset = np.mean(raw[10:100]) - np.mean(bg[10:100])
        net = (raw - offset) - bg
    else:
        net = raw
    
    # ROI and Norm
    roi = net[ROI_START:ROI_END] / int_time
    
    # SG Processing
    sm = savgol_filter(roi, SG_WIN, SG_POLY)
    drv = savgol_filter(sm, SG_DWIN, SG_DPOLY, deriv=1)
    
    # SNV
    return (drv - drv.mean()) / (drv.std() + 1e-12)

def main():
    print(f"--- Sol 9 Validation Results ---")
    print(f"Actual: Chl-a = {ACTUAL_A} | Chl-b = {ACTUAL_B}\n")
    
    # Load Models
    model_chla = joblib.load(MODELS_DIR / "chla_model.pkl")
    model_chlb = joblib.load(MODELS_DIR / "chlb_model.pkl")
    combined_pkg = joblib.load(MODELS_DIR / "combined_dual_analyte_pls.joblib")
    model_combined = combined_pkg['model']

    all_residuals = []
    
    for rec in RECORDS:
        print(f"Processing {rec['name']}...")
        
        # Load Raw
        spec_405 = load_npz(rec['file_405'])
        spec_450 = load_npz(rec['file_450'])
        bg = load_bg(rec['int_time'])
        
        # Preprocess
        feat_405 = preprocess(spec_405, bg, rec['int_time'])
        feat_450 = preprocess(spec_450, bg, rec['int_time'])
        
        # 1. Individual Models
        pred_a_ind = model_chla.predict(feat_405.reshape(1, -1))[0]
        pred_b_ind = model_chlb.predict(feat_450.reshape(1, -1))[0]
        
        # 2. Combined Model
        X_combined = np.hstack([feat_405, feat_450]).reshape(1, -1)
        preds_comb = model_combined.predict(X_combined)[0]
        
        print(f"  Individual Models: Chl-a={pred_a_ind:.4f}, Chl-b={pred_b_ind:.4f}")
        print(f"  Combined Model:   Chl-a={preds_comb[0]:.4f}, Chl-b={preds_comb[1]:.4f}")
        
        # Store for stats
        all_residuals.append({
            "target": "Chl-a", "actual": ACTUAL_A, "pred": preds_comb[0], "rec": rec['name']
        })
        all_residuals.append({
            "target": "Chl-b", "actual": ACTUAL_B, "pred": preds_comb[1], "rec": rec['name']
        })
        print("-" * 20)

    # Statistical Plotting
    res_df = pd.DataFrame(all_residuals)
    res_df['error'] = res_df['actual'] - res_df['pred']
    
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    
    # 1. Residuals vs Predicted
    axes[0].scatter(res_df['pred'], res_df['error'], c='blue', alpha=0.6)
    axes[0].axhline(0, color='red', linestyle='--')
    axes[0].set_xlabel("Predicted (mg/L)")
    axes[0].set_ylabel("Residual (Actual - Pred)")
    axes[0].set_title("Residuals vs. Predicted")
    
    # 2. Histogram
    axes[1].hist(res_df['error'], bins=5, color='green', alpha=0.6, density=True)
    axes[1].set_title("Residual Distribution")
    axes[1].set_xlabel("Error (mg/L)")
    
    # 3. Q-Q Plot
    stats.probplot(res_df['error'], dist="norm", plot=axes[2])
    axes[2].set_title("Normality Q-Q Plot")
    
    plt.tight_layout()
    plot_path = SCRIPT_DIR / "training_plots" / "sol9_residual_analysis.png"
    plt.savefig(plot_path)
    print(f"\nStats plot saved to: {plot_path}")

if __name__ == "__main__":
    main()
