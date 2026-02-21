
import numpy as np
import pandas as pd
import joblib
from pathlib import Path
from scipy.signal import savgol_filter

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent.resolve()
MODELS_DIR = SCRIPT_DIR / "models"
DATA_DIR = SCRIPT_DIR.parent / "ccd_monitor/projects/16-2-2026"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

# Files to specificy
FILES = [
    # (Filename, Model, Integration Time)
    ("rec_20260216_133810.npz", "chla_model.pkl", 300, "Chl-A"),
    ("rec_20260216_133914.npz", "chlb_model.pkl", 300, "Chl-B"),
    ("rec_20260216_134320.npz", "chla_model.pkl", 1000, "Chl-A"),
    ("rec_20260216_134352.npz", "chlb_model.pkl", 1000, "Chl-B"),
]

# Preprocessing Constants
ROI_START = 1300
ROI_END = 3500
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY = 2
SG_DERIV_WINDOW = 11
SG_DERIV_POLY = 3
SG_DERIV_ORDER = 1

# ============================================================
# HELPER FUNCTIONS
# ============================================================
def load_spectrum_npz(path):
    data = np.load(path)
    if 'pixels' in data:
        # Assuming 'pixels' is (N_frames, 3694)
        pixels = data['pixels']
        if pixels.ndim > 1:
            return np.mean(pixels, axis=0)
        return pixels
    return None

def load_bg_csv(path):
    df = pd.read_csv(path)
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def preprocess_spectrum(spectrum):
    # 1. Slice ROI FIRST
    sliced = spectrum[ROI_START:ROI_END]
    
    # 2. Smooth
    smoothed = savgol_filter(sliced, window_length=SG_SMOOTH_WINDOW, polyorder=SG_SMOOTH_POLY)
    
    # 3. Deriv
    deriv = savgol_filter(smoothed, window_length=SG_DERIV_WINDOW, polyorder=SG_DERIV_POLY, deriv=SG_DERIV_ORDER)
    
    # 4. SNV
    mean = np.mean(deriv)
    std = np.std(deriv)
    snv = (deriv - mean) / (std + 1e-8)
    
    return snv

def main():
    print("--- Manual Prediction for Sol 9 ---")
    
    # Load Models
    models = {}
    try:
        models["chla_model.pkl"] = joblib.load(MODELS_DIR / "chla_model.pkl")
        models["chlb_model.pkl"] = joblib.load(MODELS_DIR / "chlb_model.pkl")
    except Exception as e:
        print(f"Error loading models: {e}")
        return

    # Load Backgrounds
    backgrounds = {}
    bg_300_path = BACKGROUND_DIR / "background-300.csv"
    bg_1000_path = BACKGROUND_DIR / "background-1000.csv"
    
    if bg_300_path.exists():
        backgrounds[300] = load_bg_csv(bg_300_path)
    if bg_1000_path.exists():
        backgrounds[1000] = load_bg_csv(bg_1000_path)
        
    print(f"Loaded backgrounds for: {list(backgrounds.keys())} ms")

    # Process Files
    results = []
    
    for fname, model_name, int_time, label in FILES:
        fpath = DATA_DIR / fname
        if not fpath.exists():
            print(f"File not found: {fname}")
            continue
            
        # Load Spectrum
        spec = load_spectrum_npz(fpath)
        if spec is None:
            print(f"Failed to load spectrum from {fname}")
            continue

        # AUTOMATIC INVERSION CHECK
        # Legacy Data (High=Dark): Bright signal is Low (~10k)
        # Inverted Data (High=Light): Bright signal is High (~55k)
        spec_mean = np.mean(spec)
        if spec_mean < 30000:
            print(f"  [INFO] Data appears to be Legacy (Mean={spec_mean:.0f} < 30k). Inverting...")
            spec = 65535 - spec
        else:
            print(f"  [INFO] Data appears to be Inverted (Mean={spec_mean:.0f} > 30k).")
            
        # Background Subtraction
        bg = backgrounds.get(int_time)
        if bg is None:
            print(f"No background for {int_time}ms (using zeros)")
            bg = np.zeros_like(spec)
            
        # Baseline Correction (Offset matching)
        # Use first 100 pixels (dummy/dark) to align baselines
        spec_offset = np.mean(spec[10:100])
        bg_offset = np.mean(bg[10:100])
        offset_diff = spec_offset - bg_offset
        
        print(f"File {fname}: Baseline Offset = {offset_diff:.2f}")
        
        # Align Spec to Bg
        spec_aligned = spec - offset_diff
        
        min_len = min(len(spec), len(bg))
        net = spec_aligned[:min_len] - bg[:min_len]
        
        # Check signal level
        peak_val = np.max(net)
        print(f"Processing {fname} ({int_time}ms): Max Net Signal = {peak_val:.2f}")
        
        # Inversion Check (Heuristic)
        # If signal is strongly negative, maybe it needs inversion?
        # But we assume models are trained on High=Light.
        
        # Norm
        norm = net / (int_time + 1e-8)
        
        # Preprocess
        processed = preprocess_spectrum(norm)
        
        # Feature shape for sklearn (1, n_features)
        X = processed.reshape(1, -1)
        
        # Predict
        model = models.get(model_name)
        if model:
            pred = model.predict(X)
            conc = pred[0]
            # Handle SVR vs PLS return types (sometimes array, sometimes float)
            if isinstance(conc, (list, np.ndarray)):
                conc = conc[0] # Assuming single target PLS
                
            results.append({
                "File": fname,
                "Type": label,
                "Int Time": int_time,
                "Concentration": conc
            })
            print(f"  -> Prediction: {conc:.4f} mg/L")
        else:
            print(f"  Model {model_name} not found.")

    print("\n--- Summary ---")
    df_res = pd.DataFrame(results)
    print(df_res)

if __name__ == "__main__":
    main()
