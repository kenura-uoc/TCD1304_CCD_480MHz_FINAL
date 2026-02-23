
import numpy as np
import pandas as pd
import joblib
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.cross_decomposition import PLSRegression
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.svm import SVR
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split, cross_val_score, KFold
from sklearn.metrics import mean_squared_error, r2_score

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent.resolve()
DATA_DIR_A = SCRIPT_DIR / "data/chl_a"
DATA_DIR_B = SCRIPT_DIR / "data/chl_b"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

# Using CSV metadata instead of Excel
CSV_A = SCRIPT_DIR / "data/real_data/chla_data.csv"
CSV_B = SCRIPT_DIR / "data/real_data/chlb_data.csv"

OUTPUT_MODEL_A = SCRIPT_DIR / "models/chla_model.pkl"
OUTPUT_MODEL_B = SCRIPT_DIR / "models/chlb_model.pkl"

# Constants
PLS_N_COMPONENTS_A = 9
PCA_VARIANCE_B = 0.98
SVR_C_B = 10
SVR_EPSILON_B = 0.01

ROI_START = 1300
ROI_END = 3500

# Preprocessing Constants (Matcing README/Legacy)
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY = 2
SG_DERIV_WINDOW = 11
SG_DERIV_POLY = 3
SG_DERIV_ORDER = 1

# Background Maps
BACKGROUND_MAP_A = {
    240: "background-240.csv",
    250: "background-250.csv",
    300: "background-300.csv",
    500: "background-500.csv",
    1000: "background-1000.csv",
}
CLOSEST_BG_A = {580: 500, 700: 500}

BACKGROUND_MAP_B = {
    1000: "background-1000.csv",
    1200: "background-1200.csv",
    1500: "background-1500.csv",
    3000: "background-3000.csv",
    5000: "background-5000.csv",
}

# ============================================================
# HELPER FUNCTIONS
# ============================================================

def load_spectrum(path):
    df = pd.read_csv(path)
    data = df.iloc[:, 1:].values.astype(float)
    return np.mean(data, axis=0)

def load_bg(int_time, bg_map, closest_map, bg_dir):
    time_key = closest_map.get(int_time, int_time)
    fname = bg_map.get(time_key)
    if not fname: return None
    path = bg_dir / fname
    if not path.exists(): return None
    return load_spectrum(path)

def preprocess_spectrum(spectrum, roi_start=ROI_START, roi_end=ROI_END):
    # 1. Slice ROI FIRST (Remove dummy pixels)
    sliced = spectrum[roi_start:roi_end]
    
    # 2. Savitzky-Golay Smoothing
    smoothed = savgol_filter(sliced, window_length=SG_SMOOTH_WINDOW, polyorder=SG_SMOOTH_POLY)
    
    # 3. Savitzky-Golay Derivative
    deriv = savgol_filter(smoothed, window_length=SG_DERIV_WINDOW, polyorder=SG_DERIV_POLY, deriv=SG_DERIV_ORDER)
    
    # 4. SNV (Standard Normal Variate) on ROI data
    mean = np.mean(deriv)
    std = np.std(deriv)
    snv = (deriv - mean) / (std + 1e-8)
    
    return snv

def build_dataset(csv_path, data_dir, bg_map, closest_map):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return np.array([]), np.array([])

    df.columns = [c.lower().strip() for c in df.columns]
    
    # Handle aliases
    if "solution num" in df.columns:
        df.rename(columns={"solution num": "sample"}, inplace=True)
    if "sample con" in df.columns:
        df.rename(columns={"sample con": "concentration"}, inplace=True)
    if "integration time(ms)" in df.columns:
        df.rename(columns={"integration time(ms)": "integration_time"}, inplace=True)
    
    if "sample" in df.columns:
        df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)
    else:
        print("Column 'sample' not found in CSV")
        return np.array([]), np.array([])

    df["concentration"] = pd.to_numeric(df["concentration"], errors="coerce")
    if "integration_time" in df.columns:
         df["integration_time"] = pd.to_numeric(df["integration_time"], errors="coerce")
    else:
        df["integration_time"] = 1000 # Default if missing
    df = df.dropna(subset=["concentration", "integration_time"])
    
    X = []
    y = []
    bg_cache = {}
    
    for _, row in df.iterrows():
        s_num = int(row["sample_num"])
        conc = row["concentration"]
        it = int(row["integration_time"])
        
        fpath = data_dir / f"{s_num}.csv"
        if not fpath.exists(): continue
        
        spec = load_spectrum(fpath)
        
        if it not in bg_cache:
            bg_cache[it] = load_bg(it, bg_map, closest_map, BACKGROUND_DIR)
        
        bg = bg_cache[it]
        if bg is not None:
            min_len = min(len(spec), len(bg))
            spec = spec[:min_len] - bg[:min_len]
            
            # Normalize by integration time
            spec = spec / (it + 1e-8)

            # Preprocess (Slice -> Smooth -> Deriv -> SNV)
            processed = preprocess_spectrum(spec, ROI_START, ROI_END)
            
            X.append(processed)
            y.append(conc)
        
    if not X: return np.array([]), np.array([])
    X = np.array(X)
    y = np.array(y)
    return X, y

def evaluate_model(model, X, y, name="Model"):
    print(f"\n--- {name} Evaluation ---")
    
    # 1. Fit on all data
    model.fit(X, y)
    y_pred = model.predict(X)
    
    # 2. Training Metrics
    mse = mean_squared_error(y, y_pred)
    rmse = np.sqrt(mse)
    r2 = r2_score(y, y_pred)
    
    print(f"  Training RMSE: {rmse:.4f}")
    print(f"  Training R2:   {r2:.4f}")
    
    # 3. Cross-Validation (5-Fold)
    if len(X) < 5:
        print("  Not enough samples for 5-Fold CV.")
        return model

    kf = KFold(n_splits=5, shuffle=True, random_state=42)
    scores_r2 = cross_val_score(model, X, y, cv=kf, scoring='r2')
    scores_neg_mse = cross_val_score(model, X, y, cv=kf, scoring='neg_mean_squared_error')
    rmse_cv = np.sqrt(-scores_neg_mse.mean())
    
    print(f"  CV R2 (Mean):  {scores_r2.mean():.4f} (±{scores_r2.std():.4f})")
    print(f"  CV RMSE (Mean): {rmse_cv:.4f}")
    
    if r2 - scores_r2.mean() > 0.1:
        print("  [WARNING] Significant drop in CV score -> Possible Overfitting!")
    else:
        print("  [OK] Model generalizes well.")

    return model

# ============================================================
# MAIN TRAINING
# ============================================================
def train_and_save():
    print("Training Chl A Model...")
    # 1. Base Data
    Xa, ya = build_dataset(CSV_A, DATA_DIR_A, BACKGROUND_MAP_A, CLOSEST_BG_A)
    
    # 2. Append NPZ Data (New Samples)
    npz_path = SCRIPT_DIR.parent / "dataset.npz"
    if npz_path.exists():
        try:
            print(f"Loading new samples from {npz_path}...")
            data = np.load(npz_path, allow_pickle=True)
            if 'concentrations' in data and 'laser_a_spectra' in data and 'dark_spectra' in data:
                concs = data['concentrations'] # [ChlA, ChlB]
                laser_a = data['laser_a_spectra']
                dark = data['dark_spectra']
                meta = data['metadata']
                
                new_X = []
                new_y = []
                
                for i in range(len(concs)):
                    it = 1000 # Default
                    if i < len(meta):
                        it = meta[i].get('integration_time', 1000)
                        
                    spec = laser_a[i].astype(float)
                    dk = dark[i].astype(float)
                    net = spec - dk
                    
                    # Normalize
                    norm = net / (it + 1e-8)
                    
                    processed = preprocess_spectrum(norm, ROI_START, ROI_END)
                    
                    new_X.append(processed)
                    new_y.append(concs[i][0]) # Chl A conc
                    
                if new_X:
                    new_X = np.array(new_X)
                    new_y = np.array(new_y)
                    if len(Xa) > 0:
                        Xa = np.vstack([Xa, new_X])
                        ya = np.concatenate([ya, new_y])
                    else:
                        Xa = new_X
                        ya = new_y
                    print(f"  Added {len(new_X)} new samples to Chl A training set.")
        except Exception as e:
            print(f"Failed to load NPZ data: {e}")

    if len(Xa) > 0:
        ma = PLSRegression(n_components=PLS_N_COMPONENTS_A, scale=True)
        # Evaluate & Fit
        ma = evaluate_model(ma, Xa, ya, "Chl-A (PLS)")
        
        joblib.dump(ma, OUTPUT_MODEL_A)
        print(f"Saved Chl A Model to {OUTPUT_MODEL_A}")
    else:
        print("No data for Chl A")

    print("\nTraining Chl B Model...")
    Xb, yb = build_dataset(CSV_B, DATA_DIR_B, BACKGROUND_MAP_B, {})
    
    # 2. Append NPZ Data (Chl B)
    if npz_path.exists():
        try:
            data = np.load(npz_path, allow_pickle=True)
            if 'concentrations' in data and 'laser_b_spectra' in data and 'dark_spectra' in data:
                concs = data['concentrations']
                laser_b = data['laser_b_spectra']
                dark = data['dark_spectra']
                meta = data['metadata']
                
                new_X = []
                new_y = []
                
                for i in range(len(concs)):
                    it = 1000
                    if i < len(meta):
                        it = meta[i].get('integration_time', 1000)
                        
                    spec = laser_b[i].astype(float)
                    dk = dark[i].astype(float)
                    net = spec - dk
                    
                    norm = net / (it + 1e-8)
                    processed = preprocess_spectrum(norm, ROI_START, ROI_END)
                    
                    new_X.append(processed)
                    new_y.append(concs[i][1]) # Chl B conc
                    
                if new_X:
                    new_X = np.array(new_X)
                    new_y = np.array(new_y)
                    if len(Xb) > 0:
                        Xb = np.vstack([Xb, new_X])
                        yb = np.concatenate([yb, new_y])
                    else:
                        Xb = new_X
                        yb = new_y
                    print(f"  Added {len(new_X)} new samples to Chl B training set.")
        except:
            pass

    if len(Xb) > 0:
        mb = Pipeline([
            ('scaler', StandardScaler()),
            ('pca', PCA(n_components=PCA_VARIANCE_B)),
            ('svr', SVR(kernel="rbf", C=SVR_C_B, epsilon=SVR_EPSILON_B, gamma='scale'))
        ])
        
        # Evaluate & Fit
        mb = evaluate_model(mb, Xb, yb, "Chl-B (PCA+SVR)")
        
        joblib.dump(mb, OUTPUT_MODEL_B)
        print(f"Saved Chl B Model to {OUTPUT_MODEL_B}")
    else:
        print("No data for Chl B")

if __name__ == "__main__":
    train_and_save()
