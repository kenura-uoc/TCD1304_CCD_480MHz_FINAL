import numpy as np
import pandas as pd
import warnings
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.svm import SVR
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score, mean_squared_error
import matplotlib.pyplot as plt
import joblib

warnings.filterwarnings("ignore")

# ============================================================
# CONFIGURATION
# ============================================================
DATA_DIR = Path(__file__).parent / "data/chl_b"
CSV_FILE = Path(__file__).parent / "data/real_data/chlb_data.csv"

BACKGROUND_DIR = Path(__file__).parent / "data/background_data"

BACKGROUND_MAP = {
    1000: "background-1000.csv",
    1200: "background-1200.csv",
    1500: "background-1500.csv",
    3000: "background-3000.csv",
    5000: "background-5000.csv",
}

TEST_SIZE = 0.2
RANDOM_STATE = 42

# ============================================================
# 1. DATA LOADING FUNCTIONS
# ============================================================
def load_reference_data(csv_path):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return pd.DataFrame()

    # Clean up column names just in case
    df.columns = [c.lower().strip() for c in df.columns]

    if "sample" in df.columns:
        df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)
    
    df["concentration"] = pd.to_numeric(df["concentration"], errors="coerce")
    if "integration_time" in df.columns:
        df["integration_time"] = pd.to_numeric(df["integration_time"], errors="coerce")
    else:
        df["integration_time"] = 1000

    return df.dropna(subset=["concentration"]).sort_values("sample_num").reset_index(drop=True)

def load_and_average_spectrum(csv_path):
    df = pd.read_csv(csv_path)
    df = df.iloc[:, 1:]
    data = df.values.astype(float)
    return np.mean(data, axis=0)

def load_background(integration_time):
    bg_file = BACKGROUND_MAP.get(integration_time)
    if bg_file is None: return None
    bg_path = BACKGROUND_DIR / bg_file
    return load_and_average_spectrum(bg_path) if bg_path.exists() else None

# ============================================================
# 2. PREPROCESSING PIPELINE
# ============================================================
def preprocess_spectrum(spectrum):
    # Savitzky–Golay smoothing
    smoothed = savgol_filter(spectrum, window_length=11, polyorder=2)
    # First derivative to remove baseline shifts
    deriv = savgol_filter(smoothed, window_length=11, polyorder=3, deriv=1)
    # SNV normalization
    snv = (deriv - np.mean(deriv)) / (np.std(deriv) + 1e-8)
    return snv

def build_dataset(ref_data):
    X_list, y_list = [], []
    bg_cache = {}

    for _, row in ref_data.iterrows():
        sample_num, concentration, int_time = int(row["sample_num"]), row["concentration"], int(row["integration_time"])
        csv_path = DATA_DIR / f"{sample_num}.csv"
        
        if not csv_path.exists(): continue

        spectrum = load_and_average_spectrum(csv_path)
        if int_time not in bg_cache:
            bg_cache[int_time] = load_background(int_time)
        
        bg = bg_cache[int_time]
        if bg is not None:
            min_len = min(len(spectrum), len(bg))
            spectrum = spectrum[:min_len] - bg[:min_len]

        spectrum = spectrum / (int_time + 1e-8)
        processed = preprocess_spectrum(spectrum)[1300:3200] # ROI Selection

        X_list.append(processed)
        y_list.append(concentration)

    return np.array(X_list), np.array(y_list)

# ============================================================
# 3. MAIN TRAINING ENGINE
# ============================================================
if __name__ == "__main__":
    # Load and Split
    ref_data = load_reference_data(CSV_FILE)
    X, y = build_dataset(ref_data)
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=TEST_SIZE, random_state=RANDOM_STATE)

    print(f"Dataset Built. Training on {len(y_train)} samples, Testing on {len(y_test)}.")

    # Define Pipeline: Scale -> PCA -> SVR
    # PCA reduces the 1900 features to the most important "shapes" in the spectra
    # 🔥 Best parameters from 5-fold CV: C=10, epsilon=0.01, gamma='scale'
    
    best_model = Pipeline([
        ('scaler', StandardScaler()),
        ('pca', PCA(n_components=0.98)),  # Retain 98% of variance
        ('svr', SVR(kernel="rbf", C=10, epsilon=0.01, gamma='scale'))
    ])

    print("Training with optimal params: C=10, epsilon=0.01, gamma='scale'")
    best_model.fit(X_train, y_train)

    # Evaluation
    y_train_pred = best_model.predict(X_train)
    y_test_pred = best_model.predict(X_test)

    train_r2, test_r2 = r2_score(y_train, y_train_pred), r2_score(y_test, y_test_pred)
    train_rmse, test_rmse = np.sqrt(mean_squared_error(y_train, y_train_pred)), np.sqrt(mean_squared_error(y_test, y_test_pred))

    print(f"\nRESULTS:\nTrain R2: {train_r2:.3f} | Train RMSE: {train_rmse:.3f}")
    print(f"Test  R2: {test_r2:.3f} | Test  RMSE: {test_rmse:.3f}")

    # Save finalized model
    save_path = DATA_DIR / "chlb2_optimized_pipeline.pkl"
    joblib.dump(best_model, save_path)
    print(f"Model saved to {save_path}")

    # Visualization
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    ax1.scatter(y_train, y_train_pred, color='blue', alpha=0.6, edgecolors='k')
    ax1.plot([y.min(), y.max()], [y.min(), y.max()], 'r--')
    ax1.set_title(f"Train Set\nR2={train_r2:.3f}")
    
    ax2.scatter(y_test, y_test_pred, color='orange', alpha=0.6, edgecolors='k')
    ax2.plot([y.min(), y.max()], [y.min(), y.max()], 'r--')
    ax2.set_title(f"Test Set (Generalization)\nR2={test_r2:.3f}")
    
    for ax in [ax1, ax2]:
        ax.set_xlabel("Actual Concentration")
        ax.set_ylabel("Predicted Concentration")
        ax.grid(True)
        
    plt.tight_layout()
    plt.show()