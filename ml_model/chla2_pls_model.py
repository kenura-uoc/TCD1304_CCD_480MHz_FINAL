import numpy as np
import pandas as pd
import warnings
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import train_test_split, KFold, cross_val_score
from sklearn.metrics import r2_score, mean_squared_error
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore")

# ============================================================
# CONFIGURATION
# ============================================================

DATA_DIR = Path(r"C:\Users\user\Downloads\data files of project\chla2")
EXCEL_FILE = DATA_DIR / "chlorophyll data.xlsx"

BACKGROUND_MAP = {
    240:  "background-240.csv",
    250:  "background-250.csv",
    300:  "background-300.csv",
    500:  "background-500.csv",
    1000: "background-1000.csv",
}

CLOSEST_BG = {
    580: 500,
    700: 500,
}

MAX_PLS_COMPONENTS = 15
TEST_SIZE = 0.2
RANDOM_STATE = 42

# ============================================================
# 1. LOAD REFERENCE DATA
# ============================================================

def load_reference_data(excel_path):
    df = pd.read_excel(excel_path, header=None, skiprows=2)
    df.columns = ["sample", "concentration", "integration_time"]
    df["sample_num"] = df["sample"].str.extract(r"(\d+)").astype(int)
    df["concentration"] = pd.to_numeric(df["concentration"])
    df["integration_time"] = pd.to_numeric(df["integration_time"])
    return df.sort_values("sample_num").reset_index(drop=True)

# ============================================================
# 2. LOAD & AVERAGE SPECTRA
# ============================================================

def load_and_average_spectrum(csv_path):
    df = pd.read_csv(csv_path)
    df = df.iloc[:, 1:]
    data = df.values.astype(float)
    return np.mean(data, axis=0)

def load_background(integration_time):
    bg_int_time = CLOSEST_BG.get(integration_time, integration_time)
    bg_file = BACKGROUND_MAP.get(bg_int_time)

    if bg_file is None:
        return None

    bg_path = DATA_DIR / bg_file
    if not bg_path.exists():
        return None

    return load_and_average_spectrum(bg_path)

# ============================================================
# 3. PREPROCESSING PIPELINE
# ============================================================

def preprocess_spectrum(spectrum):

    # 1️⃣ Savitzky–Golay smoothing
    smoothed = savgol_filter(spectrum, window_length=11, polyorder=2)

    # 2️⃣ First derivative
    deriv = savgol_filter(smoothed, window_length=11, polyorder=3, deriv=1)

    # 3️⃣ SNV normalization
    mean = np.mean(deriv)
    std = np.std(deriv)

    snv = (deriv - mean) / (std + 1e-8)

    return snv

# ============================================================
# 4. BUILD DATASET (CORRECT INTEGRATION NORMALIZATION)
# ============================================================

def build_dataset(ref_data):

    X_list = []
    y_list = []
    bg_cache = {}

    for _, row in ref_data.iterrows():

        sample_num = int(row["sample_num"])
        concentration = row["concentration"]
        int_time = int(row["integration_time"])

        csv_path = DATA_DIR / f"{sample_num}.csv"
        if not csv_path.exists():
            continue

        spectrum = load_and_average_spectrum(csv_path)

        # Load background (cached)
        if int_time not in bg_cache:
            bg_cache[int_time] = load_background(int_time)

        bg = bg_cache[int_time]

        # Background subtraction
        if bg is not None:
            min_len = min(len(spectrum), len(bg))
            spectrum = spectrum[:min_len] - bg[:min_len]

        # 🔥 Correct physical normalization
        spectrum = spectrum / (int_time + 1e-8)

        # Preprocess
        spectrum_processed = preprocess_spectrum(spectrum)

        # 🔥 NEW: KEEP ONLY PIXELS 1500 TO 3000 🔥
        spectrum_processed = spectrum_processed[1300:3200]

        X_list.append(spectrum_processed)
        y_list.append(concentration)

    min_features = min(len(x) for x in X_list)
    X = np.array([x[:min_features] for x in X_list])
    y = np.array(y_list)

    return X, y

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    # Load data
    ref_data = load_reference_data(EXCEL_FILE)
    X, y = build_dataset(ref_data)

    print("Total samples:", len(y))
    print("Features per sample (after cropping):", X.shape[1])

    # Train/Test split
    X_train, X_test, y_train, y_test = train_test_split(
        X, y,
        test_size=TEST_SIZE,
        random_state=RANDOM_STATE
    )

    print("Train samples:", len(y_train))
    print("Test samples :", len(y_test))

    # 🔥 We found 9 is the best from the elbow graph 🔥
    # Note: If the new cropped data behaves differently, 
    # you might want to re-run the CV check later!
    best_n = 9 
    
    print("Best PLS components selected:", best_n)

    # Train final model with 9 components
    pls = PLSRegression(n_components=best_n, scale=True)
    pls.fit(X_train, y_train)

    # Predictions
    y_train_pred = pls.predict(X_train).ravel()
    y_test_pred = pls.predict(X_test).ravel()

    # Metrics
    train_r2 = r2_score(y_train, y_train_pred)
    test_r2 = r2_score(y_test, y_test_pred)

    train_rmse = np.sqrt(mean_squared_error(y_train, y_train_pred))
    test_rmse = np.sqrt(mean_squared_error(y_test, y_test_pred))

    print("\n===== TRAIN RESULTS =====")
    print("R2   :", train_r2)
    print("RMSE :", train_rmse)

    print("\n===== TEST RESULTS =====")
    print("R2   :", test_r2)
    print("RMSE :", test_rmse)

    # ========================================================
    # PLOTS
    # ========================================================

    # Train plot
    plt.figure(figsize=(6,6))
    plt.scatter(y_train, y_train_pred)
    lims = [min(y_train.min(), y_train_pred.min()),
            max(y_train.max(), y_train_pred.max())]
    plt.plot(lims, lims, 'r--')
    plt.title(f"TRAIN (R2={train_r2:.3f})")
    plt.xlabel("Actual")
    plt.ylabel("Predicted")
    plt.grid(True)
    plt.show()

    # Test plot
    plt.figure(figsize=(6,6))
    plt.scatter(y_test, y_test_pred)
    lims = [min(y_test.min(), y_test_pred.min()),
            max(y_test.max(), y_test_pred.max())]
    plt.plot(lims, lims, 'r--')
    plt.title(f"TEST (R2={test_r2:.3f})")
    plt.xlabel("Actual")
    plt.ylabel("Predicted")
    plt.grid(True)
    plt.show()