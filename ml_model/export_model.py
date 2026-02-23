#!/usr/bin/env python3
"""
export_model.py — Train Chl-a (PLS) and Chl-b (PCA+SVR) models from ml_model data,
then export all parameters to a single C header file for STM32 inference.

Usage:
    cd ml_model/
    python export_model.py

Outputs:
    ../Core/Inc/chl_model_data.h
"""

import numpy as np
import pandas as pd
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.cross_decomposition import PLSRegression
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.svm import SVR
from sklearn.pipeline import Pipeline
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score, mean_squared_error

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent.resolve()
PROJECT_DIR = SCRIPT_DIR.parent

CHLA_DATA_DIR = SCRIPT_DIR / "data/chl_a"
CHLB_DATA_DIR = SCRIPT_DIR / "data/chl_b"
BACKGROUND_DIR = SCRIPT_DIR / "data/background_data"

CHLA_CSV = SCRIPT_DIR / "data/real_data/chla_data.csv"
CHLB_CSV = SCRIPT_DIR / "data/real_data/chlb_data.csv"

OUTPUT_HEADER = PROJECT_DIR / "Core" / "Inc" / "chl_model_data.h"

# Preprocessing params (must match training scripts exactly)
SG_SMOOTH_WINDOW = 11
SG_SMOOTH_POLY = 2
SG_DERIV_WINDOW = 11
SG_DERIV_POLY = 3
SG_DERIV_ORDER = 1

ROI_START = 1300
ROI_END = 3500
ROI_LEN = ROI_END - ROI_START  # 1900

# Model hyperparams
PLS_N_COMPONENTS = 9
PCA_VARIANCE = 0.98
SVR_C = 10
SVR_EPSILON = 0.01
SVR_KERNEL = "rbf"

TEST_SIZE = 0.2
RANDOM_STATE = 42

# Background file mapping
CHLA_BACKGROUND_MAP = {
    240: "background-240.csv",
    250: "background-250.csv",
    300: "background-300.csv",
    500: "background-500.csv",
    1000: "background-1000.csv",
}
CHLA_CLOSEST_BG = {580: 500, 700: 500}

CHLB_BACKGROUND_MAP = {
    1000: "background-1000.csv",
    1200: "background-1200.csv",
    1500: "background-1500.csv",
    3000: "background-3000.csv",
    5000: "background-5000.csv",
}

# ============================================================
# DATA LOADING
# ============================================================

def load_reference_data(csv_path):
    try:
        df = pd.read_csv(csv_path)
    except Exception as e:
        print(f"Error reading {csv_path}: {e}")
        return pd.DataFrame()
    
    # Normalize column names
    df.columns = [c.lower().strip() for c in df.columns]
    
    # Handle column aliases
    if "solution num" in df.columns:
        df.rename(columns={"solution num": "sample"}, inplace=True)
    if "sample con" in df.columns:
        df.rename(columns={"sample con": "concentration"}, inplace=True)
    if "integration time(ms)" in df.columns:
        df.rename(columns={"integration time(ms)": "integration_time"}, inplace=True)

    # Extract sample number
    if "sample" in df.columns:
        df["sample_num"] = df["sample"].astype(str).str.extract(r"(\d+)").astype(float).astype(int)
    else:
        print(f"Warning: 'sample' column missing in {csv_path}. Columns found: {df.columns}")
        return pd.DataFrame()

    # Convert numeric columns
    df["concentration"] = pd.to_numeric(df["concentration"], errors="coerce")
    
    if "integration_time" in df.columns:
        df["integration_time"] = pd.to_numeric(df["integration_time"], errors="coerce")
    else:
        df["integration_time"] = 1000  # Default if missing

    return df.dropna(subset=["concentration"]).sort_values("sample_num").reset_index(drop=True)


def load_and_average_spectrum(csv_path):
    df = pd.read_csv(csv_path)
    df = df.iloc[:, 1:]  # Drop frame_number column
    data = df.values.astype(float)
    return np.mean(data, axis=0)


def load_background(integration_time, bg_map, closest_bg, bg_dir):
    bg_int = closest_bg.get(integration_time, integration_time)
    bg_file = bg_map.get(bg_int)
    if bg_file is None:
        return None
    bg_path = bg_dir / bg_file
    if not bg_path.exists():
        return None
    return load_and_average_spectrum(bg_path)


# ============================================================
# PREPROCESSING (identical to training scripts)
# ============================================================

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


def build_dataset(ref_data, data_dir, bg_map, closest_bg):
    X_list, y_list = [], []
    bg_cache = {}

    for _, row in ref_data.iterrows():
        sample_num = int(row["sample_num"])
        concentration = row["concentration"]
        int_time = int(row["integration_time"])

        csv_path = data_dir / f"{sample_num}.csv"
        if not csv_path.exists():
            continue

        spectrum = load_and_average_spectrum(csv_path)

        if int_time not in bg_cache:
            bg_cache[int_time] = load_background(int_time, bg_map, closest_bg, BACKGROUND_DIR)
        bg = bg_cache[int_time]
        if bg is not None:
            min_len = min(len(spectrum), len(bg))
            spectrum = spectrum[:min_len] - bg[:min_len]
            
            # Normalize
            spectrum = spectrum / (int_time + 1e-8)
            
            # Preprocess
            processed = preprocess_spectrum(spectrum, ROI_START, ROI_END)
            
            X_list.append(processed)
            y_list.append(concentration)

    min_features = min(len(x) for x in X_list)
    X = np.array([x[:min_features] for x in X_list])
    y = np.array(y_list)
    return X, y


# ============================================================
# C HEADER GENERATION
# ============================================================

def format_c_array(name, arr, dtype="float", per_line=8):
    lines = []
    lines.append(f"static const {dtype} {name}[{len(arr)}] = {{")
    for i in range(0, len(arr), per_line):
        chunk = arr[i:i + per_line]
        vals = ", ".join(f"{v:.10e}f" for v in chunk)
        if i + per_line < len(arr):
            vals += ","
        lines.append(f"    {vals}")
    lines.append("};")
    return "\n".join(lines)


def format_c_2d_array(name, arr_2d, dtype="float", per_line=8):
    """Format a 2D array as a flat C array (row-major)."""
    rows, cols = arr_2d.shape
    flat = arr_2d.flatten()
    lines = []
    lines.append(f"/* {rows} x {cols} matrix, row-major */")
    lines.append(f"static const {dtype} {name}[{len(flat)}] = {{")
    for i in range(0, len(flat), per_line):
        chunk = flat[i:i + per_line]
        vals = ", ".join(f"{v:.10e}f" for v in chunk)
        if i + per_line < len(flat):
            vals += ","
        lines.append(f"    {vals}")
    lines.append("};")
    return "\n".join(lines)


def compute_sg_coefficients():
    """Compute SG filter coefficients as convolution kernels."""
    # Smoothing coefficients
    impulse = np.zeros(SG_SMOOTH_WINDOW)
    impulse[SG_SMOOTH_WINDOW // 2] = 1.0
    smooth_coeffs = savgol_filter(impulse, window_length=SG_SMOOTH_WINDOW,
                                   polyorder=SG_SMOOTH_POLY)

    # Derivative coefficients
    impulse = np.zeros(SG_DERIV_WINDOW)
    impulse[SG_DERIV_WINDOW // 2] = 1.0
    deriv_coeffs = savgol_filter(impulse, window_length=SG_DERIV_WINDOW,
                                  polyorder=SG_DERIV_POLY, deriv=SG_DERIV_ORDER)

    return smooth_coeffs, deriv_coeffs


def generate_header(pls_model, chlb_pipeline, sg_smooth, sg_deriv, n_features):
    """Generate the C header file content."""

    # Extract PLS parameters
    # sklearn PLSRegression with scale=True uses internal scaling
    pls_x_mean = pls_model._x_mean.flatten()
    pls_x_std = pls_model._x_std.flatten()
    pls_coef = pls_model.coef_.flatten()
    pls_intercept = pls_model.intercept_.flatten()[0]

    # Extract Chl-b pipeline parameters
    scaler = chlb_pipeline.named_steps['scaler']
    pca = chlb_pipeline.named_steps['pca']
    svr = chlb_pipeline.named_steps['svr']

    scaler_mean = scaler.mean_
    scaler_scale = scaler.scale_
    pca_components = pca.components_  # shape: (n_pca, n_features)
    pca_mean = pca.mean_
    n_pca = pca_components.shape[0]

    # SVR with RBF kernel parameters
    svr_support_vectors = svr.support_vectors_  # shape: (n_sv, n_pca)
    svr_dual_coef = svr.dual_coef_.flatten()    # shape: (n_sv,)
    svr_intercept = svr.intercept_[0]
    n_sv = svr_support_vectors.shape[0]

    # gamma='scale' means gamma = 1 / (n_features_in * X.var())
    # We need to compute the actual gamma value used
    if svr.gamma == 'scale':
        svr_gamma = svr._gamma  # sklearn stores computed gamma here
    else:
        svr_gamma = svr.gamma

    lines = []
    lines.append("/*")
    lines.append(" * chl_model_data.h — Auto-generated ML model parameters")
    lines.append(f" * Generated by export_model.py")
    lines.append(" *")
    lines.append(f" * Chl-a: PLS Regression ({PLS_N_COMPONENTS} components)")
    lines.append(f" * Chl-b: PCA ({n_pca} components) + SVR (RBF, {n_sv} support vectors)")
    lines.append(f" * Features: {n_features} (pixels {ROI_START}-{ROI_END})")
    lines.append(" *")
    lines.append(" * DO NOT EDIT — regenerate with: python ml_model/export_model.py")
    lines.append(" */")
    lines.append("")
    lines.append("#ifndef CHL_MODEL_DATA_H")
    lines.append("#define CHL_MODEL_DATA_H")
    lines.append("")

    # Dimensions
    lines.append(f"#define NUM_FEATURES         {n_features}")
    lines.append(f"#define ROI_START            {ROI_START}")
    lines.append(f"#define ROI_END              {ROI_END}")
    lines.append(f"#define FULL_SPECTRUM_LEN    3694")
    lines.append("")

    # SG filter params
    lines.append(f"#define SG_SMOOTH_WINDOW     {SG_SMOOTH_WINDOW}")
    lines.append(f"#define SG_SMOOTH_HALF       {SG_SMOOTH_WINDOW // 2}")
    lines.append(f"#define SG_DERIV_WINDOW      {SG_DERIV_WINDOW}")
    lines.append(f"#define SG_DERIV_HALF        {SG_DERIV_WINDOW // 2}")
    lines.append("")

    lines.append("/* Savitzky-Golay smoothing coefficients */")
    lines.append(format_c_array("SG_SMOOTH_COEFFS", sg_smooth))
    lines.append("")
    lines.append("/* Savitzky-Golay derivative coefficients */")
    lines.append(format_c_array("SG_DERIV_COEFFS", sg_deriv))
    lines.append("")

    # --- Chl-a PLS ---
    lines.append("/* ========== Chl-a PLS Model ========== */")
    lines.append(f"#define PLS_N_COMPONENTS     {PLS_N_COMPONENTS}")
    lines.append(f"#define PLS_INTERCEPT        {pls_intercept:.10e}f")
    lines.append("")
    lines.append(format_c_array("PLS_COEF", pls_coef))
    lines.append("")
    lines.append(format_c_array("PLS_X_MEAN", pls_x_mean))
    lines.append("")
    lines.append(format_c_array("PLS_X_STD", pls_x_std))
    lines.append("")

    # --- Chl-b PCA+SVR ---
    lines.append("/* ========== Chl-b PCA+SVR Model ========== */")
    lines.append(f"#define CHLB_N_PCA           {n_pca}")
    lines.append(f"#define CHLB_N_SV            {n_sv}")
    lines.append(f"#define CHLB_SVR_GAMMA       {svr_gamma:.10e}f")
    lines.append(f"#define CHLB_SVR_INTERCEPT   {svr_intercept:.10e}f")
    lines.append("")

    lines.append("/* StandardScaler mean (applied before PCA) */")
    lines.append(format_c_array("CHLB_SCALER_MEAN", scaler_mean))
    lines.append("")
    lines.append("/* StandardScaler scale (applied before PCA) */")
    lines.append(format_c_array("CHLB_SCALER_SCALE", scaler_scale))
    lines.append("")
    lines.append("/* PCA mean (subtracted before projection) */")
    lines.append(format_c_array("CHLB_PCA_MEAN", pca_mean))
    lines.append("")
    lines.append(f"/* PCA components: {n_pca} x {n_features} */")
    lines.append(format_c_2d_array("CHLB_PCA_COMPONENTS", pca_components))
    lines.append("")
    lines.append(f"/* SVR support vectors: {n_sv} x {n_pca} */")
    lines.append(format_c_2d_array("CHLB_SVR_SUPPORT_VECTORS", svr_support_vectors))
    lines.append("")
    lines.append("/* SVR dual coefficients */")
    lines.append(format_c_array("CHLB_SVR_DUAL_COEF", svr_dual_coef))
    lines.append("")

    lines.append("#endif /* CHL_MODEL_DATA_H */")

    return "\n".join(lines)


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  ML Model Export for STM32")
    print("=" * 60)

    # --- Train Chl-a PLS ---
    print("\n[1/4] Loading Chl-a data...")
    chla_ref = load_reference_data(CHLA_CSV)
    X_a, y_a = build_dataset(chla_ref, CHLA_DATA_DIR, CHLA_BACKGROUND_MAP, CHLA_CLOSEST_BG)
    print(f"  Samples: {len(y_a)}, Features: {X_a.shape[1]}")

    X_a_train, X_a_test, y_a_train, y_a_test = train_test_split(
        X_a, y_a, test_size=TEST_SIZE, random_state=RANDOM_STATE)

    print(f"\n[2/4] Training Chl-a PLS ({PLS_N_COMPONENTS} components)...")
    pls = PLSRegression(n_components=PLS_N_COMPONENTS, scale=True)
    pls.fit(X_a_train, y_a_train)

    y_a_pred_train = pls.predict(X_a_train).ravel()
    y_a_pred_test = pls.predict(X_a_test).ravel()
    print(f"  Train R²: {r2_score(y_a_train, y_a_pred_train):.4f}  "
          f"RMSE: {np.sqrt(mean_squared_error(y_a_train, y_a_pred_train)):.4f}")
    print(f"  Test  R²: {r2_score(y_a_test, y_a_pred_test):.4f}  "
          f"RMSE: {np.sqrt(mean_squared_error(y_a_test, y_a_pred_test)):.4f}")

    # --- Train Chl-b PCA+SVR ---
    print(f"\n[3/4] Loading Chl-b data...")
    chlb_ref = load_reference_data(CHLB_CSV)
    X_b, y_b = build_dataset(chlb_ref, CHLB_DATA_DIR, CHLB_BACKGROUND_MAP, {})
    print(f"  Samples: {len(y_b)}, Features: {X_b.shape[1]}")

    X_b_train, X_b_test, y_b_train, y_b_test = train_test_split(
        X_b, y_b, test_size=TEST_SIZE, random_state=RANDOM_STATE)

    print(f"  Training Chl-b PCA({PCA_VARIANCE}) + SVR(C={SVR_C})...")
    chlb_pipeline = Pipeline([
        ('scaler', StandardScaler()),
        ('pca', PCA(n_components=PCA_VARIANCE)),
        ('svr', SVR(kernel=SVR_KERNEL, C=SVR_C, epsilon=SVR_EPSILON, gamma='scale'))
    ])
    chlb_pipeline.fit(X_b_train, y_b_train)

    y_b_pred_train = chlb_pipeline.predict(X_b_train)
    y_b_pred_test = chlb_pipeline.predict(X_b_test)
    print(f"  Train R²: {r2_score(y_b_train, y_b_pred_train):.4f}  "
          f"RMSE: {np.sqrt(mean_squared_error(y_b_train, y_b_pred_train)):.4f}")
    print(f"  Test  R²: {r2_score(y_b_test, y_b_pred_test):.4f}  "
          f"RMSE: {np.sqrt(mean_squared_error(y_b_test, y_b_pred_test)):.4f}")

    # --- Export ---
    print(f"\n[4/4] Exporting to {OUTPUT_HEADER}...")
    sg_smooth, sg_deriv = compute_sg_coefficients()

    n_features = X_a.shape[1]
    header_content = generate_header(pls, chlb_pipeline, sg_smooth, sg_deriv, n_features)

    OUTPUT_HEADER.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_HEADER.write_text(header_content)

    n_pca = chlb_pipeline.named_steps['pca'].n_components_
    n_sv = len(chlb_pipeline.named_steps['svr'].support_vectors_)
    file_size = OUTPUT_HEADER.stat().st_size

    print(f"\n  Output: {OUTPUT_HEADER}")
    print(f"  File size: {file_size / 1024:.1f} KB")
    print(f"  PLS features: {n_features}")
    print(f"  PCA components: {n_pca}")
    print(f"  SVR support vectors: {n_sv}")

    # Estimate Flash usage
    pls_flash = n_features * 3 * 4  # coef + mean + std, 4 bytes each
    pca_flash = n_pca * n_features * 4 + n_features * 4 * 2  # components + scaler mean/scale
    svr_flash = n_sv * n_pca * 4 + n_sv * 4  # support_vectors + dual_coef
    total_flash = pls_flash + pca_flash + svr_flash + SG_SMOOTH_WINDOW * 4 + SG_DERIV_WINDOW * 4
    print(f"\n  Estimated Flash usage: {total_flash / 1024:.1f} KB")
    print("\nDone!")
