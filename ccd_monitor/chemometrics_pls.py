import numpy as np
import matplotlib.pyplot as plt
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import cross_val_predict, KFold
from sklearn.metrics import mean_squared_error, r2_score

# ============================================================
# 1. LOAD DATA
# ============================================================
print("Loading spectra_data.npz...")
data = np.load('spectra_data.npz')
X_raw = data['spectra']              # Shape: (92, 3648)
y = data['concentrations']           # Shape: (92,)
int_times = data['integration_times'] # Shape: (92,)

print(f"Loaded {X_raw.shape[0]} samples with {X_raw.shape[1]} wavelengths.")

# ============================================================
# 2. PREPROCESSING: SNV (Standard Normal Variate)
# ============================================================
def snv(input_data):
    """
    Standard Normal Variate (SNV) Normalization.
    Subtract mean, divide by standard deviation for EACH spectrum.
    Portable to STM32.
    """
    # Define a small epsilon to avoid division by zero
    epsilon = 1e-10 
    
    # Calculate mean and std for each sample (row)
    row_means = np.mean(input_data, axis=1, keepdims=True)
    row_stds = np.std(input_data, axis=1, keepdims=True) + epsilon
    
    return (input_data - row_means) / row_stds

print("Applying SNV normalization...")
X_snv = snv(X_raw)

# ============================================================
# 3. PLS REGRESSION (Partial Least Squares) - PER RANGE
# ============================================================
# Linear PLS struggles with the full dynamic range due to saturation.
# We split it into High/Mid/Low ranges, similar to the Random Forest approach.

ranges = {
    "HIGH_CONC":  (int_times < 500),
    "MID_CONC":   ((int_times >= 500) & (int_times <= 1500)),
    "LOW_CONC":   (int_times > 1500)
}

# Store for plotting
all_y_actual = []
all_y_pred = []
all_int_times = []

print("\nTraining PLS Models per Range...")

for range_name, mask in ranges.items():
    if np.sum(mask) < 5:
        continue
        
    X_subset = X_snv[mask]
    y_subset = y[mask]
    int_subset = int_times[mask]
    
    pls = PLSRegression(n_components=5) # Reduced components for small samples
    
    # Cross-validation
    y_pred_sub = cross_val_predict(pls, X_subset, y_subset, cv=min(5, len(y_subset))).flatten()
    r2 = r2_score(y_subset, y_pred_sub)
    print(f"  {range_name} Range ({np.sum(mask)} samples): R² = {r2:.4f}")
    
    # Collect for plot
    all_y_actual.extend(y_subset)
    all_y_pred.extend(y_pred_sub)
    all_int_times.extend(int_subset)
    
    # Train final
    pls.fit(X_subset, y_subset)
    
    # Extract coefficients (handling sklearn version differences)
    # Target: Y = Intercept + dot(X_snv, Coeffs)
    # PLS model: Y_centered = X_centered * B
    # Y - Y_mean = (X - X_mean) * B
    # Y = Y_mean - X_mean*B + X*B
    
    # Safe attribute access
    y_mean = getattr(pls, '_y_mean', getattr(pls, 'y_mean_', 0))
    x_mean = getattr(pls, '_x_mean', getattr(pls, 'x_mean_', 0))
    # std is handled by SNV step, not PLS (usually PLS centers but doesn't scale if scale=False)
    
    # Calculate effective intercept
    coeffs = pls.coef_.flatten()
    # x_mean might be (N_features,) or (1, N_features). Flatten to be safe.
    x_mean_flat = x_mean.flatten() if hasattr(x_mean, 'flatten') else np.array([0])
    
    # Check if we actually got a mean vector
    if x_mean_flat.shape[0] != coeffs.shape[0]:
         # If x_mean is 0 (default from getattr failure), dot is 0
         dot_prod = 0.0
    else:
         dot_prod = np.dot(x_mean_flat, coeffs)

    intercept = y_mean - dot_prod
    
    # If intercept is an array, get scalar
    if np.ndim(intercept) > 0:
        intercept = intercept[0]
    
    print(f"    -> STM32 Coeffs: Intercept={intercept:.4f}, Weights Shape={coeffs.shape}")

# ============================================================
# 5. VISUALIZATION
# ============================================================
if len(all_y_actual) > 0:
    all_y_actual = np.array(all_y_actual)
    all_y_pred = np.array(all_y_pred)
    all_int_times = np.array(all_int_times)
    
    overall_r2 = r2_score(all_y_actual, all_y_pred)

    plt.figure(figsize=(10, 6))
    sc = plt.scatter(all_y_actual, all_y_pred, c=all_int_times, cmap='viridis', edgecolor='k', s=60)
    plt.colorbar(sc, label='Integration Time (ms)')
    plt.plot([all_y_actual.min(), all_y_actual.max()], [all_y_actual.min(), all_y_actual.max()], 'k--', alpha=0.5)
    plt.xlabel('Actual Concentration')
    plt.ylabel('Predicted Concentration (PLS)')
    plt.title(f'PLS Regression Performance (Combined R²={overall_r2:.3f})')
    plt.grid(True)
    plt.savefig('pls_performance.png')
    print("\nSaved plot to pls_performance.png")

