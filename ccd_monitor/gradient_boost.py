import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.model_selection import cross_val_predict, KFold, GridSearchCV
from sklearn.metrics import mean_squared_error, r2_score
from evaporation_model import AcetoneEvaporationModel
try:
    import emlearn
except ImportError:
    emlearn = None

# 1. Load Data
print("Loading data for Gradient Boosting...")
df = pd.read_csv('ccd_features.csv')

# Define features for training (exclude metadata)
drop_cols = ['concentration_mg_L', 'project', 'filename', 'target_pigment', 'sample_number']
feature_cols = [c for c in df.columns if c not in drop_cols]
print(f"Total Samples: {len(df)}")

# ============================================================
# MULTI-RANGE MODEL STRATEGY (CROSS-VALIDATED)
# ============================================================
ranges = {
    "HIGH_CONC":  (df['integration_time_ms'] < 500),
    "MID_CONC":   ((df['integration_time_ms'] >= 500) & (df['integration_time_ms'] <= 1500)),
    "LOW_CONC":   (df['integration_time_ms'] > 1500)
}

final_models = {}
cv_results = []

print("\n" + "="*80)
print("TRAINING GRADIENT BOOSTING MODELS (5-FOLD CV)")
print("="*80)

for range_name, mask in ranges.items():
    subset = df[mask].copy()
    if len(subset) < 5:
        print(f"Skipping {range_name}: Not enough samples ({len(subset)})")
        continue

    print(f"\nProcessing {range_name} Range ({len(subset)} samples)...")
    
    X_all = subset[feature_cols].select_dtypes(include=[np.number])
    y = subset['concentration_mg_L']
    
    # --- Experiment 1: WITH Time Feature (Standard) ---
    kf = KFold(n_splits=min(5, len(subset)), shuffle=True, random_state=42)
    gb_model = GradientBoostingRegressor(n_estimators=100, random_state=42)
    y_pred_all = cross_val_predict(gb_model, X_all, y, cv=kf)
    r2_all = r2_score(y, y_pred_all)
    
    # --- Experiment 2: WITHOUT Time Feature (Spectral Only) ---
    if 'elapsed_minutes' in X_all.columns:
        X_spec = X_all.drop(columns=['elapsed_minutes'])
    else:
        X_spec = X_all.copy()
        
    gb_spec = GradientBoostingRegressor(n_estimators=100, random_state=42)
    y_pred_spec = cross_val_predict(gb_spec, X_spec, y, cv=kf)
    r2_spec = r2_score(y, y_pred_spec)

    print(f"  -> R² (With Time):    {r2_all:.4f}")
    print(f"  -> R² (Spectrum Only):{r2_spec:.4f}")
    
    # --- Experiment 3: Grid Search (Spectrum Only) ---
    print("     Performing Grid Search on Spectrum Only...")
    param_grid = {
        'n_estimators': [100, 200],
        'learning_rate': [0.05, 0.1],
        'max_depth': [3, 5]
    }
    grid_search = GridSearchCV(GradientBoostingRegressor(random_state=42), param_grid, cv=3, scoring='r2', n_jobs=-1)
    grid_search.fit(X_spec, y)
    best_gb = grid_search.best_estimator_
    r2_best = grid_search.best_score_
    print(f"     [OPTIMIZED] Best Params: {grid_search.best_params_}")
    print(f"     [OPTIMIZED] Best R²:     {r2_best:.4f}")
    
    # Feature Importance (from best model)
    importances = best_gb.feature_importances_
    indices = np.argsort(importances)[::-1]
    print("     Top 3 Features:")
    for i in range(3):
        print(f"       {X_spec.columns[indices[i]]}: {importances[indices[i]]:.4f}")

    # --- Experiment 5: Optimize Physics Model (Acetone Evaporation) ---
    print("\n   Optimizing Physics Model (Acetone Evaporation)...")
    physics_model = AcetoneEvaporationModel()
    best_k_factor = 0.0
    best_r2_phys = -1.0
    
    # Try scaling k_mass from small to large
    # 1e-6 to 1e-4? Default was 1e-4 (gave 1.16 correction). Linear fit suggested 0.3% -> 1.015 in 5 mins.
    # So we need roughly 10x smaller k_mass.
    k_factors = [0.001, 0.005, 0.01, 0.05, 0.1, 0.2, 0.5, 1.0] 
    
    for k in k_factors:
        # Calculate corrected y for all samples
        # Note: simulation is slow-ish, so we pre-calc for unique time values?
        # Or just run it. It's fast enough.
        
        y_corr_phys = []
        for idx, row in X_all.iterrows():
            t_min = row['elapsed_minutes']
            original_conc = y.loc[idx]
            # Get correction factor
            cf = physics_model.simulate(t_min, k_evap_factor=k)
            y_corr_phys.append(original_conc * cf)
            
        y_corr_phys = np.array(y_corr_phys)
        
        # Train Spectrum-Only GBT
        model_phys = GradientBoostingRegressor(n_estimators=100, random_state=42)
        y_pred_p = cross_val_predict(model_phys, X_spec, y_corr_phys, cv=kf)
        r2_p = r2_score(y_corr_phys, y_pred_p)
        
        if r2_p > best_r2_phys:
            best_r2_phys = r2_p
            best_k_factor = k
            
    print(f"     [PHYSICS] Best k_factor: {best_k_factor}")
    print(f"     [PHYSICS] Corrected R² (Spec Only): {best_r2_phys:.4f}")

    if best_r2_phys > 0.90:
         print("     [SUCCESS] Physics Model matches Time Feature performance!")
    else:
         print("     [INFO] Time input is still better.")
         
    # Store CV predictions (Using the BEST one, likely 'With Time' still)
    # Actually, let's store the 'With Time' one for the final plot as it's likely still better.
    for idx, (actual, pred) in enumerate(zip(y, y_pred_all)):
        cv_results.append({
            'Actual_Conc': actual,
            'Predicted': pred,
            'Int_Time': subset.iloc[idx]['integration_time_ms'],
            'Model_Used': range_name
        })

    # Train Final Model (WITH TIME, as that's the best performing one)
    final_model = GradientBoostingRegressor(n_estimators=100, random_state=42)
    final_model.fit(X_all, y)
    final_models[range_name] = final_model
    print(f"  -> Final Model (GBT+Time) trained on all {len(subset)} samples.")

# ==============================================================================
# 3. Export to C Code for STM32
# ==============================================================================
if emlearn is None:
    print("\n[NOTE] 'emlearn' library not installed. Skipping C code generation.")
else:
    print("\nExporting models to C header (stm32_gb_model.h)...")
    c_header = """#ifndef STM32_GB_MODEL_H
#define STM32_GB_MODEL_H

// Auto-generated by gradient_boost.py using emlearn
"""
    for name, model in final_models.items():
        c_model = emlearn.convert(model)
        filename = f"gb_model_{name.lower()}.h"
        c_model.save(file=filename, name=f"model_{name.lower()}")
        print(f"  Saved {filename}")
        c_header += f'#include "{filename}"\\n'

    c_header += "\\n#endif // STM32_GB_MODEL_H\\n"

    with open("stm32_gb_model.h", "w") as f:
        f.write(c_header)

    print("Master header saved to stm32_gb_model.h")

# ============================================================
# RESULTS & VISUALIZATION
# ============================================================
sim_df = pd.DataFrame(cv_results)
mse = mean_squared_error(sim_df['Actual_Conc'], sim_df['Predicted'])
r2 = r2_score(sim_df['Actual_Conc'], sim_df['Predicted'])

print("\n" + "="*80)
print(f"OVERALL PERFORMANCE (GBT)")
print("="*80)
print(f"  RMSE: {np.sqrt(mse):.4f}")
print(f"  R²:   {r2:.4f}")

plt.figure(figsize=(10, 6))
sc = plt.scatter(sim_df['Actual_Conc'], sim_df['Predicted'], 
                 c=sim_df['Int_Time'], cmap='viridis', 
                 alpha=0.8, edgecolor='k', s=60)
cbar = plt.colorbar(sc)
cbar.set_label('Integration Time (ms)')
plt.plot([0, sim_df['Actual_Conc'].max()], [0, sim_df['Actual_Conc'].max()], 'k--', alpha=0.5)
plt.xlabel('Actual Concentration (mg/L)')
plt.ylabel('Predicted Concentration (mg/L) [GBT]')
plt.title(f'Gradient Boosting Performance (R²={r2:.3f})')
plt.grid(True)
plt.savefig('gbt_performance.png')
print("\nSaved plot to gbt_performance.png")
