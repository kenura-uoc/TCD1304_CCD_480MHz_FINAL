import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import cross_val_predict, KFold
from sklearn.metrics import mean_squared_error, r2_score

# 1. Load Data
print("Loading data...")
df = pd.read_csv('ccd_features.csv')

# Define features for training (exclude metadata)
drop_cols = ['concentration_mg_L', 'project', 'filename', 'target_pigment', 'sample_number']
feature_cols = [c for c in df.columns if c not in drop_cols]
print(f"Total Samples: {len(df)}")

# ============================================================
# MULTI-RANGE MODEL STRATEGY (CROSS-VALIDATED)
# ============================================================
# We use 5-Fold Cross-Validation to evaluate performance.
# This ensures every sample is tested once while being trained on the rest.

ranges = {
    "HIGH_CONC":  (df['integration_time_ms'] < 500),
    "MID_CONC":   ((df['integration_time_ms'] >= 500) & (df['integration_time_ms'] <= 1500)),
    "LOW_CONC":   (df['integration_time_ms'] > 1500)
}

final_models = {}
cv_results = []

print("\n" + "="*80)
print("TRAINING & VALIDATING MULTI-RANGE MODELS (5-FOLD CV)")
print("="*80)

for range_name, mask in ranges.items():
    subset = df[mask].copy()
    if len(subset) < 5:
        print(f"Skipping {range_name}: Not enough samples ({len(subset)}) for 5-Fold CV")
        continue

    print(f"\nProcessing {range_name} Range ({len(subset)} samples)...")
    
    X = subset[feature_cols]
    y = subset['concentration_mg_L']
    X = X.select_dtypes(include=[np.number])
    
    # 1. Cross-Validation for Evaluation (The "Scientific" Check)
    # We use K=5 (or fewer if samples are very low)
    n_splits = min(5, len(subset))
    kf = KFold(n_splits=n_splits, shuffle=True, random_state=42)
    
    model = RandomForestRegressor(n_estimators=100, random_state=42)
    y_pred_cv = cross_val_predict(model, X, y, cv=kf)
    
    mse_cv = mean_squared_error(y, y_pred_cv)
    r2_cv = r2_score(y, y_pred_cv)
    
    print(f"  -> Cross-Validated R²: {r2_cv:.4f}")
    print(f"  -> Cross-Validated MSE: {mse_cv:.4f}")
    
    # Store CV predictions for plotting (simulating performance on unseen data)
    for idx, (actual, pred) in enumerate(zip(y, y_pred_cv)):
        cv_results.append({
            'Actual_Conc': actual,
            'Predicted': pred,
            'Int_Time': subset.iloc[idx]['integration_time_ms'],
            'Model_Used': range_name
        })

    # 2. Train Final Model on ALL Data (For Deployment)
    final_model = RandomForestRegressor(n_estimators=100, random_state=42)
    final_model.fit(X, y)
    final_models[range_name] = final_model
    print(f"  -> Final Model trained on all {len(subset)} samples.")

# ============================================================
# RESULTS & VISUALIZATION
# ============================================================
sim_df = pd.DataFrame(cv_results)

# Calculate Overall Metrics
mse = mean_squared_error(sim_df['Actual_Conc'], sim_df['Predicted'])
r2 = r2_score(sim_df['Actual_Conc'], sim_df['Predicted'])

print("\n" + "="*80)
print(f"OVERALL PERFORMANCE (CROSS-VALIDATED)")
print("="*80)
print(f"  MSE: {mse:.4f}")
print(f"  R²:  {r2:.4f}")
print(f"  (This represents expected accuracy on totally new samples)")

print("\nSample Predictions (CV):")
print(sim_df.sort_values('Actual_Conc', ascending=False)[['Actual_Conc', 'Predicted', 'Int_Time', 'Model_Used']].to_string(index=False))

# Plot
plt.figure(figsize=(10, 6))

# Use a colormap to show the dynamic range of integration times
sc = plt.scatter(sim_df['Actual_Conc'], sim_df['Predicted'], 
                 c=sim_df['Int_Time'], cmap='viridis', 
                 alpha=0.8, edgecolor='k', s=60)

cbar = plt.colorbar(sc)
cbar.set_label('Integration Time (ms)')

plt.plot([0, sim_df['Actual_Conc'].max()], [0, sim_df['Actual_Conc'].max()], 'k--', alpha=0.5)
plt.xlabel('Actual Concentration (mg/L)')
plt.ylabel('Predicted Concentration (mg/L) [CV]')
plt.title(f'Cross-Validated Model Performance (R²={r2:.3f})')
plt.grid(True)
plt.savefig('cv_performance.png')
print("\nSaved plot to cv_performance.png")
