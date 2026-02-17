import pandas as pd
import numpy as np
import shutil
from pathlib import Path

# ============================================================
# CONFIGURATION
# ============================================================
SCRIPT_DIR = Path(__file__).parent
DATA_ROOT = SCRIPT_DIR / "data"
DIRS_TO_PROCESS = [
    DATA_ROOT / "chl_a",
    DATA_ROOT / "chl_b", 
    DATA_ROOT / "background_data"
]
BACKUP_ROOT = SCRIPT_DIR / "data_backup_before_inversion"

def process_file(file_path):
    try:
        # Check if file exists
        if not file_path.exists():
            print(f"  File not found: {file_path}")
            return

        df = pd.read_csv(file_path)
        
        # Heuristic: Check if already inverted
        # High=Dark (Legacy): ~58000 for background/dark. Mean > 30000.
        # High=Light (New): ~7500 for background/dark. Mean < 30000.
        
        # Calculate mean of numeric columns only (skipping first index col)
        numeric_cols = []
        for col in df.columns[1:]: # Skip "frame_number"
             if np.issubdtype(df[col].dtype, np.number):
                 numeric_cols.append(col)
        
        if not numeric_cols:
            print(f"  Skipped {file_path.name} (No numeric columns found)")
            return

        global_mean = df[numeric_cols].values.mean()
        
        if global_mean < 30000:
            print(f"  Skipped {file_path.name} (Mean {global_mean:.0f} < 30000, likely already inverted)")
            return
            
        print(f"  Inverting {file_path.name} (Mean: {global_mean:.0f} -> High=Dark detected)")

        inverted_count = 0
        for col in numeric_cols:
            # Apply Inversion: 65535 - x
            df[col] = 65535 - df[col]
            inverted_count += 1

        if inverted_count > 0:
            df.to_csv(file_path, index=False)
            new_mean = df[numeric_cols].values.mean()
            print(f"    -> Success. New Mean: {new_mean:.0f}")
        else:
            print(f"  Skipped {file_path.name} (No columns processed)")

    except Exception as e:
        print(f"  Error processing {file_path.name}: {e}")

def main():
    print("Starting Legacy Data Inversion (65535 - x)...")
    
    # 1. Backup
    if not BACKUP_ROOT.exists():
        print(f"Creating backup at {BACKUP_ROOT}")
        try:
            shutil.copytree(DATA_ROOT, BACKUP_ROOT)
        except Exception as e:
            print(f"Backup failed: {e}")
            return
    else:
        print(f"Backup already exists at {BACKUP_ROOT}, skipping backup creation.")

    # 2. Process
    for d in DIRS_TO_PROCESS:
        if not d.exists():
            print(f"Directory not found: {d}")
            continue
            
        print(f"\nProcessing directory: {d.name}")
        files = list(d.glob("*.csv"))
        if not files:
             print("  No .csv files found.")
             
        for f in files:
            process_file(f)

    print("\nInversion Complete.")

if __name__ == "__main__":
    main()
