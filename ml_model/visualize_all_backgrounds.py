import pandas as pd
import matplotlib.pyplot as plt
import os
from pathlib import Path
import math

# Set paths relative to this script
SCRIPT_DIR = Path(__file__).parent
DATA_DIR = SCRIPT_DIR / "data/background_data"

def plot_file(filepath, ax, color_idx, total_colors):
    if not filepath.exists():
        print(f"File not found: {filepath}")
        return

    try:
        df = pd.read_csv(filepath)
        # Assuming first column is frame_number and rest are data
        data = df.iloc[:, 1:] 
        
        # Plot mean
        mean_data = data.mean(axis=0)
        
        # Create a color from the colormap
        cmap = plt.get_cmap('viridis')
        color = cmap(color_idx / total_colors)
        
        ax.plot(mean_data.values, color=color, label=filepath.name, linewidth=1.5)
        
        print(f"Processed {filepath.name}: {len(df)} frames")
        
    except Exception as e:
        print(f"Error processing {filepath}: {e}")

def main():
    try:
        # Get all CSV files
        csv_files = sorted(list(DATA_DIR.glob("*.csv")))
        
        if not csv_files:
            print("No CSV files found in background folder.")
            return

        fig, ax = plt.subplots(figsize=(14, 8))
        
        for i, filepath in enumerate(csv_files):
            plot_file(filepath, ax, i, len(csv_files))
        
        ax.set_title("All Background Spectra (Mean)")
        ax.set_xlabel("Pixel Index")
        ax.set_ylabel("Intensity")
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        
        output_path = SCRIPT_DIR / "all_backgrounds_visualization.png"
        plt.savefig(output_path, dpi=100)
        print(f"Plot saved to {output_path}")
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
