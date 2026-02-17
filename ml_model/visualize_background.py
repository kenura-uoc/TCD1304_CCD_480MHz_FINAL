import pandas as pd
import matplotlib.pyplot as plt
import os
from pathlib import Path

# Set paths relative to this script
SCRIPT_DIR = Path(__file__).parent
DATA_DIR = SCRIPT_DIR / "data/background_data"
FILE_1 = DATA_DIR / "background-1000.csv"
FILE_2 = DATA_DIR / "background-1000 (1).csv"

def plot_file(filepath, label_prefix, ax, color):
    if not filepath.exists():
        print(f"File not found: {filepath}")
        return

    try:
        df = pd.read_csv(filepath)
        # Assuming first column is frame_number and rest are data
        # We need to drop the non-numeric frame_number column to plot
        data = df.iloc[:, 1:] 
        
        # Plot a subset of traces with transparency to show distribution
        # Sampling every 10th row to strip down rendering time if many rows
        subset = data.iloc[::10, :]
        for index, row in subset.iterrows():
            ax.plot(row.values, color=color, alpha=0.02)
        
        # Plot mean
        mean_data = data.mean(axis=0)
        ax.plot(mean_data.values, color=color, label=f'{label_prefix} Mean', linewidth=1.5)
        
        print(f"Processed {filepath}: {len(df)} frames")
        
    except Exception as e:
        print(f"Error processing {filepath}: {e}")

def main():
    try:
        fig, ax = plt.subplots(figsize=(12, 6))
        
        plot_file(FILE_1, "Background 1000", ax, 'blue')
        plot_file(FILE_2, "Background 1000 (1)", ax, 'red')
        
        ax.set_title("Background Spectra Visualization")
        ax.set_xlabel("Pixel Index")
        ax.set_ylabel("Intensity")
        ax.legend()
        ax.grid(True, alpha=0.3)
        plt.tight_layout()
        
        output_path = SCRIPT_DIR / "background_visualization.png"
        plt.savefig(output_path, dpi=100)
        print(f"Plot saved to {output_path}")
        
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
