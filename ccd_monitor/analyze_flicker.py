import numpy as np
import sys
import os

def analyze_file(filepath):
    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        return

    try:
        data = np.load(filepath)
        pixels = data['pixels']
        print(f"Loaded {filepath}")
        print(f"Shape: {pixels.shape}")
        
        # Calculate mean intensity per frame
        frame_means = np.mean(pixels, axis=1)
        
        print("\n--- Frame Mean Intensity Statistics ---")
        print(f"Mean of means: {np.mean(frame_means):.2f}")
        print(f"Std Dev: {np.std(frame_means):.2f}")
        print(f"Min: {np.min(frame_means):.2f}")
        print(f"Max: {np.max(frame_means):.2f}")
        print(f"Spread: {np.max(frame_means) - np.min(frame_means):.2f}")
        
        print("\n--- First 50 Frames (Mean Intensity) ---")
        for i, val in enumerate(frame_means[:50]):
            bar = "#" * int((val - np.min(frame_means)) / (np.max(frame_means) - np.min(frame_means) + 1) * 20)
            print(f"Frame {i:03d}: {val:.2f} {bar}")
            
        # Check for periodicity
        diffs = np.diff(frame_means)
        print("\n--- Frame-to-Frame Diffs (First 20) ---")
        print(diffs[:20])

    except Exception as e:
        print(f"Error analyzing file: {e}")

if __name__ == "__main__":
    analyze_file(r"c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL\ccd_monitor\projects\Default\rec_20260203_103451.npz")
