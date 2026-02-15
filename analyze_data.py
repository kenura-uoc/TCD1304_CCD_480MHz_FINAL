import numpy as np
import os

def analyze(path_405, path_450):
    print(f"Analyzing {path_405} and {path_450}...")
    
    data_405 = np.load(path_405)
    pixels_405 = data_405['pixels'] # multiple frames likely
    
    data_450 = np.load(path_450)
    pixels_450 = data_450['pixels']
    
    # 1. Average frames
    avg_405 = np.mean(pixels_405, axis=0)
    avg_450 = np.mean(pixels_450, axis=0)
    
    # 2. Basic stats
    print(f"--- 405nm Stats ---")
    print(f"Shape: {pixels_405.shape}")
    print(f"Min  : {avg_405.min():.1f}")
    print(f"Max  : {avg_405.max():.1f}")
    print(f"Mean : {avg_405.mean():.1f}")
    
    print(f"--- 450nm Stats ---")
    print(f"Shape: {pixels_450.shape}")
    print(f"Min  : {avg_450.min():.1f}")
    print(f"Max  : {avg_450.max():.1f}")
    print(f"Mean : {avg_450.mean():.1f}")
    
    # 3. Check for valid signal
    # Threshold in C is 500.
    if avg_405.max() < 500:
        print("ALERT: 405nm signal too low (<500)")
    if avg_450.max() < 500:
        print("ALERT: 450nm signal too low (<500)")
    
    # 3. Check for inversion
    # In my new firmware, inversion is in DMA, so pixels should be Light=High.
    # Dark level for TCD1304 is usually around 500-1000 after inversion (65k - 64k).
    # If pixels are mostly near 65k, it's NOT inverted.
    # If pixels are mostly near 0, it IS inverted.
    # Wait, TCD1304: Bright = Low Voltage (~1V) -> ADC value ~14k/65k.
    #                Dark = High Voltage (~4V) -> ADC value ~56k/65k.
    # After inversion (65k - pixel):
    #                Bright = 65k - 14k = 51k.
    #                Dark = 65k - 56k = 9k.
    
    # Let's check the mean. If it's very high (>50k) across the whole spectrum, maybe it's NOT inverted or saturating dark?
    # No, spectrometer typically has peaks.
    
    # Export for plotting or review (first frame as sample)
    np.savetxt('avg_405.txt', avg_405)
    np.savetxt('avg_450.txt', avg_450)

if __name__ == "__main__":
    p405 = 'c:/Users/Kenura/Documents/TCD1304_CCD_480MHz_FINAL/ccd_monitor/projects/Default/rec_20260215_235828_405nm.npz'
    p450 = 'c:/Users/Kenura/Documents/TCD1304_CCD_480MHz_FINAL/ccd_monitor/projects/Default/rec_20260215_235852_450nm.npz'
    analyze(p405, p450)
