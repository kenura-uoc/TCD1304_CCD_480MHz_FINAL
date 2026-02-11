"""
CCD Spectral Data Preprocessing & Feature Extraction Pipeline
==============================================================
Loads TCD1304 CCD recordings from .npz files, performs:
  1. Signal inversion (dark=high → light=high)
  2. Dummy pixel removal (keep pixels 32–3679)
  3. Background subtraction (matched by integration time)
  4. Moving average noise cancellation (STM32-portable)
  5. Savitzky-Golay smoothing
  6. Feature extraction (peaks, FWHM, centroid, band energies, etc.)

Produces a single ML-ready DataFrame with known chlorophyll concentrations
as the target variable. All DSP methods are STM32H743-portable.

Usage:
    python ccd_preprocess.py
"""

import os
import numpy as np
import pandas as pd
import pandas as pd
from scipy.signal import savgol_filter
from datetime import datetime

# ============================================================
# CONSTANTS
# ============================================================
CCD_PIXELS = 3694
DUMMY_START = 32       # First active pixel
DUMMY_END = 3680       # Last active pixel (exclusive)
ACTIVE_PIXELS = DUMMY_END - DUMMY_START  # 3648

PROJECTS_DIR = os.path.join(os.path.dirname(__file__), "projects")

# Moving average window (must be odd, small for STM32)
MA_WINDOW = 5

# Savitzky-Golay parameters
SAVGOL_WINDOW = 11
SAVGOL_ORDER = 3

# Peak detection
PEAK_THRESHOLD_FRAC = 0.10  # 10% of max intensity
PEAK_MIN_DISTANCE = 50      # Minimum distance between peaks (pixels)
MAX_PEAKS = 5               # Track top-N peaks

# Band energy: split active region into N equal bands
N_BANDS = 4

# ============================================================
# EXPERIMENTAL METADATA
# ============================================================
# Each entry: (sample_concentration_mg_L, integration_time_ms)
# Concentrations from serial dilution series.
# Integration times forward-filled from the experiment log.

BLUE_211_METADATA = [
    # No.  Sample_Con  Integration_time_ms
    (17.476, 1600),   # 1
    (15.728, 1600),   # 2
    (14.298, 1600),   # 3
    (13.107, 1600),   # 4
    (12.099, 1600),   # 5
    (11.234, 1600),   # 6
    (10.485, 1000),   # 7
    (9.830,  1000),   # 8
    (9.252,  1000),   # 9
    (8.738,  1000),   # 10
    (8.278,  1000),   # 11
    (7.864,  1000),   # 12
    (7.471,  1000),   # 13
    (7.097,  1000),   # 14
    (6.742,  1000),   # 15
    (6.405,  1000),   # 16
    (6.085,  1000),   # 17
    (5.781,  1000),   # 18
    (5.492,  1000),   # 19
    (5.217,  1000),   # 20
    (4.956,  1000),   # 21
    (4.709,  1000),   # 22
    (4.473,  1000),   # 23
    (4.249,  1000),   # 24
    (4.037,  1000),   # 25
    (3.835,  1000),   # 26
    (3.643,  1200),   # 27
    (3.461,  1200),   # 28
    (3.288,  1200),   # 29
    (3.124,  1200),   # 30
    (2.968,  1200),   # 31
    (2.819,  1500),   # 32
    (2.678,  3000),   # 33
    (2.544,  3000),   # 34
    (2.417,  3000),   # 35
    (2.296,  3000),   # 36
    (2.181,  3000),   # 37
    (2.072,  3000),   # 38
    (1.969,  3000),   # 39
    (1.870,  3000),   # 40
    (1.777,  3000),   # 41
    (1.688,  3000),   # 42
    (1.604,  3000),   # 43
    (1.523,  3000),   # 44
    (1.447,  3000),   # 45
    (1.375,  5000),   # 46
    (1.306,  5000),   # 47
]

VIOLET_211_METADATA = [
    (18.188, 300),    # 1
    (16.369, 300),    # 2
    (14.881, 300),    # 3
    (13.641, 300),    # 4
    (12.592, 300),    # 5
    (11.692, 300),    # 6
    (10.913, 300),    # 7
    (10.231, 300),    # 8
    (9.629,  240),    # 9
    (9.094,  240),    # 10
    (8.615,  240),    # 11
    (8.185,  240),    # 12
    (7.775,  240),    # 13
    (7.387,  240),    # 14
    (7.017,  240),    # 15
    (6.666,  240),    # 16
    (6.333,  240),    # 17
    (6.016,  240),    # 18
    (5.716,  300),    # 19
    (5.430,  300),    # 20
    (5.158,  300),    # 21
    (4.900,  300),    # 22
    (4.655,  300),    # 23
    (4.423,  250),    # 24
    (4.201,  250),    # 25
    (3.991,  250),    # 26
    (3.792,  250),    # 27
    (3.602,  300),    # 28
    (3.422,  300),    # 29
    (3.251,  300),    # 30
    (3.088,  300),    # 31
    (2.934,  300),    # 32
    (2.787,  300),    # 33
    (2.648,  500),    # 34
    (2.516,  500),    # 35
    (2.390,  500),    # 36
    (2.270,  500),    # 37
    (2.157,  500),    # 38
    (2.049,  500),    # 39
    (1.946,  500),    # 40
    (1.849,  500),    # 41
    (1.757,  1000),   # 42
    (1.669,  1000),   # 43
    (1.585,  1000),   # 44
    (1.506,  1000),   # 45
]

PROJECT_METADATA = {
    "blue-211": {
        "metadata": BLUE_211_METADATA,
        "excitation_nm": 450,       # Blue LED
        "target_pigment": "chl_a",  # Chlorophyll-a fluorescence
        "emission_nm": "650-670",
    },
    "violet-211": {
        "metadata": VIOLET_211_METADATA,
        "excitation_nm": 405,       # Violet LED
        "target_pigment": "chl_a",  # Chlorophyll-a fluorescence
        "emission_nm": "670",
    },
}


# ============================================================
# DSP FUNCTIONS (all STM32H743-portable)
# ============================================================

def invert_signal(raw: np.ndarray) -> np.ndarray:
    """TCD1304 outputs dark=high, light=low. Invert to light=high."""
    return 65535 - raw


def remove_dummy_pixels(spectrum: np.ndarray) -> np.ndarray:
    """Remove non-imaging dummy pixels from TCD1304 output."""
    return spectrum[DUMMY_START:DUMMY_END]


def moving_average(signal: np.ndarray, window: int = MA_WINDOW) -> np.ndarray:
    """
    Simple boxcar moving average filter.
    STM32 implementation: circular buffer with running sum.
    O(N) time, O(window) memory.
    """
    if len(signal) < window:
        return signal.copy()
    kernel = np.ones(window) / window
    # Use 'same' mode to preserve array length; edges handled by zero-padding
    smoothed = np.convolve(signal, kernel, mode='same')
    return smoothed


def savgol_smooth(signal: np.ndarray,
                  window: int = SAVGOL_WINDOW,
                  order: int = SAVGOL_ORDER) -> np.ndarray:
    """
    Savitzky-Golay filter for spectral smoothing.
    STM32 note: Pre-compute FIR coefficients offline, then it's just
    a fixed dot product per output sample — very efficient on Cortex-M7.
    """
    if len(signal) <= window:
        return signal.copy()
    return savgol_filter(signal, window, order)


def find_peaks_simple(signal: np.ndarray,
                      threshold_frac: float = PEAK_THRESHOLD_FRAC,
                      min_distance: int = PEAK_MIN_DISTANCE,
                      max_peaks: int = MAX_PEAKS):
    """
    Simple peak detection: local maxima above threshold.
    STM32-portable: single-pass scan with greedy distance filter.

    Returns:
        peaks_idx: array of peak pixel indices
        peaks_val: array of peak intensity values
    """
    if len(signal) < 3:
        return np.array([]), np.array([])

    threshold = np.max(signal) * threshold_frac

    # Find local maxima: signal[i] > signal[i-1] AND signal[i] > signal[i+1]
    is_max = np.zeros(len(signal), dtype=bool)
    is_max[1:-1] = (signal[1:-1] > signal[:-2]) & (signal[1:-1] > signal[2:])

    # Apply threshold
    candidates = np.where(is_max & (signal > threshold))[0]

    if len(candidates) == 0:
        return np.array([]), np.array([])

    # Greedy distance filter (pick highest peaks first)
    # Sort candidates by intensity (descending)
    sorted_idx = candidates[np.argsort(signal[candidates])[::-1]]

    peaks_idx = []
    for idx in sorted_idx:
        if len(peaks_idx) >= max_peaks:
            break
        # Check distance to all already-selected peaks
        if all(abs(idx - p) >= min_distance for p in peaks_idx):
            peaks_idx.append(idx)

    peaks_idx = np.sort(peaks_idx)  # Return in position order
    peaks_val = signal[peaks_idx]
    return peaks_idx, peaks_val


def compute_fwhm(signal: np.ndarray, peak_idx: int) -> float:
    """
    Compute Full Width at Half Maximum for a peak.
    STM32-portable: simple left/right scan from peak.
    """
    peak_val = signal[peak_idx]
    half_max = peak_val / 2.0

    # Scan left
    left = peak_idx
    while left > 0 and signal[left] > half_max:
        left -= 1

    # Scan right
    right = peak_idx
    while right < len(signal) - 1 and signal[right] > half_max:
        right += 1

    return float(right - left)


def compute_centroid(signal: np.ndarray) -> float:
    """
    Intensity-weighted center of mass.
    STM32: weighted_sum / total_sum — two running accumulators.
    """
    total = np.sum(signal)
    if total == 0:
        return len(signal) / 2.0
    indices = np.arange(len(signal), dtype=np.float64)
    return float(np.sum(indices * signal) / total)


def compute_band_energies(signal: np.ndarray, n_bands: int = N_BANDS) -> np.ndarray:
    """
    Sum intensity in N equal spectral bands.
    STM32: partial sums with fixed boundaries — O(N) single pass.
    """
    band_size = len(signal) // n_bands
    energies = np.zeros(n_bands)
    for i in range(n_bands):
        start = i * band_size
        end = start + band_size if i < n_bands - 1 else len(signal)
        energies[i] = np.sum(signal[start:end].astype(np.float64))
    return energies


def compute_snr(signal: np.ndarray, noise_region_size: int = 100) -> float:
    """
    Signal-to-noise ratio: peak / std_dev of baseline.
    Baseline estimated from the edges of the spectrum (least signal).
    STM32: simple max + std calculation on small array region.
    """
    if len(signal) < noise_region_size * 2:
        noise_region_size = len(signal) // 4

    # Use edges as noise reference
    noise = np.concatenate([signal[:noise_region_size],
                            signal[-noise_region_size:]])
    noise_std = np.std(noise)
    if noise_std == 0:
        return 0.0
    return float(np.max(signal) / noise_std)


def compute_moments(signal: np.ndarray):
    """
    Compute skewness and kurtosis of spectral shape.
    STM32: third and fourth moment calculations — O(N) single pass.
    """
    mean = np.mean(signal)
    std = np.std(signal)
    if std == 0:
        return 0.0, 0.0

    normalized = (signal - mean) / std
    skewness = float(np.mean(normalized ** 3))
    kurtosis = float(np.mean(normalized ** 4) - 3.0)  # Excess kurtosis
    return skewness, kurtosis


# ============================================================
# DATA LOADING
# ============================================================

def load_backgrounds(project_dir: str) -> dict:
    """
    Load all background-*.npz files from a project directory.
    Returns dict: {integration_time_ms: averaged_spectrum (active pixels only, inverted)}
    """
    backgrounds = {}
    for fname in os.listdir(project_dir):
        if fname.startswith("background-") and fname.endswith(".npz"):
            # Parse integration time from filename
            int_time_str = fname.replace("background-", "").replace(".npz", "")
            try:
                int_time = int(int_time_str)
            except ValueError:
                continue

            data = np.load(os.path.join(project_dir, fname))
            pixels = data['pixels']  # shape: (N, 3694)

            # Average all frames
            avg_spectrum = np.mean(pixels.astype(np.float64), axis=0)

            # Invert + remove dummies
            avg_spectrum = 65535.0 - avg_spectrum
            avg_spectrum = avg_spectrum[DUMMY_START:DUMMY_END]

            backgrounds[int_time] = avg_spectrum
            print(f"  Background {int_time}ms: {pixels.shape[0]} frames averaged")

    return backgrounds


def get_nearest_background(backgrounds: dict, target_time: int) -> np.ndarray:
    """Find the background with the closest integration time."""
    if target_time in backgrounds:
        return backgrounds[target_time]

    available = sorted(backgrounds.keys())
    nearest = min(available, key=lambda t: abs(t - target_time))
    print(f"  Warning: No background for {target_time}ms, using nearest: {nearest}ms")
    return backgrounds[nearest]


def load_recordings(project_dir: str) -> list:
    """
    Load all rec_*.npz files sorted by timestamp.
    Returns list of (filename, averaged_spectrum).
    """
    rec_files = sorted([f for f in os.listdir(project_dir)
                        if f.startswith("rec_") and f.endswith(".npz")])

    recordings = []
    for fname in rec_files:
        data = np.load(os.path.join(project_dir, fname))
        pixels = data['pixels']  # shape: (N, 3694)

        # Average all frames in the recording
        avg_spectrum = np.mean(pixels.astype(np.float64), axis=0)

        # Invert + remove dummies
        avg_spectrum = 65535.0 - avg_spectrum
        avg_spectrum = avg_spectrum[DUMMY_START:DUMMY_END]

        recordings.append((fname, avg_spectrum))

    return recordings


# ============================================================
# FEATURE EXTRACTION
# ============================================================

def extract_features(spectrum: np.ndarray, integration_time_ms: int) -> dict:
    """
    Extract all features from a preprocessed spectrum.
    All computations are STM32H743-portable.
    """
    features = {}

    # --- Basic statistics ---
    features['total_area'] = float(np.sum(spectrum))
    features['max_intensity'] = float(np.max(spectrum))
    features['mean_intensity'] = float(np.mean(spectrum))
    features['std_intensity'] = float(np.std(spectrum))

    # --- Normalize by integration time (counts per ms) ---
    features['area_per_ms'] = features['total_area'] / max(integration_time_ms, 1)
    features['max_per_ms'] = features['max_intensity'] / max(integration_time_ms, 1)

    # --- Centroid ---
    features['centroid'] = compute_centroid(spectrum)

    # --- SNR ---
    features['snr'] = compute_snr(spectrum)

    # --- Moments ---
    skewness, kurtosis = compute_moments(spectrum)
    features['skewness'] = skewness
    features['kurtosis'] = kurtosis

    # --- Peak detection ---
    peaks_idx, peaks_val = find_peaks_simple(spectrum)
    features['peak_count'] = len(peaks_idx)

    # Store top-N peak positions, intensities, and FWHM
    for i in range(MAX_PEAKS):
        if i < len(peaks_idx):
            features[f'peak_{i+1}_pos'] = int(peaks_idx[i])
            features[f'peak_{i+1}_intensity'] = float(peaks_val[i])
            features[f'peak_{i+1}_fwhm'] = compute_fwhm(spectrum, peaks_idx[i])
            features[f'peak_{i+1}_intensity_per_ms'] = features[f'peak_{i+1}_intensity'] / max(integration_time_ms, 1)
        else:
            features[f'peak_{i+1}_pos'] = 0
            features[f'peak_{i+1}_intensity'] = 0.0
            features[f'peak_{i+1}_fwhm'] = 0.0
            features[f'peak_{i+1}_intensity_per_ms'] = 0.0

    # --- Peak ratios (ratio between top-2 peaks) ---
    if len(peaks_val) >= 2:
        sorted_peaks = np.sort(peaks_val)[::-1]
        features['peak_ratio_1_2'] = float(sorted_peaks[0] / max(sorted_peaks[1], 1))
    else:
        features['peak_ratio_1_2'] = 0.0

    # --- Band energies ---
    band_energies = compute_band_energies(spectrum)
    for i, e in enumerate(band_energies):
        features[f'band_{i+1}_energy'] = float(e)
        features[f'band_{i+1}_energy_per_ms'] = float(e) / max(integration_time_ms, 1)

    # --- Band ratios ---
    if band_energies[0] > 0:
        for i in range(1, len(band_energies)):
            features[f'band_ratio_{i+1}_1'] = float(band_energies[i] / band_energies[0])
    else:
        for i in range(1, len(band_energies)):
            features[f'band_ratio_{i+1}_1'] = 0.0

    return features


def get_timestamp_from_filename(filename: str) -> datetime:
    """
    Extract timestamp from filename.
    Format 1: rec_YYYYMMDD_HHMMSS.npz
    Format 2: CCD_YYYYMMDD_HHMMSS.npz (or similar, checking patterns)
    """
    # Remove extension
    base = os.path.splitext(filename)[0]
    
    # Try different split patterns
    parts = base.split('_')
    
    # Search for YYYYMMDD part
    date_str = ""
    time_str = ""
    
    for i, part in enumerate(parts):
        if len(part) == 8 and part.isdigit() and part.startswith("20"):
            date_str = part
            if i + 1 < len(parts) and len(parts[i+1]) == 6 and parts[i+1].isdigit():
                time_str = parts[i+1]
                break
    
    if date_str and time_str:
        return datetime.strptime(f"{date_str}_{time_str}", "%Y%m%d_%H%M%S")
    
    # Fallback: file creation time (not ideal but better than crash)
    return datetime.now()



# ============================================================
# MAIN PIPELINE
# ============================================================

def process_project(project_name: str) -> tuple:
    """Process all recordings in a project and return (list of feature dicts, list of spectra arrays)."""
    project_dir = os.path.join(PROJECTS_DIR, project_name)

    if project_name not in PROJECT_METADATA:
        print(f"WARNING: No metadata for project '{project_name}', skipping.")
        return []

    meta = PROJECT_METADATA[project_name]
    sample_meta = meta["metadata"]

    print(f"\n{'='*60}")
    print(f"Processing project: {project_name}")
    print(f"  Excitation: {meta['excitation_nm']}nm")
    print(f"  Target: {meta['target_pigment']}")
    print(f"{'='*60}")

    # 1. Load backgrounds
    print("\nLoading backgrounds...")
    backgrounds = load_backgrounds(project_dir)

    # 2. Load recordings
    print("\nLoading recordings...")
    recordings = load_recordings(project_dir)
    print(f"  Found {len(recordings)} recordings")

    # SPECIAL CASE: User requested to remove first 6 records from blue-211
    if project_name == "blue-211":
        print("  ACTION: Removing first 6 records (incorrect data) as requested.")
        recordings = recordings[6:]
        sample_meta = sample_meta[6:]
        print(f"  Remaining recordings: {len(recordings)}")

    # Verify count matches metadata
    if len(recordings) != len(sample_meta):
        print(f"  WARNING: {len(recordings)} recordings but {len(sample_meta)} metadata entries!")
        print(f"  Will process min({len(recordings)}, {len(sample_meta)}) = {min(len(recordings), len(sample_meta))} samples")

    n_samples = min(len(recordings), len(sample_meta))
    rows = []
    spectra_list = []

    # Calculate start time (t0) for this project
    # We use the timestamp of the FIRST recording file (even if we skipped some data)
    # Wait, if we skipped data (blue-211), should we use the original first file? 
    # User said "time taken from 1st measuremtn of the stok solution".
    # If we removed 1-6, the NEW 1st recording is #7.
    # But Stock was measured at #1. 
    # So we should use the timestamp of the *original* list of recordings[0] if possible.
    # But we modified `recordings` list in-place.
    # Let's trust that the valid data is what matters. 
    # Or, relative to the *current* set's first sample.
    # If correct evaporation math: The concentration drifted from the START.
    # So if we skipped 1-6, sample 7 matches calculated concentration 7? 
    # Yes, calculated concentration assumes no evaporation.
    # So actual concentration 7 > calculated 7.
    # Time 7 > Time 0.
    # So we need t0 of the set.
    
    timestamps = [get_timestamp_from_filename(r[0]) for r in recordings]
    if timestamps:
        t0 = min(timestamps)
    else:
        t0 = datetime.now()

    for i in range(n_samples):
        fname, raw_spectrum = recordings[i]
        concentration, int_time = sample_meta[i]
        
        # Calculate elapsed minutes
        t_curr = timestamps[i]
        elapsed_min = (t_curr - t0).total_seconds() / 60.0


        # 3. Background subtraction
        bg = get_nearest_background(backgrounds, int_time)
        signal = raw_spectrum - bg
        signal = np.clip(signal, 0, None)  # No negative values

        # 4. Moving average (noise cancellation)
        signal_ma = moving_average(signal)

        # 5. Savitzky-Golay smoothing
        signal_smooth = savgol_smooth(signal_ma)

        # 6. Feature extraction
        features = extract_features(signal_smooth, int_time)

        # Add metadata columns
        features['project'] = project_name
        features['filename'] = fname
        features['sample_number'] = i + 1
        features['concentration_mg_L'] = concentration
        features['integration_time_ms'] = int_time
        features['integration_time_ms'] = int_time
        features['excitation_nm'] = meta['excitation_nm']
        features['target_pigment'] = meta['target_pigment']
        features['elapsed_minutes'] = float(elapsed_min)

        rows.append(features)
        spectra_list.append(signal_smooth)

    print(f"\n  Processed {len(rows)} samples successfully")
    return rows, spectra_list


def main():
    print("=" * 60)
    print("CCD Spectral Preprocessing & Feature Extraction Pipeline")
    print("=" * 60)

    all_rows = []
    all_spectra = []

    # Process each project
    for project_name in PROJECT_METADATA.keys():
        project_dir = os.path.join(PROJECTS_DIR, project_name)
        if not os.path.exists(project_dir):
            print(f"\nSkipping '{project_name}' — directory not found")
            continue
        rows, spectra = process_project(project_name)
        all_rows.extend(rows)
        all_spectra.extend(spectra)

    if not all_rows:
        print("\nERROR: No data processed!")
        return

    # Build DataFrame
    df = pd.DataFrame(all_rows)

    # Reorder columns: metadata first, then features
    meta_cols = ['project', 'filename', 'sample_number',
                 'concentration_mg_L', 'integration_time_ms',
                 'excitation_nm', 'target_pigment']
    feature_cols = [c for c in df.columns if c not in meta_cols]
    df = df[meta_cols + sorted(feature_cols)]

    # Save to CSV
    output_path = os.path.join(os.path.dirname(__file__), "ccd_features.csv")
    df.to_csv(output_path, index=False)

    # Save full spectral data for Chemometrics (PLS)
    spectra_path = os.path.join(os.path.dirname(__file__), "spectra_data.npz")
    np.savez_compressed(spectra_path, 
                        spectra=np.array(all_spectra), 
                        concentrations=df['concentration_mg_L'].values,
                        integration_times=df['integration_time_ms'].values)
    print(f"\nSaved full {len(all_spectra)} spectra to: {spectra_path}")

    # Summary
    print(f"\n{'='*60}")
    print("PIPELINE COMPLETE")
    print(f"{'='*60}")
    print(f"Total samples:    {len(df)}")
    print(f"Feature columns:  {len(feature_cols)}")
    print(f"Projects:         {df['project'].unique().tolist()}")
    print(f"Concentration range: {df['concentration_mg_L'].min():.3f} — {df['concentration_mg_L'].max():.3f} mg/L")
    print(f"\nSaved to: {output_path}")
    print(f"\nDataFrame shape: {df.shape}")
    print(f"\nFirst 5 rows (key columns):")
    display_cols = ['project', 'sample_number', 'concentration_mg_L',
                    'integration_time_ms', 'peak_count', 'max_intensity',
                    'centroid', 'snr']
    display_cols = [c for c in display_cols if c in df.columns]
    print(df[display_cols].head().to_string(index=False))
    print(f"\nLast 5 rows:")
    print(df[display_cols].tail().to_string(index=False))

    # Check for NaN or all-zero columns
    nan_cols = df.columns[df.isnull().any()].tolist()
    if nan_cols:
        print(f"\nWARNING: Columns with NaN: {nan_cols}")
    zero_cols = [c for c in feature_cols if (df[c] == 0).all()]
    if zero_cols:
        print(f"\nWARNING: All-zero columns: {zero_cols}")

    return df


if __name__ == "__main__":
    df = main()
