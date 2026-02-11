# CCD Monitor & Analysis Pipeline

A Python-based GUI application for controlling the TCD1304 CCD spectrometer (via STM32) and a complete Machine Learning pipeline for spectral analysis.

## 1. Installation

Ensure you have Python 3.10+ installed.

```bash
# Install dependencies
uv sync
# Or manually:
pip install dearpygui pyserial numpy scipy pandas scikit-learn matplotlib
```

## 2. Running the Monitor (`main.py`)

Launch the GUI to view spectral data, record samples, and manage projects.

```bash
python main.py
```

### Features:
-   **Live View**: Real-time spectral plot (inverted logic: dark=high, light=low).
-   **Controls**: Adjustable integration time (Short/Mid/Long ranges), averaging, and dummy pixel removal.
-   **Recording**: Save spectra to `.npz` files in project folders.
-   **Projects**: Organize recordings into named projects (select via dropdown).
-   **Analysis**: Real-time peak detection and flicker analysis.

## 3. Data Processing Pipeline

We perform Machine Learning on the recorded spectra to predict concentrations.

### Step A: Preprocessing (`ccd_preprocess.py`)
Converts raw recordings into a clean, feature-rich CSV dataset.

```bash
python ccd_preprocess.py
```

-   **Input**: `projects/` folder (recordings + backgrounds).
-   **Processing**:
    -   Inverts signal (`65535 - raw`).
    -   Subtracts background (matched by integration time).
    -   Smoothes noise (Moving Average + Savitzky-Golay).
    -   Extracts 40+ features (Peaks, Area, Centroid, Band Energies).
-   **Output**: `ccd_features.csv`.

### Step B: ML Training (`rand_forest.py`)
Trains Random Forest models to predict concentration from spectral features.

```bash
python rand_forest.py
```

-   **Auto-Ranging Logic**: Trains 3 separate models for High, Mid, and Low concentrations based on integration time.
-   **Evaluation**: Outputs R² score and feature importance.
-   **Visualization**: Generates `auto_range_performance.png` showing predicted vs. actual values.

## 4. STM32 Implementation Guide

The preprocessing steps are designed to be portable to the STM32H743 microcontroller:

1.  **Buffer**: `uint16_t buffer[3694]`
2.  **Inversion**: `buffer[i] = 65535 - adc_val`
3.  **Filering**: 5-point moving average + 11-point FIR filter (Savgol coefficients).
4.  **Feature Extraction**: Simple loops to find peaks, sum area, and calculate centroid.
5.  **Inference**:
    -   Check `integration_time`.
    -   Select appropriate model (High/Mid/Low).
    -   Apply model coefficients (linear or tree-based) to the extracted features.
