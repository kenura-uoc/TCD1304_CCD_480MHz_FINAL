# ML Model Pipeline — Chlorophyll Concentration from CCD Spectra

## Overview

This system predicts **Chl-a** and **Chl-b** concentrations from raw TCD1304 CCD spectra using two separate ML models, each paired with a specific laser:

| Analyte | Laser | Algorithm | Key Params |
|---------|-------|-----------|------------|
| **Chl-a** | Laser 1 (405 nm) | PLS Regression | 9 components, scale=True |
| **Chl-b** | Laser 2 (450 nm) | PCA + SVR | PCA(98% var) → SVR(rbf, C=10, ε=0.01) |

## Data Analysis Pipeline

Each raw spectrum (3694 pixels from the CCD) goes through the following steps:

### 1. Frame Averaging
Multiple CCD frames are averaged pixel-by-pixel to reduce noise.

### 2. Background Subtraction
A dark/blank spectrum captured at the same integration time is subtracted:
```
spectrum = sample_spectrum - background_spectrum
```
Background files are in `background_data/` named by integration time (e.g., `background-300.csv`).

### 3. Integration Time Normalization
The spectrum is divided by the integration time (in ms) to normalize for exposure:
```
spectrum = spectrum / integration_time_ms
```
This makes the model **independent of integration time** — it learns intensity-per-millisecond vs. concentration. Any integration time can be used at inference.

### 4. Savitzky-Golay Smoothing
A smoothing filter removes high-frequency noise:
- Window length: 11 pixels
- Polynomial order: 2

### 5. First Derivative (Savitzky-Golay)
Computes the first derivative to remove baseline offsets and enhance spectral features:
- Window length: 11 pixels
- Polynomial order: 3

### 6. SNV Normalization (Standard Normal Variate)
Scales the derivative spectrum to zero mean and unit variance:
```
snv = (spectrum - mean) / std
```
This removes multiplicative scattering effects.

### 7. ROI Crop
Only pixels **1300–3200** (1900 features) are kept. This region contains the useful spectral information and discards noisy edge pixels.

### 8. ML Prediction
- **Chl-a (PLS):** StandardScaler → dot product with 9-component PLS coefficients + intercept
- **Chl-b (PCA+SVR):** StandardScaler → PCA projection → SVR with RBF kernel

## Training Data

| Model | Samples | Integration Times (ms) | Concentration Range |
|-------|---------|----------------------|-------------------|
| Chl-a | 83 | 240, 250, 300, 500, 580, 700, 1000 | 0.33 – 18.19 mg/L |
| Chl-b | 79 | 1000, 1200, 1500, 3000, 5000 | 0.33 – 10.49 mg/L |

## Files

| File | Purpose |
|------|---------|
| `chla2_pls_model.py` | Trains Chl-a PLS model |
| `chlb2_pls.py` | Trains Chl-b PCA+SVR model |
| `export_model.py` | Trains both models & exports parameters to C header for STM32 |
| `data/real_data/chla_data.csv` | Chl-a sample metadata (concentrations, integration times) |
| `data/real_data/chlb_data.csv` | Chl-b sample metadata |
| `data/chl_a/` | Raw CCD spectra for Chl-a samples |
| `data/chl_b/` | Raw CCD spectra for Chl-b samples |
| `data/background_data/` | Background spectra at various integration times |

## STM32 Deployment

Run `export_model.py` to generate `Core/Inc/chl_model_data.h` containing all model parameters as C arrays. The firmware preprocesses live CCD data through the same pipeline and runs inference on-device.
