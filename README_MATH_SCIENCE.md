# Math & Science of the CCD Monitor

This document explains the signal processing capability and the mathematical models used to predict Chlorophyll concentrations (Chl-a and Chl-b).

## 1. The Signal Chain (`common_preprocess`)

Before any prediction happens, the raw CCD data goes through rigorous cleaning.

### Step 1: Inversion & Dark Subtraction
The TCD1304 CCD outputs "inverted" voltage (Bright light = Low voltage).
1.  **Invert**: `Signal = 65535 - ADC_Value`
2.  **Dark Subtraction**: We subtract the "Dark Current" (sensor noise when no light is present).
    *   `Clean_Signal = Signal - Dark_Signal`
    *   This removes fixed-pattern noise from the sensor pixels.

### Step 2: Despiking (Noise Removal)
*   **Problem**: Electrical noise or "hot pixels" can cause single-pixel spikes (as seen in your plots).
*   **Solution**: We apply a **Median Filter**.
    *   For each pixel, we look at its neighbors (e.g., Left, Center, Right).
    *   We replace the Center value with the *Median* (middle value) of the group.
    *   *Effect*: This deletes single-pixel spikes completely without blurring the edges of the real spectral peaks.

### Step 3: Savitzky-Golay Filtering
We don't use the raw intensity. We care about the **Shape** (slope and curvature).
1.  **Smoothing**: A weighted moving average removes high-frequency fuzz.
2.  **1st Derivative**: We calculate the *slope* of the spectrum.
    *   **Scientific Reason**: Absolute intensity fluctuates with lamp power or distance. The *Slope* (change in intensity) is much more stable and specific to the chemical absorption bands.

### Step 4: SNV (Standard Normal Variate)
We normalize the data to have **Mean = 0** and **Standard Deviation = 1**.
*   `x_new = (x - mean) / std`
*   This ensures that "Bright Spectrum" and "Dim Spectrum" look identical to the model, as long as they have the same *shape*.

---

## 2. Prediction Models

### Chl-a: PLS (Partial Least Squares)
**Function**: `pls_predict` / `chl_predict_chla`

This is a linear model. It assumes the concentration is the sum of weighted contributions from every pixel.

$$ Concentration = b + \sum_{i} (w_i \cdot x_i) $$

*   $x_i$: The processed value of pixel $i$.
*   $w_i$: The "Coefficient" (Weight) for pixel $i$.
*   $b$: The Intercept (Bias).

**The Dot Product**:
This sum represents a **Dot Product** between your Signal Vector and the Model's Weight Vector.
*   **Positive Result**: Your signal shape *matches* the Chlorophyll fingerprint.
*   **Negative Result**: Your signal shape is *opposite* or unrelated (see below).

### Chl-b: PCA + SVR
**Function**: `pca_svr_predict` / `chl_predict_chlb`

This is a non-linear model for more complex relationships.

1.  **PCA (Principal Component Analysis)**:
    *   Compresses the 1900 pixels down to ~30 "Features" (Principal Components).
    *   It does this using Dot Products with "Eigenvectors" (fundamental shapes found in the training data).
2.  **SVR (Support Vector Regression)**:
    *   Uses an **RBF Kernel** (Radial Basis Function).
    *   It basically measures the "Distance" between your sample and known "Support Vectors" (landmark samples from training).
    *   If you are "close" to a known sample with high concentration, you get a high score.

---

## 3. "Negative Concentration" Explained

**Q: How can concentration be negative?**
**A: It means "Not Detected" or "Noise".**

Mathematically, the PLS model is just adding up numbers.
*   If your signal has a "dip" where the model expects a "peak", the term $(w_i \cdot x_i)$ becomes negative.
*   If the total sum of these mismatches outweighs the matches, the final result is negative.

**Scientific Interpretation**:
*   **Positive**: Chlorophyll is present.
*   **Zero**: Pure water / Baseline.
*   **Negative**: The sample is *cleaner* than the baseline, or (more likely) the noise pattern happened to align with the negative weights.

**Action**: In the user interface, we should simply clamp Negative values to **0.00**.

---

## 4. Code Map regarding your request

1.  **`common_preprocess`**: Runs Steps 1-4 (Deep cleaning).
2.  **`pls_predict`**: Runs the Dot Product for Chl-a.
3.  **`pca_svr_predict`**: Runs the PCA compression + SVR curve fitting for Chl-b.
4.  **`chl_predict_chl*`**: Top-level wrappers that combine everything.
