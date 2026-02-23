# Data Collection Protocol for Maximum Precision

To achieve 0.001 - 0.01 mg/L precision and >99.999% accuracy, follow these guidelines for your next data collection campaign.

## 1. Hardware Stability
-   **Warm-up**: Let the laser and sensor run for 5-10 minutes before starting measurements to stabilize thermal drift.
-   **Temperature Control**: If possible, keep the ambient temperature constant. Spectral shifts can occur due to temperature changes in the laser diode.

## 2. Integration Time Strategy
-   **Dynamic Range**: Aim for a peak intensity of **48,000 - 56,000 counts** (on a 16-bit sensor). This maximizes Signal-to-Noise Ratio (SNR) while staying in the linear region of the CCD.
-   **Avoiding Saturation**: Never exceed 60,000 counts. Non-linearity increases rapidly near saturation.
-   **Normalization**: The current pipeline divides by integration time ($I_{norm} = I / T_{int}$). Ensure your integration times are recorded precisely (in ms).

## 3. Noise Reduction (The "Five Nines" Requirement)
-   **Frame Averaging**: To reach 99.999% accuracy, SNR must be extremely high. 
    -   **Recommendation**: Use at least **64 or 128 frames** per sample.
-   **Dark Correction**: Capture a "Dark" spectrum (laser OFF) at **every** integration time used. Dark current is temperature-dependent and non-linear with time.

## 4. Dataset Composition
-   **Quantity**: Aim for **150 - 200 unique samples**. PLS models start to stabilize around 100 samples.
-   **Spacing**: Do not just dilute by half each time. Use a mix of:
    -   **Linear spacing** (e.g., 1, 2, 3, 4, 5 mg/L)
    -   **Log spacing** for low concentrations (e.g., 0.01, 0.05, 0.1, 0.2 mg/L)
-   **Validation Set**: Keep aside 10-20% of samples as a "Hidden Test Set" that the model never sees during training or cross-validation.

## 5. Spectral Shape & ROI
-   **ROI**: Focus on the **1300 - 3200 pixel range**. This covers the primary fluorescence peak.
-   **Alignment**: The pipeline already aligns peaks, but ensure the laser spot is physically stable on the cuvette.

## Summary Checklist
| Parameter | Value / Recommendation |
|-----------|-------------------------|
| Samples   | 150 - 200               |
| Frames Avg| 64x - 128x              |
| Peak Count| 48,000 - 56,000         |
| Dark Curr | Required for each $T_{int}$|
| Precision | 0.01 mg/L units         |
