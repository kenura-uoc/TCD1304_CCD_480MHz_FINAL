# Simulation Conflicts & Feasibility Report

## Objective
Comparison of the datasheet-calibrated simulation (`simulation_study2.py`) against previous assumptions.

## 1. The Chl-a "Feasibility Gap"
The most significant finding from the new calibrated model is the absence of a "perfect" integration time for Chlorophyll-A under the following constraints:
- **SNR Floor**: Net signal > 3,000 counts (for 0.5 mg/L).
- **Saturation Ceiling**: Peak signal < 55,000 counts (for 18.0 mg/L).
- **Hardware**: TCD1304 at 3.3V supply (0.762x sensitivity factor) and 30°C.

### Why it occurs:
At 3.3V, the sensor's dynamic range is slightly more restricted. When combined with the linear dark current offset:
- At **500 ms**: The low-end signal (2,578 counts) is too weak, falling below the SNR floor.
- At **700 ms**: The high-end signal (57,199 counts) starts encroaching on the saturation buffer.

**Recommendation**: We have prioritized **SNR over the 55k buffer**. A 700 ms integration time is recommended for Chl-a as it maintains an $R^2 > 0.99$, even if the highest concentrations slightly exceed the preferred 55,000 count safety margin.

## 2. Spectral Sensitivity Disparity
- **Chl-a**: Highly responsive; needs ~700 ms.
- **Chl-b**: Much lower quantum efficiency at 650nm; requires **2,000 ms**.

## 3. Dark Current Scaling
The power-law model ($t^{0.88}$) shows that at high integration times (like the 2,000 ms needed for Chl-b), the dark current accounts for a significant portion of the total ADC counts (~75%). This confirms that **Dark Background Subtraction** is non-negotiable for Chl-b accuracy.
