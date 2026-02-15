import numpy as np
import scipy.signal
import math
import os

# Load model data from header (I'll extract some key values manually for verification)
# ROI: 1300 to 3200 (1900 features)
ROI_START = 1300
ROI_END = 3200
FULL_SPECTRUM_LEN = 3694

# SG Coefficients from header
SG_SMOOTH = np.array([-8.3916083916e-02, 2.0979020979e-02, 1.0256410256e-01, 1.6083916084e-01,
                      1.9580419580e-01,  2.0745920746e-01, 1.9580419580e-01, 1.6083916084e-01,
                      1.0256410256e-01,  2.0979020979e-02, -8.3916083916e-02])

SG_DERIV = np.array([1.1655011655e-01,  9.3240093240e-02,  6.9930069930e-02,
                     4.6620046620e-02,  2.3310023310e-02,  0.0,
                    -2.3310023310e-02, -4.6620046620e-02, -6.9930069930e-02,
                    -9.3240093240e-02, -1.1655011655e-01])

def apply_sg_c_style(input_data, coeffs):
    half = len(coeffs) // 2
    output = np.copy(input_data)
    for i in range(half, len(input_data) - half):
        window = input_data[i-half : i+half+1]
        output[i] = np.sum(window * coeffs)
    return output

def preprocess(avg_spectrum, integration_ms):
    # Integration normalization
    buf_a = avg_spectrum / integration_ms
    
    # SG Smooth (Full Spectrum)
    buf_b = apply_sg_c_style(buf_a, SG_SMOOTH)
    
    # SG Deriv (Full Spectrum)
    buf_a = apply_sg_c_style(buf_b, SG_DERIV)
    
    # SNV (Full Spectrum)
    mean = np.mean(buf_a)
    std = np.std(buf_a)
    buf_a = (buf_a - mean) / (std if std > 1e-8 else 1.0)
    
    # ROI Crop
    roi = buf_a[ROI_START:ROI_END]
    
    return roi

def load_pls_params():
    # I'll extract these from the file using a regex or similar because they are huge
    # For now, let's just get the intercept
    intercept = 4.2750242620
    # I'll read the coef, mean, std from the file directly in the script
    return intercept

def load_background_csv(path):
    import pandas as pd
    df = pd.read_csv(path)
    df = df.iloc[:, 1:]  # Drop frame_number
    data = df.values.astype(float)
    return np.mean(data, axis=0)

def run_prediction(path, integration_ms, bg_data=None, invert_first=False):
    data = np.load(path)
    avg = np.mean(data['pixels'], axis=0)
    
    spectrum_to_process = avg
    if bg_data is not None:
        if invert_first:
            # Device data is '65535 - raw'. Convert back to raw.
            raw_spectrum = 65535 - avg
            # Match training logic: spectrum - bg
            spectrum_to_process = raw_spectrum - bg_data
        else:
            # Maybe the CSVs were already inverted?
            spectrum_to_process = avg - bg_data
    
    # Preprocess
    roi_features = preprocess(spectrum_to_process, integration_ms)
    
    # Load PLS weights from file
    # print(f"Loading PLS weights from chl_model_data.h...")
    with open('Core/Inc/chl_model_data.h', 'r') as f:
        content = f.read()
    
    def extract_array(name, size):
        start_marker = f"static const float {name}[{size}] = {{"
        start_idx = content.find(start_marker) + len(start_marker)
        if start_idx < len(start_marker): return np.zeros(size)
        end_idx = content.find("};", start_idx)
        raw_vals = content[start_idx:end_idx].replace('f', '').replace('\n', '').split(',')
        return np.array([float(x.strip()) for x in raw_vals if x.strip()])

    pls_coef = extract_array("PLS_COEF", 1900)
    pls_x_mean = extract_array("PLS_X_MEAN", 1900)
    pls_x_std = extract_array("PLS_X_STD", 1900)
    pls_intercept = 4.2750242620
    
    # PLS Prediction
    # Note: PLS_X_MEAN and STD are from the training pipeline.
    scaled = (roi_features - pls_x_mean) / np.maximum(pls_x_std, 1e-8)
    prediction = pls_intercept + np.sum(scaled * pls_coef)
    
    return prediction

if __name__ == "__main__":
    p405 = 'c:/Users/Kenura/Documents/TCD1304_CCD_480MHz_FINAL/ccd_monitor/projects/Default/rec_20260215_235828_405nm.npz'
    bg_path = 'c:/Users/Kenura/Documents/TCD1304_CCD_480MHz_FINAL/background_data/background-1000.csv'
    
    print("Loading background data...")
    bg_data = load_background_csv(bg_path)
    
    print("\n--- RESULTS ---")
    
    # Test 1: Current device logic (Inverted data, No BG subtraction)
    res1 = run_prediction(p405, 1000.0, bg_data=None, invert_first=False)
    print(f"1. Current Device (Inverted, No BG): {res1:.4f}")
    
    # Test 2: Inverted data - Inverted BG
    bg_inverted = 65535 - bg_data
    res2 = run_prediction(p405, 1000.0, bg_data=bg_inverted, invert_first=False)
    print(f"2. Inverted - Inverted BG:            {res2:.4f}")
    
    # Test 3: Raw - Raw BG (Training Polarity)
    res3 = run_prediction(p405, 1000.0, bg_data=bg_data, invert_first=True)
    print(f"3. Raw - Raw BG (Training Polarity):  {res3:.4f}")
    
    # Test 4: Peak height (BG_raw - Raw_signal)
    data = np.load(p405)
    avg = np.mean(data['pixels'], axis=0)
    raw_avg = 65535 - avg
    peak_diff = bg_data - raw_avg # High values = high light
    
    def run_direct(spectrum, integration_ms):
        roi = preprocess(spectrum, integration_ms)
        with open('Core/Inc/chl_model_data.h', 'r') as f: content = f.read()
        def extract_array(name, size):
            start_marker = f"static const float {name}[{size}] = {{"
            start_idx = content.find(start_marker) + len(start_marker)
            end_idx = content.find("};", start_idx)
            raw_vals = content[start_idx:end_idx].replace('f', '').replace('\n', '').split(',')
            return np.array([float(x.strip()) for x in raw_vals if x.strip()])
        pls_coef = extract_array("PLS_COEF", 1900)
        pls_x_mean = extract_array("PLS_X_MEAN", 1900)
        pls_x_std = extract_array("PLS_X_STD", 1900)
        scaled = (roi - pls_x_mean) / np.maximum(pls_x_std, 1e-8)
        return 4.2750242620 + np.sum(scaled * pls_coef)
        
    res4 = run_direct(peak_diff, 1000.0)
    print(f"4. Peak Height (BG_raw - Raw):        {res4:.4f}")
    
    # Test 5: Raw Signal, No BG
    res5 = run_direct(raw_avg, 1000.0)
    print(f"5. Raw Signal (No BG):                {res5:.4f}")
    
    # Test 6: Firmware Simulation
    # Firmware DMA callback inverts both signal and BG: 65535 - raw
    # Then predictor does: (inverted_bg - inverted_signal) / int_time
    # This should algebraically equal (Raw_Signal - Raw_BG) / int_time
    inverted_signal = avg  # .npz already has 65535-raw from DMA callback
    inverted_bg = 65535 - bg_data  # bg_data is raw, firmware would see 65535-raw
    firmware_spectrum = inverted_bg - inverted_signal  # predictor formula
    res6 = run_direct(firmware_spectrum, 1000.0)
    print(f"6. Firmware Sim (InvBG-InvSig):       {res6:.4f}")
    
    print(f"\n--- VALIDATION ---")
    print(f"Test 3 == Test 6? {abs(res3 - res6) < 0.001}")
    print(f"Difference: {abs(res3 - res6):.6f}")
