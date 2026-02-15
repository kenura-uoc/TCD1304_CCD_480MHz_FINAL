
import os
import struct
import re
import math
import numpy as np
import argparse

# Configuration
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
HEADER_PATH = os.path.join(PROJECT_ROOT, "Core", "Inc", "chl_model_data.h")

def parse_header(header_path):
    """Parses chl_model_data.h to extract constants and coefficients."""
    constants = {}
    arrays = {}
    
    with open(header_path, 'r') as f:
        content = f.read()

    # Parse #defines
    for match in re.finditer(r'#define\s+(\w+)\s+([-\d.eE+]+)', content):
        name, val = match.groups()
        try:
            if '.' in val or 'e' in val.lower():
                constants[name] = float(val)
            else:
                constants[name] = int(val)
        except:
            pass
            
    # Parse arrays (simple parser assuming standard formatting)
    # Looks for: static const float NAME[SIZE] = { ... };
    array_pattern = r'static const float (\w+)\[\d+\] = \{([^}]+)\};'
    for match in re.finditer(array_pattern, content, re.DOTALL):
        name = match.group(1)
        data_str = match.group(2)
        # Clean up and split
        data_str = re.sub(r'//.*', '', data_str) # Remove comments
        data_str = data_str.replace('\n', ' ').replace(',', ' ')
        data_str = data_str.replace('f', '') # Remove float suffix
        values = [float(x) for x in data_str.split() if x]
        arrays[name] = np.array(values, dtype=np.float32)

    return constants, arrays

def apply_sg_filter(input_data, coeffs, window, half):
    """Replicates apply_sg_filter from C code."""
    length = len(input_data)
    output = np.zeros(length, dtype=np.float32)
    
    # Interior
    for i in range(half, length - half):
        filt_sum = 0.0
        for j in range(window):
            filt_sum += coeffs[j] * input_data[i - half + j]
        output[i] = filt_sum
        
    # Edges (copy)
    for i in range(half):
        output[i] = input_data[i]
    for i in range(length - half, length):
        output[i] = input_data[i]
        
    return output

def preprocess(raw_pixels, integration_time, consts, arrays):
    """Replicates common_preprocess from C code."""
    
    # 1. Normalize by integration time
    pixels_norm = np.array(raw_pixels, dtype=np.float32) / integration_time
    
    # 2. SG Smooth
    # C: apply_sg_filter(pred->buf_a, pred->buf_b, ...)
    # Input: pixels_norm, Output: smoothed
    smoothed = apply_sg_filter(pixels_norm, arrays['SG_SMOOTH_COEFFS'], 
                               consts['SG_SMOOTH_WINDOW'], consts['SG_SMOOTH_HALF'])
                               
    # 3. SG Derivative
    # C: apply_sg_filter(pred->buf_b, pred->buf_a, ...)
    # Input: smoothed, Output: deriv
    deriv = apply_sg_filter(smoothed, arrays['SG_DERIV_COEFFS'],
                            consts['SG_DERIV_WINDOW'], consts['SG_DERIV_HALF'])
                            
    # 4. Crop to ROI
    roi_start = consts['ROI_START']
    roi_end = consts['ROI_END']
    # C: pred->roi[i] = pred->buf_a[ROI_START + i];
    # Note: ROI_END is not length, it's end index? check C code loop
    # C: for (i = 0; i < NUM_FEATURES; i++) ... NUM_FEATURES is 1900
    # ROI_START 1300. 1300 + 1900 = 3200. So ROI_END is strict limit or count?
    # Header says #define ROI_END 3200.
    num_features = consts['NUM_FEATURES']
    roi = deriv[roi_start : roi_start + num_features].copy()
    
    # 5. SNV on ROI
    mean = np.mean(roi)
    std = np.std(roi) # default ddof=0 which matches C implementation
    # C uses: sqrt(sum(d*d)/N) -> Population std dev (ddof=0)
    
    if std < 1e-8:
        inv_std = 1.0
    else:
        inv_std = 1.0 / std
        
    roi = (roi - mean) * inv_std
    
    return roi

def predict_chla(roi, consts, arrays):
    """Replicates pls_predict."""
    prediction = consts['PLS_INTERCEPT']
    
    pls_coef = arrays['PLS_COEF']
    pls_mean = arrays['PLS_X_MEAN']
    pls_std = arrays['PLS_X_STD']
    
    for i in range(len(roi)):
        x_std = pls_std[i]
        val = roi[i]
        
        scaled = 0.0
        if x_std > 1e-8:
            scaled = (val - pls_mean[i]) / x_std
            
        prediction += scaled * pls_coef[i]
        
    # Non-negative constraint
    if prediction < 0:
        prediction = 0.0
        
    return prediction

def read_bin_file(filepath):
    """Reads binary pixel data (uint16)."""
    with open(filepath, 'rb') as f:
        data = f.read()
    
    # TCD1304 has 3694 pixels (CCD_BUFFER_SIZE in firmware)
    # Check size
    if len(data) != 3694 * 2:
        print(f"Warning: File size {len(data)} does not match expected 3694*2 bytes. Reading what's there.")
        
    count = len(data) // 2
    pixels = struct.unpack(f'<{count}H', data)
    return pixels

def read_config(filepath):
    """Reads config.txt for integration time."""
    config = {}
    with open(filepath, 'r') as f:
        for line in f:
            if '=' in line:
                key, val = line.strip().split('=')
                config[key] = val
    return config

def main():
    parser = argparse.ArgumentParser(description="Verify Firmware Model on PC")
    parser.add_argument("folder", help="Path to MEAS_XXX folder")
    args = parser.parse_args()
    
    print(f"Loading model constants from: {HEADER_PATH}")
    consts, arrays = parse_header(HEADER_PATH)
    
    config_path = os.path.join(args.folder, "config.txt")
    data_path = os.path.join(args.folder, "laser1_data.bin") # 405nm typically for Chl-a? 
    # Wait, Chl-a model might use one laser and Chl-b another?
    # Usually:
    # 405nm -> Chl-a ? Or both?
    # Let's check firmware call:
    # `Menu_AutoMeas_OnFrame` calls `chl_predict_chla` with `model_accum_laser1`?
    # Check usage... but typically Laser 1 is 405nm.
    
    if not os.path.exists(config_path) or not os.path.exists(data_path):
        print("Error: config.txt or laser1_data.bin not found in folder.")
        return

    config = read_config(config_path)
    integ_time = float(config.get('integration_time_ms', 1000))
    print(f"Integration Time: {integ_time} ms")
    
    print("Reading raw data...")
    pixels = read_bin_file(data_path)
    
    print("Running Preprocessing...")
    roi = preprocess(pixels, integ_time, consts, arrays)
    
    print("Predicting Chl-A...")
    chla = predict_chla(roi, consts, arrays)
    
    print("-" * 30)
    print(f"Python Prediction (PC): {chla:.4f} mg/L")
    print("-" * 30)

if __name__ == "__main__":
    main()
