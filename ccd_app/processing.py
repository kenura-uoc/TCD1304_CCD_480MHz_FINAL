import numpy as np
from scipy.signal import savgol_filter
from scipy.ndimage import median_filter

class Calibration:
    def __init__(self):
        self.enabled = False
        self.p1_px = 0
        self.p1_nm = 0.0
        self.p2_px = 3694
        self.p2_nm = 1000.0
        self.slope = 0.0
        self.intercept = 0.0
        
    def update(self, p1_px, p1_nm, p2_px, p2_nm):
        self.p1_px, self.p1_nm = p1_px, p1_nm
        self.p2_px, self.p2_nm = p2_px, p2_nm
        if (p2_px - p1_px) != 0:
            self.slope = (p2_nm - p1_nm) / (p2_px - p1_px)
            self.intercept = p1_nm - self.slope * p1_px
            self.enabled = True
        else:
            self.enabled = False
            
    def pixel_to_nm(self, px):
        if not self.enabled: return px
        return self.slope * px + self.intercept

    def get_axis_label(self):
        return "Wavelength (nm)" if self.enabled else "Pixel Index"

class PeakDetector:
    def __init__(self):
        self.threshold = 15000  # Higher default threshold
        self.min_distance = 100 # Higher default distance
        self.smooth_window = 11 # Savitzky-Golay window
        self.poly_order = 3     # Savitzky-Golay order
        self.median_filter_size = 3 # Median filter window size for despiking
        self.use_smoothing = True
        
    def find_peaks(self, data):
        if len(data) == 0: return [], []
        
        # 0. Despiking (Median Filter)
        # Apply median filter if enabled and data length is sufficient
        if self.median_filter_size > 1 and len(data) >= self.median_filter_size:
            data_for_processing = median_filter(data, size=self.median_filter_size)
        else:
            data_for_processing = data

        # 1. Smoothing (Savitzky-Golay)
        if self.use_smoothing and len(data_for_processing) > self.smooth_window:
            # Ensure window is odd and less than data length
            window = self.smooth_window | 1 
            if window < 3: window = 3
            try:
                processed_data = savgol_filter(data, window, self.poly_order)
            except:
                processed_data = data
        else:
            processed_data = data
            
        # 2. Thresholding
        candidates = np.where(processed_data > self.threshold)[0]
        if len(candidates) == 0: return [], []
        
        # 3. Local Maxima with optimized neighbor check
        #   (d[i] > d[i-1] AND d[i] > d[i+1])
        #   We use a vectorized approach manually to be safe
        
        # Pre-compute is_peak boolean array
        padded = np.pad(processed_data, (1, 1), mode='constant', constant_values=0)
        is_max = (padded[1:-1] > padded[:-2]) & (padded[1:-1] > padded[2:])
        
        # Candidate indices are where is_max is true AND value > threshold
        candidate_indices = np.where(is_max & (processed_data > self.threshold))[0]

        # 4. Filter by Min Distance (Greedy Left-to-Right)
        final_peaks_x = []
        final_peaks_y = [] # We return the SMOOTHED y or RAW y? Usually RAW y at that index looks best on graph.
                           # But for detection the smoothed one was used. Let's return RAW Y.
        
        last_peak_idx = -self.min_distance
        
        for idx in candidate_indices:
            if idx - last_peak_idx >= self.min_distance:
                final_peaks_x.append(idx)
                final_peaks_y.append(data[idx]) # Use raw data Y value
                last_peak_idx = idx
                
        return np.array(final_peaks_x), np.array(final_peaks_y)
