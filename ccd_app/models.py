import joblib
import numpy as np
import os
from scipy.signal import savgol_filter
from utils import log

class ModelInference:
    def __init__(self, model_dir):
        self.model_dir = model_dir
        self.chla_model = None
        self.chlb_model = None
        
        self.roi_start = 1300
        self.roi_end = 3200
        
        self.load_models()
        
    def load_models(self):
        try:
            path_a = os.path.join(self.model_dir, "chla_model.pkl")
            if os.path.exists(path_a):
                self.chla_model = joblib.load(path_a)
                log(f"Loaded Chl A Model from {path_a}")
            else:
                log(f"Chl A Model not found at {path_a}", "WARN")
                
            path_b = os.path.join(self.model_dir, "chlb_model.pkl")
            if os.path.exists(path_b):
                self.chlb_model = joblib.load(path_b)
                log(f"Loaded Chl B Model from {path_b}")
            else:
                log(f"Chl B Model not found at {path_b}", "WARN")
        except Exception as e:
            log(f"Error loading models: {e}", "ERROR")

    def preprocess(self, spectrum, integration_time_ms):
        # 1. Normalize by Integration Time
        norm = spectrum / (integration_time_ms + 1e-8)
        
        # 2. Savitzky-Golay Smooth
        smoothed = savgol_filter(norm, window_length=11, polyorder=2)
        
        # 3. First Derivative
        deriv = savgol_filter(smoothed, window_length=11, polyorder=3, deriv=1)
        
        # 4. SNV
        mean = np.mean(deriv)
        std = np.std(deriv)
        snv = (deriv - mean) / (std + 1e-8)
        
        # 5. Crop ROI
        cropped = snv[self.roi_start:self.roi_end]
        
        return cropped.reshape(1, -1) # Reshape for sklearn

    def predict_chla(self, spectrum, dark_frame, integration_time_ms):
        if not self.chla_model: return None
        
        net_signal = spectrum.astype(float) - dark_frame.astype(float)
        processed = self.preprocess(net_signal, integration_time_ms)
        
        # PLS predict
        res = self.chla_model.predict(processed)
        return float(res[0])

    def predict_chlb(self, spectrum, dark_frame, integration_time_ms):
        if not self.chlb_model: return None
        
        net_signal = spectrum.astype(float) - dark_frame.astype(float)
        processed = self.preprocess(net_signal, integration_time_ms)
        
        # Pipeline predict
        res = self.chlb_model.predict(processed)
        return float(res[0])
