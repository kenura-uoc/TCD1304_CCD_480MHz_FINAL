import numpy as np
import os
import json
import time
from datetime import datetime
from .utils import log

class DataManager:
    """
    Manages spectral data storage in .npz format.
    
    Format:
    - dark_spectra: (N, 3694) uint16
    - laser_a_spectra: (N, 3694) uint16
    - laser_b_spectra: (N, 3694) uint16
    - concentrations: (N, 2) float32 [Chl-A, Chl-B]
    - metadata: list of JSON dicts (timestamps, notes, etc.)
    """
    
    def __init__(self, project_dir):
        self.project_dir = project_dir
        self.filename = os.path.join(project_dir, "dataset.npz")
        
        self.dark_spectra = np.zeros((0, 3694), dtype=np.uint16)
        self.laser_a_spectra = np.zeros((0, 3694), dtype=np.uint16)
        self.laser_b_spectra = np.zeros((0, 3694), dtype=np.uint16)
        self.concentrations = np.zeros((0, 2), dtype=np.float32)
        self.metadata = []
        
        self.load()
        
    def load(self):
        if os.path.exists(self.filename):
            try:
                data = np.load(self.filename, allow_pickle=True)
                if 'dark_spectra' in data: self.dark_spectra = data['dark_spectra']
                if 'laser_a_spectra' in data: self.laser_a_spectra = data['laser_a_spectra']
                if 'laser_b_spectra' in data: self.laser_b_spectra = data['laser_b_spectra']
                if 'concentrations' in data: self.concentrations = data['concentrations']
                if 'metadata' in data: self.metadata = list(data['metadata'])
                log(f"Loaded dataset: {len(self.metadata)} samples")
            except Exception as e:
                log(f"Failed to load dataset: {e}", "ERROR")
                
    def save(self):
        try:
            np.savez_compressed(
                self.filename,
                dark_spectra=self.dark_spectra,
                laser_a_spectra=self.laser_a_spectra,
                laser_b_spectra=self.laser_b_spectra,
                concentrations=self.concentrations,
                metadata=self.metadata
            )
            log(f"Saved dataset to {self.filename}")
        except Exception as e:
            log(f"Failed to save dataset: {e}", "ERROR")
            
    def add_sample(self, dark, laser_a, laser_b, conc_a, conc_b, note="", integration_time=0):
        """
        Add a new sample trio (Dark, Laser A, Laser B) and concentrations.
        """
        # Ensure 1D arrays
        dark = np.array(dark, dtype=np.uint16).flatten()
        laser_a = np.array(laser_a, dtype=np.uint16).flatten()
        laser_b = np.array(laser_b, dtype=np.uint16).flatten()
        
        if len(dark) != 3694 or len(laser_a) != 3694 or len(laser_b) != 3694:
            log("Invalid spectrum length", "ERROR")
            return False
            
        self.dark_spectra = np.vstack([self.dark_spectra, dark])
        self.laser_a_spectra = np.vstack([self.laser_a_spectra, laser_a])
        self.laser_b_spectra = np.vstack([self.laser_b_spectra, laser_b])
        
        self.concentrations = np.vstack([self.concentrations, [conc_a, conc_b]])
        
        meta = {
            "timestamp": datetime.now().isoformat(),
            "note": note,
            "integration_time": integration_time
        }
        self.metadata.append(meta)
        
        self.save()
        return True
