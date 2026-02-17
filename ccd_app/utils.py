import threading
import time
import os
import json
from datetime import datetime
import dearpygui.dearpygui as dpg

class LogManager:
    _instance = None
    
    def __init__(self):
        self.logs = []
        self.lock = threading.Lock()
        
    @classmethod
    def get(cls):
        if cls._instance is None:
            cls._instance = LogManager()
        return cls._instance
        
    def log(self, msg, level="INFO"):
        ts = datetime.now().strftime("%H:%M:%S")
        entry = f"[{ts}] [{level}] {msg}"
        print(entry)
        with self.lock:
            self.logs.append(entry)
            if len(self.logs) > 1000: self.logs.pop(0)
            
        # Update UI if context exists and item exists
        try:
            if dpg.is_dearpygui_running() and dpg.does_item_exist("log_list"):
                dpg.configure_item("log_list", items=list(reversed(self.logs)))
        except:
            pass

def log(msg, level="INFO"):
    LogManager.get().log(msg, level)

class SettingsManager:
    def __init__(self, filename="settings.json"):
        # Resolve settings file relative to CWD or Script
        self.filename = filename
        self.defaults = {
            "enable_savgol": True,
            "savgol_window": 11,
            "frame_average": 1, # 1 = Off
            "peak_threshold": 15000,
            "peak_min_dist": 100,
            "last_project": "Default",
            "remove_dummies": False,
            "y_max": 65535,
            "integration_time_ms": 1000  # Default integration time in ms
        }
        self.data = self.defaults.copy()
        self.load()
        
    def load(self):
        if os.path.exists(self.filename):
            try:
                with open(self.filename, 'r') as f:
                    loaded = json.load(f)
                    self.data.update(loaded)
            except:
                print("Failed to load settings")
                
    def save(self):
        try:
            with open(self.filename, 'w') as f:
                json.dump(self.data, f, indent=4)
        except:
            print("Failed to save settings")
            
    def get(self, key):
        return self.data.get(key, self.defaults.get(key))
        
    def set(self, key, value):
        self.data[key] = value

class ProjectManager:
    def __init__(self, base_dir="projects"):
        self.base_dir = base_dir
        self.current_project = "Default"
        self.ensure_project("Default")
        
    def ensure_project(self, name):
        path = os.path.join(self.base_dir, name)
        if not os.path.exists(path):
            try:
                os.makedirs(path)
            except OSError as e:
                log(f"Failed to create project dir: {e}", "ERROR")
            
    def get_projects(self):
        if not os.path.exists(self.base_dir):
            return ["Default"]
        return [d for d in os.listdir(self.base_dir) if os.path.isdir(os.path.join(self.base_dir, d))]
        
    def create_project(self, name):
        name = "".join(c for c in name if c.isalnum() or c in (' ', '_', '-')).strip()
        if not name: return False
        self.ensure_project(name)
        self.current_project = name
        return True
        
    def get_recording_dir(self):
        return os.path.join(self.base_dir, self.current_project)
