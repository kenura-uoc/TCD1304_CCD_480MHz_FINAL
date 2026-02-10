"""
CCD Monitor 2.0 - TCD1304 Spectrometer Interface
"""

import dearpygui.dearpygui as dpg
import serial
import serial.tools.list_ports
import struct
import numpy as np
import threading
import time
import os
import json
from datetime import datetime
from scipy.signal import savgol_filter

# ==========================================
# CONSTANTS
# ==========================================
CCD_PIXELS = 3694
FRAME_SIZE = CCD_PIXELS * 2 + 4  # 2 magic + 2 frame count + 3694*2 pixels = 7392
BAUD_RATE = 10000000 # 10 Mbps (virtual)

# ==========================================
# LOGGING
# ==========================================
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
            
        # Update UI if context exists
        if dpg.does_item_exist("log_list"):
            dpg.configure_item("log_list", items=list(reversed(self.logs)))

def log(msg, level="INFO"):
    LogManager.get().log(msg, level)


# ==========================================
# LOGIC CLASSES
# ==========================================

class SettingsManager:
    def __init__(self, filename="settings.json"):
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
            "integration_time_ms": 18  # Default integration time in ms
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
            os.makedirs(path)
            
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
        self.use_smoothing = True
        
    def find_peaks(self, data):
        if len(data) == 0: return [], []
        
        # 1. Smoothing (Savitzky-Golay)
        if self.use_smoothing and len(data) > self.smooth_window:
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
        
        peaks_x = []
        peaks_y = []
        
        last_peak_idx = -self.min_distance
        
        # Simple Greedy Search on the *processed* data
        # We look for local maxima
        
        # Pre-compute is_peak boolean array
        padded = np.pad(processed_data, (1, 1), mode='constant', constant_values=0)
        is_max = (padded[1:-1] > padded[:-2]) & (padded[1:-1] > padded[2:])
        
        # Candidate indices are where is_max is true AND value > threshold
        candidate_indices = np.where(is_max & (processed_data > self.threshold))[0]

        # 4. Filter by Min Distance (Greedy Left-to-Right)
        final_peaks_x = []
        final_peaks_y = [] # We return the SMOOTHED y or RAW y? Usually RAW y at that index looks best on graph.
                           # But for detection the smoothed one was used. Let's return RAW Y.
        
        for idx in candidate_indices:
            if idx - last_peak_idx >= self.min_distance:
                final_peaks_x.append(idx)
                final_peaks_y.append(data[idx]) # Use raw data Y value
                last_peak_idx = idx
                
        return np.array(final_peaks_x), np.array(final_peaks_y)


class CCDReceiver:
    def __init__(self):
        # ... (init remains same)
        self.pixels = np.zeros(CCD_PIXELS, dtype=np.uint16)
        self.frame_count = 0
        self.fps = 0
        self.running = True
        self.connected = False
        self.serial = None
        self.lock = threading.Lock()
        self.last_fps_time = time.time()
        self.fps_frame_count = 0
        self.frozen = False 
        self.frame_ready = False
        self.single_shot_pending = False
        self.recording = False
        self.recording_conditional = False
        self.recorded_frames = []
        self.pending_single_shot = False # New flag for "One Shot" logic
        
        # Frame Averaging
        self.frame_avg_count = 1
        self.accum_buffer = None
        self.accum_count = 0
        
    def connect(self, port):
        if self.serial: self.serial.close()
        try:
            self.serial = serial.Serial(port, BAUD_RATE, timeout=0.5)
            self.connected = True
            log(f"Connected to {port}")
            return True
        except Exception as e:
            log(f"Connect failed: {e}", "ERROR")
            self.connected = False
            return False
            
    def disconnect(self):
        self.connected = False
        if self.serial: 
            try:
                self.serial.close()
            except:
                pass
        self.serial = None
        log("Disconnected")

    def start_recording(self):
        self.recording = True
        self.recorded_frames = []
        log("Recording Started")
        
    def stop_recording(self):
        self.recording = False
        log("Recording Stopped")
        
    def read_frame(self):
        if not self.connected or not self.serial: return False
        try:
            while self.running:
                b = self.serial.read(1)
                if not b: return False
                if b[0] == 0xCD:
                    b2 = self.serial.read(1)
                    if b2 and b2[0] == 0xAB:
                        data = self.serial.read(FRAME_SIZE - 2)
                        if len(data) == FRAME_SIZE - 2:
                            frame_num = struct.unpack('<H', data[0:2])[0]
                            # Raw pixels
                            raw_pixels = np.frombuffer(data[2:], dtype=np.uint16).copy()
                            
                            # ... (Average logic same) ...
                            if self.frame_avg_count > 1:
                                if self.accum_buffer is None:
                                    self.accum_buffer = raw_pixels.astype(np.float32)
                                    self.accum_count = 1
                                else:
                                    self.accum_buffer += raw_pixels
                                    self.accum_count += 1
                                    
                                if self.accum_count >= self.frame_avg_count:
                                    final_pixels = (self.accum_buffer / self.accum_count).astype(np.uint16)
                                    self.accum_buffer = None
                                    self.accum_count = 0
                                    # Output this average frame
                                    with self.lock:
                                        self.pixels = final_pixels
                                        self.frame_count = frame_num
                                        self.frame_ready = True
                                        self._handle_recording(frame_num, final_pixels)
                                        self._handle_singleshot()
                            else:
                                # No averaging
                                with self.lock:
                                    self.pixels = raw_pixels
                                    self.frame_count = frame_num
                                    self.frame_ready = True
                                    self._handle_recording(frame_num, raw_pixels)
                                    self._handle_singleshot()
                                    
                            self.fps_frame_count += 1
                            now = time.time()
                            if now - self.last_fps_time >= 1.0:
                                self.fps = self.fps_frame_count
                                self.fps_frame_count = 0
                                self.last_fps_time = now
                            return True
        except (serial.SerialException, OSError, PermissionError):
            self.disconnect()
            return False
        except Exception as e:
            # Handle byref error specifically or generic
            if "byref" in str(e) or "NoneType" in str(e):
                 # This happens on forceful close
                 self.disconnect()
            else:
                 log(f"Read error: {e}", "ERROR")
            return False
        return False
        
    def _handle_recording(self, frame_num, pixels):
        if self.recording or (self.recording_conditional and not self.frozen):
            self.recorded_frames.append({
                'frame_num': frame_num,
                'timestamp': time.time(),
                'pixels': pixels.copy()
            })
            
    def _handle_singleshot(self):
        if self.pending_single_shot:
            self.pending_single_shot = False
            self.frozen = True
        
    def set_mode(self, mode_idx):
        if self.connected and self.serial:
            try:
                self.serial.write(f"M{mode_idx}".encode('ascii'))
            except:
                self.disconnect()

    def trigger_single_shot(self):
        """Unfreeze, wait for next frame, then freeze"""
        self.frozen = False
        self.pending_single_shot = True

    def send_integration_time(self, time_ms):
        """Send integration time command to STM32"""
        if self.connected and self.serial:
            try:
                # Command format: I<time_ms>
                cmd = f"I{time_ms}".encode('ascii')
                self.serial.write(cmd)
                log(f"Sent integration time: {time_ms}ms")
                return True
            except:
                self.disconnect()
                return False
        return False


# ==========================================
# MAIN APP
# ==========================================

class CCDApp:
    def __init__(self):
        self.settings = SettingsManager()
        self.receiver = CCDReceiver()
        self.project_mgr = ProjectManager()
        self.calibration = Calibration()
        self.peak_detector = PeakDetector()
        
        # Load Settings - Note: Signal is always inverted (light=high)
        self.peak_detector.use_smoothing = self.settings.get("enable_savgol")
        self.peak_detector.smooth_window = self.settings.get("savgol_window")
        self.peak_detector.threshold = self.settings.get("peak_threshold")
        self.peak_detector.min_distance = self.settings.get("peak_min_dist")
        self.receiver.frame_avg_count = self.settings.get("frame_average")
        self.remove_dummies = self.settings.get("remove_dummies")
        self.y_max = self.settings.get("y_max")
        self.integration_time_ms = self.settings.get("integration_time_ms")
        # FFT Flicker Analysis
        self.frame_means = []  # Track mean intensity per frame for FFT
        self.max_fft_frames = 256  # Number of frames to keep for FFT
        
        self.project_mgr.ensure_project(self.settings.get("last_project"))
        self.project_mgr.current_project = self.settings.get("last_project")
        
        self.history_files = []
        self.history_data = None
        self.history_idx = 0
        
        self.show_peaks = True
        self.show_history = False
        
        # Playback State
        self.playback_active = False
        self.playback_last_time = 0
        
        self.t = threading.Thread(target=self.bg_loop, daemon=True)
        self.t.start()
        
        self.setup_ui()
        
    def bg_loop(self):
        while self.receiver.running:
            if self.receiver.connected:
                self.receiver.read_frame()
            else:
                time.sleep(0.5)

    def save_settings(self):
        self.settings.set("enable_savgol", self.peak_detector.use_smoothing)
        self.settings.set("savgol_window", self.peak_detector.smooth_window)
        self.settings.set("frame_average", self.receiver.frame_avg_count)
        self.settings.set("peak_threshold", self.peak_detector.threshold)
        self.settings.set("peak_min_dist", self.peak_detector.min_distance)
        self.settings.set("last_project", self.project_mgr.current_project)
        self.settings.set("remove_dummies", self.remove_dummies)
        self.settings.set("y_max", self.y_max)
        self.settings.set("integration_time_ms", self.integration_time_ms)
        self.settings.save()

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        dpg.configure_item("cb_ports", items=ports)
        if ports: dpg.set_value("cb_ports", ports[0])

    def cb_connect(self):
        port = dpg.get_value("cb_ports")
        if self.receiver.connect(port):
            dpg.set_value("status_txt", f"Connected: {port}")
        else:
            dpg.set_value("status_txt", "Connection Failed")
            
    def cb_disconnect(self):
        self.receiver.disconnect()
        dpg.set_value("status_txt", "Disconnected")

    def cb_apply_integration_time(self):
        """Apply button clicked - send integration time to STM32"""
        new_time = int(dpg.get_value("slider_integration"))
        old_time = self.integration_time_ms
        self.integration_time_ms = new_time
        if self.receiver.send_integration_time(new_time):
            log(f"Integration time changed: {old_time}ms -> {new_time}ms")
            self.save_settings()
        else:
            log("Failed to send integration time (not connected?)", "WARN")

    def cb_create_project(self):
        name = dpg.get_value("new_proj_name")
        if self.project_mgr.create_project(name):
            dpg.set_value("cur_proj_txt", f"Project: {name}")
            self.refresh_history_list()
            dpg.configure_item("proj_win", show=False)
            
    def cb_record_toggle(self):
        if self.receiver.recording:
            self.save_recording()
            dpg.configure_item("btn_rec", label="Record")
        else:
            self.receiver.start_recording()
            dpg.configure_item("btn_rec", label="Stop & Save")

    def save_recording(self):
        with self.receiver.lock:
            frames = self.receiver.recorded_frames
            self.receiver.recorded_frames = []
            self.receiver.recording = False
            
        if not frames: return
        
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(self.project_mgr.get_recording_dir(), f"rec_{ts}.npz")
        
        n = len(frames)
        pix = np.zeros((n, CCD_PIXELS), dtype=np.uint16)
        nums = np.zeros(n, dtype=np.uint16)
        for i, f in enumerate(frames):
            pix[i] = f['pixels']
            nums[i] = f['frame_num']
            
        np.savez_compressed(fname, pixels=pix, frame_numbers=nums)
        print(f"Saved {fname}")
        self.refresh_history_list()

    def refresh_history_list(self):
        d = self.project_mgr.get_recording_dir()
        if os.path.exists(d):
            files = [f for f in os.listdir(d) if f.endswith(".npz")]
            files.sort(reverse=True)
            self.history_files = files
            dpg.configure_item("lb_history", items=files)
            
    def cb_playback_toggle(self):
        self.playback_active = not self.playback_active
        label = "Pause" if self.playback_active else "Play"
        dpg.configure_item("btn_play", label=label)
        # Auto-enable history overlay
        self.show_history = True
        dpg.set_value("chk_history", True)

    def cb_playback_step(self, step):
        self.playback_active = False
        dpg.configure_item("btn_play", label="Play")
        if self.history_data:
            cur = dpg.get_value("slider_hist")
            new_idx = cur + step
            max_idx = self.history_data['frames'] - 1
            if new_idx < 0: new_idx = 0
            if new_idx > max_idx: new_idx = max_idx
            dpg.set_value("slider_hist", new_idx)
        # Auto-enable history overlay
        self.show_history = True
        dpg.set_value("chk_history", True)

    def cb_load_history(self, s, a):
        if not a: return
        path = os.path.join(self.project_mgr.get_recording_dir(), a)
        try:
            data = np.load(path)
            self.history_data = {
                'pixels': data['pixels'],
                'frames': len(data['pixels'])
            }
            self.history_idx = 0
            self.playback_active = False
            dpg.configure_item("btn_play", label="Play")
            dpg.configure_item("slider_hist", max_value=self.history_data['frames']-1)
            self.show_history = True
            dpg.configure_item("series_history_line", show=True)
            # Auto-enable history overlay
            self.show_history = True
            dpg.set_value("chk_history", True)
        except Exception as e:
            print(f"Load failed: {e}")

    def update(self):
        # 0. Apply Axis Limits
        dpg.set_axis_limits("y_axis", 0, self.y_max)
        # Update Voltage Axis Limits
        if dpg.does_item_exist("y_axis_volt"):
            dpg.set_axis_limits("y_axis_volt", 0, 3.3 * (self.y_max / 65535.0))
        
        # Calculate X Limits
        x_min, x_max = 0, CCD_PIXELS
        if self.calibration.enabled:
            x_min = self.calibration.pixel_to_nm(0)
            x_max = self.calibration.pixel_to_nm(CCD_PIXELS)
            
        if self.remove_dummies:
            if self.calibration.enabled:
                x_min = self.calibration.pixel_to_nm(32)
                x_max = self.calibration.pixel_to_nm(3680)
            else:
                x_min, x_max = 32, 3680
                
        if x_min > x_max: x_min, x_max = x_max, x_min
        dpg.set_axis_limits("x_axis", x_min, x_max)
        
        if self.receiver.frame_ready and not self.receiver.frozen:
            with self.receiver.lock:
                self.receiver.frame_ready = False
                pixels = self.receiver.pixels.copy()
            
            # 1. Inversion - Always invert (TCD1304 outputs dark=high, light=low)
            pixels = 65535 - pixels
                
            # 2. X Axis & Dummy Removal
            if self.calibration.enabled:
                full_x_data = np.array([self.calibration.pixel_to_nm(i) for i in range(CCD_PIXELS)])
            else:
                full_x_data = np.arange(CCD_PIXELS)
                
            if self.remove_dummies:
                # Slice logic: Keep 32 to 3680
                start, end = 32, 3680
                if end > len(pixels): end = len(pixels)
                display_pixels = pixels[start:end]
                display_x = full_x_data[start:end]
                # Adjust indices for peak detection relative to slice if needed, 
                # but peak detector works on passed array.
            else:
                display_pixels = pixels
                display_x = full_x_data
                
            dpg.set_value("series_live", [display_x.tolist(), display_pixels.tolist()])
            
            # 3. Peaks (Detect on DISPLAY pixels to match visual)
            if self.show_peaks:
                px, py = self.peak_detector.find_peaks(display_pixels)
                if len(px) > 0:
                    px_indices = px.astype(int)
                    px_x_coords = display_x[px_indices]
                    dpg.set_value("series_peaks", [px_x_coords.tolist(), py.tolist()])
                else:
                    dpg.set_value("series_peaks", [[], []])
            
            # 4. FFT Flicker Analysis - track mean intensity per frame
            frame_mean = float(np.mean(display_pixels))
            self.frame_means.append(frame_mean)
            if len(self.frame_means) > self.max_fft_frames:
                self.frame_means = self.frame_means[-self.max_fft_frames:]
            
            # Compute FFT on frame means to find flicker frequency
            if len(self.frame_means) >= 16:
                means = np.array(self.frame_means)
                means_centered = means - np.mean(means)  # Remove DC
                fft_vals = np.abs(np.fft.rfft(means_centered))
                freqs = np.fft.rfftfreq(len(means_centered), d=1.0)  # In cycles/frame
                # Skip DC (index 0)
                if len(fft_vals) > 1:
                    fft_vals[0] = 0
                    peak_idx = np.argmax(fft_vals)
                    peak_freq = freqs[peak_idx]
                    peak_power = fft_vals[peak_idx]
                    
                    # Update FFT plot if it exists
                    if dpg.does_item_exist("series_fft"):
                        dpg.set_value("series_fft", [freqs[1:].tolist(), fft_vals[1:].tolist()])
                    
                    fps = max(self.receiver.fps, 1)
                    flicker_hz = peak_freq * fps
                    dpg.set_value("status_bar", 
                        f"FPS: {self.receiver.fps} | Frame: {self.receiver.frame_count} | "
                        f"Flicker: {flicker_hz:.1f}Hz (power:{peak_power:.0f})")
                else:
                    dpg.set_value("status_bar", f"FPS: {self.receiver.fps} | Frame: {self.receiver.frame_count}")
            else:
                dpg.set_value("status_bar", f"FPS: {self.receiver.fps} | Frame: {self.receiver.frame_count} | FFT: collecting...")

        if self.show_history and self.history_data:
            if self.playback_active:
                now = time.time()
                if now - self.playback_last_time > 0.033: # ~30 FPS
                    self.playback_last_time = now
                    cur = dpg.get_value("slider_hist")
                    cur += 1
                    if cur >= self.history_data['frames']: cur = 0
                    dpg.set_value("slider_hist", cur)
            
            idx = dpg.get_value("slider_hist")
            h_pixels = self.history_data['pixels'][idx]
            # Always invert history data too
            h_pixels = 65535 - h_pixels
                 
            if self.calibration.enabled:
                h_full_x = np.array([self.calibration.pixel_to_nm(i) for i in range(CCD_PIXELS)])
            else:
                h_full_x = np.arange(CCD_PIXELS)
                
            if self.remove_dummies:
                start, end = 32, 3680
                display_h_pixels = h_pixels[start:end]
                display_h_x = h_full_x[start:end]
            else:
                display_h_pixels = h_pixels
                display_h_x = h_full_x
                
            dpg.set_value("series_history_line", [display_h_x.tolist(), display_h_pixels.tolist()])

    def setup_ui(self):
        dpg.create_context()
        
        with dpg.window(tag="main_win"):
            
            # TOP BAR
            with dpg.group(horizontal=True):
                dpg.add_text("FPS: 0", tag="status_bar")
            
            with dpg.group(horizontal=True):
                
                # LEFT SIDEBAR
                with dpg.child_window(width=300, tag="sidebar"):
                    
                    with dpg.tab_bar():
                        
                        # TAB 1: CONTROLS
                        with dpg.tab(label="Controls"):
                            dpg.add_text("Connection")
                            dpg.add_combo([], tag="cb_ports", width=-1)
                            with dpg.group(horizontal=True):
                                dpg.add_button(label="Refresh", callback=self.refresh_ports)
                                dpg.add_button(label="Connect", callback=self.cb_connect)
                                dpg.add_button(label="Disconnect", callback=self.cb_disconnect)
                            dpg.add_text("Not Connected", tag="status_txt", color=(255,100,100))
                            
                            dpg.add_separator()
                            dpg.add_text("Acquisition")
                            
                            with dpg.group(horizontal=True):
                                dpg.add_button(label="Run", callback=lambda: setattr(self.receiver, 'frozen', False))
                                dpg.add_button(label="Freeze", callback=lambda: setattr(self.receiver, 'frozen', True))
                                dpg.add_button(label="Single Shot", callback=self.receiver.trigger_single_shot)
                            
                            dpg.add_text("Integration Time (ms)")
                            dpg.add_text("Min 15ms (readout), Max 100ms", color=(150, 150, 150))
                            with dpg.group(horizontal=True):
                                dpg.add_slider_int(label="##integration", tag="slider_integration",
                                                  default_value=self.integration_time_ms, 
                                                  min_value=15, max_value=100, width=180)
                                dpg.add_button(label="Apply", callback=self.cb_apply_integration_time, width=60)

                            dpg.add_separator()
                            dpg.add_text("Signal Processing")
                            dpg.add_text("Signal always inverted (Light=High)", color=(100, 200, 100))
                            
                            dpg.add_text("Temporal Smoothing (Avg Frames)")
                            dpg.add_slider_int(label="Avg", default_value=self.receiver.frame_avg_count, min_value=1, max_value=20, 
                                              callback=lambda s,a: [setattr(self.receiver, 'frame_avg_count', a), self.save_settings()])
                            
                            dpg.add_separator()
                            dpg.add_text("View Control")
                            dpg.add_checkbox(label="Hide Dummy Pixels (32-3680)", default_value=self.remove_dummies,
                                            callback=lambda s,a: [setattr(self, 'remove_dummies', a), self.save_settings()])
                            dpg.add_slider_int(label="Y Max", default_value=self.y_max, min_value=1000, max_value=65535,
                                              callback=lambda s,a: [setattr(self, 'y_max', a), self.save_settings()])
                            
                            dpg.add_separator()
                            dpg.add_text("Recording")
                            dpg.add_text(f"Project: {self.project_mgr.current_project}", tag="cur_proj_txt")
                            
                            with dpg.window(label="New Project", modal=True, show=False, tag="proj_win", width=200, height=100):
                                dpg.add_input_text(tag="new_proj_name", hint="Project Name")
                                dpg.add_button(label="Create", callback=self.cb_create_project)
                            dpg.add_button(label="New Project", callback=lambda: dpg.configure_item("proj_win", show=True))
                            dpg.add_button(label="Record", tag="btn_rec", callback=self.cb_record_toggle, width=-1, height=40)

                        # TAB 2: ANALYSIS
                        with dpg.tab(label="Analysis"):
                            dpg.add_text("Calibration (Px -> nm)")
                            dpg.add_input_int(label="Px 1", tag="cal_p1_px", default_value=0, width=100)
                            dpg.add_input_float(label="nm 1", tag="cal_p1_nm", default_value=400.0, width=100)
                            dpg.add_input_int(label="Px 2", tag="cal_p2_px", default_value=3694, width=100)
                            dpg.add_input_float(label="nm 2", tag="cal_p2_nm", default_value=800.0, width=100)
                            
                            def apply_cal():
                                self.calibration.update(
                                    dpg.get_value("cal_p1_px"), dpg.get_value("cal_p1_nm"),
                                    dpg.get_value("cal_p2_px"), dpg.get_value("cal_p2_nm")
                                )
                                dpg.configure_item("x_axis", label=self.calibration.get_axis_label())
                            dpg.add_button(label="Apply Calibration", callback=apply_cal)
                            
                            dpg.add_separator()
                            dpg.add_text("Peak Detection")
                            dpg.add_checkbox(label="Show Peaks", default_value=True, callback=lambda s,a: setattr(self, 'show_peaks', a))
                            dpg.add_slider_float(label="Thresh", tag="peak_thresh", default_value=self.peak_detector.threshold, max_value=65535, 
                                                callback=lambda s,a: [setattr(self.peak_detector, 'threshold', a), self.save_settings()])
                            dpg.add_slider_int(label="Min Dist", tag="peak_dist", default_value=self.peak_detector.min_distance, max_value=500,
                                              callback=lambda s,a: [setattr(self.peak_detector, 'min_distance', a), self.save_settings()])
                            dpg.add_text("Smoothing (Savitzky-Golay)")
                            dpg.add_checkbox(label="Enable", default_value=self.peak_detector.use_smoothing, 
                                            callback=lambda s,a: [setattr(self.peak_detector, 'use_smoothing', a), self.save_settings()])
                            dpg.add_slider_int(label="window", default_value=self.peak_detector.smooth_window, max_value=51, 
                                              callback=lambda s,a: [setattr(self.peak_detector, 'smooth_window', a), self.save_settings()])
                            
                            dpg.add_separator()
                            dpg.add_text("Flicker FFT Analysis")
                            dpg.add_text("Shows frequency of intensity oscillations", color=(150, 150, 150))
                            with dpg.plot(label="FFT", height=150, width=-1):
                                dpg.add_plot_axis(dpg.mvXAxis, label="Freq (cycles/frame)", tag="fft_x_axis")
                                dpg.add_plot_axis(dpg.mvYAxis, label="Power", tag="fft_y_axis")
                                dpg.add_line_series([], [], label="FFT", parent="fft_y_axis", tag="series_fft")

                        # TAB 3: HISTORY
                        with dpg.tab(label="History"):
                            dpg.add_text("Recordings")
                            dpg.add_listbox([], tag="lb_history", width=-1, num_items=8, callback=self.cb_load_history)
                            dpg.add_separator()
                            dpg.add_checkbox(label="Overlay History", tag="chk_history", default_value=False, callback=lambda s,a: setattr(self, 'show_history', a))
                            
                            dpg.add_text("Playback Control")
                            with dpg.group(horizontal=True):
                                dpg.add_button(label="<<", callback=lambda: self.cb_playback_step(-10))
                                dpg.add_button(label="<", callback=lambda: self.cb_playback_step(-1))
                                dpg.add_button(label="Play", tag="btn_play", callback=self.cb_playback_toggle, width=60)
                                dpg.add_button(label=">", callback=lambda: self.cb_playback_step(1))
                                dpg.add_button(label=">>", callback=lambda: self.cb_playback_step(10))
                                
                            dpg.add_slider_int(label="Frame", tag="slider_hist", default_value=0, max_value=1, width=-1)

                        # TAB 4: LOGS
                        with dpg.tab(label="Logs"):
                            dpg.add_listbox([], tag="log_list", width=-1, num_items=15)
                            dpg.add_button(label="Clear Logs", callback=lambda: [LogManager.get().logs.clear(), dpg.configure_item("log_list", items=[])])

                # MAIN PLOT
                with dpg.plot(tag="main_plot", height=-1, width=-1):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="Pixel Index", tag="x_axis")
                    
                    # Primary Y Axis (Intensity)
                    dpg.add_plot_axis(dpg.mvYAxis, label="Intensity (16-bit)", tag="y_axis")
                    dpg.set_axis_limits("y_axis", 0, self.y_max)
                    
                    # Secondary Y Axis (Voltage)
                    dpg.add_plot_axis(dpg.mvYAxis, label="Voltage (V)", tag="y_axis_volt")
                    dpg.set_axis_limits("y_axis_volt", 0, 3.3 * (self.y_max / 65535.0))
                    
                    # Live Series
                    dpg.add_line_series(list(range(CCD_PIXELS)), [0]*CCD_PIXELS, label="Live", parent="y_axis", tag="series_live")
                    
                    # Peak Series (Scatter)
                    dpg.add_scatter_series([], [], label="Peaks", parent="y_axis", tag="series_peaks")
                    
                    # History Series
                    dpg.add_line_series([], [], label="History", parent="y_axis", tag="series_history_line", show=False)
                    
                    # Custom theme for history/peaks
                    with dpg.theme(tag="theme_peaks"):
                        with dpg.theme_component(dpg.mvScatterSeries):
                            dpg.add_theme_color(dpg.mvPlotCol_MarkerOutline, (255, 0, 0, 255), category=dpg.mvThemeCat_Plots)
                            dpg.add_theme_color(dpg.mvPlotCol_MarkerFill, (255, 0, 0, 100), category=dpg.mvThemeCat_Plots)
                    dpg.bind_item_theme("series_peaks", "theme_peaks")
                    
                    # Bright theme for live line
                    with dpg.theme(tag="theme_live"):
                        with dpg.theme_component(dpg.mvLineSeries):
                            dpg.add_theme_color(dpg.mvPlotCol_Line, (0, 255, 255, 255), category=dpg.mvThemeCat_Plots)  # Bright cyan
                    dpg.bind_item_theme("series_live", "theme_live")
                    
                    # Brighter history line (orange)
                    with dpg.theme(tag="theme_history"):
                        with dpg.theme_component(dpg.mvLineSeries):
                            dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 165, 0, 200), category=dpg.mvThemeCat_Plots)  # Orange
                    dpg.bind_item_theme("series_history_line", "theme_history")


        dpg.create_viewport(title='CCD Monitor 2.0', width=1200, height=800)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main_win", True)
        self.refresh_ports()
        self.refresh_history_list()
        
        while dpg.is_dearpygui_running():
            self.update()
            dpg.render_dearpygui_frame()
            
        dpg.destroy_context()
        self.save_settings() # Save on exit
        self.receiver.disconnect()
        self.receiver.running = False


if __name__ == "__main__":
    app = CCDApp()
