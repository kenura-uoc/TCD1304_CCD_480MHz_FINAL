import dearpygui.dearpygui as dpg
import threading
import time
import numpy as np
import os
import serial.tools.list_ports
from datetime import datetime

from utils import log, SettingsManager, ProjectManager
from backend import CCDReceiver, CCD_PIXELS
from processing import Calibration, PeakDetector
from data import DataManager
from models import ModelInference

class MainWindow:
    def __init__(self):
        self.settings = SettingsManager()
        self.receiver = CCDReceiver()
        self.project_mgr = ProjectManager()
        self.calibration = Calibration()
        self.peak_detector = PeakDetector()
        
        # Load Settings
        self.peak_detector.use_smoothing = self.settings.get("enable_savgol")
        self.peak_detector.smooth_window = self.settings.get("savgol_window")
        self.peak_detector.threshold = self.settings.get("peak_threshold")
        self.peak_detector.min_distance = self.settings.get("peak_min_dist")
        self.receiver.frame_avg_count = self.settings.get("frame_average")
        self.remove_dummies = self.settings.get("remove_dummies")
        self.y_max = self.settings.get("y_max")
        self.integration_time_ms = self.settings.get("integration_time_ms")
        
        # Data & Models
        self.project_mgr.ensure_project(self.settings.get("last_project"))
        self.project_mgr.current_project = self.settings.get("last_project")
        
        self.data_mgr = DataManager(self.project_mgr.get_recording_dir())
        
        # Model Inference
        # Assuming app is run from root or ccd_app, check various paths
        # Search priorities: ./ml_model/models, ../ml_model/models
        possible_paths = [
            os.path.join(os.getcwd(), "ml_model", "models"),
            os.path.join(os.getcwd(), "..", "ml_model", "models"),
            os.path.join(os.getcwd(), "ml_model"),
            os.path.join(os.getcwd(), "..", "ml_model")
        ]
        
        model_path = possible_paths[0]
        for p in possible_paths:
            if os.path.exists(p):
                model_path = p
                break
                
        self.model_inference = ModelInference(model_path)
        
        # UI State
        self.show_peaks = True
        self.show_history = False
        self.history_files = []
        self.history_data = None
        self.history_idx = 0
        self.playback_active = False
        self.playback_last_time = 0
        self.frame_means = []
        self.max_fft_frames = 256
        
        # Manual Prediction State
        self.manual_dark = None
        self.manual_laser_a = None
        self.manual_laser_b = None
        self.manual_frames_buffer = []
        self.manual_collecting = False
        self.manual_target_count = 0
        
        # Sidebar State
        self.sidebar_width = 320
        
        self.pred_conc_a = 0.0
        self.pred_conc_b = 0.0
        
        self.t = threading.Thread(target=self.bg_loop, daemon=True)
        self.t.start()
        
        self.setup_ui()
        from utils import LogManager
        mgr = LogManager.get()
        mgr.ui_ready = True
        mgr.refresh_ui()  # Show initial logs immediately
        self.refresh_history_list()
        self.refresh_ports()
        
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

    # ================= UI CALLBACKS =================

    def cb_connect(self):
        port = dpg.get_value("cb_ports")
        if self.receiver.connect(port):
            dpg.set_value("status_txt", f"Connected: {port}")
        else:
            dpg.set_value("status_txt", "Connection Failed")
            
    def cb_disconnect(self):
        self.receiver.disconnect()
        dpg.set_value("status_txt", "Disconnected")

    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        dpg.configure_item("cb_ports", items=ports)
        if ports: dpg.set_value("cb_ports", ports[0])

    def cb_integration_range_changed(self, sender, app_data):
        if app_data == "Standard (15-100ms)":
            dpg.configure_item("slider_integration", min_value=15, max_value=100, show=True)
            dpg.configure_item("input_custom_integration", show=False)
            val = dpg.get_value("slider_integration")
            if val < 15 or val > 100: dpg.set_value("slider_integration", max(15, min(val, 100)))
        elif app_data == "Extended (100-1000ms)":
            dpg.configure_item("slider_integration", min_value=100, max_value=1000, show=True)
            dpg.configure_item("input_custom_integration", show=False)
            val = dpg.get_value("slider_integration")
            if val < 100 or val > 1000: dpg.set_value("slider_integration", max(100, min(val, 1000)))
        elif app_data == "Custom":
            dpg.configure_item("slider_integration", show=False)
            dpg.configure_item("input_custom_integration", show=True)

    def cb_apply_integration_time(self):
        range_mode = dpg.get_value("combo_integration_range")
        if range_mode == "Custom":
            try:
                new_time = int(dpg.get_value("input_custom_integration"))
            except: return
        else:
            new_time = int(dpg.get_value("slider_integration"))
        
        if new_time < 15: new_time = 15
        if new_time > 10000: new_time = 10000
        
        self.integration_time_ms = new_time
        if self.receiver.send_integration_time(new_time):
            log(f"Integration time set to {new_time}ms")
            self.save_settings()

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
        log(f"Saved recording: {fname}")
        self.refresh_history_list()

    def cb_create_project(self):
        name = dpg.get_value("new_proj_name")
        if self.project_mgr.create_project(name):
            self.refresh_project_list()
            dpg.set_value("combo_project", name)
            self.cb_change_project(None, name)
            dpg.configure_item("proj_win", show=False)

    def cb_change_project(self, sender, app_data):
        if not app_data: return
        self.project_mgr.ensure_project(app_data)
        self.project_mgr.current_project = app_data
        self.data_mgr = DataManager(self.project_mgr.get_recording_dir()) # Reload data mgr
        self.refresh_history_list()
        self.save_settings()
        
    def refresh_project_list(self):
        projects = self.project_mgr.get_projects()
        dpg.configure_item("combo_project", items=projects)

    def refresh_history_list(self):
        d = self.project_mgr.get_recording_dir()
        if os.path.exists(d):
            files = [f for f in os.listdir(d) if f.endswith(".npz") and not f == "dataset.npz"]
            files.sort(reverse=True)
            self.history_files = files
            if dpg.does_item_exist("lb_history"):
                dpg.configure_item("lb_history", items=files)

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
            dpg.set_value("chk_history", True)
        except Exception as e:
            log(f"Load failed: {e}")

    def cb_playback_toggle(self):
        self.playback_active = not self.playback_active
        label = "Pause" if self.playback_active else "Play"
        dpg.configure_item("btn_play", label=label)
        self.show_history = True
        dpg.set_value("chk_history", True)

    def cb_playback_step(self, step):
        self.playback_active = False
        dpg.configure_item("btn_play", label="Play")
        if self.history_data:
            cur = dpg.get_value("slider_hist")
            new_idx = cur + step
            max_idx = self.history_data['frames'] - 1
            dpg.set_value("slider_hist", max(0, min(new_idx, max_idx)))
        self.show_history = True
        dpg.set_value("chk_history", True)

    # ================= MANUAL PREDICTION =================

    def _collect_frames_thread(self, target_count, callback):
        log(f"Collecting {target_count} frames...")
        # Clear buffer
        self.manual_frames_buffer = []
        
        # Start recording
        with self.receiver.lock:
            self.receiver.recorded_frames = []
            self.receiver.recording = True
        
        # Wait
        while True:
            time.sleep(0.1)
            current_count = len(self.receiver.recorded_frames)
            if current_count >= target_count:
                break
            if not self.receiver.connected: # Safety break
                break
                
        # Stop recording
        with self.receiver.lock:
            self.receiver.recording = False
            start_idx = 0
            # If we over-collected, just take the last N or first N? Takes first N captured.
            frames = self.receiver.recorded_frames[:target_count]
        
        if frames:
            stack = np.array([f['pixels'] for f in frames])
            avg = np.mean(stack, axis=0).astype(np.uint16)
            callback(avg)
        else:
            log("Collection failed (no frames)", "ERROR")
            callback(None)

    def cb_collect_dark(self):
        if not self.receiver.connected:
            log("Not connected!", "ERROR")
            return
        dpg.configure_item("btn_dark", label="Collecting...")
        threading.Thread(target=self._collect_frames_thread, args=(10, self._on_dark_collected), daemon=True).start()

    def _on_dark_collected(self, avg_spec):
        if avg_spec is not None:
            self.manual_dark = avg_spec
            log("Dark Frame Collected.")
            dpg.configure_item("btn_dark", label="Done (Recollect)")
            dpg.configure_item("lbl_dark_status", default_value="Dark: OK", color=(0, 255, 0))
        else:
            dpg.configure_item("btn_dark", label="Collect Dark")

    def cb_predict_chla(self):
        if self.manual_dark is None:
            log("Please collect Dark Frame first!", "ERROR")
            return
        dpg.configure_item("btn_chla", label="Measuring...")
        threading.Thread(target=self._collect_frames_thread, args=(10, self._on_chla_collected), daemon=True).start()

    def _on_chla_collected(self, avg_spec):
        if avg_spec is not None:
            self.manual_laser_a = avg_spec
            val = self.model_inference.predict_chla(avg_spec, self.manual_dark, self.integration_time_ms)
            self.pred_conc_a = val
            # Display logic for negative values
            if self.pred_conc_a < 0:
                dpg.set_value("val_chla", f"{self.pred_conc_a:.4f} (0.00)")
                dpg.configure_item("val_chla", color=(255, 100, 100)) # Red to indicate noise/negative
            else:
                dpg.set_value("val_chla", f"{self.pred_conc_a:.4f}")
                dpg.configure_item("val_chla", color=(0, 255, 0)) # Green for positive
            dpg.configure_item("btn_chla", label="Done (Recollect)")
            dpg.configure_item("btn_chla", user_data=avg_spec) # Store for convenience if needed
        else:
            dpg.configure_item("btn_chla", label="Measure Chl A")

    def cb_predict_chlb(self):
        if self.manual_dark is None:
            log("Please collect Dark Frame first!", "ERROR")
            return
        dpg.configure_item("btn_chlb", label="Measuring...")
        threading.Thread(target=self._collect_frames_thread, args=(10, self._on_chlb_collected), daemon=True).start()

    def _on_chlb_collected(self, avg_spec):
        if avg_spec is not None:
            self.manual_laser_b = avg_spec
            val = self.model_inference.predict_chlb(avg_spec, self.manual_dark, self.integration_time_ms)
            self.pred_conc_b = val
            # Display logic for negative values
            if self.pred_conc_b < 0:
                dpg.set_value("val_chlb", f"{self.pred_conc_b:.4f} (0.00)")
                dpg.configure_item("val_chlb", color=(255, 100, 100))
            else:
                dpg.set_value("val_chlb", f"{self.pred_conc_b:.4f}")
                dpg.configure_item("val_chlb", color=(0, 255, 0))
            dpg.configure_item("btn_chlb", label="Done (Recollect)")
        else:
            dpg.configure_item("btn_chlb", label="Measure Chl B")

    def cb_save_prediction(self):
        if self.manual_dark is None or self.manual_laser_a is None or self.manual_laser_b is None:
            log("Missing data (Dark, Laser A, or Laser B)", "ERROR")
            return
            
        note = dpg.get_value("input_sample_name")
        self.data_mgr.add_sample(
            self.manual_dark, 
            self.manual_laser_a, 
            self.manual_laser_b, 
            self.pred_conc_a, 
            self.pred_conc_b, 
            note=note,
            integration_time=self.integration_time_ms
        )
        # Advance sample name
        try:
            name_parts = note.rsplit('_', 1)
            if len(name_parts) == 2 and name_parts[1].isdigit():
                num = int(name_parts[1]) + 1
                dpg.set_value("input_sample_name", f"{name_parts[0]}_{num:03d}")
        except: pass
        log(f"Sample '{note}' saved to dataset.")


    def cb_retrain(self):
        dpg.configure_item("btn_retrain", label="Training...")
        threading.Thread(target=self._retrain_thread, daemon=True).start()
        
    def _retrain_thread(self):
        try:
            # Call the training script
            import subprocess
            res = subprocess.run(["python", "ml_model/train_and_save.py"], capture_output=True, text=True)
            log("Training Output:\n" + res.stdout)
            if res.stderr:
                log("Training Error:\n" + res.stderr, "ERROR")
            
            # Reload models
            self.model_inference.load_models()
            log("Models reloaded successfully.")
            dpg.configure_item("btn_retrain", label="Retrain Model with New Data")
        except Exception as e:
            log(f"Retrain failed: {e}", "ERROR")
            dpg.configure_item("btn_retrain", label="Retrain Failed")

    # ================= MAIN LOOP =================

    def update(self):
        # 0. Apply Axis Limits
        dpg.set_axis_limits("y_axis", 0, self.y_max)
        if dpg.does_item_exist("y_axis_volt"):
            dpg.set_axis_limits("y_axis_volt", 0, 3.3 * (self.y_max / 65535.0))
        
        # X Limits
        x_min, x_max = 0, CCD_PIXELS
        if self.calibration.enabled:
            x_min = self.calibration.pixel_to_nm(0)
            x_max = self.calibration.pixel_to_nm(CCD_PIXELS)
        if self.remove_dummies:
            if self.calibration.enabled:
                x_min, x_max = self.calibration.pixel_to_nm(32), self.calibration.pixel_to_nm(3680)
            else:
                x_min, x_max = 32, 3680
        if x_min > x_max: x_min, x_max = x_max, x_min
        dpg.set_axis_limits("x_axis", x_min, x_max)
        
        # 1. Update Live Data
        if self.receiver.frame_ready and not self.receiver.frozen:
            with self.receiver.lock:
                self.receiver.frame_ready = False
                pixels = self.receiver.pixels.copy()
            
            # Setup X array
            if self.calibration.enabled:
                full_x = np.array([self.calibration.pixel_to_nm(i) for i in range(CCD_PIXELS)])
            else:
                full_x = np.arange(CCD_PIXELS)
                
            display_pixels, display_x = pixels, full_x
            if self.remove_dummies:
                start, end = 32, 3680
                display_pixels = pixels[start:end]
                display_x = full_x[start:end]
                
            dpg.set_value("series_live", [display_x.tolist(), display_pixels.tolist()])
            
            # Peaks
            if self.show_peaks:
                px, py = self.peak_detector.find_peaks(display_pixels)
                if len(px) > 0:
                    px_indices = px.astype(int)
                    px_x = display_x[px_indices]
                    dpg.set_value("series_peaks", [px_x.tolist(), py.tolist()])
                else:
                    dpg.set_value("series_peaks", [[], []])
                    
            # Update Status Bar with Flicker logic
            # ... (Simplified for brevity, same as before) ...
            dpg.set_value("status_bar", f"FPS: {self.receiver.fps} | Frame: {self.receiver.frame_count}")

            # Re-enable single shot button if frozen
            if self.receiver.frozen and self.receiver.pending_single_shot == False:
                 pass # Already handled by state

        # 2. Update History Data
        if self.show_history and self.history_data:
            if self.playback_active:
                now = time.time()
                if now - self.playback_last_time > 0.033:
                    self.playback_last_time = now
                    cur = dpg.get_value("slider_hist") + 1
                    if cur >= self.history_data['frames']: cur = 0
                    dpg.set_value("slider_hist", cur)
            
            idx = dpg.get_value("slider_hist")
            h_pixels = self.history_data['pixels'][idx]
            
            hx = full_x if self.calibration.enabled else np.arange(CCD_PIXELS)
            if self.remove_dummies:
                start, end = 32, 3680
                h_pixels = h_pixels[start:end]
                hx = hx[start:end]
                
            dpg.set_value("series_history_line", [hx.tolist(), h_pixels.tolist()])

    def setup_ui(self):
        dpg.create_context()
        
        with dpg.window(tag="main_win", label="CCD Monitor v3"):
            
            # Top Stats
            with dpg.group(horizontal=True):
                dpg.add_text("FPS: 0", tag="status_bar")
                
            with dpg.group(horizontal=True):
                
                # SIDEBAR
                with dpg.child_window(width=self.sidebar_width, tag="sidebar"):
                    with dpg.tab_bar():
                        
                        # CONTROLS TAB
                        with dpg.tab(label="Controls"):
                            # Connection
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
                            dpg.add_combo(["Standard (15-100ms)", "Extended (100-1000ms)", "Custom"], 
                                         tag="combo_integration_range", default_value="Extended (100-1000ms)", 
                                         callback=self.cb_integration_range_changed, width=-1)
                            dpg.add_slider_int(label="##integration", tag="slider_integration",
                                              default_value=self.integration_time_ms, min_value=100, max_value=1000, width=-60)
                            dpg.add_input_int(label="##custom_int", tag="input_custom_integration",
                                            default_value=self.integration_time_ms, min_value=15, width=-60, show=False)
                            dpg.add_button(label="Apply", callback=self.cb_apply_integration_time, width=-1)
                            
                            dpg.add_separator()
                            dpg.add_text("Signal Processing")
                            dpg.add_slider_int(label="Avg Frames", default_value=self.receiver.frame_avg_count, min_value=1, max_value=20,
                                              callback=lambda s,a: [setattr(self.receiver, 'frame_avg_count', a), self.save_settings()])
                            
                            dpg.add_checkbox(label="Enable SavGol Smoothing", default_value=self.peak_detector.use_smoothing,
                                            callback=lambda s,a: [setattr(self.peak_detector, 'use_smoothing', a), self.save_settings()])
                            dpg.add_slider_int(label="Smooth Window", default_value=self.peak_detector.smooth_window, min_value=3, max_value=51,
                                            callback=lambda s,a: [setattr(self.peak_detector, 'smooth_window', a), self.save_settings()])
                            
                            dpg.add_separator()
                            dpg.add_text("Display Settings")
                            dpg.add_checkbox(label="Hide Dummy Pixels", default_value=self.remove_dummies,
                                            callback=lambda s,a: [setattr(self, 'remove_dummies', a), self.save_settings()])
                            dpg.add_slider_int(label="Y Max Scale", default_value=self.y_max, min_value=1000, max_value=65535,
                                            callback=lambda s,a: [setattr(self, 'y_max', a), self.save_settings()])
                            
                            dpg.add_separator()
                            dpg.add_text("Recording")
                            dpg.add_combo([], tag="combo_project", callback=self.cb_change_project, width=-1)
                            dpg.add_button(label="New Project", callback=lambda: dpg.configure_item("proj_win", show=True))
                            dpg.add_button(label="Record", tag="btn_rec", callback=self.cb_record_toggle, width=-1)
                            


                        # HISTORY TAB
                        with dpg.tab(label="History"):
                             dpg.add_listbox([], tag="lb_history", num_items=10, callback=self.cb_load_history, width=-1)
                             dpg.add_checkbox(label="Show Overlay", tag="chk_history", default_value=False,
                                             callback=lambda s,a: setattr(self, 'show_history', a))
                             
                             with dpg.group(horizontal=True):
                                 dpg.add_button(label="<", callback=lambda: self.cb_playback_step(-1))
                                 dpg.add_button(label="Play", tag="btn_play", callback=self.cb_playback_toggle)
                                 dpg.add_button(label=">", callback=lambda: self.cb_playback_step(1))
                                 
                             dpg.add_slider_int(tag="slider_hist", default_value=0, max_value=100, width=-1)
                        # LOGS TAB
                        with dpg.tab(label="Logs"):
                             with dpg.child_window(horizontal_scrollbar=True, height=-40):
                                 dpg.add_listbox([], tag="log_list", num_items=25, width=2000)
                             from utils import LogManager
                             dpg.add_button(label="Clear Logs", callback=lambda: [LogManager.get().logs.clear(), LogManager.get().refresh_ui()], width=-1)
                             
                # SPLITTER
                dpg.add_button(label="", width=4, tag="sidebar_splitter")
                with dpg.theme(tag="splitter_theme"):
                    with dpg.theme_component(dpg.mvButton):
                        dpg.add_theme_color(dpg.mvThemeCol_Button, (60, 60, 60, 255))
                        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (100, 100, 100, 255))
                        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (150, 150, 150, 255))
                dpg.bind_item_theme("sidebar_splitter", "splitter_theme")
                
                with dpg.item_handler_registry(tag="splitter_handler"):
                    dpg.add_item_active_handler(callback=self.cb_splitter_resize)
                    dpg.add_item_deactivated_handler(callback=self.cb_splitter_reset)
                dpg.bind_item_handler_registry("sidebar_splitter", "splitter_handler")

                # MAIN PLOT AREA
                with dpg.child_window(tag="plot_area"):
                    
                     with dpg.tab_bar(tag="main_tabs"):
                         with dpg.tab(label="Live Monitor"):
                             with dpg.plot(label="Spectral Data", height=-1, width=-1, anti_aliased=True):
                                 dpg.add_plot_legend()
                                 dpg.add_plot_axis(dpg.mvXAxis, label="Pixel / Wavelength", tag="x_axis")
                                 dpg.add_plot_axis(dpg.mvYAxis, label="Intensity", tag="y_axis")
                                 dpg.add_plot_axis(dpg.mvYAxis, label="Voltage (V)", tag="y_axis_volt")
                                 
                                 dpg.add_line_series([], [], label="Live", parent="y_axis", tag="series_live")
                                 dpg.add_scatter_series([], [], label="Peaks", parent="y_axis", tag="series_peaks")
                                 dpg.add_line_series([], [], label="History", parent="y_axis", tag="series_history_line", show=False)
                                 
                         with dpg.tab(label="Manual Prediction"):
                             with dpg.group():
                                 dpg.add_text("Manual Concentration Measurement Workflow", color=(0, 255, 255))
                                 dpg.add_spacer(height=10)
                                 
                                 dpg.add_text("Session Info")
                                 dpg.add_input_text(label="Sample ID", tag="input_sample_name", default_value="Sample_001")
                                 dpg.add_spacer(height=10)
                                 
                                 with dpg.collapsing_header(label="Step 1: Dark Correction", default_open=True):
                                     dpg.add_text("Ensure BOTH lasers are OFF.")
                                     with dpg.group(horizontal=True):
                                         dpg.add_button(label="Collect Dark Frame (10 avg)", tag="btn_dark", callback=self.cb_collect_dark, width=200, height=40)
                                         dpg.add_text("Status: None", tag="lbl_dark_status", color=(255, 100, 100))
                                 
                                 dpg.add_spacer(height=10)
                                 
                                 with dpg.collapsing_header(label="Step 2: Chlorophyll A (405nm)", default_open=True):
                                     dpg.add_text("Turn ON the 405nm Laser.")
                                     with dpg.group(horizontal=True):
                                         dpg.add_button(label="Measure Chl A", tag="btn_chla", callback=self.cb_predict_chla, width=200, height=40)
                                         with dpg.group():
                                             dpg.add_text("Concentration:")
                                             dpg.add_text("0.0000", tag="val_chla", color=(0, 255, 0))
                                             dpg.add_text("mg/L")

                                 dpg.add_spacer(height=10)

                                 with dpg.collapsing_header(label="Step 3: Chlorophyll B (650nm)", default_open=True):
                                     dpg.add_text("Turn ON the 650nm Laser.")
                                     with dpg.group(horizontal=True):
                                         dpg.add_button(label="Measure Chl B", tag="btn_chlb", callback=self.cb_predict_chlb, width=200, height=40)
                                         with dpg.group():
                                             dpg.add_text("Concentration:")
                                             dpg.add_text("0.0000", tag="val_chlb", color=(0, 255, 0))
                                             dpg.add_text("mg/L")
                                             
                                 dpg.add_spacer(height=20)
                                 dpg.add_separator()
                                 dpg.add_button(label="SAVE MEASUREMENT TO DATASET", callback=self.cb_save_prediction, width=-1, height=50)
                                 dpg.add_spacer(height=10)
                                 dpg.add_button(label="Retrain Model with New Data", tag="btn_retrain", callback=self.cb_retrain, width=-1)





            # New Project Window (Modal)
            with dpg.window(label="New Project", modal=True, show=False, tag="proj_win", width=300, height=100):
                 dpg.add_input_text(label="Name", tag="new_proj_name")
                 dpg.add_button(label="Create", callback=self.cb_create_project)
                 dpg.add_button(label="Cancel", callback=lambda: dpg.configure_item("proj_win", show=False))

        dpg.create_viewport(title='CCD Monitor v3.0 - Modular', width=1280, height=800)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main_win", True)
        
    def cb_splitter_reset(self):
        # Clear the base width when we stop dragging
        self._drag_start_width = None

    def cb_splitter_resize(self):
        # Store width at the exact moment the drag starts
        if not hasattr(self, "_drag_start_width") or self._drag_start_width is None:
            self._drag_start_width = dpg.get_item_width("sidebar")
            
        # Get total delta since drag started
        dx = dpg.get_mouse_drag_delta()[0]
        
        # Calculate new width based on start width + total delta
        new_width = self._drag_start_width + dx
        if 150 < new_width < 1000:
            dpg.configure_item("sidebar", width=new_width)

    def run(self):
        while dpg.is_dearpygui_running():
            self.update()
            dpg.render_dearpygui_frame()
        self.receiver.running = False
        self.receiver.disconnect()
        dpg.destroy_context()
