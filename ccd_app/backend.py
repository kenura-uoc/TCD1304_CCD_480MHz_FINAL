import serial
import serial.tools.list_ports
import struct
import numpy as np
import threading
import time
from utils import log

# ==========================================
# CONSTANTS
# ==========================================
CCD_PIXELS = 3694
FRAME_SIZE = CCD_PIXELS * 2 + 4  # 2 magic + 2 frame count + 3694*2 pixels = 7392
BAUD_RATE = 10000000 # 10 Mbps (virtual)

class CCDReceiver:
    def __init__(self):
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
                            
                            # Averaging Logic
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
