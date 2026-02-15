"""
host_predict.py
Python host script to send spectrum data to STM32H7 and get predictions.

Usage:
    python host_predict.py --port COM3 --csv "C:\\path\\to\\spectrum.csv" --int_time 250

    OR for batch prediction:
    python host_predict.py --port COM3 --dir "C:\\path\\to\\spectra\\" --int_time 250
"""

import argparse
import serial
import time
import numpy as np
import pandas as pd
from pathlib import Path


def load_and_average_spectrum(csv_path):
    """Load a CSV file and average all frames (same as Python model)."""
    df = pd.read_csv(csv_path)
    df = df.iloc[:, 1:]  # Skip first column (frame index)
    data = df.values.astype(float)
    return np.mean(data, axis=0)


def send_predict_text(ser, spectrum, integration_time, timeout=30):
    """
    Send spectrum to STM32 in TEXT mode and get prediction.
    
    Protocol:
      -> "PREDICT\r\n"
      <- "READY:INT_TIME\r\n"
      -> "<integration_time>\r\n"
      <- "READY:SPECTRUM\r\n"
      -> 3694 lines of float values
      <- "RESULT:<concentration> ug/L\r\n"
    """
    # Send PREDICT command
    ser.write(b"PREDICT\r\n")
    
    # Wait for READY:INT_TIME
    response = ser.readline().decode().strip()
    if "READY:INT_TIME" not in response:
        print(f"ERROR: Expected READY:INT_TIME, got: {response}")
        return None
    
    # Send integration time
    ser.write(f"{integration_time}\r\n".encode())
    
    # Wait for READY:SPECTRUM
    response = ser.readline().decode().strip()
    if "READY:SPECTRUM" not in response:
        print(f"ERROR: Expected READY:SPECTRUM, got: {response}")
        return None
    
    # Send spectrum values (one per line)
    print(f"  Sending {len(spectrum)} values...")
    for i, val in enumerate(spectrum):
        ser.write(f"{val:.6f}\r\n".encode())
        if (i + 1) % 500 == 0:
            print(f"    Sent {i+1}/{len(spectrum)}")
    
    # Wait for result
    response = ser.readline().decode().strip()
    if response.startswith("RESULT:"):
        conc_str = response.replace("RESULT:", "").replace("ug/L", "").strip()
        concentration = float(conc_str)
        return concentration
    else:
        print(f"ERROR: Unexpected response: {response}")
        return None


def send_predict_binary(ser, spectrum, integration_time, timeout=30):
    """
    Send spectrum to STM32 in BINARY mode (much faster).
    
    Protocol:
      -> 'B' + 4 bytes float(int_time) + 3694*4 bytes float(spectrum)
      <- 4 bytes float(concentration)
    """
    # Send binary header
    ser.write(b'B')
    
    # Send integration time as float32
    ser.write(np.float32(integration_time).tobytes())
    
    # Send spectrum as float32 array
    spectrum_f32 = np.array(spectrum, dtype=np.float32)
    ser.write(spectrum_f32.tobytes())
    
    # Read 4-byte float result
    result_bytes = ser.read(4)
    if len(result_bytes) == 4:
        concentration = np.frombuffer(result_bytes, dtype=np.float32)[0]
        return float(concentration)
    else:
        # Try reading text error
        error = ser.readline().decode().strip()
        print(f"ERROR: {error}")
        return None


def send_background(ser, bg_spectrum):
    """Send background spectrum to STM32."""
    ser.write(b"BG\r\n")
    
    response = ser.readline().decode().strip()
    if "READY:BG_SPECTRUM" not in response:
        print(f"ERROR: Expected READY:BG_SPECTRUM, got: {response}")
        return False
    
    print(f"  Sending background ({len(bg_spectrum)} values)...")
    for val in bg_spectrum:
        ser.write(f"{val:.6f}\r\n".encode())
    
    response = ser.readline().decode().strip()
    if "OK:BG_LOADED" in response:
        print("  Background loaded successfully!")
        return True
    else:
        print(f"ERROR: {response}")
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Send spectrum data to STM32H7 Chla Predictor")
    parser.add_argument("--port", required=True, 
                        help="Serial port (e.g., COM3)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--csv", type=str, 
                        help="Path to spectrum CSV file")
    parser.add_argument("--bg", type=str, 
                        help="Path to background CSV file")
    parser.add_argument("--int_time", type=float, required=True,
                        help="Integration time in ms")
    parser.add_argument("--binary", action="store_true",
                        help="Use binary mode (faster)")
    parser.add_argument("--status", action="store_true",
                        help="Print STM32 status")
    args = parser.parse_args()
    
    # Open serial connection
    print(f"Connecting to {args.port} at {args.baud} baud...")
    ser = serial.Serial(args.port, args.baud, timeout=30)
    time.sleep(2)  # Wait for STM32 reset
    
    # Flush any startup messages
    while ser.in_waiting:
        print(f"  STM32: {ser.readline().decode().strip()}")
    
    # Status check
    if args.status:
        ser.write(b"STATUS\r\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode().strip())
        ser.close()
        return
    
    # Load background if provided
    if args.bg:
        print(f"\nLoading background: {args.bg}")
        bg_spectrum = load_and_average_spectrum(args.bg)
        send_background(ser, bg_spectrum)
    
    # Load and predict
    if args.csv:
        print(f"\nLoading spectrum: {args.csv}")
        spectrum = load_and_average_spectrum(args.csv)
        print(f"  Spectrum length: {len(spectrum)}")
        print(f"  Integration time: {args.int_time} ms")
        
        send_fn = send_predict_binary if args.binary else send_predict_text
        concentration = send_fn(ser, spectrum, args.int_time)
        
        if concentration is not None:
            print(f"\n  *** Predicted Concentration: {concentration:.4f} ug/L ***")
        else:
            print("\n  Prediction failed!")
    
    ser.close()
    print("\nDone.")


if __name__ == "__main__":
    main()
