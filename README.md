
# TCD1304 CCD 480MHz Monitor

This project implements a high-speed CCD monitor using the Toshiba **TCD1304** linear CCD sensor and an **STM32H743** microcontroller. It includes a custom Python application for data visualization, recording, and real-time machine learning inference for chlorophyll concentration analysis.

## Overview

The system consists of two main components:
1.  **Firmware (STM32)**: Handles high-speed CCD readout (480MHz timer clock), signal processing (background subtraction), and USB communication.
2.  **Software (Python App)**: A GUI application for controlling the device, visualizing spectra, and running ML models.

## 1. STM32 Firmware (`Core/`)

### Setup & Flashing
1.  Open the project in **STM32CubeIDE**.
2.  Ensure the correct MCU is selected (**STM32H743VIT6**).
3.  Build the project (Hammer icon).
4.  Connect the device via ST-Link.
5.  Run/Debug to flash the firmware.

### Device Operation
The device features an on-board LCD and button interface:
-   **Main Screen**: Displays live status (Integration Time, Laser State).
-   **Menu Navigation**: Use the encoder/buttons to navigate.
-   **Auto Exposure**: Automatically adjusts integration time based on signal intensity.
-   **Manual Mode**: Allows manual setting of integration time (10µs - 1s).

**Key Pins**:
-   **CCD Data**: PA0 (ADC1_INP16)
-   **CCD Clocks**: PA1 (fM), PA2 (SH), PA3 (ICG)
-   **Laser Control**: PD14, PD15
-   **USB**: PA11 (DM), PA12 (DP)

## 2. Python Application (`ccd_app/`)

### Installation
The application uses `uv` for dependency management.

1.  Navigate to the app directory:
    ```bash
    cd ccd_app
    ```
2.  Initialize and sync dependencies:
    ```bash
    uv sync
    ```

### Running the App
Start the application with:
```bash
uv run app.py
```

### User Interface Guide

#### **Controls Tab**
-   **Connection**: Select the COM port and click **Connect**.
-   **Acquisition**:
    -   **Run/Freeze**: Start or stop live data updates.
    -   **Integration Time**: Select a range (Standard/Extended) and use the slider to adjust exposure time. Click **Apply** to send to device.
-   **Signal Processing**:
    -   **Avg Frames**: Number of frames to average (reduces noise).
    -   **Smoothing**: Enable Savitzky-Golay filter.
-   **Display Settings**:
    -   **Y Max Scale**: Adjust vertical axis limit.
    -   **Hide Dummy Pixels**: Mask the non-sensitive pixels at the start/end of the sensor.

#### **Logs Tab**
-   Displays system events, model loading status, and errors.
-   Use the **Clear Logs** button to reset the view.

#### **History Tab**
-   **Recording**: Start/Stop recording data to `.npz` files.
-   **Playback**: Load previous sessions and replay them frame-by-frame.

#### **Manual Prediction Tab**
A step-by-step workflow for measuring chlorophyll concentration:
1.  **Dark Frame**: Ensure lasers are off and capture a background reference.
2.  **Measure Chl A**: Turn on the 405nm laser and click Measure.
3.  **Measure Chl B**: Turn on the 450nm laser and click Measure.
4.  **Save**: Add the measurement (spectra + concentration) to the dataset for retraining.

## 3. Machine Learning (`ml_model/`)

The system uses PLS and SVR models to predict concentrations.
-   **Data**: Located in `ml_model/data/`.
-   **Models**: `.pkl` files are in `ml_model/models/`.
-   **Training**: Run `train_and_save.py` to retrain models with new data.

For more details on the ML pipeline, see [`ml_model/README.md`](ml_model/README.md).
