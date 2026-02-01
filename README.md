# TCD1304 Spectrometer (STM32H743)

High-performance linear CCD spectrometer driver using STM32H743VIT6 and TCD1304 sensor.

## Features
- **Acquisition Rate**: 500kHz pixel data rate (2MHz Master Clock).
- **Interface**: USB High-Speed (USB_OTG_FS used in Full Speed mode) Virtual COM Port.
- **Synchronization**: Hardware timer synchronization with One-Shot ADC and DMA for stable jitter-free frames.
- **Exposure Control**: Adjustable integration time (10µs - 1s).

## Hardware Connections

| STM32 Pin | CCD Pin | Function |
|-----------|---------|----------|
| PA2       | SH      | Shutter Gate (Pulse) |
| PA6       | fM      | Master Clock (2MHz) |
| PD15      | -       | ADC Trigger (Internal) |
| PA0       | ICG     | Integration Clear Gate |
| PA3       | OS      | Analog Output (ADC In) |
| PA11/12   | USB     | USB D-/D+ |

**Note**: The signals (especially fM and SH) are 3.3V logic. Ensure CCD power is handled correctly (usually requires 5V supply but 3-5V logic inputs).

## Firmware Configuration
The project is generated using STM32CubeMX. 
**Crucial Manual Configurations (Preserved in `.ioc`):**
- **TIM3 (fM)**: 2MHz frequency (Period = 240-1, Pulse = 120).
- **TIM4 (Sampling)**: 500kHz frequency (Period = 960-1, Pulse = 480).
- **TIM2 (Frame)**: Controls ICG period.
- **ADC1**: Configured in **One-Shot Mode** (NOT Circular) to ensure frame alignment. DMA is restarted manually in TIM2 interrupt.

## Python Application
The `ccd_monitor` folder contains the PC interface.
### Setup
```bash
cd ccd_monitor
uv run main.py
```
### Features
- Real-time plotting.
- Exposure control.
- Frame averaging and background subtraction.
- "Logs" tab for debugging.
- Recording to `.npz` files.

## Developing & Updating
If you regenerate code from CubeMX:
1. Ensure the **CMSIS-DSP** Software Pack is installed/selected if you plan to use on-board FFT.
2. Verify that `main.c` `USER CODE BEGIN` blocks are preserved. The critical logic resides in:
   - `HAL_TIM_PeriodElapsedCallback` (Frame sync)
   - `HAL_ADC_ConvCpltCallback` (Data TX)
