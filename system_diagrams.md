# System Architecture Diagrams

This document contains PlantUML diagrams for the TCD1304 Spectrometer project. You can copy the code blocks below and paste them into any PlantUML viewer (like [PlantText](https://www.planttext.com/)) or use the Markdown Kroki extension to view them.

## 1. Hardware Block Diagram

This diagram shows the physical connections between the STM32H7 microcontroller and the various peripherals (CCD Sensor, Lasers, Display, Storage).


```plantuml
@startuml
skinparam componentStyle rectangle

package "Power Supply" {
    [5V Input] --> [3.3V LDO]
    [5V Input] --> [Lasers]
}

package "Optical Unit" {
    [TCD1304 CCD Sensor] as CCD
    [Laser 405nm] as L1
    [Laser 450nm] as L2
    [Servo Motor (Shutter/Switch)] as Servo
}

package "User Interface" {
    [I2C LCD (16x2)] as LCD
    [Buttons (Up/Down/Ok)] as Btns
    [Potentiometers] as Pots
}

package "Communication & Storage" {
    [USB-C (PC Data)] as USB
    [SD Card (SDMMC)] as SD
}

node "STM32H743VIT6 Core" as MCU {
    [ADC1 (DMA)] as ADC
    [TIM4 (CCD Clock)] as TimCCD
    [TIM3/5 (PWM)] as PWM
    [I2C1]
    [SDMMC1]
    [USB OTG FS]
}

' Connections
CCD --> ADC : Analog Out (OS)
MCU --> CCD : Clocks (ICG, SH, MCLK)
L1 <-- PWM : PWM Control
L2 <-- PWM : PWM Control
Servo <-- PWM : PWM Control

LCD <-- [I2C1] : I2C Bus
Btns --> MCU : GPIO
Pots --> [ADC3] : Analog In

MCU <--> SD : SDMMC Bus
MCU <--> USB : Virtual COM Port

@enduml
```

## 2. Firmware Architecture

This diagram illustrates the organization of the C code, specifically the `main` loop, the `menu` system helper, and the machine learning predictor.

```plantuml
@startuml
allow_mixing

package "Core Application" {
    class "Main Loop" as Main {
        + SystemClock_Config()
        + MX_ADC_Init()
        + MX_TIM_Init()
        + while(1) Loop
        --
        Controls Signal Acquisition
        Calls Menu_Update()
    }

    class "Menu System" as Menu {
        + Menu_Update()
        + Render_Screen()
        --
        Handles Button Inputs
        Manages UI State Machine
        Controls Auto-Measurement
        Controls Auto-Exposure
    }
}

package "Signal Processing" {
    class "CCD Driver" as CCD {
        + Buffer_A[3694]
        + readCCD()
        + Global Inversion (65535-val)
    }

    class "Predictor (ML)" as ML {
        + chl_predict_chla()
        + chl_predict_chlb()
        --
        Preprocessing (SG Filter)
        PLS / SVR Models
    }
}

package "Storage & IO" {
    class "SD Storage" as Storage {
        + SD_SaveMeasResult()
        + SD_SaveFrame()
    }
    
    class "USB Comm" as USB {
        + CDC_Transmit_FS()
        + Process_USB_Command()
    }
}

Main *-- CCD : Drives
Main --> Menu : Updates
Menu --> ML : Calls for Prediction
Menu --> Storage : Saves Data
Main --> USB : Sends Raw Data
@enduml
```

## 3. Auto-Measurement State Machine

This diagram details the logic flow within the "Auto Measure" feature in `menu.c`, showing how the device switches lasers, captures frames, and ensures valid predictions.

```plantuml
@startuml
[*] --> AUTO_IDLE

AUTO_IDLE --> AUTO_MOVE_LASER1 : User Press OK
AUTO_MOVE_LASER1 : Set Servo Position 1
AUTO_MOVE_LASER1 : Turn ON Laser 405nm
AUTO_MOVE_LASER1 --> AUTO_CAPTURE_LASER1 : Wait Servo Time

state AUTO_CAPTURE_LASER1 {
    [*] --> Capture_L1
    Capture_L1 : Accumulate Frames
    Capture_L1 --> Capture_L1 : Count < Target
    Capture_L1 --> Predict_L1 : Count >= Target
    Predict_L1 : Run ML Model (Chl-A)
    Predict_L1 : Check Errors (-1/-2)
}

AUTO_CAPTURE_LASER1 --> AUTO_MOVE_LASER2 : Done
AUTO_MOVE_LASER2 : Set Servo Position 2
AUTO_MOVE_LASER2 : Turn OFF L1, Turn ON L2
AUTO_MOVE_LASER2 --> AUTO_CAPTURE_LASER2 : Wait Servo Time

state AUTO_CAPTURE_LASER2 {
    [*] --> Capture_L2
    Capture_L2 : Accumulate Frames
    Capture_L2 --> Capture_L2 : Count < Target
    Capture_L2 --> Predict_L2 : Count >= Target
    Predict_L2 : Run ML Model (Chl-B)
    Predict_L2 : Check Errors
}

AUTO_CAPTURE_LASER2 --> AUTO_SAVING : Done
AUTO_SAVING : Write to SD Card
AUTO_SAVING : Save to Backup SRAM
AUTO_SAVING --> AUTO_COMPLETE

AUTO_COMPLETE : Display A & B Values
AUTO_COMPLETE --> AUTO_IDLE : User Press OK

@enduml
```

## 4. Auto-Exposure State Machine (NEW)

This diagram details the new Auto-Exposure logic which optimizes integration time based on peak signal intensity.

```plantuml
@startuml
[*] --> AUTOEXP_IDLE

AUTOEXP_IDLE --> AUTOEXP_MOVE_LASER1 : User Press OK

state "Tuning Laser 1" as TuningL1 {
    AUTOEXP_MOVE_LASER1 --> AUTOEXP_CAPTURE_LASER1
    
    state AUTOEXP_CAPTURE_LASER1 {
        [*] --> Capture_Frame
        Capture_Frame --> Median_Filter : 3-point
        Median_Filter --> Check_Peak
        Check_Peak --> Integration_Update : Proportional P-Control
        Integration_Update --> Capture_Frame : Not Converged
        Integration_Update --> [*] : Converged / Max Iter
    }
}

TuningL1 --> AUTOEXP_MOVE_LASER2 : Done (Save Result A)

state "Tuning Laser 2" as TuningL2 {
    AUTOEXP_MOVE_LASER2 --> AUTOEXP_CAPTURE_LASER2
    
    state AUTOEXP_CAPTURE_LASER2 {
        [*] --> Capture_Frame2
        Capture_Frame2 --> Median_Filter2
        Median_Filter2 --> Check_Peak2
        Check_Peak2 --> Integration_Update2
        Integration_Update2 --> Capture_Frame2 : Not Converged
        Integration_Update2 --> [*] : Converged
    }
}

TuningL2 --> AUTOEXP_DONE : Done (Save Result B)
AUTOEXP_DONE --> AUTOEXP_IDLE : User Press OK
@enduml
```

## 5. Signal Processing Data Flow

This diagram explains how the raw analog signal from the CCD is transformed into the final Chlorophyll concentration value.

```plantuml
@startuml
skinparam cloudBackgroundColor white

partition "Hardware" {
    :CCD Sensor (TCD1304);
    note right: Dark = ~2.5V (High)\nLight = ~0.5V (Low)
    :STM32 ADC (16-bit);
    note right: Raw ADC Values\n(High=Dark)
}

partition "Firmware (Main.c)" {
    :DMA Transfer to Buffer;
    :Global Inversion;
    note right: Val = 65535 - ADC\n(Now High=Light)
}

partition "Firmware (Menu.c)" {
    :Accumulate Frames;
    :Average (Divide by N);
}

partition "ML Predictor (chl_predictor.c)" {
    :Savitzky-Golay Smoothing;
    :First Derivative;
    :Standard Normal Variate (SNV);
    
    split
        :PLS Model (Matrix Dot Product);
        -> Result;
        :Chl-A Value;
    split again
        :PCA Reduction;
        :SVR Model (RBF Kernel);
        -> Result;
        :Chl-B Value;
    end split
}

partition "Output" {
    :Display on LCD;
    :Save to SD Card (CSV/Txt);
}

@enduml
```
