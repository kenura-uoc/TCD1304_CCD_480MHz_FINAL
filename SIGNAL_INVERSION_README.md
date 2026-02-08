# TCD1304 Signal Inversion Implementation Plan

## Overview

The TCD1304 CCD outputs an **inverted signal** (dark = high voltage, light = low voltage).
For intuitive display and ML training, we invert the signal so that **light = high values**.

## Current Implementation

| Location | Status | Description |
|----------|--------|-------------|
| `main.py` | ✅ Active | `pixels = 65535 - pixels` in Python |
| `main.c` | ❌ Not yet | Planned firmware-level inversion |

## Inversion Math

```
Raw ADC value:  0 to 65535 (16-bit)
Inverted:       65535 - raw_value

Example:
  Dark (raw ~35000)  → Inverted: 30535 (low baseline)
  Light (raw ~20000) → Inverted: 45535 (high peak)
```

**No underflow/overflow risk** - result always within uint16 range.

---

## Firmware Implementation (When Ready to Flash)

### File: `Core/Src/main.c`

**Option A: Invert in ADC callback**
```c
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  if (hadc->Instance == ADC1) {
    ccd_frame.magic = 0xABCD;
    ccd_frame.frame_num = frame_counter++;
    
    // Invert signal during copy
    for (int i = 0; i < CCD_BUFFER_SIZE; i++) {
      ccd_frame.pixels[i] = 65535 - Buffer_A[i];
    }
    
    frame_ready = 1;
  }
}
```

**Option B: Invert in send function**
```c
void Send_CCD_Frame_Binary(void) {
  // Invert before sending
  for (int i = 0; i < CCD_BUFFER_SIZE; i++) {
    ccd_frame.pixels[i] = 65535 - ccd_frame.pixels[i];
  }
  // ... rest of send logic
}
```

### After Firmware Update

Remove Python inversion in `main.py`:
```python
# DELETE this line after firmware update:
# pixels = 65535 - pixels
```

---

## Recording Data (.npz Files)

| Current Behavior | Data Format |
|-----------------|-------------|
| Recording | Raw (non-inverted) from STM32 |
| Display | Inverted in Python |

**For ML Training**: Raw data is fine - invert during preprocessing if needed.

---

## Integration Time Control

Added USB command: `I<time_ms>` (e.g., "I25" for 25ms)

| Parameter | Range | Default |
|-----------|-------|---------|
| Integration Time | 15-100ms | 18ms |

Minimum 15ms required to read all 3694 pixels at 250kHz ADC rate.

---

## Files Modified

- `ccd_monitor/main.py` - UI changes, always-on inversion
- `Core/Src/main.c` - Integration time control (inversion pending)
