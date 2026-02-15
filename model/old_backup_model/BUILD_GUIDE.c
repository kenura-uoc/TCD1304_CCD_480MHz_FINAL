/*
 * STM32H7 Chla Predictor - Build Configuration
 * 
 * This file documents how to integrate the prediction system
 * into your STM32H7 project.
 *
 * ============================================================
 * FILES TO ADD TO YOUR STM32 PROJECT
 * ============================================================
 *
 * Source files (.c):
 *   - preprocessing.c
 *   - pls_predict.c
 *   - chla_predictor.c
 *   - main.c  (or integrate into your existing main.c)
 *
 * Header files (.h):
 *   - preprocessing.h
 *   - pls_predict.h
 *   - chla_predictor.h
 *   - pls_model_data.h  (auto-generated, contains model weights)
 *   - test_vector.h     (optional, for self-test)
 *
 * ============================================================
 * COMPILER FLAGS
 * ============================================================
 *
 * Required:
 *   -DSTM32H7XX          (enables STM32 HAL includes)
 *   -lm                  (link math library for sqrtf)
 *
 * Recommended for performance:
 *   -O2 or -Os           (optimization level)
 *   -mfpu=fpv5-d16       (hardware FPU for float operations)
 *   -mfloat-abi=hard     (hardware float calling convention)
 *
 * ============================================================
 * MEMORY REQUIREMENTS
 * ============================================================
 *
 * Flash (code + const data):
 *   - pls_model_data.h arrays: ~45 KB
 *   - Code (.text):            ~2 KB
 *   - Total:                   ~47 KB
 *
 * RAM (variables + buffers):
 *   - chla_predictor_t:        ~44 KB (3 x 3694 x 4 bytes)
 *   - spectrum_buffer:         ~14.8 KB
 *   - Total:                   ~59 KB
 *
 * ============================================================
 * INTEGRATION WITH YOUR EXISTING PROJECT
 * ============================================================
 *
 * In your existing main.c (e.g., TCD1304_CCD_480MHz project):
 *
 *   1. #include "chla_predictor.h"
 *
 *   2. Declare predictor globally:
 *      static chla_predictor_t predictor;
 *
 *   3. In your init function:
 *      chla_predictor_init(&predictor);
 *
 *   4. When ADC data is ready (DMA complete callback):
 *      float concentration;
 *      chla_predict_from_raw(&predictor, adc_data, integration_time, &concentration);
 *
 *   5. Send result via UART/USB:
 *      printf("Concentration: %.4f ug/L\r\n", concentration);
 *
 * ============================================================
 * TESTING ON PC (without STM32)
 * ============================================================
 *
 * Compile for desktop testing:
 *   gcc -o chla_test preprocessing.c pls_predict.c chla_predictor.c main.c -lm
 *
 * Run:
 *   ./chla_test
 *   > STATUS
 *   > TEST
 *
 */
