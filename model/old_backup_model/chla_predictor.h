/*
 * chla_predictor.h
 * Chlorophyll-a concentration predictor for STM32H7 + TCD1304 CCD
 *
 * Integrates directly with existing CCD firmware:
 *   - Accepts uint16_t ADC buffer (from DMA)
 *   - Background subtraction
 *   - Integration time normalization
 *   - SG preprocessing + PLS prediction
 */

#ifndef CHLA_PREDICTOR_H
#define CHLA_PREDICTOR_H

#include <stdint.h>
#include "preprocessing.h"

/* Status codes */
typedef enum {
    CHLA_OK = 0,
    CHLA_ERR_NOT_INITIALIZED,
    CHLA_ERR_INVALID_INPUT,
    CHLA_ERR_PREDICTION_FAILED
} chla_status_t;

/* Predictor context (holds working buffers) */
typedef struct {
    float work_buf1[SPECTRUM_LENGTH];   /* Working buffer 1 (~14.8 KB) */
    float work_buf2[SPECTRUM_LENGTH];   /* Working buffer 2 (~14.8 KB) */
    float background[SPECTRUM_LENGTH];  /* Stored background spectrum (~14.8 KB) */
    uint8_t initialized;
    uint8_t background_loaded;
} chla_predictor_t;

/*
 * Initialize the predictor.
 * Must be called once before any predictions.
 */
chla_status_t chla_predictor_init(chla_predictor_t *pred);

/*
 * Capture and store a background (dark) spectrum from uint16_t ADC buffer.
 * Call this with a dark measurement (laser off / no sample).
 *
 * @param pred       Pointer to predictor context
 * @param adc_buf    uint16_t ADC buffer from TCD1304 DMA (SPECTRUM_LENGTH values)
 */
chla_status_t chla_set_background_u16(chla_predictor_t *pred,
                                       const uint16_t *adc_buf);

/*
 * Predict chlorophyll-a concentration from raw uint16_t ADC data.
 *
 * Full pipeline:
 *   1. Convert uint16_t -> float
 *   2. Background subtraction (if loaded)
 *   3. Divide by integration time (normalization)
 *   4. SG Smoothing (w=11, p=2)
 *   5. SG 1st Derivative (w=11, p=3)
 *   6. SNV Normalization
 *   7. PLS Prediction
 *
 * @param pred               Predictor context
 * @param adc_buf            uint16_t ADC values from TCD1304 (SPECTRUM_LENGTH)
 * @param integration_time   Integration time in milliseconds
 * @param concentration      Output: predicted concentration (ug/L)
 */
chla_status_t chla_predict_from_adc(chla_predictor_t *pred,
                                     const uint16_t *adc_buf,
                                     float integration_time,
                                     float *concentration);

/*
 * Predict from already-preprocessed float spectrum.
 * Use this if you handle preprocessing externally.
 */
chla_status_t chla_predict_preprocessed(const float *preprocessed,
                                         float *concentration);

#endif /* CHLA_PREDICTOR_H */
