/*
 * chla_predictor.c
 * Chlorophyll-a concentration predictor for STM32H7 + TCD1304 CCD
 *
 * Pipeline: uint16_t ADC -> float -> bg subtract -> int_time normalize
 *           -> SG smooth -> 1st deriv -> SNV -> PLS predict
 */

#include "chla_predictor.h"
#include "pls_predict.h"
#include "pls_model_data.h"
#include <string.h>

/* --------------------------------------------------------------- */
chla_status_t chla_predictor_init(chla_predictor_t *pred)
{
    if (pred == NULL) {
        return CHLA_ERR_INVALID_INPUT;
    }

    memset(pred, 0, sizeof(chla_predictor_t));
    pred->initialized = 1;
    pred->background_loaded = 0;

    return CHLA_OK;
}

/* --------------------------------------------------------------- */
chla_status_t chla_set_background_u16(chla_predictor_t *pred,
                                       const uint16_t *adc_buf)
{
    int i;

    if (pred == NULL || adc_buf == NULL) {
        return CHLA_ERR_INVALID_INPUT;
    }
    if (!pred->initialized) {
        return CHLA_ERR_NOT_INITIALIZED;
    }

    /* Convert uint16_t ADC values to float */
    for (i = 0; i < SPECTRUM_LENGTH; i++) {
        pred->background[i] = (float)adc_buf[i];
    }
    pred->background_loaded = 1;

    return CHLA_OK;
}

/* --------------------------------------------------------------- */
chla_status_t chla_predict_from_adc(chla_predictor_t *pred,
                                     const uint16_t *adc_buf,
                                     float integration_time,
                                     float *concentration)
{
    int i;
    float inv_int_time;

    if (pred == NULL || adc_buf == NULL || concentration == NULL) {
        return CHLA_ERR_INVALID_INPUT;
    }
    if (!pred->initialized) {
        return CHLA_ERR_NOT_INITIALIZED;
    }

    /* ----- Step 1: Convert uint16_t ADC -> float ----- */
    for (i = 0; i < SPECTRUM_LENGTH; i++) {
        pred->work_buf1[i] = (float)adc_buf[i];
    }

    /* ----- Step 2: Background subtraction ----- */
    if (pred->background_loaded) {
        for (i = 0; i < SPECTRUM_LENGTH; i++) {
            pred->work_buf1[i] -= pred->background[i];
        }
    }

    /* ----- Step 3: Integration time normalization ----- */
    /*  spectrum = spectrum / (integration_time + 1e-8)  */
    inv_int_time = 1.0f / (integration_time + 1e-8f);
    for (i = 0; i < SPECTRUM_LENGTH; i++) {
        pred->work_buf1[i] *= inv_int_time;
    }

    /* ----- Step 4: Preprocessing ----- */
    /* SG smooth: work_buf1 (input) -> work_buf2 (smoothed) */
    sg_smooth(pred->work_buf1, pred->work_buf2, SPECTRUM_LENGTH);
    /* SG 1st derivative: work_buf2 (smoothed) -> work_buf1 (derivative) */
    sg_first_derivative(pred->work_buf2, pred->work_buf1, SPECTRUM_LENGTH);
    /* SNV normalize in-place on work_buf1 */
    snv_normalize(pred->work_buf1, SPECTRUM_LENGTH);

    /* ----- Step 5: PLS Prediction ----- */
    *concentration = pls_predict(pred->work_buf1);

    return CHLA_OK;
}

/* --------------------------------------------------------------- */
chla_status_t chla_predict_preprocessed(const float *preprocessed,
                                         float *concentration)
{
    if (preprocessed == NULL || concentration == NULL) {
        return CHLA_ERR_INVALID_INPUT;
    }

    *concentration = pls_predict(preprocessed);

    return CHLA_OK;
}
