/*
 * chl_predictor.h — Chlorophyll concentration predictor for STM32
 *
 * Provides on-device inference for Chl-a (PLS) and Chl-b (PCA+SVR)
 * from raw TCD1304 CCD spectra.
 *
 * Pipeline: ADC → float → bg_sub → int_time_norm → SG_smooth →
 *           SG_deriv → SNV → ROI_crop → ML_predict
 */

#ifndef CHL_PREDICTOR_H
#define CHL_PREDICTOR_H

#include "chl_model_data.h"
#include <stdint.h>


/* Status codes */
typedef enum {
  CHL_OK = 0,
  CHL_ERR_NULL_PTR,
  CHL_ERR_INVALID_TIME
} chl_status_t;

/*
 * Working memory for the predictor.
 * Total RAM: ~44 KB (3 float buffers of FULL_SPECTRUM_LEN)
 * Place in a large RAM section (e.g., AXI SRAM on STM32H7).
 */
typedef struct {
  float buf_a[FULL_SPECTRUM_LEN]; /* Working buffer A (~14.8 KB) */
  float buf_b[FULL_SPECTRUM_LEN]; /* Working buffer B (~14.8 KB) */
  float roi[NUM_FEATURES];        /* ROI-cropped spectrum (~7.6 KB) */
  float pca_out[CHLB_N_PCA];      /* PCA projection output */
} chl_predictor_t;

/**
 * Predict Chl-a concentration from a pre-averaged float spectrum.
 *
 * @param pred             Predictor workspace (must be allocated by caller)
 * @param avg_spectrum     Averaged spectrum (FULL_SPECTRUM_LEN floats)
 * @param integration_ms   Integration time in milliseconds (> 0)
 * @param concentration    Output: predicted Chl-a in mg/L
 * @return CHL_OK on success
 */
chl_status_t chl_predict_chla(chl_predictor_t *pred, const float *avg_spectrum,
                              float integration_ms, float *concentration);

/**
 * Predict Chl-b concentration from a pre-averaged float spectrum.
 *
 * @param pred             Predictor workspace
 * @param avg_spectrum     Averaged spectrum (FULL_SPECTRUM_LEN floats)
 * @param integration_ms   Integration time in milliseconds (> 0)
 * @param concentration    Output: predicted Chl-b in mg/L
 * @return CHL_OK on success
 */
chl_status_t chl_predict_chlb(chl_predictor_t *pred, const float *avg_spectrum,
                              float integration_ms, float *concentration);

#endif /* CHL_PREDICTOR_H */
