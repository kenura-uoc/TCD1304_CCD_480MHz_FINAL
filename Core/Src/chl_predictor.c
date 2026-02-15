/*
 * chl_predictor.c — Chlorophyll concentration predictor
 *
 * Implements on-device preprocessing (SG smooth, derivative, SNV)
 * and ML inference (PLS for Chl-a, PCA+SVR for Chl-b).
 *
 * Adapted from model/old_backup_model/ preprocessing.c + pls_predict.c
 */

#include "chl_predictor.h"
#include <math.h>
#include <string.h>

/* ===== Internal: Savitzky-Golay convolution ===== */

static void apply_sg_filter(const float *input, float *output, int length,
                            const float *coeffs, int window, int half) {
  int i, j;
  /* Interior points: full convolution */
  for (i = half; i < length - half; i++) {
    float sum = 0.0f;
    for (j = 0; j < window; j++) {
      sum += coeffs[j] * input[i - half + j];
    }
    output[i] = sum;
  }
  /* Edge points: copy from input (no distortion) */
  for (i = 0; i < half; i++) {
    output[i] = input[i];
  }
  for (i = length - half; i < length; i++) {
    output[i] = input[i];
  }
}

/* ===== Internal: preprocessing pipeline ===== */

static void preprocess_spectrum(const float *input, float *output, float *work,
                                int length) {
  int i;
  float mean, std, inv_std;

  /* Step 1: SG smoothing */
  apply_sg_filter(input, work, length, SG_SMOOTH_COEFFS, SG_SMOOTH_WINDOW,
                  SG_SMOOTH_HALF);

  /* Step 2: SG first derivative */
  apply_sg_filter(work, output, length, SG_DERIV_COEFFS, SG_DERIV_WINDOW,
                  SG_DERIV_HALF);

  /* Step 3: SNV normalization (zero mean, unit variance) */
  mean = 0.0f;
  for (i = 0; i < length; i++) {
    mean += output[i];
  }
  mean /= (float)length;

  std = 0.0f;
  for (i = 0; i < length; i++) {
    float d = output[i] - mean;
    std += d * d;
  }
  std = sqrtf(std / (float)length);
  inv_std = (std > 1e-8f) ? (1.0f / std) : 1.0f;

  for (i = 0; i < length; i++) {
    output[i] = (output[i] - mean) * inv_std;
  }
}

/* ===== Internal: PLS prediction ===== */

static float pls_predict(const float *features) {
  float prediction = PLS_INTERCEPT;
  int i;
  for (i = 0; i < NUM_FEATURES; i++) {
    float x_std = PLS_X_STD[i];
    float scaled =
        (x_std > 1e-8f) ? (features[i] - PLS_X_MEAN[i]) / x_std : 0.0f;
    prediction += scaled * PLS_COEF[i];
  }
  return prediction;
}

/* ===== Internal: PCA + SVR prediction ===== */

static float pca_svr_predict(const float *features, float *pca_out) {
  int i, j;
  float prediction;

  /* Step 1: StandardScaler → (x - mean) / scale */
  /* We scale in-place conceptually, but use scaled values directly in PCA */

  /* Step 2: PCA projection */
  for (i = 0; i < CHLB_N_PCA; i++) {
    float dot = 0.0f;
    const float *comp = &CHLB_PCA_COMPONENTS[i * NUM_FEATURES];
    for (j = 0; j < NUM_FEATURES; j++) {
      float scaled =
          (features[j] - CHLB_SCALER_MEAN[j]) / (CHLB_SCALER_SCALE[j] + 1e-8f);
      float centered = scaled - CHLB_PCA_MEAN[j];
      dot += comp[j] * centered;
    }
    pca_out[i] = dot;
  }

  /* Step 3: SVR with RBF kernel */
  prediction = CHLB_SVR_INTERCEPT;
  for (i = 0; i < CHLB_N_SV; i++) {
    /* Compute ||pca_out - sv_i||^2 */
    float dist_sq = 0.0f;
    const float *sv = &CHLB_SVR_SUPPORT_VECTORS[i * CHLB_N_PCA];
    for (j = 0; j < CHLB_N_PCA; j++) {
      float d = pca_out[j] - sv[j];
      dist_sq += d * d;
    }
    /* RBF kernel: exp(-gamma * dist^2) */
    float kernel = expf(-CHLB_SVR_GAMMA * dist_sq);
    prediction += CHLB_SVR_DUAL_COEF[i] * kernel;
  }

  return prediction;
}

/* ===== Common preprocessing ===== */

static chl_status_t common_preprocess(chl_predictor_t *pred,
                                      const float *avg_spectrum,
                                      float integration_ms) {
  int i;

  if (!pred || !avg_spectrum)
    return CHL_ERR_NULL_PTR;
  if (integration_ms <= 0.0f)
    return CHL_ERR_INVALID_TIME;

  /* Copy spectrum to buf_a and normalize by integration time */
  for (i = 0; i < FULL_SPECTRUM_LEN; i++) {
    pred->buf_a[i] = avg_spectrum[i] / integration_ms;
  }

  /* Run preprocessing: SG smooth → derivative → SNV */
  /* Output in buf_a, uses buf_b as work buffer */
  preprocess_spectrum(pred->buf_a, pred->buf_b, pred->buf_a, FULL_SPECTRUM_LEN);

  /* Note: after preprocess_spectrum, the result is in buf_b (output param) */

  /* ROI crop: pixels ROI_START to ROI_END */
  for (i = 0; i < NUM_FEATURES; i++) {
    pred->roi[i] = pred->buf_b[ROI_START + i];
  }

  return CHL_OK;
}

/* ===== Public API ===== */

chl_status_t chl_predict_chla(chl_predictor_t *pred, const float *avg_spectrum,
                              float integration_ms, float *concentration) {
  chl_status_t status;

  if (!concentration)
    return CHL_ERR_NULL_PTR;

  status = common_preprocess(pred, avg_spectrum, integration_ms);
  if (status != CHL_OK)
    return status;

  *concentration = pls_predict(pred->roi);
  return CHL_OK;
}

chl_status_t chl_predict_chlb(chl_predictor_t *pred, const float *avg_spectrum,
                              float integration_ms, float *concentration) {
  chl_status_t status;

  if (!concentration)
    return CHL_ERR_NULL_PTR;

  status = common_preprocess(pred, avg_spectrum, integration_ms);
  if (status != CHL_OK)
    return status;

  *concentration = pca_svr_predict(pred->roi, pred->pca_out);
  return CHL_OK;
}
