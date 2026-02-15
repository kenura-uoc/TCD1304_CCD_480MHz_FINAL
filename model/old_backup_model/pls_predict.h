/*
 * pls_predict.h
 * PLS Regression prediction engine for STM32H7
 * 
 * Performs: prediction = sum((x[i] - mean[i]) / std[i] * coef[i]) + intercept
 */

#ifndef PLS_PREDICT_H
#define PLS_PREDICT_H

/*
 * Predict chlorophyll-a concentration using PLS regression.
 * 
 * Input must be a fully preprocessed spectrum (after SG smooth,
 * 1st derivative, and SNV normalization).
 * 
 * Applies sklearn-compatible scaling and dot product:
 *   scaled[i] = (input[i] - x_mean[i]) / x_std[i]
 *   prediction = sum(scaled[i] * coef[i]) + intercept
 *
 * @param preprocessed_spectrum  Preprocessed spectrum (NUM_FEATURES floats)
 * @return                       Predicted chlorophyll-a concentration (ug/L)
 */
float pls_predict(const float *preprocessed_spectrum);

#endif /* PLS_PREDICT_H */
