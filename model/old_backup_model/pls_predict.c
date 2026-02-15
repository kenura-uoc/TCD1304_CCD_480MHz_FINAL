/*
 * pls_predict.c
 * PLS Regression prediction engine for STM32H7
 * 
 * This is equivalent to sklearn PLSRegression.predict() with scale=True.
 * The prediction is computed as:
 *   1. Scale: x_scaled[i] = (x[i] - x_mean[i]) / x_std[i]
 *   2. Dot product: result = sum(x_scaled[i] * coef[i]) + intercept
 */

#include "pls_predict.h"
#include "pls_model_data.h"

float pls_predict(const float *preprocessed_spectrum)
{
    float prediction = PLS_INTERCEPT;
    int i;
    
    /*
     * Fused scale + dot product for efficiency:
     * prediction += ((x - mean) / std) * coef
     *            = (x - mean) * (coef / std)
     * 
     * We keep it readable here; the compiler will optimize.
     */
    for (i = 0; i < NUM_FEATURES; i++) {
        float scaled = (preprocessed_spectrum[i] - PLS_X_MEAN[i]) / PLS_X_STD[i];
        prediction += scaled * PLS_COEF[i];
    }
    
    return prediction;
}
