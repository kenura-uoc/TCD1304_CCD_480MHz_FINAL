/*
 * preprocessing.c
 * Spectral preprocessing for TCD1304 CCD data on STM32H7
 * 
 * Implements: SG Smooth -> SG 1st Derivative -> SNV normalization
 * All SG filter coefficients come from pls_model_data.h (pre-computed)
 */

#include "preprocessing.h"
#include "pls_model_data.h"
#include <math.h>

/* ---------------------------------------------------------------
 * Internal: Apply a convolution filter (SG filter) to a spectrum
 * Uses edge-extension (repeat boundary values) for border handling
 * --------------------------------------------------------------- */
static void apply_sg_filter(const float *input, float *output, 
                            int len, const float *coeffs, int window_size)
{
    int half_w = window_size / 2;
    int i, j;
    
    for (i = 0; i < len; i++) {
        float sum = 0.0f;
        for (j = 0; j < window_size; j++) {
            int idx = i + j - half_w;
            /* Clamp to boundaries (edge extension) */
            if (idx < 0) idx = 0;
            if (idx >= len) idx = len - 1;
            sum += coeffs[j] * input[idx];
        }
        output[i] = sum;
    }
}

/* ---------------------------------------------------------------
 * sg_smooth: SG smoothing (window=11, polyorder=2, deriv=0)
 * --------------------------------------------------------------- */
void sg_smooth(const float *input, float *output, int len)
{
    apply_sg_filter(input, output, len, SG_SMOOTH_COEFFS, SG_WINDOW_SIZE);
}

/* ---------------------------------------------------------------
 * sg_first_derivative: SG 1st derivative (window=11, polyorder=3, deriv=1)
 * --------------------------------------------------------------- */
void sg_first_derivative(const float *input, float *output, int len)
{
    apply_sg_filter(input, output, len, SG_DERIV1_COEFFS, SG_WINDOW_SIZE);
}

/* ---------------------------------------------------------------
 * snv_normalize: Standard Normal Variate normalization (in-place)
 *   x[i] = (x[i] - mean) / (std + 1e-8)
 * --------------------------------------------------------------- */
void snv_normalize(float *data, int len)
{
    int i;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float mean, std_dev;
    
    /* Compute mean */
    for (i = 0; i < len; i++) {
        sum += data[i];
    }
    mean = sum / (float)len;
    
    /* Compute standard deviation */
    for (i = 0; i < len; i++) {
        float diff = data[i] - mean;
        sum_sq += diff * diff;
    }
    std_dev = sqrtf(sum_sq / (float)len);
    
    /* Normalize */
    float inv_std = 1.0f / (std_dev + 1e-8f);
    for (i = 0; i < len; i++) {
        data[i] = (data[i] - mean) * inv_std;
    }
}

/* ---------------------------------------------------------------
 * preprocess_spectrum: Full pipeline
 *   1. SG Smooth (w=11, p=2)
 *   2. SG 1st Derivative (w=11, p=3)
 *   3. SNV Normalization
 * --------------------------------------------------------------- */
void preprocess_spectrum(const float *raw_spectrum, float *processed, 
                         float *work_buffer)
{
    /* Step 1: Savitzky-Golay smoothing */
    sg_smooth(raw_spectrum, work_buffer, SPECTRUM_LENGTH);
    
    /* Step 2: 1st derivative of smoothed spectrum */
    sg_first_derivative(work_buffer, processed, SPECTRUM_LENGTH);
    
    /* Step 3: SNV normalization (in-place on processed) */
    snv_normalize(processed, SPECTRUM_LENGTH);
}
