/*
 * preprocessing.h
 * Spectral preprocessing for TCD1304 CCD data on STM32H7
 * 
 * Pipeline: SG Smooth (w=11,p=2) -> SG 1st Derivative (w=11,p=3) -> SNV
 */

#ifndef PREPROCESSING_H
#define PREPROCESSING_H

#include <stdint.h>

#define SPECTRUM_LENGTH    3694

/*
 * Apply Savitzky-Golay smoothing filter.
 * @param input   Raw spectrum array (SPECTRUM_LENGTH elements)
 * @param output  Smoothed spectrum array (SPECTRUM_LENGTH elements)
 * @param len     Length of spectrum
 */
void sg_smooth(const float *input, float *output, int len);

/*
 * Apply Savitzky-Golay 1st derivative filter.
 * @param input   Smoothed spectrum array
 * @param output  1st derivative spectrum array
 * @param len     Length of spectrum
 */
void sg_first_derivative(const float *input, float *output, int len);

/*
 * Apply Standard Normal Variate (SNV) normalization.
 * Normalizes in-place: x[i] = (x[i] - mean) / (std + 1e-8)
 * @param data    Spectrum array (modified in-place)
 * @param len     Length of spectrum
 */
void snv_normalize(float *data, int len);

/*
 * Apply full preprocessing pipeline:
 *   1. SG Smoothing (window=11, polyorder=2)
 *   2. SG 1st Derivative (window=11, polyorder=3)  
 *   3. SNV Normalization
 * 
 * @param raw_spectrum    Input raw spectrum (SPECTRUM_LENGTH floats)
 * @param processed       Output processed spectrum (SPECTRUM_LENGTH floats)
 * @param work_buffer     Working buffer (at least SPECTRUM_LENGTH floats)
 */
void preprocess_spectrum(const float *raw_spectrum, float *processed, 
                         float *work_buffer);

#endif /* PREPROCESSING_H */
