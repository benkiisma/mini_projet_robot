/*
 * audio_processing.h
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#ifndef AUDIO_PROCESSING_H
#define AUDIO_PROCESSING_H


#define FFT_SIZE 	1024

void FFT_optimized(uint16_t size, float* complex_buffer);

void processAudioData(int16_t *data, uint16_t num_samples);

int get_stop(void);

void audio_start(void);

#endif /* AUDIO_PROCESSING_H */
