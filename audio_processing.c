/*
 * audio_processing.c
 *
 *  Created on: 18 avr. 2021
 *  We used a part of the TP5 code for sound analysis
 *      Author: Nicolas & Ismail
 */

#include "ch.h"
#include "hal.h"
#include <main.h>

#include <audio/microphone.h>
#include <arm_math.h>
#include <arm_const_structs.h>
#include <leds.h>

#include <audio_processing.h>
#include <detection.h>

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];

//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];

//Variable changed by the detection of 1000Hz
static int stop;


//Constants definition
#define MIN_VALUE_THRESHOLD	10000 

#define MIN_FREQ		56	//we don't analyze before this index to not use resources for nothing
#define FREQ_STOP		66	//1000Hz
#define MAX_FREQ		76	//we don't analyze after this index to not use resources for nothing

#define FREQ_STOP_L		(FREQ_STOP-1)
#define FREQ_STOP_H		(FREQ_STOP+1)


static THD_WORKING_AREA(waAudio, 256);
static THD_FUNCTION(Audio, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

    //Starts the microphones processing thread.
    mic_start(&processAudioData);

	while(1){

		time = chVTGetSystemTime();

		//If we detect 1000Hz, robot stops and body_led blinks
        if(stop){
        	stop_robot();
        	set_body_led(2);
        }
        //50Hz
        chThdSleepUntilWindowed(time, time + MS2ST(50));
	}
}


//Wrapper to call a very optimized fft function provided by ARM
void FFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
}


//Simple function used to detect the highest value in a buffer
void sound_remote(float* data){
	float max_norm = MIN_VALUE_THRESHOLD;
	int16_t max_norm_index = -1; 

	//Search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++){
		if(data[i] > max_norm){
			max_norm = data[i];
			max_norm_index = i;
		}
	}

	// Stops the robot if it detects 1000Hz
	if(max_norm_index >= FREQ_STOP_L && max_norm_index <= FREQ_STOP_H ){
		stop = 1;
	}
	else{
		stop = 0;
	}
}

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];

		nb_samples++;

		micLeft_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		FFT_optimized(FFT_SIZE, micLeft_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);

		nb_samples = 0;

		sound_remote(micLeft_output);
	}
}

//Send the value stop to detection.c
int get_stop(void){
	return stop;
}

//Starts the audio thread
void audio_start(void){
	chThdCreateStatic(waAudio, sizeof(waAudio), NORMALPRIO+1, Audio, NULL);
}
