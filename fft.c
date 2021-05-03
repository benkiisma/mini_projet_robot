/*
 * fft.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#include "ch.h"
#include "hal.h"
#include <main.h>
#include <fft.h>

#include <arm_const_structs.h>


/*
*	Wrapper to call a very optimized fft function provided by ARM
*	which uses a lot of tricks to optimize the computations
*/

void FFT_optimized(uint16_t size, float* complex_buffer){
	if(size == 1024)
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, complex_buffer, 0, 1);
	
}
