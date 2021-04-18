/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>

#include<detection.h>

static THD_WORKING_AREA(waDetection, 256);
static THD_FUNCTION(Detection, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;



}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO, Detection, NULL);
}
