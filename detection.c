/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#include <main.h>

#include <detection.h>
#include <sensors/proximity.h>
//#include <usbcfg.h>
#include <leds.h>
#include <audio_processing.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"

static int robot_state;

static THD_WORKING_AREA(waDetection, 512);
static THD_FUNCTION(Detection, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	systime_t time;

	while(1){
		time = chVTGetSystemTime();

		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		//Changing the state of the robot depending on the sensor values
		if(!get_stop()){
			if(get_calibrated_prox(0) < 150 && get_calibrated_prox(7) < 150){
				robot_state = 0;
			}
			else if(get_calibrated_prox(0) > 150 && get_calibrated_prox(7) > 150){
				if(get_calibrated_prox(2) > DETECT_DIST && get_calibrated_prox(5) < DETECT_DIST){
					robot_state = 1;
				}
				else if(get_calibrated_prox(5) > DETECT_DIST && get_calibrated_prox(2) < DETECT_DIST){
					robot_state = 2;
				}
				else if(get_calibrated_prox(2) > DETECT_DIST && get_calibrated_prox(5) > DETECT_DIST){
					robot_state = 3;
				}
				else{
					robot_state = 4;
				}
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(10)); // Refresh @ 100 Hz
	}
}

int get_robot_state(void){
	return robot_state;
}

void stop_robot(void){
	robot_state = 4;
}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO+1, Detection, NULL);
}
