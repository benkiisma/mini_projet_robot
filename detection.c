/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>

#include <detection.h>
#include <sensors/proximity.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <leds.h>

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

		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz

		//on modifie le robot_state pour que le robot choisisse la direction � prendre
		if(get_calibrated_prox(0) < 300 && get_calibrated_prox(7) < 300){
			robot_state = 0;
		}
		else if(get_calibrated_prox(0) > 300 && get_calibrated_prox(7) > 300){
			if(get_calibrated_prox(2) > 300){
				set_front_led(2);
				robot_state = 1;
			}
			else if(get_calibrated_prox(5) > 300){
				robot_state = 2;
			}
			else if(get_calibrated_prox(2) > 300 && get_calibrated_prox(5) > 300){
				robot_state = 3;
			}
			else{
				robot_state = 4;
				set_body_led(2);
			}
		}
	}
}

int get_robot_state(void){
	return robot_state;
}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO, Detection, NULL);
}
