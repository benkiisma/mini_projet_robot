/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#include <main.h>

#include <detection.h>
#include <sensors/proximity.h>

//#include "ch.h"
//#include "hal.h"
//#include "memory_protection.h"

static int robot_state;

void define_robot_state(void){
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
}

int get_robot_state(void){
	return robot_state;
}

void stop_robot(void){
	robot_state = 4;
}
