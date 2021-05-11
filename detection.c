/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#include <main.h>

#include <sensors/proximity.h>

#include <detection.h>
#include <audio_processing.h>

//Variable changed by detection of obstacles
static int robot_state;

//Changing the state of the robot depending on the sensor values
void define_robot_state(void){
	if(!get_stop()){
		if(get_calibrated_prox(0) < DETECT_DIST_FRONT && get_calibrated_prox(7) < DETECT_DIST_FRONT){
			robot_state = 0;
		}
		else if(get_calibrated_prox(0) > DETECT_DIST_FRONT && get_calibrated_prox(7) > DETECT_DIST_FRONT){
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

//Send the value of robot_state to movement.c
int get_robot_state(void){
	return robot_state;
}

void stop_robot(void){
	robot_state = 4;
}
