/*
 * robot_control.c
 *
 *  Created on: 6 mai 2021
 *      Author: Nicolas & Ismail
 */

#include <main.h>
#include "ch.h"
#include "hal.h"

#include <detection.h>
#include <movement.h>

static THD_WORKING_AREA(waMovement, 256);
static THD_FUNCTION(Movement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){

		time = chVTGetSystemTime();

		define_robot_state();

		read_and_move();
	}
	//100Hz
	chThdSleepUntilWindowed(time, time + MS2ST(10));
}

void movement_start(void){
	chThdCreateStatic(waMovement, sizeof(waMovement), NORMALPRIO, Movement, NULL);
}
