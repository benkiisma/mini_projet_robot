/*
 * mouvement.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>

#include <motors.h>
#include <mouvement.h>

static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	left_motor_set_speed(600);
	right_motor_set_speed(600);
}
void mouvement_start(void){
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

