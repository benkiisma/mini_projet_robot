/*
 * mouvement.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>

#include <motors.h>
#include <mouvement.h>
#include <detection.h>

static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	switch(get_robot_state()){
		case 0:
			left_motor_set_speed(600);
			right_motor_set_speed(600);
			break;
		case 1:
			//obstacle devant, pas à gauche --> tourne à gauche --> case 0
			left_motor_set_speed(-600);
			right_motor_set_speed(600);
			break;
		case 2:
			//obstacle devant, pas à droite --> tourne à droite --> case 0
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
			break;
		case 3:
			//bloqué de partout, demi-tour 2 fois le temps d'un virage
			left_motor_set_speed(600);
			right_motor_set_speed(-600);
			break;
		case 4:
			left_motor_set_speed(0);
			right_motor_set_speed(0);
			break;
		}
}
void mouvement_start(void){
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

