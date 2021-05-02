/*
 * mouvement.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>
#include "ch.h"
#include "hal.h"

#include <motors.h>
#include <mouvement.h>
#include <detection.h>
#include <sensors/proximity.h>
#include <leds.h>

static int16_t speed_correction;
static int16_t distance_side;
static int16_t goal_distance;

static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){

		time = chVTGetSystemTime();

		switch(get_robot_state()){
			case 0:
				//Calcul distance of the proximity sensor on right side or left side
				distance_side = get_calibrated_prox(2);
				if(get_calibrated_prox(2) < 150){
					distance_side = get_calibrated_prox(5);
				}

				if(get_calibrated_prox(2) > 125 && get_calibrated_prox(5) > 125){
					goal_distance = (get_calibrated_prox(2) + get_calibrated_prox(5))/2;
					speed_correction = pi_regulator(distance_side, goal_distance);
				}else{
					speed_correction = 0;
				}
				if(distance_side == get_calibrated_prox(2)){
					left_motor_set_speed(600 - speed_correction);
					right_motor_set_speed(600 + speed_correction);
				}
				else{
					left_motor_set_speed(600 + speed_correction);
					right_motor_set_speed(600 - speed_correction);
				}
				break;
			case 1:
				//obstacle devant, pas à gauche --> tourne à gauche --> case 0
				left_motor_set_speed(-600);
				right_motor_set_speed(600);
				chThdSleepMilliseconds(520);
				break;
			case 2:
				//obstacle devant, pas à droite --> tourne à droite --> case 0
				left_motor_set_speed(600);
				right_motor_set_speed(-600);
				chThdSleepMilliseconds(520);
				break;
			case 3:
				//bloqué de partout, demi-tour 2 fois le temps d'un virage
				left_motor_set_speed(600);
				right_motor_set_speed(-600);
				chThdSleepMilliseconds(1040);
				break;
			case 4:
				left_motor_set_speed(0);
				right_motor_set_speed(0);
				break;
			}
	}
	//100Hz
	chThdSleepUntilWindowed(time, time + MS2ST(10));
}

//simple PI regulator implementation
int16_t pi_regulator(int16_t distance, int16_t goal){

	int16_t error = 0;
	int16_t speed = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want
	if(abs(error) < ERROR_THRESHOLD){
		set_front_led(2);
		return 0;
	}

	speed = KP * error;

    return (int16_t)speed;
}

void mouvement_start(void){
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

