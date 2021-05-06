/*
 * mouvement.c
 *
 *  Created on: 18 avr. 2021
 *  We used a part of the TP4 code for the P regulator implementation
 *      Author: Nicolas & Ismail
 */

#include <main.h>
#include "ch.h"
#include "hal.h"

#include <usbcfg.h>
#include <chprintf.h>

#include <motors.h>
#include <mouvement.h>
#include <detection.h>
#include <sensors/proximity.h>
#include <leds.h>

//Constants definition
#define ERROR_THRESHOLD				10
#define KP							0.2f
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)
#define MOTOR_SPEED					600
#define RATIO_DIAG					1
//#define RATIO_DIAG_MAX			1.91f
//#define RATIO_DIAG_MIN			1.89f

static THD_WORKING_AREA(waMouvement, 256);
static THD_FUNCTION(Mouvement, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	systime_t time;

	while(1){

		time = chVTGetSystemTime();

		switch(get_robot_state()){
			case 0:
				//correct the path if the robot is not going straight
				path_correction();
				break;
			case 1:
				turn_left();
				break;
			case 2:
				turn_right();
				break;
			case 3:
				turn_back();
				break;
			case 4:
				stop_moving();
				break;
			}
	}
	//100Hz
	chThdSleepUntilWindowed(time, time + MS2ST(10));
}

//P regulator implementation
int16_t p_regulator(int16_t distance, int16_t goal){

	int16_t error = 0;
	int16_t speed = 0;

	error = distance - goal;

	//disables the P regulator if the error is to small
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP * error;

    return (int16_t)speed;
}

void path_correction(void){
	int16_t distance_side;
	int16_t speed_correction;

	distance_side = get_calibrated_prox(2);

	if(get_calibrated_prox(2) < 150){
		distance_side = get_calibrated_prox(5);
	}
	int16_t ratio_g = get_calibrated_prox(6)/get_calibrated_prox(4);
	int16_t ratio_d = get_calibrated_prox(1)/get_calibrated_prox(3);
	chprintf((BaseSequentialStream *)&SD3, "%4d,",ratio_g);
	chprintf((BaseSequentialStream *)&SD3, "%4d,",ratio_d);
	chprintf((BaseSequentialStream *)&SD3, "\r\n");
	//determining the necessary correction
	if(get_calibrated_prox(2) > DETECT_DIST && get_calibrated_prox(5) > DETECT_DIST){
		speed_correction = p_regulator(distance_side, (get_calibrated_prox(2) + get_calibrated_prox(5))/2);
		//if the robot is to the wall, we don't correct
		if(ratio_g == 1 && ratio_d == 1){
			speed_correction = 0;
			set_body_led(2);
		}
	}else{
		speed_correction = 0;
	}



	//correction of the path
	if(distance_side == get_calibrated_prox(2)){
		left_motor_set_speed(MOTOR_SPEED - speed_correction);
		right_motor_set_speed(MOTOR_SPEED + speed_correction);
	}
	else{
		left_motor_set_speed(MOTOR_SPEED + speed_correction);
		right_motor_set_speed(MOTOR_SPEED - speed_correction);
	}
}

void turn_right(void){
	left_motor_set_speed(MOTOR_SPEED);
	right_motor_set_speed(-MOTOR_SPEED);
	chThdSleepMilliseconds(517);
}

void turn_left(void){
	left_motor_set_speed(-MOTOR_SPEED);
	right_motor_set_speed(MOTOR_SPEED);
	chThdSleepMilliseconds(517);
}

void turn_back(void){
	left_motor_set_speed(MOTOR_SPEED);
	right_motor_set_speed(-MOTOR_SPEED);
	chThdSleepMilliseconds(1050);
}

void stop_moving(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

void mouvement_start(void){
	chThdCreateStatic(waMouvement, sizeof(waMouvement), NORMALPRIO, Mouvement, NULL);
}

