/*
 * movement.c
 *
 *  Created on: 18 avr. 2021
 *  We used a part of the TP4 code for the P regulator implementation
 *      Author: Nicolas & Ismail
 */

#include <main.h>

#include <stdlib.h>

#include <sensors/proximity.h>
#include <motors.h>

#include <detection.h>
#include <movement.h>

//Constants definition
#define ERROR_THRESHOLD				10
#define KP							0.2f
#define MOTOR_SPEED					600
#define RATIO_DIAG_L				1.3f
#define RATIO_DIAG_H				2.5f

void read_and_move(void){
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

/*P regulator implementation
 *
 * We chose three variable in int16_t instead of float because we use values from proximity sensor
 * that work with unsigned int as well as motor for the speed.
 *
 * We took 16 bits because the values can't reach 2^16 = 65'536 and not less to avoid overflow
 */
int16_t p_regulator(int16_t distance, int16_t goal){

	int16_t error = 0;
	int16_t speed = 0;

	error = distance - goal;

	//disables the P regulator if the error is too small
	if(abs(error) < ERROR_THRESHOLD){
		return 0;
	}

	speed = KP * error;

    return (int16_t)speed;
}

void path_correction(void){

	float ratio; //we take the ratio between diagonal values of the sensors, need to be a float
	int16_t distance_side;
	int16_t speed_correction; // here again we have int16_t because we deal with proximity values

	//we put a reference on the right side. If the wall is too far, we take the left side
	distance_side = get_calibrated_prox(2);
	ratio = (float)get_calibrated_prox(1)/get_calibrated_prox(3);

	if(get_calibrated_prox(2) < DETECT_DIST){
		distance_side = get_calibrated_prox(5);
		ratio = (float)get_calibrated_prox(6)/get_calibrated_prox(4);
	}

	//determining the necessary correction
	if(get_calibrated_prox(2) > DETECT_DIST && get_calibrated_prox(5) > DETECT_DIST){
		speed_correction = p_regulator(distance_side, (get_calibrated_prox(2) + get_calibrated_prox(5))/2);
		//if the robot is parallel to the wall, we don't correct
		if(ratio >= RATIO_DIAG_L && ratio <= RATIO_DIAG_H){
			speed_correction = 0;
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
	chThdSleepMilliseconds(525); //Wait until ~90° rotation
}

void turn_left(void){
	left_motor_set_speed(-MOTOR_SPEED);
	right_motor_set_speed(MOTOR_SPEED);
	chThdSleepMilliseconds(525);
}

void turn_back(void){
	left_motor_set_speed(MOTOR_SPEED);
	right_motor_set_speed(-MOTOR_SPEED);
	chThdSleepMilliseconds(1060); //Wait until ~180° rotation
}

void stop_moving(void){
	left_motor_set_speed(0);
	right_motor_set_speed(0);
}

