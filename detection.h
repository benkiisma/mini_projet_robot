/*
 * detection.h
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#ifndef DETECTION_H_
#define DETECTION_H_

//Constant definition
#define DETECT_DIST 		150
#define DETECT_DIST_FRONT 	200

void define_robot_state(void);

int get_robot_state(void);

void stop_robot(void);


#endif /* DETECTION_H_ */
