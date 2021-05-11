/*
 * mouvement.h
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

int16_t p_regulator(int16_t distance, int16_t goal);

void read_and_move(void);

void path_correction(void);

void turn_right(void);
void turn_left(void);
void turn_back(void);
void stop_moving(void);



#endif /* MOVEMENT_H_ */
