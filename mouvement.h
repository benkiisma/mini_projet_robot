/*
 * mouvement.h
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas & Ismail
 */

#ifndef MOUVEMENT_H_
#define MOUVEMENT_H_

int16_t p_regulator(int16_t distance, int16_t goal);

void mouvement_start(void);
void path_correction(void);

void turn_right(void);
void turn_left(void);
void turn_back(void);
void stop_moving(void);



#endif /* MOUVEMENT_H_ */
