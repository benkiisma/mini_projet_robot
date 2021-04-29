/*
 * mouvement.h
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */

#ifndef MOUVEMENT_H_
#define MOUVEMENT_H_

int16_t pi_regulator(int16_t distance, int16_t goal);

//start the PI regulator thread
void mouvement_start(void);


#endif /* MOUVEMENT_H_ */
