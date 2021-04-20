/*
 * detection.c
 *
 *  Created on: 18 avr. 2021
 *      Author: Nicolas
 */
#include <main.h>

#include <detection.h>
#include <sensors/proximity.h>
#include <chprintf.h>
#include <usbcfg.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"


static THD_WORKING_AREA(waDetection, 512);
static THD_FUNCTION(Detection, arg) {

	chRegSetThreadName(__FUNCTION__);
	(void)arg;

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;

	systime_t time;

	while(1){
		time = chVTGetSystemTime();
		//for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
//		int valeur;
//		for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
//			valeur = get_calibrated_prox(i);
//			chprintf((BaseSequentialStream *)&SDU1, "%4d,", valeur);
//		}

		messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));

		for (uint8_t i = 0; i < sizeof(prox_values.ambient)/sizeof(prox_values.ambient[0]); i++) {
				        //for (uint8_t i = 0; i < PROXIMITY_NB_CHANNELS; i++) {
				        	chprintf((BaseSequentialStream *)&SD3, "%4d,", prox_values.ambient[i]);
				        	chprintf((BaseSequentialStream *)&SD3, "%4d,", prox_values.reflected[i]);
				        	chprintf((BaseSequentialStream *)&SD3, "%4d", prox_values.delta[i]);
				        	chprintf((BaseSequentialStream *)&SD3, "\r\n");
				        }
				        chprintf((BaseSequentialStream *)&SD3, "\r\n");

		chThdSleepUntilWindowed(time, time + MS2ST(100)); // Refresh @ 10 Hz
	}
}
	//on regarde les capteurs de devant et sur les côté --> trouver les numéros associés
//	if(){
//		switch(robot_state){
//			case 0:
//				//pas d'obstacle --> avance tout droit
//				break;
//			case 1:
//				//obstacle devant, pas à gauche --> tourne à gauche --> case 0
//				break;
//			case 2:
//				//obstacle devant, pas à droite --> tourne à droite --> case 0
//				break;
//			case 3:
//				//bloqué de partout, demi-tour
//				break;
//		}
//	}

void detection_start(void){
	chThdCreateStatic(waDetection, sizeof(waDetection), NORMALPRIO, Detection, NULL);
}
