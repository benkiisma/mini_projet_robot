/*
 * main.c
 *
 *  Created on: 18 avr. 2021S
 *  We used a part of the TP5 code
 *      Author: Nicolas & Ismail
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <motors.h>
#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <arm_math.h>

#include <main.h>
#include <audio_processing.h>
#include <robot_control.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //starts the proximity sensor
    proximity_start();
    //starts detection and movement
    movement_start();
    //starts detection of the audio
    audio_start();

    /* Infinite loop. */
    while (1) {
    	;
    }
}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
