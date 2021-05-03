/*
 * main.c
 *
 *  Created on: 18 avr. 2021S
 *      Author: Nicolas & Ismail
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
//#include <chprintf.h>
#include <motors.h>
//#include <audio/microphone.h>
#include <sensors/proximity.h>
#include <arm_math.h>

#include <main.h>
#include <audio_processing.h>
#include <fft.h>
#include <communications.h>
#include <detection.h>
#include <mouvement.h>

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);


#define SEND_FROM_MIC

//uncomment to use double buffering to send the FFT to the computer
//#define DOUBLE_BUFFERING

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{

    halInit();
    chSysInit();
    mpu_init();

    messagebus_init(&bus, &bus_lock, &bus_condvar);

    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    //starts the proximity sensor
    proximity_start();
    //starts the detection of obstacles
    detection_start();
    // starts the movement of the robot
    mouvement_start();
    // starts the detection of the sound
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
