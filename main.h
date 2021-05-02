#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

//Constants definition
//#define GOAL_DISTANCE_SIDE			200
#define ERROR_THRESHOLD				50
#define KP							0.2f
#define MAX_SUM_ERROR 				(MOTOR_SPEED_LIMIT/KI)



/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
