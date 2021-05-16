#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			35
#define MAX_LINE_WIDTH			150
#define WHITE_VALUE				225 			//value of the white strips on the post captured by the camera
#define ROTATION_THRESHOLD		12
#define ROTATION_COEFF			3
#define COEFF_VITESSE			0.5f
#define MOVE_SPEED				300 			// movement speed to control many movements. [steps]
#define PXTOCM					535.0f 			//experimental value
#define GOAL_DISTANCE 			4.0f
#define MAX_DISTANCE 			20.0f
#define ERROR_THRESHOLD			0.1f			//[cm] because of the noise of the camera
#define KP						200.0f
#define KI 						3.5f			//must not be zero
#define MAX_SUM_ERROR			MOTOR_SPEED_LIMIT/KI
#define RIGHT					1
#define LEFT					2
#define POSITION_REACHED		1
#define POSITION_NOT_REACHED	0
#define BLOCK_PASSED			1
#define BLOCK_NOT_PASSED		0
#define NSTEP_ONE_TURN      	1000
#define WHEEL_DISTANCE      	5.35f
#define PERIMETER_EPUCK     	(PI * WHEEL_DISTANCE)
#define PI                  	3.1415926536f
#define WHEEL_PERIMETER     	13

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
