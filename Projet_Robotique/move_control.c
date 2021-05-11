#include "ch.h"
#include "hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <chprintf.h>
#include <usbcfg.h>
#include <stdbool.h>

#include <main.h>
#include <move_control.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <camera/po8030.h>


// module de gestion du deplacement avec la camera et les capteurs de proximite

//static BSEMAPHORE_DECL(block_passed, TRUE);				//permet de signaler quand le bloc est depasse
//static BSEMAPHORE_DECL(position_reached, TRUE);			//signale quand le robot est a la bonne distance du prochain bloc

//static bool position_reached = 0;
static THD_WORKING_AREA(waMoveControl, 1024);
static THD_FUNCTION(MoveControl, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;
	int16_t speed =0;
	int16_t speed_correction = 0;
	while(1)
	{
		time = chVTGetSystemTime();
//		move_to_block(speed, speed_correction);
////
////
//		if (get_distance_cm() == GOAL_DISTANCE)
//		{
//			right_motor_set_speed(0);
//			left_motor_set_speed(0);
////			position_reached = POSITION_REACHED;
//			move_between_blocks(get_block(), 5);
//			break;
//		}


//		chThdSleepUntilWindowed(time, time + MS2ST(1));

		test_capteur();
		chThdSleepMilliseconds(100);
	}
}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}

void test_capteur(void)
{

	calibrate_ir();
	int front_left = get_prox(6);
	int front_right= get_prox(1);
	int left = get_prox(5);
	int right = get_prox(2);
//	if(left!=0)
//	{
//		palClearPad(GPIOD, GPIOD_LED7);
//	}else
//	{
//		palSetPad(GPIOD, GPIOD_LED7);
//	}
	chprintf((BaseSequentialStream *)&SDU1, "front_left= %d front_right = %d left= %d right= %d \n", front_left, front_right, left, right);

}

void move_to_block(int16_t speed, int16_t speed_correction)
{
	speed = pi_regulator_blocks(get_distance_cm(), GOAL_DISTANCE);

	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	if(abs(speed_correction)< ROTATION_THRESHOLD)
	{
		speed_correction = 0;
	}
	right_motor_set_speed(COEFF_VITESSE*(speed - ROTATION_COEFF * speed_correction));
	left_motor_set_speed(COEFF_VITESSE*(speed + ROTATION_COEFF * speed_correction));
}

void turn(uint block)
{
	int32_t count = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	switch(block)
	{
	case RIGHT:
		right_motor_set_speed(400);
		left_motor_set_speed(-400);
		do
		{
			count = right_motor_get_pos();
		} while(abs(count) < (NSTEP_ONE_TURN*PERIMETER_EPUCK/4/WHEEL_PERIMETER));
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	case LEFT:
		right_motor_set_speed(-400);
		left_motor_set_speed(400);
		do
		{
			count = right_motor_get_pos();
		} while(abs(count) < (NSTEP_ONE_TURN*PERIMETER_EPUCK/4/WHEEL_PERIMETER));
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	case 0:
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	}
}

void move_cm(uint distance)
{
	int32_t count = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	right_motor_set_speed(400);
	left_motor_set_speed(400);
	do
	{
		count = right_motor_get_pos();
	}while(abs(count) < (distance *NSTEP_ONE_TURN/WHEEL_PERIMETER));
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void move_between_blocks(uint block, uint distance)
{
	turn(block);
	move_cm(distance);
	switch(block)
	{
	case RIGHT:
		turn(LEFT);
		break;
	case LEFT:
		turn(RIGHT);
		break;
	case 0:
		break;
	}
}
