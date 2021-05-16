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
#include "sensors/imu.h"
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <camera/po8030.h>
#include <leds.h>


// module de gestion du deplacement avec la camera et les capteurs de proximite

static bool position_reached = POSITION_REACHED; 			//indicate when the position is reached to activate the proximity sensors
static bool block_passed = BLOCK_NOT_PASSED;				//indicate when the blocks are passed, and the camera can be used again

static THD_WORKING_AREA(waMoveControl, 1024);
static THD_FUNCTION(MoveControl, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;
	clear_leds();
	while(1)
	{
		time = chVTGetSystemTime();
		while(!position_reached)						//This loop will make the robot travel to the center between two blocks
		{												//It uses the camera for this
			move_to_block();
			if (get_distance_cm() == GOAL_DISTANCE)
			{
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				move_between_blocks(get_block(), DISTANCE_BETWEEN_BLOCKS);
				block_passed = BLOCK_NOT_PASSED;
				position_reached = POSITION_REACHED;
				chThdSleepMilliseconds(10);
				break;
			}
		}
		while(!block_passed) 						//This loop will guide the robot between the to blocks to the exit of them,
		{											//facing the next block, using the proximity sensors
			calibrate_ir();
			right_motor_set_speed (MOVE_SPEED/2 + pi_regulator_capteurs(get_prox(2),get_prox(5)));
			left_motor_set_speed (MOVE_SPEED/2 - pi_regulator_capteurs(get_prox(2),get_prox(5)));
			if ((get_prox(5)< 400) &&(get_prox(2) < 400))
			{
				block_passed = BLOCK_PASSED;
				position_reached = POSITION_NOT_REACHED;
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				break;
			}
		}

		chThdSleepUntilWindowed(time, time + MS2ST(1));
	}
}

static THD_WORKING_AREA(waImuEnding, 512);
static THD_FUNCTION(ImuEnding, arg)					// This thread will end the programm once the robot is down the slope
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;
	while(1)
	{
		time = chVTGetSystemTime();
		if(position_reached == POSITION_NOT_REACHED)
		{
			if(get_acc_filtered(2,50) < -16300)
			{
				set_led(LED1, 1);
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				position_reached = POSITION_REACHED;
				block_passed = BLOCK_PASSED;
				turn(RIGHT, 1);
				turn(LEFT, 1);
			}
		}
		chThdSleepUntilWindowed(time, time + MS2ST(100));
	}
}

void imu_ending_start(void)
{
	chThdCreateStatic(waImuEnding, sizeof(waImuEnding), NORMALPRIO, ImuEnding, NULL);
}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}

void move_to_block(void)
{
	int16_t speed =0;
	int16_t speed_correction = 0;
	speed = pi_regulator_blocks(get_distance_cm(), GOAL_DISTANCE);

	speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
	if(abs(speed_correction)< ROTATION_THRESHOLD)
	{
		speed_correction = 0;
	}
	right_motor_set_speed(COEFF_VITESSE*speed - ROTATION_COEFF * speed_correction);
	left_motor_set_speed(COEFF_VITESSE*speed + ROTATION_COEFF * speed_correction);
}

void turn(uint block, float number_turns)
{
	int32_t count = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	switch(block)
	{
	case RIGHT:
		right_motor_set_speed(MOVE_SPEED);
		left_motor_set_speed(-MOVE_SPEED);
		do
		{
			count = right_motor_get_pos();
		} while(abs(count) < (number_turns*NSTEP_ONE_TURN*PERIMETER_EPUCK/WHEEL_PERIMETER));
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	case LEFT:
		right_motor_set_speed(-MOVE_SPEED);
		left_motor_set_speed(MOVE_SPEED);
		do
		{
			count = right_motor_get_pos();
		} while(abs(count) < (number_turns*NSTEP_ONE_TURN*PERIMETER_EPUCK/WHEEL_PERIMETER));
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	case 0:
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	}
}

void move_cm(float distance)
{
	int32_t count = 0;
	left_motor_set_pos(0);
	right_motor_set_pos(0);
	right_motor_set_speed(MOVE_SPEED);
	left_motor_set_speed(MOVE_SPEED);
	do
	{
		count = right_motor_get_pos();
	}while(abs(count) < (distance *NSTEP_ONE_TURN/WHEEL_PERIMETER));
	right_motor_set_speed(0);
	left_motor_set_speed(0);
}

void move_between_blocks(uint block, float distance)
{
	turn(block, QUARTER_TURN);
	move_cm(distance);
	switch(block)
	{
	case RIGHT:
		turn(LEFT, QUARTER_TURN);
		break;
	case LEFT:
		turn(RIGHT, QUARTER_TURN);
		break;
	case 0:
		break;
	}
	move_cm(DISTANCE_TO_BLOCK);
}
