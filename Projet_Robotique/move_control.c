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
#include <leds.h>


// module de gestion du deplacement avec la camera et les capteurs de proximite

//static BSEMAPHORE_DECL(block_passed, TRUE);				//permet de signaler quand le bloc est depasse
//static BSEMAPHORE_DECL(position_reached, TRUE);			//signale quand le robot est a la bonne distance du prochain bloc

static bool position_reached = POSITION_NOT_REACHED;
static bool block_passed = BLOCK_PASSED;
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
		while(!position_reached)
		{
			set_led(LED3,0);
			set_led(LED1,1);
			move_to_block();
			if (get_distance_cm() == GOAL_DISTANCE)
			{
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				move_between_blocks(get_block(), 4.8);
				block_passed = BLOCK_NOT_PASSED;
				position_reached = POSITION_REACHED;
				chThdSleepMilliseconds(1000);
//				break;
			}
		}
		while(!block_passed)
		{
			set_led(LED1,0);
			set_led(LED3,1);
			calibrate_ir();
			right_motor_set_speed (MOVE_SPEED/2 + pi_regulator_capteurs(get_prox(2),get_prox(5)));
			left_motor_set_speed (MOVE_SPEED/2 - pi_regulator_capteurs(get_prox(2),get_prox(5)));
			if ((get_prox(2)<get_prox(3)) && (get_prox(5)<get_prox(4)) && (get_prox(4)> 400) &&(get_prox(3) > 400))
			{
				block_passed = BLOCK_PASSED;
				position_reached = POSITION_NOT_REACHED;
				right_motor_set_speed(0);
				left_motor_set_speed(0);
				chThdSleepMilliseconds(1000);
//				break;
			}
		}

		chThdSleepUntilWindowed(time, time + MS2ST(1));
	}
}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}

void test_capteur(void)
{

	calibrate_ir();
//	int front_left = get_prox(6);
//	int front_right= get_prox(1);
//	int left = get_prox(5);
//	int right = get_prox(2);
//	if(left!=0)
//	{
//		palClearPad(GPIOD, GPIOD_LED7);
//	}else
//	{
//		palSetPad(GPIOD, GPIOD_LED7);
//	}
//	chprintf((BaseSequentialStream *)&SDU1, "front_left= %d front_right = %d left= %d right= %d \n", front_left, front_right, left, right);

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

void turn(uint block)
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
		} while(abs(count) < (NSTEP_ONE_TURN*PERIMETER_EPUCK/4/WHEEL_PERIMETER));
		right_motor_set_speed(0);
		left_motor_set_speed(0);
		break;
	case LEFT:
		right_motor_set_speed(-MOVE_SPEED);
		left_motor_set_speed(MOVE_SPEED);
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
