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

static BSEMAPHORE_DECL(block_passed, TRUE);				//permet de signaler quand le bloc est depasse
static BSEMAPHORE_DECL(position_reached, TRUE);			//signale quand le robot est a la bonne distance du prochain bloc

//int16_t speed =0;
//int16_t speed_correction = 0;

static THD_WORKING_AREA(waMoveControl, 1024);
static THD_FUNCTION(MoveControl, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;
	while(1)
	{
		time = chVTGetSystemTime();
//		speed = pi_regulator(get_distance_cm(), GOAL_DISTANCE);
//
//		speed_correction = (get_line_position() - (IMAGE_BUFFER_SIZE/2));
//		if(abs(speed_correction)< ROTATION_THRESHOLD)
//		{
//			speed_correction = 0;
//		}
//		right_motor_set_speed(COEFF_VITESSE*(speed - ROTATION_COEFF * speed_correction));
//		left_motor_set_speed(COEFF_VITESSE*(speed + ROTATION_COEFF * speed_correction));

		if (get_distance_cm() == GOAL_DISTANCE)
		{
			right_motor_set_speed(0);
			left_motor_set_speed(0);
			position = POSITION_REACHED;
		}


		chThdSleepUntilWindowed(time, time + MS2ST(1));

//		test_capteur();
//		chThdSleepMilliseconds(100);
	}
}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}

void test_capteur(void)
{
	int left;
	int right;
	calibrate_ir();
	left = get_prox(6);
	right= get_prox(3);
	if(left!=0)
	{
		palClearPad(GPIOD, GPIOD_LED7);
	}else
	{
		palSetPad(GPIOD, GPIOD_LED7);
	}
//	chprintf((BaseSequentialStream *)&SDU1, "valeur capteur6= %d \n", left);

}

