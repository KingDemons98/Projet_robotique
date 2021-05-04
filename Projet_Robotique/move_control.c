#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <stdbool.h>

#include <move_control.h>
#include <proximity.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <camera/po8030.h>
#include <main.h>


// module de gestion du déplacement avec la caméra et les capteurs de proximité
// module de gestion du d�placement avec la cam�ra et les capteurs de proximit�

static BSEMAPHORE_DECL(block_passed, TRUE);				//permet de signaler quand le bloc est d�pass�
static BSEMAPHORE_DECL(position_reached, TRUE);			//signale quand le robot est a la bonne distance du prochain bloc

static THD_WORKING_AREA(waMoveControl, 256);
static THD_FUNCTION(MoveControl, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;
	chprintf((BaseSequentialStream *)&SDU1, "Test\n");
//	int left = test_capteur();
//	chprintf((BaseSequentialStream *)&SDU1, "valeur capteur6= %d\n", left);
	chThdSleepMilliseconds(100);
}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}
int test_capteur(void)
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
	return left;
}

