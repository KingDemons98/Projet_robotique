#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



#include <move_control.h>
#include <sensors/proximity.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <camera/po8030.h>


// module de gestion du d�placement avec la cam�ra et les capteurs de proximit�

static THD_WORKING_AREA(waMoveControl, 256);
static THD_FUNCTION(MoveControl, arg)
{
	chRegSetThreadName(__FUNCTION__);
	(void)arg;
	systime_t time;


}

void move_control_start(void)
{
	chThdCreateStatic(waMoveControl, sizeof(waMoveControl), NORMALPRIO, MoveControl, NULL);
}
