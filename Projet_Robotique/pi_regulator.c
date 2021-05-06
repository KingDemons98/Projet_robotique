#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

//static THD_WORKING_AREA(waPiRegulator, 256);
//static THD_FUNCTION(PiRegulator, arg) {
//
//    chRegSetThreadName(__FUNCTION__);
//    (void)arg;
//
//    systime_t time;
////    int16_t speed =0;
//
//
//
//    while(1){
//        time = chVTGetSystemTime();
//
//        /*
//		*	To complete
//		*/
//
//
//
//        //applies the speed from the PI regulator
////		 right_motor_set_speed(speed - ROTATION_COEFF * speed_correction);
////		 left_motor_set_speed(speed + ROTATION_COEFF * speed_correction);
//
//        //100Hz
//        chThdSleepUntilWindowed(time, time + MS2ST(10));
//    }
//}
//
//void pi_regulator_start(void){
//	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
//}

int16_t pi_regulator(float distance, float goal)
{
	float speed = 0;
	static float somme_erreur = 0;
	float erreur = distance - goal;
	if(fabs(erreur)< ERROR_THRESHOLD)
	{
		return 0;
	}
	somme_erreur += erreur;
	if (somme_erreur > MAX_SUM_ERROR)
	{
		somme_erreur = MAX_SUM_ERROR;
	}
	if (somme_erreur < -MAX_SUM_ERROR)
	{
		somme_erreur = -MAX_SUM_ERROR;
	}
	speed = KP *erreur + KI*somme_erreur;
	return (int16_t)speed;
}
