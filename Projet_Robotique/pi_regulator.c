#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>


int16_t pi_regulator_blocks(float distance, float goal)
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
	speed = KP *erreur;
//			+ KI*somme_erreur;
	return (int16_t)speed;
}

//int16_t pi_regulator_capteurs(int capteur_right, int capteur_left)
//{
//
//	return speed;
//}
