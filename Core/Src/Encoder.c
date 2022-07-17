/*
 * Encoder.c
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#include "EncoderInt.h"


void get_Encoder_Speed(Encoder_Handle_TypeDef *hdl, float *speed){
	float theta_k_1 = hdl->theta;
	float Prev_Speed = hdl->Speed;
	unsigned long Time_Saved = hdl->Time;
	unsigned long Time_Now = __MICROS();

	float Ang_Speed_Unfiltered;
	float Ang_Speed;

	float theta_k = (float)(hdl->tim->CNT)/((float)(RESOLUTION*CPR))*2*PI;	/* Angle position in rad */

	Ang_Speed_Unfiltered = (theta_k - theta_k_1) / (Time_Now - Time_Saved);	/* Angular velocity calculation rad/us */
	/* FEATURE : correzione della velocità nell'evenienza ci sia un valore non veritiero, a cavallo del reset del counter  */

	/* FIXME : determine if this low-pass filter is appropriate */
	Ang_Speed = ((1 - SPEED_FILTER) * Ang_Speed_Unfiltered) + (SPEED_FILTER * Prev_Speed);

	*speed = Ang_speed;
	hdl->theta = theta_k;
	hdl->Speed = Ang_speed;
	hdl->Time = Time_Now;
}
