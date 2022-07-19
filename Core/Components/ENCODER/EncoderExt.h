/*
 * EncoderExt.h
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#ifndef __ENCODEREXT_H_
#define __ENCODEREXT_H_

#include "main.h"

typedef struct {
	/* Timer Handler */
	TIM_TypeDef *tim;
	/* Variables */
	float theta;
	float Speed;
	unsigned long Time;
	/* IMPORTANT : the DIR bit is stored in tim->CR1 */
} Encoder_Handle_TypeDef;

/* Public Functions */

void get_Encoder_Speed(Encoder_Handle_TypeDef *hdl, float *speed);

#endif /* __ENCODEREXT_H_ */
