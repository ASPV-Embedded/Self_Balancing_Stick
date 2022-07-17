/*
 * EncoderExt.h
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#ifndef __ENCODEREXT_H_
#define __ENCODEREXT_H_

#include "stm32f446xx.h"

typedef struct __Encoder_Handle {
	/* Timer Handler */
	TIM_TypeDef *tim;
	/* Variables */
	float theta = 0;
	float Speed = 0;
	unsigned long Time = 0;
	/* IMPORTANT : the DIR bit is stored in tim->CR1 */
} Encoder_Handle_TypeDef;

/* Public Functions */

void get_Encoder_Speed(float *speed);

#endif /* __ENCODEREXT_H_ */
