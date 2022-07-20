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
	float float_LastTheta;
	float float_LastSpeed;
	uint32_t xLastTick;
	uint16_t uint16_LastCnt;
	/* IMPORTANT : the DIR bit is stored in tim->CR1 */
} Encoder_Handle_TypeDef;

typedef enum
{
	ENCODER_X = 0,
	ENCODER_Y = 1,
	ENCODER_NUMBER = 2
} Encoder_te;

/* Public Functions */

void Encoder_Init(TIM_TypeDef *pTim3Handle, TIM_TypeDef *pTim4Handle);
void Encoder_GetEncoderSpeed(Encoder_te Enum_Encoder, float *pfloat_Speed);

void TIM3_IRQHandler(void);
void TIM4_IRQHandler(void);

#endif /* __ENCODEREXT_H_ */
