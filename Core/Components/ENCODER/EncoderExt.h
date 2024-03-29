/*
 * EncoderExt.h
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#ifndef __ENCODEREXT_H_
#define __ENCODEREXT_H_

#include "main.h"

typedef struct
{
	/* Timer Handler */
	TIM_TypeDef *tim;

	/* Variables */
	float float_LastTheta;
	float float_LastSpeed;
	uint32_t xLastTick;
	uint16_t uint16_LastCnt;
	Bool_t Bool_IsValueToBeDiscarded;
} Encoder_Handle_TypeDef;

typedef enum
{
	ENCODER_X = 0,
	ENCODER_Y = 1,
	ENCODER_NUMBER = 2
} Encoder_te;

/* Public Functions */

Error_t Encoder_Init(TIM_TypeDef *pTim3Handle, TIM_TypeDef *pTim4Handle);
Error_t Encoder_GetEncoderSpeed(Encoder_te Enum_Encoder, float *pfloat_Speed);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* __ENCODEREXT_H_ */
