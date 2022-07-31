/*
 * Encoder.c
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#include "EncoderInt.h"


void Encoder_Init(TIM_TypeDef *pTim3Handle, TIM_TypeDef *pTim4Handle)
{
	_sEncoderX.tim = pTim3Handle;
	_sEncoderX.float_LastTheta = 0;
	_sEncoderX.float_LastSpeed = 0;
	_sEncoderX.xLastTick = 0;
	_sEncoderX.uint16_LastCnt = 0;
	_sEncoderX.Bool_IsValueToBeDiscarded = FALSE;

	_sEncoderY.tim = pTim4Handle;
	_sEncoderY.float_LastTheta = 0;
	_sEncoderY.float_LastSpeed = 0;
	_sEncoderY.xLastTick = 0;
	_sEncoderY.uint16_LastCnt = 0;
	_sEncoderY.Bool_IsValueToBeDiscarded = FALSE;
}

void Encoder_GetEncoderSpeed(Encoder_te Enum_Encoder, float *pfloat_Speed)
{
	float float_AngSpeedUnfiltered = 0;
	float float_AngSpeedFiltered = 0;
	uint32_t xCurrentTick = HAL_GetTick();

	Encoder_Handle_TypeDef *pEncoder_Handle;

	if (ENCODER_X == Enum_Encoder)
	{
		pEncoder_Handle = &_sEncoderX;
	}
	else if (ENCODER_Y == Enum_Encoder)
	{
		pEncoder_Handle = &_sEncoderY;
	}

	float float_ThetaK_1 = pEncoder_Handle->float_LastTheta;
	float Prev_Speed = pEncoder_Handle->float_LastSpeed;
	uint32_t xLastTick = pEncoder_Handle->xLastTick;

	float_ThetaK = (float)(pEncoder_Handle->tim->CNT)/((float)(RESOLUTION*CPR))*2*PI;	/* Angle position in rad */
	float float_ThetaK = (float)(pEncoder_Handle->tim->CNT + *puint16_CntOverflow)/((float)(RESOLUTION*CPR))*2*PI;	/* Angle position in rad */

	if (FALSE == pEncoder_Handle->Bool_IsValueToBeDiscarded)
	{
	float_AngSpeedUnfiltered = ((float_ThetaK - float_ThetaK_1) / (xCurrentTick - xLastTick)) * 1000;	/* Angular velocity calculation rad/s */

	/* FEATURE : correzione della velocità nell'evenienza ci sia un valore non veritiero, a cavallo del reset del counter  */

	/* FIXME : determine if this low-pass filter is appropriate */
	float_AngSpeedFiltered = ((1 - SPEED_FILTER) * float_AngSpeedUnfiltered) + (SPEED_FILTER * Prev_Speed);
	}

	/* Update Values */
	*pfloat_Speed = float_AngSpeedFiltered;
	pEncoder_Handle->float_LastTheta = float_ThetaK;
	pEncoder_Handle->float_LastSpeed = float_AngSpeedFiltered;
	pEncoder_Handle->uint16_LastCnt = pEncoder_Handle->tim->CNT;
	pEncoder_Handle->xLastTick = xCurrentTick;
	pEncoder_Handle->Bool_IsValueToBeDiscarded = FALSE;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		_sEncoderX.Bool_IsValueToBeDiscarded = TRUE;
	}
	else if (htim == &htim4)
	{
		_sEncoderY.Bool_IsValueToBeDiscarded = TRUE;
	}
}
