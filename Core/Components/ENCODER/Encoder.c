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
	_sEncoderY.tim = pTim4Handle;
}

void Encoder_GetEncoderSpeed(Encoder_te Enum_Encoder, float *pfloat_Speed)
{
	float float_AngSpeedUnfiltered = 0;
	float float_AngSpeedFiltered = 0;
	uint32_t xCurrentTick = HAL_GetTick();

	Encoder_Handle_TypeDef *pEncoder_Handle;
	uint16_t *puint16_CntOverflow;

	if (ENCODER_X == Enum_Encoder)
	{
		pEncoder_Handle = &_sEncoderX;
		puint16_CntOverflow = &_uint16_Tim3CntOverflow;
	}
	else if (ENCODER_Y == Enum_Encoder)
	{
		pEncoder_Handle = &_sEncoderY;
		puint16_CntOverflow = &_uint16_Tim4CntOverflow;
	}

	float float_ThetaK_1 = pEncoder_Handle->float_LastTheta;
	float Prev_Speed = pEncoder_Handle->float_LastSpeed;
	uint32_t xLastTick = pEncoder_Handle->xLastTick;

	float float_ThetaK = (float)(pEncoder_Handle->tim->CNT + *puint16_CntOverflow)/((float)(RESOLUTION*CPR))*2*PI;	/* Angle position in rad */

	float_AngSpeedUnfiltered = ((float_ThetaK - float_ThetaK_1) / (xCurrentTick - xLastTick)) * 1000;	/* Angular velocity calculation rad/s */

	/* FEATURE : correzione della velocità nell'evenienza ci sia un valore non veritiero, a cavallo del reset del counter  */

	/* FIXME : determine if this low-pass filter is appropriate */
	float_AngSpeedFiltered = ((1 - SPEED_FILTER) * float_AngSpeedUnfiltered) + (SPEED_FILTER * Prev_Speed);

	/* Update Values */
	*pfloat_Speed = float_AngSpeedFiltered;
	pEncoder_Handle->float_LastTheta = float_ThetaK;
	pEncoder_Handle->float_LastSpeed = float_AngSpeedFiltered;
	pEncoder_Handle->uint16_LastCnt = pEncoder_Handle->tim->CNT;
	pEncoder_Handle->xLastTick = xCurrentTick;

	/* Reset uint16_CntOverflow to 0*/
	*puint16_CntOverflow = 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		//FIXME: Capire quanto vale CNT in questo istante
		_uint16_Tim3CntOverflow = abs(htim->Instance->CNT - _sEncoderX.uint16_LastCnt);
	}
	else if (htim == &htim4)
	{
		//FIXME: Capire quanto vale CNT in questo istante
		_uint16_Tim4CntOverflow = abs(htim->Instance->CNT - _sEncoderY.uint16_LastCnt);
	}
}
