/*
 * Encoder.c
 *
 *  Created on: 11 lug 2022
 *      Author: Federico Altieri
 */

#include "EncoderInt.h"


Error_t Encoder_Init(TIM_TypeDef *pTim3Handle, TIM_TypeDef *pTim4Handle)
{
	Error_t Error = E_NOT_OK;

	if(NULL != pTim3Handle && NULL != pTim4Handle)
	{
		Error = E_OK;
	}

	if(E_OK == Error)
	{
		_sEncoderX.tim = pTim4Handle;
		_sEncoderX.float_LastTheta = 0;
		_sEncoderX.float_LastSpeed = 0;
		_sEncoderX.xLastTick = 0;
		_sEncoderX.uint16_LastCnt = 0;
		_sEncoderX.Bool_OnTimerCntReset = FALSE;

		_sEncoderY.tim = pTim3Handle;
		_sEncoderY.float_LastTheta = 0;
		_sEncoderY.float_LastSpeed = 0;
		_sEncoderY.xLastTick = 0;
		_sEncoderY.uint16_LastCnt = 0;
		_sEncoderY.Bool_OnTimerCntReset = FALSE;
	}

	return Error;
}

Error_t Encoder_GetEncoderSpeed(Encoder_te Enum_Encoder, float *pfloat_Speed)
{
	Error_t Error = E_NOT_OK;

	if (NULL != pfloat_Speed)
	{
		Error = E_OK;
	}

	if (E_OK == Error)
	{
		float float_ThetaK = 0;
		float float_ThetaK_1 = 0;
		float float_PrevSpeed = 0;
		float float_AngSpeedUnfiltered = 0;
		float float_AngSpeedFiltered = 0;
		uint32_t xCurrentTick = HAL_GetTick();
		uint32_t xLastTick = 0;

		Encoder_Handle_TypeDef *pEncoder_Handle;

		if (ENCODER_X == Enum_Encoder)
		{
			pEncoder_Handle = &_sEncoderX;
		}
		else if (ENCODER_Y == Enum_Encoder)
		{
			pEncoder_Handle = &_sEncoderY;
		}

		float_ThetaK_1 = pEncoder_Handle->float_LastTheta;
		float_PrevSpeed = pEncoder_Handle->float_LastSpeed;
		xLastTick = pEncoder_Handle->xLastTick;

		float_ThetaK = (float)(pEncoder_Handle->tim->CNT)/((float)(RESOLUTION*CPR))*2*PI;	/* Angle position in rad */

		/* Calcolo della velocità */
		if (FALSE == pEncoder_Handle->Bool_OnTimerCntReset)
		{
			float_AngSpeedUnfiltered = ((float_ThetaK - float_ThetaK_1) / (xCurrentTick - xLastTick)) * 1000;	/* Angular velocity calculation rad/s */
		}
		else
		{
			if (pEncoder_Handle->tim->CNT < pEncoder_Handle->uint16_LastCnt)
			{
				float_AngSpeedUnfiltered = ((((float)pEncoder_Handle->tim->ARR - (float)pEncoder_Handle->uint16_LastCnt + (float)pEncoder_Handle->tim->CNT)/((float)(RESOLUTION*CPR))*2*PI)   / (xCurrentTick - xLastTick)) * 1000;	/* Angular velocity calculation rad/s */
			}
			else
			{
				float_AngSpeedUnfiltered = ((((float)pEncoder_Handle->tim->CNT - (float)pEncoder_Handle->tim->ARR - (float)pEncoder_Handle->uint16_LastCnt)/((float)(RESOLUTION*CPR))*2*PI)   / (xCurrentTick - xLastTick)) * 1000;	/* Angular velocity calculation rad/s */
			}
		}

		/* FIXME : determine if this low-pass filter is appropriate */
		float_AngSpeedFiltered = ((1 - SPEED_FILTER) * float_AngSpeedUnfiltered) + (SPEED_FILTER * float_PrevSpeed);
		*pfloat_Speed = float_AngSpeedFiltered;

		/* Update Values */
		pEncoder_Handle->float_LastTheta = float_ThetaK;
		pEncoder_Handle->float_LastSpeed = float_AngSpeedFiltered;
		pEncoder_Handle->uint16_LastCnt = pEncoder_Handle->tim->CNT;
		pEncoder_Handle->xLastTick = xCurrentTick;
		pEncoder_Handle->Bool_OnTimerCntReset = FALSE;
	}

	return Error;
}

void Encoder_InterruptCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim3)
	{
		_sEncoderY.Bool_OnTimerCntReset = TRUE;
	}
	else if (htim == &htim4)
	{
		_sEncoderX.Bool_OnTimerCntReset = TRUE;
	}
}
