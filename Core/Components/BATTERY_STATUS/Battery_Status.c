/*
 * Battery_Status.c
 *
 *  Created on: 21 lug 2022
 *      Author: Federico Altieri
 */

#include "Battery_StatusInt.h"

/**
 * @brief  Read and convert from the ADC the battery voltage coming
 *         from a voltage divider.
 * @param  [in] ADC handler address
 * @param  [out] Battery Charge value in percentage
 */

Error_t Battery_Status_GetBatteryChargePercentage (ADC_HandleTypeDef *pAdcHandle, float *float_BatteryPercentage)
{
	Error_t Error = E_NOT_OK;

	if ((NULL != pAdcHandle) && (NULL != float_BatteryPercentage))
	{
		Error = E_OK;
	}

	if (E_OK == Error)
	{
		uint16_t uint16_ADC_Value = 0;
		float float_Vin = 0;

		ADC_Select_CH14();
		HAL_ADC_Start(pAdcHandle);

		if (HAL_ADC_PollForConversion(pAdcHandle, 100) != HAL_OK)
		{
//			Error_Handler();
			Error = E_NOT_OK;
		}
		else
		{
			uint16_ADC_Value = (uint16_t)HAL_ADC_GetValue(pAdcHandle);
			float_Vin = (float) uint16_ADC_Value * _VDDA / ADC_GET_RESOLUTION(pAdcHandle); /* AdcHandle->Init->Resolution */
		}

		if (E_OK == Error)
		{
			*float_BatteryPercentage = float_Vin / _VDDA * 100.0;
			HAL_ADC_Stop(pAdcHandle);
		}
	}

	return Error;
}

/**
 * @brief  Calculate VDDA.
 * @param  ADC handler address for Vrefint channel
 */

void Battery_Status_AdcCal(ADC_HandleTypeDef *pAdcVrefintHandle)
{
	/*https://electronics.stackexchange.com/questions/449478/adc-calibration-vdd-calculation*/
	/* IMPORTANT: minimum sampling time 10us */
	ADC_Select_VREFINT();
	HAL_ADC_Start(pAdcVrefintHandle);
	_VDDA = VREFINT_CAL_VREF * (*_pVrefin_cal) / pAdcVrefintHandle->Instance->DR;/* Vrefin_cal is a binary number, VDDA in mV */
	HAL_ADC_Stop(pAdcVrefintHandle);
}

void ADC_Select_VREFINT()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}

void ADC_Select_CH14()
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
}
