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
 * @param  ADC handler address
 * @retval Battery Status in percentage
 */

HAL_StatusTypeDef get_Battery_Status (ADC_HandleTypeDef *hadc, float *stat){

	uint16_t ADC_Value;
	float Vin;


	ADC_Select_CH14();
	HAL_ADC_Start(hadc);

	if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK){
		 Error_Handler();
	} else {
		ADC_Value = (uint16_t)HAL_ADC_GetValue(hadc);
		Vin = (float) ADC_Value * VDDA / ADC_GET_RESOLUTION(hadc); /* hadc->Init->Resolution */

		return HAL_OK;
	}

	*stat = Vin / VDDA * 100.0;
	HAL_ADC_Stop(hadc);
}


/**
 * @brief  Calculate VDDA.
 * @param  ADC handler address for Vrefint channel
 * @retval VDDA value
 */

void Battery_Status_ADC_Cal(ADC_HandleTypeDef *hVrefint) {
	/*https://electronics.stackexchange.com/questions/449478/adc-calibration-vdd-calculation*/
	/* IMPORTANT: minimum sampling time 10us */
	ADC_Select_VREFINT();
	HAL_ADC_Start(hadc);
	VDDA = VREFINT_CAL_VREF * (*Vrefin_cal) / hVrefint->Instance->DR;/* Vrefin_cal is a binary number, VDDA in mV */
	HAL_ADC_Stop(hadc);
}



