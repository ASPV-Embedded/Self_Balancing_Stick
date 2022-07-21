/*
 * Battery_Status.c
 *
 *  Created on: 21 lug 2022
 *      Author: Federico Altieri
 */

/* ADC1 IN14 PC4 Alias BAT_STATUS */

#include "Battery_StatusInt.h"


HAL_StatusTypeDef get_Battery_Status (ADC_HandleTypeDef *hadc, float *stat){

	uint16_t ADC_Value;
	float Vin;

	/* FIXME: in the main ??*/
	HAL_ADC_Start(hadc);
	/* FIXME: Calibrate the ADC for better accuracy */

	if (HAL_ADC_PollForConversion(hadc, 100) != HAL_OK){
		 Error_Handler(); /* Return Status */
	} else {
		ADC_Value = (uint16_t)HAL_ADC_GetValue(hadc);
		Vin = (float) ADC_Value * VREF / ADC_GET_RESOLUTION(hadc); /* hadc->Init->Resolution */

		return HAL_OK;
	}

	*stat = Vin / VREF * 100.0;
}





