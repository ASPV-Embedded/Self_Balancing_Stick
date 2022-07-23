/*
 * Battery_StatusExt.h
 *
 *  Created on: 21 lug 2022
 *      Author: Federico Altieri
 */

#ifndef COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_
#define COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_

#include "main.h"
#include "adc.h"

/* Public Functions */

Error_t Battery_Status_GetBatteryChargePercentage (ADC_HandleTypeDef *pAdcHandle, float *float_BatteryPercentage);

void Battery_Status_AdcCal(ADC_HandleTypeDef *pAdcVrefintHandle);

#endif /* COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_ */
