/*
 * Battery_StatusExt.h
 *
 *  Created on: 21 lug 2022
 *      Author: Federico Altieri
 */

#ifndef COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_
#define COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_

/* Public Functions */

HAL_StatusTypeDef get_Battery_Status (ADC_HandleTypeDef hadc, float *stat);
void Battery_Status_ADC_Cal(ADC_HandleTypeDef *hVrefint);

#endif /* COMPONENTS_BATTERY_STATUS_BATTERY_STATUSEXT_H_ */
