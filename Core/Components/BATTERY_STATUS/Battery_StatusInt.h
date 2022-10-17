/*
 * Battery_StatusInt.h
 *
 *  Created on: 21 lug 2022
 *      Author: Federico Altieri
 */

#ifndef COMPONENTS_BATTERY_STATUS_BATTERY_STATUSINT_H_
#define COMPONENTS_BATTERY_STATUS_BATTERY_STATUSINT_H_

#include "Battery_StatusExt.h"

uint16_t *_pVrefin_cal = VREFINT_CAL_ADDR;

float _VDDA;

void ADC_Select_VREFINT();
void ADC_Select_CH14();

#endif /* COMPONENTS_BATTERY_STATUS_BATTERY_STATUSINT_H_ */
