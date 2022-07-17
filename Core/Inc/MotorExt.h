/*
 * MotorExt.h
 *
 *  Created on: Jul 9, 2022
 *      Author: Federico Altieri
 */

#ifndef __MOTOREXT_H_
#define __MOTOREXT_H_

#include "stm32f4xx_hal.h"


typedef enum __Motor_Status {
	CW = 1,
	CCW = 0,
	STOPPED = -1
} Motor_Status_TypeDef;


typedef struct __Motor_Handle
{
	/* Timer handler */
	TIM_HandleTypeDef *htim;
	uint32_t TIM_CHANNEL;
	/* Spin Sense Pin */
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pinx;
	/* Brake Pin */
	GPIO_TypeDef *GPIOy;
	uint16_t GPIO_Piny;

	float duty_cycle;
	uint8_t spin_direction;
	uint8_t brake;

} Motor_Handle_TypeDef;


void MotorInit (Motor_Handle_TypeDef *hdl);
Motor_Status_TypeDef set_DutyCycle (Motor_Handle_TypeDef *hdl, float duty_cycle);
Motor_Status_TypeDef set_SpinDirection (Motor_Handle_TypeDef *hdl, Motor_Status_TypeDef direction);
void set_Brake (Motor_Handle_TypeDef *hdl, GPIO_PinState state);





#endif /* __MOTOREXT_H_ */
