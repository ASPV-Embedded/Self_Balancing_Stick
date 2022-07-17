/*
 * Motor.c
 *
 *  Created on: Jul 9, 2022
 *      Author: Federico Altieri
 */

#include "MotorInt.h"

void MotorInit (Motor_Handle_TypeDef *hdl){
	/* Operazione singola valida per entrambi i motori, timer unico */
	HAL_TIM_PWM_Stop(hdl->htim, hdl->TIM_CHANNEL);
	hdl->htim->Instance->ARR = ARR_VALUE;
	hdl->htim->Instance->PSC = PSC_VALUE;
}

Motor_Status_TypeDef set_DutyCycle(Motor_Handle_TypeDef *hdl, float duty_cycle) {
	hdl->duty_cycle = abs(duty_cycle);
	if (duty_cycle == 0) {
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
		/* Immediate update */
		hdl->htim->Instance->EGR = TIM_EGR_UG;
		return STOPPED;

	} else if (duty_cycle < 0) {
		HAL_GPIO_WritePin(hdl->GPIOx, hdl->GPIO_Pinx, CCW);
		return CCW;

	} else {
		HAL_GPIO_WritePin(hdl->GPIOy, hdl->GPIO_Piny, CW);
		return CW;

	}

	ccr = (uint16_t) (abs(duty_cycle) * (float) (1 + ARR_VALUE))
	__HAL_TIM_SET_COMPARE(hdl->htim, hdl->TIM_CHANNEL, ccr);
	/* Immediate update */
	hdl->htim->Instance->EGR = TIM_EGR_UG;
}

Motor_Status_TypeDef set_SpinDirection(Motor_Handle_TypeDef *hdl, Motor_Status_TypeDef direction) {

	if (direction == CW) {
		hdl->spin_direction = CW;
		HAL_GPIO_WritePin(hdl->GPIOx, hdl->GPIO_Pinx, CW);
		return CW;

	} else if (direction == CCW) {
		hdl->spin_direction = CCW;
		HAL_GPIO_WritePin(hdl->GPIOx, hdl->GPIO_Pinx, CCW);
		return CCW;

	} else {
		/*Stop the motor from PWM (set dc to 0)*/
		__HAL_TIM_SET_COMPARE(hdl->htim, hdl->TIM_CHANNEL, 0);
		/* Immediate update */
		hdl->htim->Instance->EGR = TIM_EGR_UG;
		return STOPPED;
	}
}

void set_Brake(Motor_Handle_TypeDef *hdl, GPIO_PinState state) {
	hdl->brake = state;
	HAL_GPIO_WritePin(hdl->GPIOy, hdl->GPIO_Piny, state);
}

