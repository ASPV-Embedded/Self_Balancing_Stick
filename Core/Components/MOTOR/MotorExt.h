/*
 * MotorExt.h
 *
 *  Created on: Jul 9, 2022
 *      Author: Federico Altieri
 */

#ifndef __MOTOREXT_H_
#define __MOTOREXT_H_

#include "main.h"

typedef enum
{
	MOTOR_STATUS_CW = 1,
	MOTOR_STATUS_CCW = 0,
	MOTOR_STATUS_STOPPED = -1,
	MOTOR_STATUS_INVALID = 0xFF,
} Motor_Status_te;

typedef enum
{
	BRAKE_OFF = 0,
	BRAKE_ON = 1
} Motor_BrakeState_te;

typedef struct
{
	/* Timer handler */
	TIM_HandleTypeDef *htim;
	uint32_t uint32_TimChannel;

	/* Spin Direction Pin */
	GPIO_TypeDef *GPIO_Port_SpinDirection;
	uint16_t GPIO_Pin_SpinDirection;

	float float_DutyCycle;
	uint8_t uint8_SpinDirection;

} Motor_Handle_t;

typedef struct
{
	/* Brake Pin */
	GPIO_TypeDef *GPIO_Port_Brake;
	uint16_t GPIO_Pin_Brake;

	Motor_BrakeState_te Enum_BrakeState;

} Motor_Brake_t;


void Motor_Init (Motor_Handle_t *MotorHandle, Motor_Brake_t *psMotorBrake);

Motor_Status_te Motor_SetDutyCycle (Motor_Handle_t *MotorHandle, float float_DutyCycle);

Motor_Status_te Motor_SetSpinDirection (Motor_Handle_t *MotorHandle, Motor_Status_te Enum_Direction);

void Motor_SetBrake(Motor_Brake_t *psMotorBrake, Motor_BrakeState_te Enum_BrakeState);

#endif /* __MOTOREXT_H_ */
