/*
 * Motor.c
 *
 *  Created on: Jul 9, 2022
 *      Author: Federico Altieri
 */

#include "MotorInt.h"


/*
 * @brief Initialize Motor
 * @param Pointer to Motor Handle structure
 */
void Motor_Init (Motor_Handle_t *MotorHandle, Motor_Brake_t *psMotorBrake)
{
	Motor_BrakeState_te Enum_BrakeState = BRAKE_ON;
	Motor_SetBrake(psMotorBrake, Enum_BrakeState);

	/* Operazione da effettuare per entrambi i motori, timer unico ma diversi canali */
	HAL_TIM_PWM_Stop(MotorHandle->htim, MotorHandle->uint32_TimChannel);
	MotorHandle->htim->Instance->ARR = ARR_VALUE;
	MotorHandle->htim->Instance->PSC = PSC_VALUE;
	HAL_TIM_PWM_Start(MotorHandle->htim, MotorHandle->uint32_TimChannel);
	Motor_SetSpinDirection(MotorHandle, MOTOR_STATUS_STOPPED);
}

/*
 * @brief Set PWM signal Duty Cycle
 * @param Pointer to Motor Handle structure
 * @param Duty Cycle, PID output value [-1.0; 1.0]
 */
Motor_Status_te Motor_SetDutyCycle(Motor_Handle_t *MotorHandle, float float_DutyCycle)
{
	Motor_Status_te Enum_MotorStatus = MOTOR_STATUS_INVALID;
	uint16_t uint16_Ccr = 0;

	MotorHandle->float_DutyCycle = fabs(float_DutyCycle);

	if (float_DutyCycle == 0)
	{
		Enum_MotorStatus = Motor_SetSpinDirection(MotorHandle, MOTOR_STATUS_STOPPED);
	}
	else if (float_DutyCycle < 0)
	{
		Enum_MotorStatus = Motor_SetSpinDirection(MotorHandle, MOTOR_STATUS_CCW);
	}
	else
	{
		Enum_MotorStatus = Motor_SetSpinDirection(MotorHandle, MOTOR_STATUS_CW);
	}
	/* Calculate value of CCR register */
	uint16_Ccr = (uint16_t) (MotorHandle->float_DutyCycle * (float)(ARR_VALUE));
	__HAL_TIM_SET_COMPARE(MotorHandle->htim, MotorHandle->uint32_TimChannel, (ARR_VALUE - uint16_Ccr));

	/* Immediate update */
	MotorHandle->htim->Instance->EGR = TIM_EGR_UG;

	return Enum_MotorStatus;
}

/*
 * @brief Set motor spin direction
 * @param Pointer to Motor Handle structure
 * @param Spin Direction (Clockwise, CounterClockwise or Stopped)
 */
Motor_Status_te Motor_SetSpinDirection(Motor_Handle_t *MotorHandle, Motor_Status_te Enum_Direction)
{
	if (Enum_Direction == MOTOR_STATUS_CW)
	{
		MotorHandle->uint8_SpinDirection = MOTOR_STATUS_CW;
		HAL_GPIO_WritePin(MotorHandle->GPIO_Port_SpinDirection, MotorHandle->GPIO_Pin_SpinDirection, GPIO_PIN_SET);
		return MOTOR_STATUS_CW;
	}
	else if (Enum_Direction == MOTOR_STATUS_CCW)
	{
		MotorHandle->uint8_SpinDirection = MOTOR_STATUS_CCW;
		HAL_GPIO_WritePin(MotorHandle->GPIO_Port_SpinDirection, MotorHandle->GPIO_Pin_SpinDirection, GPIO_PIN_RESET);
		return MOTOR_STATUS_CCW;
	}
	else
	{
		/*Stop the motor from PWM (set dc to 0)*/
		__HAL_TIM_SET_COMPARE(MotorHandle->htim, MotorHandle->uint32_TimChannel, ARR_VALUE);
		/* Immediate update */
		MotorHandle->htim->Instance->EGR = TIM_EGR_UG;
		return MOTOR_STATUS_STOPPED;
	}
}

/*
 * @brief Set Brake (On or Off)
 * @param Pointer to Motor Brake structure
 * @param Brake state to apply
 */
void Motor_SetBrake(Motor_Brake_t *psMotorBrake, Motor_BrakeState_te Enum_BrakeState)
{
	GPIO_PinState PinState = GPIO_PIN_RESET;
	psMotorBrake->Enum_BrakeState = Enum_BrakeState;

	if (BRAKE_OFF == Enum_BrakeState)
	{
		PinState = GPIO_PIN_SET;
	}
	else if (BRAKE_ON == Enum_BrakeState)
	{
		PinState = GPIO_PIN_RESET;
	}

	HAL_GPIO_WritePin(psMotorBrake->GPIO_Port_Brake, psMotorBrake->GPIO_Pin_Brake, PinState);
}
