/*
 * controller.c
 *
 *  Created on: 11 July 2022
 *      Author: Team ASPV
 *
 *  Implements PID Controller functionality
 */

#include "controller_Int.h"

/*
 * @brief initializes a new controller object
 * @param psController - Controller instance
 * @param Enum_ControllerAxis - The direction associated with this controller, x or y
 * @param float_AngleSetpoint - Set point
 * @param float_Kp - PID imu proportional gain
 * @param float_Ki - PID imu integral gain
 * @param float_Kd - PID imu derivative gain
 * @param float_Ks - PID rotor speed gain
 * @param float_Friction
 * @param float_angle_Integral_Max - Max value of integral error
 * @param float_angle_Integral_Min - Min value of integral error
 *
 */
Error_t Controller_Init(Controller_t *psController,
						ControllerAxis_te Enum_ControllerAxis,
						float float_AngleSetpoint,
						float float_Kp,
						float float_Ki,
						float float_Kd,
						float float_Ks,
						float float_Friction,
						float float_angle_Integral_Max,
						float float_angle_Integral_Min)
{
	Error_t Error = E_OK;

	if (NULL == psController)
	{
		Error = E_NOT_OK;
	}
	else
	{
		psController->Enum_ControllerAxis = Enum_ControllerAxis;

		psController->float_AngleSetpoint = float_AngleSetpoint;
		psController->float_Kp = float_Kp;
		psController->float_Ki = float_Ki;
		psController->float_Kd = float_Kd;
		psController->float_Ks = float_Ks;

		psController->float_FrictionValue = float_Friction;

		psController->float_AngleIntegralMax = float_angle_Integral_Max;
		psController->float_AngleIntegralMin = float_angle_Integral_Min;

		psController->float_AngleIntegralError = 0;
		psController->float_LastError = 0;
		psController->uint32_LastTick = 0;
		psController->float_PIDValue = 0;

		psController->Bool_InputSaturation = FALSE;
		psController->Bool_FirstMeasureDone = FALSE;
	}
	return Error;
}

/*
 * @brief calculate a motor voltage based on encoder, motor, and imu data
 * @param psController - Controller structure
 * @param pfloat_VoltageValue - PID output value
 */
Error_t Controller_GetPIDVoltageValue(uint32_t uint32_CurrentTick, MPU6050_Angles_t sAngles,
									  Controller_t *psController, float *pfloat_VoltageValue)
{
	Error_t Error = E_OK;

	static float float_CurrentAngle = 0;
	float float_AngleError = 0;
	float float_AngleDerivativeError = 0;
	float float_DeltaT = 0;
	float float_WheelSpeed = 0;
//	float float_friction = 0;

	if (CONTROLLER_AXIS_X == psController->Enum_ControllerAxis)
	{
		float_CurrentAngle = sAngles.AngleX;
		Encoder_GetEncoderSpeed(ENCODER_X, &float_WheelSpeed);
	}
	else if (CONTROLLER_AXIS_Y == psController->Enum_ControllerAxis)
	{
		float_CurrentAngle = sAngles.AngleY;
		Encoder_GetEncoderSpeed(ENCODER_Y, &float_WheelSpeed);
	}
	else
	{
		Error = E_NOT_OK;
	}

	if (E_OK == Error)
	{
		if (TRUE == psController->Bool_FirstMeasureDone)
		{
			// calculate angle error
			float_AngleError = psController->float_AngleSetpoint - float_CurrentAngle;

			// calculate DeltaT [s]
			float_DeltaT = (float)(uint32_CurrentTick - psController->uint32_LastTick) / 1000;
			psController->uint32_LastTick = uint32_CurrentTick;

			// limited integrator
			if (FALSE == psController->Bool_InputSaturation)
			{
				// integral angle error
				psController->float_AngleIntegralError += (float_AngleError * float_DeltaT);
			}

			// derivative angle error
			float_AngleDerivativeError = (float_AngleError - psController->float_LastError) / float_DeltaT;
			psController->float_LastError = float_AngleError;

			// calculation of Proportional, Integral and Derivative actions
			float float_PropValue = psController->float_Kp * float_AngleError;
			float float_IntValue = psController->float_Ki * psController->float_AngleIntegralError;
			float float_DerivValue = psController->float_Kd * float_AngleDerivativeError;
			//		float S_Accel = psController->float_Ks * float_WheelSpeed;

			// Integral action wind-up control
			//		if (float_IntValue > psController->float_AngleIntegralMax)
			//		{
			//			float_IntValue = psController->float_AngleIntegralMax;
			//			psController->Bool_InputSaturation = TRUE;
			//		}
			//		else if (float_IntValue < -psController->float_AngleIntegralMin)
			//		{
			//			float_IntValue = -psController->float_AngleIntegralMin;
			//			psController->Bool_InputSaturation = TRUE;
			//		}
			//		else
			//		{
			//			psController->Bool_InputSaturation = FALSE;
			//		}

			psController->float_PIDValue = float_PropValue + float_IntValue + float_DerivValue;// + S_Accel;

			//		// friction_Value is applied at low speeds
			//		if ((float_WheelSpeed * RAD_S_TO_RPM_CONVERSION_FACTOR) > psController->float_StictionSpeedThreshold)
			//		{
			//			float_friction = psController->float_FrictionValue;
			//		}
			//		else if ((float_WheelSpeed * RAD_S_TO_RPM_CONVERSION_FACTOR) < -psController->float_StictionSpeedThreshold)
			//		{
			//			float_friction = -psController->float_FrictionValue;
			//		}

			*pfloat_VoltageValue = psController->float_PIDValue;
		}
		else
		{
			psController->uint32_LastTick = uint32_CurrentTick;
			psController->Bool_FirstMeasureDone = TRUE;
		}
	}

	return Error;
}

/*
 * @brief calculate duty cycle value normalizing PID output to the max voltage applicable on the motor
 * @param psController - Controller structure
 * @param float_VoltageValue- PID Output
 * @param pfloat_DutyCycle - Duty Cycle value (between -1.0 and 1.0)
 */
Error_t Controller_CalculateDutyCycle(Controller_t *psController,
								      float float_VoltageValue,
									  float *pfloat_DutyCycle)
{
	Error_t Error = E_OK;
	float float_NormalizedValue = 0;

	if (NULL == pfloat_DutyCycle)
	{
		Error = E_NOT_OK;
	}
	else
	{
		//Check bounds
		if (float_VoltageValue < -MOTOR_VMAX)
		{
			float_VoltageValue = -MOTOR_VMAX;
			psController->Bool_InputSaturation = TRUE;
		}
		else if(float_VoltageValue > MOTOR_VMAX)
		{
			float_VoltageValue = MOTOR_VMAX;
			psController->Bool_InputSaturation = TRUE;
		}
		else
		{
			psController->Bool_InputSaturation = FALSE;
		}

		/* Normalize Voltage value [-1 ; 1] */
		float_NormalizedValue = float_VoltageValue / (float)MOTOR_VMAX;
		*pfloat_DutyCycle = float_NormalizedValue;
	}

	return Error;
}
