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
 * @param angle The direction associated with this controller, x or y
 * @param angle set point
 * @param Kp PID imu proportional gain
 * @param Ki PID imu integral gain
 * @param Kd PID imu derivative gain
 * @param Ks PID rotor speed gain
 * @param friction
 * @param Max value of integral error
 * @param Min value of integral error
 *
 */
Error_t Controller_Init(Controller_t *psController,
						uint8_t uint8_Angle,
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
		psController->uint8_Angle = uint8_Angle;

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
		psController->int32_PID_Voltage = 0;

	}
	return Error;
}

/*
 * @brief calculate a motor voltage based on encoder, motor, and imu data
 * @param Controller structure
 * @param Voltage value, PID output value
 */
Error_t Controller_GetPIDVoltageValue(Controller_t *psController, float *pfloat_VoltageValue)
{
	Error_t Error = E_OK;

	MPU6050_Angles_t sAngles;
	static float float_AngleNow = 0;
	float float_AngleError = 0;
	float float_AngleDerivativeError = 0;

	MPU6050_Get_Angles(&sAngles);

	if (0 == psController->uint8_Angle)
	{
		float_AngleNow = sAngles.AngleX;
	}
	else if (1 == psController->uint8_Angle)
	{
		float_AngleNow = sAngles.AngleY;
	}
	else
	{
		Error = E_NOT_OK;
	}

	if (E_OK == Error)
	{
		// calculate angle error
		float_AngleError = psController->float_AngleSetpoint - float_AngleNow;

		// integral angle error
		psController->float_AngleIntegralError += float_AngleError; //* uint32_deltaTime / 1000000.0;

		// Integral wind-up control
		if (psController->float_AngleIntegralError > psController->float_AngleIntegralMax)
		{
			psController->float_AngleIntegralError = psController->float_AngleIntegralMax;
		}
		else if (psController->float_AngleIntegralError < psController->float_AngleIntegralMin)
		{
			psController->float_AngleIntegralError = psController->float_AngleIntegralMin;
		}

		// derivative angle error
		float_AngleDerivativeError = float_AngleError - psController->float_LastError;
		psController->float_LastError = float_AngleError;


		//	float speed;
		//	ENCODER_GetSpeed(&speed);
		float P_Accel = psController->float_Kp * float_AngleError;
		float I_Accel = psController->float_Ki * psController->float_AngleIntegralError;
		float D_Accel = psController->float_Kd * float_AngleDerivativeError;
		//	float S_Accel = Ks * speed / 1000.;
		float PID_Accel = P_Accel + I_Accel + D_Accel;// + S_Accel;

		//	float friction = 0;
		//	if (speed > psController->float_stiction_speed_threshold)
		//	{
		//		friction = psController->float_friction_Value;
		//	}
		//	else if (speed < -psController->float_stiction_speed_threshol)
		//	{
		//		friction = -psController->float_friction_Value;
		//	}

		*pfloat_VoltageValue = (PID_Accel);
	}

	return Error;
}

/*
 * @brief calculate duty cycle value normalizing PID output to the max voltage applicable on the motor
 * @param PID Output
 * @param Duty Cycle value (between -1.0 and 1.0)
 */
Error_t Controller_CalculateDutyCycle(float float_VoltageValue, float *pfloat_DutyCycle)
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
		}
		else if(float_VoltageValue > MOTOR_VMAX)
		{
			float_VoltageValue = MOTOR_VMAX;
		}
		else
		{
			/* Nothing to be done */
		}

		/* Normalize Voltage value [-1 ; 1] */
		float_NormalizedValue = float_VoltageValue / (float)MOTOR_VMAX;
		*pfloat_DutyCycle = float_NormalizedValue;
	}

	return Error;
}
