/*
 * controller_Ext.h
 *
 *  Created on: 11 July 2022
 *      Author: Team ASPV
 */

#ifndef CONTROLLER_EXT_H_
#define CONTROLLER_EXT_H_

#include "main.h"
#include "MotorExt.h"
#include "mpu6050_Ext.h"
#include "EncoderExt.h"

typedef enum
{
	CONTROLLER_AXIS_X = 0,
	CONTROLLER_AXIS_Y = 1,
	CONTREOLLER_AXIS_NUMBER
} ControllerAxis_te;

typedef struct
{
	ControllerAxis_te Enum_ControllerAxis;
	float float_PIDValue;

	float float_Kp;
	float float_Ki;
	float float_Kd;
	float float_Ks;

	float float_AngleSetpoint;
	float float_AngleIntegralError;
	float float_LastError;
	float float_AngleIntegralMax;
	float float_AngleIntegralMin;

	float float_StictionSpeedThreshold;  // RPM
	float float_FrictionValue;

	uint32_t uint32_LastTick;

} Controller_t;

Error_t Controller_Init(Controller_t *psController,
						ControllerAxis_te Enum_ControllerAxis,
						float float_AngleSetpoint,
						float float_Kp,
						float float_Ki,
						float float_Kd,
						float float_Ks,
						float float_Friction,
						float float_angle_Integral_Max,
						float float_angle_Integral_Min);

Error_t Controller_GetPIDVoltageValue(uint32_t uint32_CurrentTick, Controller_t *psController, float *pfloat_VoltageValue);

Error_t Controller_CalculateDutyCycle(float pfloat_VoltageValue, float *pfloat_DutyCycle);

//void set_angle_Average_Filter(float filter_value);
//float get_angle_Average_Filter(void);
//void set_angle_Smoothed_Filter(float filter_value);
//float get_angle_Smoothed_Filter(void);
//void set_Zero_Filter(float filter_value);
//float get_Zero_Filter(void);
//void set_Zero(float filter_value);
//float get_Zero(void);
//void set_Kp(float value);
//void set_Ki(float value);
//void set_Kd(float value);
//void set_Ks(float value);
//void set_friction(float value);
//float get_Kp(void);
//float get_Ki(void);
//float get_Kd(void);
//float get_Ks(void);
//float get_friction(void);
//void set_Ktd(float value);
//float get_Ktd(void);
//void set_Kt(float value);
//float get_Kt(void);
//void set_mode(controller_mode new_mode);
//controller_mode get_mode(void);
//void set_default_voltage(int voltage);
//int get_defult_voltage(void);

#endif /* CONTROLLER_EXT_H_ */


