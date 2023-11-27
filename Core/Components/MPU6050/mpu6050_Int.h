/*
 * mpu6050_Int.h
 *
 *  Created on: 11 July 2022
 *      Author: Team ASPV
 */

#ifndef MPU6050_INT_H_
#define MPU6050_INT_H_

//////////// INCLUDES //////////////////

#include "mpu6050_Ext.h"

//////////// DEFINES //////////////////

// Conversion Factor
#define RAD_TO_DEG 57.295779513082320876798154814105

// MPU6050 registers
#define MPU6050_ADDR 0xD0
#define INT_PIN_CFG_REG 0x37
#define USER_CONTROL_REG 0x6A
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Timeout of 5 min during manual calibration
// Imu sample rate = 1KHz, each clock tick is 1 ms, so 3*10^6 ms = 5 min.
#define MPU6050_CALIBRATION_TIMEOUT (300000)

#define MPU6050_SETPOINT_CALC_TIMEOUT (1000)

#define MPU6050_I2C_TIMEOUT  (100)

#define MPU6050_VERTICAL_THRESHOLD	 (float)3 //[°]

// MPU6050 offsets
typedef struct {

    int16_t Offset_Accel_X;
    int16_t Offset_Accel_Y;
    int16_t Offset_Accel_Z;

    int16_t Offset_Gyro_X;
    int16_t Offset_Gyro_Y;
    int16_t Offset_Gyro_Z;
} MPU6050_Offsets_t;

// Kalman structure
typedef struct {
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} MPU6050_Kalman_t;

//////////// PRIVATE VARIABLES //////////////////

const double _Accel_Z_corrector = 14418.0;

uint32_t _uint32_Timer;
uint32_t _uint32_Now;

I2C_HandleTypeDef *_pI2C_handle;

MPU6050_Angles_t _sAngles;

MPU6050_Offsets_t _MPU6050_Offsets = {
		.Offset_Accel_X = 1449,
		.Offset_Accel_Y = -326,
		.Offset_Accel_Z = 862,
		.Offset_Gyro_X = -362,
		.Offset_Gyro_Y = -345,
		.Offset_Gyro_Z = -174,
};

MPU6050_Kalman_t _KalmanX = {
				 .Q_angle = 0.001f,
				 .Q_bias = 0.003f,
				 .R_measure = 0.03f
};

MPU6050_Kalman_t _KalmanY = {
				 .Q_angle = 0.001f,
				 .Q_bias = 0.003f,
				 .R_measure = 0.03f,
};

//////////// PRIVATE FUNCTIONS //////////////////

float MPU6050_Kalman_CalculateAngle(MPU6050_Kalman_t *Kalman, float newAngle, float newRate, float dt);

#endif /* MPU6050_INT_H_ */
