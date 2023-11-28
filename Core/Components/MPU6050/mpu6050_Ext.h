/*
 * mpu6050_Ext.h
 *
 *  Created on: 11 July 2022
 *      Author: Team ASPV
 */

#ifndef MPU6050_EXT_H_
#define MPU6050_EXT_H_

/* INCLUDES */
#include "i2c.h"
#include "main.h"

/* DEFINES */

/* MPU6050 structure */
typedef struct {

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    float Ax;
    float Ay;
    float Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    float Gx;
    float Gy;
    float Gz;

    float Temperature;

    float KalmanAngleX;
    float KalmanAngleY;
} MPU6050_Data_t;

typedef struct {

	float AngleX;
	float AngleY;

} MPU6050_Angles_t;

/* PUBLIC FUNCTIONS */

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx);

void MPU6050_Read_Accel(MPU6050_Data_t *pDataStruct);

void MPU6050_Read_Gyro(MPU6050_Data_t *pDataStruct);

void MPU6050_Read_Temp(MPU6050_Data_t *pDataStruct);

void MPU6050_Read_All();

void MPU6050_Get_Angles(MPU6050_Angles_t *pAngles);

void MPU6050_Calibrate();

Bool_t MPU6050_IsVertical();

void MPU6050_CalculateSetPoint();

//void HAL_I2C_MemRxCpltCallback();

#endif /* MPU6050_EXT_H_ */

