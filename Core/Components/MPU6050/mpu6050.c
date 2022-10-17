/*
 * mpu6050.c
 *
 *  Created on: 11 July 2022
 *      Author: Team ASPV
 *
 * |---------------------------------------------------------------------------------
 * | Kalman filter algorithm used from https://github.com/TKJElectronics/KalmanFilter
 * |---------------------------------------------------------------------------------
 */

#include "mpu6050_Int.h"

uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check = 0;
    uint8_t Data = 0;

    //FIXME: remove
    uint8_t IntPinCfg_read = 0;
    uint8_t UserControl_read = 0;

    _pI2C_handle = I2Cx;

//    memset(& _MPU6050_Offsets, 0, sizeof(_MPU6050_Offsets));

    // check device ID WHO_AM_I
    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_I2C_TIMEOUT);

    // Read content of INT_PIN_CFG_REG and USER_CONTROL_REG
    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &IntPinCfg_read, 1, MPU6050_I2C_TIMEOUT);
    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, USER_CONTROL_REG, 1, &UserControl_read, 1, MPU6050_I2C_TIMEOUT);

    if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, MPU6050_I2C_TIMEOUT);

        // Enable Auxiliary I2C bypass in INT_PIN_CFG_REG Register
        IntPinCfg_read |= 0x02;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &IntPinCfg_read, 1, MPU6050_I2C_TIMEOUT);

        // Disable auxiliary I2C master mode in USER_CONTROL_REG Register
        UserControl_read |= 0x00;
        HAL_I2C_Mem_Write(_pI2C_handle, MPU6050_ADDR, USER_CONTROL_REG, 1, &UserControl_read, 1, MPU6050_I2C_TIMEOUT);

        return 0;
    }
    return 1;
}


void MPU6050_Read_Accel(MPU6050_Data_t *pDataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_I2C_TIMEOUT);

    pDataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]) - _MPU6050_Offsets.Offset_Accel_X;
    pDataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]) - _MPU6050_Offsets.Offset_Accel_Y;
    pDataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]) - _MPU6050_Offsets.Offset_Accel_Z;

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    pDataStruct->Ax = pDataStruct->Accel_X_RAW / 16384.0;
    pDataStruct->Ay = pDataStruct->Accel_Y_RAW / 16384.0;
    pDataStruct->Az = pDataStruct->Accel_Z_RAW / 16384.0; //Accel_Z_corrector;
}


void MPU6050_Read_Gyro(MPU6050_Data_t *pDataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, MPU6050_I2C_TIMEOUT);

    pDataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]) - _MPU6050_Offsets.Offset_Gyro_X;
    pDataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]) - _MPU6050_Offsets.Offset_Gyro_Y;
    pDataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]) - _MPU6050_Offsets.Offset_Gyro_Z;

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    pDataStruct->Gx = pDataStruct->Gyro_X_RAW / 131.0;
    pDataStruct->Gy = pDataStruct->Gyro_Y_RAW / 131.0;
    pDataStruct->Gz = pDataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(MPU6050_Data_t *pDataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, MPU6050_I2C_TIMEOUT);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    pDataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}

void MPU6050_Read_All(MPU6050_Data_t *pDataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_I2C_TIMEOUT);

    pDataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]) - _MPU6050_Offsets.Offset_Accel_X;
    pDataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]) - _MPU6050_Offsets.Offset_Accel_Y;
    pDataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]) - _MPU6050_Offsets.Offset_Accel_Z;
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    pDataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]) - _MPU6050_Offsets.Offset_Gyro_X;
    pDataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]) - _MPU6050_Offsets.Offset_Gyro_Y;
    pDataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]) - _MPU6050_Offsets.Offset_Gyro_Z;

    pDataStruct->Ax = pDataStruct->Accel_X_RAW / 16384.0;
    pDataStruct->Ay = pDataStruct->Accel_Y_RAW / 16384.0;
    pDataStruct->Az = pDataStruct->Accel_Z_RAW / 16384.0;//Accel_Z_corrector;
    pDataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
    pDataStruct->Gx = pDataStruct->Gyro_X_RAW / 131.0;
    pDataStruct->Gy = pDataStruct->Gyro_Y_RAW / 131.0;
    pDataStruct->Gz = pDataStruct->Gyro_Z_RAW / 131.0;

}

void MPU6050_Get_Angles(MPU6050_Angles_t *pAngles)
{
	MPU6050_Data_t DataStruct;
	MPU6050_Read_All(&DataStruct);

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - _uint32_Timer) / 1000;
    _uint32_Timer = HAL_GetTick();

    // https://www.nxp.com/files-static/sensors/doc/app_note/AN3461.pdf
    double roll = atan2(DataStruct.Accel_Y_RAW, DataStruct.Accel_Z_RAW) * RAD_TO_DEG;
    double pitch;
    double pitch_sqrt = sqrt(
    		DataStruct.Accel_Y_RAW * DataStruct.Accel_Y_RAW + DataStruct.Accel_Z_RAW * DataStruct.Accel_Z_RAW);
    pitch = atan2(-DataStruct.Accel_X_RAW, pitch_sqrt) * RAD_TO_DEG;

    DataStruct.KalmanAngleX = MPU6050_Kalman_CalculateAngle(&_KalmanX, roll, DataStruct.Gx, dt);
    DataStruct.KalmanAngleY = MPU6050_Kalman_CalculateAngle(&_KalmanY, pitch, DataStruct.Gy, dt);

    pAngles->AngleX = DataStruct.KalmanAngleX;
    pAngles->AngleY = DataStruct.KalmanAngleY;
}

double MPU6050_Kalman_CalculateAngle(MPU6050_Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};

void MPU6050_Calibrate()
{
	uint8_t Enable = 1;
	int64_t Measures[6] = {0}; // [ax, ay, az, gx, gy, gz]
	int32_t i = 0;

    uint8_t Rec_Data[14];

	while ((Enable == 1) && (i < MPU6050_CALIBRATION_TIMEOUT))
	{
		HAL_I2C_Mem_Read(_pI2C_handle, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, MPU6050_I2C_TIMEOUT);

		Measures[0] += (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]); // ax
		Measures[1] += (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]); // ay
		Measures[2] += (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]); // az
		Measures[3] += (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]); // gx
		Measures[4] += (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]); // gy
		Measures[5] += (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]); // gz

		i++;
		HAL_Delay (1);
	}

	_MPU6050_Offsets.Offset_Accel_X = Measures[0] / i;
	_MPU6050_Offsets.Offset_Accel_Y = Measures[1] / i;
	_MPU6050_Offsets.Offset_Accel_Z = Measures[2] / i;
	_MPU6050_Offsets.Offset_Gyro_X =  Measures[3] / i;
	_MPU6050_Offsets.Offset_Gyro_Y =  Measures[4] / i;
	_MPU6050_Offsets.Offset_Gyro_Z =  Measures[5] / i;

}

Bool_t MPU6050_IsVertical()
{
	Bool_t Bool_IsVertical = FALSE;
	MPU6050_Angles_t sAngles;

	memset(&sAngles, 0, sizeof(sAngles));
	MPU6050_Get_Angles(&sAngles);

	if ((abs(sAngles.AngleX) < MPU6050_VERTICAL_THRESHOLD) &&
		(abs(sAngles.AngleY) < MPU6050_VERTICAL_THRESHOLD))
	{
		Bool_IsVertical = TRUE;
	}

	return Bool_IsVertical;
}

