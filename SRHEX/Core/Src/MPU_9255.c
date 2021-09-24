/*
 * MPU_9255.c
 *
 *  Created on: Jul 15, 2021
 *      Author: Meliksah Sagun
 *
 *      Disclaimer:
 *      This is a free software under the GNU license as published by the Free Software Foundation, either version 3 of the License, or any later version.
 *      This program is published without any warranty and not responsible for any errors or omissions, or for the results obtained from the use of this information.
 *      All information is provided "as is", with no guarantee of completeness, accuracy, timeliness or of the results obtained from the use of this information,
 *      and without warranty of any kind, express or implied.
 *
 *      This document was last updated on July 15, 2021
 *
 */


#include "MPU_9255.h"

static I2C_HandleTypeDef hi2c_handle;
uint8_t Data_TX=0, Data_RX=0;
HAL_StatusTypeDef error;

HAL_StatusTypeDef MPU9255_Init(I2C_HandleTypeDef *hi2c_){

	memcpy(&hi2c_handle, hi2c_, sizeof(*hi2c_)); // GET THE hi2c ADDRESS

	error = HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1) , WHO_AM_I, 1, &Data_RX, 1, i2c_timeout); // CHECK DEVICE ID

	if(Data_RX != 0x71 || error != HAL_OK){
		return error;
	}


	error = HAL_I2C_Mem_Write(&hi2c_handle, (ADDRESS_9255 << 1), POWER_1 , 1, 0, 1, 100); // POWER 1
	if(error != HAL_OK) return error;

	error = HAL_I2C_Mem_Write(&hi2c_handle, (ADDRESS_9255 << 1), POWER_2 , 1, 0, 1, 100); // POWER 2
	if(error != HAL_OK) return error;

	error = HAL_I2C_Mem_Write(&hi2c_handle, (ADDRESS_9255 << 1), SMPLRT_DIV , 1, (uint8_t *)7, 1, 100); // SMPLRT_DIV
	if(error != HAL_OK) return error;

	error = HAL_I2C_Mem_Write(&hi2c_handle, (ADDRESS_9255 << 1), ACCEL_CONFIG , 1, 0, 1, 100); // ACCEL_CONFIG
	if(error != HAL_OK) return error;

	error = HAL_I2C_Mem_Write(&hi2c_handle, (ADDRESS_9255 << 1), GYRO_CONFIG, 1, 0, 1, 100); // GYRO_CONFIG
	if(error != HAL_OK) return error;

	return HAL_OK;
}
void MPU9255_Accel_Raw(MPU_DataStruct *rawAcc){
    uint8_t Accel_Raw[6];

   HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), ACCEL_XOUT_H , 1, Accel_Raw, 6, 100);

   rawAcc->Ax_RAW = (int16_t) (Accel_Raw[0] << 8 | Accel_Raw[1]);
   rawAcc->Ay_RAW = (int16_t) (Accel_Raw[2] << 8 | Accel_Raw[3]);
   rawAcc->Az_RAW = (int16_t) (Accel_Raw[4] << 8 | Accel_Raw[5]);
}

void MPU9255_Accel_Scaled(MPU_DataStruct *scaledAcc){
    uint8_t Accel_Raw[6];

    HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), ACCEL_XOUT_H , 1, Accel_Raw, 6, 100);

    scaledAcc->Ax_RAW = (int16_t) (Accel_Raw[0] << 8 | Accel_Raw[1]);
	scaledAcc->Ay_RAW = (int16_t) (Accel_Raw[2] << 8 | Accel_Raw[3]);
	scaledAcc->Az_RAW = (int16_t) (Accel_Raw[4] << 8 | Accel_Raw[5]);

	scaledAcc->Ax = scaledAcc->Ax_RAW/16384.0;
	scaledAcc->Ay = scaledAcc->Ay_RAW/16384.0;
	scaledAcc->Az = scaledAcc->Az_RAW/16384.0;

}

void MPU9255_Gyro_Raw(MPU_DataStruct *rawGyro){

	uint8_t Gyro_Raw[6];

	HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), GYRO_XOUT_H , 1, Gyro_Raw, 6, 100);
    rawGyro->Gx_RAW = (int16_t) (Gyro_Raw[0] << 8 | Gyro_Raw[1]);
    rawGyro->Gy_RAW= (int16_t) (Gyro_Raw[2] << 8 | Gyro_Raw[3]);
    rawGyro->Gz_RAW = (int16_t) (Gyro_Raw[4] << 8 | Gyro_Raw[5]);

}

void MPU9255_Gyro_Scaled(MPU_DataStruct *scaledGyro){
	uint8_t Gyro_Raw[6];

	HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), GYRO_XOUT_H , 1, Gyro_Raw, 6, 100);

	scaledGyro->Gx_RAW = (int16_t) (Gyro_Raw[0] << 8 | Gyro_Raw[1]);
	scaledGyro->Gy_RAW= (int16_t) (Gyro_Raw[2] << 8 | Gyro_Raw[3]);
	scaledGyro->Gz_RAW = (int16_t) (Gyro_Raw[4] << 8 | Gyro_Raw[5]);

	scaledGyro->Gx = scaledGyro->Gx_RAW / 131.0;
	scaledGyro->Gy = scaledGyro->Gy_RAW / 131.0;
	scaledGyro->Gz = scaledGyro->Gz_RAW / 131.0;
}

void MPU_ALL(MPU_DataStruct *ALL){
    uint8_t Accel_Raw[6];

    MPU9255_Gyro_Scaled(ALL);
    MPU9255_Accel_Scaled(ALL);
    ALL->roll = atan(ALL->Ay / sqrt(pow(ALL->Ax, 2) + pow(ALL->Az, 2))) * 180 / 3.1415;
    ALL->pitch = atan(-1 * ALL->Ax / sqrt(pow(ALL->Ay, 2) + pow(ALL->Az, 2))) * 180 / 3.1415;

    HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), ACCEL_XOUT_H , 1, Accel_Raw, 6, 100);

    ALL->Ax_RAW = (int16_t) (Accel_Raw[0] << 8 | Accel_Raw[1]);
    ALL->Ay_RAW = (int16_t) (Accel_Raw[2] << 8 | Accel_Raw[3]);
    ALL->Az_RAW = (int16_t) (Accel_Raw[4] << 8 | Accel_Raw[5]);

    ALL->Ax = ALL->Ax_RAW/16384.0;
    ALL->Ay = ALL->Ay_RAW/16384.0;
    ALL->Az = ALL->Az_RAW/16384.0;

	uint8_t Gyro_Raw[6];

	HAL_I2C_Mem_Read(&hi2c_handle, (ADDRESS_9255 << 1), GYRO_XOUT_H , 1, Gyro_Raw, 6, 100);

	ALL->Gx_RAW = (int16_t) (Gyro_Raw[0] << 8 | Gyro_Raw[1]);
	ALL->Gy_RAW= (int16_t) (Gyro_Raw[2] << 8 | Gyro_Raw[3]);
	ALL->Gz_RAW = (int16_t) (Gyro_Raw[4] << 8 | Gyro_Raw[5]);

	ALL->Gx = ALL->Gx_RAW / 131.0;
	ALL->Gy = ALL->Gy_RAW / 131.0;
	ALL->Gz = ALL->Gz_RAW / 131.0;

}



