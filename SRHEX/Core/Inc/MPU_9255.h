/*
 * MPU_9255.h
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

#ifndef MPU_9255_H_
#define MPU_9255_H_

#include "stm32f4xx_hal.h"
#include <math.h>			//Pow() and atan()
#include <string.h>

#define ADDRESS_9255	0x68

#define WHO_AM_I 0x75
#define POWER_1		0x6B
#define POWER_2		0x6C

#define CONFIG			0x1A
#define GYRO_CONFIG	0x1B
#define ACCEL_CONFIG	0x1C

#define SMPLRT_DIV	0x19

#define INT_STATUS	0x3A

#define ACCEL_XOUT_H 0x3B
#define TEMP_OUT_H	0x41
#define GYRO_XOUT_H	0x43

#define i2c_timeout 100

typedef struct
{
	int16_t Ax_RAW;
	int16_t Ay_RAW;
	int16_t Az_RAW;

	double Ax;
	double Ay;
	double Az;

	int16_t Gx_RAW;
	int16_t Gy_RAW;
	int16_t Gz_RAW;

	double Gx;
	double Gy;
	double Gz;

	double roll;
	double pitch;

	double Temp; // Temperature in C
}MPU_DataStruct;

HAL_StatusTypeDef MPU9255_Init(I2C_HandleTypeDef *hi2c_handle); // Initialize

void MPU9255_Accel_Raw(MPU_DataStruct *rawAcc);// Raw Acceleration values

void MPU9255_Accel_Scaled(MPU_DataStruct *scaledAcc); //Scaled Acceleration values

void MPU9255_Gyro_Raw(MPU_DataStruct *rawGyro); // Raw Gyro values

void MPU9255_Gyro_Scaled(MPU_DataStruct *scaledGyro); // Scaled

void MPU9255_Angle(MPU_DataStruct *getAngle);

void MPU_READ_ALL(MPU_DataStruct *ALL); // ALL VALUES

#endif /* MPU_9255_H_ */
