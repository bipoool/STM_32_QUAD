/*
 * DZMPU.h
 *
 *  Created on: 27-Nov-2022
 *      Author: bipoool
 */

#ifndef SRC_DZMPU_H_
#define SRC_DZMPU_H_

#include<stdio.h>
#include<stdlib.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include <math.h>

#include<DZ_MPU_REGISTER_DESC.h>

class DZ_MPU {
	private:

	float deltat = 0.0f;                             // integration interval for both filter schemes
	int lastUpdate = 0, firstUpdate = 0, Now = 0;    // used to calculate integration interval
	float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};           // vector to hold quaternion

	// Usefull Values
	float PI = 3.14159265358979323846f;
	float GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
	float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
	float GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
	float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

	// communication variables
	I2C_HandleTypeDef* hi2c;
	UART_HandleTypeDef *huart;

public:

	// Error Flags
	HAL_StatusTypeDef errorFlag 		= HAL_ERROR;
	char errorMsg[50];

	// Useful variable
	uint8_t MPU_READ_BIT 	= 0x00;
	uint8_t MPU_WRITE_BIT 	= 0x01;
	uint8_t setBit = 0x1;
	uint8_t resetBit = 0x0;

	uint8_t MPU_RESET_BYTE = 0b10000000;

	uint8_t DZ_MPU_MAX_DELAY = 100;

	// MPU address
	uint16_t MPU_ADDR        = 0x68;
	uint16_t MPU_READ_ADDR	 = this->MPU_ADDR << 1 | this->MPU_READ_BIT;
	uint16_t MPU_WRITE_ADDR	 = this->MPU_ADDR << 1 | this->MPU_WRITE_BIT;

	float accData[3], gyroData[3], magData[3]; // variables to hold latest sensor data values
	uint8_t accRawData[6], gyroRawData[6], magRawData[6];
	float temperature;

	float pitch, yaw, roll;


	// To configure acc and gyro
	enum accScale {
	  AFS_2G = 0,
	  AFS_4G,
	  AFS_8G,
	  AFS_16G
	};

	enum gyroScale {
	  GFS_250DPS = 0,
	  GFS_500DPS,
	  GFS_1000DPS,
	  GFS_2000DPS
	};

	// Object configurations
	float accResolution = 0x00,
		  gyroResolution = 0x00,
		  magResolution = 0x00;

	accScale aScale;
	gyroScale gScale;

private:
	float getAccResolution(uint8_t aScale);
	float getGyroResolution(uint8_t gScale);

public:
	DZ_MPU(uint8_t aScale, uint8_t gScale, I2C_HandleTypeDef* hi2c);
	DZ_MPU(uint8_t aScale, uint8_t gScale, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef *huart);
	void INIT();
	HAL_StatusTypeDef wakeUp();
	HAL_StatusTypeDef readAndProcessAccData();
	HAL_StatusTypeDef getAccRawData();
	void processAccData();
	HAL_StatusTypeDef readAndProcessGyroData();
	HAL_StatusTypeDef getGyroRawData();
	void processGyroData();
	HAL_StatusTypeDef resetMpu();
	HAL_StatusTypeDef writeByte(uint16_t i2cAddr, uint8_t writeAddr, uint8_t data);
	uint8_t readByte(uint16_t i2cAddr, uint8_t readAddr);
	HAL_StatusTypeDef assertStatus(HAL_StatusTypeDef status);
	HAL_StatusTypeDef readBytes(uint16_t i2cAddr, uint8_t readAddr, uint8_t* destination, uint8_t countOfBytesToRead);
	void sendMsgAccordingToStatus(HAL_StatusTypeDef status, char *msg);
	HAL_StatusTypeDef isActive();
	HAL_StatusTypeDef configureAcc();
	HAL_StatusTypeDef configureGyro();
	void printStringViaUART(char* msg);

	virtual ~DZ_MPU();
};

#endif /* SRC_DZMPU_H_ */
