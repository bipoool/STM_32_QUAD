/*
 * DZMPU.cpp
 *
 *  Created on: 27-Nov-2022
 *      Author: bipoool
 */

#include "DZMPU.h"

// public functions

DZ_MPU::DZ_MPU(uint8_t aScale, uint8_t gScale,I2C_HandleTypeDef* hi2c) { // @suppress("Class members should be properly initialized")

	this->hi2c = hi2c;

}

DZ_MPU::DZ_MPU(uint8_t aScale, uint8_t gScale, I2C_HandleTypeDef* hi2c, UART_HandleTypeDef *huart) {

	this->hi2c = hi2c;
	this->huart = huart;

	this->aScale = (accScale)aScale;
	this->gScale = (gyroScale)gScale;

	this->accResolution = this->getAccResolution(aScale);
	this->gyroResolution = this->getGyroResolution(gScale);

}

void DZ_MPU::INIT(){

	this->printStringViaUART((char *)"\r\nInitializing MPU....\r\n");
	if(this->isActive() != HAL_OK)return;
	HAL_Delay(10);
	this->resetMpu();
	HAL_Delay(10);
	this->configureAcc();
	HAL_Delay(10);
	this->configureGyro();
	HAL_Delay(10);
	this->wakeUp();
	HAL_Delay(10);

}

HAL_StatusTypeDef DZ_MPU::wakeUp(){
	HAL_StatusTypeDef writeStatus = this->writeByte(this->MPU_WRITE_ADDR, PWR_MGMT_1, this->resetBit);
	return writeStatus;
}

HAL_StatusTypeDef DZ_MPU::configureAcc(){

	  uint8_t accConfig = 0x00;

	  // Clear self-test bits [7:5]
	  accConfig &= ~0xE0;

	  // Clear AFS bits [4:3]
	  accConfig &= ~0x18;

	  // Setting up the scale value
	  accConfig |= this->aScale;

	  HAL_StatusTypeDef configStatus = this->writeByte(this->MPU_WRITE_ADDR, ACCEL_CONFIG, accConfig);
	  this->sendMsgAccordingToStatus(configStatus, (char*)"config acc");

	  return this->assertStatus(configStatus);


}

HAL_StatusTypeDef DZ_MPU::configureGyro(){

	  uint8_t gyroConfig = 0x00;

	  // Clear self-test bits [7:5]
	  gyroConfig &= ~0xE0;

	  // Clear AFS bits [4:3]
	  gyroConfig &= ~0x18;

	  // Setting up the scale value
	  gyroConfig |= this->gScale;

	  HAL_StatusTypeDef configStatus = this->writeByte(this->MPU_WRITE_ADDR, ACCEL_CONFIG, gyroConfig);
	  this->sendMsgAccordingToStatus(configStatus, (char*)"config gyro");

	  return this->assertStatus(configStatus);


}

HAL_StatusTypeDef DZ_MPU::readAndProcessGyroData(){
	HAL_StatusTypeDef readStatus = this->getGyroRawData();
	this->processGyroData();
	return readStatus;
}

HAL_StatusTypeDef DZ_MPU::getGyroRawData(){

	 HAL_StatusTypeDef readStatus = this->readBytes(this->MPU_READ_ADDR, GYRO_XOUT_H, this->gyroRawData, 6);
	 return this->assertStatus(readStatus);

}

void DZ_MPU::processGyroData(){

	this->gyroData[0] = (this->gyroRawData[0] << 8 | this->gyroRawData[1]) * this->gyroResolution;
	this->gyroData[1] = (this->gyroRawData[2] << 8 | this->gyroRawData[3]) * this->gyroResolution;
	this->gyroData[2] = (this->gyroRawData[4] << 8 | this->gyroRawData[5]) * this->gyroResolution;

}

HAL_StatusTypeDef DZ_MPU::readAndProcessAccData(){
	HAL_StatusTypeDef readStatus = this->getAccRawData();
	this->processAccData();
	return readStatus;
}

HAL_StatusTypeDef DZ_MPU::getAccRawData(){

	 HAL_StatusTypeDef readStatus = this->readBytes(this->MPU_READ_ADDR, ACCEL_XOUT_H, this->accRawData, 6);
	 return this->assertStatus(readStatus);

}

void DZ_MPU::processAccData(){

	this->accData[0] = (this->accRawData[0] << 8 | this->accRawData[1]) * this->accResolution;
	this->accData[1] = (this->accRawData[2] << 8 | this->accRawData[3]) * this->accResolution;
	this->accData[2] = (this->accRawData[4] << 8 | this->accRawData[5]) * this->accResolution;

}


HAL_StatusTypeDef DZ_MPU::resetMpu(){
	return this->writeByte(this->MPU_WRITE_ADDR, PWR_MGMT_1, this->MPU_RESET_BYTE);
}

HAL_StatusTypeDef DZ_MPU::isActive(){
	HAL_StatusTypeDef connectionStatus = HAL_I2C_IsDeviceReady(this->hi2c, this->MPU_READ_ADDR, 1, 100);
	return this->assertStatus(connectionStatus);
}

HAL_StatusTypeDef DZ_MPU::writeByte(uint16_t i2cAddr, uint8_t writeAddr, uint8_t data){
	HAL_StatusTypeDef writeStatus = HAL_I2C_Mem_Write(this->hi2c, i2cAddr, writeAddr, sizeof(writeAddr), &data, sizeof(data), this->DZ_MPU_MAX_DELAY);
	return this->assertStatus(writeStatus);
}

uint8_t DZ_MPU::readByte(uint16_t i2cAddr, uint8_t readAddr){

	uint8_t readByte;
	HAL_StatusTypeDef readStatus = HAL_I2C_Mem_Read(this->hi2c, i2cAddr, readAddr, sizeof(readAddr), &readByte, sizeof(readByte), this->DZ_MPU_MAX_DELAY);
	this->assertStatus(readStatus);
	return readByte;

}

HAL_StatusTypeDef DZ_MPU::readBytes(uint16_t i2cAddr, uint8_t readAddr, uint8_t* destination, uint8_t countOfBytesToRead){

	HAL_StatusTypeDef readStatus = HAL_I2C_Mem_Read(this->hi2c, i2cAddr, readAddr, sizeof(readAddr), destination, countOfBytesToRead, this->DZ_MPU_MAX_DELAY);
	return this->assertStatus(readStatus);

}

HAL_StatusTypeDef DZ_MPU::assertStatus(HAL_StatusTypeDef status){
	if(status != HAL_OK){
		this->errorFlag = HAL_ERROR;
	}
	else{
		this->errorFlag = HAL_OK;
	}
	return this->errorFlag;
}

void DZ_MPU::sendMsgAccordingToStatus(HAL_StatusTypeDef status, char *msg){

	char dataStr[50];
	sprintf(dataStr, msg);

	const char *cStatus = status == HAL_OK ? " success\n\r":" error\n\r";
	strcat(dataStr, cStatus);
	this->printStringViaUART(dataStr);
}

void DZ_MPU::printStringViaUART(char* msg){
	char dataStr[50];
	sprintf(dataStr, msg);
	HAL_UART_Transmit(this->huart, (uint8_t*)dataStr, strlen(dataStr), HAL_MAX_DELAY);
}

/////////////////////////////////////////////////////// Private functions /////////////////////////////////////////////

float DZ_MPU::getGyroResolution(uint8_t gScale) {
	float gyroRes;
	switch (gScale)
	{
    	// Possible gyro scales (and their register bit settings) are:
    	// 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    	case GFS_250DPS:
    		gyroRes = 250.0/32768.0;
			break;
    	case GFS_500DPS:
    		gyroRes = 500.0/32768.0;
			break;
    	case GFS_1000DPS:
    		gyroRes = 1000.0/32768.0;
			break;
    	case GFS_2000DPS:
    		gyroRes = 2000.0/32768.0;
			break;
    	default:
    		gyroRes = 250.0/32768.0;
       return gyroRes;

  }
}


float DZ_MPU::getAccResolution(uint8_t aScale) {

	float accRes;
	switch (aScale)
	{
    	// Possible accelerometer scales (and their register bit settings) are:
    	// 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    	case AFS_2G:
    		accRes = 2.0/32768.0;
		break;
    	case AFS_4G:
    		accRes = 4.0/32768.0;
		break;
    	case AFS_8G:
    		accRes = 8.0/32768.0;
		break;
    	case AFS_16G:
    		accRes = 16.0/32768.0;
		break;
    	default:
    		accRes = 2.0/32768.0;
    	return accRes;

	}
}

DZ_MPU::~DZ_MPU() {

}
