/*
 * Source file for mpu6050.h
 */

#include "mpu6050.h"

void MPU_ReadData(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data, uint8_t size) {
	HAL_I2C_Mem_Read(hi2c, MPU_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void MPU_WriteData(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data, uint8_t size) {
	HAL_I2C_Mem_Write(hi2c, MPU_ADDRESS, reg_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

void MPU_Init(I2C_HandleTypeDef* hi2c) {
	uint8_t data = 0x80;
	MPU_WriteData(hi2c, MPU_PWR_MGMT1_REG, &data, 1);			//reset chip
	HAL_Delay(100);
	data = 0x07;
	MPU_WriteData(hi2c, MPU_SIGPATH_RST_REG, &data, 1);			//reset acc, temp and gyro
	HAL_Delay(100);
	data = 0x00;
	MPU_WriteData(hi2c, MPU_INT_EN_REG, &data, 1);				//disable Interrupt
	data = 0x00;
	MPU_WriteData(hi2c, MPU_GYRO_CFG_REG, &data, 1);			//set full scale range of gyro: 250 deg/s
	MPU_WriteData(hi2c, MPU_ACCEL_CFG_REG, &data, 1);			//full sclae range of acce: 2g
	MPU_WriteData(hi2c, MPU_FIFO_EN_REG, &data, 1);				//disable FIFO
	MPU_WriteData(hi2c, MPU_USER_CTRL_REG, &data, 1);
	data = 50;
	MPU_WriteData(hi2c, MPU_SAMPLE_RATE_REG, &data, 1);
}

void MPU_GetGyroValue(I2C_HandleTypeDef* hi2c, uint8_t* gx, uint8_t* gy, uint8_t* gz) {
	uint8_t data[6];
	HAL_I2C_Mem_Read(hi2c, MPU_ADDRESS, MPU_GYRO_XOUTH_REG, I2C_MEMADD_SIZE_8BIT, &data[0], 6, HAL_MAX_DELAY);
	gx[0] = data[0];
	gx[1] = data[1];
	gy[0] = data[2];
	gy[1] = data[3];
	gz[0] = data[4];
	gz[1] = data[5];
}

void MPU_GetAcceleratorValue(I2C_HandleTypeDef* hi2c, uint8_t* ax, uint8_t* ay, uint8_t* az) {
	uint8_t data[6];
	HAL_I2C_Mem_Read(hi2c, MPU_ADDRESS, MPU_ACCEL_XOUTH_REG, I2C_MEMADD_SIZE_8BIT, &data[0], 6, HAL_MAX_DELAY);
	ax[0] = data[0];
	ax[1] = data[1];
	ay[0] = data[2];
	ay[1] = data[3];
	az[0] = data[4];
	az[1] = data[5];
}
