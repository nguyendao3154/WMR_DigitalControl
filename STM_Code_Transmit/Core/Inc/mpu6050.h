/*
 * Header file for project
 * Application: Communication with MPU-6050 sensor
 * Peripheral: Standard I2C
 */

/*
 * @include
 */

#include "i2c.h"

/*
 * @define
 * Register group
 */

#define MPU_ADDRESS	0x68 			//7 bits has been shifted left

/************** MPU6050 Address of correlation register*********************/
#define MPU_ACCEL_OFFS_REG		0X06	//Accel? Off register, readable version number, not mentioned in register manual
#define MPU_PROD_ID_REG			0X0C	//prod id register, not mentioned in register manual
#define MPU_SELF_TESTX_REG		0X0D	//Self check register X
#define MPU_SELF_TESTY_REG		0X0E	//Self check register Y
#define MPU_SELF_TESTZ_REG		0X0F	//Self check register Z
#define MPU_SELF_TESTA_REG		0X10	//Self check register A
#define MPU_SAMPLE_RATE_REG		0X19	//Sampling frequency divider
#define MPU_CFG_REG				0X1A	//Configuration register
#define MPU_GYRO_CFG_REG		0X1B	//Gyroscope configuration register
#define MPU_ACCEL_CFG_REG		0X1C	//Accelerometer configuration register
#define MPU_MOTION_DET_REG		0X1F	//Motion detection threshold setting register
#define MPU_FIFO_EN_REG			0X23	//FIFO enable register
#define MPU_I2CMST_CTRL_REG		0X24	//IIC host control register
#define MPU_I2CSLV0_ADDR_REG	0X25	//IIC slave 0 device address register
#define MPU_I2CSLV0_REG			0X26	//IIC slave 0 data address register
#define MPU_I2CSLV0_CTRL_REG	0X27	//IIC slave 0 control register
#define MPU_I2CSLV1_ADDR_REG	0X28	//IIC slave 1 device address register
#define MPU_I2CSLV1_REG			0X29	//IIC slave 1 data address register
#define MPU_I2CSLV1_CTRL_REG	0X2A	//IIC slave 1 control register
#define MPU_I2CSLV2_ADDR_REG	0X2B	//IIC slave 2 device address register
#define MPU_I2CSLV2_REG			0X2C	//IIC slave 2 data address register
#define MPU_I2CSLV2_CTRL_REG	0X2D	//IIC slave 2 control register
#define MPU_I2CSLV3_ADDR_REG	0X2E	//IIC slave 3 device address register
#define MPU_I2CSLV3_REG			0X2F	//IIC slave 3 data address register
#define MPU_I2CSLV3_CTRL_REG	0X30	//IIC slave 3 control register
#define MPU_I2CSLV4_ADDR_REG	0X31	//IIC slave 4 device address register
#define MPU_I2CSLV4_REG			0X32	//IIC slave 4 data address register
#define MPU_I2CSLV4_DO_REG		0X33	//IIC slave 4 write data register
#define MPU_I2CSLV4_CTRL_REG	0X34	//IIC slave 4 control register
#define MPU_I2CSLV4_DI_REG		0X35	//IIC slave 4 read data register

#define MPU_I2CMST_STA_REG		0X36	//IIC host status register
#define MPU_INTBP_CFG_REG		0X37	//Interrupt / bypass setting register
#define MPU_INT_EN_REG			0X38	//Interrupt enable register
#define MPU_INT_STA_REG			0X3A	//Interrupt status register

#define MPU_ACCEL_XOUTH_REG		0X3B	//Acceleration value, X-axis high 8-bit register
#define MPU_ACCEL_XOUTL_REG		0X3C	//Acceleration value, X-axis low 8-bit register
#define MPU_ACCEL_YOUTH_REG		0X3D	//Acceleration value, Y-axis high 8-bit register
#define MPU_ACCEL_YOUTL_REG		0X3E	//Acceleration value, Y-axis low 8-bit register
#define MPU_ACCEL_ZOUTH_REG		0X3F	//Acceleration value, Z-axis high 8-bit register
#define MPU_ACCEL_ZOUTL_REG		0X40	//Acceleration value, Z-axis low 8-bit register

#define MPU_TEMP_OUTH_REG		0X41	//Temperature value high octet register
#define MPU_TEMP_OUTL_REG		0X42	//Temperature value low 8-bit register

#define MPU_GYRO_XOUTH_REG		0X43	//Gyroscope value, X-axis high 8-bit register
#define MPU_GYRO_XOUTL_REG		0X44	//Gyroscope value, X-axis low 8-bit register
#define MPU_GYRO_YOUTH_REG		0X45	//Gyroscope value, Y-axis high 8-bit register
#define MPU_GYRO_YOUTL_REG		0X46	//Gyroscope value, Y-axis low 8-bit register
#define MPU_GYRO_ZOUTH_REG		0X47	//Gyroscope value, Z-axis high 8-bit register
#define MPU_GYRO_ZOUTL_REG		0X48	//Gyroscope value, Z-axis low 8-bit register

#define MPU_I2CSLV0_DO_REG		0X63	//IIC slave 0 data register
#define MPU_I2CSLV1_DO_REG		0X64	//IIC slave 1 data register
#define MPU_I2CSLV2_DO_REG		0X65	//IIC slave 2 data register
#define MPU_I2CSLV3_DO_REG		0X66	//IIC slave 3 data register

#define MPU_I2CMST_DELAY_REG	0X67	//IIC host delay management register
#define MPU_SIGPATH_RST_REG		0X68	//Signal channel reset register
#define MPU_MDETECT_CTRL_REG	0X69	//Motion detection control register
#define MPU_USER_CTRL_REG		0X6A	//User control register
#define MPU_PWR_MGMT1_REG		0X6B	//Power management register 1
#define MPU_PWR_MGMT2_REG		0X6C	//Power management register 2
#define MPU_FIFO_CNTH_REG		0X72	//FIFO count register high octet
#define MPU_FIFO_CNTL_REG		0X73	//FIFO count register low octet
#define MPU_FIFO_RW_REG			0X74	//FIFO read write register
#define MPU_DEVICE_ID_REG		0X75	//Device ID register

/**************** Function Prototype *****************************/

void MPU_ReadData(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data, uint8_t size);
void MPU_WriteData(I2C_HandleTypeDef* hi2c, uint8_t reg_addr, uint8_t* data, uint8_t size);

void MPU_Init(I2C_HandleTypeDef* hi2c);
void MPU_GetGyroValue(I2C_HandleTypeDef* hi2c, uint8_t* gx, uint8_t* gy, uint8_t* gz);
void MPU_GetAcceleratorValue(I2C_HandleTypeDef* hi2c, uint8_t* ax, uint8_t* ay, uint8_t* az);
