/*
 * mpu6050.h
 *
 *  Created on: Feb 19, 2025
 *      Author: zk836
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "i2c.h"
#include "kalmanFilter_1D.h"
/*--------------------------------------------------------------------------------------*/

#define MPU_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU_SELF_TESTA_REG		0X10	//自检寄存器A

#define MPU_SAMPLE_RATE_REG		0X19	//采样频率分频器寄存器

#define MPU_CFG_REG			0X1A	//配置寄存器

#define MPU_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器（bit4 bit3） | 0 -> 250 | 1-> 500 | 2 -> 1000 | 3- > 2000 |
#define MPU_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器（bit4 bit3） | 0 -> +-2g | 1 -> +-4g | 2 -> +-8g | 3 -> +-16g |

#define MPU_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU_FIFO_EN_REG			0X23	//FIFO使能寄存器

#define MPU_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU_I2CSLV0_ADDR_REG	        0X25	//IIC从机0器件地址寄存器
#define MPU_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU_I2CSLV0_CTRL_REG	        0X27	//IIC从机0控制寄存器
#define MPU_I2CSLV1_ADDR_REG	        0X28	//IIC从机1器件地址寄存器
#define MPU_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU_I2CSLV1_CTRL_REG	        0X2A	//IIC从机1控制寄存器
#define MPU_I2CSLV2_ADDR_REG	        0X2B	//IIC从机2器件地址寄存器
#define MPU_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU_I2CSLV2_CTRL_REG	        0X2D	//IIC从机2控制寄存器
#define MPU_I2CSLV3_ADDR_REG	        0X2E	//IIC从机3器件地址寄存器
#define MPU_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU_I2CSLV3_CTRL_REG	        0X30	//IIC从机3控制寄存器
#define MPU_I2CSLV4_ADDR_REG	        0X31	//IIC从机4器件地址寄存器
#define MPU_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU_I2CSLV4_CTRL_REG	        0X34	//IIC从机4控制寄存器
#define MPU_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU_I2CMST_STA_REG		0X36	//IIC主机状态寄存器

#define MPU_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU_INT_EN_REG			0X38	//中断使能寄存器
#define MPU_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU_TEMP_OUTH_REG		0X41	//温度值高8位寄存器
#define MPU_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU_I2CMST_DELAY_REG	        0X67	//IIC主机延时管理寄存器
#define MPU_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU_MDETECT_CTRL_REG	        0X69	//运动检测控制寄存器
#define MPU_USER_CTRL_REG		0X6A	//用户控制寄存器

#define MPU_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU_PWR_MGMT2_REG		0X6C	//电源管理寄存器2

#define MPU_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU_FIFO_RW_REG			0X74	//FIFO读写寄存器

#define MPU_DEVICE_ID_REG		0X75	//器件Address ID寄存器

#define MPU_ADDR				0X68    // AD0-Ground -> Device Address: 0X68(7-bit)； AD0-V3.3 -> Device Address: 0X69(7-bit);
#define MPU_READ    0XD1    //  (0x68 << 1) + 1 -> 0xD1 ; (0x69 << 1) + 1 -> 0XD3
#define MPU_WRITE   0XD0    //  (0x68 << 1) + 0 -> 0xD0 ; (0x69 << 1) + 0 -> 0XD2

/*--------------------------------------------------------------------------------------*/

uint8_t mpu_init(void); 						//初始化MPU6050

HAL_StatusTypeDef MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf);
HAL_StatusTypeDef MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf);
HAL_StatusTypeDef MPU_Write_Byte(uint8_t reg,uint8_t data);
HAL_StatusTypeDef MPU_Read_Byte(uint8_t reg,uint8_t *data);

HAL_StatusTypeDef MPU_Set_Gyro_Fsr(uint8_t fsr);
HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t fsr);
HAL_StatusTypeDef MPU_Set_LPF(uint16_t lpf);
HAL_StatusTypeDef MPU_Set_Rate(uint16_t rate);

HAL_StatusTypeDef MPU_Get_Gyroscope_Data(int16_t *gx,int16_t *gy,int16_t *gz);
HAL_StatusTypeDef MPU_Get_Accelerometer_Data(int16_t *ax,int16_t *ay,int16_t *az);

HAL_StatusTypeDef MPU_Calibrate_Accel_Data(int16_t *accel_bias);
HAL_StatusTypeDef MPU_Calibrate_Gyro_Data(int16_t *gyro_bias);

HAL_StatusTypeDef MPU_Get_Gyroscope(float *gx,float *gy,float *gz, int16_t *gyro_bias);
HAL_StatusTypeDef MPU_Get_Accelerometer(float *ax,float *ay,float *az, int16_t *accel_bias);
HAL_StatusTypeDef MPU_Get_Temperature(float* temp);

/*--------------------------------------------------------------------------------------*/

typedef struct {
    float Q;      // Kalman 过程噪声协方差
    float R;      // Kalman 测量噪声协方差
    float P;      // Kalman 估计误差协方差
    float K;      // Kalman 卡尔曼增益
    float roll_hat, pitch_hat;  // 估计值 （先验估计->后验估计）
} AngleProcessor;

typedef struct {
	float time_interval;
	int16_t accel_bias;
	int16_t gyro_bias;
	KalmanFilter pitch_processor;
	KalmanFilter roll_processor;

} IMUProcessor;

void AngleProcessor_Init(AngleProcessor* processor, float Q, float R);

void imu_processor_init(IMUProcessor* processor, float Q, float R, float time_interval);

void imu_process(IMUProcessor* processor);

void send_message_to_user(const char *text);

#endif /* INC_MPU6050_H_ */
