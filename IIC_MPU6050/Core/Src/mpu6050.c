/*
 * mpu6050.c
 *
 *  Created on: Feb 19, 2025
 *      Author: zk836
 */


#include "mpu6050.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
static I2C_HandleTypeDef* mpu_i2c_handlePtr = &hi2c1;

#define ACCEL_SCALE 16384.0f  // ±2g -> 65536 / 4
#define GYRO_SCALE 16.4f     // ±2000°/s -> 65536 / 4000

void MPU_SET_I2C_Handle(I2C_HandleTypeDef * hi2cPtr){
	mpu_i2c_handlePtr = hi2cPtr;
}

//初始化MPU6050 --Return: 0 -> Success; Other -> False;

uint8_t MPU_Init(void){

	uint8_t *templete_data = 0;
	HAL_StatusTypeDef ret;
    HAL_I2C_Init(mpu_i2c_handlePtr);

    ret = HAL_I2C_IsDeviceReady(mpu_i2c_handlePtr, MPU_WRITE, 1, 100);
    if (ret == HAL_OK){
    	SendMessage(&huart2, "The device is ready \n");
    } else {
    	SendMessage(&huart2, "The device is not ready \n");
    }

    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050

    ret = MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//Exiting from the Sleeping mode
    if (ret == HAL_OK){
    	SendMessage(&huart2, "Exiting from the Sleep \n");
    } else {
    	SendMessage(&huart2, "Not exiting from the Sleep \n");
    }

    ret = MPU_Set_Gyro_Fsr(3);	//Configuring Accelerometer
    if (ret == HAL_OK){
    	SendMessage(&huart2, "Configuring Accelerometer \n");
    } else {
    	SendMessage(&huart2, "Not Configuring Accelerometer \n");
    }

    ret = MPU_Set_Accel_Fsr(0);	//Configuring Gyroscope
    if (ret == HAL_OK){
    	SendMessage(&huart2, "Configuring Gyroscope \n");
    } else {
    	SendMessage(&huart2, "Not Configuring Gyroscope \n");
    }

    MPU_Set_Rate(50);						//设置采样率--50Hz
    MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效

    ret = MPU_Read_Byte(MPU_DEVICE_ID_REG, templete_data);
	printf("\r\nMPU6050:0x%2x\r\n", *templete_data);
    if(*templete_data == MPU_ADDR)//器件ID正确
    {
    	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL--PLL|X轴为参考
    	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
    } else {
    	return 0;
    }
    return 1;
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Write_Len(uint8_t reg,uint8_t len,uint8_t *buf){

	HAL_StatusTypeDef ret;
	ret = HAL_I2C_Mem_Write(mpu_i2c_handlePtr, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    HAL_Delay(100);

    return ret;
}

HAL_StatusTypeDef MPU_Read_Len(uint8_t reg,uint8_t len,uint8_t *buf){

	HAL_StatusTypeDef ret;
    ret = HAL_I2C_Mem_Read(mpu_i2c_handlePtr, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 0xfff);
    HAL_Delay(100);

    return ret;
}

HAL_StatusTypeDef MPU_Write_Byte(uint8_t reg,uint8_t data){

	HAL_StatusTypeDef ret;
    unsigned char W_Data=0;
    W_Data = data;
    ret = HAL_I2C_Mem_Write(mpu_i2c_handlePtr, MPU_WRITE, reg, I2C_MEMADD_SIZE_8BIT, &W_Data, 1, 0xfff);
    HAL_Delay(100);

    return ret;
}

HAL_StatusTypeDef MPU_Read_Byte(uint8_t reg,uint8_t *data){

	HAL_StatusTypeDef ret;
    unsigned char R_Data=0;
    ret = HAL_I2C_Mem_Read(mpu_i2c_handlePtr, MPU_READ, reg, I2C_MEMADD_SIZE_8BIT, &R_Data, 1, 0xfff);
    *data = R_Data;
    HAL_Delay(100);

    return ret;
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Set_Gyro_Fsr(uint8_t fsr){

	return MPU_Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);  //设置陀螺仪满量程范围
}

HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t fsr){

	return MPU_Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);  //设置加速度传感器满量程范围
}

HAL_StatusTypeDef MPU_Set_LPF(uint16_t lpf){

	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;
	return MPU_Write_Byte(MPU_CFG_REG,data);  //设置数字低通滤波器
}

HAL_StatusTypeDef MPU_Set_Rate(uint16_t rate){

	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	  //自动设置LPF为采样率的一半
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Get_Gyroscope_Data(int16_t *gx,int16_t *gy,int16_t *gz){

	HAL_StatusTypeDef ret;
    uint8_t buf[6];
	ret = MPU_Read_Len(MPU_GYRO_XOUTH_REG, 6, buf);
	if(ret == HAL_OK){
		*gx = (((uint16_t)buf[0] << 8) | buf[1]);
		*gy = (((uint16_t)buf[2] << 8) | buf[3]);
		*gz = (((uint16_t)buf[4] << 8) | buf[5]);
	}
    return ret;
}


HAL_StatusTypeDef MPU_Get_Accelerometer_Data(int16_t *ax,int16_t *ay,int16_t *az){

	HAL_StatusTypeDef ret;
    uint8_t buf[6];
	ret = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 6, buf);
	if(ret == HAL_OK){
		*ax = (((uint16_t)buf[0] << 8) | buf[1]);
		*ay = (((uint16_t)buf[2] << 8) | buf[3]);
		*az = (((uint16_t)buf[4] << 8) | buf[5]);
	}
    return ret;;
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Calibrate_Accel_Data(int16_t *accel_bias) {

	HAL_StatusTypeDef ret;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    const uint16_t num_samples = 500;  // 采样数（越多越精确）

    for (int i = 0; i < num_samples; i++) {

        int16_t ax, ay, az;
        ret = MPU_Get_Accelerometer_Data(&ax, &ay, &az);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        HAL_Delay(2);  // 控制采样间隔

    }

    // 计算平均偏移（理想静止时 az 应为 1g，其他轴为 0）
    accel_bias[0] = sum_ax / num_samples;
    accel_bias[1] = sum_ay / num_samples;
    accel_bias[2] = sum_az / num_samples - (int16_t)(1.0f * ACCEL_SCALE); // 减去理论重力值（±2g 时 16384 = 1g）

    return ret;
}

HAL_StatusTypeDef MPU_Calibrate_Gyro_Data(int16_t *gyro_bias) {

	HAL_StatusTypeDef ret;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    const uint16_t num_samples = 500;

    for (int i = 0; i < num_samples; i++) {
        int16_t gx, gy, gz;
        ret = MPU_Get_Gyroscope_Data(&gx, &gy, &gz);

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        HAL_Delay(2);  // 控制采样间隔
    }

    gyro_bias[0] = sum_gx / num_samples;  // X 轴偏移
    gyro_bias[1] = sum_gy / num_samples;  // Y 轴偏移
    gyro_bias[2] = sum_gz / num_samples;  // Z 轴偏移

    return ret;
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Get_Gyroscope(float *gx,float *gy,float *gz, int16_t *gyro_bias){

	HAL_StatusTypeDef ret;
    int16_t gx_data, gy_data, gz_data;
	ret = MPU_Get_Gyroscope_Data(&gx_data, &gy_data, &gz_data);
	if(ret == HAL_OK){
		*gx = (gx_data - gyro_bias[0]) / GYRO_SCALE;
		*gy = (gx_data - gyro_bias[1]) / GYRO_SCALE;
		*gz = (gz_data - gyro_bias[2]) / GYRO_SCALE;
	}
    return ret;
}

HAL_StatusTypeDef MPU_Get_Accelerometer(float *ax,float *ay,float *az, int16_t *accel_bias){

	HAL_StatusTypeDef ret;
    int16_t ax_data, ay_data, az_data;
    ret = MPU_Get_Accelerometer_Data(&ax_data, &ay_data, &az_data);
	if(ret == HAL_OK){
		*ax = (ax_data - accel_bias[0]) / ACCEL_SCALE;
		*ay = (ay_data - accel_bias[1]) / ACCEL_SCALE;
		*az = (az_data - accel_bias[2]) / ACCEL_SCALE;
	}
    return ret;;
}

float MPU_Get_Temperature(void){

    unsigned char  buf[2];
    int16_t raw;
    float temp;

    MPU_Read_Len(MPU_TEMP_OUTH_REG, 2, buf);
    raw = ( buf[0]<<8 ) | buf[1];
    temp = (36.53 + ((double)raw)/340) * 100;  //  temp = (long)((35 + (raw / 340)) * 65536L);

    return temp/100.0f;
}

void AngleProcessor_Init(AngleProcessor* processor, float Q, float R) {

	processor->Q = Q;
	processor->R = R;
	processor->P = 1.0f;
	processor->K = 0.0f;

	processor->roll_hat = 0.0f;
	processor->pitch_hat = 0.0f;

}

void MPU_Get_Angle_KalmanFilter(int16_t* accel_bias ,int16_t* gyro_bias, AngleProcessor* processor, float time_interval){

	float dt = time_interval;
	float denominator = 0.0f;
	float gx, gy, gz;
	float roll_angle_gyro, pitch_angle_gyro;
	float roll_angle = processor->roll_hat, pitch_angle = processor->pitch_hat;

	float ax, ay, az;
	float roll_v, pitch_v;
	float roll_angle_accel, pitch_angle_accel;

	MPU_Get_Gyroscope(&gx, &gy, &gz, gyro_bias);
	roll_v = gx + (sin(roll_angle)*tan(pitch_angle))*gy + (cos(roll_angle)*tan(pitch_angle))*gz;
	pitch_v = cos(roll_angle)*gy - sin(roll_angle)*gz;
	roll_angle_gyro = roll_angle + roll_v * dt;
	pitch_angle_gyro = pitch_angle + pitch_v * dt;

	MPU_Get_Accelerometer(&ax, &ay, &az, accel_bias);

	denominator = az;
	if (fabs(denominator) < 1e-6) {denominator = 1e-6f;}  // 避免除零
	roll_angle_accel = atan2(ay, denominator) * 57.296f;   // arctan returns: -pi tp pi -- 180/pi = 57.296;

	denominator = sqrt(ay*ay + az*az);
	if (fabs(denominator) < 1e-6) {denominator = 1e-6f;}
	pitch_angle_accel = -atan2(ax, denominator) * 57.296f;

	processor->P = processor->P + processor->Q;                 // 协方差时间更新(P_k-1 -> P_k')
	processor->K = processor->P / (processor->P + processor->R);       // 卡尔曼增益更新 (K_k)

	processor->roll_hat = roll_angle_gyro + processor->K * (roll_angle_accel - processor->roll_hat); //  数据融合 X_hat = X_hat' + K_k*(X_measure - X_hat')
	processor->pitch_hat = pitch_angle_gyro + processor->K * (pitch_angle_accel - processor->pitch_hat); //  数据融合 X_hat = X_hat' + K_k*(X_measure - X_hat')

	processor->P *= (1 - processor->K);                  // 协方差后验更新(k' -> k)

}

void SendMessage(UART_HandleTypeDef *huart, const char *text) {
    uint8_t is_sent = 0;
    if (!is_sent) {
        HAL_UART_Transmit(huart, (uint8_t*)text, strlen(text), HAL_MAX_DELAY);
        is_sent = 1;
    }
}
