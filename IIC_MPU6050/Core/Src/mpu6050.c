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

uint8_t mpu_init(void){

	char templete_message[50];
	uint8_t* templete_data = 0;
	HAL_StatusTypeDef ret;
    HAL_I2C_Init(mpu_i2c_handlePtr);

    ret = HAL_I2C_IsDeviceReady(mpu_i2c_handlePtr, MPU_WRITE, 1, 100);
    if (ret == HAL_OK){
    	send_message_to_user("The device is ready \n");
    } else {
    	send_message_to_user("The device is not ready \n");
    }

    // Reset
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);

    // Exiting from the Sleeping Mode
    ret = MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);
    if (ret == HAL_OK){
    	send_message_to_user("Exiting from the Sleep \n");
    } else {
    	send_message_to_user("Not exiting from the Sleep \n");
    }

    // Configuring Accelerometer
    ret = MPU_Set_Gyro_Fsr(3);
    if (ret == HAL_OK){
    	send_message_to_user("Configuring Gyroscope \n");
    } else {
    	send_message_to_user("Not Configuring Gyroscope \n");
    }

    // Configuring Gyroscope
    ret = MPU_Set_Accel_Fsr(0);
    if (ret == HAL_OK){
    	send_message_to_user("Configuring Accelerometer \n");
    } else {
    	send_message_to_user("Not Configuring Accelerometer \n");
    }

    MPU_Set_Rate(50);						// Configure Sampling Rate - 50Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);	// Disable the Interrupt
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00);	// Disable the IIC Master Function
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);	// Diable FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80);	// Configure INT Pin - Enable at Low

    ret = MPU_Read_Byte(MPU_DEVICE_ID_REG, templete_data);
	sprintf(templete_message, "\r\nMPU6050:0x%2x\r\n", *templete_data);
	send_message_to_user(templete_message);

    if(*templete_data == MPU_ADDR)			// Check the Device ID
    {
    	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	// Configure CLKSEL - PLL|x-axis as reference
    	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	// Enable the accelemeter and gyrometer
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

	return MPU_Write_Byte(MPU_GYRO_CFG_REG, (fsr << 3));
}

HAL_StatusTypeDef MPU_Set_Accel_Fsr(uint8_t fsr){

	return MPU_Write_Byte(MPU_ACCEL_CFG_REG, (fsr<<3));
}

HAL_StatusTypeDef MPU_Set_LPF(uint16_t lpf){

	uint8_t data = 0;
	if(lpf >= 188) data=1;
	else if(lpf >= 98) data=2;
	else if(lpf >= 42) data=3;
	else if(lpf >= 20) data=4;
	else if(lpf >= 10) data=5;
	else data = 6;

	return MPU_Write_Byte(MPU_CFG_REG,data);
}

HAL_StatusTypeDef MPU_Set_Rate(uint16_t rate){

	uint8_t data;
	if(rate > 1000) rate = 1000;
	if(rate < 4) rate = 4;

	data = 1000 / (rate-1);
	data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data);

 	return MPU_Set_LPF((rate / 2));
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Get_Gyroscope_Data(int16_t *gx,int16_t *gy,int16_t *gz){

	HAL_StatusTypeDef ret;
    uint8_t buf[6];
	ret = MPU_Read_Len(MPU_GYRO_XOUTH_REG, 6, buf);
	if(ret == HAL_OK){
		*gx = ( ((uint16_t)buf[0] << 8) | buf[1]);
		*gy = ( ((uint16_t)buf[2] << 8) | buf[3]);
		*gz = ( ((uint16_t)buf[4] << 8) | buf[5]);
	}
    return ret;
}


HAL_StatusTypeDef MPU_Get_Accelerometer_Data(int16_t *ax,int16_t *ay,int16_t *az){

	HAL_StatusTypeDef ret;
    uint8_t buf[6];
	ret = MPU_Read_Len(MPU_ACCEL_XOUTH_REG, 6, buf);
	if(ret == HAL_OK){
		*ax = ( ((uint16_t)buf[0] << 8) | buf[1]);
		*ay = ( ((uint16_t)buf[2] << 8) | buf[3]);
		*az = ( ((uint16_t)buf[4] << 8) | buf[5]);
	}
    return ret;
}

/*-------------------------------------------------------------*/

HAL_StatusTypeDef MPU_Calibrate_Accel_Data(int16_t *accel_bias) {

	HAL_StatusTypeDef ret;
    int32_t sum_ax = 0, sum_ay = 0, sum_az = 0;

    const uint16_t num_samples = 200;
    for (int i = 0; i < num_samples; i++) {

        int16_t ax, ay, az;
        ret = MPU_Get_Accelerometer_Data(&ax, &ay, &az);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        HAL_Delay(2);
    }

    accel_bias[0] = sum_ax / num_samples;
    accel_bias[1] = sum_ay / num_samples;
    accel_bias[2] = (sum_az / num_samples) - (int16_t)(1.0f * ACCEL_SCALE);

    return ret;
}

HAL_StatusTypeDef MPU_Calibrate_Gyro_Data(int16_t *gyro_bias) {

	HAL_StatusTypeDef ret;
    int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;

    const uint16_t num_samples = 200;
    for (int i = 0; i < num_samples; i++) {

        int16_t gx, gy, gz;
        ret = MPU_Get_Gyroscope_Data(&gx, &gy, &gz);

        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        HAL_Delay(2);
    }

    gyro_bias[0] = sum_gx / num_samples;
    gyro_bias[1] = sum_gy / num_samples;
    gyro_bias[2] = sum_gz / num_samples;

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

    return ret;
}

HAL_StatusTypeDef MPU_Get_Temperature(float* temp){

	HAL_StatusTypeDef ret;
    uint8_t buf[2];

    int16_t temp_data;
    ret = MPU_Read_Len(MPU_TEMP_OUTH_REG, 2, buf);
    temp_data = ( buf[0]<<8 ) | buf[1];

    if (ret == HAL_OK){
    	*temp = (float)(36.53 + ((double)temp_data)/340);
    }

    return ret;
}

/*---------------------------------------------------------------------------------*/

void imu_processor_init(IMUProcessor* processor, float Q, float R, float time_interval) {

	processor->pitch_processor.A = 1;
	processor->pitch_processor.B = time_interval;
	processor->pitch_processor.H = 1;

	processor->roll_processor.A = 1;
	processor->roll_processor.B = time_interval;
	processor->roll_processor.H = 1;

	processor->pitch_processor.Q = processor->roll_processor.Q = Q;
	processor->pitch_processor.R = processor->roll_processor.R = R;

	processor->pitch_processor.K = processor->roll_processor.R = 0.0f;

	processor->pitch_processor.x = processor->roll_processor.x = 0.0f;


	MPU_Calibrate_Accel_Data(&(processor->accel_bias));
	MPU_Calibrate_Gyro_Data(&(processor->accel_bias));

}

void imu_process(IMUProcessor* processor){

	float denominator = 0.0f;
	float gx, gy, gz;

	float roll_angle = processor->roll_processor.x;
	float pitch_angle = processor->pitch_processor.x;

	float ax, ay, az;
	float roll_v, pitch_v;
	float roll_angle_accel, pitch_angle_accel;

	MPU_Get_Gyroscope(&gx, &gy, &gz, &(processor->gyro_bias));

	roll_v = gx + (sin(roll_angle)*tan(pitch_angle))*gy + (cos(roll_angle)*tan(pitch_angle))*gz;
	pitch_v = cos(roll_angle)*gy - sin(roll_angle)*gz;

	MPU_Get_Accelerometer(&ax, &ay, &az, &(processor->accel_bias));

	denominator = az;
	if (fabs(denominator) < 1e-6) {denominator = 1e-6f;}
	roll_angle_accel = atan2(ay, denominator) * 57.296f;   // arctan returns: -pi tp pi -- 180/pi = 57.296;

	denominator = sqrt(ay*ay + az*az);
	if (fabs(denominator) < 1e-6) {denominator = 1e-6f;}
	pitch_angle_accel = -atan2(ax, denominator) * 57.296f;

	kf_prediction(&(processor->pitch_processor), pitch_v);
	kf_prediction(&(processor->roll_processor), roll_v);

	kf_correction(&(processor->pitch_processor), pitch_angle_accel);
	kf_correction(&(processor->roll_processor), roll_angle_accel);

}

/*-------------------------------------------------------------------------------------*/

void send_message_to_user(const char *text) {
    uint8_t is_sent = 0;
    if (!is_sent) {
        HAL_UART_Transmit(&huart2, (uint8_t*)text, strlen(text), HAL_MAX_DELAY);
        is_sent = 1;
    }
}
