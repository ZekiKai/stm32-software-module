/*
 * UM982.h
 *
 *  Created on: Jul 7, 2025
 *      Author: zk836
 */

#ifndef INC_UM982_H_
#define INC_UM982_H_

#include "stm32f4xx_hal.h"

typedef struct {
    uint8_t latitude[20];            //纬度
    uint8_t longitude[20];           //经度
    uint8_t altitude[10];    	    //高度

    uint8_t Location[4];            //定位有效A 定位无效V
    uint8_t gps_num[4];             //卫星数量
	uint8_t gps_status[4];		    //定位状态

} Mgps_msg;

typedef struct {

    float latitude;            //纬度
    float longitude;           //经度
	float altitude;    		   //高度

    uint8_t gps_num;           //卫星数量
	uint8_t gps_status;		   //定位状态

	float length;  			   //基线长
	float heading; 			   //航向角
	float pitch;			   //俯仰角

} Position_Data;

extern Position_Data Track_Home;
extern Position_Data Plane_Position;

void Andly_GPS(Position_Data *last,uint8_t *buf);				//解析定位信息

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t cx);

#endif /* INC_UM982_H_ */
