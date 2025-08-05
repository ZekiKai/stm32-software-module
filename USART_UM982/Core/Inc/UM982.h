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

    float latitude;
    float longitude;
	float altitude;

    uint8_t gps_num;
	uint8_t gps_status;

	float length;
	float heading;
	float pitch;

} SatelliteData;

typedef struct {

	char *gpgga_start;

	char *lat_p;
	char *lon_p;
	char *alt_p;

	char *status_p;
	char *num_p;

    float latitude;
    float longitude;
	float altitude;

    uint8_t gps_num;
	uint8_t gps_status;

} ProcessorForGPGGA;

typedef struct {

	char *uniheading_start;

    char *heading_p;
    char *pitch_p;
    char *length_p;

	float length;
	float heading;
	float pitch;

} ProcessorForUNIHEADING;

extern SatelliteData plane_from_satellite;

void satellite_analyze(SatelliteData *last,uint8_t *buf);

void gpgga_analyze(SatelliteData* data, uint8_t* buf);
void uniheading_analyze(SatelliteData* data, uint8_t* buf);

#endif /* INC_UM982_H_ */
