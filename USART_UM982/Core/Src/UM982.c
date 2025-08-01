/*
 * UM982.c
 *
 *  Created on: Jul 7, 2025
 *      Author: zk836
 */
#include "UM982.h"

#include "stdlib.h"
#include "usart.h"
#include "string.h"
#include "math.h"
#include <stdio.h>

Position_Data Track_Home;
Position_Data Plane_Position = {
	    .latitude = 0.0f,
	    .longitude = 0.0f,
	    .altitude = 0.0f,
	    .gps_num = 0,
	    .gps_status = 0,
		.length = 0.0f,
		.heading = 0.0f,
		.pitch = 0.0f
	};

/*
void Andly_GPS(Position_Data *last,uint8_t *buf){

    Mgps_msg GPS_MSG = {0};
	double degree_latitude, degree_longitude;
    uint8_t index = 0, temp_index = 0;

    if (buf[0] == 0) buf[0] = 1;		//防止有时候首字节为0 出现解析失败

	if(strstr((const char *)buf,"GGA")!=NULL){

		buf = (uint8_t *)(strstr((const char *)buf,"GGA,"));
        if(buf != NULL){

        index = NMEA_Comma_Pos(buf,2);//第2个逗号所在位置
        temp_index = index;
        for (index = temp_index; index < temp_index+20; index++)
            {
                if (buf[index] == ',') break;
                GPS_MSG.latitude[index-temp_index] = buf[index];//维度
            }

        index = NMEA_Comma_Pos(buf,4);//第4个逗号所在位置
        temp_index = index;
        for (index = temp_index; index < temp_index+20; index++)
            {
                if (buf[index] == ',') break;
                GPS_MSG.longitude[index-temp_index] = buf[index];//维度
            }

        index = NMEA_Comma_Pos(buf,6);//第6个逗号所在位置
        temp_index = index;
        for (index = temp_index; index < temp_index+20; index++)
            {
                if (buf[index] == ',') break;
                GPS_MSG.gps_status[index-temp_index] = buf[index];//维度
            }

        index = NMEA_Comma_Pos(buf,7);//第7个逗号所在位置
        temp_index = index;
        for (index = temp_index; index < temp_index+20; index++)
            {
                if (buf[index] == ',') break;
                GPS_MSG.gps_num[index-temp_index] = buf[index];//维度
            }

        index = NMEA_Comma_Pos(buf,9);//第9个逗号所在位置
        temp_index = index;
        for (index = temp_index; index < temp_index+20; index++)
            {
                if (buf[index] == ',') break;
                GPS_MSG.altitude[index-temp_index] = buf[index];//维度
            }

		double latitude = atof((char *)(GPS_MSG.latitude));
		float minute_latitude = 100.0f * modf(latitude / 100.0f, &degree_latitude);
		last->latitude = degree_latitude + minute_latitude / 60.0f;

		double longtitude = atof((char *)(GPS_MSG.longitude));
		float minute_longitude = 100.0f * modf(longtitude / 100.0f, &degree_longitude);
		last->longitude = degree_longitude + minute_longitude / 60.0f;

		last->gps_num = atof((char *)(GPS_MSG.gps_num));

//	printf("status is %s ;num is %s\r\n",GPS_MSG.latitude,GPS_MSG.longitude);

		last->gps_status = (uint8_t)(atof((char *)(GPS_MSG.gps_status)));

		last->altitude = (float)(atof((char *)(GPS_MSG.altitude)));

//  printf("GPSlat is %f,GPSlon is %f,GPSalt is %f,GPSnum is %d\r\n",last->latitude,last->longitude,last->altitude,last->gps_num);
        }
	}
}
*/

uint8_t NMEA_Comma_Pos(uint8_t *buf,uint8_t comma_number) {

    uint8_t *temp_buf = buf;
    while(comma_number) {
        if(*buf == '*'||*buf < ' '|| *buf > 'z') return 0XFF;//遇到'*'或者非法字符,则不存在第cx个逗号
        if(*buf == ',') comma_number--;
        buf++;
    }
    return (buf - temp_buf);
}

/*
static const char* find_field(const char* sentence, int field_index) {
    const char* p = sentence;
    // Skip the message ID, e.g., "$GPGGA,"
    p = strchr(p, ',');
    if (p == NULL) return NULL;

    while (field_index > 0 && p != NULL) {
        p = strchr(p + 1, ',');
        field_index--;
    }

    // Return pointer to the character *after* the comma
    return (p) ? (p + 1) : NULL;
*/


void Andly_GPS(Position_Data *data, uint8_t *buf) {

    const char* gga_start = strstr((const char *)buf, "GGA,");
    if (gga_start == NULL) {
        return;									// Not a GGA message or malformed, just exit.
    }

    // Pointers to the start of each relevant field
    const char *lat_p, *lon_p, *status_p, *num_p, *alt_p;

    // Use a single-pass approach to find all fields.(much more efficient than re-scanning for each field)
    const char* p = gga_start;

    p = strchr(p, ','); if (!p) return;			// Field 1: Coordinated Universal Time

    p = strchr(p+1, ','); if (!p) return;		// Field 2: Latitude
    lat_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 3: Latitude Direction

    p = strchr(p+1, ','); if (!p) return;		// Field 4: Longitude
    lon_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 5: Longitude Direction

    p = strchr(p+1, ','); if (!p) return;		// Field 6: GPS State Quality
    status_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 7: Number of Satellite
    num_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 8: hdop

    p = strchr(p+1, ','); if (!p) return;		// Field 9: Altitude
    alt_p = p + 1;


    const char* uniheading_ASCII_start = strstr((const char *)buf, "UNIHEADINGA,");
    if (uniheading_ASCII_start == NULL){
    	return;									// Not a UNIHEADINGA message or malformed, just exit.
    }

    const char *heading_p, *pitch_p, *length_p;
    p = uniheading_ASCII_start;

    p = strchr(p, ';'); if (!p) return;			// Field 1: Solution Status

    p = strchr(p+1, ','); if (!p) return;		// Field 2: Position Type

    p = strchr(p+1, ','); if (!p) return;		// Field 3: Length
    length_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 4: Heading
    heading_p = p + 1;

    p = strchr(p+1, ','); if (!p) return;		// Field 5: Pitch
    pitch_p = p + 1;

    // --- Now parse the fields --- Use sscanf for safe and efficient parsing. It handles empty fields gracefully.

    // If field is empty, sscanf returns 0, and data->gps_status is unchanged.
    if (*status_p != ',') {
        sscanf(status_p, "%hhu", &data->gps_status);
    }

    // We only trust data if we have a fix.
    if (data->gps_status > 0) {
        // Number of satellites
        if (*num_p != ',') {
            sscanf(num_p, "%hhu", &data->gps_num);
        }

        // Altitude
        if (*alt_p != ',') {
            sscanf(alt_p, "%f", &data->altitude);
        }

        // Latitude & Longitude (DDMM.MMMM format)
        double lat_nmea = 0.0, lon_nmea = 0.0;
        double lat_deg = 0.0, lon_deg = 0.0;

        if (*lat_p != ',') {
            sscanf(lat_p, "%lf", &lat_nmea);
            lat_deg = floor(lat_nmea / 100.0);
            data->latitude = lat_deg + (lat_nmea - lat_deg * 100.0) / 60.0;
        }

        if (*lon_p != ',') {
            sscanf(lon_p, "%lf", &lon_nmea);
            lon_deg = floor(lon_nmea / 100.0);
            data->longitude = lon_deg + (lon_nmea - lon_deg * 100.0) / 60.0;
        }

        // Length
    	if (*length_p != ','){
    		sscanf(length_p, "%f", &data->length);
    	}

    	// Heading
    	if (*heading_p != ','){
    		sscanf(heading_p, "%f", &data->heading);
    	}

    	// Pitch
    	if (*pitch_p != ','){
    		sscanf(pitch_p, "%f", &data->pitch);
    	}

        if (data->gps_status == 4 || data->gps_status == 5){


        }
    }
}
