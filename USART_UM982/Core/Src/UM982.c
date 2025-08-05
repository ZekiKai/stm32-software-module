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

SatelliteData plane_from_satellite = {

	    .latitude = 0.0f,
	    .longitude = 0.0f,
	    .altitude = 0.0f,

	    .gps_num = 0,
	    .gps_status = 0,

		.length = 0.0f,
		.heading = 0.0f,
		.pitch = 0.0f
	};


void satellite_analyze(SatelliteData *data, uint8_t *buf) {

	gpgga_analyze(data, buf);

	if (data->gps_status == 4 || data->gps_status == 5){

		uniheading_analyze(data, buf);
	}
}

void gpgga_analyze(SatelliteData* data, uint8_t* buf){

	ProcessorForGPGGA gpgga_analyzer;

	gpgga_analyzer.gpgga_start = strstr((const char *)buf, "GGA,");

	// Not a GGA message or malformed, just exit.
    if (gpgga_analyzer.gpgga_start == NULL) {
        return;
    }

    // Use a single-pass approach to find all fields.(much more efficient than re-scanning for each field)
    char* pointer = gpgga_analyzer.gpgga_start;

    pointer = strchr(pointer, ','); if (!pointer) return;		// Field 1: Coordinated Universal Time

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 2: Latitude
    gpgga_analyzer.lat_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 3: Latitude Direction

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 4: Longitude
    gpgga_analyzer.lon_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 5: Longitude Direction

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 6: GPS State Quality
    gpgga_analyzer.status_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 7: Number of Satellite
    gpgga_analyzer.num_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 8: hdop

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 9: Altitude
    gpgga_analyzer.alt_p = pointer + 1;

    // --- Now parse the fields --- Use sscanf for safe and efficient parsing. It handles empty fields gracefully.
    // If field is empty, sscanf returns 0, and data->gps_status is unchanged.

    if (*(gpgga_analyzer.status_p) != ',') {
        sscanf(gpgga_analyzer.status_p, "%hhu", &data->gps_status);
    }

    if ((data->gps_status) > 0) {

        if (*(gpgga_analyzer.num_p) != ',') {
            sscanf(gpgga_analyzer.num_p, "%hhu", &data->gps_num);
        }

        if (*(gpgga_analyzer.alt_p) != ',') {
            sscanf(gpgga_analyzer.alt_p, "%f", &data->altitude);
        }

        double lat_nmea = 0.0, lon_nmea = 0.0;
        double lat_deg = 0.0, lon_deg = 0.0;

        if (*(gpgga_analyzer.lat_p) != ',') {

            sscanf(gpgga_analyzer.lat_p, "%lf", &lat_nmea);
            lat_deg = floor(lat_nmea / 100.0);
            data->latitude = lat_deg + (lat_nmea - lat_deg * 100.0) / 60.0;
        }

        if (*(gpgga_analyzer.lon_p) != ',') {

            sscanf(gpgga_analyzer.lon_p, "%lf", &lon_nmea);
            lon_deg = floor(lon_nmea / 100.0);
            data->longitude = lon_deg + (lon_nmea - lon_deg * 100.0) / 60.0;
        }
    }
}

void uniheading_analyze(SatelliteData* data, uint8_t* buf){

	ProcessorForUNIHEADING uniheading_analyzer;

	uniheading_analyzer.uniheading_start = strstr((const char *)buf, "UNIHEADINGA,");
    if (uniheading_analyzer.uniheading_start == NULL){
    	return;									// Not a UNIHEADINGA message or malformed, just exit.
    }

    char* pointer = uniheading_analyzer.uniheading_start;

    pointer = strchr(pointer, ';'); if (!pointer) return;			// Field 1: Solution Status

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 2: Position Type

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 3: Length
    uniheading_analyzer.length_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 4: Heading
    uniheading_analyzer.heading_p = pointer + 1;

    pointer = strchr(pointer+1, ','); if (!pointer) return;		// Field 5: Pitch
    uniheading_analyzer.pitch_p = pointer + 1;

    if (*(uniheading_analyzer.length_p) != ',') {
        sscanf(uniheading_analyzer.length_p, "%f", &data->length);
    }
    if (*(uniheading_analyzer.heading_p) != ',') {
        sscanf(uniheading_analyzer.heading_p, "%f", &data->heading);
    }
    if (*(uniheading_analyzer.pitch_p) != ',') {
        sscanf(uniheading_analyzer.pitch_p, "%f", &data->pitch);
    }

}
