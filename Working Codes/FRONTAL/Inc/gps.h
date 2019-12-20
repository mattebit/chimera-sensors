//----------------GPS----------------//
/*
 *To use GPS functions you have to setup the UART port to communicate with it.
 *Possibly set the port as interrupt because the received data are cleaner.
 *Call gps_init() to setup the GPS
 *Every loop read the input data.
 *Then the function gps_read() looks for a sequence of letters like "GPRMC"
 *Look at NMEA protocol, there you can find all the strings that a GPS can send,
 *every one of them defines the types of data that the GPS sends.
 *checksum calculator http://www.hhhh.org/wiml/proj/nmeaxor.html
*/

#ifndef __GPS_H__
#define __GPS_H__

#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"

//GPS CONSTANTS
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ "$PMTK220,10000*2F\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ "$PMTK220,5000*1B\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_2HZ "$PMTK220,500*2B\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"

#define PMTK_CMD_STANDBY_MODE "$PMTK161,0*28\r\n" //enter in sleep mode

#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C\r\n"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F\r\n"
#define PMKT_SER_BAUD_DEFAULT "$PMTK251,0*28" //It works only if: a.full cold start command issue 
											  //				     b.enter standby mode
/*
	Supported NMEA Sentences
	0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude
	1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence
	2 NMEA_SEN_VTG, // GPVTG interval - Course over Ground and Ground Speed
	3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data
	4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites
	5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View
	6 //Reserved
	7 //Reserved
	13 //Reserved
	14 //Reserved
	15 //Reserved
	16 //Reserved
	17 //Reserved
	18 NMEA_SEN_MCHN, // PMTKCHN interval � GPS channel status

	Supported Frequency Setting
	0 - Disabled or not supported sentence
	1 - Output once every one position fix
	2 - Output once every two position fixes
	3 - Output once every three position fixes
	4 - Output once every four position fixes
	5 - Output once every five position fixes

 */
//turn on GPGGA and GPTVG
#define PMTK_SET_NMEA_OUTPUT_GGAVTG "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n" //activate GPGGA, GPVTG
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

#define PMTK_LOCUS_STARTLOG "$PMTK185,0*22"
#define PMTK_LOCUS_STOPLOG "$PMTK185,1*23"
#define PMTK_LOCUS_STARTSTOPACK "$PMTK001,185,3*3C"
#define PMTK_LOCUS_QUERY_STATUS "$PMTK183*38"
#define PMTK_LOCUS_ERASE_FLASH "$PMTK184,1*22"
#define LOCUS_OVERLAP 0
#define LOCUS_FULLSTOP 1

#define PMTK_ENABLE_SBAS "$PMTK313,1*2E"
#define PMTK_ENABLE_WAAS "$PMTK301,2*2E"

// standby command & boot successful message
#define PMTK_STANDBY "$PMTK161,0*28"
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36" // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

//----------------GPS----------------//
typedef struct
{
	UART_HandleTypeDef* huart_gps;
	char speed[7];			  //string of speed
	int speed_i;			  //speed converted into an int
	char latitude[10];		  //latitude string ddmm.mmmm
	long int latitude_calib;  //latitude_calibration (initial latitude)
	char latitude_o[2];		  //latitude orientation_string N=north or S=south
	long int latitude_i;	  //latitude converted into an long int
	int latitude_i_h;		  //high latitude (upper than comma)
	int latitude_i_l;		  //low latitude (lower than comma)
	char longitude[11];		  //longitude string dddmm.mmmm
	long int longitude_calib; //longitude_calibration (initial longitude)
	char longitude_o[2];	  //longitude orientation_string E=east or W=west
	long int longitude_i;	  //longitude converted into an long int
	int longitude_i_h;		  //high longitude (upper than comma)
	int longitude_i_l;		  //low longitude (lower than comma)
	char altitude[8];		  //altitude string
	int altitude_i;			  //altitude converted into a string
	char true_track_mode[5];  //true track mode string
	int true_track_mode_i;	  //true track mode integer
	char hour[3];			  //hour string
    char min[3];			  //min string
    char sec[3];			  //sec string
	char fix_status;
	char string[100];
} gps_struct;
int gps_read(UART_HandleTypeDef *huart, gps_struct *gps);
int gps_init(UART_HandleTypeDef *huart, gps_struct *gps);

#endif
