#ifndef GPS_H
#define GPS_H

#include "stm32f4xx_hal.h"
#include "function.h"
#include "gps.h"
#include "string.h"

UART_HandleTypeDef* huart_GPS;
uint8_t* dato_p;
int start_string_gps=0;
char string_gps[100];
int cont_string,cont_comma,end_control_string_def;
char data_string_gps;
char speed[]="000.00,km/h";
char latitude[]="0000.0000,N";
char longitude[]="00000.0000,W";
char altitude[]="0000.00,M";
char buffer_gps[100];

int GPS_INIT(UART_HandleTypeDef* huart){
	//default gps baud rate is 9600
	//set gps baud rate to 57600
	huart_GPS=huart;

	/*
	for(int i=0;i<10;i++)HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_57600, strlen(PMTK_SET_BAUD_57600), 100);
	HAL_Delay(50);
	huart_GPS->Init.BaudRate = 9600;
	if (HAL_UART_Init(huart_GPS) != HAL_OK){
		return 0;
	}*/

	//send other commands to speed up the data flow
	/*for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(huart, (uint8_t*)PMTK_API_SET_FIX_CTL_5HZ, strlen(PMTK_API_SET_FIX_CTL_5HZ), 20);
		HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
	}

	for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(huart, (uint8_t*)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 20);
		HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
	}*/
	HAL_Delay(100);
	HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1);
	return 1;
}
void GPS_Awake(UART_HandleTypeDef *huart){
	for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(huart, (uint8_t*)PMTK_AWAKE, strlen(PMTK_AWAKE), 20);
		HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
	}
	for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(huart, (uint8_t*)PMTK_Q_RELEASE, strlen(PMTK_Q_RELEASE), 20);
		HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
	}
}
int GPS_INTERRUPT(UART_HandleTypeDef *huart){
	int ret=0;
	/*
	* Example of strings
	* $GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65
	* $GPGSA,A,3,29,21,26,15,18,09,06,10,,,,,2.32,0.95,2.11*00
	* $GPGSV,3,1,09,29,36,029,42,21,46,314,43,26,44,020,43,15,21,321,39*7D
	  $GPGSV,3,2,09,18,26,314,40,09,57,170,44,06,20,229,37,10,26,084,37*77
	  $GPGSV,3,3,09,07,,,26*73
	* $GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C
	* $GPVTG,165.48,T,,M,0.03,N,0.06,K,A*37
	* $PGTOP,11,3 *6F
	*
	*
	*
	* 	$GPBOD - Bearing, origin to destination
		$GPBWC - Bearing and distance to waypoint, great circle
		$GPGGA - Global Positioning System Fix Data
		$GPGLL - Geographic position, latitude / longitude
		$GPGSA - GPS DOP and active satellites
		$GPGSV - GPS Satellites in view
		$GPHDT - Heading, True
		$GPR00 - List of waypoints in currently active route
		$GPRMA - Recommended minimum specific Loran-C data
		$GPRMB - Recommended minimum navigation info
		$GPRMC - Recommended minimum specific GPS/Transit data
		$GPRTE - Routes
		$GPTRF - Transit Fix Data
		$GPSTN - Multiple Data ID
		$GPVBW - Dual Ground / Water Speed
		$GPVTG - Track made good and ground speed
		$GPWPL - Waypoint location
		$GPXTE - Cross-track error, Measured
		$GPZDA - Date & Time
		http://aprs.gids.nl/nmea/
	*/
	if(huart==huart_GPS){
		HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1);
		data_string_gps=buffer_gps[0]; //convert a pointer into a char
		if((start_string_gps==1)&&(data_string_gps!='$')){ //check that the new string has not started yet
			string_gps[cont_string]=data_string_gps; //save che data into the array
			cont_string++;
			if(string_gps[cont_string-3]=='*'){  //indicates that the string is finishing
				string_gps[cont_string]='\0';
				data_string_gps=0;
				//print(string_gps);
				if(checksum(string_gps,cont_string)==1){ //check the checksum (if==true -> enter)
					if(string_gps[2]=='G'&&string_gps[3]=='G'&&string_gps[4]=='A'){ // operation when the string is GPGGA //

						int cont_comma=0,cont_latitude=0,cont_longitude=0,cont_altitude=0;
						for(int i=5;i<cont_string;i++){
							if(string_gps[i]==',')cont_comma++;
							else{
								if(cont_comma==2){
									latitude[cont_latitude]=string_gps[i];
									cont_latitude++;
								}else if(cont_comma==3){
									latitude[10]=string_gps[i];;
								}else if(cont_comma==4){
									longitude[cont_longitude]=string_gps[i];
									cont_longitude++;
								}else if(cont_comma==3){
									longitude[11]=string_gps[i];
								}if(cont_comma==9){
									altitude[cont_altitude]=string_gps[i];
									cont_altitude++;
								}
							}

						}

					}else if(string_gps[2]=='G'&&string_gps[3]=='S'&&string_gps[4]=='A'){ // operation when the string is GPGSA //
						//print("GPGSA");
					}else if(string_gps[2]=='G'&&string_gps[3]=='S'&&string_gps[4]=='V'){ // operation when the string is GPGSV //
						//print("GPGSV");
					}else if(string_gps[2]=='R'&&string_gps[3]=='M'&&string_gps[4]=='C'){ // operation when the string is GPRMC //
						//print("GPRMC");
					}else if(string_gps[2]=='V'&&string_gps[3]=='T'&&string_gps[4]=='G'){ 	// operation when the string is GPVTG //
						//print("GPVTG");
						int cont_comma=0,cont_speed=0;
						for(int i=5;i<cont_string;i++){
							if(string_gps[i]==',')cont_comma++;
							else{
								if(cont_comma==7){
									speed[cont_speed]=string_gps[i];
									cont_speed++;
								}
							}
						}
						print(speed);
						print("\n");
						print(latitude);
						print(longitude);
						print(altitude);
						print("\n");

					}else if(string_gps[2]=='T'&&string_gps[3]=='O'&&string_gps[4]=='P'){ // operation when the string is GPTOP //
						//print("GPTOP");
					}
					ret=1;
				}else{
					//print("error in the checksum control"); // error in the checksum control
					//data_string_gps=0; //end the gps function
					ret=0;
				}
			}
		}else{
			if(data_string_gps=='$'){ //check if data indicates the start of new string
				start_string_gps=1; //new string started
				cont_string=0; //set the counter to 1
			}
		}
	}
	return ret;

}

int checksum(char * string_checksum, int size_string_checksum){ //check the checksum
	int res=0;
	int offset_maiusc=(int)('A')-(int)('a');
	for(int i=0; i<size_string_checksum-3;i++){
		res=res^string_checksum[i];
	}
	char check[2]={string_checksum[size_string_checksum-2],string_checksum[size_string_checksum-1]};
	char res_char[2];
	sprintf(res_char,"%x",res);
	for(int i=0;i<2;i++){
		if((int)res_char[i]>='a'&&(int)res_char[i]<='f'){
			res_char[i]=(char)((int)res_char[i]+offset_maiusc);
		}
	}
	if(res_char[0]==check[0]&&res_char[1]==check[1]){ //--------------trovare modo per convertire res in esadecimale-------------------//
		return 1; //checksum is correct
	}else return 0; //checksum failed
}
#endif
