#ifndef GPS_H
#define GPS_H
/*
 *
 *
 *
 * GPS library
 * GPS_INIT() ->initialize the GPS. Put it in the main initialization. Example:
	gps_struct gps_main; //define the name of gps_structure istance
	if(GPS_INIT(&huart3,&gps_main)==0){
	  /--error--/
	}
 * GPS_INTERRUPT() -> put it in interrupt. Example:
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		GPS_INTERRUPT(huart,&gps_main);
	}
 *
 *
 *
 *
 */
#include "stm32f4xx_hal.h"
#include "function.h"
#include "gps.h"
#include "string.h"
#include "stdlib.h"

UART_HandleTypeDef* huart_GPS;
extern UART_HandleTypeDef huart2;
extern int ok;
uint8_t* dato_p;
int start_string_gps=0;
char string_gps[100];
int cont_string,cont_comma;
char data_string_gps;
char buffer_gps[2];
static int checksum(char * string_checksum, int size_string_checksum);
/*
 *
 *
 *
 * GPS library
 * gps_init() ->initialize the GPS. Put it in the main initialization. Example:
	gps_struct gps_main; //define the name of gps_structure istance
	if(gps_init(&huart3,&gps_main)==0){
	  /--error--/
	}
 * gps_read_it() -> put it in interrupt. Example:
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		gps_read_it(huart,&gps_main);
	}
 *
 *
 *
 *
 */



int gps_init(UART_HandleTypeDef* huart,gps_struct * gps){ //initialization of GPS
	//if return--> 0=error,1=ok
	//baud=9600
	huart_GPS=huart;
	huart_GPS->Init.BaudRate = 9600;
	HAL_UART_Init(huart_GPS);
	HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
	huart_GPS->Init.BaudRate = 57600;
	HAL_UART_Init(huart_GPS);
	HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
	huart_GPS->Init.BaudRate = 115200;
	HAL_UART_Init(huart_GPS);
	HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
	HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 200);
	HAL_Delay(500);
	HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAVTG, strlen(PMTK_SET_NMEA_OUTPUT_GGAVTG), 200);
	HAL_Delay(500);
	strcpy(gps->speed,"000.00");
	strcpy(gps->latitude,"0000.0000");
	strcpy(gps->latitude_o,"N");
	strcpy(gps->longitude,"00000.0000");
	strcpy(gps->longitude_o,"W");
	strcpy(gps->altitude,"0000.0");
	strcpy(gps->time,"000000");
	/*risposte:
		$PMTK001,220,3*30
		$PMTK001,314,3*36
	 */

	HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1); //request of rx buffer interrupt
	return 1;
}
int gps_read_it(UART_HandleTypeDef *huart, gps_struct* gps){
	int ret=0; //return--> 0=error,1=ok
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
		 //check if it's the huart_gps interrupt

		HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1); //request interrupt for the next data
		data_string_gps=buffer_gps[0]; //convert a pointer into a char
		if((start_string_gps==1)&&(data_string_gps!='$')){ //check that the new string has not started yet
			string_gps[cont_string]=data_string_gps; //save the data into the array
			cont_string++;
			if(string_gps[cont_string-1]=='\r'||string_gps[cont_string-1]=='\n'){  //indicates that the string is finishing
				cont_string--;
				string_gps[cont_string]='\0'; // '\0'=end of the string
				start_string_gps=0; //end of string
				if(string_gps[2]=='G'&&string_gps[3]=='G'&&string_gps[4]=='A'){ // operation when the string is GPGGA //
					strcpy(gps->GPGGA,string_gps);
					gps->gpgga_dim=cont_string;
					/*
					print_it(&huart2,gps->latitude);
					print_it(&huart2," ");
					print_it(&huart2,gps->longitude);
					print_it(&huart2," ");
					print_it(&huart2,gps->altitude);
					print_it(&huart2," ");*/
					//print_it(&huart2,"\r\n");
					/*gps->latitude_i=(long int)(atof(gps->latitude)*10000);
					gps->latitude_i_h=(int)(gps->latitude_i/10000);
					gps->latitude_i_l=(int)(gps->latitude_i-gps->latitude_i_h*10000);
					gps->longitude_i=(long int)(atof(gps->longitude)*100000);
					gps->longitude_i_h=(int)(gps->longitude_i/100000);
					gps->longitude_i_l=(int)(gps->longitude_i-gps->longitude_i_h*100000);
					gps->altitude_i=(int)(atof(gps->altitude)*100);
					CanSendMSG[0] = 0x08;
					CanSendMSG[1] = gps->longitude_i_h / 256;
					CanSendMSG[2] = gps->longitude_i_h % 256;
					CanSendMSG[3] = gps->longitude_i_l / 256;
					CanSendMSG[4] = gps->longitude_i_l % 256;
					CanSendMSG[5] = (int)gps->longitude_o;
					CanSendMSG[6] = gps->altitude_i / 256;
					CanSendMSG[7] = gps->altitude_i % 256;
					CAN_Send(0xC0, CanSendMSG, 8);
					*/

				}
				if(string_gps[2]=='V'&&string_gps[3]=='T'&&string_gps[4]=='G'){ 	// operation when the string is GPVTG //
					if(checksum(gps->GPGGA,gps->gpgga_dim)==1){ //check the checksum (if==true -> enter)
						int cont_comma=0,cont_latitude=0,cont_longitude=0,cont_altitude=0,cont_time=0;
						for(int i=5;i<100;i++){
							if(gps->GPGGA[i]==',')cont_comma++;
							else{
								if(cont_comma==1){
									gps->time[cont_time]=gps->GPGGA[i];
									cont_time++;
								}else if(cont_comma==2){ //save latitude
									gps->latitude[cont_latitude]=gps->GPGGA[i];
									cont_latitude++;
								}else if(cont_comma==3){ //save orientation of latitude
									gps->latitude_o[0]=gps->GPGGA[i];
								}else if(cont_comma==4){ //save longitude
									gps->longitude[cont_longitude]=gps->GPGGA[i];
									cont_longitude++;
								}else if(cont_comma==3){ //save orientation of longitude
									gps->longitude_o[0]=gps->GPGGA[i];
								}else if(cont_comma==9){ //save altitude
									gps->altitude[cont_altitude]=gps->GPGGA[i];
									cont_altitude++;
								}else if(cont_comma==10){
									i=100; //end the cicle
								}
							}

						}
					}else{
						ret=0;
					}
					if(checksum(string_gps,cont_string)==1){ //check the checksum (if==true -> enter)
						cont_comma=0;
						int cont_speed=0;
						for(int i=5;i<cont_string;i++){
							if(string_gps[i]==',')cont_comma++;
							else{
								if(cont_comma==7){ //save the speed
									gps->speed[cont_speed]=string_gps[i];
									cont_speed++;
								}else if(cont_comma==8){
									i=cont_string;
								}
							}
						}
						ok=1;
						ret=1;
						/*print_it(&huart2,gps->speed);
						print_it(&huart2,"\r\n");*/
						/*
						gps->speed_i=(int)(atof(gps->speed)*100);
						CanSendMSG[0] = 0x07;
						CanSendMSG[1] = gps->latitude_i_h / 256;
						CanSendMSG[2] = gps->latitude_i_h % 256;
						CanSendMSG[3] = gps->latitude_i_l / 256;
						CanSendMSG[4] = gps->latitude_i_l % 256;
						CanSendMSG[5] = (int)gps->latitude_o;
						CanSendMSG[6] = gps->speed_i / 256;
						CanSendMSG[7] = gps->speed_i % 256;
						CAN_Send(0xC0, CanSendMSG, 8);
						*/
						}else{
							ret=0;
						}
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
static int checksum(char * string_checksum, int size_string_checksum){ //check the checksum
	//return 1;
	int res=0;
	int offset_maiusc=(int)('A')-(int)('a');
	int i=0;
	for(i=0;(i<size_string_checksum)&&(string_checksum[i]!='*');i++){
		res=res^string_checksum[i];
	}
	char check[2]={string_checksum[i+1],string_checksum[i+2]};
	char res_char[2];
	sprintf(res_char,"%x",res);
	if(res<17){
		res_char[1]=res_char[0];
		res_char[0]='0';
	}
	for(int j=0;j<2;j++){ //convert to upper case letter
		if((int)res_char[j]>='a'&&(int)res_char[j]<='f'){
			res_char[j]=(char)((int)res_char[j]+offset_maiusc);
		}
	}
	if(res_char[0]==check[0]&&res_char[1]==check[1]){
		return 1; //checksum is correct
	}else {
		print_it(&huart2,"\r\n");
		print_it(&huart2,res_char);
		print_it(&huart2," ");
		print_it(&huart2,check);
		print_it(&huart2,"\r\n");
		return 0; //checksum failed
	}
}

#endif
