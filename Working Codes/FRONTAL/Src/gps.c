#include "gps.h"

static int checksum(char *string_checksum, int size_string_checksum);
int start_string_gps = 0;
char string_gps[100];
int cont_string, cont_comma;
char data_string_gps;
int msg_arrived_s = 0;
extern char buffer_gps[50];
extern int msg_arrived;
extern gps_struct gps;
extern char msg_gps[3];
extern UART_HandleTypeDef huart2;


int gps_init(UART_HandleTypeDef *huart, gps_struct *gps)
{ //initialization of GPS

	//if return--> 0=error,1=ok
	huart->Init.BaudRate = 9600;
    if (HAL_UART_DeInit(huart) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2,(uint8_t*)"ERRORE 3\r\n",6,10);
    }else{
        HAL_UART_Transmit(&huart2,(uint8_t*)"deinit OK\r\n",strlen("deinit OK\r\n"),10);
    }
	if (HAL_UART_Init(huart) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2,(uint8_t*)"ERRORE 4\r\n",6,10);
    }else{
        HAL_UART_Transmit(&huart2,(uint8_t*)"init OK\r\n",strlen("init OK\r\n"),10);
    }
	/*HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
	huart->Init.BaudRate = 57600;
    HAL_UART_DeInit(huart);
	HAL_UART_Init(huart);
	HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
    if (HAL_UART_DeInit(huart) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2,(uint8_t*)"ERRORE 5\r\n",6,10);
    }
    huart->Init.BaudRate = 115200;
	if (HAL_UART_Init(huart) != HAL_OK)
    {
        HAL_UART_Transmit(&huart2,(uint8_t*)"ERRORE 6\r\n",6,10);
    }
	HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
	HAL_Delay(500);
	HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 200);
	HAL_Delay(500);
    */
	HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_NMEA_OUTPUT_GGAVTG, strlen(PMTK_SET_NMEA_OUTPUT_GGAVTG), 200);
	HAL_Delay(500);
    HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_NMEA_OUTPUT_GGAVTG, strlen(PMTK_SET_NMEA_OUTPUT_GGAVTG), 200);
	HAL_Delay(500);
    HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_NMEA_OUTPUT_GGAVTG, strlen(PMTK_SET_NMEA_OUTPUT_GGAVTG), 200);
	HAL_Delay(500);
    HAL_UART_Transmit(huart, (uint8_t *)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 200);
	strcpy(gps->speed, "000.00");
	strcpy(gps->latitude, "0000.0000");
	strcpy(gps->latitude_o, "N");
	strcpy(gps->longitude, "00000.0000");
	strcpy(gps->longitude_o, "W");
	strcpy(gps->altitude, "0000.0");
	strcpy(gps->hour, "00");
    strcpy(gps->min, "00");
    strcpy(gps->sec, "00");
	//HAL_UART_Receive_IT(huart, (uint8_t *)msg_gps, 1); //request of rx buffer interrupt

	return 1;
}

int gps_read(UART_HandleTypeDef *huart, gps_struct *gps)
{

	int ret = 0; //return--> 0=error,1=ok

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
    //check if it's the huart_gps interrupt
    
    char buffer_gps_s[50]; 
    //HAL_UART_Transmit(&huart2,(uint8_t*)&buffer_gps[0],1,10);
    if(msg_arrived > 0){
        for(int i = 0; i < msg_arrived; i++){
            buffer_gps_s[i] = buffer_gps[i];
            //HAL_UART_Transmit(&huart2,(uint8_t*)&buffer_gps_s[i],1,10);
        }
        for(int j = 0; j < msg_arrived; j++){
            data_string_gps = buffer_gps_s[j];
            if ((start_string_gps == 1) && (data_string_gps != '$'))
            {											   //check that the new string has not started yet
                string_gps[cont_string] = data_string_gps; //save the data into the array
                if (string_gps[cont_string] == '\r' )
                { //indicates that the string is finishing
                    string_gps[cont_string] = '\0'; // '\0'=end of the string
                    start_string_gps = 0; //end of string
                    /*char txt[100];
                    sprintf(txt,"%c%c%c\r\n",string_gps[2], string_gps[3], string_gps[4]);
                    HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);*/
                    if (string_gps[2] == 'G' && string_gps[3] == 'G' && string_gps[4] == 'A')
                    { // operation when the string is GPGGA //
                        //memcpy(gps->string, "", 100);
                        //memcpy(gps->string, string_gps, strlen(string_gps));

                        if (checksum(string_gps, cont_string) == 1)
                        { //check the checksum (if==true -> enter)
                            int cont_comma = 0, cont_latitude = 0, cont_longitude = 0, cont_altitude = 0, cont_time = 0;
                            for (int i = 5; i < cont_string; i++)
                            {
                                if (string_gps[i] == ',')
                                    cont_comma++;
                                else
                                {
                                    if (cont_comma == 1)
                                    { //save the time
                                        if(cont_time < 2){
                                            gps->hour[cont_time] = string_gps[i];
                                            cont_time++;
                                        }else if(cont_time < 4){
                                            gps->min[cont_time % 2] = string_gps[i];
                                            cont_time++;
                                        }else if(cont_time <6){
                                            gps->sec[cont_time % 2] = string_gps[i];
                                            cont_time++;
                                        }
                                        
                                    }
                                    else if (cont_comma == 2)
                                    { //save latitude

                                        gps->latitude[cont_latitude] = string_gps[i];
                                        cont_latitude++;
                                    }
                                    else if (cont_comma == 3)
                                    { //save orientation of latitude
                                        gps->latitude_o[0] = string_gps[i];
                                    }
                                    else if (cont_comma == 4)
                                    { //save longitude
                                        gps->longitude[cont_longitude] = string_gps[i];
                                        cont_longitude++;
                                    }
                                    else if (cont_comma == 5)
                                    { //save orientation of longitude
                                        gps->longitude_o[0] = string_gps[i];
                                    }
                                    else if (cont_comma == 6)
                                    {
                                        gps->fix_status = string_gps[i];
                                    }
                                    else if (cont_comma == 9)
                                    { //save altitude
                                        gps->altitude[cont_altitude] = string_gps[i];
                                        cont_altitude++;
                                    }
                                    else if (cont_comma == 10)
                                    {
                                        i = cont_string; //end the cicle
                                    }
                                }
                            }

                            //-- operation to split data and send them --//
                            if (gps->fix_status == '0')
                            {
                                char txt[100];
                                //sprintf(txt,"NO CONNECTION GGA\r\n");
                                //HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                                gps->latitude_i_h = 0;
                                gps->latitude_i_l = 0;
                                gps->longitude_i_h = 0;
                                gps->longitude_i_l = 0;
                                gps->altitude_i = 0;
                            }
                            else
                            {
                                char txt[100];
                                gps->latitude_i = (long int)(atof(gps->latitude) * 1000);
                                gps->longitude_i = (long int)(atof(gps->longitude) * 1000);
                                
                                /*sprintf(txt,"latitude %ld %c\r\n", gps->latitude_i, gps->latitude_o[0]);
                                HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                                gps->longitude_i = (long int)(atof(gps->longitude) * 100);
                                sprintf(txt,"longitude %ld %c\r\n",gps->longitude_i, gps->longitude_o[0]);
                                HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);/*
                                /*sprintf(txt,"time %c%c:%c%c:%c%c\r\n", gps->hour[0], gps->hour[1], gps->min[0], gps->min[1], gps->sec[0], gps->sec[1]);
                                HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);*/
                                gps->altitude_i = (int)(atof(gps->altitude) * 10);
                                gps->latitude_i_h = (int)(gps->latitude_i >> 16);
                                gps->latitude_i_l = (int)(gps->latitude_i - (gps->latitude_i_h << 16));
                                gps->longitude_i_h = (int)(gps->longitude_i >> 16);
                                gps->longitude_i_l = (int)(gps->longitude_i - (gps->longitude_i_h << 16));
                            }
                            ret = 1;
                        }
                        else
                        {
                            char txt[100];
                            //sprintf(txt,"\r\nCHECKSUM FAIL GGA\r\n");
                            //HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                            ret = 0; //checksum failed
                        }
                    }
                    else if (string_gps[2] == 'V' && string_gps[3] == 'T' && string_gps[4] == 'G')
                    { // operation when the string is GPVTG //
                        if (checksum(string_gps, cont_string) == 1)
                        { //check the checksum (if==true -> enter)
                            cont_comma = 0;
                            int cont_speed = 0;
                            int cont_true_track_mode = 0;
                            for (int i = 5; i < cont_string; i++)
                            {
                                if (string_gps[i] == ',')
                                    cont_comma++;
                                else
                                {
                                    if (cont_comma == 1){
                                        gps->true_track_mode[cont_true_track_mode] = string_gps[i];
                                        cont_true_track_mode++;
                                    }
                                    else if (cont_comma == 7)
                                    { //save the speed
                                        gps->speed[cont_speed] = string_gps[i];
                                        cont_speed++;
                                    }
                                    else if (cont_comma == 8)
                                    {
                                        i = cont_string;
                                    }
                                }
                            }
                            //-- operation to split data and send them --//
                            if (gps->fix_status == '0')
                            {
                                char txt[100];
                                //sprintf(txt,"NO CONNECTION VTG\r\n");
                                //HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                                gps->speed_i = 0;
                            }
                            else
                            {
                                char txt[100];
                                gps->speed_i = (int)(atof(gps->speed) * 100);
                                gps->true_track_mode_i = (int)(atof(gps->true_track_mode)*10);
                                
                                /*sprintf(txt,"speed: %d\r\n",gps->speed_i);
                                HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                                sprintf(txt,"TTM: %d\r\n", gps->true_track_mode_i);
                                HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);*/
                            }

                            ret = 1;
                        }
                        else
                        {
                            char txt[100];
                            //sprintf(txt,"\r\nCHECKSUM FAIL VTG\r\n");
                            //HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                            ret = 0; //checksum failed
                        }
                    }
                    else{
                        char txt[100];
                            //sprintf(txt,"\r\nstring sconosciuta FAIL VTG\r\n");
                            //HAL_UART_Transmit(&huart2,(uint8_t*)txt,strlen(txt),10);
                    }
                    strcpy(string_gps, "");
                    //cont_string = 0;

                    
                    
                }
                cont_string++;
            }
            else
            {
                if (data_string_gps == '$')
                {						  //check if data indicates the start of new string
                    start_string_gps = 1; //new string started
                    cont_string = 0;	  //set the counter to 1
                }
            }
        }
        msg_arrived = 0;
        //data_string_gps = buffer_gps[0];						  //convert a pointer into a char
    }
	return ret;
}
static int checksum(char *string_checksum, int size_string_checksum)
{ //check the checksum
	//return 1;

	int res = 0;
	int offset_maiusc = (int)('A') - (int)('a');
	int i = 0;
    string_checksum[0] = 'G';
    string_checksum[1] = 'P';

	for (i = 0; (i < size_string_checksum) && (string_checksum[i] != '*'); i++)
	{
		res = res ^ string_checksum[i];
	}
	char check[2] = {string_checksum[i + 1], string_checksum[i + 2]};
	char res_char[3] = {'0','0','\0'};

	sprintf(res_char, "%x", res);

	if (res < 16)
	{
		res_char[1] = res_char[0];
		res_char[0] = '0';
	}	
    
    for (int j = 0; j < 2; j++)
	{ //convert to upper case letter
		if ((int)res_char[j] >= 'a' && (int)res_char[j] <= 'f')
		{
			res_char[j] = (char)((int)res_char[j] + offset_maiusc);
		}
	}
	if (res_char[0] == check[0] && res_char[1] == check[1])
	{
		return 1; //checksum is correct
	}
	else
	{
		return 0; //checksum failed
	}
}