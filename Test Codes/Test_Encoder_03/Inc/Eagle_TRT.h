#ifndef _EAGLE_TRT_H
#define _EAGLE_TRT_H

#include "inttypes.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//----------------GPS----------------//
char* Get_Requested_Data(char * bufferRx, int data_pos, char * requested_data);
int* Is_Valid(char * bufferRx, int  * fix, char * requested_data);
int Get_Sentence(char * bufferRx, char (*sentences)[5], int len);
void GPS_INIT(UART_HandleTypeDef *huart);
void GPS_Awake();

//----------------ENCODER----------------//
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"
	double read_encoder(TIM_HandleTypeDef *TimerInstance);
	void encoder_tim_interrupt(TIM_HandleTypeDef *htim, int * interrupt_flag, double * angles_array, double * speed);
	int implausibility_check(TIM_HandleTypeDef *TimerInstance, int * Val0_100, int * Val1_100);
#endif
int bin_dec(int* bin, int size);
double Power(int base, int expn);
double get_speed_encoder(float angle0, float angle1, int refresh, float wheel_diameter);
void shift_array(double *array, int size, double data);
double dynamic_average(double *array, int size);

//----------------IMU----------------//
#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"
	void gyro_calib(SPI_HandleTypeDef *hspi, float * X_G_axis_offset, float * Y_G_axis_offset, float * Z_G_axis_offset);
	void accel_calib(SPI_HandleTypeDef *hspi, float * X_A_axis_offset, float * Y_A_axis_offset, float * Z_A_axis_offset);
	int LSMD9S0_check(SPI_HandleTypeDef *hspi);
	float LSMD9S0_read(SPI_HandleTypeDef *hspi,GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp);
	float LSM9DS0_calib(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp);
	void gyro_read(SPI_HandleTypeDef *hspi,float * X_G_axis, float * Y_G_axis, float * Z_G_axis, float X_G_axis_offset, float Y_G_axis_offset, float Z_G_axis_offset);
	void accel_read(SPI_HandleTypeDef *hspi,float * X_A_axis, float * Y_A_axis, float * Z_A_axis,float X_A_axis_offset, float Y_A_axis_offset, float Z_A_axis_offset);
#endif
void gyro_init();
void magn_accel_init();

//----------------CAN----------------//
#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
  int CAN_Send(CAN_HandleTypeDef *hcan,int id, uint8_t dataTx[], int size);
  int CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t *DataRx, int size);
#endif

//----------------MISCELLANEOUS----------------//
#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
  void print(UART_HandleTypeDef *huart, char* text);
#endif
void calc_pot_value(int max, int min, int range, float * val0_100, int * val);
void set_min(int * val, int * min1, int * max1, int * min2, int * max2);
void set_max(int * val, int * min1, int * max1, int * min2, int * max2);



//GPS CONSTANTS
#define PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ  "$PMTK220,10000*2F\r\n" // Once every 10 seconds, 100 millihertz.
#define PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ  "$PMTK220,5000*1B\r\n"  // Once every 5 seconds, 200 millihertz.
#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r\n"
#define PMTK_SET_NMEA_UPDATE_2HZ  "$PMTK220,500*2B\r\n"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r\n"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r\n"

#define  PMTK_CMD_STANDBY_MODE "$PMTK161,0*28\r\n"	//enter in sleep mode

#define PMTK_API_SET_FIX_CTL_100_MILLIHERTZ  "$PMTK300,10000,0,0,0,0*2C" // Once every 10 seconds, 100 millihertz.
#define PMTK_API_SET_FIX_CTL_200_MILLIHERTZ  "$PMTK300,5000,0,0,0,0*18"  // Once every 5 seconds, 200 millihertz.
#define PMTK_API_SET_FIX_CTL_1HZ  "$PMTK300,1000,0,0,0,0*1C"
#define PMTK_API_SET_FIX_CTL_5HZ  "$PMTK300,200,0,0,0,0*2F"
// Can't fix position faster than 5 times a second!

#define PMTK_SET_BAUD_57600 "$PMTK251,57600*2C"
#define PMTK_SET_BAUD_9600 "$PMTK251,9600*17"
#define PMKT_SER_BAUD_DEFAULT "$PMTK251,0*28"//It works only if: a.full cold start command issue
										   //				     b.enter standby mode

// turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_RMCONLY "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn on ALL THE DATA
#define PMTK_SET_NMEA_OUTPUT_ALLDATA "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
// turn off output
#define PMTK_SET_NMEA_OUTPUT_OFF "$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"

#define PMTK_LOCUS_STARTLOG  "$PMTK185,0*22"
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
#define PMTK_STANDBY_SUCCESS "$PMTK001,161,3*36"  // Not needed currently
#define PMTK_AWAKE "$PMTK010,002*2D"

// ask for the release and version
#define PMTK_Q_RELEASE "$PMTK605*31"

// request for updates on antenna status
#define PGCMD_ANTENNA "$PGCMD,33,1*6C"
#define PGCMD_NOANTENNA "$PGCMD,33,0*6D"

// how long to wait when we're looking for a response
#define MAXWAITSENTENCE 10

#endif
