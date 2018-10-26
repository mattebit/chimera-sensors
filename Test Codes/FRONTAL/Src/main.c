
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "Eagle_TRT.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <inttypes.h>
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_it.h"
#include "string.h"
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilter;
HAL_CAN_StateTypeDef state;
uint32_t full;
int val0_100, val1_100;
uint32_t valMax0, valMin0, val0rang;
uint32_t ADC_buffer[3], val[3];
uint8_t CheckControl[4];

char val0[256];
char value_Error[256];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/*
 * Error number 1 occur when the board does not receive data from the GPS like if the TX (of the GPS) cable is not connected
 * Error number 2 occur when the received data cannot be read from the board
 */

//GPS-->> Board
//RX -->> D2 = PA10
//TX -->> D8 = PA9

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

// to generate your own sentences, check out the MTK command datasheet and use a checksum calculator
// such as the awesome http://www.hhhh.org/wiml/proj/nmeaxor.html

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
//////////////////sentence with speed in it are: GPRMA; GPRMC; GPVTG; GPVBW
//////////////////GPVBW Water referenced and ground referenced speed data

char requested_data[50];
char bufferRX[100];
char bufferRX_prec[100];
int len = 5;
char sentences[5][5] = {{"GPRMA"},{"GPRMC"},{"GPVTG"},{"GPVBW"},{"GPGGA"}};
int baud = 9600;
int fix = 0;//if the GPS is connected to satellites is 1 else is 0 if not connected else is -1 if I didn't found the data
int counter_A = 0;
int counter_B = 0;
int Error1 = 0;
int Error2 = 0;

//$GPRMC,225446,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68
//sentence example
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for (int i = 0; i < 1; i++)
	{
		val[i] = ADC_buffer[i];
	}
}

///IMU VARIABLES///
uint8_t ZERO = 0x00;
uint8_t WHO_AM_I_G = 0x8F;
uint8_t WHO_AM_I_G_VAL;
uint8_t WHO_AM_I_XM = 0x8F;
uint8_t WHO_AM_I_XM_VAL;

uint8_t CTRL_REG1_G_ADD = 0x20;
uint8_t CTRL_REG1_G_VAL = 0x0F;
uint8_t CTRL_REG4_G_ADD = 0x23;
uint8_t CTRL_REG4_G_VAL = 0x10;

uint8_t CTRL_REG1_XM_ADD = 0x20;
uint8_t CTRL_REG1_XM_VAL = 0xA7;
uint8_t CTRL_REG2_XM_ADD = 0x21;
uint8_t CTRL_REG2_XM_VAL = 0x08;
uint8_t CTRL_REG5_XM_ADD = 0x24;
uint8_t CTRL_REG5_XM_VAL = 0x70;
uint8_t CTRL_REG6_XM_ADD = 0x25;
uint8_t CTRL_REG6_XM_VAL = 0x20;
uint8_t CTRL_REG7_XM_ADD = 0x26;
uint8_t CTRL_REG7_XM_VAL = 0x00;

uint8_t OUT_X_L_G_ADD = 0xE8;
uint8_t OUT_X_H_G_ADD = 0xE9;
uint8_t OUT_Y_L_G_ADD = 0xEA;
uint8_t OUT_Y_H_G_ADD = 0xEB;
uint8_t OUT_Z_L_G_ADD = 0xEC;
uint8_t OUT_Z_H_G_ADD = 0xED;

uint8_t OUT_X_L_A_ADD = 0xE8;
uint8_t OUT_X_H_A_ADD = 0xE9;
uint8_t OUT_Y_L_A_ADD = 0xEA;
uint8_t OUT_Y_H_A_ADD = 0xEB;
uint8_t OUT_Z_L_A_ADD = 0xEC;
uint8_t OUT_Z_H_A_ADD = 0xED;

int Error;

char G_ok[256] = "Gyro is OK!!\r\n";
char A_ok[256] = "Axel is OK!!\r\n";
char value_Error[256];
char value_G[256];
char value_A[256];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  valMax0 = 4039;
  valMin0 = 2503;
  val0rang = abs(valMax0 - valMin0);

  ///GPS///

  GPS_INIT();

  GPS_Awake();

  double speed;
  double latitude;
  double longitude;
  char txt[20];

  char* char_speed;
  char* char_latitude;
  char* char_longitude;
  char* char_N;
  char* char_W;

  ///IMU///

  ///WRITING GYRO INITIALIZATION
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG1_G_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG1_G_VAL, 1, 10); ///Writing 0b00001111 to enable PowerMode and x,y,z axis
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG4_G_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG4_G_VAL, 1, 10); ///Writing 0b00010000 to set full-scale selection to 500dps
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0

  ///WRITING AXEL/MAGN INTIALIZATION
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG1_XM_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG1_XM_VAL, 1, 10); ///Writing 0b10100111 to enable 1600Hz and x,y,z axis
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG2_XM_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG2_XM_VAL, 1, 10); ///Writing 0b00001000 to set +/-4g range for axel axis
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG5_XM_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG5_XM_VAL, 1, 10); ///Writing 0b01110000 to set high resolution for magn and 50Hz
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG6_XM_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG6_XM_VAL, 1, 10); ///Writing 0b00100000 to set +/-4 gauss range for magn axis
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG7_XM_ADD, 1, 10); ///Writing the address
  HAL_SPI_Transmit(&hspi1, (uint8_t*)&CTRL_REG7_XM_VAL, 1, 10); ///Writing 0b00000000 to set continuos conversion for magn axis
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

  HAL_Delay(5000);

  ///GYRO CALIBRATION
  float kp_G = 0.0175;
  float X_G_axis_offset = LSM9DS0_calib(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_X_L_G_ADD, OUT_X_H_G_ADD, kp_G);
  float Y_G_axis_offset = LSM9DS0_calib(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Y_L_G_ADD, OUT_Y_H_G_ADD, kp_G);
  float Z_G_axis_offset = LSM9DS0_calib(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Z_L_G_ADD, OUT_Z_H_G_ADD, kp_G);

  ///AXEL CALIBRATION
  float kp_A = 0.00119782; ///0.000122 * 9,81
  float X_A_axis_offset = LSM9DS0_calib(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_X_L_A_ADD, OUT_X_H_A_ADD, kp_A);
  float Y_A_axis_offset = LSM9DS0_calib(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Y_L_A_ADD, OUT_Y_H_A_ADD, kp_A);
  float Z_A_axis_offset = LSM9DS0_calib(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Z_L_A_ADD, OUT_Z_H_A_ADD, kp_A);

  sFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilter.FilterIdLow = 0;
  sFilter.FilterIdHigh = 0;
  sFilter.FilterMaskIdHigh = 0;
  sFilter.FilterMaskIdLow = 0;
  sFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  sFilter.FilterBank = 0;
  sFilter.FilterScale  = CAN_FILTERSCALE_16BIT;
  sFilter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(&hcan1, &sFilter);

  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  Error = 0;
	  uint8_t CanSendMSG[8];
	  uint8_t RxData[8];

	  HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 1);
	  val0_100 = (int)100-(abs(val[0] - valMin0)*100/(val0rang)); //val0_100 -->STEER --> 0 = SX | 100 = DX
	  if (val[0] <= valMin0){
		  val0_100 = 100;
	  }
	  if (val[0] >= valMax0){
		  val0_100 = 0;
	  }

	  CanSendMSG[0] = 0x01;
	  CanSendMSG[1] = val0_100;
	  CanSendMSG[2] = 0;
	  CanSendMSG[3] = 0;
	  CanSendMSG[4] = 0;
	  CanSendMSG[5] = 0;
	  CanSendMSG[6] = 0;
	  CanSendMSG[7] = 0;
	  CAN_Send(0xC0, CanSendMSG, 8);

	  HAL_Delay(25);

	  ///Check for Errors
	  Error = LSMD9S0_check(); //IMU IS DISCONNECTED? 0 = 0K ; 1 = G_NOTOK ; 2 = A_NOTOK

	  ///Reading G_axis values
	  float X_G_axis = LSMD9S0_read(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_X_L_G_ADD, OUT_X_H_G_ADD, kp_G);
	  X_G_axis = X_G_axis - X_G_axis_offset;
	  float Y_G_axis = LSMD9S0_read(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Y_L_G_ADD, OUT_Y_H_G_ADD, kp_G);
	  Y_G_axis = Y_G_axis - Y_G_axis_offset;
	  float Z_G_axis = LSMD9S0_read(GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Z_L_G_ADD, OUT_Z_H_G_ADD, kp_G);
	  Z_G_axis = Z_G_axis - Z_G_axis_offset;

	  ///Reading A_axis values
	  float X_A_axis = LSMD9S0_read(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_X_L_A_ADD, OUT_X_H_A_ADD, kp_A);
	  X_A_axis = X_A_axis - X_A_axis_offset;
	  float Y_A_axis = LSMD9S0_read(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Y_L_A_ADD, OUT_Y_H_A_ADD, kp_A);
	  Y_A_axis = Y_A_axis - Y_A_axis_offset;
	  float Z_A_axis = LSMD9S0_read(GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Z_L_A_ADD, OUT_Z_H_A_ADD, kp_A);
	  Z_A_axis = Z_A_axis - Z_A_axis_offset + 9.81;

	  ///DEBUG SECTION
	  /*sprintf(value_Error, "Error: %d \r\n", Error);
	  HAL_UART_Transmit(&huart2, (uint8_t*)value_Error, strlen(value_Error), 10);

	  sprintf(value_G, "GYRO ---> X: %.2f  Y: %.2f  Z: %.2f \r\n", Y_G_axis, (0 - X_G_axis), Z_G_axis);
	  HAL_UART_Transmit(&huart2, (uint8_t*)value_G, strlen(value_G), 10);

	  sprintf(value_A, "ACCELEROMETER ---> X: %.2f Y: %.2f  Z: %.2f \r\n", (0 - Y_A_axis), (X_A_axis), Z_A_axis);
	  HAL_UART_Transmit(&huart2, (uint8_t*)value_A, strlen(value_A), 10);*/

	  /*sprintf(value_A, "apps ---> X: %d\r\n", val0_100);
	  HAL_UART_Transmit(&huart2, (uint8_t*)value_A, strlen(value_A), 10);

	  HAL_Delay(500);*/

	  ///CAN IMU --> ID2 / ERR / GYR0(X,Y,Z) --> ID2 / ERR / ACC(X,Y,Z)

	  int16_t val_G_X = Y_G_axis * 100;
	  int16_t val_G_Y = (0 - X_G_axis) * 100;
	  int16_t val_G_Z = Z_G_axis * 100;

	  CanSendMSG[0] = 0x04;
	  CanSendMSG[1] = val_G_X / 256;
	  CanSendMSG[2] = val_G_X % 256;
	  CanSendMSG[3] = val_G_Y / 256;
	  CanSendMSG[4] = val_G_Y % 256;
	  CanSendMSG[5] = val_G_Z / 256;
	  CanSendMSG[6] = val_G_Z % 256;
	  CanSendMSG[7] = Error;
	  CAN_Send(0xC0, CanSendMSG, 8);

	  HAL_Delay(25);

	  int16_t val_A_X = (0 - Y_A_axis) * 100;
	  int16_t val_A_Y = X_A_axis * 100;
	  int16_t val_A_Z = Z_A_axis * 100;

	  CanSendMSG[0] = 0x05;
	  CanSendMSG[1] = val_A_X / 256;
	  CanSendMSG[2] = val_A_X % 256;
	  CanSendMSG[3] = val_A_Y / 256;
	  CanSendMSG[4] = val_A_Y % 256;
	  CanSendMSG[5] = val_A_Z / 256;
	  CanSendMSG[6] = val_A_Z % 256;
	  CanSendMSG[7] = Error;
	  CAN_Send(0xC0, CanSendMSG, 8);

	  HAL_Delay(25);

	  ///GPS///
	  //delete all the characters in the buffer
	  for(int i = 0; i < 100; i++){
		  bufferRX[i] = '-';
	  }

	  //receive the data
	  HAL_UART_Receive(&huart1, (uint8_t*)bufferRX, strlen(bufferRX), 15);
	  //print(bufferRX);

	  if(Error1 == 0 || Error2 == 0){
		  //if the data has a known sentence, get the requested data in the right position
		  switch(Get_Sentence()){
			case 0:										//GPRMA
				break;

			case 1:										//GPRMC
				break;

			case 2:										//GPVTG
				char_speed = Get_Requested_Data(7);
				speed = atof(char_speed);
				speed = speed * 100; //2 decimals

				speed = (int)speed;

				break;

			case 3:										//GPVBW
				break;

			case 4:
				/*char_latitude = Get_Requested_Data(2);
				latitude = atof(char_latitude);
				latitude = latitude * 10000;

				char_N = Get_Requested_Data(3);

				char_longitude = Get_Requested_Data(4);
				longitude = atof(char_longitude);
				longitude = longitude * 10000;

				char_W = Get_Requested_Data(5);

				//sprintf(txt, "%d", (int)latitude);

				if(latitude != 0){
					//print(txt);
				}
				//print(txt);*/

				break;

			default:
			  break;
		  }

	  }

	  strcpy(bufferRX_prec, bufferRX);

	  //sprintf(txt, "%d", (int)speed);
	  int16_t speed_Send = speed;
	  //print(txt);

	  //GPS Velocity
	  CanSendMSG[0] = 0x03;
	  CanSendMSG[1] = speed_Send / 256;
	  CanSendMSG[2] = speed_Send % 256;
	  CanSendMSG[3] = 0;
	  CanSendMSG[4] = 0;
	  CanSendMSG[5] = 0;
	  CanSendMSG[6] = Error1;
	  CanSendMSG[7] = 0;

	  CAN_Send(0xD0, CanSendMSG, 8);
	  HAL_Delay(20);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 288;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* CAN1_TX_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	///CALIBRATION CODE///
	uint8_t RxData[8];
		  int idsave = CAN_Receive(RxData, 8);
		  uint8_t CanSendMSG[8];

		  if (idsave == 0xBB){
			//  sprintf(val0, "APPS1: %d \r\n", idsave);  //use "%lu" for long, "%d" for int
			  //		  HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), 10);
			  if ((RxData[0] == 2) && (RxData[1] == 0)){
				  valMin0 = val[0];
				  //CheckControl[0] = 1;
				  CanSendMSG[0] = 2;
				  CanSendMSG[1] = 0;
				  CanSendMSG[2] = 0;
				  CanSendMSG[3] = 0;
				  CanSendMSG[4] = 0;
				  CanSendMSG[5] = 0;
				  CanSendMSG[6] = 0;
				  CanSendMSG[7] = 0;
				  CAN_Send(0xBC, CanSendMSG, 8);
			  }
			  if ((RxData[0] == 2) && (RxData[1] == 1)){
				  valMax0 = val[0];
				  //CheckControl[1] = 1;
				  CanSendMSG[0] = 2;
				  CanSendMSG[1] = 1;
				  CanSendMSG[2] = 0;
				  CanSendMSG[3] = 0;
				  CanSendMSG[4] = 0;
				  CanSendMSG[5] = 0;
				  CanSendMSG[6] = 0;
				  CanSendMSG[7] = 0;
				  CAN_Send(0xBC, CanSendMSG, 8);

			  }
			  val0rang = abs(valMax0 - valMin0);
		 }
}
int LSMD9S0_check(void)
{
	int check = 0;

	///GYRO IS WORKING
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); ///CS_G to 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); ///CS_XM to 1
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&WHO_AM_I_G, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10); ///Reading from register sending a 0x00
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0

	///AXEL/MAGN ARE WORKING
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0
	HAL_SPI_Transmit(&hspi1, (uint8_t*)&WHO_AM_I_XM, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_XM (giving by &variable)
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_XM_VAL, 1, 10); ///Reading from register sending a 0x00
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); ///CS_G to 0
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); ///CS_XM to 1

	///AXEL/GYRO STATUS
	if (WHO_AM_I_G_VAL != 212){
		check = 1;
	}
	if (WHO_AM_I_XM_VAL != 212){
		check = 2;
	}
	if ((WHO_AM_I_G_VAL != 212) & (WHO_AM_I_XM_VAL != 212)){
				check = 3;
	}

	return check;
}

float LSMD9S0_read(GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp)
{
	uint8_t OUT_L_VAL;
	uint8_t OUT_H_VAL;

	///READING X_AXIS ROTATION
	HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
	HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
	HAL_SPI_Transmit(&hspi1, &REG_L, 1, 10); ///Writing LOW address
	HAL_SPI_Receive(&hspi1, (uint8_t*)&OUT_L_VAL, 1, 10); ///Saving LOW data
	HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
	HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0


	HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
	HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
	HAL_SPI_Transmit(&hspi1, &REG_H, 1, 10); ///Writing HIGH address
	HAL_SPI_Receive(&hspi1, (uint8_t*)&OUT_H_VAL, 1, 10); ///Saving HIGH data
	HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
	HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0

	///CALCULATING X_AXIS ROTATION
	float axis = OUT_H_VAL << 8 | OUT_L_VAL;	///Calculating axis value shifting and using a logic OR
	if (axis > 32767){ ///Generating positive and negative value of rotation
		axis = axis - 65536;
	}
	axis = axis * kp; ///Scaling axis value with appropriate conversion factor from datasheet

	return axis;
}

float LSM9DS0_calib(GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp)
{
	float axis_cal;
	float sum_cal = 0.0000;
	for(int i = 0; i < 10000; i++){
		  float tmp = LSMD9S0_read(GPIOx_InUse, GPIO_Pin_InUse, GPIOx_NotInUse, GPIO_Pin_NotInUse, REG_L, REG_H, kp);
		  sum_cal = sum_cal + tmp;
	}
	axis_cal = sum_cal / 10000;
	return axis_cal;
}

int CAN_Send(int id, uint8_t dataTx[], int size){

	uint32_t mailbox;
	uint8_t flag = 0;

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0 && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) == 0){
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, dataTx, &mailbox);
		flag = 1;
	}

	return flag;
}

int CAN_Receive(uint8_t *DataRx, int size){

	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0){
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, DataRx);
	}

	int id = RxHeader.StdId;

	return id;
}

char* Get_Requested_Data(int data_pos){


	int count = 0;
	int found = 0; //if data found 1 else 0
	int index = 0;

	//clear this buffer
	for(int i = 0; i < strlen(requested_data); i++){
		requested_data[i] = ' ';
	}

	for(int i = 0; i < strlen(bufferRX); i++){
		//count all the comma
		if(bufferRX[i] == ','){
			count++;
		}
		//the '-' is the last character in the input data
		if(bufferRX[i] == '-'){
			break;
		}

		if(found == 1){
			//until we find an another comma, add the data to the char
			if(bufferRX[i] == ','){
				return requested_data;
			}
			else{
				requested_data[index] = bufferRX[i];
				index ++;
			}
		}

		if(count == data_pos){
			found = 1;
		}

	}
	return "0";
}

//checks if the GPS is connected to some satellites
//if fix is 1 is connected if fix is 0 is not connected otherwise is -1 if I didn't found the data
int* Is_Valid(){
	char* letter = Get_Requested_Data(2);

	if(*letter == 'A'){
		fix = 1;
		return &fix;
	}
	else{
		if(*letter == 'V'){
			fix = 0;
			return &fix;
		}
		else{
			fix = -1;
			return &fix;
		}
	}
}

int Get_Sentence(){
	//buffer
	char sentence[5];
	int flag = 0;
	char * pointer;

	//check in the matrix where the strings are saved
	for(int i = 0; i < len; i++){
		for(int j = 0; j < 5; j++){
			sentence[j] = sentences[i][j];
		}

		if(strstr(bufferRX, sentence) != NULL){
			pointer = strstr(bufferRX, sentence);
			strcpy(bufferRX, pointer);

			flag = 1;
		}
		else{
			flag = 0;
			continue;
		}
		if(flag == 1){
			//return(i);

			if(i >= 0 && i <= len){
				return i;
			}
			else{
				return -1;
			}
/*
			switch(i){
			case 0:
				return 0;  MX_USART1_USART_Init();

				break;
			case 1:
				return 1;
				break;
			case 2:
				return 2;
				break;
			case 3:
				return 3;
				break;
			case 4:
				return 4;
				break;
			default:
				return -1;
				break;
			}*/
		}
		else{
			continue;
		}
	}
	return -1;
}

void print(char* text){
	HAL_UART_Transmit(&huart2, (uint8_t*)text, strlen(text), 5);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 1);

}

void GPS_INIT(){
	//default gps baud rate is 9600
		//set gps baud rate to 57600

		baud = 9600;
		MX_USART1_UART_Init();

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_BAUD_57600, strlen(PMTK_SET_BAUD_57600), 20);
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
		}

		//change the baud rate of the stm to 57600
		HAL_Delay(50);
		baud = 57600;
		MX_USART1_UART_Init();

		//send other commands to speed up the data flow
		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_API_SET_FIX_CTL_5HZ, strlen(PMTK_API_SET_FIX_CTL_5HZ), 20);
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
		}

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 20);
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
		}

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA), 20);
			HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
		}

		HAL_Delay(100);
}

void GPS_Awake(){
	for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_AWAKE, strlen(PMTK_AWAKE), 20);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
	}
	for(int i = 0; i < 50; i++){
		HAL_UART_Transmit(&huart1, (uint8_t*)PMTK_Q_RELEASE, strlen(PMTK_Q_RELEASE), 20);
		HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 4);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
