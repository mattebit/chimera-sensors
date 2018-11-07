
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "fatfs.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "inttypes.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
CAN_FilterTypeDef sFilter;
CAN_RxHeaderTypeDef RxHeader;

FIL loggingFile;
FIL log_names_f;

TCHAR message[256];

static TIM_HandleTypeDef a_TimerInstance6 = {.Instance = TIM6};
static TIM_HandleTypeDef a_TimerInstance7 = {.Instance = TIM7};

int secondsElapsed = 0;
int cont_rx=0;
char messagesToWrite[200][256];
int time=0;
int printable_time = 0;
int byteswritten;
int delta;
char filename[256] = "abcabc.txt";
char filename_1[256]="log_names.txt";
char txt[100];
int interrupt_flag = 0;

int mount_ok = 0;
int msg_counter = 0;
int msg_index = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void print(UART_HandleTypeDef *huart, char* text);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_USART3_UART_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
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

	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_Delay(500);

	char buffer[256]="Starting Antenna Logging\r\n";
	int bytes_read;

	char *pointer;
	char log_names[256];
	FRESULT res_open;


	print(&huart2, "---mounting---\r\n");
	FRESULT res_mount = f_mount(&SDFatFS, (TCHAR const*)SDPath, 0);

	if (res_mount == FR_OK) {
		sprintf(filename_1,"name.txt");
		res_open=f_open(&log_names_f, (TCHAR const*)&filename_1, FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
		f_read(&log_names_f, log_names, 256, (void*)&bytes_read);

		print(&huart2, "mounted, opened\r\n");

		char name[256];

		for(int i = 0; i < 20; i++){

		sprintf(name, "Log_%d", i);

		pointer = strstr(log_names,name);

		if(i == 0 && pointer == NULL){

			sprintf(filename, "Log_0\r\n");

			f_write(&log_names_f, filename, strlen(filename), (void*)&byteswritten);
			f_close(&log_names_f);

			sprintf(filename, "Log_0.txt");

			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
			f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
			f_close(&loggingFile);
			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

			print(&huart2, "created -> Log_0\r\n");

			break;
		}
		if(pointer == NULL){
			sprintf(filename, "Log_%d\r\n", i);

			f_write(&log_names_f, filename, strlen(filename), (void*)&byteswritten);
			f_close(&log_names_f);

			sprintf(filename, "Log_%d.txt", i);

			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
			f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
			f_close(&loggingFile);
			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

			print(&huart2, "created -> ");
			print(&huart2, filename);
			print(&huart2, "\r\n");

			break;
		}
		if(i==9){
			sprintf(filename,"default.txt");

			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );
			f_write(&loggingFile, buffer, strlen(buffer), (void*)&byteswritten);
			f_close(&loggingFile);
			f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_READ | FA_WRITE );

			print(&huart2, "created -> ");
			print(&huart2, filename);
			print(&huart2, "\r\n");
		}


		}

		mount_ok = 1;
		print(&huart2, "files closed\r\n");
	}else {
		mount_ok = 0;
	}

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
	HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);

	print(&huart2, "Can Init Done");

	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(interrupt_flag == 1){
		  HAL_CAN_DeactivateNotification(&hcan1, CAN1_RX0_IRQn);
		  HAL_CAN_DeactivateNotification(&hcan1, CAN1_RX1_IRQn);
		  interrupt_flag = 0;
		  print(&huart2, "<->\r\n");
		  //delta=__HAL_TIM_GET_COUNTER(&a_TimerInstance6); //10 microseconds needed tocexecute all the if
		  msg_counter = 0;
		  HAL_TIM_Base_Stop_IT(&htim6);
		  HAL_TIM_Base_Stop_IT(&htim7);
		  //6 microseconds needed to close and open
		  f_close(&loggingFile);
		  f_open(&loggingFile, (TCHAR const*)&filename, FA_OPEN_APPEND | FA_OPEN_ALWAYS | FA_WRITE );

		  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
		  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);
		  HAL_TIM_Base_Start_IT(&htim6);
		  HAL_TIM_Base_Start_IT(&htim7);
		  //delta=__HAL_TIM_GET_COUNTER(&a_TimerInstance6)-delta;
		  //sprintf(txt, "%d\r\n", delta);
          //print(&huart2, txt);
	  }
	  if(msg_index >= 0){
	  	  f_write(&loggingFile,messagesToWrite[msg_index],strlen(messagesToWrite[msg_index]),(void*)&byteswritten);
	  	  msg_index --;
	  }


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

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
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /* CAN1_RX0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* CAN1_RX1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* CAN1_SCE_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* TIM6_DAC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
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
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void)
{

  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 360;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
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

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
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
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void print(UART_HandleTypeDef *huart, char* text){
  HAL_UART_Transmit(huart, (uint8_t*)text, strlen(text), 5);
  //HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1);
}
int CAN_Receive(uint8_t *DataRx, int size){

	if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) != 0){
		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, DataRx);
	}

	int id = RxHeader.StdId;

	return id;
}
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	uint8_t RxData[8];
	int id;
	id=CAN_Receive(RxData, 8);
	cont_rx++;
	//int time = secondsElapsed * 1000 + __HAL_TIM_GET_COUNTER(&htim6) / 20; //20 ticks for each millisecond
	//printable_time=time*1000+ __HAL_TIM_GET_COUNTER(&htim6);
	delta=0;

	if(mount_ok == 1){
		msg_index ++;
		printable_time = time*1000+ __HAL_TIM_GET_COUNTER(&htim6);
		msg_counter ++;
		//sprintf(messagesToWrite[msg_index], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", time, msg_index, 000, 2, 3, 4, 5, 6, 7);
		sprintf(messagesToWrite[msg_index], "%d\t%d\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\r\n", time, id,RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	}

	/*
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	delta=__HAL_TIM_GET_COUNTER(&a_TimerInstance6);
	sprintf(messagesToWrite, "%d\t%d\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\r\n", time, id,RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
	delta=__HAL_TIM_GET_COUNTER(&a_TimerInstance6)-delta;
	f_write(&loggingFile,messagesToWrite,strlen(messagesToWrite),(void*)&byteswritten);*/
	/*if(changedState == 1)
		{
			for(int i= 0; i < 5; i++)
			{
				sprintf(messagesToWrite[secondIndexWrite], "-----\r\n");
				secondIndexWrite = (secondIndexWrite + 1) % DIM_ARRAY_MSG_TO_WRITE;
			}
			changedState = 0;
		}

		isInSetupOrRun = 1; // TODO: TOGLIERE QUESTA RIGA DI MERDA, SPADA TI AMMAZZO
		if(isInSetupOrRun == 1)
		{
			sprintf(messagesToWrite[secondIndexWrite], "%d\t%d\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\r\n", time, id, RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
			secondIndexWrite = (secondIndexWrite + 1) % DIM_ARRAY_MSG_TO_WRITE;
		}*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim7){
		interrupt_flag = 1;
	}
}

/*
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
  {
	if(firstBMSIndex != secondBMSIndex)
	{
		if (htim->Instance == TIM7 && isInBMSDataMode == 1) {
			HAL_UART_Transmit(&huart3, bmsMessages[firstBMSIndex], strlen(bmsMessages[firstBMSIndex]), 5);
			firstBMSIndex = (firstBMSIndex + 1) % 200;
		}
	}


	if(htim->Instance == TIM6)
	{
		if(isInBMSDataMode == 0)
		{
			char antennaMessage[256];
			sprintf(antennaMessage, "-\r\n%s%s%s%s%s", carStatus, antennaMessageHV, antennaMessageLV, antennaMessageINVSX, antennaMessageINVDX);
			HAL_UART_Transmit(&huart3, antennaMessage, strlen(antennaMessage), 5);
		}
		secondsElapsed++;
	}
  }*/

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
	  HAL_Delay(500);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	  HAL_Delay(2000);
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
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
