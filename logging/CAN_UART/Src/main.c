/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_FilterTypeDef sFilter;
CAN_RxHeaderTypeDef RxHeader;

int a=0;

int cont_rx=0;
char messagesToWrite[200][256];
int time=0;
long int printable_time = 0;
int delta;

int mount_ok = 0;
int msg_counter = 0;
int msg_index = 0;

static TIM_HandleTypeDef a_TimerInstance6 = {.Instance = TIM6};
static TIM_HandleTypeDef a_TimerInstance7 = {.Instance = TIM7};

int interrupt_flag = 0;

uint8_t huart_rx[50];
char msg_can_to_send[50];
int cont_msg_can_to_send;
int flag_rx=0;
int cont_huart_rx;
int cont_length_num;

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
static void MX_ADC1_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();

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
	HAL_UART_Transmit(&huart2, (uint8_t*)"can init done_1\r\n", 17, 5);
	HAL_UART_Transmit(&huart3, (uint8_t*)"start_1\r\n", 9, 5);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	HAL_Delay(500);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
	HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);

	HAL_UART_Transmit(&huart2, (uint8_t*)"can init done\r\n", 15, 5);
	HAL_UART_Transmit(&huart3, (uint8_t*)"start\r\n", 7, 5);

	HAL_TIM_Base_Start(&htim6);
	HAL_TIM_Base_Start(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);


	HAL_UART_Receive_IT(&huart2,huart_rx, 35);

  char txt[200];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    a++;
	  HAL_Delay(1);
	  //print(&huart2,"ciao\r\n");
	  //HAL_Delay(10);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)"ciao\r\n", 7, 5);

	  //sprintf(txt, "192 5 10 10 10 10 10 10 10\r\n");
	  //HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 5);


	  if(flag_rx == 1){
		  flag_rx = 0;
		  HAL_UART_Transmit(&huart2, (uint8_t*)msg_can_to_send, strlen(msg_can_to_send), 5);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 5);
		  uint32_t mailbox;
		  uint8_t messaggio_can[8];
		  char can_id[] = {msg_can_to_send[0], msg_can_to_send[1], msg_can_to_send[2],'\0'};
		  int zero = (int)('0');
		  messaggio_can[0] = ((int)(msg_can_to_send[4])-zero)*100+((int)(msg_can_to_send[5])-zero)*10+((int)(msg_can_to_send[6])-zero);
		  messaggio_can[1] = ((int)(msg_can_to_send[8])-zero)*100+((int)(msg_can_to_send[9])-zero)*10+((int)(msg_can_to_send[10])-zero);
		  messaggio_can[2] = ((int)(msg_can_to_send[12])-zero)*100+((int)(msg_can_to_send[13])-zero)*10+((int)(msg_can_to_send[14])-zero);
		  messaggio_can[3] = ((int)(msg_can_to_send[16])-zero)*100+((int)(msg_can_to_send[17])-zero)*10+((int)(msg_can_to_send[18])-zero);
		  messaggio_can[4] = ((int)(msg_can_to_send[20])-zero)*100+((int)(msg_can_to_send[21])-zero)*10+((int)(msg_can_to_send[22])-zero);
		  messaggio_can[5] = ((int)(msg_can_to_send[24])-zero)*100+((int)(msg_can_to_send[25])-zero)*10+((int)(msg_can_to_send[26])-zero);
		  messaggio_can[6] = ((int)(msg_can_to_send[26])-zero)*100+((int)(msg_can_to_send[29])-zero)*10+((int)(msg_can_to_send[30])-zero);
		  messaggio_can[7] = ((int)(msg_can_to_send[30])-zero)*100+((int)(msg_can_to_send[31])-zero)*10+((int)(msg_can_to_send[34])-zero);

		  CAN_TxHeaderTypeDef TxHeader;
		  TxHeader.StdId = atoi(can_id);
		  TxHeader.IDE = CAN_ID_STD;
		  TxHeader.RTR = CAN_RTR_DATA;
		  TxHeader.DLC = 8;
		  TxHeader.TransmitGlobalTime = DISABLE;

		  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0 && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) == 0){
			  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, messaggio_can, &mailbox);
		  }
		  //HAL_UART_Receive_IT(&huart2,huart_rx, 36);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
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
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SDIO|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
  PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  if (HAL_SD_Init(&hsd) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 36;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 3600;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 2250000;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

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

int CAN_Send(char *msg_can){

		uint32_t mailbox;
		uint8_t flag = 0;

		char msg_id[3];
		msg_id[0] = msg_can[0];
		msg_id[1] = msg_can[1];
		msg_id[2] = '\0';

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = (uint32_t)atoi(msg_id);
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = 8;
		TxHeader.TransmitGlobalTime = DISABLE;

		uint8_t data_to_send[8];
		int j=0;
		for(int i = 0; i < 50 || msg_can[i+2] == '\0'; i++){
			if(msg_can[i+2] == '\t' || msg_can[i+2] == ' '){// founded separation
				if(msg_can[i+4] == '\t' || msg_can[i+4] == ' '){ //one space number
					data_to_send[j] = msg_can[i+3]-(int)('0');
					i = i+1;
				}else if(msg_can[i+5] == '\t' || msg_can[i+5] == ' '){ //two space number
					char num[3];
					num[0] = msg_can[i+3];
					num[1] = msg_can[i+4];
					num[2] = '0';
					data_to_send[j] = atoi(num);
					i = i+2;
				}else if(msg_can[i+6] == '\t' || msg_can[i+6] == ' '){ //three space number
					char num[4];
					num[0] = msg_can[i+3];
					num[1] = msg_can[i+4];
					num[2] = msg_can[i+5];
					num[3] = '0';
					data_to_send[j] = atoi(num);
					i = i+3;
				}
			}
		}

		if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 0 && HAL_CAN_IsTxMessagePending(&hcan1, CAN_TX_MAILBOX0) == 0){
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data_to_send, &mailbox);
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
void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
	uint8_t RxData[8];
	int id;
	id=CAN_Receive(RxData, 8);
	cont_rx++;
	//int time = secondsElapsed * 1000 + __HAL_TIM_GET_COUNTER(&htim6) / 20; //20 ticks for each millisecond
	//printable_time=time*1000+ __HAL_TIM_GET_COUNTER(&htim6);
	delta=0;
	mount_ok=1;
	if(mount_ok == 1){
		msg_index ++;
		printable_time = time*1000+ __HAL_TIM_GET_COUNTER(&htim6);
		msg_counter ++;
		//sprintf(messagesToWrite[msg_index], "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n", time, msg_index, 000, 2, 3, 4, 5, 6, 7);
		sprintf(messagesToWrite[msg_index], "%ld\t%d\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u\r\n", printable_time, id,RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		//print(&huart2,messagesToWrite[msg_index]);
		HAL_UART_Transmit(&huart2, (uint8_t*)messagesToWrite[msg_index],(uint16_t)strlen(messagesToWrite[msg_index]),100);
		/*char printable_time_s[100];
		sprintf(printable_time_s,"%d\r\n",printable_time);
		print(&huart2,printable_time_s);*/
		msg_counter=0;
		msg_index=0;
	}
}
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	print_it(huart);
}*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

	flag_rx=1;
	for( int i = 0; i < 35; i++){
		msg_can_to_send[i] = huart_rx[i];
	}
	msg_can_to_send[35] = '\0';
	HAL_UART_Transmit(&huart3, (uint8_t*)msg_can_to_send, strlen(msg_can_to_send), 5);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 5);
	HAL_UART_Receive_IT(&huart2,huart_rx, 35);

	/*if(huart_rx[0]=='-'){
		HAL_UART_Transmit(&huart3, (uint8_t*)msg_can_to_send, strlen(msg_can_to_send), 5);
		HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 5);
		flag_rx=1;
		msg_can_to_send[cont_huart_rx] = '\0';
		cont_huart_rx = 0;
		HAL_UART_Transmit(&huart3, (uint8_t*)msg_can_to_send, strlen(msg_can_to_send), 5);
		HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 5);
	}else{
		HAL_UART_Transmit(&huart3, (uint8_t*)huart_rx, strlen(huart_rx), 5);
		//msg_can_to_send[cont_huart_rx] = (char)huart_rx[0];
		//cont_huart_rx++;
		HAL_UART_Receive_IT(&huart2,huart_rx, 10);
	}*/

	/*if(huart_rx[0] == ' ' || huart_rx[0] == '\t'){
		if(cont_length_num == 2){
			msg_can_to_send[cont_huart_rx] = msg_can_to_send[cont_huat_rx-1];
			msg_can_to_send[cont_huart_rx-1]='0';
			cont_huart_rx+=1;
		}else if(cont_length_num == 3){
			msg_can_to_send[cont_huart_rx] = msg_can_to_send[cont_huart_rx-2];
			msg_can_to_send[cont_huart_rx-1]='0';
			msg_can_to_send[cont_huart_rx-2]='0';
			cont_huart_rx+=2;
		}
		cont_length_num = 0;
	}else{
		//if(cont_huart_rx == 26)
		msg_can_to_send[cont_huart_rx] = (char)huart_rx[0];
		cont_huart_rx++;
		cont_length_num++;
	}
	if(cont_huart_rx >= 27){
		HAL_UART_Transmit(&huart3, (uint8_t*)msg_can_to_send, strlen(msg_can_to_send), 5);
		HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 5);
		cont_length_num = 0;
		cont_huart_rx = 0;
		flag_rx = 1;
	}*/
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim7){
		interrupt_flag = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
