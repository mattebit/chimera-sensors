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
#include "stm32f4xx_hal_gpio.h"
#include "Eagle_TRT.h"
#include "stm32f4xx_it.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
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
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern pot_stc pot_1;
extern pot_stc pot_2;
extern pot_stc pot_3;
extern can_stc can;

int val0_100, val1_100, val2_100, Error, SCS, SCS1, SCS_Send, SCS_Send_real, Time1, Time2, fake_i, check, flag;
int fake_min0 = 0, fake_max0 = 6000, fake_min1 = 60000, fake_max1 = 0;
uint32_t fake1[5];
uint32_t fake2[5];

uint32_t valMax0, valMin0, valMax1, valMin1, valMax2, valMin2, val0rang, val1rang, val2rang;
uint32_t ADC_buffer[3], val[3];

static TIM_HandleTypeDef a_TimerInstance2 = {.Instance = TIM2};
static TIM_HandleTypeDef a_TimerInstance3 = {.Instance = TIM3};
static TIM_HandleTypeDef a_TimerInstance4 = {.Instance = TIM4};
static TIM_HandleTypeDef a_TimerInstance7 = {.Instance = TIM7};

CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilter;
HAL_CAN_StateTypeDef state;
uint32_t full;

uint8_t CheckControl[4];

char val0[256];
char value_Error[256];
char txt[500];

int multiplier = 1;

int pc6 = 0;

int timer_flag = 0;
int command_flag = 0;

int steer_wheel_prescaler;
int previous_millis;

int stampa = 0;

uint8_t RxData[8];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM7_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	pot_1.val = ADC_buffer[0];
	pot_2.val = ADC_buffer[1];
	pot_3.val = ADC_buffer[2];
	stampa ++;
}
void print_Max_Min(){
	if(fake_min0 <= pot_1.val){
		fake_min0 = pot_1.val;
	}
	if(fake_max0 >= pot_1.val){
		fake_max0 = pot_1.val;
	}
	if(fake_min1 >= pot_2.val){
		fake_min1 = pot_2.val;
	}
	if(fake_max1 <= pot_2.val){
		fake_max1 = pot_2.val;
	}
	sprintf(txt, "valMIN0 = %d valMAX0 = %d \t valMIN1 = %d valMAX1 = %d val0_100 = %d val1_100 = %d \r\n", fake_min0, fake_max0, fake_min1, fake_max1, pot_1.val, pot_2.val);
	HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	pot_1.min = 4001;
	pot_1.max = 3872;	//released
	pot_1.range = abs(pot_1.max - pot_1.min);
	pot_2.min = 2614;
	pot_2.max = 2730; //released
	pot_2.range = abs(pot_2.max - pot_2.min);
	check = 0;
	fake_i = 0;

	val[1] = 0;

	Error = 0;

	CheckControl[0] = 0;
	CheckControl[1] = 0;
	CheckControl[2] = 0;
	CheckControl[3] = 0;

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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
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

  HAL_CAN_Start(&hcan1);

  uint8_t TxData[8];

  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);

  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start(&htim3);
  //HAL_TIM_Base_Start(&htim4);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_Base_Start_IT(&htim4);

  __HAL_TIM_SET_COUNTER(&a_TimerInstance2, 0);
  __HAL_TIM_SET_COUNTER(&a_TimerInstance3, 0);
  //__HAL_TIM_SET_COUNTER(&a_TimerInstance4, 500);

  pot_1.TimerInstance = &htim3;
  can.hcan = &hcan1;

  steer_wheel_prescaler = htim2.Init.Period;
  steer_wheel_prescaler /= 8;
  steer_wheel_prescaler /= 20;
  steer_wheel_prescaler += 2;

  HAL_UART_Transmit(&huart2, (uint8_t*)"vivo\r\n", strlen("vivo\r\n"), 10);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  uint8_t CanSendMSG[8];
	  uint8_t RxData[8];
	  //SCS = 0;
	  SCS_Send = 0;
	  SCS_Send_real = 0;

	  ///CALCULATING APPS% GAIN///
	  HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 3);

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_SET){
		  pc6 = 1;
	  }
	  else{
		  pc6 = 0;
	  }

	  // If CAN is free from important messages, send data
	  if(command_flag == 0){
		if (previous_millis != HAL_GetTick()){
			send_CAN_data(HAL_GetTick());
			previous_millis = HAL_GetTick();
		}
	  }
	  else{
		HAL_Delay(1);
		command_flag = 0;
	  }

	  //print_Max_Min();
/*
	  if(command_flag == 1){
		  HAL_TIM_Base_Stop_IT(&htim2);
		  HAL_Delay(500);
		  HAL_TIM_Base_Start_IT(&htim2);
		  __HAL_TIM_SET_COUNTER(&a_TimerInstance2, 0);

		  command_flag = 0;
	  }*/



	  ///LED STRIP BRAKE LIGHT CODE///
	  if (pc6 == GPIO_PIN_SET){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	  }
	  else if (pc6 == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	  }

	  calc_pot_value(&pot_1);
	  calc_pot_value(&pot_2);

	  /*if(pot_1.val < 300 || pot_2.val < 300){
		  pot_1.val = 0;
		  pot_2.val = 0;
		  //pot_1.val_100 = 0;
		  //pot_2.val_100 = 0;
		  SCS1 = 1;
		  SCS_Send = 1;
		  SCS = 1;
	  }else{
		  SCS = 0;
		  SCS1 = 0;
		  SCS_Send = 0;
	  }*/

	  //print_Max_Min();

	  /*if(implausibility_check(&pot_1, &pot_2) == 1){
		  pot_1.val_100 = 0;
		  pot_2.val_100 = 0;
		  SCS_Send = 1;
	  }*/

	  /*if (SCS != 0 || SCS1 != 0){
		  pot_1.val_100 = 0;
		  pot_2.val_100 = 0;
		  val2_100 = GPIO_PIN_RESET;
		  SCS_Send = 1;
	  }*/

	  /*sprintf(txt, "val0: %d , val1: %d , val0_100: %d, val1_100: %d, SCS: %d, SCS1: %d, SCS_send: %d \r\n",
			  pot_1.val, pot_2.val, pot_1.val_100, 100 - pot_2.val_100, SCS, SCS1, SCS_Send);
	  HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);*/


	  //sprintf(txt,"adc1: %d, adc2: %d, adc3: %d\r\n",(int)ADC_buffer[0],(int)ADC_buffer[1],(int)ADC_buffer[2]);
	  //HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);
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
  RCC_OscInitStruct.PLL.PLLQ = 2;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  /* ADC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC_IRQn);
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
  /* TIM3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* TIM4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
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
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
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
  hcan1.Init.AutoWakeUp = ENABLE;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3600;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42666;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3600;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim7.Init.Prescaler = 18;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000;
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
  huart2.Init.BaudRate = 115200;
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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	///CALIBRATION CODE///
		  int idsave = CAN_Receive(&can);
		  uint8_t CanSendMSG[8];
/*
		  if(idsave == 0x55 || idsave == 0x201){
			  if(can.dataRx[0] == 0x51 || can.dataRx[0] == 0x03 || can.dataRx[0] == 0x04 || can.dataRx[0] == 0x05 || can.dataRx[0] == 0x08 || can.dataRx[0] == 0x0A || can.dataRx[0] == 0x0B){
				  command_flag = 1;
				  idsave = 0;
			  }
		  }
		  if(idsave == 0xA0 || idsave == 0xAA || idsave == 0x181){
			  if(can.dataRx[0] == 0x03 || can.dataRx[0] == 0x04 || can.dataRx[0] == 0x05 || can.dataRx[0] == 0x08 || can.dataRx[0] == 0xD8){
				  command_flag = 1;
				  idsave = 0;
			  }
		  }*/

		  if (idsave == 0xBB){
			//  sprintf(val0, "APPS1: %d \r\n", idsave);  //use "%lu" for long, "%d" for int
			  //		  HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), 10);
			  if ((can.dataRx[0] == 0) && (can.dataRx[1] == 0)){
				  set_min(&pot_1);
				  set_min(&pot_2);
				  //CheckControl[0] = 1;
				  can.dataTx[0] = 0;
				  can.dataTx[1] = 0;
				  can.dataTx[2] = 0;
				  can.dataTx[3] = 0;
				  can.dataTx[4] = 0;
				  can.dataTx[5] = 0;
				  can.dataTx[6] = 0;
				  can.dataTx[7] = 0;
				  can.id = 0xBC;
				  can.size = 8;
				  CAN_Send(&can);
				  check = 1;
			  }
			  if ((can.dataRx[0] == 0) && (can.dataRx[1] == 1)){
				  set_max(&pot_1);
				  set_max(&pot_2);
				  //CheckControl[1] = 1;
				  can.dataTx[0] = 0;
				  can.dataTx[1] = 1;
				  can.dataTx[2] = 0;
				  can.dataTx[3] = 0;
				  can.dataTx[4] = 0;
				  can.dataTx[5] = 0;
				  can.dataTx[6] = 0;
				  can.dataTx[7] = 0;
				  can.id = 0xBC;
				  can.size = 8;
				  CAN_Send(&can);
				  check = 0;
			  }
			  //val0rang = abs(valMax0 - valMin0);
			  //val1rang = abs(valMax1 - valMin1);
			  pot_1.range = abs(pot_1.max - pot_1.min);
			  pot_2.range = abs(pot_2.max - pot_2.min);
		  }
		  if(idsave==195 && can.dataRx[0]==0){
			  multiplier = can.dataRx[1] * 256 + can.dataRx[2];
		  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim2){/*
		switch(timer_flag){
		case 0:
			can.dataTx[0] = 0x02;
			can.dataTx[1] = pc6;
			can.dataTx[2] = 0;
			can.dataTx[3] = steer_wheel_prescaler;
			can.dataTx[4] = 0;
			can.dataTx[5] = 0;
			can.dataTx[6] = SCS_Send;
			can.dataTx[7] = 0;
			can.id = 0xB0;
			can.size = 8;
			CAN_Send(&can);
			timer_flag ++;
			//SCS = 0;
			//SCS_Send = 0;
			break;
		case 1:
			timer_flag ++;
			break;
		case 2:
			timer_flag ++;
			break;
		case 3:
			if (check != 1){
			  can.dataTx[0] = 0x01;
			  can.dataTx[1] = pot_2.val_100;
			  can.dataTx[2] = pot_1.val_100;
			  can.dataTx[3] = steer_wheel_prescaler;
			  can.dataTx[4] = 0;
			  can.dataTx[5] = 0;
			  can.dataTx[6] = SCS_Send;
			  can.dataTx[7] = 0;
			  can.id = 0xB0;
			  can.size = 8;
			  CAN_Send(&can);
			  timer_flag ++;
			  //SCS = 0;
			  //SCS_Send = 0;
			}
			break;
		case 4:
			timer_flag ++;
			break;
		case 5:
			timer_flag ++;
			break;
		case 6:
			timer_flag ++;
			break;
		case 7:
			timer_flag = 0;
			break;
		}*/
		if(timer_flag == 1* multiplier){
				can.dataTx[0] = 0x02;
				can.dataTx[1] = pc6;
				can.dataTx[2] = 0;
				can.dataTx[3] = steer_wheel_prescaler;
				can.dataTx[4] = 0;
				can.dataTx[5] = 0;
				can.dataTx[6] = SCS_Send;
				can.dataTx[7] = 0;
				can.id = 0xB0;
				can.size = 8;
				CAN_Send(&can);
				//SCS = 0;
				//SCS_Send = 0;
		}
		if(timer_flag == 2* multiplier){
		}
		if(timer_flag == 3* multiplier){
		}
		if(timer_flag == 4* multiplier){
			if (check != 1){
			  can.dataTx[0] = 0x01;
			  can.dataTx[1] = pot_2.val_100;
			  can.dataTx[2] = pot_1.val_100;
			  can.dataTx[3] = steer_wheel_prescaler;
			  can.dataTx[4] = 0;
			  can.dataTx[5] = 0;
			  can.dataTx[6] = SCS_Send;
			  can.dataTx[7] = 0;
			  can.id = 0xB0;
			  can.size = 8;
			  CAN_Send(&can);
			  //SCS = 0;
			  //SCS_Send = 0;
			}
		}
		if(timer_flag == 5* multiplier){
		}
		if(timer_flag == 6* multiplier){
		}
		if(timer_flag == 7* multiplier){
		}
		if(timer_flag == 8* multiplier){
		}

		if(timer_flag > 8*multiplier){
			timer_flag = 0;
		}
		else{
			timer_flag ++;
		}

	}
}

int send_CAN_data(uint32_t millis){

    int sent_flag = 0;

    if(millis % 25 == 0){

    	can.dataTx[0] = 0x02;
		can.dataTx[1] = pc6;
		can.dataTx[2] = 0;
		can.dataTx[3] = steer_wheel_prescaler;
		can.dataTx[4] = 0;
		can.dataTx[5] = 0;
		can.dataTx[6] = SCS_Send;
		can.dataTx[7] = 0;
		can.id = 0xB0;
		can.size = 8;
		CAN_Send(&can);

    }
    millis += 1;

    if(millis % 25 == 0){

		if (check != 1){
		  can.dataTx[0] = 0x01;
		  can.dataTx[1] = pot_2.val_100;
		  can.dataTx[2] = pot_1.val_100;
		  can.dataTx[3] = steer_wheel_prescaler;
		  can.dataTx[4] = 0;
		  can.dataTx[5] = 0;
		  can.dataTx[6] = SCS_Send;
		  can.dataTx[7] = 0;
		  can.id = 0xB0;
		  can.size = 8;
		  CAN_Send(&can);
		  //SCS = 0;
		  //SCS_Send = 0;
		}

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
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
