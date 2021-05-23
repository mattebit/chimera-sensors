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
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define USE_STEER 1
#define USE_ENCODER 1

// ms beween two messages of same device
#define CAN_SEND_FREQUENCY 20

// Diameter in meters
#define WHEEL_DIAMETER 0.395

#define DEBUG 1
#define CSV 0
#define FULL_DEBUG 1
#define DEBUG_DELAY 200

extern can_stc can;
extern pot_stc pot_1;
extern pot_stc pot_2;
extern pot_stc pot_3;

struct Encoder_Data enc_data_right;
struct Encoder_Settings enc_setting_right;
struct Encoder_Data enc_data_left;
struct Encoder_Settings enc_setting_left;

CAN_FilterTypeDef sFilter;
uint32_t valMax0, valMin0, val0rang;
uint32_t ADC_buffer[4], val[3];
char txt[100];
int flag = 0;
int multiplier = 1;
int timer_factor = 2;
int calibration_flag = 0;
int inverter_rpm = 0;

int left_sample = 0;
int right_sample = 0;
int left_sample_printable = 0;
int right_sample_printable = 0;

int steer_sample = 0;
int steer_sample_printable = 0;

int adc_conv_compleated = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  steer_sample++;
  pot_1.val = ADC_buffer[0];
  pot_2.val = ADC_buffer[1];
  pot_3.val = ADC_buffer[2];

  adc_conv_compleated = 1;
}

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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();

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
  sFilter.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilter.FilterActivation = ENABLE;

  HAL_CAN_Start(&hcan1);

  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
  HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  // ------------      CAN      ------------ //
  can.hcan = &hcan1;

  // imu initialization //
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);  ///CS_G to 1
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);  ///CS_XM to 1

  // ------------    ENCODER    ------------ //
  init_encoder_settings(&enc_setting_right, &enc_setting_left);
  init_encoder_data(&enc_setting_right, &enc_data_right, &enc_setting_left, &enc_data_left);

  // ------------ POTENTIOMETER ------------ //
  pot_2.max = 4055;
  pot_2.min = 2516;
  pot_2.range = fabs(pot_2.max - pot_2.min);

  // ------------     TIMERS    ------------ //

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim7);
  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_Base_Start(&htim11);

  HAL_Delay(1);

  HAL_TIM_Base_Start_IT(&htim7);

  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim7, 0);
  __HAL_TIM_SET_COUNTER(&htim10, 0);
  __HAL_TIM_SET_COUNTER(&htim11, 0);

  HAL_Delay(1);

  // Enabling auto retransmit of speed calculated from inverters
  can.dataTx[0] = 0x3D;
  can.dataTx[1] = 0xA8;
  can.dataTx[2] = 0x64;
  can.id = 0x201;
  can.size = 3;
  CAN_Send(&can);

  HAL_Delay(1);

  can.dataTx[0] = 0x3D;
  can.dataTx[1] = 0xA8;
  can.dataTx[2] = 0x64;
  can.id = 0x202;
  can.size = 3;
  CAN_Send(&can);
  /*
  // Enabling auto retransmit of torque calculated from inverters
  can.dataTx[0] = 0x3D;
  can.dataTx[1] = 0xA0;
  can.dataTx[2] = 0x64;
  can.id = 0x201;
  can.size = 3;
  CAN_Send(&can);

  HAL_Delay(10);

  can.dataTx[0] = 0x3D;
  can.dataTx[1] = 0xA0;
  can.dataTx[2] = 0x64;
  can.id = 0x202;
  can.size = 3;
  CAN_Send(&can);
  */

  uint32_t tick = HAL_GetTick();
  uint32_t previous_millis = tick;

  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    tick = HAL_GetTick();

    if(adc_conv_compleated && USE_STEER){
      calc_pot_value(&pot_2, 200);
    }

    if(CSV && enc_data_right.new_data == 1){
      sprintf(txt, "%d.%d;%d.%d;%d.%d;%d.%d;%d;%d\r\n",
              (int)(enc_data_right.speed),
              decimals(enc_data_right.speed),
              (int)(enc_data_right.average_speed),
              decimals(enc_data_right.average_speed),
              (int)(enc_data_right.angle_prec),
              decimals(enc_data_right.angle_prec),
              (int)(enc_data_right.angle),
              decimals(enc_data_right.angle),
              (int)(enc_data_right.delta_angle*1000),
              (int)enc_data_right.wheel_rotation);
      HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
      enc_data_right.new_data = 0;
    }

    if (previous_millis != tick) {
      previous_millis = tick;

      send_CAN_data(tick);

      // sprintf(txt, "%d\r\n", (int)(enc_data_right.actual_frequency));
      // HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
      if (tick % 999 == 0) {
        left_sample_printable = left_sample;
        right_sample_printable = right_sample;
        left_sample = 0;
        right_sample = 0;

        steer_sample_printable = steer_sample;
        steer_sample = 0;


        if(DEBUG && FULL_DEBUG){
          full_debug();
        }
      }

      if (DEBUG && tick % DEBUG_DELAY == 0) {
        if(!CSV && !FULL_DEBUG){
          sprintf(txt, "ENC_L: %d.%d\r\nENC_R: %d.%d\r\nSteer(*1000): %d\r\n",
                  (int)(enc_data_left.average_speed),
                  decimals(enc_data_left.average_speed),
                  (int)(enc_data_right.average_speed),
                  decimals(enc_data_right.average_speed),
                  (int)(pot_2.val_100 * 1000));
          HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
        }
      }
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
  /* TIM7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM7_IRQn);
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
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
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
  htim3.Init.Prescaler = 18;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65500;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  // APB1 timer
  // not sure but I think APB1 is at 72 MHz

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 72;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 4000;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 72;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 50000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 72;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 50000;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    //print_it(&huart2);
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan) {
  /// CALIBRATION CODE///
  int id = CAN_Receive(&can);

  if (id == 0xD2){
    int rotations = 0;
    rotations  = can.dataRx[3];
    rotations += can.dataRx[2] << 8;
    rotations += can.dataRx[1] << 16;

    enc_data_left .wheel_rotation = rotations;
    enc_data_right.wheel_rotation = rotations;
    enc_data_left .Km = rotations * enc_setting_left .wheel_diameter * (M_PI);
    enc_data_right.Km = rotations * enc_setting_right.wheel_diameter * (M_PI);
  }

  if (id == 0xBB) {
    if ((can.dataRx[0] == 2) && (can.dataRx[1] == 0)) {
      set_min(&pot_2);
      calibration_flag = 1;

      can.dataTx[0] = 2;
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
    }
    if ((can.dataRx[0] == 2) && (can.dataRx[1] == 1)) {
      set_max(&pot_2);
      calibration_flag = 0;

      can.dataTx[0] = 2;
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
    }
    pot_2.range = fabs(pot_2.max - pot_2.min);
    float max_tmp = pot_2.max;
    float min_tmp = pot_2.min;
    if (max_tmp > min_tmp) {
      pot_2.max = max_tmp;
      pot_2.min = min_tmp;
    }
    if (max_tmp < min_tmp) {
      pot_2.max = min_tmp;
      pot_2.min = max_tmp;
    }
  }

  // TIMER Interrupt setup via CAN Message
  if (id == 195 && can.dataRx[0] == 1) {
    multiplier = can.dataRx[1] * 256 + can.dataRx[2];
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  if (htim == &htim7) {
    if (USE_ENCODER) {
      left_sample++;
      right_sample++;
      encoder_tim_interrupt(&enc_setting_right, &enc_data_right);
      encoder_tim_interrupt(&enc_setting_left, &enc_data_left);

      // Start conversion of 3 ADC
      HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 3);
    }
  }
}

void send_CAN_data(uint32_t millis) {
  int increment_value = 1;

  if (USE_ENCODER) {
    //-------------------SEND Encoder-------------------//
    if (millis % CAN_SEND_FREQUENCY == 0) {
      double right_buffer = enc_data_right.average_speed;
      if (right_buffer < 0)
        right_buffer *= -1;

      uint16_t speed_kmh  = (uint16_t)(right_buffer * ((enc_setting_right.wheel_diameter / 2) * 3.6));
      uint16_t speed_rads = (uint16_t)(right_buffer * 100);

      can.dataTx[0] = 0x06;
      can.dataTx[1] = (uint8_t)(speed_kmh >> 8);
      can.dataTx[2] = (uint8_t)(speed_kmh);
      can.dataTx[3] = (uint8_t)(enc_data_right.speed_sign);
      can.dataTx[4] = (uint8_t)(speed_rads >> 8);
      can.dataTx[5] = (uint8_t)(speed_rads);
      can.dataTx[6] = (uint8_t)(enc_data_right.error_flag);
      can.dataTx[7] = (uint8_t)(enc_setting_right.steer_enc_prescaler);
      can.id = 0xD0;
      can.size = 8;
      CAN_Send(&can);
    }

    millis += increment_value;

    if (millis % CAN_SEND_FREQUENCY == 0) {
      double left_buffer  = enc_data_left.average_speed;
      double right_buffer = enc_data_right.average_speed;

      if (left_buffer < 0)
        left_buffer *= -1;
      if (right_buffer < 0)
        right_buffer *= -1;

      uint32_t left_rads  = left_buffer * 10000;
      uint32_t right_rads = right_buffer * 10000;

      can.dataTx[0] = 0x07;
      can.dataTx[1] = (uint8_t)(left_rads >> 16);
      can.dataTx[2] = (uint8_t)(left_rads >> 8);
      can.dataTx[3] = (uint8_t)(left_rads);
      can.dataTx[4] = (uint8_t)(right_rads >> 16);
      can.dataTx[5] = (uint8_t)(right_rads >> 8);
      can.dataTx[6] = (uint8_t)(right_rads);
      can.dataTx[7] = enc_data_right.speed_sign;
      can.id = 0xD0;
      can.size = 8;
      CAN_Send(&can);
    }

    millis += increment_value;

    if (millis % CAN_SEND_FREQUENCY == 0) {
      uint32_t rotations  = enc_data_right.wheel_rotation;
      uint32_t km         = enc_data_right.Km;

      can.dataTx[0] = (uint8_t)(rotations >> 16);
      can.dataTx[1] = (uint8_t)(rotations >> 8);
      can.dataTx[2] = (uint8_t)(rotations);
      can.dataTx[3] = (uint8_t)(km >> 16);
      can.dataTx[4] = (uint8_t)(km >> 8);
      can.dataTx[5] = (uint8_t)(km);
      can.dataTx[6] = (uint8_t)(enc_data_right.error_flag);
      can.dataTx[7] = (uint8_t)(enc_data_left.error_flag);
      can.id = 0xD1;
      can.size = 8;
      CAN_Send(&can);
    }

    millis += increment_value;

    //--------------------ENCODER DEBUG DATA--------------------//
    if (millis % CAN_SEND_FREQUENCY == 0) {
      uint16_t la = (uint16_t)(enc_data_left.angle * 100);
      uint16_t ra = (uint16_t)(enc_data_right.angle * 100);
      uint16_t da = (uint16_t)(enc_data_right.delta_angle * 100);

      can.dataTx[0] = 0x015;
      can.dataTx[1] = (uint8_t)(la >> 8);
      can.dataTx[2] = (uint8_t)(la);
      can.dataTx[3] = (uint8_t)(ra >> 8);
      can.dataTx[4] = (uint8_t)(ra);
      can.dataTx[5] = (uint8_t)(da >> 8);
      can.dataTx[6] = (uint8_t)(da);
      can.dataTx[7] = (uint8_t)(enc_setting_right.steer_enc_prescaler);
      can.id = 0xD0;
      can.size = 8;
      CAN_Send(&can);
    }

    millis += increment_value;
  }

  if (USE_STEER) {
    //--------------------SEND Steer--------------------//
    if (millis % CAN_SEND_FREQUENCY == 0) {
      if (calibration_flag == 0) {
        uint16_t angle = pot_2.val_100 * 100;

        can.dataTx[0] = 2;
        can.dataTx[1] = (uint8_t)(angle >> 8);
        can.dataTx[2] = (uint8_t)(angle);
        can.dataTx[3] = 0;
        can.dataTx[4] = 0;
        can.dataTx[5] = 0;
        can.dataTx[6] = 0;
        can.dataTx[7] = 0;
        can.id = 0xC0;
        can.size = 8;
        CAN_Send(&can);
      }
    }

    millis += increment_value;
  }
}

void init_encoder_settings(struct Encoder_Settings *right, struct Encoder_Settings *left) {
  //---------------- RIGHT ----------------//
  right->steer_enc_prescaler = CAN_SEND_FREQUENCY / 2;

  right->DataPinName          = GPIOC;
  right->ClockPinName         = GPIOC;
  right->DataPinNumber        = GPIO_PIN_8;
  right->ClockPinNumber       = GPIO_PIN_6;

  right->dx_wheel             = 0;
  right->interrupt_flag       = 0;
  right->clock_timer          = &htim3;
  right->wheel_diameter       = WHEEL_DIAMETER;
  right->clock_period         = 4;
  right->data_size            = 15;
  right->speed_size           = 10;

  right->max_delta_angle      = M_PI/2;
  right->frequency_timer      = &htim7;
  right->frequency_timer_Hz   = 72000000;
  right->frequency            = right->frequency_timer_Hz / (htim7.Init.Prescaler * htim7.Init.Period);

  right->microsecond_timer    = &htim10;

  right->conversion = (((double)encoder_Power(2, right->data_size))/(2*M_PI));

  //---------------- LEFT ----------------//
  left->steer_enc_prescaler   = CAN_SEND_FREQUENCY / 2;

  left->DataPinName           = GPIOB;
  left->ClockPinName          = GPIOB;
  left->DataPinNumber         = GPIO_PIN_9;
  left->ClockPinNumber        = GPIO_PIN_8;

  left->dx_wheel              = 0;
  left->interrupt_flag        = 0;
  left->clock_timer           = &htim3;
  left->wheel_diameter        = WHEEL_DIAMETER;
  left->clock_period          = 2;
  left->data_size             = 15;
  left->speed_size            = 10;

  left->max_delta_angle       = right->max_delta_angle;
  left->frequency_timer       = &htim7;
  left->frequency_timer_Hz    = 72000000;
  left->frequency             = right->frequency_timer_Hz / (htim7.Init.Prescaler * htim7.Init.Period);

  left->microsecond_timer    = &htim11;

  left->conversion = (((double)encoder_Power(2, left->data_size))/(2*M_PI));

  // CLOCK PINS TO HIGH
  HAL_GPIO_WritePin(right->ClockPinName, right->ClockPinNumber, GPIO_PIN_SET);
  HAL_GPIO_WritePin(left->ClockPinName,  left->ClockPinNumber,  GPIO_PIN_SET);
}

void init_encoder_data(struct Encoder_Settings *right, struct Encoder_Data *right_d,
                       struct Encoder_Settings *left, struct Encoder_Data *left_d) {
  // RIGHT
  right_d->Km = 0;
  right_d->average_speed = 0;
  right_d->wheel_rotation = 0;

  right_d->binary_data = malloc(sizeof(int) * right->data_size);
  memset(right_d->binary_data, 0, sizeof(int) * right->data_size);
  right_d->speed_array = malloc(sizeof(double) * right->speed_size);
  memset(right_d->speed_array, 0, sizeof(double) * right->speed_size);

  // LEFT
  left_d->Km = 0;
  left_d->average_speed = 0;
  left_d->wheel_rotation = 0;

  left_d->binary_data = malloc(sizeof(int) * left->data_size);
  memset(left_d->binary_data, 0, sizeof(int) * left->data_size);
  left_d->speed_array = malloc(sizeof(double) * left->speed_size);
  memset(left_d->speed_array, 0, sizeof(double) * left->speed_size);
}


int decimals(double number){
  return abs(1000*(floor(number) - number));
}

void full_debug(){
  // ALL DATA
  char* str = "\n\n________________________________________________________________________\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
  str = "DATA\t\t\t| LEFT\t\t\t| RIGHT\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);

  // Encoder Raw Data
  sprintf(txt, "Raw:\t\t\t| %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d  %d",
          enc_data_left.binary_data[0],
          enc_data_left.binary_data[1],
          enc_data_left.binary_data[2],
          enc_data_left.binary_data[3],
          enc_data_left.binary_data[4],
          enc_data_left.binary_data[5],
          enc_data_left.binary_data[6],
          enc_data_left.binary_data[7],
          enc_data_left.binary_data[8],
          enc_data_left.binary_data[9],
          enc_data_left.binary_data[10],
          enc_data_left.binary_data[11],
          enc_data_left.binary_data[12],
          enc_data_left.binary_data[13],
          enc_data_left.binary_data[14],
          enc_data_left.error_flag);
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);

  sprintf(txt, "\t| %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d  %d\r\n\n",
          enc_data_right.binary_data[0],
          enc_data_right.binary_data[1],
          enc_data_right.binary_data[2],
          enc_data_right.binary_data[3],
          enc_data_right.binary_data[4],
          enc_data_right.binary_data[5],
          enc_data_right.binary_data[6],
          enc_data_right.binary_data[7],
          enc_data_right.binary_data[8],
          enc_data_right.binary_data[9],
          enc_data_right.binary_data[10],
          enc_data_right.binary_data[11],
          enc_data_right.binary_data[12],
          enc_data_right.binary_data[13],
          enc_data_right.binary_data[14],
          enc_data_right.error_flag);
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  // Encoder calcuated speeds
  sprintf(txt, "Speed [rads*1000]:\t| %d\t\t\t| %d\r\n",
          (int)(enc_data_left.average_speed * 1000),
          (int)(enc_data_right.average_speed * 1000));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Angle [rad*1000]:\t| %d\t\t\t| %d\r\n",
          (int)(enc_data_left.angle * 1000),
          (int)(enc_data_right.angle * 1000));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Km:\t\t\t| %d\t\t\t| %d\r\n",
          (int)(enc_data_left.Km * 1000),
          (int)(enc_data_right.Km * 1000));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Rotations:\t\t| %d\t\t\t| %d\r\n\n",
          (int)(enc_data_left.wheel_rotation),
          (int)(enc_data_right.wheel_rotation));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  // Encoder Frequency
  sprintf(txt, "Callbacks:\t\t| %d\t\t\t| %d\r\n",
          (int)(left_sample_printable),
          (int)(right_sample_printable));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Frequency:\t\t| %d\t\t\t| %d\r\n",
          (int)(enc_setting_left.frequency),
          (int)(enc_setting_right.frequency));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  str = "________________________________________________________________________\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);

  // Steer        
  str = "DATA\t\t\t| STEER\t\t\t| \r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);

  sprintf(txt, "Raw:\t\t\t| %d\r\n",
          (pot_2.val));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Converted:\t\t| %d\r\n",
          (int)(pot_2.val_100 * 1000));
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);
  sprintf(txt, "Callbacks:\t\t| %d\r\n",
          steer_sample_printable);
  HAL_UART_Transmit(&huart2, (uint8_t *)txt, strlen(txt), 10);

  str = "________________________________________________________________________\r\n\n\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
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
  while (1) {
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
