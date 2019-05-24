
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
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_it.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <inttypes.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int val0_100, val1_100, val2_100, Error, SCS, SCS1, SCS_Send, SCS_Send_real, Time1, Time2, fake_i, check;
int fake_min0 = 0, fake_max0 = 6000, fake_min1 = 6000, fake_max1 = 0;
uint32_t fake1[5];
uint32_t fake2[5];

uint32_t valMax0, valMin0, valMax1, valMin1, valMax2, valMin2, val0rang, val1rang, val2rang;
uint32_t ADC_buffer[3], val[3];

static TIM_HandleTypeDef s_TimerInstance = {.Instance = TIM3};

CAN_RxHeaderTypeDef RxHeader;
CAN_FilterTypeDef sFilter;
HAL_CAN_StateTypeDef state;
uint32_t full;

uint8_t CheckControl[4];

char val0[256];
char value_Error[256];
char txt[100];

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
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for (int i = 0; i < 3; i++)
	{
		val[i] = ADC_buffer[i];
	}
}

//CALIBRATION FUNCTION, ONLY SHOWS THE NEW MAX & MIN VALUES
//PRESS AND RELEASE COMPLEATELY THE ACCELERATOR
void print_Max_Min(){
	if(fake_min0 < val[0]){
		fake_min0 = val[0];
	}
	if(fake_max0 > val[0]){
		fake_max0 = val[0];
	}
	if(fake_min1 > val[1]){
		fake_min1 = val[1];
	}
	if(fake_max1 < val[1]){
		fake_max1 = val[1];
	}
	sprintf(txt, "valMIN0 = %d valMAX0 = %d \t valMIN1 = %d valMAX1 = %d val0_100 = %d val1_100 = %d \r\n", fake_min0, fake_max0, fake_min1, fake_max1, val0_100, val1_100);
	HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	valMin0 = 3417;
	valMax0 = 3294;	//released
	val0rang = abs(valMax0 - valMin0);
	valMin1 = 2353;
	valMax1 = 2465; //released
	val1rang = abs(valMax1 - valMin1);
	check = 0;
	fake_i = 0;

	val[1] = 0;

	Error = 0;

	CheckControl[0] = 0;
	CheckControl[1] = 0;
	CheckControl[2] = 0;
	CheckControl[3] = 0;

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
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim3);

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


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  uint8_t CanSendMSG[8];
	  uint8_t RxData[8];
	  SCS = 0;
	  SCS_Send = 0;
	  SCS_Send_real = 0;

	  ///CALCULATING APPS% GAIN///
	  HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 3);

	  int pc6;
	  pc6 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6);
	  if (pc6 != GPIO_PIN_SET && pc6 != GPIO_PIN_RESET){
		  SCS = 1;
	  }

	  val0_100 = 0;
	  val1_100 = 0;
	  fake1[fake_i] = (int)100-(abs(val[0] - valMin0)*100/(val0rang)); //val0_100 --> APPS1
	  fake2[fake_i] = (int)100-(abs(val[1] - valMin1)*100/(val1rang)); //val1_100 --> APPS2
	  for (int i = 0; i <= 4; i++){
		  val0_100 = val0_100 + fake1[i];
		  val1_100 = val1_100 + fake1[i];
	  }
	  val0_100 = val0_100 / 5;
	  val1_100 = val1_100 / 5;
	  val0_100 = 100 - val0_100;
	  val1_100 = 100 - val1_100;

	  if (fake_i >= 4){
		  fake_i = 0;
	  }
	  else{
		  fake_i = fake_i + 1;
	  }

	  ///NO POTENTIOMETER CONNECTED///
	  if(val[0] >= 4096){
		  SCS = 1;
	  }
	  if(val[1] >= 4096){
		  SCS = 1;
	  }
	  if(val[0] <= 0){
		  SCS = 1;
	  }
	  if(val[1] <= 0){
		  SCS = 1;
	  }
	  if(val[0] <= 100){
		  SCS = 1;
	  }
	  if(val[1] <= 100){
		  SCS = 1;
	  }

	  ///LED STRIP BRAKE LIGHT CODE///
	  if (pc6 == GPIO_PIN_SET){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
	  }
	  else if (pc6 == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
	  }

	  ///SAFETY RANGES CODE///
	  if (val0_100 >= 100){
		  val0_100 = 100;
	  }
	  if (val0_100 <= 5){
		  val0_100 = 0;
	  }

	  if (val1_100 >= 100){
		  val1_100 = 100;
	  }
	  if (val1_100 <= 5){
		  val1_100 = 0;
	  }

	  ///ERRORS CONTROL CODE///
	  if(abs(val0_100 - val1_100) >= 10){
		  if(__HAL_TIM_GET_COUNTER(&s_TimerInstance) - Time1 > 100){
			  SCS1 = 1;
		  }
	  }
	  else{
		  Time1 = __HAL_TIM_GET_COUNTER(&s_TimerInstance);
		  SCS1 = 0;
	  }


	  if (SCS != 0 || SCS1 != 0){
		  val0_100 = 0;
		  val1_100 = 0;
		  val2_100 = GPIO_PIN_RESET;
		  SCS_Send = 1;
	  }

	  if (check != 1){
		  CanSendMSG[0] = 0x01;
		  CanSendMSG[1] = val0_100;
		  CanSendMSG[2] = val1_100;
		  CanSendMSG[3] = 0;
		  CanSendMSG[4] = 0;
		  CanSendMSG[5] = 0;
		  CanSendMSG[6] = SCS_Send;
		  CanSendMSG[7] = 0;
		  CAN_Send(0xB0, CanSendMSG, 8);
	  }
	  HAL_Delay(20);

	  CanSendMSG[0] = 0x02;
	  CanSendMSG[1] = pc6;
	  CanSendMSG[2] = 0;
	  CanSendMSG[3] = 0;
	  CanSendMSG[4] = 0;
	  CanSendMSG[5] = 0;
	  CanSendMSG[6] = SCS_Send;
	  CanSendMSG[7] = 0;
	  CAN_Send(0xB0, CanSendMSG, 8);

	  HAL_Delay(20);

	  print_Max_Min();
	  //}

	  ///DEBUG CODE when in release to change with sending in CANbus///
	  //if (SCS_Send != 0){
/*
	  sprintf(val0, "percent APPS1: %d percent Apps2: %d apps1 %d apps2 %d\r\n", val0_100, val1_100, val[0], val[1]);  //use "%lu" for long, "%d" for int
	  HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), 10);


	  sprintf(value_Error, "SCS: %d  BRK: %d\r\n", SCS_Send, val2_100);
	  HAL_UART_Transmit(&huart2, (uint8_t*)value_Error, strlen(value_Error), 10);
	  //}
	  HAL_Delay(100);*/
	  /*char msg[256];
	  	  for(int i = 0; i < 4; i++){
	  		  sprintf(msg, "msg: %d /r/n", CheckControl[i]);
	  		  HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), 10);
	  	  }*/

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
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
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

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 42666;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
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

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan){
	///CALIBRATION CODE///
		  int idsave = CAN_Receive(RxData, 8);
		  uint8_t CanSendMSG[8];

		  if (idsave == 0xBB){
			//  sprintf(val0, "APPS1: %d \r\n", idsave);  //use "%lu" for long, "%d" for int
			  //		  HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), 10);
			  if ((RxData[0] == 0) && (RxData[1] == 0)){
				  valMin0 = val[0];
				  valMin1 = val[1];
				  //CheckControl[0] = 1;
				  CanSendMSG[0] = 0;
				  CanSendMSG[1] = 0;
				  CanSendMSG[2] = 0;
				  CanSendMSG[3] = 0;
				  CanSendMSG[4] = 0;
				  CanSendMSG[5] = 0;
				  CanSendMSG[6] = 0;
				  CanSendMSG[7] = 0;
				  CAN_Send(0xBC, CanSendMSG, 8);
				  check = 1;
			  }
			  if ((RxData[0] == 0) && (RxData[1] == 1)){
				  valMax0 = val[0];
				  valMax1 = val[1];
				  //CheckControl[1] = 1;
				  CanSendMSG[0] = 0;
				  CanSendMSG[1] = 1;
				  CanSendMSG[2] = 0;
				  CanSendMSG[3] = 0;
				  CanSendMSG[4] = 0;
				  CanSendMSG[5] = 0;
				  CanSendMSG[6] = 0;
				  CanSendMSG[7] = 0;
				  CAN_Send(0xBC, CanSendMSG, 8);
				  check = 0;
			  }
			  val0rang = abs(valMax0 - valMin0);
			  val1rang = abs(valMax1 - valMin1);
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