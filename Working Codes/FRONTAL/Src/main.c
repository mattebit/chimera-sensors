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
#include "Eagle_TRT.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include <inttypes.h>
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_it.h"
#include <math.h>
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern can_stc can;
extern imu_stc accel;
extern imu_stc gyro;
extern enc_stc enc;
extern pot_stc pot_1;
extern pot_stc pot_2;
extern pot_stc pot_3;

CAN_FilterTypeDef sFilter;
uint32_t valMax0, valMin0, val0rang;
uint32_t ADC_buffer[4], val[3];
char txt[100];
int flag = 0;
int multiplier = 1;
int timer_factor = 2;
int command_flag = 0;
int calibration_flag = 0;

TIM_HandleTypeDef a_TimerInstance2 = {.Instance = TIM2};
TIM_HandleTypeDef a_TimerInstance3 = {.Instance = TIM3};
TIM_HandleTypeDef a_TimerInstance4 = {.Instance = TIM4};
TIM_HandleTypeDef a_TimerInstance5 = {.Instance = TIM5};
TIM_HandleTypeDef a_TimerInstance6 = {.Instance = TIM6};
TIM_HandleTypeDef a_TimerInstance7 = {.Instance = TIM7};
TIM_HandleTypeDef a_TimerInstance10 = {.Instance = TIM10};

gps_struct gps_main;
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
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

int send_CAN_data(uint32_t);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    pot_1.val = ADC_buffer[0];
    pot_2.val = ADC_buffer[1];
    pot_3.val = ADC_buffer[2];
    /*int txt_1[100];
    sprintf(txt_1, "sterzo: %d %d %d\r\n", pot_1.val, pot_2.val, pot_3.val);
    HAL_UART_Transmit(&huart2, (uint8_t*)txt_1, strlen(txt_1), 10);*/
}

int steer_enc_prescaler;
int encoder_counter;
int previous_millis;

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
    MX_SPI1_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM6_Init();
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
    HAL_CAN_ConfigFilter(&hcan1, &sFilter);

    HAL_CAN_Start(&hcan1);

    HAL_CAN_ActivateNotification(&hcan1, CAN1_RX0_IRQn);
    HAL_CAN_ActivateNotification(&hcan1, CAN1_RX1_IRQn);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    can.hcan = &hcan1;
    // can initialization //

    // imu initialization //
    accel.GPIOx_InUse = GPIOC;
    accel.GPIO_Pin_InUse = GPIO_PIN_9;
    accel.hspi = &hspi1;

    gyro.GPIOx_InUse = GPIOA;
    gyro.GPIO_Pin_InUse = GPIO_PIN_8;
    gyro.hspi = &hspi1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

    if (gps_init(&huart1, &gps_main) == 0)
    {
        //--error--//
    }

    steer_enc_prescaler = htim3.Init.Period;
    steer_enc_prescaler /= 3;
    steer_enc_prescaler /= 20;
    steer_enc_prescaler += 40;
    enc.steer_enc_prescaler = steer_enc_prescaler;

    pot_2.max = 4060;
    pot_2.min = 2350;
    pot_2.range = abs(pot_2.max - pot_2.min);

    enc.dx_wheel = 1;
    enc.interrupt_flag = 0;
    enc.TimerInstance = &a_TimerInstance3;
    enc.average_speed = 0;
    enc.wheel_diameter = 0.395;
    enc.samle_delta_time = htim7.Init.Period;
    enc.data_size = 15;
    enc.clock_period = 2;
    enc.wheel_rotation = 0;
    enc.Km = 0;

    enc.max_delta_angle = 40;
    enc.frequency = 0;
    enc.frequency_timer = &htim7;
    enc.frequency_timer_Hz = 72000000;

    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim3);
    //HAL_TIM_Base_Start(&htim4);
    //HAL_TIM_Base_Start(&htim5);
    //HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start(&htim7);
    HAL_TIM_Base_Start(&htim10);

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim3);
    //HAL_TIM_Base_Start_IT(&htim4);
    //HAL_TIM_Base_Start_IT(&htim5);
    //HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);
    HAL_TIM_Base_Start_IT(&htim10);

    __HAL_TIM_SET_COUNTER(&a_TimerInstance2, 0);
    __HAL_TIM_SET_COUNTER(&a_TimerInstance3, 0);
    //__HAL_TIM_SET_COUNTER(&a_TimerInstance4, 0);
    //__HAL_TIM_SET_COUNTER(&a_TimerInstance5, 0);
    //__HAL_TIM_SET_COUNTER(&a_TimerInstance6, 0);
    __HAL_TIM_SET_COUNTER(&a_TimerInstance7, 0);
    __HAL_TIM_SET_COUNTER(&a_TimerInstance10, 0);

    // INIT Encoder
    enc_calculate_optimal_frequency(&enc);

    accel.scale = 4;
    gyro.scale = 500;

    //HAL_Delay(1000);
    LSMD9S0_accel_gyro_init(&accel, &gyro);
    //LSMD9S0_check(&imu);

    LSM9DS0_calibration(&accel);
    LSM9DS0_calibration(&gyro);

    encoder_counter = 0;

    while (1)
    {

        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        HAL_ADC_Start_DMA(&hadc1, ADC_buffer, 3);

        // If CAN is free from important messages, send data
        if (command_flag == 0)
        {
            if (previous_millis != HAL_GetTick())
            {
                send_CAN_data(HAL_GetTick());
                previous_millis = HAL_GetTick();
            }
        }
        else
        {
            HAL_Delay(1);
            command_flag = 0;
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
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
    /* TIM6_DAC_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    /* TIM5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    /* TIM2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    /* TIM1_UP_TIM10_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    /* USART1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
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
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
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
    TIM_SlaveConfigTypeDef sSlaveConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 36;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000;
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
    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
    sSlaveConfig.InputTrigger = TIM_TS_ITR0;
    if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
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
    htim4.Init.Period = 2000;
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 36;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 2000;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
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
    htim6.Init.Prescaler = 3600;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 2000;
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
    htim7.Init.Prescaler = 36;
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = 6000;
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
    htim10.Init.Prescaler = 36;
    htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim10.Init.Period = 500;
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
    htim11.Init.Prescaler = 0;
    htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim11.Init.Period = 0;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
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
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        gps_read_it(huart, &gps_main);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart2)
    {
        print_it(&huart2);
    }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    /// CALIBRATION CODE///
    int idsave = CAN_Receive(&can);
    //201/202

    if (idsave == 0x55 || idsave == 0x201)
    {
        if (can.dataRx[0] == 0x51 || can.dataRx[0] == 0x03 || can.dataRx[0] == 0x04 || can.dataRx[0] == 0x05 || can.dataRx[0] == 0x08 || can.dataRx[0] == 0x0A || can.dataRx[0] == 0x0B)
        {
            command_flag = 1;
            idsave = 0;
        }
    }
    if (idsave == 0xA0 || idsave == 0xAA || idsave == 0x181)
    {
        if (can.dataRx[0] == 0x03 || can.dataRx[0] == 0x04 || can.dataRx[0] == 0x05 || can.dataRx[0] == 0x08 || can.dataRx[0] == 0xD8)
        {
            command_flag = 1;
            idsave = 0;
        }
    }

    if (idsave == 0xBB)
    {
        //sprintf(val0, "APPS1: %d \r\n", idsave);  //use "%lu" for long, "%d" for int
        //HAL_UART_Transmit(&huart2, (uint8_t*)val0, strlen(val0), 10);
        if ((can.dataRx[0] == 2) && (can.dataRx[1] == 0))
        {
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
            for (int i = 0; i < 10; i++)
            {
                CAN_Send(&can);
            }
        }
        if ((can.dataRx[0] == 2) && (can.dataRx[1] == 1))
        {
            set_max(&pot_2);
            calibration_flag = 0;

            can.dataTx[0] = 2;
            can.dataTx[1] = 1;
            can.dataTx[2] = 0;
            can.dataTx[3] = 0;
            can.dataTx[4] = 0;
            can.dataTx[5] = 0;
            ;
            can.dataTx[6] = 0;
            can.dataTx[7] = 0;
            can.id = 0xBC;
            can.size = 8;
            for (int i = 0; i < 10; i++)
            {
                CAN_Send(&can);
            }
        }
        //val0rang = abs(valMax0 - valMin0);
        pot_2.range = abs(pot_2.max - pot_2.min);
        int max_tmp = pot_2.max;
        int min_tmp = pot_2.min;
        if (max_tmp > min_tmp)
        {
            pot_2.max = max_tmp;
            pot_2.min = min_tmp;
        }
        if (max_tmp < min_tmp)
        {
            pot_2.max = min_tmp;
            pot_2.min = max_tmp;
        }
    }

    //TIMER Interrupt setup via CAN Message
    if (idsave == 195 && can.dataRx[0] == 1)
    {
        multiplier = can.dataRx[1] * 256 + can.dataRx[2];
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    if (htim == &htim10)
    {
        //READING SENSORS
        if (flag == 1 * multiplier)
        {
            // ACCEL
            LSMD9S0_accel_read(&accel);
            imu_elaborate_data(&accel);
        }
        else if (flag == 2 * multiplier)
        {
            // STEER
            calc_pot_value(&pot_2);
        }
        else if (flag == 3 * multiplier)
        {
            // GYRO
            LSMD9S0_gyro_read(&gyro);
            imu_elaborate_data(&gyro);
        }

        if (flag >= (3 * multiplier))
        {
            flag = 0;
        }
        else
        {
            flag++;
        }
    }

    if (htim == &htim7)
    {
        encoder_tim_interrupt(&enc);
    }
}

int send_CAN_data(uint32_t millis)
{

    int sent_flag = 0;

    //-------------------SEND Encoder-------------------//
    if (millis % 100 == 0)
    {
        uint16_t speed_Send = enc.average_speed;

        can.dataTx[0] = 0x06;
        can.dataTx[1] = speed_Send / 256;
        can.dataTx[2] = speed_Send % 256;
        can.dataTx[3] = enc.speed_sign;
        can.dataTx[4] = 0;
        can.dataTx[5] = 0;
        can.dataTx[6] = enc.error_flag;
        can.dataTx[7] = enc.steer_enc_prescaler;
        can.id = 0xD0;
        can.size = 8;
        CAN_Send(&can);

        sent_flag = 1;
    }

    millis += 5;

    //-------------SEND KM & WHEEL ROTAIONS-------------//
    if (millis % 500 == 0)
    {

        uint16_t Km = (enc.Km);
        uint16_t rotations = enc.wheel_rotation;

        can.dataTx[0] = 0x08;
        can.dataTx[1] = Km >> 8;
        can.dataTx[2] = Km;
        can.dataTx[3] = (uint8_t)rotations >> 8;
        can.dataTx[4] = (uint8_t)rotations;
        can.dataTx[5] = (enc.angle0 / 10);
        can.dataTx[6] = enc.clock_period;
        can.dataTx[7] = 0;
        can.id = 0xD0;
        can.size = 8;
        CAN_Send(&can);

        sent_flag = 2;
    }

    millis += 5;

    //--------------------SEND Accel--------------------//
    if (millis % 100 == 0)
    {

        //removing negative values
        uint16_t val_a_x = accel.x + accel.scale * 1000;
        uint16_t val_a_y = accel.y + accel.scale * 1000;
        uint16_t val_a_z = accel.z + accel.scale * 1000;

        can.dataTx[0] = 0x03;
        can.dataTx[1] = val_a_x / 256;
        can.dataTx[2] = val_a_x % 256;
        can.dataTx[3] = val_a_y / 256;
        can.dataTx[4] = val_a_y % 256;
        can.dataTx[5] = val_a_z / 256;
        can.dataTx[6] = val_a_z % 256;
        can.dataTx[7] = 0;
        can.id = 0xC0;
        can.size = 8;
        CAN_Send(&can);

        sent_flag = 3;
    }

    millis += 5;

    //---------------------SEND Gyro---------------------//
    if (millis % 100 == 0)
    {
        uint16_t val_g_x = gyro.x + gyro.scale * 1000;
        uint16_t val_g_y = gyro.y + gyro.scale * 1000;
        uint16_t val_g_z = gyro.z + gyro.scale * 1000;

        can.dataTx[0] = 0x04;
        can.dataTx[1] = val_g_x / 256;
        can.dataTx[2] = val_g_x % 256;
        can.dataTx[3] = val_g_y / 256;
        can.dataTx[4] = val_g_y % 256;
        can.dataTx[5] = val_g_z / 256;
        can.dataTx[6] = val_g_z % 256;
        can.dataTx[7] = 0;
        can.id = 0xC0;
        can.size = 8;
        CAN_Send(&can);

        sent_flag = 4;
    }

    millis += 5;

    //--------------------SEND Steer--------------------//
    if (millis % 100 == 0)
    {
        if (calibration_flag == 0)
        {
            can.dataTx[0] = 2;
            can.dataTx[1] = pot_2.val_100;
            can.dataTx[2] = 0;
            can.dataTx[3] = 0;
            can.dataTx[4] = 0;
            can.dataTx[5] = 0;
            can.dataTx[6] = 0;
            can.dataTx[7] = 0;
            can.id = 0xC0;
            can.size = 8;
            CAN_Send(&can);

            sent_flag = 5;
        }
    }

    return sent_flag;
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
    while (1)
    {
        HAL_UART_Transmit(&huart2, (uint8_t *)"Error on file: ", strlen("Error on file: "), 10);
        //HAL_UART_Transmit(&huart2, (uint8_t *)file, strlen(file), 10);
        HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, 10);
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
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
