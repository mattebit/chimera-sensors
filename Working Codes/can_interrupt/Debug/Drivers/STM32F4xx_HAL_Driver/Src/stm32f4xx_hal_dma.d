/**
  ******************************************************************************
  * @file    stm32f4xx_hal_tim.c
  * @author  MCD Application Team
  * @brief   TIM HAL module driver.
  *          This file provides firmware functions to manage the following 
  *          functionalities of the Timer (TIM) peripheral:
  *           + Time Base Initialization
  *           + Time Base Start
  *           + Time Base Start Interruption
  *           + Time Base Start DMA
  *           + Time Output Compare/PWM Initialization
  *           + Time Output Compare/PWM Channel Configuration
  *           + Time Output Compare/PWM  Start
  *           + Time Output Compare/PWM  Start Interruption
  *           + Time Output Compare/PWM Start DMA
  *           + Time Input Capture Initialization
  *           + Time Input Capture Channel Configuration
  *           + Time Input Capture Start
  *           + Time Input Capture Start Interruption 
  *           + Time Input Capture Start DMA
  *           + Time One Pulse Initialization
  *           + Time One Pulse Channel Configuration
  *           + Time One Pulse Start 
  *           + Time Encoder Interface Initialization
  *           + Time Encoder Interface Start
  *           + Time Encoder Interface Start Interruption
  *           + Time Encoder Interface Start DMA
  *           + Commutation Event configuration with Interruption and DMA
  *           + Time OCRef clear configuration
  *           + Time External Clock configuration
  @verbatim 
  ==============================================================================
                      ##### TIMER Generic features #####
  ==============================================================================
  [..] The Timer features include: 
       (#) 16-bit up, down, up/down auto-reload counter.
       (#) 16-bit programmable prescaler allowing dividing (also on the fly) the 
           counter clock frequency either by any factor between 1 and 65536.
       (#) Up to 4 independent channels for:
           (++) Input Capture
           (++) Output Compare
           (++) PWM generation (Edge and Center-aligned Mode)
           (++) One-pulse mode output               
   
                        ##### How to use this driver #####
  ==============================================================================
    [..]
     (#) Initialize the TIM low level resources by implementing the following functions 
         depending from feature used :
           (++) Time Base : HAL_TIM_Base_MspInit() 
           (++) Input Capture : HAL_TIM_IC_MspInit()
           (++) Output Compare : HAL_TIM_OC_MspInit()
           (++) PWM generation : HAL_TIM_PWM_MspInit()
           (++) One-pulse mode output : HAL_TIM_OnePulse_MspInit()
           (++) Encoder mode output : HAL_TIM_Encoder_MspInit()
           
     (#) Initialize the TIM low level resources :
        (##) Enable the TIM interface clock using __TIMx_CLK_ENABLE(); 
        (##) TIM pins configuration
            (+++) Enable the clock for the TIM GPIOs using the following function:
                 __GPIOx_CLK_ENABLE();   
            (+++) Configure these TIM pins in Alternate function mode using HAL_GPIO_Init();  

     (#) The external Clock can be configured, if needed (the default clock is the 
         internal clock from the APBx), using the following function:
         HAL_TIM_ConfigClockSource, the clock configuration should be done before 
         any start function.
  
     (#) Configure the TIM in the desired functioning mode using one of the 
         initialization function of this driver:
         (++) HAL_TIM_Base_Init: to use the Timer to generate a simple time base
         (++) HAL_TIM_OC_Init and HAL_TIM_OC_ConfigChannel: to use the Timer to generate an 
              Output Compare signal.
         (++) HAL_TIM_PWM_Init and HAL_TIM_PWM_ConfigChannel: to use the Timer to generate a 
              PWM signal.
         (++) HAL_TIM_IC_Init and HAL_TIM_IC_ConfigChannel: to use the Timer to measure an 
              external signal.
         (++) HAL_TIM_OnePulse_Init and HAL_TIM_OnePulse_ConfigChannel: to use the Timer 
              in One Pulse Mode.
         (++) HAL_TIM_Encoder_Init: to use the Timer Encoder Interface.
         
     (#) Activate the TIM peripheral using one of the start functions depending from the feature used: 
           (++) Time Base : HAL_TIM_Base_Start(), HAL_TIM_Base_Start_DMA(), HAL_TIM_Base_Start_IT()
           (++) Input Capture :  HAL_TIM_IC_Start(), HAL_TIM_IC_Start_DMA(), HAL_TIM_IC_Start_IT()
           (++) Output Compare : HAL_TIM_OC_Start(), HAL_TIM_OC_Start_DMA(), HAL_TIM_OC_Start_IT()
           (++) PWM generation : HAL_TIM_PWM_Start(), HA