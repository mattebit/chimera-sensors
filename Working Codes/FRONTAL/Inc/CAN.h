#ifndef _EAGLE_TRT_H
#define _EAGLE_TRT_H

#include "inttypes.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

//----------------CAN----------------//
#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
typedef struct
{

	int id;
	int size;

	uint8_t dataTx[8];
	uint8_t dataRx[8];

	CAN_HandleTypeDef *hcan;
} can_stc;

int CAN_Send(can_stc *);
int CAN_Receive(can_stc *);
#endif

#endif