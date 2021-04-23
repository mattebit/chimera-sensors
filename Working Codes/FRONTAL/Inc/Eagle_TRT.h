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

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
///---queue---///
typedef struct
{
	int head, tail;
	int dim;
	char *elem[40];
	char stringa[50];
} queue;
enum retval
{
	FAIL,
	OK
};
void init(queue *);
int push(char *, queue *);
int pop(char *, queue *);
///---end queue---///
int print(UART_HandleTypeDef *huart, char *text_print_function);
void print_it(UART_HandleTypeDef *huart);

#endif

//----------------ENCODER----------------//
#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"

typedef struct
{

	float val_100;
	float max;
	float min;
	int range;
	int val;

	TIM_HandleTypeDef *TimerInstance;
} pot_stc;
int implausibility_check(pot_stc *, pot_stc *);
void calc_pot_value(pot_stc *, float);
void set_max(pot_stc *);
void set_min(pot_stc *);

#endif

int bin_dec(int *bin, int size);
double Power(int base, int expn);
void shift_array(long double *array, int size, long double data);
double speed_filter(double *data, int size);
double dynamic_average(long double *array, int size);

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