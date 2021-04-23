#ifndef __ADC_H
#define __ADC_H

#include "inttypes.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

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

void adc_shift_array(long double *array, int size, long double data);
double adc_dynamic_average(long double *array, int size);

#endif