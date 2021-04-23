#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include "inttypes.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"

#include "stm32f4xx_hal_tim.h"

struct Encoder_Settings{
	int data_size;		// Bits sent from the sensor. exclude the error flag
	int interrupt_flag; // Flag to switch from angles to speed calculations
	int steer_enc_prescaler;
	int dx_wheel; // 1 if the encoder stc is for the right wheel

	float wheel_diameter;
	float max_delta_angle;

	float frequency;
	float clock_period;	// Period of the clock generated
	int frequency_timer_Hz;

	TIM_HandleTypeDef *TimerInstance; // Instance to the timer used to generate the clock
	TIM_HandleTypeDef *frequency_timer;

	GPIO_TypeDef *ClockPinName;
	GPIO_TypeDef *DataPinName;
	uint16_t ClockPinNumber;
	uint16_t DataPinNumber;
};

struct Encoder_Data{
  	int* Data;
    int converted_data;		 // Angle data

    int error_flag;		// Return value if the encoder has errors

    long int wheel_rotation;
    float Km;

    int speed_sign;
    double average_speed;	 // Filtered speed
    double* speed_array; // Array to store lasts speed

    double angle0; // First angle calculated
    double angle1; // Second angle calculated
    double angle0_prec;
    double angle1_prec;
    double delta_angle;
};

void read_SSI(struct Encoder_Settings *, struct Encoder_Data *);
void encoder_tim_interrupt(struct Encoder_Settings *, struct Encoder_Data *);
void get_speed_encoder(struct Encoder_Settings *, struct Encoder_Data *);

int encoder_bin_dec(int *bin, int size);
double encoder_Power(double base, int expn);
void encoder_shift_array(double *array, int size, double data);
double encoder_speed_filter(double *data, int size);
double encoder_dynamic_average(double *array, int size);

#endif