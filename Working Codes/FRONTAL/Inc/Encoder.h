#ifndef ENCODER_H
#define ENCODER_H

#include <math.h>
#include <stdio.h>
#include <inttypes.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

//----------------ENCODER----------------//
/*
 *To use encoder functions you have to initialize two timers, one for the clock and one to generate an interrupt
 *To get the rotational speed of the encoder, you can setup the interrupt timer and call 'encoder_tim_interrupt()';
 *you have to declare a variable and pass it as argument to the encoder_tim_interrupt() function.
 *The porpouse of that variable is to switch from the three phases needed to calculate the rotational speed.
 *The first and the second phases are to request two angles from the encoder, then third is to calculate the speed.
 *For the configuration of the first timer go to the description of the read_encoder() function.
 *For the configuration of the second timer you have to configure it to generate an interrupt.
 *That interrupt must be long enough to calculate a speed but not too long because you have to get the two angles in the same wheel rotation.
 *
 *working settings:
 *interrupt timer -> prescaler 36, counter period 1000
 *timer2 -> prescaler = 18, counter period = 65500
 *pin PC8 = data in
 *pin PC9 = clock pin
 *angles_array[15]
*/

#include "stm32f4xx_hal_tim.h"

struct Encoder_Settings{
	int data_size;		// Bits sent from the sensor. exclude the error flag
	int speed_size;
	int interrupt_flag; // Flag to switch from angles to speed calculations
	int steer_enc_prescaler;
	int dx_wheel; // 1 if the encoder stc is for the right wheel

	float wheel_diameter;
	float max_delta_angle;

	float frequency;
	float clock_period;	// Period of the clock generated
	int frequency_timer_Hz;

	double conversion;

	TIM_HandleTypeDef *clock_timer; // Instance to the timer used to generate the clock
	TIM_HandleTypeDef *frequency_timer;
	TIM_HandleTypeDef *microsecond_timer; // timer to count elased microseconds. Set one tick per microsecond

	GPIO_TypeDef *ClockPinName;
	GPIO_TypeDef *DataPinName;
	uint16_t ClockPinNumber;
	uint16_t DataPinNumber;
};

struct Encoder_Data{
  	int* binary_data;
	int  decimal_data;

    int error_flag;		// Return value if the encoder has errors

    long int wheel_rotation;
    float Km;

    int speed_sign;
	double speed;
    double average_speed;	 // Filtered speed
    double* speed_array; // Array to store lasts speed

    double angle; // First angle calculated
    double angle_prec;
    double delta_angle;

	double actual_frequency;

	int new_data;
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