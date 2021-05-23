#include "Encoder.h"

/**
 * Current encoder
 * Zettlex
 * INC-8-37.050-151001-SSI3-RC371-5-S
 * https://www.zettlex.com/wp-content/uploads/IncOder-Product-Guide_Rev_4.11.1.pdf
 * 
 * SSI Protocol
 * T: clock period 100kHz - 2 MHz
 * Tmu: update time 20 +-1 us (minimum between two requsts of angle)
 * First falling exdge starts the ReadCycle
 * Each rising clock is a bit starting from D(n-1)
 * After last clock ad error flag is sent for the period Tmu - 0.5*T
*/


// Interrupt function of tim 2
// Call this function in the timer callback function of the stm
// htim = timer clock_timer of the timer that you are using for the clock of the encoder
// Interrupt_flag = initilize a int variable in the main file
// Angles_array = array to store the last angles
// Speed = pointer to the speed value
void encoder_tim_interrupt(struct Encoder_Settings *settings, struct Encoder_Data* data)
{
	// Requesting second angle
	data->angle_prec = data->angle;
	read_SSI(settings, data);
	data->angle = ((double)data->decimal_data) / settings->conversion;
	

	// Timer counter, the timer MUST me setted to tick one per microsecond.
	// data->actual_frequency = settings->microsecond_timer->Instance->CNT;
	data->actual_frequency = 1000000/((double)(__HAL_TIM_GET_COUNTER(settings->microsecond_timer)));
	__HAL_TIM_SET_COUNTER(settings->microsecond_timer, 0);

	// Calculate speed from the two angles
	get_speed_encoder(settings, data);

    data->speed_sign = data->average_speed < 0 ? 1 : 0;

	data->new_data = 1;
}



// Function to request data from encoder via SSI communication
// This function is called from the interrupt callback of the timer that you are using for the encoder
// The tim used for this function must be initialized at most at 2 microsecond per tick
// Lower the number of microseconds per tick better it is
// clock_timer = struct of the tim used for the encoder
void read_SSI(struct Encoder_Settings *settings, struct Encoder_Data* data)
{

  	// From HIGH set to LOW
	HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period){}
	HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period){}
	HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_RESET);
	__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period){}

	// Starting the clock to retrieve data_size bits from the sensor
	for (int i = 0; i < settings->data_size; i++)
	{
		// CLOCK HIGH
    	// Waiting only half of period to read incoming data
		HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	  	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period/2){}

		//Reading the Pin at the half of the clock period
		// Set the bit as the pin state (0 or 1)
		data->binary_data[i] = HAL_GPIO_ReadPin(settings->DataPinName, settings->DataPinNumber);

		__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	  	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period/2){}

		// CLOCK LOW
		HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	  	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period){}
	}

	// Requesting an other bit for the eventual error sent from the sensor
	HAL_GPIO_WritePin(settings->ClockPinName, settings->ClockPinNumber, GPIO_PIN_SET);
	__HAL_TIM_SET_COUNTER(settings->clock_timer, 0);
	while (__HAL_TIM_GET_COUNTER(settings->clock_timer) <= settings->clock_period){}

	data->error_flag = HAL_GPIO_ReadPin(settings->DataPinName, settings->DataPinNumber);

	// Converting bits into number and converting it into angle in degrees (0 ~ 359)
	data->decimal_data = encoder_bin_dec(data->binary_data, settings->data_size);
}



// Funtion to calculate the speed
// Angle0 = last angle calculated
// Angle1 = previous angle calculated
// Wheel_diameter = diameter of the wheel expressed meters
void get_speed_encoder(struct Encoder_Settings *settings, struct Encoder_Data* data)
{

	if (settings->dx_wheel == 1)
		data->delta_angle = data->angle - data->angle_prec;
	else
		data->delta_angle = data->angle_prec - data->angle;

	if ((data->angle_prec 	< settings->max_delta_angle && data->angle 		> (2 * M_PI) - settings->max_delta_angle) ||
		(data->angle 		< settings->max_delta_angle && data->angle_prec > (2 * M_PI) - settings->max_delta_angle))
	{
		if (data->delta_angle < 0)
		{
			data->delta_angle = (2 * M_PI) + data->delta_angle;
		}
		else
		{
      		data->delta_angle = data->delta_angle - (2 * M_PI);
		}
	}

	data->speed = data->delta_angle * settings->frequency;

	// Remove noise mediating previous values with actual
	encoder_shift_array(data->speed_array, settings->speed_size, data->speed);
	//data->average_speed = encoder_dynamic_average(data->speed_array, settings->speed_size);
	data->average_speed = encoder_speed_filter(data->speed_array, settings->speed_size);

	if (data->average_speed < -1 || data->average_speed > 1)
	{
		if (((data->angle_prec > (2 * M_PI) - settings->max_delta_angle) && (data->angle      < settings->max_delta_angle)) ||
			  ((data->angle    > (2 * M_PI) - settings->max_delta_angle) && (data->angle_prec < settings->max_delta_angle)))
		{
			data->wheel_rotation++;
			data->Km = data->wheel_rotation * settings->wheel_diameter * M_PI;
		}
	}
}


//function to calculate the decimal value from MSB binary array
//bin = pointer to binary array
//max = size of the array
int encoder_bin_dec(int *bin, int size)
{
	int dec = 0;

	for (int i = 0; i < size; i++)
		dec += bin[i] << (size-i-1);

	return dec;
}

//function to calculate the power of a given number
double encoder_Power(double base, int expn)
{
	double result = 1.0;

	if (expn != 0)
		for (int j = 0; j < expn; j++)
			result *= base;

	return result;
}

// shift all the data of a numeric array and add another one value
// array = array to be shifted
// size = size of the array
// data = value to be added in the last position of the array
void encoder_shift_array(double *array, int size, double data)
{
	for (int i = 1; i < size; i++)
		array[i - 1] = array[i];

	array[size - 1] = data;
}

double encoder_speed_filter(double *data, int size)
{
	double min = data[0];
	double max = data[0];
	long double sum = 0;
	int index_1 = 0;
	int index_2 = 0;

	for (int i = 0; i < size; i++)
	{
		if (data[i] < min)
		{
			min = data[i];
			index_1 = i;
		}

		if (data[i] > max)
		{
			max = data[i];
			index_2 = i;
		}
	}

	for (int i = 0; i < size; i++)
	{
		if (i != index_1 && i != index_2)
		{
			sum += data[i];
		}
	}

	return sum / (size - 2);
}

//function that calculate the average of all the numbers in one array
double encoder_dynamic_average(double *array, int size)
{
	double sum = 0;

	for (int i = 0; i < size; i++)
	{
		sum += array[i];
	}

	return sum / size;
}
