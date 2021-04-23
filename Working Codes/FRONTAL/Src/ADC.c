#include "ADC.h"

pot_stc pot_1;
pot_stc pot_2;
pot_stc pot_3;
void calc_pot_value(pot_stc *pot, float maxVal)
{
	pot->val_100 = ((fabs(pot->val - pot->min) * maxVal / (pot->range)));
  //val0_100 -->STEER --> 0 = SX | 100 = DX
}

//Function to check if the two ADC values are approximately the same
//if the values are different for more tha 10 points percentage for more than 100 milliseconds returns the SCS Error
//TimerInstance = pointer to the timer needed to check the SCS error
//val0_100 = pointer to the first potentiometer
//val1_100 = pointer to the second potentiometer
int implausibility_check(pot_stc *pot_1, pot_stc *pot_2)
{

	int SCS1 = 0;

	if (pot_1->val_100 >= 100)
		pot_1->val_100 = 100;

	if (pot_1->val_100 <= 5)
		pot_1->val_100 = 0;

	if (pot_2->val_100 >= 100)
		pot_2->val_100 = 100;

	if (pot_2->val_100 <= 5)
		pot_2->val_100 = 0;

	if (fabs(pot_1->val_100 - pot_2->val_100) >= 10)
	{
		if (__HAL_TIM_GET_COUNTER(pot_1->TimerInstance) > 100)
		{
			SCS1 = 1;
		}
	}
	else
	{
		__HAL_TIM_SET_COUNTER(pot_1->TimerInstance, 0);
		SCS1 = 0;
	}

	return SCS1;
}

//function to set the value of the potentiometer when the pedal is released
//val = array pointer to the potentiometer values
//max1 = maximum value of the APPS1
//max2 = maximum value of the APPS2
void set_max(pot_stc *pot_1)
{
	pot_1->max = pot_1->val;
}

//function to set the value of the potentiometer when the pedal is pressed
//val = array pointer to the potentiometer values
//min1 = minimum value of the APPS1
//min2 = minimum value of the APPS2
void set_min(pot_stc *pot_1)
{
	pot_1->min = pot_1->val;
}

//shift all the data of a numeric array and add another one value
//array = array to be shifted
//size = size of the array
//data = value to be added in the last position of the array
void adc_shift_array(long double *array, int size, long double data)
{

	for (int i = 1; i < size; i++)
	{
		array[i - 1] = array[i];
	}
	array[size - 1] = data;
}

//function that calculate the average of all the numbers in one array
double adc_dynamic_average(long double *array, int size)
{

	double sum = 0;
	double average = 0;

	for (int i = 0; i < size; i++)
	{
		sum += array[i];
	}
	average = sum / size;

	return average;
}