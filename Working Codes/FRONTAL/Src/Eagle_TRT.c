#include "Eagle_TRT.h"
#include "stm32f4xx_hal_conf.h"

/*
 *Driver for all the stm in the Eagle_TRT veichle
 *incude this driver in the main file
 *you can't use all the functions unless you set up the CubeMx project correctly
*/

#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
//function that sends an array via CAN
//hcan = pointer to can port
//id = id of the message to be sent
//dataTx = pointer to array that contains the data to be sent
//size = size of the array
can_stc can;
int CAN_Send(can_stc *can)
{

	uint32_t mailbox;
	uint8_t flag = 0;

	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.StdId = can->id;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = can->size;
	TxHeader.TransmitGlobalTime = DISABLE;

	if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0 && HAL_CAN_IsTxMessagePending(can->hcan, CAN_TX_MAILBOX0) == 0)
	{
		HAL_CAN_AddTxMessage(can->hcan, &TxHeader, can->dataTx, &mailbox);
		flag = 1;
	}

	return flag;
}

//receive a buffer from the CAN communication
//you can call this function in the callback of the CAN interrupt
//hcan = pointer to can port
//DataRx = pointer to the buffer you are receiveng
//size = size of the buffer you are using
int CAN_Receive(can_stc *can)
{

	CAN_RxHeaderTypeDef RxHeader;

	if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) != 0)
	{
		HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &RxHeader, can->dataRx);
	}

	int id = RxHeader.StdId;

	return id;
}
#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"



pot_stc pot_1;
pot_stc pot_2;
pot_stc pot_3;
void calc_pot_value(pot_stc *pot, float maxVal)
{

	pot->val_100 = ((abs(pot->val - pot->min) * maxVal / (pot->range))); //val0_100 -->STEER --> 0 = SX | 100 = DX
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
	{
		pot_1->val_100 = 100;
	}
	if (pot_1->val_100 <= 5)
	{
		pot_1->val_100 = 0;
	}
	if (pot_2->val_100 >= 100)
	{
		pot_2->val_100 = 100;
	}
	if (pot_2->val_100 <= 5)
	{
		pot_2->val_100 = 0;
	}
	if (abs(pot_1->val_100 - pot_2->val_100) >= 10)
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
//max1 = pointer to the maximum value of the APPS1
//max2 = pointer to the maximum value of the APPS2
void set_max(pot_stc *pot_1)
{
	pot_1->max = pot_1->val;
}

//function to set the value of the potentiometer when the pedal is pressed
//val = array pointer to the potentiometer values
//min1 = pointer to the minimum value of the APPS1
//min2 = pointer to the minimum value of the APPS2
void set_min(pot_stc *pot_1)
{
	pot_1->min = pot_1->val;
}

#endif

//function to calculate the decimal value from MSB binary array
//bin = pointer to binary array
//max = size of the array
int bin_dec(int *bin, int size)
{

	int dec = 0;

	for (int i = 0; i < size; i++)
	{
		if (bin[i] == 1)
		{
			dec += Power(2, size - i - 1);
		}
	}
	return dec;
}

//function to calculate the power of a given number
double Power(int base, int expn)
{

	double result = 1;

	if (expn != 0)
	{
		for (int j = 0; j < expn; j++)
		{
			result = result * base;
		}
	}

	return result;
}

//shift all the data of a numeric array and add another one value
//array = array to be shifted
//size = size of the array
//data = value to be added in the last position of the array
void shift_array(long double *array, int size, long double data)
{

	for (int i = 1; i < size; i++)
	{
		array[i - 1] = array[i];
	}
	array[size - 1] = data;
}

double speed_filter(double *data, int size)
{
	double min = 100000000000000;
	double max = -min;
	double sum = 0;
	double average = 0;
	int index_1;
	int index_2;
	int average_members = 0;

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
		if (i != index_1 || i != index_2)
		{
			sum += data[i];
			average_members++;
		}
	}
	average = sum / average_members;

	return average;
}

//function that calculate the average of all the numbers in one array
double dynamic_average(long double *array, int size)
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
