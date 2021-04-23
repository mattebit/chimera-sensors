#include "CAN.h"
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