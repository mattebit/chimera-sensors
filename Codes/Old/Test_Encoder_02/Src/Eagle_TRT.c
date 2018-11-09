#include "Eagle_TRT.h"
#include "stm32f4xx_hal_conf.h"

//ADC interrupt
//gps interrupt

/*
 *Driver for all the stm in the Eagle_TRT veichle
 *incude this driver in the main file
 *you can't use all the functions unless you set up the CubeMx project correctly
*/

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
*/

//----------------GPS----------------//
/*
 *To use GPS functions you have to setup the UART port to communicate with it.
 *Possibly set the port as interrupt because the received data are cleaner.
 *Call GPS_INIT() to setup the GPS
 *Every loop read the input data.
 *Then the function Get_Sentence() looks for a sequence of letters like "GPRMC"
 *Look at NMEA protocol, there you can find all the strings that a GPS can send,
 *every one of them defines the types of data that the GPS sends.
 *If Get_Sentence() found one string, returns the number of the position of that string in the char Matrix.
 *Go to Get_Sentence() description to find an example of Matrix
 *Once you have the number of the string you can call Get_Requested_Data() to get the data that you need.
 *If you need to know if the GPS is connected to at least 3 satellites call Is_Valid() function
 *
 *checksum calculator http://www.hhhh.org/wiml/proj/nmeaxor.html
*/

//----------------IMU----------------//
/*
 *Setup SPI port to communicate with the sensor
 *Call gyro_init() and magn_accel_init() to setup the sensor
 *If you want to calibrate the sensor call gyro_calib() and accel_calib().
 *You can read the x, y, z data from the Gyroscope by calling gyro_read()
 *You can read the x, y, z data from the Gyroscope by calling accel_read()
*/


#ifdef HAL_SPI_MODULE_ENABLED
#include "stm32f4xx_hal_spi.h"
	//gyro initialization function
	//call this function before requesting data from the sensor
	void gyro_init(SPI_HandleTypeDef *hspi){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG1_G_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG1_G_VAL, 1, 10); ///Writing 0b00001111 to enable PowerMode and x,y,z axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG4_G_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG4_G_VAL, 1, 10); ///Writing 0b00010000 to set full-scale selection to 500dps
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
	}

	//accelerometer and magnetometer initialization
	//call this function before requesting data from the sensor
	void magn_accel_init(SPI_HandleTypeDef *hspi){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG1_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG1_XM_VAL, 1, 10); ///Writing 0b10100111 to enable 1600Hz and x,y,z axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG2_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG2_XM_VAL, 1, 10); ///Writing 0b00001000 to set +/-4g range for axel axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG5_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG5_XM_VAL, 1, 10); ///Writing 0b01110000 to set high resolution for magn and 50Hz
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG6_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG6_XM_VAL, 1, 10); ///Writing 0b00100000 to set +/-4 gauss range for magn axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG7_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(hspi, (uint8_t*)&CTRL_REG7_XM_VAL, 1, 10); ///Writing 0b00000000 to set continuos conversion for magn axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
	}

	//this function is used to calibrate the gyroscope
	void gyro_calib(SPI_HandleTypeDef *hspi, float * X_G_axis_offset, float * Y_G_axis_offset, float * Z_G_axis_offset){
		float kp_G = 0.0175;
		*X_G_axis_offset = LSM9DS0_calib(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_X_L_G_ADD, OUT_X_H_G_ADD, kp_G);
		*Y_G_axis_offset = LSM9DS0_calib(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Y_L_G_ADD, OUT_Y_H_G_ADD, kp_G);
		*Z_G_axis_offset = LSM9DS0_calib(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Z_L_G_ADD, OUT_Z_H_G_ADD, kp_G);
	}


	//this function is used to calibrate the accelerometer
	void accel_calib(SPI_HandleTypeDef *hspi, float * X_A_axis_offset, float * Y_A_axis_offset, float * Z_A_axis_offset){
		float kp_A = 0.00119782; ///0.000122 * 9,81
		*X_A_axis_offset = LSM9DS0_calib(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_X_L_A_ADD, OUT_X_H_A_ADD, kp_A);
		*Y_A_axis_offset = LSM9DS0_calib(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Y_L_A_ADD, OUT_Y_H_A_ADD, kp_A);
		*Z_A_axis_offset = LSM9DS0_calib(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Z_L_A_ADD, OUT_Z_H_A_ADD, kp_A);
	}


	float LSMD9S0_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp)
	{
		uint8_t OUT_L_VAL;
		uint8_t OUT_H_VAL;

		///READING X_AXIS ROTATION
		HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
		HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
		HAL_SPI_Transmit(hspi, &REG_L, 1, 10); ///Writing LOW address
		HAL_SPI_Receive(hspi, (uint8_t*)&OUT_L_VAL, 1, 10); ///Saving LOW data
		HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
		HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0


		HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
		HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
		HAL_SPI_Transmit(hspi, &REG_H, 1, 10); ///Writing HIGH address
		HAL_SPI_Receive(hspi, (uint8_t*)&OUT_H_VAL, 1, 10); ///Saving HIGH data
		HAL_GPIO_WritePin(GPIOx_InUse, GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
		HAL_GPIO_WritePin(GPIOx_NotInUse, GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0

		///CALCULATING X_AXIS ROTATION
		float axis = OUT_H_VAL << 8 | OUT_L_VAL;	///Calculating axis value shifting and using a logic OR
		if (axis > 32767){ ///Generating positive and negative value of rotation
			axis = axis - 65536;
		}
		axis = axis * kp; ///Scaling axis value with appropriate conversion factor from datasheet

		return axis;
	}

	float LSM9DS0_calib(SPI_HandleTypeDef *hspi, GPIO_TypeDef* GPIOx_InUse, uint16_t GPIO_Pin_InUse, GPIO_TypeDef* GPIOx_NotInUse, uint16_t GPIO_Pin_NotInUse, uint8_t REG_L, uint8_t REG_H, float kp)
	{
		float axis_cal;
		float sum_cal = 0.0000;
		for(int i = 0; i < 10000; i++){
			float tmp = LSMD9S0_read(hspi, GPIOx_InUse, GPIO_Pin_InUse, GPIOx_NotInUse, GPIO_Pin_NotInUse, REG_L, REG_H, kp);
			sum_cal = sum_cal + tmp;
		}
		axis_cal = sum_cal / 10000;
		return axis_cal;
	}
	int LSMD9S0_check(SPI_HandleTypeDef *hspi){
		int check = 0;

		///GYRO IS WORKING
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(hspi, (uint8_t*)&WHO_AM_I_G, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
		HAL_SPI_TransmitReceive(hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10); ///Reading from register sending a 0x00
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0

		///AXEL/MAGN ARE WORKING
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(hspi, (uint8_t*)&WHO_AM_I_XM, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_XM (giving by &variable)
		HAL_SPI_TransmitReceive(hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_XM_VAL, 1, 10); ///Reading from register sending a 0x00
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); ///CS_XM to 1

		///AXEL/GYRO STATUS
		if (WHO_AM_I_G_VAL != 212){
			check = 1;
		}
		if (WHO_AM_I_XM_VAL != 212){
			check = 2;
		}
		if ((WHO_AM_I_G_VAL != 212) & (WHO_AM_I_XM_VAL != 212)){
			check = 3;
		}

		return check;
	}
	///Reading G_axis values
	void gyro_read(SPI_HandleTypeDef *hspi,float * X_G_axis, float * Y_G_axis, float * Z_G_axis, float *X_G_axis_offset,float * Y_G_axis_offset,float * Z_G_axis_offset){
		float kp_G = 0.0175;
		*X_G_axis = LSMD9S0_read(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_X_L_G_ADD, OUT_X_H_G_ADD, kp_G);
		*X_G_axis = X_G_axis - X_G_axis_offset;
		*Y_G_axis = LSMD9S0_read(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Y_L_G_ADD, OUT_Y_H_G_ADD, kp_G);
		*Y_G_axis = Y_G_axis - Y_G_axis_offset;
		*Z_G_axis = LSMD9S0_read(hspi,GPIOA, GPIO_PIN_8, GPIOC, GPIO_PIN_9, OUT_Z_L_G_ADD, OUT_Z_H_G_ADD, kp_G);
		*Z_G_axis = Z_G_axis - Z_G_axis_offset;
	}
	///Reading A_axis values
	void accel_read(SPI_HandleTypeDef *hspi,float * X_A_axis, float * Y_A_axis, float * Z_A_axis,float *X_A_axis_offset,float * Y_A_axis_offset,float * Z_A_axis_offset){
		float kp_A = 0.00119782; ///0.000122 * 9,81
		*X_A_axis = LSMD9S0_read(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_X_L_A_ADD, OUT_X_H_A_ADD, kp_A);
		*X_A_axis = X_A_axis - X_A_axis_offset;
		*Y_A_axis = LSMD9S0_read(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Y_L_A_ADD, OUT_Y_H_A_ADD, kp_A);
		*Y_A_axis = Y_A_axis - Y_A_axis_offset;
		*Z_A_axis = LSMD9S0_read(hspi,GPIOC, GPIO_PIN_9, GPIOA, GPIO_PIN_8, OUT_Z_L_A_ADD, OUT_Z_H_A_ADD, kp_A);
		*Z_A_axis = Z_A_axis - Z_A_axis_offset + 9.81;
	}

#endif


#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
	//function that sends an array via CAN
	//hcan = pointer to can port
	//id = id of the message to be sent
	//dataTx = the array that contains the data to be sent
	//size = size of the array
	int CAN_Send(CAN_HandleTypeDef *hcan,int id, uint8_t dataTx[], int size){

		uint32_t mailbox;
		uint8_t flag = 0;

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = id;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = size;
		TxHeader.TransmitGlobalTime = DISABLE;

		if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) != 0 && HAL_CAN_IsTxMessagePending(hcan, CAN_TX_MAILBOX0) == 0){
			HAL_CAN_AddTxMessage(hcan, &TxHeader, dataTx, &mailbox);
			flag = 1;
		}

		return flag;
	}


	//receive a buffer from the CAN communication
	//you can call this function in the callback of the CAN interrupt
	//DataRx = pointer to the buffer you are receiveng
	//size = size of the buffer you are using
	int CAN_Receive(CAN_HandleTypeDef *hcan,uint8_t *DataRx, int size){
		CAN_RxHeaderTypeDef RxHeader;

		if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) != 0){
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, DataRx);
		}

		int id = RxHeader.StdId;

		return id;
	}
#endif

#ifdef HAL_UART_MODULE_ENABLED
#include "stm32f4xx_hal_uart.h"
	//print the given buffer
	//huart = number of huart to be used
	//text = char buffer to be sent
	void print(UART_HandleTypeDef *huart, char* text){
		HAL_UART_Transmit(huart, (uint8_t*)text, strlen(text), 5);
		HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 1);
	}


	//function to initilize the GPS call it before requesting data from it
	//to use this function, initialize a global int variable to 9600
	//you have to put that variable in the huart Init function of the huart used for the GPS
	//in particular you have to put it in the baudrate of the huart init function
	//int his function that variable changes value from 9600 to 57600
	//huart = huart at which GPS is connected
	//baud = globl variable initilized to 9600
	void GPS_INIT(UART_HandleTypeDef *huart){

		huart->Init.BaudRate = 57600;
		if (HAL_UART_Init(huart) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_SET_BAUD_57600, strlen(PMTK_SET_BAUD_57600), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}

		//change the baud rate of the stm to 57600
		HAL_Delay(50);
		//MX_USART1_UART_Init();
		huart->Init.BaudRate = 57600;
		if (HAL_UART_Init(huart) != HAL_OK)
		{
			_Error_Handler(__FILE__, __LINE__);
		}

		//send other commands to speed up the data flow
		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_API_SET_FIX_CTL_5HZ, strlen(PMTK_API_SET_FIX_CTL_5HZ), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}

		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_SET_NMEA_OUTPUT_ALLDATA, strlen(PMTK_SET_NMEA_OUTPUT_ALLDATA), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}

		HAL_Delay(100);
	}

	//function to awake the GPS
	void GPS_Awake(UART_HandleTypeDef *huart){
		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_AWAKE, strlen(PMTK_AWAKE), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}
		for(int i = 0; i < 50; i++){
			HAL_UART_Transmit(huart, (uint8_t*)PMTK_Q_RELEASE, strlen(PMTK_Q_RELEASE), 20);
			HAL_UART_Transmit(huart, (uint8_t*)"\r\n", 2, 4);
		}
	}
#endif



#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"

	//function to request data from encoder via SSI communication
	//this function is called from the interrupt callback of the timer that you are using for the encoder
	//the tim used for this function must be initialized at most at 2 microsecond per tick
	//lower the number of microseconds per tick better it is
	//TimerInstance = struct of the tim used for the encoder
	double read_encoder(TIM_HandleTypeDef *TimerInstance){
		int clock_loop = 16;
		double int_data = 0;
		int clock_period = 2;
		int Data[50];

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	//clock was high: reset to low
		__HAL_TIM_SET_COUNTER(TimerInstance, 0);			//delay of 1 microsecond like from datasheet
		while(__HAL_TIM_GET_COUNTER(TimerInstance) <= clock_period){
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	//clock set to high to request the bit
		__HAL_TIM_SET_COUNTER(TimerInstance, 0);			//delay of 1 microsecond like from datasheet
		while(__HAL_TIM_GET_COUNTER(TimerInstance) <= clock_period){
		}
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);	//clock was high: reset to low

		for(int i = 0; i < clock_loop; i++){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);	//clock set to high to request the bit

			__HAL_TIM_SET_COUNTER(TimerInstance, 0);			//delay of 1 microsecond like from datasheet
			while(__HAL_TIM_GET_COUNTER(TimerInstance) <= clock_period){
			}

			if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET && i <= 15){	//reading the data input
				Data[i] = 1;
			}
			else{
				Data[i] = 0;
			}

			if(i == clock_loop-1){												//if it is the last loop set the clock pin to high, else set to low
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

				__HAL_TIM_SET_COUNTER(TimerInstance, 0);			//delay of 20 microsecond like from datasheet
				while(__HAL_TIM_GET_COUNTER(TimerInstance) <= 40){
				}
			}
			else{
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
			}

			__HAL_TIM_SET_COUNTER(TimerInstance, 0);					//delay of anothe 1 micros like from datasheet
			while(__HAL_TIM_GET_COUNTER(TimerInstance) <= clock_period){
			}

			if(i == 15){												//if it is the last loop cast the sata from binary to decimal
				int_data = bin_dec(Data);

				//sprintf(txt, "%d", (int)int_data);

				int_data = int_data / 45.5055;							//conversions from raw data to angle
				int_data /= 2;

				//Error = Data[15];										//If 0 no error, if 1 the encoder is unplugged

				//sprintf(txt, "%d7891", (int)int_data);
				//sprintf(txt, "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d", Data[0], Data[1], Data[2], Data[3], Data[4], Data[5], Data[6], Data[7], Data[8], Data[9], Data[10], Data[11], Data[12], Data[13], Data[14], Data[15]);
				//print(txt);
			}

		}
		return int_data;
	}

	//interrupt function of tim 2
	//call this function in the timer callback function of the stm
	//htim = timer TimerInstance of the timer that you are using for the clock of the encoder
	void encoder_tim_interrupt(TIM_HandleTypeDef *htim, int * interrupt_flag, double * angles_array, double * speed){
		double average_speed = 0;
		if(*interrupt_flag == 0){									//every 3 times request the angle from encoder
			angles_array[0] = read_encoder(htim);
			interrupt_flag ++;
		}
		else{
			if(*interrupt_flag == 1){									//every 3 times request the angle from encoder
				angles_array[1] = read_encoder(htim);
				interrupt_flag++;
			}
			else{
				if(*interrupt_flag == 2){									//calculate the speed from the two last angles
					double Speed = get_speed_encoder(angles_array[0], angles_array[1],1000,0.4064);
					if(abs(Speed - speed[8]) <= abs(speed[8] * 10)){			//exclude the wrong speeds
						shift_array(speed, 15, Speed);
						average_speed = dynamic_average(speed, 15);
					}
					*interrupt_flag = 0;
				}
			}
		}
	}


	//Function to check if the two ADC values are approximately the same
	//if the values are different for more tha 10 points percentage for more than 100 milliseconds returns the SCS Error
	int implausibility_check(TIM_HandleTypeDef *TimerInstance, int * Val0_100, int * Val1_100){

		int SCS1 = 0;

		if (*Val0_100 >= 100){
			*Val0_100 = 100;
		}
		if (*Val0_100 <= 5){
			*Val0_100 = 0;
		}

		if (*Val1_100 >= 100){
			*Val1_100 = 100;
		}
		if (*Val1_100 <= 5){
			*Val1_100 = 0;
		}

		if(abs(*Val0_100 - *Val1_100) >= 10){
			if(__HAL_TIM_GET_COUNTER(TimerInstance) > 100){
				SCS1 = 1;
			}
		}
		else{
			__HAL_TIM_SET_COUNTER(TimerInstance, 0);
			SCS1 = 0;
		}

		return SCS1;
	}

#endif


//function to calculate the decimal value from MSB binary array
int bin_dec(int* bin){
	int dec = 0;
	int max = 15;

	for(int i = 0; i < max; i++){
		if(bin[i] == 1){
			dec += Power(2, max-i-1);
		}
	}

	return dec;
}


//function to calculate the power of a given number
double Power(int base, int expn){
	double result = 1;
	if(expn != 0){
		for(int j = 0; j < expn; j++){
			result = result * base;
		}
	}

	return result;
}


//shift all the data of a numeric array and add another one value
//array = array to be shifted
//size = size of the array
//data = value to be added in the last position of the array
void shift_array(double *array, int size, double data){
	for(int i = 1; i < size; i++){
		array[i-1] = array[i];
	}
	array[size-1] = data;
}


//funtion to calculate the speed
//angle0 = last angle calculated
//angle1 = previous angle calculated
//refresh = delta-time from the two calculations, express it in microseconds
//wheel_diameter = diameter of the wheel expressed meters
double get_speed_encoder(float angle0, float angle1, int refresh, float wheel_diameter){
	double meters_per_second = 0;
	double dt = 0;

	dt = refresh / 1000000;
	meters_per_second = ((angle0 - angle1)/360)*3.14159265359*(wheel_diameter);			//calculating the speed using the circumference arc
	meters_per_second /= dt;

	//sprintf(txt, "%d", (int)(speed[9]*100));
	//print(txt);

	return meters_per_second;
}

//function that calculate the average of all the numbers in one array
double dynamic_average(double *array, int size){
	double sum = 0;
	double average = 0;

	for(int i = 0; i < size; i++){
		sum += array[i];
	}
	average = sum / size;

	return average;
}

void calc_pot_value(int max, int min, int range, float * val0_100, int * val){
	*val0_100 = (int)100-(abs(val[0] - min)*100/(range)); //val0_100 -->STEER --> 0 = SX | 100 = DX
	if (val[0] <= min){
		*val0_100 = 100;
	}
	if (val[0] >= max){
		*val0_100 = 0;
	}
}


//get the requested data from an NMEA string received from the GPS
//data_pos = position of the requested data
char* Get_Requested_Data(char * bufferRx, int data_pos, char * requested_data){
	int count = 0;
	int found = 0; //if data found 1 else 0
	int index = 0;

	//clear this buffer
	for(int i = 0; i < strlen(requested_data); i++){
		requested_data[i] = ' ';
	}

	for(int i = 0; i < strlen(bufferRx); i++){
		//count all the comma
		if(bufferRx[i] == ','){
			count++;
		}
		//the '-' is the last character in the input data
		if(bufferRx[i] == '-'){
			break;
		}

		if(found == 1){
			//until we find an another comma, add the data to the char
			if(bufferRx[i] == ','){
				return requested_data;
			}
			else{
				requested_data[index] = bufferRx[i];
				index ++;
			}
		}

		if(count == data_pos){
			found = 1;
		}

	}
	return "0";
}

//checks if the GPS is connected to some satellites
//if fix is 1 is connected if fix is 0 is not connected otherwise is -1 if I didn't found the data
int* Is_Valid(char * bufferRx, int * fix, char * requested_data){
	char* letter = Get_Requested_Data(bufferRx, 2, requested_data);

	if(*letter == 'A'){
		*fix = 1;
		return fix;
	}
	else{
		if(*letter == 'V'){
			*fix = 0;
			return fix;
		}
		else{
			*fix = -1;
			return fix;
		}
	}
}


//returns the number of the sentence found of the string given as argument
//bufferRX = string to be checked
//sentences = matrix of char in which the strings are saved
//len = length of the matrix sentences
//	0	1	2	3	4
//0	G	P	R	M	A
//1	G	P	R	M	C
//2 G	P	V	T	G
//3 G	P	V	B	W
//4 G	P	G	G	A
int Get_Sentence(char * bufferRx, char (*sentences)[5], int len){
	char sentence[5];
	int flag = 0;
	char * pointer;

	//check in the matrix where the strings are saved
	for(int i = 0; i < len; i++){
		for(int j = 0; j < 5; j++){
			sentence[j] = sentences[i][j];
		}

		if(strstr(bufferRx, sentence) != NULL){
			pointer = strstr(bufferRx, sentence);
			strcpy(bufferRx, pointer);

			flag = 1;
		}
		else{
			flag = 0;
			continue;
		}
		if(flag == 1){
			//return(i);

			if(i >= 0 && i <= len){
				return i;
			}
			else{
				return -1;
			}
			/*
			switch(i){
			case 0:
			return 0;  MX_USART1_USART_Init();

			break;
			case 1:
			return 1;
			break;
			case 2:
			return 2;
			break;
			case 3:
			return 3;
			break;
			case 4:
			return 4;
			break;
			default:
			return -1;
			break;
		}
		*/
	}
	else{
		continue;
	}
}
return -1;
}
/*
//function to set the value of the potentiometer when the pedal is released
void set_max(int * val, int * min1, int * max1, int * min2, int * max2){
	&max1 = val[0];
	&max2 = val[1];
}

//function to set the value of the potentiometer when the pedal is pressed
void set_min(int * val, int * min1, int * max1, int * min2, int * max2){
	&min1 = val[0];
	&min2 = val[1];
}
*/

///IMU VARIABLES///
uint8_t ZERO = 0x00;
uint8_t WHO_AM_I_G = 0x8F;
uint8_t WHO_AM_I_G_VAL;
uint8_t WHO_AM_I_XM = 0x8F;
uint8_t WHO_AM_I_XM_VAL;

uint8_t CTRL_REG1_G_ADD = 0x20;
uint8_t CTRL_REG1_G_VAL = 0x0F;
uint8_t CTRL_REG4_G_ADD = 0x23;
uint8_t CTRL_REG4_G_VAL = 0x10;

uint8_t CTRL_REG1_XM_ADD = 0x20;
uint8_t CTRL_REG1_XM_VAL = 0xA7;
uint8_t CTRL_REG2_XM_ADD = 0x21;
uint8_t CTRL_REG2_XM_VAL = 0x08;
uint8_t CTRL_REG5_XM_ADD = 0x24;
uint8_t CTRL_REG5_XM_VAL = 0x70;
uint8_t CTRL_REG6_XM_ADD = 0x25;
uint8_t CTRL_REG6_XM_VAL = 0x20;
uint8_t CTRL_REG7_XM_ADD = 0x26;
uint8_t CTRL_REG7_XM_VAL = 0x00;

uint8_t OUT_X_L_G_ADD = 0xE8;
uint8_t OUT_X_H_G_ADD = 0xE9;
uint8_t OUT_Y_L_G_ADD = 0xEA;
uint8_t OUT_Y_H_G_ADD = 0xEB;
uint8_t OUT_Z_L_G_ADD = 0xEC;
uint8_t OUT_Z_H_G_ADD = 0xED;

uint8_t OUT_X_L_A_ADD = 0xE8;
uint8_t OUT_X_H_A_ADD = 0xE9;
uint8_t OUT_Y_L_A_ADD = 0xEA;
uint8_t OUT_Y_H_A_ADD = 0xEB;
uint8_t OUT_Z_L_A_ADD = 0xEC;
uint8_t OUT_Z_H_A_ADD = 0xED;


//////BITCH THAT'S ALL
