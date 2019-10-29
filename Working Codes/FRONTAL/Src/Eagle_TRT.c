#include "Eagle_TRT.h"
#include "stm32f4xx_hal_conf.h"

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
 *
 *working settings:
 *interrupt timer -> prescaler 36, counter period 1000
 *timer2 -> prescaler = 18, counter period = 65500
 *pin PC8 = data in
 *pin PC9 = clock pin
 *angles_array[15]
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

	extern TIM_HandleTypeDef htim2;
	extern UART_HandleTypeDef huart2;

	///IMU VARIABLES///
	uint8_t ZERO=0x00;
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

	// GYROSCOPE SCALE SETTING
	uint8_t CTRL_REG4_G = 0x23;

	uint8_t SCL_G_245 = 0x00;
	uint8_t SCL_G_500 = 0x10;
	uint8_t SCL_G_1000 = 0x20;
	uint8_t SCL_G_2000 = 0x30;

	// ACCELERORMETER SCALE SETTING
	uint8_t CTRL_REG2_XM = 0x21;

	uint8_t SCL_A_2 = 0x00;
	uint8_t SCL_A_4 = 0x08;
	uint8_t SCL_A_6 = 0x10;
	uint8_t SCL_A_8 = 0x18;
	uint8_t SCL_A_16 = 0x20;

	// MAGNETORMETER SCALE SETTING
	uint8_t CTRL_REG6_XM = 0x25;

	uint8_t SCL_M_2 = 0x00;
	uint8_t SCL_M_4 = 0x20;
	uint8_t SCL_M_8 = 0x40;
	uint8_t SCL_M_12 = 0x60;

	// OUTPUT REG
	uint8_t OUT_X_L_G_ADD = 0xA8;
	uint8_t OUT_X_H_G_ADD = 0xA9;
	uint8_t OUT_Y_L_G_ADD = 0xAA;
	uint8_t OUT_Y_H_G_ADD = 0xAB;
	uint8_t OUT_Z_L_G_ADD = 0xAC;
	uint8_t OUT_Z_H_G_ADD = 0xAD;

	uint8_t OUT_X_L_A_ADD = 0xA8;
	uint8_t OUT_X_H_A_ADD = 0xA9;
	uint8_t OUT_Y_L_A_ADD = 0xAA;
	uint8_t OUT_Y_H_A_ADD = 0xAB;
	uint8_t OUT_Z_L_A_ADD = 0xAC;
	uint8_t OUT_Z_H_A_ADD = 0xAD;

	imu_stc imu;
	can_stc can;

	void send_config(GPIO_TypeDef* pinx, uint16_t pinn, uint8_t * addr, uint8_t * val){
		HAL_GPIO_WritePin(pinx, pinn, GPIO_PIN_RESET); 					///CS_InUse to 0
		htim2.Instance->CNT=0; 																		//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)
		HAL_SPI_Transmit(imu.hspi, addr, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu.hspi, val, 1, 10); ///Writing 0b00001111 to enable PowerMode and x,y,z axis
		htim2.Instance->CNT=0; 																	//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)
		HAL_GPIO_WritePin(pinx, pinn, GPIO_PIN_SET); 					///CS_InUse to 1
	}

	//accelerometer, gyroscope and magnetometer initialization
	//call this function before requesting data from the sensor
	//hspi = pointer to the spi port defined
	void LSMD9S0_accel_gyro_init(imu_stc* imu){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG1_G_ADD, (uint8_t*)&CTRL_REG1_G_VAL);
		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG1_XM_ADD, (uint8_t*)&CTRL_REG1_XM_VAL);

		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG2_XM, (uint8_t*)&SCL_A_4);
		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG4_G, (uint8_t*)&SCL_G_500);

		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG5_XM_ADD, (uint8_t*)&CTRL_REG5_XM_VAL);
		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG6_XM_ADD, (uint8_t*)&CTRL_REG6_XM_VAL);
		send_config(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, (uint8_t*)&CTRL_REG7_XM_ADD, (uint8_t*)&CTRL_REG7_XM_VAL);

		HAL_Delay(1);

		HAL_UART_Transmit(&huart2, (uint8_t*)"<IMU> Initialization -> Done\r\n", 26, 10);
	}

	//this function is used to calibrate the gyroscope
	//hspi = pointer to the spi port defined
	//X_G_axis_offset = gyroscope x axis offset value that changes after executing this function
	//Y_G_axis_offset = gyroscope y axis offset value that changes after executing this function
	//Z_G_axis_offset = gyroscope z axis offset value that changes after executing this function
	/*void LSMD9S0_gyro_calib(imu_stc* imu){

		imu->kp = 0.0175;

		imu->X_G_axis_offset = LSM9DS0_calib(imu);
		imu->Y_G_axis_offset = LSM9DS0_calib(imu);
		imu->Z_G_axis_offset = LSM9DS0_calib(imu);
	}*/

	//this function is used to calibrate the accelerometer
	//hspi = pointer to the spi port defined
	//X_A_axis_offset = accelerometer x axis offset value that changes after executing this function
	//Y_A_axis_offset = accelerometer y axis offset value that changes after executing this function
	//Z_A_axis_offset = accelerometer z axis offset value that changes after executing this function
	/*void LSMD9S0_accel_calib(imu_stc* imu){

		imu->kp = 0.00119782; ///0.000122 * 9,81

		imu->X_A_axis_offset = LSM9DS0_calib(imu);
		imu->Y_A_axis_offset = LSM9DS0_calib(imu);
		imu->Z_A_axis_offset = LSM9DS0_calib(imu);
	}*/

	float LSMD9S0_read(imu_stc* imu){

		uint8_t OUT_L_VAL;
		uint8_t OUT_H_VAL;

		//__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);			//delay of 1 microsecond like from datasheet
		//while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){
		//}

		///READING ROTATION
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); 					///CS_InUse to 0
		htim2.Instance->CNT=0; 																		//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)

		HAL_SPI_Transmit(imu->hspi, &(imu->REG_L), 1, 10); 											///Writing LOW address
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&OUT_L_VAL, 1, 10); 									///Saving LOW data

		htim2.Instance->CNT=0; 																	//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); 					///CS_InUse to 1

		htim2.Instance->CNT=0; 																		//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); 					///CS_InUse to 0

		HAL_SPI_Transmit(imu->hspi, &(imu->REG_H), 1, 10); 											///Writing HIGH address
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&OUT_H_VAL, 1, 10); 									///Saving HIGH data

		htim2.Instance->CNT=0; 																		//set counter to 0
		while(htim2.Instance->CNT<=20){} 															//delay (must be >5ns)

		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); 					///CS_InUse to 1



		///CALCULATING ROTATION
		uint32_t value = (OUT_H_VAL << 8) | OUT_L_VAL;	///Calculating axis value shifting and using a logic OR
		float axis = value;

		//axis = axis * imu->kp; ///Scaling axis value with appropriate conversion factor from datasheet

		/*char imu_str_1[100];
		int imu_val_ret = OUT_H_VAL << 8 | OUT_L_VAL;
		sprintf(imu_str_1,"gyro: %d\n\r",imu_val_ret);
		HAL_UART_Transmit(&huart2, (uint8_t*)imu_str_1, strlen(imu_str_1), 10);*/

		return axis;
	}

	/*float LSM9DS0_calib(imu_stc* imu){

		float axis_cal;
		float sum_cal = 0.0000;

		for(int i = 0; i < 10000; i++){
			float tmp = LSMD9S0_read(imu);
			sum_cal = sum_cal + tmp;
		}
		axis_cal = sum_cal / 10000;

		return axis_cal;
	}*/

	int LSMD9S0_check(imu_stc* imu){

		int check = 0;

		///AXEL/MAGN ARE WORKING
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		htim2.Instance->CNT=0; //set counter to 0
		while(htim2.Instance->CNT<=20){} //delay (must be >5ns)
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&WHO_AM_I_XM, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
		//HAL_SPI_TransmitReceive(imu->hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_XM_VAL, 1, 10); ///Reading from register sending a 0x00
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&WHO_AM_I_XM_VAL, 1, 10);
		htim2.Instance->CNT=0; //set counter to 0
		while(htim2.Instance->CNT<=20){} //delay (must be >5ns)
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		///GYRO IS WORKING
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		htim2.Instance->CNT=0; //set counter to 0
		while(htim2.Instance->CNT<=20){} //delay (must be >5ns)
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&WHO_AM_I_G, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
		//HAL_SPI_TransmitReceive(imu->hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10); ///Reading from register sending a 0x00
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10);
		htim2.Instance->CNT=0; //set counter to 0
		while(htim2.Instance->CNT<=20){} //delay (must be >5ns)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1



		char imu_str[100];

		//sprintf(imu_str,"Test with %d and %d\r\n%d, %d\r\n",WHO_AM_I_G,WHO_AM_I_XM,WHO_AM_I_G_VAL,WHO_AM_I_XM_VAL);
		//HAL_UART_Transmit(&huart2, (uint8_t*)imu_str, strlen(imu_str), 10);

		///AXEL/GYRO STATUS
		if ((WHO_AM_I_G_VAL == 212) & (WHO_AM_I_XM_VAL == 73)){
			check = 1;
			//HAL_UART_Transmit(&huart2, (uint8_t*)"imu int correct\r\n", 17, 10);
		}else{
			check = 0;
			//HAL_UART_Transmit(&huart2, (uint8_t*)"imu int failed\r\n", 16, 10);
		}

		return check;
	}

	void LSM9DS0_calibration(imu_stc*imu){
		long int x = 0, y = 0, z = 0;
		int iterations = 0;
		for (int i = 0; i < iterations; i++){
			LSMD9S0_accel_read(imu);
			x += imu->X_A_axis;
			y += imu->Y_A_axis;
			z += imu->Z_A_axis;
			HAL_Delay(1);
		}

		imu->X_A_axis_offset = x / iterations;
		imu->Y_A_axis_offset = y / iterations;
		imu->Z_A_axis_offset = z / iterations;

		imu->calibration_done = 1;
	}

	//Reading G_axis values
	//hspi = pointer to the spi port defined
	//X_G_axis = pointer gyroscope x variable
	//Y_G_axis = pointer gyroscope y variable
	//Z_G_axis = pointer gyroscope z variable
	//X_G_axis_offset = offset x value
	//Y_G_axis_offset = offset y value
	//Z_G_axis_offset = offset z value
	void LSMD9S0_gyro_read(imu_stc* imu){

		imu->kp = 0.0175;

		imu->GPIOx_InUse=GPIOA;
		imu->GPIO_Pin_InUse=GPIO_PIN_8;

		imu->REG_H = OUT_X_H_G_ADD;
		imu->REG_L = OUT_X_L_G_ADD;
		imu->X_G_axis = LSMD9S0_read(imu);
		//imu->X_G_axis = imu->X_G_axis - imu->X_G_axis_offset;

		imu->REG_H = OUT_Y_H_G_ADD;
		imu->REG_L = OUT_Y_L_G_ADD;
		imu->Y_G_axis = LSMD9S0_read(imu);
		//imu->Y_G_axis = imu->Y_G_axis - imu->Y_G_axis_offset;

		imu->REG_H = OUT_Z_H_G_ADD;
		imu->REG_L = OUT_Z_L_G_ADD;
		imu->Z_G_axis = LSMD9S0_read(imu);
		//imu->Z_G_axis = imu->Z_G_axis - imu->Z_G_axis_offset;

		///AXEL/GYRO STATUS
		if ((WHO_AM_I_G_VAL == 212) & (WHO_AM_I_XM_VAL == 73)){
			imu->error_flag = 0;
		}else{
			imu->error_flag = 1;
		}

		/*int16_t val_g_x = imu->Y_G_axis * 100;
		int16_t val_g_y = (0 - imu->X_G_axis) * 100;
		int16_t val_g_z = imu->Z_G_axis * 100;*/

		if(imu->X_G_axis > 32768){
			imu->X_G_axis -= 65536;
		}
		if(imu->Y_G_axis > 32768){
			imu->Y_G_axis -= 65536;
		}
		if(imu->Z_G_axis > 32768){
			imu->Z_G_axis -= 65536;
		}


		shift_array(imu->X_G_axis_array, 10, imu->X_G_axis);
		shift_array(imu->Y_G_axis_array, 10, imu->Y_G_axis);
		shift_array(imu->Z_G_axis_array, 10, imu->Z_G_axis);

		imu->X_G_axis = dynamic_average(imu->X_G_axis_array, 10);
		imu->Y_G_axis = dynamic_average(imu->Y_G_axis_array, 10);
		imu->Z_G_axis = dynamic_average(imu->Z_G_axis_array, 10);

		if(imu->X_G_axis >= 0){
			imu->x_g_sign = 0;
		}
		else{
			imu->x_g_sign = 1;
			imu->X_G_axis *= -1;
		}
		if(imu->Y_G_axis >= 0){
			imu->y_g_sign = 0;
		}
		else{
			imu->y_g_sign = 1;
			imu->Y_G_axis *= -1;
		}
		if(imu->Z_G_axis >= 0){
			imu->z_g_sign = 0;
		}
		else{
			imu->z_g_sign = 1;
			imu->Z_G_axis *= -1;
		}

		/*char imu_str_1[100];

		sprintf(imu_str_1,"gyro: %d %d %d\n\r", (int)imu->X_G_axis, (int)imu->Y_G_axis, (int)imu->Z_G_axis);
		HAL_UART_Transmit(&huart2, (uint8_t*)imu_str_1, strlen(imu_str_1), 10);*/
	}

	///Reading A_axis values
	//hspi = pointer to the spi port defined
	//X_A_axis = pointer accelerometer x variable
	//Y_A_axis = pointer accelerometer y variable
	//Z_A_axis = pointer accelerometer z variable
	//X_A_axis_offset = offset x value
	//Y_A_axis_offset = offset y value
	//Z_A_axis_offset = offset z value
	void LSMD9S0_accel_read(imu_stc* imu){

		imu->kp= 0.00119782; ///0.000122 * 9,81

		imu->GPIOx_InUse=GPIOC;
		imu->GPIO_Pin_InUse=GPIO_PIN_9;

		imu->REG_H = OUT_X_H_A_ADD;
		imu->REG_L = OUT_X_L_A_ADD;
		imu->X_A_axis = LSMD9S0_read(imu);

		//imu->X_A_axis = imu->X_A_axis - imu->X_A_axis_offset;

		imu->REG_H = OUT_Y_H_A_ADD;
		imu->REG_L = OUT_Y_L_A_ADD;
		imu->Y_A_axis = LSMD9S0_read(imu);
		//imu->Y_A_axis = imu->Y_A_axis - imu->Y_A_axis_offset;

		imu->REG_H = OUT_Z_H_A_ADD;
		imu->REG_L = OUT_Z_L_A_ADD;
		imu->Z_A_axis = LSMD9S0_read(imu);
		//imu->Z_A_axis = imu->Z_A_axis - imu->Z_A_axis_offset + 9.81;

		if(imu->calibration_done){

			imu->X_A_axis -= imu->X_A_axis_offset;
			imu->Y_A_axis -= imu->Y_A_axis_offset;
			imu->Z_A_axis -= imu->Z_A_axis_offset;

			if(imu->X_A_axis > 32768){
				imu->X_A_axis -= 65536;
			}
			if(imu->Y_A_axis > 32768){
				imu->Y_A_axis -= 65536;
			}
			if(imu->Z_A_axis > 32768){
				imu->Z_A_axis -= 65536;
			}
/*
			shift_array(imu->X_A_axis_array, 10, imu->X_A_axis);
			shift_array(imu->Y_A_axis_array, 10, imu->Y_A_axis);
			shift_array(imu->Z_A_axis_array, 10, imu->Z_A_axis);

			imu->X_A_axis = dynamic_average(imu->X_A_axis_array, 10);
			imu->Y_A_axis = dynamic_average(imu->Y_A_axis_array, 10);
			imu->Z_A_axis = dynamic_average(imu->Z_A_axis_array, 10);*/
		}
/*
		if(imu->X_A_axis >= 0){
			imu->x_a_sign = 0;
		}
		else{
			imu->x_a_sign = 1;
			imu->X_A_axis *= -1;
		}
		if(imu->Y_A_axis >= 0){
			imu->y_a_sign = 0;
		}
		else{
			imu->y_a_sign = 1;
			imu->Y_A_axis *= -1;
		}
		if(imu->Z_A_axis >= 0){
			imu->z_a_sign = 0;
		}
		else{
			imu->z_a_sign = 1;
			imu->Z_A_axis *= -1;
		}*/

	}

#endif


#ifdef HAL_CAN_MODULE_ENABLED
#include "stm32f4xx_hal_can.h"
	//function that sends an array via CAN
	//hcan = pointer to can port
	//id = id of the message to be sent
	//dataTx = pointer to array that contains the data to be sent
	//size = size of the array
	can_stc can;
	int CAN_Send(can_stc* can){

		uint32_t mailbox;
		uint8_t flag = 0;

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = can->id;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;
		TxHeader.DLC = can->size;
		TxHeader.TransmitGlobalTime = DISABLE;

		if (HAL_CAN_GetTxMailboxesFreeLevel(can->hcan) != 0 && HAL_CAN_IsTxMessagePending(can->hcan, CAN_TX_MAILBOX0) == 0){
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
	int CAN_Receive(can_stc* can){

		CAN_RxHeaderTypeDef RxHeader;

		if (HAL_CAN_GetRxFifoFillLevel(can->hcan, CAN_RX_FIFO0) != 0){
			HAL_CAN_GetRxMessage(can->hcan, CAN_RX_FIFO0, &RxHeader, can->dataRx);
		}

		int id = RxHeader.StdId;

		return id;
	}
#endif

#ifdef HAL_UART_MODULE_ENABLED
	#include "stm32f4xx_hal_uart.h"
	#include "malloc.h"
	///---- queue ---- ///
	static int next(int ret, int dim){

	  return (ret+1)%dim;
	}

	// Implementazione dinamica
	void init(queue * q){

	  q->tail=q->head=0;
	  q->dim=40;
	}

	static int emptyp(const queue * q){

	  return (q->tail==q->head);
	}

	static int fullp(const queue * q){

	  return (next(q->tail,q->dim)==q->head);
	}

	int push (char * str,queue * q){
		int res;
		if (fullp(q)){
			res = FAIL;
		}
		else{
			int length=strlen(str);
			q->elem[q->tail] = (char*) malloc(sizeof(char)*length);
			strcpy(q->elem[q->tail],str);
			//q->tail = next(q->tail,q->dim);
			if(q->tail==39){
				q->tail=0;
			}
			else{
				q->tail++;
			}
			res=OK;
		}

		return res;
	}

	int pop(char * str,queue * q){
		int res;
		if (emptyp(q)){
			res = FAIL;
		}
		else {
			strcpy(str,q->elem[q->head]);
			free(q->elem[q->head]);
			//q->head = next(q->head,q->dim);
			if(q->head==39){
				q->head=0;
			}else{
				q->head++;
			}
			res=OK;
		}

		return res;
	}
	/// ---- end queue ----///
	queue print_q={.head=0,.tail=0,.dim=0};

	int print(UART_HandleTypeDef *huart,char * text_print_function){

		int ret=0;

		if(HAL_UART_Transmit_IT(huart, (uint8_t*)text_print_function, strlen(text_print_function))==HAL_OK){
			ret=1;
		}else{
			if(push(text_print_function,&print_q)==FAIL){
				ret=0;
			}else{
				ret=1;
			}
		}

		return ret;
	}

	void print_it(UART_HandleTypeDef *huart){ //put in the uart interrupt

		char text_print_function[50];

		if(pop(text_print_function,&print_q)==OK){
			HAL_UART_Transmit_IT(huart, (uint8_t*)text_print_function, strlen(text_print_function));
		}
	}

	UART_HandleTypeDef* huart_GPS;
	int start_string_gps=0;
	char string_gps[100];
	int cont_string,cont_comma;
	char data_string_gps;
	char buffer_gps[2];
	static int checksum(char * string_checksum, int size_string_checksum);

	/* GPS library
	gps_init() ->initialize the GPS. Put it in the main initialization. Example:
	gps_struct gps_main; //define the name of gps_structure istance
	if(gps_init(&huart3,&gps_main)==0){
		/--error--/
	}
	gps_read_it() -> put it in interrupt. Example:
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		gps_read_it(huart,&gps_main);
	}*/

	int gps_init(UART_HandleTypeDef* huart,gps_struct * gps){ //initialization of GPS

		//if return--> 0=error,1=ok
		huart_GPS=huart;
		huart_GPS->Init.BaudRate = 9600;
		HAL_UART_Init(huart_GPS);
		HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
		HAL_Delay(500);
		huart_GPS->Init.BaudRate = 57600;
		HAL_UART_Init(huart_GPS);
		HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
		HAL_Delay(500);
		huart_GPS->Init.BaudRate = 115200;
		HAL_UART_Init(huart_GPS);
		HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_BAUD_115200, strlen(PMTK_SET_BAUD_115200), 200);
		HAL_Delay(500);
		HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_NMEA_UPDATE_10HZ, strlen(PMTK_SET_NMEA_UPDATE_10HZ), 200);
		HAL_Delay(500);
		HAL_UART_Transmit(huart_GPS, (uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAVTG, strlen(PMTK_SET_NMEA_OUTPUT_GGAVTG), 200);
		HAL_Delay(500);
		strcpy(gps->speed,"000.00");
		strcpy(gps->latitude,"0000.0000");
		strcpy(gps->latitude_o,"N");
		strcpy(gps->longitude,"00000.0000");
		strcpy(gps->longitude_o,"W");
		strcpy(gps->altitude,"0000.0");
		strcpy(gps->time,"000000");
		HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1); //request of rx buffer interrupt

		return 1;
	}

	int gps_read_it(UART_HandleTypeDef *huart, gps_struct* gps){

			int ret=0; //return--> 0=error,1=ok

			/*
			* Example of strings
			* $GPGGA,064951.000,2307.1256,N,12016.4438,E,1,8,0.95,39.9,M,17.8,M,,*65
			* $GPGSA,A,3,29,21,26,15,18,09,06,10,,,,,2.32,0.95,2.11*00
			* $GPGSV,3,1,09,29,36,029,42,21,46,314,43,26,44,020,43,15,21,321,39*7D
			  $GPGSV,3,2,09,18,26,314,40,09,57,170,44,06,20,229,37,10,26,084,37*77
			  $GPGSV,3,3,09,07,,,26*73
			* $GPRMC,064951.000,A,2307.1256,N,12016.4438,E,0.03,165.48,260406,3.05,W,A*2C
			* $GPVTG,165.48,T,,M,0.03,N,0.06,K,A*37
			* $PGTOP,11,3 *6F
			*
			*
			*
			* 	$GPBOD - Bearing, origin to destination
				$GPBWC - Bearing and distance to waypoint, great circle
				$GPGGA - Global Positioning System Fix Data
				$GPGLL - Geographic position, latitude / longitude
				$GPGSA - GPS DOP and active satellites
				$GPGSV - GPS Satellites in view
				$GPHDT - Heading, True
				$GPR00 - List of waypoints in currently active route
				$GPRMA - Recommended minimum specific Loran-C data
				$GPRMB - Recommended minimum navigation info
				$GPRMC - Recommended minimum specific GPS/Transit data
				$GPRTE - Routes
				$GPTRF - Transit Fix Data
				$GPSTN - Multiple Data ID
				$GPVBW - Dual Ground / Water Speed
				$GPVTG - Track made good and ground speed
				$GPWPL - Waypoint location
				$GPXTE - Cross-track error, Measured
				$GPZDA - Date & Time
				http://aprs.gids.nl/nmea/
			*/
			if(huart==huart_GPS){
				//check if it's the huart_gps interrupt
				HAL_UART_Receive_IT(huart_GPS, (uint8_t *)buffer_gps, 1); //request interrupt for the next data
				data_string_gps=buffer_gps[0]; //convert a pointer into a char
				if((start_string_gps==1)&&(data_string_gps!='$')){ //check that the new string has not started yet
					string_gps[cont_string]=data_string_gps; //save the data into the array
					cont_string++;
					if(string_gps[cont_string-1]=='\r'||string_gps[cont_string-1]=='\n'){  //indicates that the string is finishing
						cont_string--;
						string_gps[cont_string]='\0'; // '\0'=end of the string
						start_string_gps=0; //end of string
						if(string_gps[2]=='G'&&string_gps[3]=='G'&&string_gps[4]=='A'){ // operation when the string is GPGGA //
							if(checksum(string_gps,cont_string)==1){ //check the checksum (if==true -> enter)
								int cont_comma=0,cont_latitude=0,cont_longitude=0,cont_altitude=0,cont_time=0;
								for(int i=5;i<100;i++){
									if(string_gps[i]==',')cont_comma++;
									else{
										if(cont_comma==1){ //save the time
											gps->time[cont_time]=string_gps[i];
											cont_time++;
										}
										else if(cont_comma==2){ //save latitude
											gps->latitude[cont_latitude]=string_gps[i];
											cont_latitude++;
										}
										else if(cont_comma==3){ //save orientation of latitude
											gps->latitude_o[0]=string_gps[i];
										}
										else if(cont_comma==4){ //save longitude
											gps->longitude[cont_longitude]=string_gps[i];
											cont_longitude++;
										}
										else if(cont_comma==5){ //save orientation of longitude
											gps->longitude_o[0]=string_gps[i];
										}
										else if (cont_comma==6){
											gps->fix_status=string_gps[i];
										}
										else if(cont_comma==9){ //save altitude
											gps->altitude[cont_altitude]=string_gps[i];
											cont_altitude++;
										}
										else if(cont_comma==10){
											i=100; //end the cicle
										}
									}

								}
								//-- operation to split data and send them --//
								if(gps->fix_status=='0'){
									gps->latitude_i_h=0;
									gps->latitude_i_l=0;
									gps->longitude_i_h=0;
									gps->longitude_i_l=0;
									gps->altitude_i=0;
								}
								else{
									gps->latitude_i=(long int)(atof(gps->latitude)*10000);
									gps->longitude_i=(long int)(atof(gps->longitude)*100000);
									gps->altitude_i=(int)(atof(gps->altitude)*100);
									gps->latitude_i_h=(int)(gps->latitude_i/10000);
									gps->latitude_i_l=(int)(gps->latitude_i-gps->latitude_i_h*10000);
									gps->longitude_i_h=(int)(gps->longitude_i/100000);
									gps->longitude_i_l=(int)(gps->longitude_i-gps->longitude_i_h*100000);
								}

								can.dataTx[0] = 0x08;
								can.dataTx[1] = gps->longitude_i_h / 256;
								can.dataTx[2] = gps->longitude_i_h % 256;
								can.dataTx[3] = gps->longitude_i_l / 256;
								can.dataTx[4] = gps->longitude_i_l % 256;
								can.dataTx[5] = (int)gps->longitude_o;
								can.dataTx[6] = gps->altitude_i / 256;
								can.dataTx[7] = gps->altitude_i % 256;
								can.id = 0xD0;
								can.size = 8;
								CAN_Send(&can);
							}
							else{
								ret=0; //checksum failed
							}
						}
						else if(string_gps[2]=='V'&&string_gps[3]=='T'&&string_gps[4]=='G'){ 	// operation when the string is GPVTG //
							if(checksum(string_gps,cont_string)==1){ //check the checksum (if==true -> enter)
								cont_comma=0;
								int cont_speed=0;
								for(int i=5;i<cont_string;i++){
									if(string_gps[i]==',')cont_comma++;
									else{
										if(cont_comma==7){ //save the speed
											gps->speed[cont_speed]=string_gps[i];
											cont_speed++;
										}
										else if(cont_comma==8){
											i=cont_string;
										}
									}
								}
								//-- operation to split data and send them --//
								if(gps->fix_status=='0'){
									gps->speed_i=0;
								}else{
									gps->speed_i=(int)(atof(gps->speed)*100);
								}
								can.dataTx[0] = 0x07;
								can.dataTx[1] = gps->latitude_i_h / 256;
								can.dataTx[2] = gps->latitude_i_h % 256;
								can.dataTx[3] = gps->latitude_i_l / 256;
								can.dataTx[4] = gps->latitude_i_l % 256;
								can.dataTx[5] = (int)gps->latitude_o;
								can.dataTx[6] = gps->speed_i / 256;
								can.dataTx[7] = gps->speed_i % 256;
								can.id = 0xD0;
								can.size = 8;
								CAN_Send(&can);
								ret=1;
							}
							else{
								ret=0;  //checksum failed
							}
						}
					}
				}
				else{
					if(data_string_gps=='$'){ //check if data indicates the start of new string
						start_string_gps=1; //new string started
						cont_string=0; //set the counter to 1
					}
				}


			}
			return ret;
		}
	static int checksum(char * string_checksum, int size_string_checksum){ //check the checksum
		//return 1;

		int res=0;
		int offset_maiusc=(int)('A')-(int)('a');
		int i=0;

		for(i=0;(i<size_string_checksum)&&(string_checksum[i]!='*');i++){
			res=res^string_checksum[i];
		}
		char check[2]={string_checksum[i+1],string_checksum[i+2]};
		char res_char[3];
		sprintf(res_char,"%x",res);
		if(res<17){
			res_char[1]=res_char[0];
			res_char[0]='0';
		}
		for(int j=0;j<2;j++){ //convert to upper case letter
			if((int)res_char[j]>='a'&&(int)res_char[j]<='f'){
				res_char[j]=(char)((int)res_char[j]+offset_maiusc);
			}
		}
		if(res_char[0]==check[0]&&res_char[1]==check[1]){
			return 1; //checksum is correct
		}
		else {
			return 0; //checksum failed
		}
	}

#endif

#ifdef HAL_TIM_MODULE_ENABLED
#include "stm32f4xx_hal_tim.h"

	extern UART_HandleTypeDef huart2;
	extern char txt;

	// Function to request data from encoder via SSI communication
	// This function is called from the interrupt callback of the timer that you are using for the encoder
	// The tim used for this function must be initialized at most at 2 microsecond per tick
	// Lower the number of microseconds per tick better it is
	// TimerInstance = struct of the tim used for the encoder
	enc_stc enc;
	double read_encoder(enc_stc *enc){

		enc->clock_period = 2;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}


		// Starting the clock to retrieve 14 bits from the sensor
		for (int i = 0; i < enc->data_size; i++){

			// CLOCK HIGH
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
			while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

			// Set the bit as the pin state (0 or 1)
			enc->Data[i] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

			// CLOCK LOW
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
			__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
			while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}
		}

		// Requesting an other bit for the aventual error sent from the sensor
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);
		while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

		enc->error_flag = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);

		// Converting bits into number and converting it into angle in degrees (0 ~ 359)
		enc->converted_data = bin_dec(enc->Data, enc->data_size);
		enc->converted_data = enc->converted_data / 45.5055;

		return enc->converted_data;
	}

	// Interrupt function of tim 2
	// Call this function in the timer callback function of the stm
	// htim = timer TimerInstance of the timer that you are using for the clock of the encoder
	// Interrupt_flag = initilize a int variable in the main file
	// Angles_array = array to store the last angles
	// Speed = pointer to the speed value
	void encoder_tim_interrupt(enc_stc* enc){


		if(enc->interrupt_flag == 0){
			// Requesting first angle
			enc->angle0_prec = enc->angle0;
			enc->angle0 = read_encoder(enc);
		}
		else if(enc->interrupt_flag == 1){
			// Requesting second angle
			enc->angle1_prec = enc->angle1;
			enc->angle1 = read_encoder(enc);
		}
		else if(enc->interrupt_flag == 2){
			// Calculate speed from the two angles
			get_speed_encoder(enc);

			enc->average_speed *= 10;

			// Get the speed sign to be sent in CAN
			if(enc->average_speed < 0){
				enc->average_speed *= -1;
				enc->speed_sign = 1;
			}else{
				enc->speed_sign = 0;
			}
		}

		// Cycle between steps
		if(enc->interrupt_flag >= 2){
			enc->interrupt_flag = 0;
		}
		else{
			enc->interrupt_flag ++;
		}
	}

	// Funtion to calculate the speed
	// Angle0 = last angle calculated
	// Angle1 = previous angle calculated
	// Refresh = delta-time from the two calculations, express it in microseconds
	// Wheel_diameter = diameter of the wheel expressed meters
	void get_speed_encoder(enc_stc* enc){

		long double speed = 0;
		double dt = 0;
		double d_angle;

		dt = enc->samle_delta_time;

		enc->angle0 *= 1000;
		enc->angle1 *= 1000;

		if(enc->dx_wheel == 1){
			d_angle = enc->angle1 - enc->angle0;
		}
		else{
			d_angle = enc->angle0 - enc->angle1;
		}

		// Calculating rad/s, then m/s, then Km/h
		speed = (d_angle/360)*3.1415*(enc->wheel_diameter);
		speed *= 3.6;
		speed /= dt;
		speed = round((speed*10000))/100000;

		enc->angle0 /= 1000;
		enc->angle1 /= 1000;

		// Start detecting eventual new wheel roation
		// If the speed is too low, don't count rotations
		if(enc->average_speed < -0.5 || enc->average_speed > 0.5){
			if((enc->angle0_prec <= 361 && enc->angle0_prec > 350) && (enc->angle0 >= -1 && enc->angle0 < 10)){
				enc->wheel_rotation ++;
				enc->Km += (3.14 * enc->wheel_diameter)/1000;
			}
			if((enc->angle0_prec >= -1 && enc->angle0_prec < 10) && (enc->angle0 <= 361 && enc->angle0 > 350)){
				enc->wheel_rotation ++;
				enc->Km += (3.14 * enc->wheel_diameter)/1000;
			}
		}

		// Don't use the speed if the two samples are near to 0/360
		if((enc->angle0 < 40 && enc->angle1 > 320) || (enc->angle1 < 40 && enc->angle0 > 320)){

		}
		else{
			shift_array(enc->speed_array, 50, speed);
			enc->average_speed = dynamic_average(enc->speed_array, 50);
		}
	}

	pot_stc pot_1;
	pot_stc pot_2;
	pot_stc pot_3;
	void calc_pot_value(pot_stc *pot) {

		pot->val_100 = round(100 - (abs(pot->val - pot->min) * 100 / (pot->range))); //val0_100 -->STEER --> 0 = SX | 100 = DX
		if (pot->val <= pot->min) {
			pot->val_100 = 100;
		}
		if (pot->val >= pot->max) {
			pot->val_100 = 0;
		}
	}

	//Function to check if the two ADC values are approximately the same
	//if the values are different for more tha 10 points percentage for more than 100 milliseconds returns the SCS Error
	//TimerInstance = pointer to the timer needed to check the SCS error
	//val0_100 = pointer to the first potentiometer
	//val1_100 = pointer to the second potentiometer
	int implausibility_check(pot_stc * pot_1, pot_stc * pot_2){

		int SCS1 = 0;

		if (pot_1->val_100 >= 100){
			pot_1->val_100 = 100;
		}
		if (pot_1->val_100 <= 5){
			pot_1->val_100 = 0;
		}
		if (pot_2->val_100 >= 100){
			pot_2->val_100 = 100;
		}
		if (pot_2->val_100 <= 5){
			pot_2->val_100 = 0;
		}
		if(abs(pot_1->val_100 - pot_2->val_100) >= 10){
			if(__HAL_TIM_GET_COUNTER(pot_1->TimerInstance) > 100){
				SCS1 = 1;
			}
		}
		else{
			__HAL_TIM_SET_COUNTER(pot_1->TimerInstance, 0);
			SCS1 = 0;
		}

		return SCS1;
	}

	//function to set the value of the potentiometer when the pedal is released
	//val = array pointer to the potentiometer values
	//max1 = pointer to the maximum value of the APPS1
	//max2 = pointer to the maximum value of the APPS2
	void set_max(pot_stc *pot_1){
		pot_1->max = pot_1->val;
	}

	//function to set the value of the potentiometer when the pedal is pressed
	//val = array pointer to the potentiometer values
	//min1 = pointer to the minimum value of the APPS1
	//min2 = pointer to the minimum value of the APPS2
	void set_min(pot_stc *pot_1){
		pot_1->min = pot_1->val;
	}

#endif

//function to calculate the decimal value from MSB binary array
//bin = pointer to binary array
//max = size of the array
int bin_dec(int* bin, int size){

	int dec = 0;

	for(int i = 0; i < size; i++){
		if(bin[i] == 1){
			dec += Power(2, size-i-1);
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
void shift_array(long double *array, int size, long double data){

	for(int i = 1; i < size; i++){
		array[i-1] = array[i];
	}
	array[size-1] = data;
}

double speed_filter(double * data, int size){
	double min = 100000000000000;
	double max = -min;
	double sum = 0;
	double average = 0;
	int index_1;
	int index_2;
	int average_members = 0;

	for(int i = 0; i < size; i++){
		if(data[i] < min){
			min = data[i];
			index_1 = i;
		}

		if(data[i] > max){
			max = data[i];
			index_2 = i;
		}
	}

	for(int i = 0; i < size; i++){
		if(i != index_1 || i != index_2){
			sum += data[i];
			average_members ++;
		}
	}
	average = sum / average_members;

	return average;

}

//function that calculate the average of all the numbers in one array
double dynamic_average(long double *array, int size){

	double sum = 0;
	double average = 0;

	for(int i = 0; i < size; i++){
		sum += array[i];
	}
	average = sum / size;

	return average;
}
