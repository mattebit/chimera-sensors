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

	imu_stc imu;
	can_stc can;
	//gyro initialization function
	//call this function before requesting data from the sensor
	//hspi = pointer to the spi port defined
	void LSMD9S0_gyro_init(imu_stc* imu){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG1_G_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG1_G_VAL, 1, 10); ///Writing 0b00001111 to enable PowerMode and x,y,z axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG4_G_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG4_G_VAL, 1, 10); ///Writing 0b00010000 to set full-scale selection to 500dps
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
	}

	//accelerometer and magnetometer initialization
	//call this function before requesting data from the sensor
	//hspi = pointer to the spi port defined
	void LSMD9S0_gyro_accel_init(imu_stc* imu){

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG1_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG1_XM_VAL, 1, 10); ///Writing 0b10100111 to enable 1600Hz and x,y,z axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG2_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG2_XM_VAL, 1, 10); ///Writing 0b00001000 to set +/-4g range for axel axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG5_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG5_XM_VAL, 1, 10); ///Writing 0b01110000 to set high resolution for magn and 50Hz
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG6_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG6_XM_VAL, 1, 10); ///Writing 0b00100000 to set +/-4 gauss range for magn axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG7_XM_ADD, 1, 10); ///Writing the address
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&CTRL_REG7_XM_VAL, 1, 10); ///Writing 0b00000000 to set continuos conversion for magn axis
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET); ///CS_XM to 1
	}

	//this function is used to calibrate the gyroscope
	//hspi = pointer to the spi port defined
	//X_G_axis_offset = gyroscope x axis offset value that changes after executing this function
	//Y_G_axis_offset = gyroscope y axis offset value that changes after executing this function
	//Z_G_axis_offset = gyroscope z axis offset value that changes after executing this function
	void LSMD9S0_gyro_calib(imu_stc* imu){

		imu->kp = 0.0175;

		imu->X_G_axis_offset = LSM9DS0_calib(imu);
		imu->Y_G_axis_offset = LSM9DS0_calib(imu);
		imu->Z_G_axis_offset = LSM9DS0_calib(imu);
	}

	//this function is used to calibrate the accelerometer
	//hspi = pointer to the spi port defined
	//X_A_axis_offset = accelerometer x axis offset value that changes after executing this function
	//Y_A_axis_offset = accelerometer y axis offset value that changes after executing this function
	//Z_A_axis_offset = accelerometer z axis offset value that changes after executing this function
	void LSMD9S0_accel_calib(imu_stc* imu){

		imu->kp = 0.00119782; ///0.000122 * 9,81

		imu->X_A_axis_offset = LSM9DS0_calib(imu);
		imu->Y_A_axis_offset = LSM9DS0_calib(imu);
		imu->Z_A_axis_offset = LSM9DS0_calib(imu);
	}

	float LSMD9S0_read(imu_stc* imu){

		uint8_t OUT_L_VAL;
		uint8_t OUT_H_VAL;

		///READING ROTATION
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
		HAL_GPIO_WritePin(imu->GPIOx_NotInUse, imu->GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
		HAL_SPI_Transmit(imu->hspi, &(imu->REG_L), 1, 10); ///Writing LOW address
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&OUT_L_VAL, 1, 10); ///Saving LOW data
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
		HAL_GPIO_WritePin(imu->GPIOx_NotInUse, imu->GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0

		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_RESET); ///CS_InUse to 0
		HAL_GPIO_WritePin(imu->GPIOx_NotInUse, imu->GPIO_Pin_NotInUse, GPIO_PIN_SET); ///CS_NotInUse to 1
		HAL_SPI_Transmit(imu->hspi, &(imu->REG_H), 1, 10); ///Writing HIGH address
		HAL_SPI_Receive(imu->hspi, (uint8_t*)&OUT_H_VAL, 1, 10); ///Saving HIGH data
		HAL_GPIO_WritePin(imu->GPIOx_InUse, imu->GPIO_Pin_InUse, GPIO_PIN_SET); ///CS_InUse to 1
		HAL_GPIO_WritePin(imu->GPIOx_NotInUse, imu->GPIO_Pin_NotInUse, GPIO_PIN_RESET); ///CS_NotInUse to 0

		///CALCULATING ROTATION
		float axis = OUT_H_VAL << 8 | OUT_L_VAL;	///Calculating axis value shifting and using a logic OR
		if (axis > 32767){ ///Generating positive and negative value of rotation
			axis = axis - 65536;
		}
		axis = axis * imu->kp; ///Scaling axis value with appropriate conversion factor from datasheet

		return axis;
	}

	float LSM9DS0_calib(imu_stc* imu){

		float axis_cal;
		float sum_cal = 0.0000;

		for(int i = 0; i < 10000; i++){
			float tmp = LSMD9S0_read(imu);
			sum_cal = sum_cal + tmp;
		}
		axis_cal = sum_cal / 10000;

		return axis_cal;
	}

	int LSMD9S0_check(imu_stc* imu){

		int check = 0;

		///GYRO IS WORKING
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); ///CS_G to 0
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); ///CS_XM to 1
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&WHO_AM_I_G, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_G (giving by &variable)
		HAL_SPI_TransmitReceive(imu->hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_G_VAL, 1, 10); ///Reading from register sending a 0x00
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0

		///AXEL/MAGN ARE WORKING
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET); ///CS_G to 1
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); ///CS_XM to 0
		HAL_SPI_Transmit(imu->hspi, (uint8_t*)&WHO_AM_I_XM, 1, 10); ///Writing on register ----> (uint8_t*) it's the cast of the pointer to WHO_AM_I_XM (giving by &variable)
		HAL_SPI_TransmitReceive(imu->hspi, (uint8_t*)&ZERO, (uint8_t*)&WHO_AM_I_XM_VAL, 1, 10); ///Reading from register sending a 0x00
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

		imu->X_G_axis = LSMD9S0_read(imu);
		imu->X_G_axis = imu->X_G_axis - imu->X_G_axis_offset;
		imu->Y_G_axis = LSMD9S0_read(imu);
		imu->Y_G_axis = imu->Y_G_axis - imu->Y_G_axis_offset;
		imu->Z_G_axis = LSMD9S0_read(imu);
		imu->Z_G_axis = imu->Z_G_axis - imu->Z_G_axis_offset;

		int16_t val_g_x = imu->Y_G_axis * 100;
		int16_t val_g_y = (0 - imu->X_G_axis) * 100;
		int16_t val_g_z = imu->Z_G_axis * 100;

		can.dataTx[0] = 0x04;
		can.dataTx[1] = val_g_x / 256;
		can.dataTx[2] = val_g_x % 256;
		can.dataTx[3] = val_g_y / 256;
		can.dataTx[4] = val_g_y % 256;
		can.dataTx[5] = val_g_z / 256;
		can.dataTx[6] = val_g_z % 256;
		can.dataTx[7] = 0;
		can.id = 0xC0;
		can.size = 8;
		CAN_Send(&can);
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

		imu->X_A_axis = LSMD9S0_read(imu);
		imu->X_A_axis = imu->X_A_axis - imu->X_A_axis_offset;
		imu->Y_A_axis = LSMD9S0_read(imu);
		imu->Y_A_axis = imu->Y_A_axis - imu->Y_A_axis_offset;
		imu->Z_A_axis = LSMD9S0_read(imu);
		imu->Z_A_axis = imu->Z_A_axis - imu->Z_A_axis_offset + 9.81;

		int16_t val_a_x = (0 - imu->Y_A_axis) * 100;
		int16_t val_a_y = imu->X_A_axis * 100;
		int16_t val_a_z = imu->Z_A_axis * 100;

		can.dataTx[0] = 0x05;
		can.dataTx[1] = val_a_x / 256;
		can.dataTx[2] = val_a_x % 256;
		can.dataTx[3] = val_a_y / 256;
		can.dataTx[4] = val_a_y % 256;
		can.dataTx[5] = val_a_z / 256;
		can.dataTx[6] = val_a_z % 256;
		can.dataTx[7] = 0;
		can.id = 0xC0;
		can.size = 8;
		CAN_Send(&can);
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

	//function to request data from encoder via SSI communication
	//this function is called from the interrupt callback of the timer that you are using for the encoder
	//the tim used for this function must be initialized at most at 2 microsecond per tick
	//lower the number of microseconds per tick better it is
	//TimerInstance = struct of the tim used for the encoder
	enc_stc enc;
	double read_encoder(enc_stc *enc){

		enc->clock_period = 2;

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	//clock was high: reset to low
		__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);			//delay of 1 microsecond like from datasheet
		while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){
		}

		for(int i = 0; i <= enc->data_size; i++){

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//clock set to high to request the bit
			__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);			//delay of 1 microsecond like from datasheet
			while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

			if(i < enc->data_size){
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){	//reading the data input
					enc->Data[i] = 1;
				}
				else{
					enc->Data[i] = 0;
				}

				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
				__HAL_TIM_SET_COUNTER(enc->TimerInstance, 0);					//delay of anothe 1 micros like from datasheet
				while(__HAL_TIM_GET_COUNTER(enc->TimerInstance) <= enc->clock_period){}

			}
			else{
				if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET){
					enc->error_flag = 1;
				}
				else{
					enc->error_flag = 0;
				}

				enc->converted_data = bin_dec(enc->Data, enc->data_size);
				enc->converted_data = enc->converted_data / 45.5055;							//conversions from raw data to angle
				enc->converted_data /= 2;
				//sprintf(txt, "%d", (int)(enc->converted_data * 100));
				//HAL_UART_Transmit(&huart2, (uint8_t*)txt, strlen(txt), 10);
			}
		}

		return enc->converted_data;
	}

	//interrupt function of tim 2
	//call this function in the timer callback function of the stm
	//htim = timer TimerInstance of the timer that you are using for the clock of the encoder
	//interrupt_flag = initilize a int variable in the main file
	//angles_array = array to store the last angles
	//speed = pointer to the speed value
	void encoder_tim_interrupt(enc_stc* enc){


		if(enc->interrupt_flag == 0){									//every 3 times request the angle from encoder
			enc->angle0 = read_encoder(enc);
		}
		else{
			if(enc->interrupt_flag == 1){									//every 3 times request the angle from encoder
				enc->angle1 = read_encoder(enc);
			}
			else{
				if(enc->interrupt_flag == 2){									//calculate the speed from the two last angles
					get_speed_encoder(enc);

					uint8_t speed_Send = enc->average_speed;

					can.dataTx[0] = 0x06;
					can.dataTx[1] = speed_Send / 256;
					can.dataTx[2] = speed_Send % 256;
					can.dataTx[2] = 0;
					can.dataTx[3] = 0;
					can.dataTx[4] = 0;
					can.dataTx[5] = 0;
					can.dataTx[6] = 0;
					can.dataTx[7] = enc->steer_enc_prescaler;
					can.id = 0xD0;
					can.size = 8;
					CAN_Send(&can);
				}
			}
		}

		if(enc->interrupt_flag == 2){
			enc->interrupt_flag = 0;
		}
		else{
			enc->interrupt_flag ++;
		}
	}

	//funtion to calculate the speed
	//angle0 = last angle calculated
	//angle1 = previous angle calculated
	//refresh = delta-time from the two calculations, express it in microseconds
	//wheel_diameter = diameter of the wheel expressed meters
	void get_speed_encoder(enc_stc* enc){

		char text[100];
		double meters_per_second = 0;
		double dt = 0;

		dt = enc->refresh;

		enc->angle0 *= 100;
		enc->angle1 *= 100;

		meters_per_second = ((enc->angle1 - enc->angle0)/360)*3.1415*(enc->wheel_diameter);			//calculating the speed using the circumference arc
		meters_per_second /= dt;
		meters_per_second *= 10000;
		meters_per_second = round(meters_per_second)/100;

		//if(abs(meters_per_second - enc->speed[8]) <= abs(enc->speed[8] * 10)){			//exclude the wrong speeds
		shift_array(enc->speed, 15, meters_per_second);
		enc->average_speed = dynamic_average(enc->speed, 15);
		//}
	}

	pot_stc pot_1;
	pot_stc pot_2;
	void calc_pot_value(pot_stc *pot_1){

		pot_1->val_100 = (int)100-(abs(pot_1->val - pot_1->min)*100/(pot_1->range)); //val0_100 -->STEER --> 0 = SX | 100 = DX
		if (pot_1->val <= pot_1->min){
			pot_1->val_100 = 100;
		}
		if (pot_1->val >= pot_1->max){
			pot_1->val_100 = 0;
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
void shift_array(double *array, int size, double data){

	for(int i = 1; i < size; i++){
		array[i-1] = array[i];
	}
	array[size-1] = data;
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
