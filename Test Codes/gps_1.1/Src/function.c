#ifndef FUNCTION_H
#define FUNCTION_H

#include "stm32f4xx_hal.h"
#include "string.h"
#include "function.h"



extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int b;
extern char text[];



void funzione(){
	b++;
	text[5]=b+48;
	print(text);
}

void print(char * text_print_function){

	HAL_UART_Transmit(&huart2, (uint8_t*)text_print_function, strlen(text_print_function), 5);
	HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 1);

}
void print_3(char * text_print_function){

	HAL_UART_Transmit(&huart3, (uint8_t*)text_print_function, strlen(text_print_function), 100);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 1);

}

#endif
