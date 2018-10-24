#ifndef FUNCTION_H
#define FUNCTION_H

#include "stm32f4xx_hal.h"
#include "string.h"
#include "function.h"
#include "queue.h"



extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern int b;
extern char text[];

queue print_q={.head=0,.tail=0,.dim=0};


void funzione(){
	b++;
	text[5]=b+48;
	print(text);
}

void print(char * text_print_function){

	HAL_UART_Transmit(&huart2, (uint8_t*)text_print_function, strlen(text_print_function), 5);
	//HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 1);

}
void print_3(char * text_print_function){

	HAL_UART_Transmit(&huart3, (uint8_t*)text_print_function, strlen(text_print_function), 100);
	HAL_UART_Transmit(&huart3, (uint8_t*)"\r\n", 2, 1);

}
int printf_it(UART_HandleTypeDef *huart,char * text_print_function){
	int ret=0;
	if(HAL_UART_Transmit_IT(huart, (uint8_t*)text_print_function, strlen(text_print_function))==HAL_OK){
		if(push("\n\r",&print_q)==FAIL){
			ret=0;
		}else{
			ret=1;
		}
	}else{
		if(push(text_print_function,&print_q)==FAIL){
			ret=0;
		}else{
			if(push("\n\r",&print_q)==FAIL){
				ret=0;
			}else{
				ret=1;
			}
		}
	}
	return ret;
}
int print_it(UART_HandleTypeDef *huart,char * text_print_function){
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
void PRINT_INTERRUPT(UART_HandleTypeDef *huart){
	char text_print_function[50];
	if(pop(text_print_function,&print_q)==OK){
		HAL_UART_Transmit_IT(huart, (uint8_t*)text_print_function, strlen(text_print_function));
	}
}

#endif
