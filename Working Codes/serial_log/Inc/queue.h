#ifndef QUEUE_H
#define QUEUE_H
#include "stm32f4xx_hal.h"
typedef struct
{
  int head, tail;
  int dim;
  char * elem[40];
  char stringa[50];
}queue;
enum retval { FAIL, OK };
void init (queue *);
int push(char *,queue *);
int pop(char  *,queue *);

int print(UART_HandleTypeDef *huart,char * text_print_function);
void print_it(UART_HandleTypeDef *huart);
#endif
