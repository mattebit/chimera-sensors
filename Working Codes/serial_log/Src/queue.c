#include "malloc.h"
#include "queue.h"
#include "string.h"

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
