void funzione();
void print(char *); //send the argument to USART2
void print_3(char * text_print_function);
int print_it(UART_HandleTypeDef *huart,char * text_print_function);
int printf_it(UART_HandleTypeDef *huart,char * text_print_function);
void PRINT_INTERRUPT(UART_HandleTypeDef *huart);
