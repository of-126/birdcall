/*
 * new_uart.h
 *
 *  Created on: Feb 24, 2025
 *      Author: TEJAS
 */

#ifndef INC_NEW_UART_H_
#define INC_NEW_UART_H_

#include "main.h"
#include "new_rtc.h"

#define RX_buffersize 256
#define EOP_string "EOP"
extern char rxbuffer[RX_buffersize];
extern volatile uint8_t rxindex;
extern volatile uint8_t datarecived;
extern rtc_timedate_t timedate;


extern char uart_buffer[50];

extern int id1;

extern UART_HandleTypeDef huart3;

void senddata(char* str,uint8_t size);
char* recivedata(void);

void parse_and_print(char *buffer);

void UART_Print(const char *str);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);


#endif /* INC_NEW_UART_H_ */
