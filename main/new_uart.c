/*
 * new_uart.c
 *
 *  Created on: Feb 24, 2025
 *      Author: TEJAS
 */



#include "new_uart.h"


char rxbuffer[RX_buffersize];
volatile uint8_t rxindex = 0;
volatile uint8_t datarecived = 0;
rtc_timedate_t timedate;

char uart_buffer[50]; // Definition of uart_buffer
int id1 = 0;          // Definition of id1

void senddata(char* str,uint8_t size)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)"\n\r",2, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart3, (uint8_t*)str,size, HAL_MAX_DELAY);
	HAL_Delay(100);
}

char* recivedata(void)
{
	static char rxdata1[10];
	HAL_UART_Receive(&huart3, (uint8_t*)rxdata1, sizeof(rxdata1)-1, HAL_MAX_DELAY);
	HAL_Delay(100);
	rxdata1[sizeof(rxdata1)-1]='\0';

	return rxdata1;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART3)
	{
		if(rxindex<RX_buffersize-1)
		{
			rxindex++;
		}
		else
		{
			rxbuffer[RX_buffersize-1]='\0';
			datarecived=1;
			return;
		}
		if(rxindex>=strlen(EOP_string)&&strncmp(&rxbuffer[rxindex-strlen(EOP_string)],EOP_string,strlen(EOP_string))==0)
		{
			rxbuffer[RX_buffersize-1]='\0';
			datarecived=1;
			rxindex=0;

			return;
		}

		HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuffer[rxindex], 1);
	}

}

void parse_and_print(char *buffer) {
    char temp[100]; // Temporary buffer to hold a copy of input string
    strcpy(temp, buffer); // Copy to avoid modifying original string


    char *token = strtok(temp, "|");
    char msg[50];

    // Extract and print ID
    if (token) {
    	int id;
    	sscanf(token,"%d",&id);
        sprintf(msg, "id: %d\r\n", id);
        UART_Print(msg);
        token = strtok(NULL, "|");
        id1=id;
    }
    if(id1==1)
    {

    // Extract and print Name
    if (token) {
        sprintf(msg, "name: %s\r\n", token);
        UART_Print(msg);
        token = strtok(NULL, "|");
    }

    // Extract and print Location
    if (token) {
        sprintf(msg, "loc: %s\r\n", token);
        UART_Print(msg);
        token = strtok(NULL, "|");
    }

    // Extract and print Time (hh:mm:ss)
    if (token) {
        UART_Print("time-\r\n");
        int hh, mm, ss;
        if (sscanf(token, "%d:%d:%d", &hh, &mm, &ss) == 3) {
            sprintf(msg, "hour: %d\r\n", hh);
            UART_Print(msg);
            sprintf(msg, "min: %d\r\n", mm);
            UART_Print(msg);
            sprintf(msg, "sec: %d\r\n", ss);
            UART_Print(msg);
            timedate.hour=hh;
            timedate.minute=mm;
            timedate.second=ss;
        }
        token = strtok(NULL, "|");
    }

    // Extract and print Date (mm/dd/yyyy)
    if (token) {
        UART_Print("date-\r\n");
        int month, day, year;
        if (sscanf(token, "%d/%d/%d", &month, &day, &year) == 3) {
            sprintf(msg, "month: %d\r\n", month);
            UART_Print(msg);
            sprintf(msg, "day: %d\r\n", day);
            UART_Print(msg);
            sprintf(msg, "year: %d\r\n", year);
            UART_Print(msg);
            timedate.month=month;
            timedate.day=day;
            timedate.year=year;
        } else {
            UART_Print("Error: Invalid Date Format\r\n");
        }
        token = strtok(NULL, "|");
    }
   }
    else if(id1==2)
    {

    }
}


void UART_Print(const char *str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}




