/*
 * new_uart.c
 *
 *  Created on: Feb 24, 2025
 *      Author: TEJAS
 */



#include "new_uart.h"
#include "new_rtc.h"


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
    if(id1==1)
    {
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
    ret=rtc_settime(&timedate);
    	if(ret==HAL_OK)
    	{
    		HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set ok\n\r", 17, HAL_MAX_DELAY);
    		HAL_Delay(100);
    	}
    	else
    	{
    		HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set notok\n\r", 20, HAL_MAX_DELAY);
    		HAL_Delay(100);
    	}
    }
    /*else if(id1==2)
    {
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
    	    }*/
    	/*
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
    	    }*/

    	          /*ret=rtc_enable_alarm_interrupt();
    	          if(ret==HAL_OK)
    	          {
    	        	  HAL_UART_Transmit(&huart3, (uint8_t*)"interrupt enabled\n\r", 19, HAL_MAX_DELAY);
    	        	  HAL_Delay(100);
    	        	  //read_and_transmit(0x01);
    	          }
    	          else
    	          {
    	        	  HAL_UART_Transmit(&huart3, (uint8_t*)"interrupt notenabled\n\r", 22, HAL_MAX_DELAY);
    	        	  HAL_Delay(100);
    	          }

    	          ret=rtc_setalarm(&timedate);
    	          if(ret==HAL_OK)
    	          {

    	        	  HAL_UART_Transmit(&huart3, (uint8_t*)"alarm set\n\r", 11, HAL_MAX_DELAY);
    	        	  HAL_Delay(100);
    	        	  uint8_t second=rtc_read(Second_alarm);
    	        	  uint8_t minute=rtc_read(Minute_alarm);
    	        	  uint8_t hour=rtc_read(Hour_alarm);
    	        	  sprintf(uart_buffer,"alarm set for %02d:%02d:%02d\n\r",bcd_to_decimal(hour),bcd_to_decimal(minute),bcd_to_decimal(second));
    	        	  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    	        	  HAL_Delay(1000);
    	          }
    	          else
    	          {
    	        	  HAL_UART_Transmit(&huart3, (uint8_t*)"alarm notset\n\r", 14, HAL_MAX_DELAY);
    	        	  HAL_Delay(100);
    	          }
    }*/
    else if (id1 == 2) {
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
                    timedate.hour = hh;
                    timedate.minute = mm;
                    timedate.second = ss;
                    timedate.day = 0xFF;    // Disable day
                    timedate.weekday = 0xFF; // Disable weekday
                    timedate.month = 0xFF;  // Optional: Disable month
                    timedate.year = 0xFF;   // Optional: Disable year
                }
                token = strtok(NULL, "|");
            }

                    ret = rtc_setalarm(&timedate);  // Sets AIE and clears MSF
                    if (ret == HAL_OK) {
                        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm set\n\r", 11, HAL_MAX_DELAY);
                        uint8_t second = rtc_read(Second_alarm);
                        uint8_t minute = rtc_read(Minute_alarm);
                        uint8_t hour = rtc_read(Hour_alarm);
                        sprintf(uart_buffer, "alarm set for %02d:%02d:%02d\n\r", bcd_to_decimal(hour), bcd_to_decimal(minute), bcd_to_decimal(second));
                        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                    } else {
                        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm notset\n\r", 14, HAL_MAX_DELAY);
                    }
                }


}


void UART_Print(const char *str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}




