/*
 * new_uart.c
 *
 *  Created on: Feb 24, 2025
 *      Author: TEJAS
 */



#include "new_uart.h"
#include "new_rtc.h"
#include "new_eeprom.h"


char rxbuffer[RX_buffersize];
volatile uint8_t rxindex = 0;
volatile uint8_t datarecived = 0;
rtc_timedate_t timedate;

char uart_buffer[50]; // Definition of uart_buffer
int id1 = 0;          // Definition of id1
int mode1=0;
int freq=0;

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


/*void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
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

}*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3 || huart->Instance == USART2)
  {
    if (rxindex < RX_buffersize - 1)
    {
      rxindex++;
    }
    else
    {
      rxbuffer[RX_buffersize - 1] = '\0';
      datarecived = 1;
      return;
    }
    if (rxindex >= strlen(EOP_string) && strncmp(&rxbuffer[rxindex - strlen(EOP_string)], EOP_string, strlen(EOP_string)) == 0)
    {
      rxbuffer[RX_buffersize - 1] = '\0';
      datarecived = 1;
      rxindex = 0;
      return;
    }

    // Restart interrupt for the specific UART instance
    if (huart->Instance == USART3)
    {
      HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuffer[rxindex], 1);
    }
    else if (huart->Instance == USART2)
    {
      HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxbuffer[rxindex], 1);
    }
  }
}


/*void parse_and_print(char *buffer) {
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
    }*/
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
    /*else if (id1 == 2) {
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


}*/




void parse_and_print(char *buffer) {
    char temp[RX_buffersize]; // Temporary buffer to hold a copy of input string
    strcpy(temp, buffer); // Copy to avoid modifying original string


    char *token = strtok(temp, "|");
    char msg[50];

    /*ret = eeprom_clear(ALARM_CONFIG_ADDR, RX_buffersize);
        if (ret != HAL_OK) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM clear failed\n\r", 21, HAL_MAX_DELAY);
        }*/

    //static rtc_timedate_t alarm_queue[MAX_ALARMS];
    //static uint8_t alarm_count = 0;
    //static uint8_t current_idx = 0;


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

    /*if (token) {
    		freq=0;
    		sscanf(token,"%d",&freq);
            sprintf(msg, "freq: %d\r\n", freq);
            UART_Print(msg);
            token = strtok(NULL, "|");
        }*/

    /*// Extract and print Location
    if (token) {
    	int mode=0;
    	sscanf(token,"%d",&mode);
    	sprintf(msg, "mode: %d\r\n", mode);
    	UART_Print(msg);
    	token = strtok(NULL, "|");
        }*/

    switch (id1) {
                case 1:
                    process_id1(token);
                    break;
                case 2:
                    process_id2(token,buffer);
                    break;
                case 3:
                	process_id3(token);
                	break;
                /*case 4:
                	process_id4(token);
                	break;*/
                default:
                    UART_Print("Unknown ID\r\n");
                    break;
            }

    //if(id1==1)
    //{
    // Extract and print Time (hh:mm:ss)
    /*if (token) {
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
    	}*/
    //}
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
    /*else if (id1 == 2) {
            if (token) {

            	int num_alarms;
            	sscanf(token, "%d", &num_alarms);
            	token = strtok(NULL, "|");


            	if (num_alarms > MAX_ALARMS) {
            		HAL_UART_Transmit(&huart3, (uint8_t*)"Too many!\n\r", 11, HAL_MAX_DELAY);
            		return;
            	        }
            	alarm_count = 0;
            	current_idx = 0;
            	while (token && alarm_count < num_alarms)
            	{


                UART_Print("time-\r\n");
                int hh, mm, ss;
                if (sscanf(token, "%d:%d:%d", &hh, &mm, &ss) == 3) {*/
                    /*sprintf(msg, "hour: %d\r\n", hh);
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
                    timedate.year = 0xFF;   // Optional: Disable year*/

                	/*sprintf(msg, "Alarm %d: %02d:%02d:%02d\r\n", alarm_count + 1, hh, mm, ss);
                	HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                	alarm_queue[alarm_count].hour = hh;
                	alarm_queue[alarm_count].minute = mm;
                	alarm_queue[alarm_count].second = ss;
                	alarm_queue[alarm_count].day = 0xFF;
                	alarm_queue[alarm_count].weekday = 0xFF;
                	alarm_queue[alarm_count].month = 0xFF;
                	alarm_queue[alarm_count].year = 0xFF;
                	alarm_count++;

                }
                token = strtok(NULL, "|");
            }*/

            	/*eeprom_store_config(buffer);
            	HAL_UART_Transmit(&huart3, (uint8_t*)"Alarms set\n\r", 12, HAL_MAX_DELAY);

            	if (alarm_count > 0) {
            	            ret = rtc_setalarm(&alarm_queue[0]);
            	            if (ret == HAL_OK) {
            	                HAL_UART_Transmit(&huart3, (uint8_t*)"First alarm set\n\r", 17, HAL_MAX_DELAY);
            	                if (alarm_count > 1) {
            	                    sprintf(uart_buffer, "Next: %02d:%02d:%02d\n\r",
            	                            alarm_queue[0].hour, alarm_queue[0].minute, alarm_queue[0].second);
            	                    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
            	                }
            	            }
            	        }*/

                    /*ret = rtc_setalarm(&timedate);  // Sets AIE and clears MSF
                    if (ret == HAL_OK) {
                        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm set\n\r", 11, HAL_MAX_DELAY);
                        uint8_t second = rtc_read(Second_alarm);
                        uint8_t minute = rtc_read(Minute_alarm);
                        uint8_t hour = rtc_read(Hour_alarm);
                        sprintf(uart_buffer, "alarm set for %02d:%02d:%02d\n\r", bcd_to_decimal(hour), bcd_to_decimal(minute), bcd_to_decimal(second));
                        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
                    } else {
                        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm notset\n\r", 14, HAL_MAX_DELAY);
                    }*/
                //}
    //}



}
// Function for ID 1: Parse and process time/date
void process_id1(char *token) {
    char msg[50];

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
    }
    token = strtok(NULL, "|");

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
            timedate.month = month;
            timedate.day = day;
            timedate.year = year;
        } else {
            UART_Print("Error: Invalid Date Format\r\n");
        }
    }

    int ret = rtc_settime(&timedate);
    if (ret == 0) {
        UART_Print("rtc time set ok\n\r");
    } else {
        UART_Print("rtc time set notok\n\r");
    }
}

// Function for ID 2: Parse and process alarms
void process_id2(char *token,char *buffer) {
    char msg[50];
    int num_alarms;

    // Extract and print mode
            if (token) {
            	sscanf(token,"%d",&mode1);
            	sprintf(msg, "mode: %d\r\n", mode1);
            	UART_Print(msg);
            	token = strtok(NULL, "|");
                }


            if(mode1!=1)
            {

            	if (rtc_clearalarm() == HAL_OK) {
            		                HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm flag cleared via INT\n\r", 28, HAL_MAX_DELAY);
            		            } else {
            		                HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to clear alarm flag via INT\n\r", 36, HAL_MAX_DELAY);
            		            }

    sscanf(token, "%d", &num_alarms);
    token = strtok(NULL, "|");

    if (num_alarms > MAX_ALARMS) {
        UART_Print("Too many alarms!\n\r");
        return;
    }

    alarm_count = 0;
    while (token && alarm_count < num_alarms) {
        UART_Print("time-\r\n");
        int hh, mm, ss;
        if (sscanf(token, "%d:%d:%d", &hh, &mm, &ss) == 3) {
            sprintf(msg, "Alarm %d: %02d:%02d:%02d\r\n", alarm_count + 1, hh, mm, ss);
            UART_Print(msg);

            alarm_queue[alarm_count].hour = hh;
            alarm_queue[alarm_count].minute = mm;
            alarm_queue[alarm_count].second = ss;
            alarm_queue[alarm_count].day = 0xFF;
            alarm_queue[alarm_count].weekday = 0xFF;
            alarm_queue[alarm_count].month = 0xFF;
            alarm_queue[alarm_count].year = 0xFF;
            alarm_count++;
        }
        token = strtok(NULL, "|");
    }
            }
            else
            {
            	process_id4(token);
            }

    // Write to EEPROM only for id=2
        ret = eeprom_write(0x0000, (uint8_t*)buffer, strlen(buffer) + 1);
        if (ret == HAL_OK) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"write ok\n\r", 10, HAL_MAX_DELAY);
            HAL_Delay(100);

            uint8_t eeprom_data[RX_buffersize];
            memset(eeprom_data, 0, RX_buffersize); // Clear the buffer
            if (eeprom_read(0x0000, eeprom_data, RX_buffersize) != NULL) {
                HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read OK: ", 16, HAL_MAX_DELAY);
                HAL_UART_Transmit(&huart3, eeprom_data, strlen((char*)eeprom_data), HAL_MAX_DELAY);
                HAL_UART_Transmit(&huart3, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
            } else {
                HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read failed\n\r", 20, HAL_MAX_DELAY);
            }
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"write notok\n\r", 13, HAL_MAX_DELAY);
            HAL_Delay(100);
        }

    //eeprom_store_config(token); // Store alarms in EEPROM
    UART_Print("Alarms set\n\r");

    if (alarm_count > 0) {
        int ret = rtc_setalarm(&alarm_queue[0]);
        if (ret == 0) {
            UART_Print("First alarm set\n\r");
            if (alarm_count > 1) {
                sprintf(msg, "Next: %02d:%02d:%02d\n\r",
                        alarm_queue[0].hour, alarm_queue[0].minute, alarm_queue[0].second);
                UART_Print(msg);
            }
        }
    }
}

void process_id3(char *token)
{
	printtime();
}

void process_id4(char *token)
{
	char msg[50];
	int numofalarm=0;
	int alarmarray[24];
	int hh,mm,ss;
	time1 first;

	// Extern variables from new_rtc.c for alarm management
	    extern rtc_timedate_t alarm_queue[MAX_ALARMS];
	    extern uint8_t alarm_count;
	    extern uint8_t current_idx;


	/*if (token) {
		sscanf(token,"%d",&mode);
		sprintf(msg, "mode: %d\r\n", mode);
		UART_Print(msg);
		token = strtok(NULL, "|");
	                }*/
	    if (rtc_clearalarm() == HAL_OK) {
	    	                HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm flag cleared via INT\n\r", 28, HAL_MAX_DELAY);
	    	            } else {
	    	                HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to clear alarm flag via INT\n\r", 36, HAL_MAX_DELAY);
	    	            }
	if(token)
	{
	sscanf(token,"%d:%d:%d",&hh,&mm,&ss);
	sprintf(msg, "current time: %02d:%02d:%02d\r\n", hh, mm, ss);
	UART_Print(msg);
	first.hh=hh;
	first.mm=mm;
	first.ss=ss;


	}

	token = strtok(NULL, "|");

	if(token)
		{
		sscanf(token,"%d:%d:%d",&hh,&mm,&ss);
		sprintf(msg, "stop time: %02d:%02d:%02d\r\n", hh, mm, ss);
		UART_Print(msg);

		first.hh1=hh;
		first.mm1=mm;
		first.ss1=ss;


		}

	token = strtok(NULL, "|");

	if(first.hh<0 || first.hh>23 ||first.mm<0 || first.mm>59 || first.ss<0 || first.ss>59 || first.hh1<0 || first.hh1>23 ||first.mm1<0 || first.mm1>59 || first.ss1<0 || first.ss1>59)
	{
		UART_Print("invalid start and stop");
	}
	int i=0;
		int currnthour=first.hh;

		while(currnthour!=first.hh1 && i<24)
		{
		    alarmarray[i]=currnthour;
		    //printf("%d\n",alarmarray[i]);
		    currnthour++;
		    if(currnthour>=24)
		    {
		        currnthour=0;
		    }
		    i++;
		}
		    if(i<24 && currnthour == first.hh1)
		    {
		        alarmarray[i]=currnthour;
		        //printf("%d\n",alarmarray[i]);
		        i++;
		    }
		    numofalarm=i;

		    // Check if number of alarms exceeds MAX_ALARMS
		        if (numofalarm > MAX_ALARMS) {
		            UART_Print("Too many alarms!\n\r");
		            return;
		        }

		        // Reset alarm queue
		            alarm_count = 0;
		            current_idx = 0;


		            // Populate alarm_queue with hourly alarms
		                for (int j = 0; j < numofalarm; j++) {
		                    sprintf(uart_buffer, "Alarm %d: %02d:%02d:%02d\r\n", j + 1, alarmarray[j], first.mm, first.ss);
		                    UART_Print(uart_buffer);

		                    alarm_queue[j].hour = alarmarray[j];
		                    alarm_queue[j].minute = first.mm;    // Use start time's minutes
		                    alarm_queue[j].second = first.ss;    // Use start time's seconds
		                    alarm_queue[j].day = 0xFF;           // Disable day
		                    alarm_queue[j].weekday = 0xFF;       // Disable weekday
		                    alarm_queue[j].month = 0xFF;         // Disable month
		                    alarm_queue[j].year = 0xFF;          // Disable year
		                    alarm_count++;
		                }



		    /*for(int j=0;j<i;j++)
		    sprintf(uart_buffer,"%2d:00:00\n",alarmarray[j]);
		    UART_Print(uart_buffer);
		    numofalarm=i;*/

		//}
		sprintf(uart_buffer,"%d\n\r",numofalarm);
		UART_Print(uart_buffer);

		/*// Set the first alarm in the RTC
		    if (alarm_count > 0) {
		        ret = rtc_setalarm(&alarm_queue[0]);
		        if (ret == HAL_OK) {
		            UART_Print("First alarm set\n\r");
		            uint8_t second = rtc_read(Second_alarm);
		            uint8_t minute = rtc_read(Minute_alarm);
		            uint8_t hour = rtc_read(Hour_alarm);
		            sprintf(uart_buffer, "Alarm set for %02d:%02d:%02d\n\r",
		                    bcd_to_decimal(hour), bcd_to_decimal(minute), bcd_to_decimal(second));
		            UART_Print(uart_buffer);
		            if (alarm_count > 1) {
		                sprintf(uart_buffer, "Next: %02d:%02d:%02d\n\r",
		                        alarm_queue[1].hour, alarm_queue[1].minute, alarm_queue[1].second);
		                UART_Print(uart_buffer);
		            }
		        } else {
		            UART_Print("Failed to set first alarm\n\r");
		        }
		    }*/




}


void modeselect(void)
{
	if(mode1==1)
	{
		senddata("mode =1\n\r",strlen("mode =1\n\r"));
	}
	else
	{
		senddata("mode =0\n\r", strlen("mode =0\n\r"));
	}
}



void printtime(void)
{
	ret=rtc_gettime(&timedate);
	        	  	            if(ret==HAL_OK)
	        	  	            {
	        	  	          	  sprintf(uart_buffer,"current time = %02d:%02d:%02d\n\r",timedate.hour,timedate.minute,timedate.second);
	        	  	          	  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	        	  	          	  sprintf(uart_buffer,"current date = %02d/%02d/20%02d\n\r",timedate.day,timedate.month,timedate.year);
	        	  	          	  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	        	  	          	  HAL_Delay(100);
	        	  	            }
	        	  	            else
	        	  	            {
	        	  	          	  HAL_UART_Transmit(&huart3, (uint8_t*)"rtc get time notok\n\r", 20, HAL_MAX_DELAY);
	        	  	          	  HAL_Delay(100);
	        	  	            }

}





/*// Main parsing function
void parse_and_print(char *buffer) {
    char temp[RX_buffersize];
    strcpy(temp, buffer);

    char *token = strtok(temp, "|");
    char msg[50];

    if (token) {
        int id;
        sscanf(token, "%d", &id);
        sprintf(msg, "id: %d\r\n", id);
        UART_Print(msg);
        token = strtok(NULL, "|");

        if (token) {
            sprintf(msg, "name: %s\r\n", token);
            UART_Print(msg);
            token = strtok(NULL, "|");
        }

        if (token) {
            sprintf(msg, "loc: %s\r\n", token);
            UART_Print(msg);
            token = strtok(NULL, "|");
        }

        switch (id) {
            case 1:
                process_id1(token);
                break;
            case 2:
                process_id2(token);
                break;
            default:
                UART_Print("Unknown ID\r\n");
                break;
        }
    }
}*/


void UART_Print(const char *str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}




