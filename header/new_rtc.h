/*
 * new_rtc.h
 *
 *  Created on: Jan 30, 2025
 *      Author: TEJAS
 */

#ifndef INC_NEW_RTC_H_
#define INC_NEW_RTC_H_

#include <stdio.h>
#include <string.h>
#include "main.h"


#define slaveread 0xA3
#define slavwrite 0XA2

#define Seconds 0x03
#define Minutes 0x04
#define Hours 0x05
#define Days 0x06
#define Weekdays 0x07
#define Months 0x08
#define Years 0x09

#define Second_alarm 0x0A
#define Minute_alarm 0x0B
#define Hour_alarm 0x0C
#define Day_alarm 0x0D
#define Weekday_alarm 0x0E

#define CLKOUT_ctl 0x0F
#define Control_1 0x00
#define Control_2 0x01
#define Control_3 0x02

#define MAX_ALARMS 14


extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;
extern HAL_StatusTypeDef ret;

typedef struct {
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t day;
	uint8_t month;
	uint8_t year;
	uint8_t weekday;
}rtc_timedate_t;

extern rtc_timedate_t alarm_queue[MAX_ALARMS]; // Alarm queue
extern uint8_t alarm_count;                    // Number of alarms
extern uint8_t current_idx;                    // Current alarm index


uint8_t decimal_to_bcd(uint8_t decimal);
//uint8_t decimal_to_bcd(uint8_t decimal) {
//    return ((decimal / 10) << 4) | (decimal % 10);
//}

uint8_t bcd_to_decimal(uint8_t bcd);
//uint8_t bcd_to_decimal(uint8_t bcd) {
//    return ((bcd >> 4) * 10) + (bcd & 0x0F);
//}
void rtc_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef rtc_wirte(uint8_t reg_addr,uint8_t value);
uint8_t rtc_read(uint8_t reg_addr);
HAL_StatusTypeDef rtc_settime(const rtc_timedate_t* timedate);
HAL_StatusTypeDef rtc_gettime(rtc_timedate_t* timedate);
HAL_StatusTypeDef rtc_setalarm(rtc_timedate_t* timedate);
HAL_StatusTypeDef rtc_clearalarm(void);
uint8_t rtc_check_alarm_flag(void);
HAL_StatusTypeDef rtc_enable_alarm_interrupt(void);

void read_and_transmit(uint8_t reg_addr);

#endif /* INC_NEW_RTC_H_ */
