/*
 * new_eeprom.h
 *
 *  Created on: Feb 18, 2025
 *      Author: TEJAS
 */

#ifndef INC_NEW_EEPROM_H_
#define INC_NEW_EEPROM_H_

#include <stdio.h>
#include <string.h>

#include "stm32f4xx_hal.h"

#define EEPROMWIRTE 0xA0
#define EEPROMREAD 0xA1
#define pagesize 128
#define EEPROMSIZE 65536


extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart3;

HAL_StatusTypeDef eeprom_init(void);
HAL_StatusTypeDef eeprom_writebyte(uint16_t addr,uint8_t data);
uint8_t eeprom_readbyte(uint16_t addr);
HAL_StatusTypeDef eeprom_write(uint16_t addr,uint8_t* data,uint8_t len);
//uint8_t* eeprom_read(uint16_t addr);
uint8_t* eeprom_read(uint16_t addr, uint8_t* buffer, uint8_t len);


void eeprom_read_and_print_string(UART_HandleTypeDef *huart, uint16_t addr, uint8_t* buffer, uint8_t max_len);




#endif /* INC_NEW_EEPROM_H_ */
