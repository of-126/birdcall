/*
 * new_eeprom.c
 *
 *  Created on: Feb 18, 2025
 *      Author: TEJAS
 */



#include "new_eeprom.h"


HAL_StatusTypeDef eeprom_init(void)
{
	if(HAL_I2C_IsDeviceReady(&hi2c1, EEPROMWIRTE, 10, HAL_MAX_DELAY)!=HAL_OK)
	{
		return HAL_ERROR;
	}
	else
	{
		return HAL_OK;
	}
}


HAL_StatusTypeDef eeprom_writebyte(uint16_t addr,uint8_t data)
{
	return HAL_I2C_Mem_Write(&hi2c1, EEPROMWIRTE, addr, 2, &data, 1, HAL_MAX_DELAY);
}


uint8_t eeprom_readbyte(uint16_t addr)
{
	uint8_t data=0xFF;
	if(HAL_I2C_Mem_Read(&hi2c1, EEPROMREAD, addr, 2, &data, 1, HAL_MAX_DELAY)==HAL_OK)
	{
		return data;
	}
	else
	{
		return 0xFF;
	}
}


HAL_StatusTypeDef eeprom_write(uint16_t addr,uint8_t* data,uint8_t len)
{
	if(len > 128)
	{
		return HAL_ERROR;
	}
	else
	{
		if(HAL_I2C_Mem_Write(&hi2c1, EEPROMWIRTE, addr, 2, data, len, HAL_MAX_DELAY)!=HAL_OK)
		{
		return HAL_ERROR;
		}
		HAL_Delay(10);
		return HAL_OK;
	}
}


uint8_t* eeprom_read(uint16_t addr, uint8_t* buffer, uint8_t len) {
    if (buffer == NULL || len == 0) {
        return NULL;  // Invalid input
    }
    if (HAL_I2C_Mem_Read(&hi2c1, EEPROMREAD, addr, 2, buffer, len, HAL_MAX_DELAY) != HAL_OK) {
        return NULL;  // Read failed
    }
    return buffer;  // Return pointer to the provided buffer
}


void eeprom_read_and_print_string(UART_HandleTypeDef *huart, uint16_t addr, uint8_t* buffer, uint8_t max_len)
{
    if (buffer == NULL || max_len == 0)
    {
        HAL_UART_Transmit(huart, (uint8_t*)"Invalid buffer or length\n\r", 27, HAL_MAX_DELAY);
        return;
    }

    // Read data from EEPROM
    if (eeprom_read(addr, buffer, max_len) == NULL)
    {
        HAL_UART_Transmit(huart, (uint8_t*)"EEPROM read failed\n\r", 20, HAL_MAX_DELAY);
        return;
    }

    // Find the null terminator to determine the string length
    uint8_t len = 0;
    while (len < max_len && buffer[len] != '\0')
    {
        len++;
    }

    // Print the string via UART
    HAL_UART_Transmit(huart, (uint8_t*)"String read from EEPROM: ", 25, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, buffer, len, HAL_MAX_DELAY);
    HAL_UART_Transmit(huart, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
}

