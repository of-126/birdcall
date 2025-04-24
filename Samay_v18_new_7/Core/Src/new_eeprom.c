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


/*uint8_t* eeprom_read(uint16_t addr, uint8_t* buffer, uint8_t len) {
    if (buffer == NULL || len == 0) {
        return NULL;  // Invalid input
    }
    if (HAL_I2C_Mem_Read(&hi2c1, EEPROMREAD, addr, 2, buffer, len, HAL_MAX_DELAY) != HAL_OK) {
        return NULL;  // Read failed
    }
    return buffer;  // Return pointer to the provided buffer
}*/
uint8_t* eeprom_read(uint16_t addr, uint8_t* buffer, uint16_t len) {
    // Removed initial check to debug I2C read
    uint16_t offset = 0;
    while (offset < len) {
        uint16_t chunk_size = (len - offset > pagesize) ? pagesize : (len - offset);
        HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, EEPROMREAD, addr + offset, 2, buffer + offset, chunk_size, HAL_MAX_DELAY);
        if (status != HAL_OK) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read fail at offset ", 27, HAL_MAX_DELAY);
            char msg[20];
            sprintf(msg, "%d, status: %d\n\r", offset, status);
            HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
            return NULL;
        }
        offset += chunk_size;
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read complete\n\r", 22, HAL_MAX_DELAY);
    return buffer;
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

HAL_StatusTypeDef eeprom_store_config(const char* config) {
    uint16_t len = strlen(config) + 1;
    if (len > EEPROMSIZE)
    	{
    	HAL_UART_Transmit(&huart3, (uint8_t*)"Config too long\n\r", 17, HAL_MAX_DELAY);
    	return HAL_ERROR;
    	}
    	uint16_t offset = 0;
        while (offset < len) {
            uint16_t chunk_size = (len - offset > pagesize) ? pagesize : (len - offset);
            if (HAL_I2C_Mem_Write(&hi2c1, EEPROMWIRTE, ALARM_CONFIG_ADDR + offset, 2, (uint8_t*)(config + offset), chunk_size, HAL_MAX_DELAY) != HAL_OK) {
                HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM write fail\n\r", 19, HAL_MAX_DELAY);
                return HAL_ERROR;
            }
            HAL_Delay(10);  // EEPROM write cycle delay
            offset += chunk_size;
        }
        return HAL_OK;
}

/*HAL_StatusTypeDef eeprom_read_config(char* buffer, uint16_t max_len) {
    if (eeprom_read(EEPROMREAD, (uint8_t*)buffer, max_len) != NULL) {
        buffer[max_len - 1] = '\0';
        return HAL_OK;
    }
    return HAL_ERROR;
}*/
/* eeprom_read_config(char* buffer, uint16_t max_len) {
    const uint16_t read_size = 256;  // Read 2 pages
    if (max_len < read_size) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Buffer too small for 2 pages\n\r", 30, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    if (eeprom_read(ALARM_CONFIG_ADDR, (uint8_t*)buffer, read_size) != NULL) {
        buffer[read_size - 1] = '\0';
        if (strlen(buffer) > 0 && buffer[0] != 0xFF && buffer[0] != 0x00) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Config read success (256 bytes)\n\r", 33, HAL_MAX_DELAY);
            return HAL_OK;
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Config empty or invalid\n\r", 25, HAL_MAX_DELAY);
            return HAL_ERROR;
        }
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read config fail\n\r", 25, HAL_MAX_DELAY);
    return HAL_ERROR;
}*/

HAL_StatusTypeDef eeprom_read_config(char* buffer, uint16_t max_len) {
    if (max_len < pagesize) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Buffer too small\n\r", 18, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
    if (eeprom_read(ALARM_CONFIG_ADDR, (uint8_t*)buffer, max_len) != NULL) {
        // Find the actual length of the string (up to EOP or null)
        uint16_t len = 0;
        while (len < max_len && buffer[len] != '\0' && strncmp(&buffer[len], "EOP", 3) != 0) {
            len++;
        }
        if (len < max_len && strncmp(&buffer[len], "EOP", 3) == 0) {
            len += 3; // Include "EOP" in the length
        }
        buffer[len] = '\0'; // Null-terminate at the end of valid data
        if (len > 0 && buffer[0] != 0xFF && buffer[0] != 0x00) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Config read success\n\r", 21, HAL_MAX_DELAY);
            return HAL_OK;
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Config empty or invalid\n\r", 25, HAL_MAX_DELAY);
            return HAL_ERROR;
        }
    }
    HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read config fail\n\r", 25, HAL_MAX_DELAY);
    return HAL_ERROR;
}




/*
HAL_StatusTypeDef eeprom_clear(uint16_t addr, uint16_t len) {
    if (addr + len > EEPROMSIZE) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"Clear range exceeds EEPROM size\n\r", 33, HAL_MAX_DELAY);
        return HAL_ERROR;
    }

    uint8_t zero_buffer[pagesize] = {0};  // Buffer filled with zeros
    uint16_t offset = 0;

    while (offset < len) {
        uint16_t chunk_size = (len - offset > pagesize) ? pagesize : (len - offset);
        if (HAL_I2C_Mem_Write(&hi2c1, EEPROMWIRTE, addr + offset, 2, zero_buffer, chunk_size, HAL_MAX_DELAY) != HAL_OK) {
            HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM clear fail\n\r", 19, HAL_MAX_DELAY);
            return HAL_ERROR;
        }
        HAL_Delay(10);  // Write cycle delay
        offset += chunk_size;
    }

    HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM cleared\n\r", 16, HAL_MAX_DELAY);
    return HAL_OK;
}*/




