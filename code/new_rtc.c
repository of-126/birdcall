/*
 * new_rtc.c
 *
 *  Created on: Jan 30, 2025
 *      Author: TEJAS
 */

#include "new_rtc.h"

uint8_t decimal_to_bcd(uint8_t decimal) {
    return ((decimal / 10) << 4) | (decimal % 10);
}

uint8_t bcd_to_decimal(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}
void rtc_init(I2C_HandleTypeDef *hi2c)
{// 1 check if rtc is ready
	ret=HAL_I2C_IsDeviceReady(&hi2c1, slavwrite, 10, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  HAL_Delay(100);
		  HAL_UART_Transmit(&huart3, (uint8_t*)"RTC ok\n\r", 8, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"RTC notok\n\r", 11, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  //2 perform otp refresh Performing an OTP refresh ensures that the device operates with the correct calibration data after power-on or reset.
	  uint8_t data[2];
	  data[0]=CLKOUT_ctl;
	  data[1]=0x01;//set it to 1 to initiate the refresh process

	  ret=HAL_I2C_Master_Transmit(hi2c, slavwrite, data, 2, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"RTC OTP refersh ok\n\r", 20, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"RTC OTP refersh notok\n\r", 23, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }

	  //3 Configure Control_1 Register (0x00) for basic settings
	  data[0]=Control_1;
	  data[1]=0x00;
	  ret=HAL_I2C_Master_Transmit(hi2c, slavwrite, data, 2, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_1 ok\n\r", 14, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_1 notok\n\r", 17, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }

	  //4 Configure Control_2 Register (0x01) for alarm
	  data[0]=Control_2;
	  data[1]=0x00;
	  ret=HAL_I2C_Master_Transmit(hi2c, slavwrite, data, 2, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_2 ok\n\r", 14, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_2 notok\n\r", 17, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }

	  //5 Configure Control_3 Register (0x02) for battery
	  data[0]=Control_3;
	  data[1]=0x00;
	  ret=HAL_I2C_Master_Transmit(hi2c, slavwrite, data, 2, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_3 ok\n\r", 14, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"control_3 notok\n\r", 17, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }

	  //6 Set the CLKOUT frequency, if needed
	  data[0]=CLKOUT_ctl;
	  data[1]=0x07;
	  ret=HAL_I2C_Master_Transmit(hi2c, slavwrite, data, 2, HAL_MAX_DELAY);
	  if(ret==HAL_OK)
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"clkout ok\n\r", 11, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }
	  else
	  {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"clkout notok\n\r", 14, HAL_MAX_DELAY);
		  HAL_Delay(100);
	  }

}

HAL_StatusTypeDef rtc_wirte(uint8_t reg_addr,uint8_t value)
{
	uint8_t data[2];
	data[0]=reg_addr;
	data[1]=value;

	ret=HAL_I2C_Master_Transmit(&hi2c1, slavwrite, data, sizeof(data), HAL_MAX_DELAY);
	/*if(ret==HAL_OK)
	{
		return HAL_OK;
	}
	else
	{
		return HAL_ERROR;
	}*/
	if (ret != HAL_OK) {
	        HAL_UART_Transmit(&huart3, (uint8_t*)"I2C write fail\n\r", 16, HAL_MAX_DELAY);
	        return HAL_ERROR;
	    }
	    return HAL_OK;

}

uint8_t rtc_read(uint8_t reg_addr)
{
	 HAL_StatusTypeDef ret1;
	    uint8_t value = 0xFF; // Default value in case of error

	    // Transmit the register address
	    ret1 = HAL_I2C_Master_Transmit(&hi2c1, slavwrite, &reg_addr, 1, HAL_MAX_DELAY);
	    if (ret1 != HAL_OK) {
	        return value; // Return default value if transmit fails
	    }

	    // Receive the register value
	    ret1 = HAL_I2C_Master_Receive(&hi2c1, slaveread, &value, 1, HAL_MAX_DELAY);
	    if (ret1 != HAL_OK) {
	        return 0xFF; // Return default value if receive fails
	    }

	    return value; // Return the register value
}

HAL_StatusTypeDef rtc_settime(const rtc_timedate_t* timedate)
{
	if(rtc_wirte(Seconds, decimal_to_bcd(timedate->second))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(rtc_wirte(Minutes, decimal_to_bcd(timedate->minute))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(rtc_wirte(Hours, decimal_to_bcd(timedate->hour))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(rtc_wirte(Days, decimal_to_bcd(timedate->day))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(rtc_wirte(Months, decimal_to_bcd(timedate->month))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	if(rtc_wirte(Years, decimal_to_bcd(timedate->year))!=HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;

}

HAL_StatusTypeDef rtc_gettime(rtc_timedate_t* timedate)
{
	uint8_t temp;

	temp=rtc_read(Seconds);//second register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->second= bcd_to_decimal(temp);
	temp=rtc_read(Minutes);//minute register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->minute= bcd_to_decimal(temp);
	temp=rtc_read(Hours);//hour register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->hour= bcd_to_decimal(temp);
	temp=rtc_read(Days);//day register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->day= bcd_to_decimal(temp);
	temp=rtc_read(Months);//second register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->month= bcd_to_decimal(temp);
	temp=rtc_read(Years);//second register
	if(temp==0XFF)
	{
		return HAL_ERROR;
	}
	timedate->year= bcd_to_decimal(temp);

	return HAL_OK;
}


HAL_StatusTypeDef rtc_setalarm(rtc_timedate_t* timedate)
{
	//second alarm
	if(rtc_wirte(Second_alarm, (timedate->second==0xFF)?0x80:decimal_to_bcd(timedate->second))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	//minute alarm
	if(rtc_wirte(Minute_alarm, (timedate->minute==0xFF)?0x80:decimal_to_bcd(timedate->minute))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	//hour alarm
	if(rtc_wirte(Hour_alarm, (timedate->hour==0xFF)?0x80:decimal_to_bcd(timedate->hour))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	//day alarm
	if(rtc_wirte(Day_alarm, (timedate->day==0xFF)?0x80:decimal_to_bcd(timedate->day))!=HAL_OK)
	{
		return HAL_ERROR;
	}
	//week alarm
	if(rtc_wirte(Weekday_alarm, (timedate->weekday==0xFF)?0x80:decimal_to_bcd(timedate->weekday))!=HAL_OK)
	{
		return HAL_ERROR;
	}

	uint8_t control2 = rtc_read(Control_2);
	    if (control2 == 0xFF) return HAL_ERROR;
	    control2 &= ~(1 << 7);  // Clear MSF
	    control2 |= (1 << 1);   // Set AIE
	    if (rtc_wirte(Control_2, control2) != HAL_OK) return HAL_ERROR;

	    HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm set complete\n\r", 20, HAL_MAX_DELAY);
	    read_and_transmit(Control_2);

	/*ret=rtc_enable_alarm_interrupt();
	if(ret==HAL_OK)
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)"interrupt enabled\n\r", 19, HAL_MAX_DELAY);
		HAL_Delay(100);
	}
	else
	{
		HAL_UART_Transmit(&huart3, (uint8_t*)"interrupt notenabled\n\r", 22, HAL_MAX_DELAY);
		HAL_Delay(100);
	}*/
	return HAL_OK;
}

HAL_StatusTypeDef rtc_clearalarm(void)
{
	uint8_t control2;
	//read_and_transmit(0x01);
	control2=rtc_read(Control_2);
	//read_and_transmit(0x01);
	if(control2==0xFF)
	{
		return HAL_ERROR;
	}
	control2 &=~(1<<4);
	//control2=0x00;
	if(rtc_wirte(Control_2, control2)!=HAL_OK)
	{
		//read_and_transmit(0x01);
		return HAL_ERROR;
	}

	return HAL_OK;
}

uint8_t rtc_check_alarm_flag(void)
{
	{
	uint8_t control2 = rtc_read(Control_2); // Read the Control_2 register
	read_and_transmit(Control_2);
	    if (control2 == 0xFF) {
	    	HAL_UART_Transmit(&huart3, (uint8_t*)"Flag read error\n\r", 17, HAL_MAX_DELAY);
	        return 0xFF; // Error occurred while reading
	    }

	    return (control2 & (1 << 4)) ? 1 : 0; // Return 1 if AF is set, otherwise 0
	}
}

HAL_StatusTypeDef rtc_enable_alarm_interrupt(void) {
    uint8_t control2 = rtc_read(Control_2); // Read the Control_2 register
    if (control2 == 0xFF) {
        return HAL_ERROR; // Error occurred while reading
    }

    control2 |= (1 << 1); // Set the AIE bit (bit 1)
    rtc_wirte(Control_2, control2);
    //read_and_transmit(Control_2);
    return HAL_OK;
}

void read_and_transmit(uint8_t reg_addr) {
    char uart_buffer[50];
    uint8_t reg_value = rtc_read(reg_addr);

    // Check if the read was successful
    if (reg_value == 0xFF) {
        // Error handling: Send an error message over UART
        sprintf(uart_buffer, "Error reading register 0x%02X\r\n", reg_addr);
    } else {
        // Send the read value over UART
        sprintf(uart_buffer, "Register 0x%02X: 0x%02X\r\n", reg_addr, reg_value);
    }

    // Transmit the message via UART3
    HAL_UART_Transmit(&huart3, (uint8_t *)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
}
