/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include "new_eeprom.h"
#include "new_rtc.h"
#include "new_uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_rx;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef ret;
char temp[500];
uint8_t* dataread;

volatile uint8_t alarmflag=0;


#define RX_buffersize 128
#define EOP_string "EOP"

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2S2_Init();
  MX_SDIO_SD_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */


  //char txdata[50]="send data:";

  HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuffer[rxindex], 1);


ret=eeprom_init();
if(ret==HAL_OK)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)"eeprom ok\n\r", 11, HAL_MAX_DELAY);
	HAL_Delay(100);
}
else
{
	HAL_UART_Transmit(&huart3, (uint8_t*)"eeprom notok\n\r", 14, HAL_MAX_DELAY);
	HAL_Delay(100);
}
rtc_init(&hi2c1);

	rtc_timedate_t timedate = {1, 1, 0, 1, 1, 25, 0xFF};  // 01:01:00, Jan 1, 2025
    ret = rtc_settime(&timedate);
    if (ret == HAL_OK) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set ok\n\r", 17, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set notok\n\r", 20, HAL_MAX_DELAY);
    }
    /*
    // Set alarm to 01:01:10 (10 seconds ahead)
    rtc_timedate_t alarm = {1, 1, 10, 0xFF, 0xFF, 0xFF, 0xFF};
    ret = rtc_setalarm(&alarm);
    if (ret == HAL_OK) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm set ok\n\r", 14, HAL_MAX_DELAY);
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"alarm set notok\n\r", 17, HAL_MAX_DELAY);
    }
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (datarecived)  // Full message received
	  	      {
		  	  	  uint8_t eeprom_data[pagesize];
	  	          senddata(rxbuffer, strlen(rxbuffer)); // Send received data
	  	          ret=eeprom_write(0x0000, (uint8_t*)&rxbuffer, strlen(rxbuffer)+1);
	  	          if(ret==HAL_OK)
	  	          {
	  	        	  HAL_UART_Transmit(&huart3, (uint8_t*)"write ok\n\r", 10, HAL_MAX_DELAY);
	  	        	  HAL_Delay(100);
	  	        	  //eeprom_read_and_print_string(&huart3, 0x0000, (uint8_t*)txdata, 50);
	  	        	  //uint8_t eeprom_data[pagesize];  // Buffer to hold EEPROM data
	  	        	  memset(eeprom_data, 0, pagesize);  // Clear the buffer

	  	        	  if (eeprom_read(0x0000, eeprom_data, pagesize) != NULL)
	  	        	  {
	  	        		  HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read OK: ", 16, HAL_MAX_DELAY);
	  	        	      HAL_UART_Transmit(&huart3, eeprom_data, strlen((char*)eeprom_data), HAL_MAX_DELAY);
	  	        	      HAL_UART_Transmit(&huart3, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
	  	        	   }
	  	        	   else
	  	        	   {
	  	        	      HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read failed\n\r", 20, HAL_MAX_DELAY);
	  	        	   }
	  	          }
	  	          else
	  	          {
	  	        	 HAL_UART_Transmit(&huart3, (uint8_t*)"write notok\n\r", 13, HAL_MAX_DELAY);
	  	        	 HAL_Delay(100);
	  	          }

	  	        char rxCopy[RX_buffersize];
	  	        strcpy(rxCopy, (char*)eeprom_data);
	  	        parse_and_print(rxCopy);
	  	      // Debug alarm registers after parsing
	  	              read_and_transmit(Second_alarm);
	  	              read_and_transmit(Minute_alarm);
	  	              read_and_transmit(Hour_alarm);
	  	              read_and_transmit(Day_alarm);
	  	              read_and_transmit(Weekday_alarm);
	  	        /*ret=rtc_settime(&timedate);
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
	  	      /*ret=rtc_gettime(&timedate);
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
	  	            }*/
	  	      	  	          // Reset for the next reception
	  	          datarecived = 0;
	  	          rxindex = 0;
	  	          memset(rxbuffer, 0, RX_buffersize); // Clear buffer
	  	          HAL_UART_Receive_IT(&huart3, (uint8_t *)&rxbuffer[rxindex], 1);
	  	      }
	  /*rtc_gettime(&timedate);
	  	      	  sprintf(uart_buffer,"current time = %02d:%02d:%02d\n\r",timedate.hour,timedate.minute,timedate.second);
	  	      	  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	  	      	  HAL_Delay(1000);*/

	  	      	  /*alarmflag=rtc_check_alarm_flag();
	  	      	  if(alarmflag==0)
	  	      	  {
	  	      		  rtc_gettime(&timedate);
	  	      		  sprintf(uart_buffer,"current time = %02d:%02d:%02d\n\r",timedate.hour,timedate.minute,timedate.second);
	  	      		  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	  	      		  HAL_Delay(1000);
	  	      		  //HAL_UART_Transmit(&huart3, (uint8_t*)"interrupt flag set\n\r", 20, HAL_MAX_DELAY);
	  	      	  }
	  	      	  else
	  	      	  {
	  	      		  if (rtc_clearalarm() == HAL_OK) {
	  	      			  HAL_UART_Transmit(&huart3, (uint8_t *)"Alarm flag cleared.\n\r", 22, HAL_MAX_DELAY);
	  	      		  } else {
	  	      			  HAL_UART_Transmit(&huart3, (uint8_t *)"Failed to clear alarm flag.\n\r", 30, HAL_MAX_DELAY);
	  	      		  }
	  	      		  //break;
	  	      	  }*/

	  //read_and_transmit(0x01);
	  /*alarmflag = rtc_check_alarm_flag();
	  	      if (alarmflag == 0xFF) {
	  	          HAL_UART_Transmit(&huart3, (uint8_t*)"Error checking flag\n\r", 21, HAL_MAX_DELAY);
	  	      } else if (alarmflag == 1) {
	  	          HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm triggered!\n\r", 18, HAL_MAX_DELAY);
	  	          if (rtc_clearalarm() == HAL_OK) {
	  	              HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm flag cleared.\n\r", 22, HAL_MAX_DELAY);
	  	              break;
	  	          } else {
	  	              HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to clear alarm flag.\n\r", 30, HAL_MAX_DELAY);
	  	          }
	  	      }
	  //read_and_transmit(0x01);
	  rtc_gettime(&timedate);
	  sprintf(uart_buffer, "current time = %02d:%02d:%02d\n\r", timedate.hour, timedate.minute, timedate.second);
	  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	  HAL_Delay(1000);
*/
	  alarmflag = rtc_check_alarm_flag();
	          if (alarmflag == 0xFF) {
	              HAL_UART_Transmit(&huart3, (uint8_t*)"Error checking flag\n\r", 21, HAL_MAX_DELAY);
	          } else if (alarmflag == 1) {
	              HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm triggered!\n\r", 18, HAL_MAX_DELAY);
	              if (rtc_clearalarm() == HAL_OK) {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm flag cleared.\n\r", 22, HAL_MAX_DELAY);
	              } else {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to clear alarm flag.\n\r", 30, HAL_MAX_DELAY);
	              }
	          }

	          ret = rtc_gettime(&timedate);
	          if (ret == HAL_OK) {
	              sprintf(uart_buffer, "current time = %02d:%02d:%02d\n\r", timedate.hour, timedate.minute, timedate.second);
	              HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	          }
	          else {
	                  HAL_UART_Transmit(&huart3, (uint8_t*)"rtc get time notok\n\r", 20, HAL_MAX_DELAY);
	              }
	          HAL_Delay(1000);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 165;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s2.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s2.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE4 PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD3 PD4
                           PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
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
        sprintf(msg, "id: %d\r\n", token);
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


void UART_Print(const char *str) {
    HAL_UART_Transmit(&huart3, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0) {
	        if (rtc_check_alarm_flag() == 1) {
	            HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm via INT!\n\r", 16, HAL_MAX_DELAY);
	            if (rtc_clearalarm() == HAL_OK) {
	                HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm flag cleared via INT\n\r", 28, HAL_MAX_DELAY);
	            } else {
	                HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to clear alarm flag via INT\n\r", 36, HAL_MAX_DELAY);
	            }
	        }
	    }

}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
