/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "ff.h"
#include "string.h"
#include "File_Handling.h"
#include <stdlib.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <math.h>

#include "new_eeprom.h"
#include "new_rtc.h"
#include "new_uart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*	Change Start */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DE_BUG 0

// for new_uart.c
#define RX_buffersize 256
#define EOP_string "EOP"

HAL_StatusTypeDef ret;
char temp[500];
uint8_t* dataread;


#define print(msg) \
    HAL_UART_Transmit(&huart3, (uint8_t *)(msg), sizeof(msg) - 1, HAL_MAX_DELAY)

#define HEADER_SIZE 44 // Size of the WAVE header
#define AUDIO_DATA_SIZE 32 // Size of the audio data (16 bytes in this case)

#define AUDIO_BUFFER_SIZE 2000  // Define the size of the audio buffer
#define DEFAULT_AUDIO_IN_FREQ 16000
#define WR_BUFFER_SIZE        4096
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;


// for new_eeprom.c new_rtc.c new_uart.c
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s2;
I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi2_rx;
DMA_HandleTypeDef hdma_spi3_rx;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
HAL_StatusTypeDef I2S_Status;
typedef struct
{
  uint32_t   ChunkID;       /* 0 */
  uint32_t   FileSize;      /* 4 */
  uint32_t   FileFormat;    /* 8 */
  uint32_t   SubChunk1ID;   /* 12 */
  uint32_t   SubChunk1Size; /* 16*/
  uint16_t   AudioFormat;   /* 20 */
  uint16_t   NbrChannels;   /* 22 */
  uint32_t   SampleRate;    /* 24 */

  uint32_t   ByteRate;      /* 28 */
  uint16_t   BlockAlign;    /* 32 */
  uint16_t   BitPerSample;  /* 34 */
  uint32_t   SubChunk2ID;   /* 36 */
  uint32_t   SubChunk2Size; /* 40 */

}WAVE_FormatTypeDef;

typedef struct {
  int32_t offset;
  uint32_t fptr;
}Audio_BufferTypeDef;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_I2S2_Init(void);
static void MX_I2S3_Init(void);
static void MX_I2C2_Init(void);


// for new_eeprom.c new_rtc.c new_uart.c
static void MX_I2C1_Init(void);

void handle_recording(void);

/* USER CODE BEGIN PFP */
void I2C_CODEC_Settings(void);
void SD_Card(void);
void message(const char *format, ...);
void configure_codec(void);
void CODEC_RST(void);
void TS472IQT_EN(void);
void History_update(void);
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct);
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct);
static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t* pHeader);
int Start_Recording(void);
int stop_recording(void);

void printtime(void);



HAL_StatusTypeDef I2C1_ClockConfig(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char buffer[100];
int indx = 0;
int Select_SD[] = {1,0,0,0};
int number = 0;
//uint16_t audio_data[50000];
//uint16_t audio_data_buffer[10000];

extern FATFS fs;  // file system
extern FIL fil; // File
extern FILINFO fno;
extern FRESULT fresult;  // result
extern UINT br, bw;  // File read/write count

/*	Local Files	*/
FATFS fs_local;          // File system object
FIL file_1,file_2;        // File object
FRESULT res;		    // Result code
UINT bw_1,bw_2;         	  // Bytes written
FILINFO fno_local;
UINT Wr_len_1, Wr_len_2;
const char *text = "Hello, FATFS!\r\n";
char File_name_1[32];
char File_name_2[32];
char file_path[32];

uint16_t audio_buffer_2[AUDIO_BUFFER_SIZE];  // DMA buffer for audio data	ADC 3&4
uint16_t audio_buffer_1[AUDIO_BUFFER_SIZE];  // DMA buffer for audio data	ADC 1&2

int save_frequency = 0;
int stop_saving = 0;


extern rtc_timedate_t alarm_queue[MAX_ALARMS]; // Alarm queue
extern uint8_t alarm_count;              // Number of alarms
extern uint8_t current_idx;              // Current alarm index

volatile uint8_t alarm_triggered_flag = 0; // Flag set by EXTI interrupt


volatile uint8_t is_recording = 0; // 0: not recording, 1: recording

volatile uint8_t mode = 0; // 0: debug, 1: config


WAVE_FormatTypeDef WaveFormat;
Audio_BufferTypeDef  BufferCtl;

uint32_t WaveRecStatus = 0;
uint32_t byteswritten = 0;

uint8_t pHeaderBuff[44];
uint16_t WrBuffer[WR_BUFFER_SIZE];

long file_size = 0;

typedef struct {
    uint8_t reg_address;
    uint8_t value;
} CodecConfig;

// Adjusted codec configuration to reduce sensitivity
CodecConfig codec_configs[] = {
    {0x00, 0x01}, // Power up and release reset
    {0x01, 0x02}, // MCLKIN = 256 × Fs (Assuming 12.288 MHz for 16 kHz Fs)
    {0x04, 0x3F}, // Enable ADC1 and ADC2 (ADC_EN1 = 1, ADC_EN2 = 1); disable ADC3/ADC4
    {0x05, 0x01}, // I2S mode, 16-bit data, (SDATA_FMT = 00)
    {0x06, 0x11}, // SDATAOUT1, stereo output, no TDM
    {0x07, 0x32}, // Some configuration
    {0x08, 0x10}, // Some configuration
    {0x09, 0xF0}, // Some configuration
	{0x0A, 0x50}, /*		ADC 1(U22) 	*/
	{0x0B, 0x50}, /*		ADC 2(U21) 	*/
	{0x0C, 0x50}, /*		ADC 3(U20) 	*/
	{0x0D, 0x50}, /*		ADC 4(U19)	*/
    {0x0E, 0x00}, // Disable summing, independent channels
	//{0x19, 0x0F},
    {0x1A, 0xFF},  // Some configuration
};


// Define the I2C device address
#define I2C_DEVICE_ADDR (0x71 << 1)

// Define the registers with names and reset values
typedef struct {
    uint8_t address;
    const char *name;
    uint8_t reset_value;
} Register;

const Register registers[] = {
    {0x00, "M_POWER", 0x00},
    {0x01, "PLL_CONTROL", 0x41},
    {0x04, "BLOCK_POWER_SAI", 0x3F},
    {0x05, "SAI_CTRL0", 0x02},
    {0x06, "SAI_CTRL1", 0x00},
    {0x07, "SAI_CMAP12", 0x10},
    {0x08, "SAI_CMAP34", 0x32},
    {0x09, "SAI_OVERTEMP", 0xF0},
    {0x0A, "POSTADC_GAIN1", 0xA0},
    {0x0B, "POSTADC_GAIN2", 0xA0},
    {0x0C, "POSTADC_GAIN3", 0xA0},
    {0x0D, "POSTADC_GAIN4", 0xA0},
    {0x0E, "MISC_CONTROL", 0x02},
    {0x19, "ASDC_CLIP", 0x00},
    {0x1A, "DC_HPF_CAL", 0x00}
};
/**
 *
Reg		Used		Used		Reg Name	Bit 7		Bit 6		Bit 5	Bit 4	Bit 3	Bit 2	Bit 1	Bit 0	Reset	Access
0x00	0x0000 0001	0x01	M_POWER	S_RST	RESERVED						PWUP	0x00	RW
0x01	0x0100 0010	0x42	PLL_CONTROL		PLL_LOCK	PLL_MUTE	RESERVED	CLK_S	RESERVED	MCS			0x41	RW
0x04	0x0011 1111	0x3F	BLOCK_POWER_SAI	LR_POL		BCLKEDGE	LDO_EN	VREF_EN	ADC_EN4	ADC_EN3	ADC_EN2	ADC_EN1	0x3F	RW
0x05	0x0000 0001	0x01	SAI_CTRL0		SDATA_FMT	SAI			FS			0x02	RW
0x06	0x1001 0001	0x91	SAI_CTRL1		SDATA_SEL	SLOT_WIDTH		DATA_WIDTH	LR_MODE	SAI_MSB	BCLKRATE	SAI_MS	0x00	RW
0x07	0x0011 0010	0x32	SAI_CMAP12		CMAP_C2				CMAP_C1				0x10	RW
0x08	0x0001 0000	0x10	SAI_CMAP34		CMAP_C4				CMAP_C3				0x32	RW
0x09						SAI_OVERTEMP	SAI_DRV_C4	SAI_DRV_C3	SAI_DRV_C2	SAI_DRV_C1	DRV_HIZ	RESERVED		OT	0xF0	RW
0x0A						POSTADC_GAIN1	PADC_GAIN1								0xA0	RW
0x0B						POSTADC_GAIN2	PADC_GAIN2								0xA0	RW
0x0C						POSTADC_GAIN3	PADC_GAIN3								0xA0	RW
0x0D						POSTADC_GAIN4	PADC_GAIN4								0xA0	RW
0x0E	0x0000 0011	0x03	MISC_CONTROL	SUM_MODE	RESERVED	MMUTE		RESERVED			DC_CAL	0x02	RW
0x19						ASDC_CLIP		RESERVED				ADC_CLIP4	ADC_CLIP3	ADC_CLIP2	ADC_CLIP1	0x00	RW
0x1A	0x1111 1111	0xFF	DC_HPF_CAL		DC_SUB_C4	DC_SUB_C3	DC_SUB_C2	DC_SUB_C1	DC_HPF_C4	DC_HPF_C3	DC_HPF_C2	DC_HPF_C1	0x00	RW
 *
 * */
// Number of registers
#define NUM_REGISTERS (sizeof(registers) / sizeof(registers[0]))

// Function to read and print register values
void read_and_print_registers(void);

/**********************************************************************	GAIN Control	************/
// ADAU1978 I2C address
#define ADAU1978_I2C_ADDR (0x71 << 1)  // I2C Address
// AGC parameters
#define TARGET_AMPLITUDE 30000  // Target RMS value for AGC
#define AGC_STEP_SIZE    1      // Gain step size
#define MAX_GAIN         60     // Maximum gain in dB
#define MIN_GAIN         -30      // Minimum gain in dB
uint8_t current_gain = 0;  // Start with a mid-level gain

// ADAU1978 Gain Register Address (example)
#define ADAU1978_GAIN_REG 0xD0  // Replace with actual gain control register

void set_adau1978_gain(uint8_t gain_db) {
    uint8_t gain_value = gain_db;  // Assuming 1 dB steps
    message("Message: Gain Setting to: %x\r\n", gain_value);
    HAL_I2C_Mem_Write(&hi2c2, ADAU1978_I2C_ADDR, ADAU1978_GAIN_REG, I2C_MEMADD_SIZE_8BIT, &gain_value, 1, HAL_MAX_DELAY);
}

uint16_t compute_rms(uint16_t* buffer, uint16_t length) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i < length; i++) {
        sum += buffer[i] * buffer[i];
    }
    return (uint16_t)sqrt(sum / length);
}

void auto_gain_adjustment(uint16_t* buffer, uint16_t length, uint8_t* current_gain) {
    // Calculate RMS of the buffer
    uint16_t rms = compute_rms(buffer, length);

    // Adjust gain based on target amplitude
    if (rms < TARGET_AMPLITUDE) {
        if (*current_gain < MAX_GAIN) {
            (*current_gain) += AGC_STEP_SIZE;
            set_adau1978_gain(*current_gain);
        }
    } else if (rms > TARGET_AMPLITUDE) {
        if (*current_gain > MIN_GAIN) {
            (*current_gain) -= AGC_STEP_SIZE;
            set_adau1978_gain(*current_gain);
        }
    }
}

/**********************************************************************	GAIN Control end	************/

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_SDIO_SD_Init();
  MX_I2S2_Init();
  MX_I2S3_Init();
  MX_I2C2_Init();


  // for new_eeprom.c new_rtc.c new_uart.c
  if (I2C1_ClockConfig() != HAL_OK) {
          Error_Handler(); // Handle clock config failure
      }
  MX_I2C1_Init();





  /* USER CODE BEGIN 2 */
  HAL_Delay(1000);

  TS472IQT_EN();	/*	4 Preamp has to set to 1	*/

  CODEC_RST();		/*	PE1 For Codec Reset		*/

  configure_codec();


  HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxbuffer[rxindex], 1);
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxbuffer[rxindex], 1);

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

  	rtc_timedate_t timedate = {1, 1, 0, 1, 1, 01, 0xFF};
      ret = rtc_settime(&timedate);
      HAL_Delay(100);
      if (ret == HAL_OK) {
          HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set ok\n\r", 17, HAL_MAX_DELAY);
      } else {
          HAL_UART_Transmit(&huart3, (uint8_t*)"rtc time set notok\n\r", 20, HAL_MAX_DELAY);
      }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  switch(mode)
	  {

	  case 0:

		  // Check if this is the first entry into Mode 0
		  static uint8_t first_entry = 1;
		  if(first_entry)
		  {
			  /*char config_buffer[RX_buffersize];
			            //eeprom_read_and_print_string(&huart3, 0x0000, (uint8_t*)config_buffer, pagesize);
			  if (eeprom_read_config(config_buffer, RX_buffersize) == HAL_OK) {
			            	//if(eeprom_read(0x0000, (uint8_t*)config_buffer, 256)==HAL_OK)

				  HAL_UART_Transmit(&huart3, (uint8_t*)"Restoring: ", 11, HAL_MAX_DELAY);
				  HAL_UART_Transmit(&huart3, (uint8_t*)config_buffer, strlen(config_buffer), HAL_MAX_DELAY);
				  HAL_UART_Transmit(&huart3, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);
				  parse_and_print(config_buffer);
			  } else {
				  HAL_UART_Transmit(&huart3, (uint8_t*)"No config\n\r", 11, HAL_MAX_DELAY);
			  }*/

			  // Read previous configuration from EEPROM
			        char config_buffer[RX_buffersize];
			        memset(config_buffer, 0, RX_buffersize); // Clear the buffer
			        if (eeprom_read(0x0000, (uint8_t*)config_buffer, RX_buffersize) != NULL)
			        {
			          // Check if the data is valid (not all 0xFF or empty)
			          if (strlen(config_buffer) > 0 && config_buffer[0] != 0xFF && config_buffer[0] != 0x00)
			          {
			            HAL_UART_Transmit(&huart3, (uint8_t*)"Restoring previous configuration: ", 33, HAL_MAX_DELAY);
			            HAL_UART_Transmit(&huart3, (uint8_t*)config_buffer, strlen(config_buffer), HAL_MAX_DELAY);
			            HAL_UART_Transmit(&huart3, (uint8_t*)"\n\r", 2, HAL_MAX_DELAY);

			            // Parse the configuration to restore alarms (id=2)
			            parse_and_print(config_buffer);
			          }
			          else
			          {
			            HAL_UART_Transmit(&huart3, (uint8_t*)"No valid previous configuration found\n\r", 39, HAL_MAX_DELAY);
			          }
			        }
			        else
			        {
			          HAL_UART_Transmit(&huart3, (uint8_t*)"EEPROM read failed during init\n\r", 32, HAL_MAX_DELAY);
			        }

			  first_entry = 0; // Only run this once when entering Mode 0
		  }



	  if (alarm_triggered_flag) {
	              handle_recording();

	              // Manage alarm queue (optional)
	              if (current_idx < alarm_count - 1) {
	                  current_idx++;
	                  if (rtc_setalarm(&alarm_queue[current_idx]) == HAL_OK) {
	                      sprintf(uart_buffer, "Next alarm set: %02d:%02d:%02d\n\r",
	                              alarm_queue[current_idx].hour, alarm_queue[current_idx].minute, alarm_queue[current_idx].second);
	                      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	                  } else {
	                      HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to set next alarm\n\r", 26, HAL_MAX_DELAY);
	                  }
	              }

	              alarm_triggered_flag = 0; // Clear the flag after handling
	          } else if (is_recording) {
	              // Continue checking save_frequency even if no new alarm
	              handle_recording();
	          }


	  ret = rtc_gettime(&timedate);
	  if (ret == HAL_OK) {
# if DE_BUG
		  sprintf(uart_buffer, "current time = %02d:%02d:%02d\n\r", timedate.hour, timedate.minute, timedate.second);
		  HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
#endif
	  }
	  else {
		  HAL_UART_Transmit(&huart3, (uint8_t*)"rtc get time not ok\n\r", 21, HAL_MAX_DELAY);
		  return 1;
	  }
	  HAL_Delay(1000);
	  break;

	  case 1:
		  if(datarecived)
		  	  {
		  		  //uint8_t eeprom_data[RX_buffersize];
		  		  senddata(rxbuffer, strlen(rxbuffer)); // Send received data

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

		  		        	  	                	  	      parse_and_print((char*)rxbuffer);



		  		  /*ret=eeprom_write(0x0000, (uint8_t*)&rxbuffer, strlen(rxbuffer)+1);
		  		  if(ret==HAL_OK)
		  		  {
		  			  HAL_UART_Transmit(&huart3, (uint8_t*)"write ok\n\r", 10, HAL_MAX_DELAY);
		  			  HAL_Delay(100);

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
		  		  strcpy(rxCopy, (char*)eeprom_data);*/


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

		  		  //parse_and_print((char*)rxCopy);
		  		  mode = 0; // Switch to Debug Mode after writing config
		  		  HAL_UART_Transmit(&huart3, (uint8_t*)"Switching to Debug Mode\n\r", 25, HAL_MAX_DELAY);


		  		  datarecived = 0;
		  		  rxindex = 0;
		  		  memset(rxbuffer, 0, RX_buffersize); // Clear buffer
		  		  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rxbuffer[rxindex], 1);
		  		  HAL_UART_Receive_IT(&huart2, (uint8_t*)&rxbuffer[rxindex], 1);
		  	  }
		  break;

	  default:
	          HAL_UART_Transmit(&huart3, (uint8_t*)"Invalid mode\n\r", 14, HAL_MAX_DELAY);
	          mode = 1; // Reset to config mode if something goes wrong
	          break;


	  }

  /* USER CODE END 3 */
}
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
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_SLAVE_RX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_16K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

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
//  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;

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
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
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
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);


  /*Configure GPIO pins : PE4 PE5 PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD15 PD3
                           PD4 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA9 PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



  /*Configure GPIO pin : PD0 */

  GPIO_InitStruct.Pin = GPIO_PIN_0;

  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;

  GPIO_InitStruct.Pull = GPIO_PULLUP;

  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);


  /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);


    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);


/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */






HAL_StatusTypeDef I2C1_ClockConfig(void) {
    // Enable the clock for I2C1 (APB1 peripheral)
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Optional: Verify clock is enabled by checking the RCC register
    if (__HAL_RCC_I2C1_IS_CLK_ENABLED()) {
        HAL_UART_Transmit(&huart3, (uint8_t*)"I2C1 clock enabled\n\r", 20, HAL_MAX_DELAY);
        return HAL_OK;
    } else {
        HAL_UART_Transmit(&huart3, (uint8_t*)"I2C1 clock enable failed\n\r", 26, HAL_MAX_DELAY);
        return HAL_ERROR;
    }
}


void handle_recording(void)
{
    if (!is_recording) {
        if (Start_Recording() == 0) {
            is_recording = 1;
            save_frequency = 0; // Reset save_frequency to start fresh
            HAL_UART_Transmit(&huart3, (uint8_t*)"Recording started due to alarm\n\r", 32, HAL_MAX_DELAY);
        } else {
            HAL_UART_Transmit(&huart3, (uint8_t*)"Failed to start recording\n\r", 27, HAL_MAX_DELAY);
        }
    }

    // Debug save_frequency
        //sprintf(uart_buffer, "save_frequency = %lu\n\r", save_frequency);
        //HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);


    // Check save_frequency condition
    if (is_recording && save_frequency >= 1880) {
        stop_recording();
        is_recording = 0;
        //HAL_UART_Transmit(&huart3, (uint8_t*)"Recording stopped (save_frequency >= 1880)\n\r", 44, HAL_MAX_DELAY);
        HAL_Delay(100); // Safe in main loop context
    }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	static uint32_t last_trigger_time = 0;
	    uint32_t current_time = HAL_GetTick();

    if (GPIO_Pin == GPIO_PIN_0) {
        if (rtc_check_alarm_flag() == 1) {
            alarm_triggered_flag = 1; // Raise flag for main loop
            rtc_clearalarm();         // Clear the alarm flag
            HAL_UART_Transmit(&huart3, (uint8_t*)"Alarm interrupt triggered!\n\r", 28, HAL_MAX_DELAY);
        }
    }

    else if (GPIO_Pin == GPIO_PIN_2) { // EXTI2 for PE2

    	if ((current_time - last_trigger_time) < 50) {
    	            return;
    	        }

    	        last_trigger_time = current_time;

            if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) == GPIO_PIN_RESET) {
                mode = 1; // VCC (high)
                HAL_UART_Transmit(&huart3, (uint8_t*)"EXTI2: Mode set to 1\n\r", 28, HAL_MAX_DELAY);
            } else {
                mode = 0; // Ground (low)
                HAL_UART_Transmit(&huart3, (uint8_t*)"EXTI2: Mode set to 0\n\r", 28, HAL_MAX_DELAY);
            }
        }

}




int Start_Recording(void)
{
    /********************************************************************************************************** SD card ***/
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // OE SD 1 and 2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET); // OE SD 3 and 4
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);

    /********************************************************************************************************** SD 1 ***/
    message("Message: %s\r\n", "________________SD Card 1___________________________\r\n");
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // OE Pin pulled low SD 1 and 2
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // SEL Pin pulled low SD 1

    message("Message: %s\r\n", "________________Started to Stor 2 files in SD Card 1___________________________\r\n");

    fresult = f_mount(&fs_local, "/", 1);
    if (fresult != FR_OK) {
        message("ERROR!!! in mounting SD CARD...\r\n");
        return -1;
    }

    History_update();


    sprintf(File_name_1, "file_%d_1.wav", number);
    sprintf(File_name_2, "file_%d_2.wav", number);

    fresult = f_stat(File_name_1, &fno_local);
    fresult = f_stat(File_name_2, &fno_local);
    if (fresult == FR_OK) {
        message("ERROR!!! *%s* already exists!!!!\r\n use Update_File \r\n", "FILE1.txt");
        return -1;
    } else {
        fresult = f_open(&file_1, File_name_1, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
        if (fresult != FR_OK) {
            message("ERROR!!! No. %d in creating file *%s*\r\n", fresult, File_name_1);
            return -1;
        }
        fresult = f_open(&file_2, File_name_2, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
        if (fresult != FR_OK) {
            message("ERROR!!! No. %d in creating file *%s*\r\n", fresult, File_name_2);
            return -1;
        }
        fresult = f_close(&file_1);
        fresult = f_close(&file_2);
        if (fresult != FR_OK) message("ERROR No. %d in closing file *%s*\r\n", fresult, File_name_1);
    }

    f_open(&file_1, File_name_1, FA_OPEN_APPEND | FA_WRITE);
    f_open(&file_2, File_name_2, FA_OPEN_APPEND | FA_WRITE);

    //read_and_print_registers();

    WavProcess_EncInit(DEFAULT_AUDIO_IN_FREQ, pHeaderBuff);
    f_write(&file_1, pHeaderBuff, 44, (void *)&byteswritten);
    f_write(&file_2, pHeaderBuff, 44, (void *)&byteswritten);

    memset(audio_buffer_1, 0, sizeof(audio_buffer_1));
    memset(audio_buffer_2, 0, sizeof(audio_buffer_2));

    message("Message: %s\r\n", "________________ Recording Started _____________\r\n");

    // Stop any ongoing DMA
    HAL_I2S_DMAStop(&hi2s2);
    HAL_I2S_DMAStop(&hi2s3);

    // Check I2S state before starting
    HAL_I2S_StateTypeDef i2s2_state = HAL_I2S_GetState(&hi2s2);
    HAL_I2S_StateTypeDef i2s3_state = HAL_I2S_GetState(&hi2s3);
    sprintf(uart_buffer, "I2S2 State before start: %d\n\r", i2s2_state);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "I2S3 State before start: %d\n\r", i2s3_state);
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    // Start I2S DMA
    if (HAL_I2S_Receive_DMA(&hi2s2, audio_buffer_1, AUDIO_BUFFER_SIZE) != HAL_OK) {
        message("Message: %s\r\n", "________________HAL_I2S_Receive_DMA Error I2S2 !!!!__!!__\r\n");
        sprintf(uart_buffer, "I2S2 Error: %d\n\r", HAL_I2S_GetError(&hi2s2));
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        Error_Handler();
        return -1;
    }
    if (HAL_I2S_Receive_DMA(&hi2s3, audio_buffer_2, AUDIO_BUFFER_SIZE) != HAL_OK) {
        message("Message: %s\r\n", "________________HAL_I2S_Receive_DMA Error I2S3 !!!!__!!__\r\n");
        sprintf(uart_buffer, "I2S3 Error: %d\n\r", HAL_I2S_GetError(&hi2s3));
        HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
        Error_Handler();
        return -1;
    }

    // Confirm DMA started
    sprintf(uart_buffer, "I2S2 State after start: %d\n\r", HAL_I2S_GetState(&hi2s2));
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "I2S3 State after start: %d\n\r", HAL_I2S_GetState(&hi2s3));
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);\

    stop_saving = 0;  // Reset stop_saving for callbacks
    BufferCtl.fptr = 0;  // Reset file pointer

    return 0;
}
int stop_recording(void)
{
    HAL_I2S_DMAStop(&hi2s2);
    HAL_I2S_DMAStop(&hi2s3);

    // Debug I2S state after stopping
    sprintf(uart_buffer, "I2S2 State after stop: %d\n\r", HAL_I2S_GetState(&hi2s2));
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
    sprintf(uart_buffer, "I2S3 State after stop: %d\n\r", HAL_I2S_GetState(&hi2s3));
    HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);

    //read_and_print_registers();

    // Update WAV headers
    f_lseek(&file_1, 0);
    f_lseek(&file_2, 0);
    WavProcess_HeaderUpdate(pHeaderBuff, &WaveFormat);
    f_write(&file_1, pHeaderBuff, 44, (void*)&byteswritten);
    f_write(&file_2, pHeaderBuff, 44, (void*)&byteswritten);

    f_close(&file_1);
    f_close(&file_2);

    fresult = f_mount(NULL, "/", 1);
    if (fresult != FR_OK) {
        message("ERROR!!! in UNMOUNTING SD CARD\r\n");
        return -1;
    }

    message("\r\nDATA Saved and SD Card un mounted____Check____\r\n");

    stop_saving = 1;  // Moved after file operations
    save_frequency = 0;  // Reset here is fine, but handle_recording resets it too

    HAL_Delay(200);  // Reduced delay, adjust if needed

    return 0;
}
// DMA transfer complete callback
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s) {
	if(!stop_saving){
	    if (hi2s->Instance == SPI2) {
	        // Save second half of the buffer to the file
			if (f_write(&file_1, &audio_buffer_1[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2 * sizeof(uint16_t), &Wr_len_1) != FR_OK) {
				 message("Error: Failed to write to file_1\n\r");
				Error_Handler();  // File write failed
			}

	    }
	    if (hi2s->Instance == SPI3) {
			if (f_write(&file_2, &audio_buffer_2[AUDIO_BUFFER_SIZE / 2], AUDIO_BUFFER_SIZE / 2 * sizeof(uint16_t), &Wr_len_2) != FR_OK) {
				 message("Error: Failed to write to file_2\n\r");
				Error_Handler();  // File write failed
			}
		}
	    save_frequency ++;
	    BufferCtl.fptr += Wr_len_1;

	    /*if (save_frequency % 50 == 0) {  // Log every 50 increments
	                sprintf(uart_buffer, "save_frequency = %lu\n\r", save_frequency);
	                HAL_UART_Transmit(&huart3, (uint8_t*)uart_buffer, strlen(uart_buffer), HAL_MAX_DELAY);
	            }*/

	}
}

// DMA transfer half-complete callback
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s) {
	if(!stop_saving){
		if (hi2s->Instance == SPI2) {
	        // Save first half of the buffer to the file
				if (f_write(&file_1, &audio_buffer_1[0], AUDIO_BUFFER_SIZE / 2 * sizeof(uint16_t), &Wr_len_1) != FR_OK) {
					Error_Handler();  // File write failed
				}
	    }
	    if (hi2s->Instance == SPI3) {
	        // Save first half of the buffer to the file
				if (f_write(&file_2, &audio_buffer_2[0], AUDIO_BUFFER_SIZE / 2 * sizeof(uint16_t), &Wr_len_2) != FR_OK) {
					Error_Handler();  // File write failed
	    	}
	    }
	}

}

void configure_codec() {
    uint8_t i2c_tx;
    uint8_t i2c_rbf;

    if (HAL_I2C_IsDeviceReady(&hi2c2, (0x71 << 1), 5, 1000) != HAL_OK) {
        message("Message: %s\r\n", "_____I2C_DeviceReady Not Happy!!!_____\r\n");
    }
    HAL_Delay(500);

    // Loop through the codec configurations
    for (size_t i = 0; i < sizeof(codec_configs) / sizeof(codec_configs[0]); i++) {
        i2c_tx = codec_configs[i].value;
        if (HAL_I2C_Mem_Write(&hi2c2, (0x71 << 1), codec_configs[i].reg_address, 1, &i2c_tx, 1, 1000) != HAL_OK) {
            message("Message: %s\r\n", "_____I2C_I2C_Mem_Write Not Happy!!!_____\r\n");
        }
        HAL_Delay(100);
    }

    // Read back from register 0x06
    if (HAL_I2C_Mem_Read(&hi2c2, (0x71 << 1), 0x06, 1, &i2c_rbf, 1, 1000) == HAL_OK) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
        i2c_rbf = 0x00;

        HAL_Delay(10);
    } else {
        message("Message: %s\r\n", "_____I2C_I2C_Mem_Read Not Happy Check (*)!!!_____\r\n");
    }

//    read_and_print_registers();
}

void SD_Card(void)
{
	  HAL_Delay(100);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);// OE	SD 1 and 2
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);// OE	SD 3 and 4
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
	  HAL_Delay(100);


	  /**********************************************************************************************************	SD 1	***/
	  if(Select_SD[0] == 1){
		  message("Message: %s\r\n", "________________SD Card 1___________________________\r\n");
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	// OE  Pin pulled low		SD 1 and 2
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);	// SEL Pin pulled low		SD 1
		  HAL_Delay(100);

		  Mount_SD("/");
		  Format_SD();
		  Create_File("FILE1.TXT");
		  Create_File("FILE2.TXT");
		  Unmount_SD("/");

		  Mount_SD("/");
		  sprintf(buffer, "Hello ---> %d\n", indx);
		  Update_File("FILE1.TXT", buffer);
		  sprintf(buffer, "world ---> %d\n", indx);
		  Update_File("FILE2.TXT", buffer);
		  Unmount_SD("/");
		  indx++;

	  }
	  /**********************************************************************************************************	SD 2	***/
	  if(Select_SD[1] == 1){
		  message("Message: %s\r\n", "________________SD Card 2___________________________\r\n");
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);	// OE  Pin pulled low		SD 1 and 2
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);	// SEL Pin pulled low		SD 2
		  HAL_Delay(100);

		  Mount_SD("/");
		  Format_SD();
		  Create_File("FILE1.TXT");
		  Create_File("FILE2.TXT");
		  Unmount_SD("/");

		  Mount_SD("/");
		  sprintf(buffer, "Hello ---> %d\n", indx);
		  Update_File("FILE1.TXT", buffer);
		  sprintf(buffer, "world ---> %d\n", indx);
		  Update_File("FILE2.TXT", buffer);
		  Unmount_SD("/");
		  indx++;

		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);// OE	SD 1 and 2
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_Delay(100);
	  }
	  /**********************************************************************************************************	SD 3	***/
	  if(Select_SD[2] == 1){
		  message("Message: %s\r\n", "________________SD Card 3___________________________\r\n");
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);	// OE  Pin pulled low		SD 3 and 4
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);	// SEL Pin pulled low		SD 3
		  HAL_Delay(100);

		  Mount_SD("/");
		  Format_SD();
		  Create_File("FILE1.TXT");
		  Create_File("FILE2.TXT");
		  Unmount_SD("/");

		  Mount_SD("/");
		  sprintf(buffer, "Hello ---> %d\n", indx);
		  Update_File("FILE1.TXT", buffer);
		  sprintf(buffer, "world ---> %d\n", indx);
		  Update_File("FILE2.TXT", buffer);
		  Unmount_SD("/");
		  indx++;

	  }
	  /**********************************************************************************************************	SD 4	***/
	  if(Select_SD[3] == 1){
		  message("Message: %s\r\n", "________________SD Card 4___________________________\r\n");
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);	// OE  Pin pulled low		SD 3 and 4
		  HAL_Delay(100);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);	// SEL Pin pulled low		SD 4
		  HAL_Delay(100);

		  Mount_SD("/");
		  Format_SD();
		  Create_File("FILE1.TXT");
		  Create_File("FILE2.TXT");
		  Unmount_SD("/");

		  Mount_SD("/");
		  sprintf(buffer, "Hello ---> %d\n", indx);
		  Update_File("FILE1.TXT", buffer);
		  sprintf(buffer, "world ---> %d\n", indx);
		  Update_File("FILE2.TXT", buffer);
		  Unmount_SD("/");
		  indx++;
	  }
}

void CODEC_RST(void){
  message("Message: %s\r\n", "_____I2C_I2C CODEC Reset_____\r\n");
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);	/*	Settle time is around 0.82 S	*/
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_Delay(1000);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_Delay(1000);	/*	Need to Sart State machine	*/

}

void TS472IQT_EN(void){
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(100);
}


// Function to read and print register values
void read_and_print_registers() {
    uint8_t i2c_rbf; // Buffer to store the read byte
    HAL_StatusTypeDef ret;

    message("Reg\tName\t\tRead Value\tReset Value\r\n");
    message("----------------------------------------------------------\r\n");

    for (uint8_t i = 0; i < NUM_REGISTERS; i++) {
        uint8_t reg = registers[i].address;
        ret = HAL_I2C_Mem_Read(&hi2c2, I2C_DEVICE_ADDR, reg, 1, &i2c_rbf, 1, 1000);

        if (ret == HAL_OK) {
        	message("0x%02X\t%-15s\t0x%02X\t\t0x%02X\r\n", reg, registers[i].name, i2c_rbf, registers[i].reset_value);
        } else {
        	message("0x%02X\t%-15s\tError\t\t0x%02X\r\n", reg, registers[i].name, registers[i].reset_value);
        }
    }

    message("----------------------------------------------------------\r\n");
}

void History_update(void)
{

	fresult = f_stat ("History.txt", &fno_local);	/*	Check For File Presence	*/
	if (fresult == FR_OK){
		message("History!!! History exists!!!! \r\n");
	}
	else {
		message("History!!! Missing !!!\r\n");
		f_open(&file_1, "History.txt", FA_WRITE | FA_CREATE_ALWAYS);
		sprintf(buffer, "%d", number);
		f_write(&file_1, buffer, strlen(buffer), &bw);
		f_close(&file_1);
	}
	// Open the file for reading
	res = f_open(&file_1, "History.txt", FA_READ);
	if (res == FR_OK) {
		// Read data from the file
		res = f_read(&file_1, buffer, sizeof(buffer) - 1, &br);
		if (res == FR_OK && br > 0) {
			buffer[br] = '\0';           // Null-terminate the string
			number = atoi(buffer);       // Convert string to integer
			number++;                    // Increment the number
			message("Experiment Number: %d\r\n", number);
		} else {
			message("Failed to read from file.\r\n");
		}
		f_close(&file_1); // Close the file

		/*	write	*/

	// Open the file for writing to update the number
		res = f_open(&file_1, "History.txt", FA_WRITE | FA_CREATE_ALWAYS);
		if (res == FR_OK) {
			// Convert the incremented number back to a string and write to the file
			sprintf(buffer, "%d", number);
			res = f_write(&file_1, buffer, strlen(buffer), &bw);
			if (res == FR_OK) {
	//			message("Updated History.txt with number: %d\r\n", number);
			} else {
				message("Failed to write to file.\r\n");
			}
			f_close(&file_1); // Close the file
		} else {
			message("Failed to open History.txt for writing.\r\n");
		}

	} else {
		message("Failed to open History.txt.\r\n");
	}


}


/**
  * @brief  Encoder initialization.
  * @param  Freq: Sampling frequency.
  * @param  pHeader: Pointer to the WAV file header to be written.
  * @retval 0 if success, !0 else.
  */

static uint32_t WavProcess_EncInit(uint32_t Freq, uint8_t* pHeader)
{
  /* Initialize the encoder structure */
  WaveFormat.SampleRate = Freq;        /* Audio sampling frequency */
  WaveFormat.NbrChannels = 2;          /* Number of channels: 1:Mono or 2:Stereo */
  WaveFormat.BitPerSample = 16;        /* Number of bits per sample (16, 24 or 32) */
  WaveFormat.FileSize = 0x001D4C00;    /* Total length of useful audio data (payload) */
  WaveFormat.SubChunk1Size = 44;       /* The file header chunk size */
  WaveFormat.ByteRate = (WaveFormat.SampleRate * \
                        (WaveFormat.BitPerSample/8) * \
                         WaveFormat.NbrChannels);     /* Number of bytes per second  (sample rate * block align)  */
  WaveFormat.BlockAlign = WaveFormat.NbrChannels * \
                         (WaveFormat.BitPerSample/8); /* channels * bits/sample / 8 */

  /* Parse the wav file header and extract required information */
  if(WavProcess_HeaderInit(pHeader, &WaveFormat))
  {
    return 1;
  }
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderInit(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write chunkID, must be 'RIFF'  ------------------------------------------*/
  pHeader[0] = 'R';
  pHeader[1] = 'I';
  pHeader[2] = 'F';
  pHeader[3] = 'F';

  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = 0x00;
  pHeader[5] = 0x4C;
  pHeader[6] = 0x1D;
  pHeader[7] = 0x00;

  /* Write the file format, must be 'WAVE' -----------------------------------*/
  pHeader[8]  = 'W';
  pHeader[9]  = 'A';
  pHeader[10] = 'V';
  pHeader[11] = 'E';

  /* Write the format chunk, must be'fmt ' -----------------------------------*/
  pHeader[12]  = 'f';
  pHeader[13]  = 'm';
  pHeader[14]  = 't';
  pHeader[15]  = ' ';

  /* Write the length of the 'fmt' data, must be 0x10 ------------------------*/
  pHeader[16]  = 0x10;
  pHeader[17]  = 0x00;
  pHeader[18]  = 0x00;
  pHeader[19]  = 0x00;

  /* Write the audio format, must be 0x01 (PCM) ------------------------------*/
  pHeader[20]  = 0x01;
  pHeader[21]  = 0x00;

  /* Write the number of channels, ie. 0x01 (Mono) ---------------------------*/
  pHeader[22]  = pWaveFormatStruct->NbrChannels;
  pHeader[23]  = 0x00;

  /* Write the Sample Rate in Hz ---------------------------------------------*/
  /* Write Little Endian ie. 8000 = 0x00001F40 => byte[24]=0x40, byte[27]=0x00*/
  pHeader[24]  = (uint8_t)((pWaveFormatStruct->SampleRate & 0xFF));
  pHeader[25]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 8) & 0xFF);
  pHeader[26]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 16) & 0xFF);
  pHeader[27]  = (uint8_t)((pWaveFormatStruct->SampleRate >> 24) & 0xFF);

  /* Write the Byte Rate -----------------------------------------------------*/
  pHeader[28]  = (uint8_t)((pWaveFormatStruct->ByteRate & 0xFF));
  pHeader[29]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 8) & 0xFF);
  pHeader[30]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 16) & 0xFF);
  pHeader[31]  = (uint8_t)((pWaveFormatStruct->ByteRate >> 24) & 0xFF);

  /* Write the block alignment -----------------------------------------------*/
  pHeader[32]  = pWaveFormatStruct->BlockAlign;
  pHeader[33]  = 0x00;

  /* Write the number of bits per sample -------------------------------------*/
  pHeader[34]  = pWaveFormatStruct->BitPerSample;
  pHeader[35]  = 0x00;

  /* Write the Data chunk, must be 'data' ------------------------------------*/
  pHeader[36]  = 'd';
  pHeader[37]  = 'a';
  pHeader[38]  = 't';
  pHeader[39]  = 'a';

  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  pHeader[40]  = 0x00;
  pHeader[41]  = 0x4C;
  pHeader[42]  = 0x1D;
  pHeader[43]  = 0x00;

  /* Return 0 if all operations are OK */
  return 0;
}

/**
  * @brief  Initialize the wave header file
  * @param  pHeader: Header Buffer to be filled
  * @param  pWaveFormatStruct: Pointer to the wave structure to be filled.
  * @retval 0 if passed, !0 if failed.
  */
static uint32_t WavProcess_HeaderUpdate(uint8_t* pHeader, WAVE_FormatTypeDef* pWaveFormatStruct)
{
  /* Write the file length ----------------------------------------------------*/
  /* The sampling time: this value will be be written back at the end of the
     recording operation.  Example: 661500 Bytes = 0x000A17FC, byte[7]=0x00, byte[4]=0xFC */
  pHeader[4] = (uint8_t)(BufferCtl.fptr);
  pHeader[5] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[6] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[7] = (uint8_t)(BufferCtl.fptr >> 24);
  /* Write the number of sample data -----------------------------------------*/
  /* This variable will be written back at the end of the recording operation */
  BufferCtl.fptr -=44;
  pHeader[40] = (uint8_t)(BufferCtl.fptr);
  pHeader[41] = (uint8_t)(BufferCtl.fptr >> 8);
  pHeader[42] = (uint8_t)(BufferCtl.fptr >> 16);
  pHeader[43] = (uint8_t)(BufferCtl.fptr >> 24);
  /* Return 0 if all operations are OK */
  return 0;
}

void message(const char *format, ...) {
    char *buf = malloc(100 * sizeof(char)); // Allocate memory for the buffer
    if (buf != NULL) { // Check if malloc was successful
        va_list args;
        va_start(args, format);
        vsnprintf(buf, 100, format, args); // Format the string safely
        va_end(args);
        Send_Uart(buf); // Send the formatted string over UART
        free(buf);      // Free the allocated memory
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
