/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
//#define DISPLAY128X64
#define DISPLAY128X32
#include "gmarduino.h"
#include "utils_stm32.h"
#include <string.h>
#include "u8g.h"
#include "u8g_arm.h"
#include <dht22.h>
#include <GMBufferedSerial.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

// ----- display ------
static u8g_t u8g;
#define MAMP_BUFFER_LEN 20
char mAmpDisplayBuffer[MAMP_BUFFER_LEN];
#define HT_BUFFER_LEN 20
char htDisplayBuffer[HT_BUFFER_LEN];
#define XBEE_BUFFER_LEN 25
char xBeeDisplayBuffer[XBEE_BUFFER_LEN];
#define TIME_BUFFER_LEN 10
char timeDisplayBuffer[XBEE_BUFFER_LEN];
#define UART_BUFFER_LEN 30
char uartBuffer[UART_BUFFER_LEN];
uint8_t displayCounter;
uint8_t displayType;

// ---- XBee and sensors -----
#define XBEE_ANALOG_PREFIX  "OA"
#define XBEE_TEMPERATURE_PREFIX "OT"
#define XBEE_HUMIDITY_PREFIX "OH"
#define XBEE_SEND_INTERVAL  60
#define AMPS_SAMPLE_INTERVAL  5
#define TEMP_HUM_SAMPLE_INTERVAL  15
#define TX_BUFFER_LEN 1000

char sendBuffer[TX_BUFFER_LEN];
uint8_t dmaRxBuffer;
uint32_t milliAmpTotal;
int16_t temperatureTotal;
uint16_t humidityTotal;
uint16_t numMilliAmpReadings;
uint16_t numTempHumReadings;
uint32_t nextAmpsTime;
uint32_t nextTHTime;
uint32_t nextSendTime;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint8_t addReadingToDisplayBuffer(int val, const char *suffix, char *buffer);
uint32_t calcMilliAmps(uint32_t analogRange);
void clearSensorTotals();
void draw();
static void Error_Handler(void);
void initADC(void);
int8_t readAnalogRange(uint32_t channel, uint16_t intervalMs, uint32_t *result);
void readMilliAmps();
void sendSensorReading(const char* prefix, uint16_t totalReading, uint16_t numReadings);
int sendXBee();
void processUARTData();
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int8_t readAnalogRange(uint32_t channel, uint16_t intervalMs, uint32_t *result)
{
	uint32_t maxValue = 0;
	uint32_t minValue = 4096;
	uint32_t endTime = HAL_GetTick() + intervalMs;
	HAL_ADC_Stop(&hadc1); // need to stop ADC before configuring channel
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = channel;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	while (HAL_GetTick() < endTime)
	{
		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
		{
			uint32_t val = HAL_ADC_GetValue(&hadc1);
			if (val > maxValue)
			{
				maxValue = val;
				
			}
			
			if (val < minValue)
			{
				minValue = val;
				
			}
		}
	}

	HAL_ADC_Stop(&hadc1);
	
	if (maxValue < minValue)
	{
		*result = 0;
		return -1;
	}
	else
	{
		*result = maxValue - minValue;
		return 0;
	}
	
}

void draw()
{
	u8g_uint_t x;
	u8g_uint_t y;
	char *displayText;
	
	if (displayType == 1 && strlen(timeDisplayBuffer) > 0)
	{
		displayText = timeDisplayBuffer;
		displayType = 0;
	}
	else
	{
		displayText = mAmpDisplayBuffer;
		displayType = 1;
	}
		
	if (displayCounter == 0)
	{
		x = 2;
		y = 20;
	}
	else if (displayCounter == 1)
	{
		x = 15;
		y = 25;
	}
	else
	{
		x = 30;
		y = 30;
	}


	u8g_FirstPage(&u8g);
	do
	{
#ifdef DISPLAY128X64		
		u8g_SetFont(&u8g, u8g_font_profont10);//set current font
		u8g_DrawStr(&u8g, x, 10, htDisplayBuffer);
		u8g_DrawStr(&u8g, x, 24, xBeeDisplayBuffer);
		u8g_SetFont(&u8g, u8g_font_fub20);//set current font
		u8g_DrawStr(&u8g, x, 54, displayText);//write string - you set coordinates and string
#endif
#ifdef DISPLAY128X32		
		//u8g_SetFont(&u8g, u8g_font_profont10);//set current font
		//u8g_DrawStr(&u8g, 10, 10, mAmpDisplayBuffer);
		u8g_SetFont(&u8g, u8g_font_fub20);//set current font
		u8g_DrawStr(&u8g, x, y, displayText);	
#endif		
	} while (u8g_NextPage(&u8g));
	
	displayCounter += 1;
	if (displayCounter > 2)
	{
		displayCounter = 0;
	}
	
}

void initADC()
{
	  /* Run the ADC calibration */  
	if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
	{
	  /* Calibration Error */
		Error_Handler();
	}
	HAL_Delay(1000);
	HAL_ADC_Start(&hadc1);
}

uint32_t calcMilliAmps(uint32_t analogRange)
{
	// convert to millivolts, then multiply by 1000 to get the final result in mA
	uint32_t numerator = analogRange * 3300 * 1000;
	// divide by analog scale, divide by 2 because ACS712 zero voltage is 5V/2, divide by RMS denominator and divider by mv per Amp
	uint32_t denominator = 4096 * 2 * 1.414 * 185;
	return numerator / denominator;
}

uint8_t addReadingToDisplayBuffer(int val, const char *suffix, char *buffer)
{
	int16_t num = val / 10;
	uint8_t dec = val % 10;
	strcpy(buffer, itoa(num));
	strcat(buffer, ".");
	strcat(buffer, itoa(dec));
	strcat(buffer, suffix);
	return strlen(buffer);
}

int readTempHum()
{
#ifdef __DHT22_H
	int rc = DHT22GetData();
	if (rc == 0)
	{
		int temp = getTemperature();
		int hum = getHumidity();
		temperatureTotal += temp;
		humidityTotal += hum;
		numTempHumReadings += 1;
		blink(250, 2);
		uint8_t len = addReadingToDisplayBuffer(temp, "C  ", htDisplayBuffer);
		addReadingToDisplayBuffer(hum, "%", htDisplayBuffer + len);
	}
	else
	{
		
		strcpy(htDisplayBuffer, "DHT22 rc=");
		strcat(htDisplayBuffer, itoa(rc));	
		blink(100, 5);
	}

	return rc;
#else
	return 0;
#endif
}

void sendSensorReading(const char* prefix, uint16_t totalReading, uint16_t numReadings)
{
	int16_t avgReading = totalReading / numReadings;
	int16_t num = avgReading / 10;
	uint8_t dec = avgReading % 10;
	strcat(sendBuffer, prefix);
	strcat(sendBuffer, ":");
	strcat(sendBuffer, itoa(num));
	strcat(sendBuffer, ".");
	strcat(sendBuffer, itoa(dec));
	strcat(sendBuffer, "\r\n");
}

int sendXBee()
{
	if (numMilliAmpReadings + numTempHumReadings == 0)
	{
		return 0; // nothing to send
	}
	//char * bufferOffset = sendBuffer;
	//char * endBuffer = sendBuffer + TX_BUFFER_LEN - 1; // allow for ending null
	sendBuffer[0] = 0;
	if (numMilliAmpReadings > 0)
	{
		
		// convert to Watts
		uint16_t avg = (milliAmpTotal * 120) / (numMilliAmpReadings * 1000);
		strcat(sendBuffer, XBEE_ANALOG_PREFIX);
		strcat(sendBuffer, ":");
		strcat(sendBuffer, itoa(avg));
		strcat(sendBuffer, "\r\n");

	}

	
	if (numTempHumReadings > 0)
	{
		sendSensorReading(XBEE_TEMPERATURE_PREFIX, temperatureTotal, numTempHumReadings);
		sendSensorReading(XBEE_HUMIDITY_PREFIX, humidityTotal, numTempHumReadings);

	}

	uint16_t sendLen = strlen(sendBuffer);
	uint16_t result = GMBuff_WriteData(sendBuffer, sendLen);
	if (result  == 0)
	{
	
		strcpy(xBeeDisplayBuffer, "UART err");
		return -1;
	}
	else
	{
		strcpy(xBeeDisplayBuffer, "UART OK");	
	}
//	  /*##-2- Start the transmission process #####################################*/  
//	
//#if UART_ASYNC	
//  /* While the UART in reception process, user can transmit data through 
//     "aTxBuffer" buffer */
//	if (HAL_UART_Transmit_IT(&huart2, (uint8_t*)sendBuffer, sendLen) != HAL_OK)
//	{
//		//Error_Handler();
//		return -1;
//	}
//  
//	/*##-3- Wait for the end of the transfer ###################################*/   
//	while (UartReady != SET)
//	{
//	}
//  
//	/* Reset transmission flag */
//	UartReady = RESET;
//#else
//	int rc = HAL_UART_Transmit(&huart2, (uint8_t*)sendBuffer, sendLen, 15000);
//	if (rc != HAL_OK)
//	{
//	
//		strcpy(xBeeDisplayBuffer, "UART rc=");
//		strcat(xBeeDisplayBuffer, itoa(rc));	
//		return -1;
//	}
//	else
//	{
//		strcpy(xBeeDisplayBuffer, "UART OK");	
//	}
//#endif

	return 0;
}

void readMilliAmps()
{
	uint32_t val;
	if (readAnalogRange(1, 1000, &val) < 0)
	{
		strcpy(mAmpDisplayBuffer, "Analog error");
		blink(100, 4);
	}
	val = calcMilliAmps(val);
	strcpy(mAmpDisplayBuffer, itoa(val));
	strncat(mAmpDisplayBuffer, " mA", MAMP_BUFFER_LEN - 3);
	milliAmpTotal += val;
	numMilliAmpReadings += 1;
	blink(250, 3);
}

void clearSensorTotals()
{
	milliAmpTotal = 0;
	temperatureTotal = 0;
	humidityTotal = 0;
	numMilliAmpReadings = 0;
	numTempHumReadings = 0;
}

void processUARTData()
{
	
	char *ptr, *lastptr;
	lastptr = uartBuffer;
	// all data should end with LF
	ptr = strchr(uartBuffer, 10);
	
	while (ptr != NULL)
	{
		// replace end of data with nulls
		*ptr = 0;
		// if there is a CR, replace it with null
		if (*(ptr - 1) == 13)
		{
			*(ptr - 1) = 0;
		}
		if (strlen(lastptr) > 3)
		{
			if (*(lastptr + 2) == ':')
			{
				// Time begins with "TI:"
				if (*lastptr == 'T' && *(lastptr + 1) == 'I')
				{
					strcpy(timeDisplayBuffer, lastptr + 3);
				}
			}
		}
		lastptr = ptr + 1;
		ptr = strchr(ptr + 1, 10);
	}

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
		// start TIM1
	HAL_TIM_Base_Start(&htim1);
	
#ifdef DISPLAY128X64	
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, u8g_com_hw_i2c_fn);
#endif	
#ifdef DISPLAY128X32	
	u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x32_i2c, u8g_com_hw_i2c_fn);
#endif	
	initADC();
	if (HAL_UART_Receive_DMA(&huart2, &dmaRxBuffer, 1) != HAL_OK)
	{
		Error_Handler();
	}
	
	// do initial read of sensors
	readTempHum();
	readMilliAmps();
	nextSendTime =	HAL_GetTick() + XBEE_SEND_INTERVAL * 1000;
	nextAmpsTime =	HAL_GetTick() + AMPS_SAMPLE_INTERVAL * 1000;
	nextTHTime =	HAL_GetTick() + TEMP_HUM_SAMPLE_INTERVAL * 1000;
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
#ifdef TIMER_TEST	  
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

			  // delay 500 ms
		HAL_Delay(500);

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); 
			  // delay 500 ms
		HAL_Delay(500);
#endif	

		if (HAL_GetTick() >= nextAmpsTime)
		{
			readMilliAmps();
			nextAmpsTime = HAL_GetTick() + AMPS_SAMPLE_INTERVAL * 1000;
		}
		
		if (HAL_GetTick() >= nextTHTime)
		{
			readTempHum();
			nextTHTime = HAL_GetTick() + TEMP_HUM_SAMPLE_INTERVAL * 1000;
		}
		
		if (HAL_GetTick() >= nextSendTime)
		{
			sendXBee();
			clearSensorTotals();
			nextSendTime =	HAL_GetTick() + XBEE_SEND_INTERVAL * 1000;
		}
		
		if (GMBuff_ReadUpToChar(uartBuffer, UART_BUFFER_LEN, 0x0a) > 0)
		{
			processUARTData();
		}
		
		draw();
		HAL_Delay(2000);
	
	}
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  HAL_ADC_Init(&hadc1);

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c1);

}

/* TIM1 init function */
void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim1);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOD_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DHT22_Pin|RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//	__HAL_UART_FLUSH_DRREGISTER(&huart2); // Clear the buffer to prevent overrun
	GMBuff_Enqueue(&RXq, &dmaRxBuffer, 1);  
}

/**
  * @brief  UART error callbacks
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
	Error_Handler();
}

static void Error_Handler(void)
{
  /* User may add here some code to deal with a potential error */
#ifdef DEBUGGING	
	puts("Error_Handler: game over.");
#endif	
  /* In case of error, LED3 is toggling at a frequency of 1Hz */
	while (1)
	{
		blink(100, 10);
	}
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
