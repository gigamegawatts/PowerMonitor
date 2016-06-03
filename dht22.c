//////////////////////////////////////////////
// Copyright (c) GHI Electronics
/////////////////////////////////////////////

#include "stm32f1xx_hal.h"
/* includes --------------------------------------------------------- */
#include "dht22.h"
void Delay(uint32_t nTime); // in main.c

uint8_t waitForPinChange(uint8_t state, uint16_t timeout_us);
/* variables ---------------------------------------------------------------*/
//uint8_t DHT22data[6];
volatile uint32_t garbage;
uint8_t lastState, state, position;

int temperature;
int humidity;
uint16_t TIMEOUT_US = 1000;
GPIO_InitTypeDef GPIO_InitStructure;

// defined in main.c
extern TIM_HandleTypeDef htim1;

/* functions -------------------------------------------------------- */
/*
  configure DHT22_DATA as input
 */
void DHT22pinIn(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	  /*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = DHT22_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DHT22_GPIO, &GPIO_InitStruct);

}


/*
  configure DHT22_DATA as output
 */
void DHT22pinOut(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	  /*Configure GPIO pin : PA1 */
	GPIO_InitStruct.Pin = DHT22_DATA_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(DHT22_GPIO, &GPIO_InitStruct);
	
}


void DHT22_Init(void){

	// nothing to do
}

int DHT22GetData()
{
	
	uint64_t data = 0;

	// NOTE - this macro is defined in stm32f1xx_hal_tim.h
	__HAL_TIM_SET_COUNTER(&htim1, 0);


    DHT22pinOut();

	// NOTE - the following timings are based on the Adafruit code and/or the DHT22 datasheet
	// pull high for 2 seconds
	HAL_GPIO_WritePin(DHT22_GPIO, DHT22_DATA_PIN, GPIO_PIN_SET);
	HAL_Delay(2000);

  /* pull low for 20 ms */
	HAL_GPIO_WritePin(DHT22_GPIO, DHT22_DATA_PIN, GPIO_PIN_RESET);
	HAL_Delay(20);

  // pull high
	HAL_GPIO_WritePin(DHT22_GPIO, DHT22_DATA_PIN, GPIO_PIN_SET);
	//delayMicroseconds(40);

  /* switch to input and wait for sensor response */
  DHT22pinIn();

	__HAL_TIM_SET_COUNTER(&htim1, 0);

	/* sensor should do a low-high-low sequence before sending data */

	/* wait for sensor to pull pin low - should be 20 us, but we'll wait 100 */
	if (waitForPinChange(GPIO_PIN_RESET, 100) > 0)
	{
		return -1;
	}

		/* wait for sensor to pull pin high - - should be 80 us, but we'll wait 200 */
	if (waitForPinChange(GPIO_PIN_SET, 200) > 0)
	{
		return -2;
	}

	/* wait for sensor to pull pin low again - - should be 80 us, but we'll wait 200 */
	if (waitForPinChange(GPIO_PIN_RESET, 200) > 0)
	{
		return -2;
	}

	//delayMicroseconds(150);

  /* from this point on, data transitions are data */

	lastState = GPIO_PIN_RESET;
	position = 0;


	// reset microsecond counter
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	// loop until something breaks or returns
	while (1)
	{
		state = HAL_GPIO_ReadPin(DHT22_GPIO, DHT22_DATA_PIN);
		//Still same state then before, just check for timeout, then continue looping
		if (state == lastState)
		{
			if (__HAL_TIM_GET_COUNTER(&htim1) > TIMEOUT_US)
			{
				return (-1 * position - 10) ;
			} else
			{
				continue; // loop around
			}
		}

		// if state has changed to low, then sensor has finished sending a bit of data
		if (state == GPIO_PIN_RESET)
		{

			//If the pin was high for more then 40 us passed (technically 26-28), it was a 1
			data <<= 1;
			if (__HAL_TIM_GET_COUNTER(&htim1) > 40)
			{
				data++;
			}

			position++;
			//Stop after receiving 40 bits
			if (position == 40)
				break; // exit while loop
		}

		// reset counter and pin state
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		lastState = state;
	}

	//Not enough bits received
	if (position != 40)
		return -4;

	//Wrong checksum
	if ( (((data >> 32) + (data >> 24) + (data >> 16) + (data >> 8)) & 0xFF) != (data & 0xFF) )
	{
		return -5;
	}

	//Decode data
	humidity = (data >> 24 & 0xFFFF);
	//humidity = humidity / (float)10;

	temperature = (data >> 8  & 0x7FFF);// / (float)10;
	if ((data >> 8 & 0x8000) != 0)
		temperature = -temperature;

	//OK

	return 0;

}

int getHumidity()
{
		return humidity;
}

int getTemperature()
{
	return temperature;
}

void getTemperatureParts(int *t1, int *t2 )
{
	*t1 = temperature / 10;
	*t2 = temperature % 10;
}

void getHumidityParts(int *h1, int *h2 )
{
	*h1 = humidity / 10;
	*h2 = humidity % 10;
}


uint8_t waitForPinChange(uint8_t state, uint16_t timeout_us)
{
	uint16_t start_us = __HAL_TIM_GET_COUNTER(&htim1);
	while (HAL_GPIO_ReadPin(DHT22_GPIO, DHT22_DATA_PIN) != state)
	{
		if ((__HAL_TIM_GET_COUNTER(&htim1) - start_us) > timeout_us)
		{
			return 1;
		}
	}
	return 0;
}
