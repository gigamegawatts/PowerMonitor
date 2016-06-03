#include "stm32f1xx_hal.h"

// defined in main.c
extern TIM_HandleTypeDef htim1;	// advances every microsecond

void blink(uint32_t periodms, uint8_t numBlinks)
{
	for (int i = 0; i < numBlinks; i++)
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(periodms);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(periodms);
	}
}

// NOTE!! - this requires that TIM1 be configured with a 1 us tick (e.g. Prescalar of 8 and Clock of 8 Mhz) and Period > delay
void delayMicroseconds(uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	uint16_t start_us = __HAL_TIM_GET_COUNTER(&htim1);
	uint16_t current_us = start_us;
	while (current_us - start_us < delay)
	{
		current_us = __HAL_TIM_GET_COUNTER(&htim1);
	}
}

// from http://opensource.apple.com/source/groff/groff-10/groff/libgroff/itoa.c
#define INT_DIGITS 19		/* enough for 64 bit integer */

char *itoa(int i)
{
  /* Room for INT_DIGITS digits, - and '\0' */
	static char buf[INT_DIGITS + 2];
	char *p = buf + INT_DIGITS + 1;	/* points to terminating '\0' */
	if (i >= 0) {
		do {
			* --p = '0' + (i % 10);
			i /= 10;
		} while (i != 0);
		return p;
	}
	else {			/* i < 0 */
		do {
			* --p = '0' - (i % 10);
			i /= 10;
		} while (i != 0);
		* --p = '-';
	}
	return p;
}


// FROM http://omarfrancisco.com/reversing-bits-in-a-byte/
uint8_t reverseByte(uint8_t x)
{
	x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
	x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
	x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
	return x;
}

#ifdef OLD_HANGSWHENOPTIMIZED
void delayMicroseconds(uint32_t micros) {
#if !defined(STM32F0xx)
	// initialize the timer used to delay microseconds if not already initialized
	if ((DWT->CTRL & 1) == 0)
	{
		DWT->CTRL |= 0x01;
		DWT->CYCCNT = 0;
	}
	
	uint32_t start = DWT->CYCCNT;
    
	/* Go to number of cycles for system */
	micros *= (HAL_RCC_GetHCLKFreq() / 1000000);
    
	/* Delay till end */
	while ((DWT->CYCCNT - start) < micros)
		;
#else
	    /* Go to clock cycles */
	micros * = (SystemCoreClock / 1000000) / 5;
    
	/* Wait till done */
	while (micros--)
		;
#endif


//NOTE - the following code is taken from http://www.carminenoviello.com/2015/09/04/precisely-measure-microseconds-stm32/
#pragma GCC push_options
#pragma GCC optimize ("O3")
//NOTE!! - this will hang unless DWT->CTRL & 1
void delayMicroseconds(uint32_t us) {
	volatile uint32_t cycles = (SystemCoreClock / 1000000L)*us;
	volatile uint32_t start = DWT->CYCCNT;
	do {
	} while (DWT->CYCCNT - start < cycles);
}
#pragma GCC pop_options
#endif