#ifndef __UTILS_STM32_H
#define __UTILS_STM32_H

#include "stm32f1xx_hal.h"

void blink(uint32_t periodms, uint8_t numBlinks);
void delayMicroseconds(uint16_t micros);
char *itoa(int i);
uint8_t reverseByte(uint8_t x);
#endif