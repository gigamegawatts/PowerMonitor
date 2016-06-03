// --------------------  I2C Utility functions -----------------------

#include "stm32f1xx_hal.h"
#include "gmi2c.h"

extern I2C_HandleTypeDef hi2c1; // in main.c
const int I2C_TIMEOUT_MS = 1000;


int8_t gm_read8(uint16_t devAddr, uint8_t regAddr, uint8_t *result)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &regAddr, 1, I2C_TIMEOUT_MS) == HAL_OK)
	{
		if (!HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, result, 1, I2C_TIMEOUT_MS) == HAL_OK) 
		{
			return -2;
		}
		return 0;
	}
	return -1;
}

int8_t gm_read16(uint16_t devAddr, uint8_t regAddr, uint16_t *result)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &regAddr, 1, I2C_TIMEOUT_MS) == HAL_OK)
	{
		if (!HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, (uint8_t *)result, 2, I2C_TIMEOUT_MS) == HAL_OK) 
		{
			return -2;
		}
		return 0;
	}
	return -1;
}

// reverse low-order and high-order bytes
int8_t gm_read16rev(uint16_t devAddr, uint8_t regAddr, uint16_t *result)
{
	uint8_t buffer[2];
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &regAddr, 1, I2C_TIMEOUT_MS) == HAL_OK)
	{
		if (!HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, (uint8_t *)buffer, 2, I2C_TIMEOUT_MS) == HAL_OK) 
		{
			return -2;
		}
		*result = 256u * buffer[0] + buffer[1]; 
		return 0;
	}
	return -1;
}

int8_t gm_write8(uint16_t devAddr, uint8_t regAddr, uint8_t val)
{
	uint8_t buff[2];
	buff[0] = regAddr;
	buff[1] = val;
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, buff, 2, I2C_TIMEOUT_MS) == HAL_OK)
	{
		return 0;
	}
	return -1;
}

int8_t gm_readmultiple(uint16_t devAddr, uint8_t regAddr, uint8_t *result, uint8_t bytesToRead)
{
	if (HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, &regAddr, 1, I2C_TIMEOUT_MS) == HAL_OK)
	{
		if (!HAL_I2C_Master_Receive(&hi2c1, devAddr << 1, (uint8_t *)result, bytesToRead, I2C_TIMEOUT_MS) == HAL_OK) 
		{
			return -2;
		}
		return 0;
	}
	return -1;
}

int8_t gm_writemultiple(uint16_t devAddr, uint8_t regAddr, uint8_t *val, uint8_t bytesToWrite)
{
	uint8_t buff[bytesToWrite + 1];
	buff[0] = regAddr;
	for (int i = 0; i < bytesToWrite; i++)
	{
		buff[i + 1] = val[i];
	}
	return HAL_I2C_Master_Transmit(&hi2c1, devAddr << 1, buff, bytesToWrite + 1, I2C_TIMEOUT_MS);
}