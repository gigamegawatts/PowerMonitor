
#include "stm32f1xx_hal.h"
#include "GMBufferedSerial.h"
// defined in main.c
extern UART_HandleTypeDef huart2;

//GMBuff_GMBufferedSerial(const char *pserialDevice, const unsigned int pbaud, const bool penableFlowControl)
//	: GMSerial(pserialDevice, pbaud, penableFlowControl)
//{
//	RXq.pRD = 0;
//	RXq.pWR = 0;
//	TXq.pRD = 0;
//	TXq.pWR = 0;
//}


uint16_t GMBuff_QueueFull(struct GMQueue *q)
{
	return (((q->pWR + 1) % QUEUE_SIZE) == q->pRD);
}

uint16_t GMBuff_QueueEmpty(struct GMQueue *q)
{
	return (q->pWR == q->pRD);
}

uint16_t GMBuff_QueueAvail(struct GMQueue *q)
{
	return (QUEUE_SIZE + q->pWR - q->pRD) % QUEUE_SIZE;
}

uint16_t GMBuff_Enqueue(struct GMQueue *q, const uint8_t *data, uint16_t len)
{
	uint16_t i;
	for (i = 0; !GMBuff_QueueFull(q) && (i < len); i++)
	{
		q->q[q->pWR] = data[i];
		q->pWR = ((q->pWR + 1) ==  QUEUE_SIZE) ? 0 : q->pWR + 1;
	}
	return i;
}

uint16_t GMBuff_Dequeue(struct GMQueue *q, uint8_t *data, uint16_t len)
{
	uint16_t i;
	for (i = 0; !GMBuff_QueueEmpty(q) && (i < len); i++)
	{
		data[i] = q->q[q->pRD];
		q->pRD = ((q->pRD + 1) ==  QUEUE_SIZE) ? 0 : q->pRD + 1;
	}
	return i;
}

uint16_t GMBuff_DequeueUpToChar(struct GMQueue *q, uint8_t *data, uint16_t len, uint8_t endChar)
{
	uint16_t i;
	uint16_t ptr = q->pRD;
	for (i = 0; !GMBuff_QueueEmpty(q) && (i < len); i++)
	{
		data[i] = q->q[ptr];
		ptr = ((ptr + 1) ==  QUEUE_SIZE) ? 0 : ptr + 1;
		if (data[i] == endChar)
		{
			q->pRD = ptr;
			return i;
		}

	}
	return 0;
}


uint16_t GMBuff_SendBuffer()
{
	uint8_t data;
	uint16_t bytesSent = 0;
	while (GMBuff_Dequeue(&TXq, &data, 1))
	{
		//WriteByte(data);
		HAL_UART_Transmit(&huart2, &data, 1, 15000);
		bytesSent += 1;
	}
	return bytesSent;
}

//uint16_t GMBuff_FillBuffer()
//{
//	uint8_t data;
//	uint16_t bytesRead = 0;
//	while (Available())
//	{
//		if (ReadByte(&data))
//		{
//			GMBuff_Enqueue(&RXq, &data, 1);
//			bytesRead += 1;
//		}
//		else
//		{
//			// shouldn't happen
//			return bytesRead;
//		}
//
//	}
//	return bytesRead;
//}

uint16_t GMBuff_ReadData(uint8_t *data, uint16_t maxlen)
{
	//FillBuffer();
	return GMBuff_Dequeue(&RXq, data, maxlen);
}

uint16_t GMBuff_WriteData(uint8_t *data, uint16_t len)
{
	GMBuff_Enqueue(&TXq, data, len);
	return GMBuff_SendBuffer();
}

uint16_t GMBuff_PeekData(uint8_t *data, uint16_t maxlen)
{
	uint16_t numbytes;
	//FillBuffer();
	uint16_t index = RXq.pRD;
	numbytes = GMBuff_Dequeue(&RXq, data, maxlen);
	RXq.pRD = index;
	return numbytes;
}

uint16_t GMBuff_ReadAvail()
{
	return GMBuff_QueueAvail(&RXq);
}

uint16_t GMBuff_ReadUpToChar(uint8_t *data, uint16_t len, uint8_t endChar)
{
	return GMBuff_DequeueUpToChar(&RXq, data, len, endChar);
}


