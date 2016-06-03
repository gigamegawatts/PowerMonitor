#pragma once
#define QUEUE_SIZE 500


struct GMQueue {
	uint16_t pRD, pWR;
	uint8_t  q[QUEUE_SIZE]; 
};
	
uint16_t GMBuff_QueueFull(struct GMQueue *q);
uint16_t GMBuff_QueueEmpty(struct GMQueue *q);
uint16_t GMBuff_QueueAvail(struct GMQueue *q);
uint16_t GMBuff_Enqueue(struct GMQueue *q, const uint8_t *data, uint16_t len);
uint16_t GMBuff_Dequeue(struct GMQueue *q, uint8_t *data, uint16_t len);
uint16_t GMBuff_DequeueUpToChar(struct GMQueue *q, uint8_t *data, uint16_t len, uint8_t endChar);
uint16_t GMBuff_SendBuffer();
uint16_t GMBuff_FillBuffer();
uint16_t GMBuff_ReadData(uint8_t *data, uint16_t maxlen);
uint16_t GMBuff_WriteData(uint8_t *data, uint16_t len);
uint16_t GMBuff_PeekData(uint8_t *data, uint16_t maxlen);
uint16_t GMBuff_ReadAvail();
uint16_t GMBuff_ReadUpToChar(uint8_t *data, uint16_t len, uint8_t endChar);


struct GMQueue TXq, RXq;


