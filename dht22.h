
#ifndef __DHT22_H
#define __DHT22_H

#define DHT22_DATA_PIN GPIO_PIN_4
#define DHT22_GPIO		GPIOA
// DAW - NOTE that TIM2 is assumed to be running at 8 Mhz
#define DHT22_TIM		TIM1

void TIM2Init(void);
void DHT22pinIn(void);
void DHT22pinOut(void);
void DHT22_Init(void);
int DHT22GetData(void);
int getHumidity(void);
int getTemperature(void);
void getHumidityParts(int *h1, int *h2 );
void getTemperatureParts(int *t1, int *t2 );
#endif
