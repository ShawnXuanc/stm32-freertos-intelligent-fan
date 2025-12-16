/*
 * dht11.h
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include "FreeRTOS.h"
#include "task.h"

#define ALPHA 0.2

typedef struct {
	float alpha;
	float value;
	uint8_t inited;
} ema_filter_t;

void DHT11_task(void *pvParameters);

#endif /* INC_DHT11_H_ */
