/*
 * dht11.c
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */
#include "main.h"
#include "dht11.h"
#include "state.h"
#include "semphr.h"

#define DHT11_PORT GPIOB
// PB9
#define DHT11_PIN  GPIO_PIN_9
#define PERIOD_MS  2000u

static uint32_t pMillis, cMillis;


void microDelay(uint16_t delay) {
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay)
    ;
}

uint8_t DHT11_Start(void) {
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT11_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 0);        // pull the pin low
  HAL_Delay(20);                                      // wait for 20ms
  HAL_GPIO_WritePin(DHT11_PORT, DHT11_PIN, 1);        // pull the pin high
  microDelay(30);                                     // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT11_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay(40);
  if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) {
	  microDelay(80);
	  if ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)))
      Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) && pMillis + 2 > cMillis) {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT11_Read(void) {
  uint8_t a = 0, b = 0;
  for (a = 0; a < 8; a++) {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();

    while (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) &&
           pMillis + 2 > cMillis) {
      cMillis = HAL_GetTick();
    }

    microDelay(40);                                 // wait for 40 us
    if (!(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN))) // if the pin is low
      b &= ~(1 << (7 - a));
    else
      b |= (1 << (7 - a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)) &&
           pMillis + 2 > cMillis) { // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

int check_dht11(float *temp, float *humidity) {
	if (DHT11_Start()) {
	      uint8_t RHI, RHD, TCI, TCD, SUM;
	      RHI = DHT11_Read(); // Relative humidity integral
	      RHD = DHT11_Read(); // Relative humidity decimal
	      TCI = DHT11_Read(); // Celsius integral
	      TCD = DHT11_Read(); // Celsius decimal
	      SUM = DHT11_Read(); // Check sum
	      if (RHI + RHD + TCI + TCD != SUM) {
	    	  return 0;
	      }
	      // Can use RHI and TCI for any purposes if whole number only needed
		  *temp = (float)TCI + (float)(TCD / 10.0);
		  *humidity = (float)RHI;

		  return 1;
	 }
	 return 0;
}

void ema_init(ema_filter_t *f, float alpha) {
	f->alpha = alpha;
	f->value = 0;
	f->inited = 0;
}

void ema_update(ema_filter_t *f, float sample) {
	if (!f->inited) {
		f->value = sample;
		f->inited = 1;
		return;
	}
	f->value = f->alpha * sample + (1 - f->alpha) * f->value;
}

void DHT11_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	HAL_TIM_Base_Start(&htim1);

	TickType_t lastWakeTime = xTaskGetTickCount();
	const TickType_t period = pdMS_TO_TICKS(PERIOD_MS);

	ema_filter_t temp_filter;
	ema_filter_t humi_filter;

	ema_init(&temp_filter, ALPHA);
	ema_init(&humi_filter, ALPHA);

	for (;;) {

		float temp, humidity;
		if (check_dht11(&temp, &humidity)) {

			ema_update(&temp_filter, temp);
			ema_update(&humi_filter, humidity);

			if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
				sys->temperature = temp_filter.value;
				sys->humidity = humi_filter.value;
				sys->last_update_ms = HAL_GetTick();
				xSemaphoreGive(state_mutex);
			}
		}
		// period wake up
		vTaskDelayUntil(&lastWakeTime, period);
	}
}
