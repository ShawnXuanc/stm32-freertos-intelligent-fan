/*
 * ir_sensor.c
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "state.h"
#include "ir_sensor.h"

void ir_init() {
	RCC_AHB1ENR |=  (1U << 0); // enable GPIOA clock
	GPIOA_MODER &= ~(3U << 0); // PA0 input
}

uint8_t ir_read_reg() {
	return (GPIOA_IDR & 1) == 0 ? 1 : 0;
}

void IR_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	for (;;) {
		uint8_t ir_ = ir_read_reg();
		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE){
			sys->ir_state = ir_;
			xSemaphoreGive(state_mutex);
		}
		vTaskDelay(100);
	}

}

