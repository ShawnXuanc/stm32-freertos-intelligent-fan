/*
 * state.c
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */
#include "state.h"
#include "FreeRTOS.h"

SystemState_t sys_state = {
    .temperature    = 0,
	.humidity       = 0,
    .ir_state       = 0,
    .fan_pwm        = SPEED_LOW,
    .fan_enable     = 0,
    .mode           = MODE_AUTO,
    .last_update_ms = 0
};

SemaphoreHandle_t state_mutex = NULL;

void state_init() {
	state_mutex = xSemaphoreCreateMutex();
}
