/*
 * state.h
 *
 *  Created on: Nov 27, 2025
 *      Author: shawn
 */

#ifndef INC_STATE_H_
#define INC_STATE_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"

enum Mode {
	MODE_AUTO,
	MODE_MANUAL
};

typedef struct SystemState {
	float temperature;
	float humidity;
	uint8_t ir_state;
	uint16_t fan_pwm;
	uint8_t fan_enable;
	enum Mode mode;
	uint32_t last_update_ms;

} SystemState_t;


#define SPEED_LOW   800
#define SPEED_MID  1450
#define SPEED_HIGH 1999


extern SystemState_t sys_state;
// include priority inheritance
extern SemaphoreHandle_t state_mutex;

void state_init();
#endif /* INC_STATE_H_ */
