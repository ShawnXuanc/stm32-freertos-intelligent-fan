/*
 * fan_control.h
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */

#ifndef INC_FAN_CONTROL_H_
#define INC_FAN_CONTROL_H_

#include <stdint.h>
#include "state.h"
#include "FreeRTOS.h"
#include "task.h"

#define PWM_UPDATE_THRESHOLD 15

void FanControl_task(void *);
void turn_on_PWM(uint16_t );
void turn_off_PWM();

#endif /* INC_FAN_CONTROL_H_ */
