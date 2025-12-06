/*
 * display_oled.c
 *
 *  Created on: Dec 5, 2025
 *      Author: shawn
 */


#include "display_oled.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "state.h"
#include "string.h"
#include <stdio.h>

void OLED_task(void *pvParameters) {
    SystemState_t *sys = &sys_state;

    float prev_temp = 0.0f;
	float prev_humidity = 0.0f;
    uint16_t prev_pwm = SPEED_LOW;
	enum Mode prev_mode = MODE_AUTO;

	for (;;) {
		//  Non-blocking take. If mutex is busy, keep using previous cached values.
		if (xSemaphoreTake(state_mutex, 0) == pdTRUE) {
			prev_temp = sys->temperature;
			// humidity
			prev_humidity = sys->humidity;
			prev_pwm = sys->fan_pwm;
			prev_mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}
		char temp_str[20], humi_str[20];
		sprintf(temp_str, "%.2f", prev_temp);
		sprintf(humi_str, "%.0f%%", prev_humidity);
		ssd1306_print(prev_pwm, temp_str, humi_str, prev_mode);
		vTaskDelay(100);
	}
}
