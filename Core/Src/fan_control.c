#include "fan_control.h"
#include "log_queue.h"
#include "main.h"
#include "semphr.h"
#include <stdio.h>


static uint16_t cal_pwm_by_temp(float temp)
{
    if (temp >= 30) return SPEED_HIGH;
    if (temp >= 28) return SPEED_MID;
    return SPEED_LOW;
}

void turn_on_PWM(uint16_t pwm)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm);
}

void turn_off_PWM(void)
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

static void change_fan_state_auto(uint8_t ir_, float temp, enum Mode mode,
                                  uint8_t *fan_enable, uint16_t *pwm)
{
    if (mode != MODE_AUTO)
        return;

    *fan_enable = ir_ ? 1 : 0;
    if (!(*fan_enable))
        return;

    *pwm = cal_pwm_by_temp(temp);
}

void FanControl_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	uint8_t  last_enable = sys->fan_enable;
	uint16_t last_pwm    = sys->fan_pwm;
	enum Mode last_mode = sys->mode;

	for (;;) {
		uint8_t fan_enable;
		uint16_t pwm;
		uint8_t ir_;
		float temp;
		enum Mode mode;
		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			ir_ = sys->ir_state;
			temp = sys->temperature;
			fan_enable = sys->fan_enable;
			pwm = sys->fan_pwm;
			mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}

		change_fan_state_auto(ir_, temp, mode, &fan_enable, &pwm);

		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			// Apply auto result only if mode is still AUTO,
			// ensuring MANUAL updates take priority
			if (sys->mode == MODE_AUTO) {
				sys->fan_enable = fan_enable;
				sys->fan_pwm = pwm;
			}

			fan_enable = sys->fan_enable;
			pwm = sys->fan_pwm;
			mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}


		if (fan_enable != last_enable || pwm != last_pwm || mode != last_mode) {

			log_record(LOG_SRC_FAN, LOG_TYPE_INFO,
					  "state changed: mode=%s en=%u pwm=%u",
					  mode == MODE_AUTO ? "AUTO" : "MANUAL",
					  fan_enable, pwm);

			last_enable = fan_enable;
			last_pwm = pwm;
			last_mode = mode;
		}


		if (fan_enable) {
			turn_on_PWM(pwm);
		} else {
			turn_off_PWM();
		}

		vTaskDelay(100);
	}
}
