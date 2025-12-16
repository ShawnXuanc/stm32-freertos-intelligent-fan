#include "fan_control.h"
#include "log_queue.h"
#include "main.h"
#include "semphr.h"
#include <stdio.h>

static float bound01(float x)
{
    if (x < 0) return 0;
    if (x > 1) return 1;
    return x;
}

static float ramp_up(float x, float x0, float x1)
{
    return bound01((x - x0) / (x1 - x0));
}

static float ramp_down(float x, float x0, float x1)
{
    return bound01((x1 - x) / (x1 - x0));
}

static float temp_cool_level(float t)
{
    return ramp_down(t, 22, 26);
}

static float temp_hot_level(float t)
{
    return ramp_up(t, 28, 32);
}

static float humid_dry_level(float h)
{
    return ramp_down(h, 40, 55);
}

static float humid_wet_level(float h)
{
    return ramp_up(h, 60, 80);
}

static float humid_comfy_level(float h)
{
    if (h <= 40 || h >= 70)
        return 0;

    if (h < 50)
        return ramp_up(h, 40, 50);

    if (h <= 60)
        return 1;

    return ramp_down(h, 60, 70);
}


static float temp_warm_level(float t)
{
    if (t <= 24 || t >= 30)
        return 0;

    if (t < 27)
        return ramp_up(t, 24, 27);

    return ramp_down(t, 27, 30);
}


static uint16_t cal_pwm_fuzzy(float temp, float humi)
{
    float t_cool = temp_cool_level(temp);
    float t_warm = temp_warm_level(temp);
    float t_hot  = temp_hot_level(temp);

    float h_dry   = humid_dry_level(humi);
    float h_comfy = humid_comfy_level(humi);
    float h_humid = humid_wet_level(humi);

    float w_low  = fmaxf(t_cool, h_dry);
    float w_mid  = fmaxf(t_warm, h_comfy);
    float w_high = fmaxf(t_hot,  h_humid);

    float sum = w_low + w_mid + w_high;
    if (sum < 0.001f)
        return SPEED_LOW;

    float pwm_f =
        (w_low  * SPEED_LOW +
         w_mid  * SPEED_MID +
         w_high * SPEED_HIGH) / sum;

    return (uint16_t)pwm_f;
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

static void change_fan_state_auto(SystemState_t *s)
{
    if (s->mode != MODE_AUTO)
        return;

    s->fan_enable = s->ir_state ? 1 : 0;
    if (!s->fan_enable)
        return;

    s->fan_pwm = cal_pwm_fuzzy(s->temperature, s->humidity);
}


uint16_t pwm_stabilize(uint16_t prev, uint16_t next) {
	int diff = (int)next - (int)prev;
	if (diff < 0)
		diff = -diff;
	return diff >= PWM_UPDATE_THRESHOLD ? next : prev;
}

void FanControl_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;

	uint8_t  last_enable = sys->fan_enable;
	uint16_t last_pwm = sys->fan_pwm;
	enum Mode last_mode = sys->mode;

	for (;;) {
		SystemState_t local_sys;
		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			local_sys = *sys;
			xSemaphoreGive(state_mutex);
		}

		uint16_t prev_pwm = local_sys.fan_pwm;

		change_fan_state_auto(&local_sys);
		local_sys.fan_pwm = pwm_stabilize(prev_pwm, local_sys.fan_pwm);

		if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
			// Apply auto result only if mode is still AUTO,
			// ensuring MANUAL updates take priority
			if (sys->mode == MODE_AUTO) {
				sys->fan_enable = local_sys.fan_enable;
				sys->fan_pwm = local_sys.fan_pwm;
			}

			local_sys.fan_enable = sys->fan_enable;
			local_sys.fan_pwm = sys->fan_pwm;
			local_sys.mode = sys->mode;
			xSemaphoreGive(state_mutex);
		}


		if (local_sys.fan_enable != last_enable ||
			local_sys.fan_pwm != last_pwm ||
			local_sys.mode != last_mode) {

			log_record(LOG_SRC_FAN, LOG_TYPE_INFO,
					  "state changed: mode=%s en=%u pwm=%u",
					  local_sys.mode == MODE_AUTO ? "AUTO" : "MANUAL",
					  local_sys.fan_enable, local_sys.fan_pwm);

			last_enable = local_sys.fan_enable;
			last_pwm = local_sys.fan_pwm;
			last_mode = local_sys.mode;
		}


		if (local_sys.fan_enable) {
			turn_on_PWM(local_sys.fan_pwm);
		} else {
			turn_off_PWM();
		}

		vTaskDelay(100);
	}
}
