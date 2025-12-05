/*
 * bt.c
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */
#include "bt.h"
#include "state.h"
#include "fan_control.h"
#include "log_queue.h"
#include "main.h"
#include "queue.h"
#include "semphr.h"
#include <string.h>
#include <stdio.h>



#define Q_SIZE 128
QueueHandle_t btQueue;
uint8_t rxData;

typedef struct {
	SystemState_t *sys;
	int set_pwm;
	int set_temp;
} bt_cmd_context_t ;

typedef void (*bt_cmd_handler_t)(bt_cmd_context_t *bct);

typedef struct {
	char *cmd;
	bt_cmd_handler_t bt_handler;
} bt_cmd_entry_t;


void bt_set_fan_state(SystemState_t *sys, uint8_t enable, uint16_t pwm, enum Mode mode)
{
	sys->fan_enable = enable;
	sys->fan_pwm    = pwm;
	sys->mode       = mode;
}

#define BT_CMD_MODE(name, enable, pwm, mode) \
	void bt_handler_##name(bt_cmd_context_t *bct) { \
	    bt_set_fan_state(bct->sys, enable, pwm, mode); \
	}

BT_CMD_MODE(on, 1, SPEED_LOW, MODE_MANUAL)
BT_CMD_MODE(off, 0, SPEED_LOW, MODE_MANUAL)
BT_CMD_MODE(low, 1, SPEED_LOW, MODE_MANUAL)
BT_CMD_MODE(mid, 1, SPEED_MID, MODE_MANUAL)
BT_CMD_MODE(high, 1, SPEED_HIGH, MODE_MANUAL)
BT_CMD_MODE(auto, 0, SPEED_LOW, MODE_AUTO)

void bt_handler_set_pwm(bt_cmd_context_t *bct) {
	int pwm = bct->set_pwm;
	if (pwm < 0 || pwm > 2000)
	    pwm = 0;
	bt_set_fan_state(bct->sys, 1, pwm, MODE_MANUAL);
}

static const bt_cmd_entry_t bt_cmd_table[] = {
	{.cmd = "on", .bt_handler = bt_handler_on},
	{.cmd = "off", .bt_handler = bt_handler_off},
	{.cmd = "low", .bt_handler = bt_handler_low},
	{.cmd = "mid", .bt_handler = bt_handler_mid},
	{.cmd = "high", .bt_handler = bt_handler_high},
	{.cmd = "pwm", .bt_handler = bt_handler_set_pwm},
	{.cmd = "auto", .bt_handler = bt_handler_auto}
};

#define BT_CMD_TABLE_SIZE (sizeof(bt_cmd_table) / sizeof(bt_cmd_table[0]))

int s_to_int(char *str) {
	int num = 0;
	while (*str) {
		if (*str < '0' || *str > '9')
			return -1;
		num = num * 10  + (*str - '0');
		str++;
	}
	return num;
}

void bt_process_string(SystemState_t *sys, char *str) {
	char *cur = str;
	while (*cur) {
		if (*cur >= 'A' && *cur <= 'Z') {
			*cur = (*cur - 'A' + 'a');
		}
		cur++;
	}

	log_record(LOG_SRC_BT, LOG_TYPE_INFO, "receive cmd =\"%s\"", str);

	char *num = NULL;
	cur = str;
	while (*cur && *cur != ' ') {
		cur++;
	}

	if (*cur  == ' ') {
		*cur = '\0';
		cur++;
		while (*cur == ' ') {
			cur++;
		}
		if (*cur != '\0')
			num = cur;
	}

	int pwm_ = 0;
	if (num != NULL) {
		int tmp = s_to_int(num);
		if (tmp > 0)
			pwm_ = tmp;
	}


	 bt_cmd_context_t bct = {
	     .sys      = sys,
		 .set_pwm  = pwm_,
		 .set_temp = 0,
	  };

    int flag = 0;
	for (int i = 0; i < BT_CMD_TABLE_SIZE; ++i) {
		if (!strcmp(str, bt_cmd_table[i].cmd)) {
			if (xSemaphoreTake(state_mutex, portMAX_DELAY) == pdTRUE) {
				bt_cmd_table[i].bt_handler(&bct);
				xSemaphoreGive(state_mutex);
				flag = 1;
			}
			break;
		}
	}

	if (!flag) {
		log_record(LOG_SRC_BT, LOG_TYPE_WARN, "unknown cmd=\"%s\"", str);
	}
}

void BtReceive_task(void *pvParameters) {
	SystemState_t *sys = &sys_state;
	char str[20];
	int cur = 0;
	for (;;) {
		// Block until the data is received in the　queue
		char cmd;
		if (xQueueReceive(btQueue, &cmd, portMAX_DELAY) == pdPASS) {
			if (cmd == '\r')
				continue;
			if (cmd == '\n') {
				str[cur] = '\0';
				bt_process_string(sys, str);
				cur = 0;
			} else {
				if (cur >= sizeof(str) - 1) {
					cur = 0;
					continue;
				}
				str[cur++] = cmd;
			}
		}
	}
}


void bt_init() {
	btQueue = xQueueCreate(Q_SIZE, sizeof(uint8_t));
	HAL_UART_Receive_IT(&huart3,&rxData,1);
}

void bt_uart_from_isr() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	uint8_t c = rxData;
	xQueueSendFromISR(btQueue, &c, &xHigherPriorityTaskWoken);

	// Restart the UART receive interrupt
	HAL_UART_Receive_IT(&huart3, &rxData, 1);

	// If a higher priority task was unblocked by xQueueSendFromISR
	// request an immediate context switch before exiting the ISR
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
