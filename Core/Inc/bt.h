/*
 * bt.h
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */

#ifndef INC_BT_H_
#define INC_BT_H_

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"


void bt_init();
void BtReceive_task(void *);
void bt_uart_from_isr();

#endif /* INC_BT_H_ */
