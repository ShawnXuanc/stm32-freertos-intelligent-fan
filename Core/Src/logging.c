/*
 * logging.c
 *
 *  Created on: Dec 5, 2025
 *      Author: shawn
 */
#include "main.h"
#include "logging.h"
#include "stdio.h"
#include "log_queue.h"


void format_time(char *buf, uint32_t tick_ms) {
    uint32_t ms = tick_ms % 1000;
    uint32_t sec = (tick_ms / 1000) % 60;
    uint32_t min = (tick_ms / 60000) % 60;
    uint32_t hr = tick_ms / 3600000;

    snprintf(buf, 16, "%02lu:%02lu:%02lu.%03lu", hr, min, sec, ms);
}


void Log_task(void *pvParameters)
{
    log_item_t item;
    char time_buf[16];
    for (;;) {
        if (log_pop(&item)) {
            format_time(time_buf, item.timestamp_ms);
            char buf[80];
            snprintf(buf, sizeof(buf),
                     "[%s] SRC=%s TYPE=%s MSG=%s\r\n",
                     time_buf,
                     log_src_str(item.src),
                     log_type_str(item.type),
                     item.msg);
            HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
        }

        vTaskDelay(100);
    }
}

