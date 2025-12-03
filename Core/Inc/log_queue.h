/*
 * log_queue.h
 *
 *  Created on: Dec 2, 2025
 *      Author: shawn
 */

#ifndef INC_LOG_QUEUE_H_
#define INC_LOG_QUEUE_H_

#include <stdint.h>
#include <stdbool.h>


#define LOG_MSG_LEN 80


typedef enum {
    LOG_SRC_SYSTEM = 0,
    LOG_SRC_FAN,
    LOG_SRC_BT,
    LOG_SRC_SENSOR,
	LOG_SRC_DHT,
	LOG_SRC_OLED,
	LOG_SRC_MAX
} log_source_t;


typedef enum {
    LOG_TYPE_INFO = 0,
    LOG_TYPE_STATE,
	LOG_TYPE_WARN,
	LOG_TYPE_MAX
} log_type_t;

typedef struct {
	uint32_t timestamp_ms;
	uint8_t src;
	uint8_t type;
	char msg[LOG_MSG_LEN];
} log_item_t;


void log_init();
void log_push(log_source_t, log_type_t, char *);
int log_pop(log_item_t *);
void log_record(log_source_t, log_type_t, const char *, ...);
const char *log_src_str(log_source_t);
const char *log_type_str(log_type_t);

#endif /* INC_LOG_QUEUE_H_ */
