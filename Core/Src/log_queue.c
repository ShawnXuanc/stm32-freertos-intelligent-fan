#include "log_queue.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <string.h>

#define LOG_BUF_SIZE 80

typedef struct {
	log_item_t buf[LOG_BUF_SIZE];
	size_t head;
	size_t tail;
	uint32_t overflow_count;
	SemaphoreHandle_t mutex;
} log_queue_t;

static const char *log_src_table[LOG_SRC_MAX] = {
    "SYSTEM",
	"FAN",
	"BT",
	"SENSOR",
	"DHT",
	"OLED"
};


static const char *log_type_table[LOG_TYPE_MAX] = {
	"INFO",
	"STATE",
	"WARN"
};

static log_queue_t log_q;

void log_init() {
	log_q.head = 0;
	log_q.tail = 0;
	log_q.overflow_count = 0;
	log_q.mutex = xSemaphoreCreateMutex();
}

static inline int is_empty(void)
{
    return (log_q.head == log_q.tail);
}

static inline int is_full(void)
{
    size_t next = (log_q.head + 1) % LOG_BUF_SIZE;
    return (next == log_q.tail);
}

void log_push(log_source_t src, log_type_t type, char *msg) {
	if (xSemaphoreTake(log_q.mutex, 0) == pdTRUE) {

		size_t next_head = (log_q.head + 1) % LOG_BUF_SIZE;

		if (next_head == log_q.tail) {
		    log_q.tail = (log_q.tail + 1) % LOG_BUF_SIZE;
		    log_q.overflow_count++;
		}

		log_item_t *item = &log_q.buf[log_q.head];
		item->timestamp_ms = HAL_GetTick();
		item->src = src;
		item->type = type;

		memset(item->msg, 0, LOG_MSG_LEN);
		strncpy(item->msg, msg, LOG_MSG_LEN);

		log_q.head = next_head;
		xSemaphoreGive(log_q.mutex);
	}
}

int log_pop(log_item_t *item) {
	if (item == NULL)
		return 0;
	if (xSemaphoreTake(log_q.mutex, 0) != pdTRUE) {
		 return 0;
	}

	if (is_empty()) {
		xSemaphoreGive(log_q.mutex);
		return 0;
	}

	*item = log_q.buf[log_q.tail];
	log_q.tail = (log_q.tail + 1) % LOG_BUF_SIZE;

	xSemaphoreGive(log_q.mutex);
	return 1;
}

void log_record(log_source_t src, log_type_t type, const char *fmt, ...)
{
    char buf[LOG_MSG_LEN];

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    log_push(src, type, buf);
}

const char *log_src_str(log_source_t src) {
	return src < LOG_SRC_MAX ? log_src_table[src] : "UNKWON SRC";
}

const char *log_type_str(log_type_t type) {
	return type < LOG_TYPE_MAX ? log_type_table[type] : "UNKNOW TYPE";
}


