/*
 * ir.h
 *
 *  Created on: Dec 4, 2025
 *      Author: shawn
 */

#ifndef INC_IR_SENSOR_H_
#define INC_IR_SENSOR_H_

#include <stdint.h>

#define REG32(addr) (*(volatile uint32_t *)(addr))

#define GPIOA_BASE (0x40020000U)
#define RCC_BASE (0x40023800U)
#define GPIOA_MODER REG32(GPIOA_BASE + 0x00U)
#define GPIOA_IDR REG32(GPIOA_BASE + 0x10U)
#define RCC_AHB1ENR REG32(RCC_BASE + 0x30U)

void ir_init(void);
void IR_task(void *);

#endif /* INC_IR_SENSOR_H_ */
