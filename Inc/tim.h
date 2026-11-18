/*
 * tim.h
 *
 *  Created on: 1 nov 2026
 *      Author: uliss
 */

#ifndef TIM_H_
#define TIM_H_

#include "stm32f4xx.h"
#include "robot_config.h"

#define SR_UIF (1U<<0)

extern volatile uint32_t tim2_ticks;

void tim2_200hz_interrupt_init(void);

#endif /* TIM_H_ */
