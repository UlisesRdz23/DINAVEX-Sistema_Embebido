/*
 * adc.h
 *
 *  Created on: 15 oct 2026
 *      Author: uliss
 */

#ifndef ADC_H_
#define ADC_H_
#include "stm32f4xx.h"
#include <stdint.h>

#define ADC_CH0 0U
#define ADC_CH1 1U
#define VREF_ADC 3.3f
#define	ADC_MAX 4095.0f
#define R1_DIVISOR 150000.0f
#define R2_DIVISOR 10000.0f
#define FACTOR_DIVISOR ((R1_DIVISOR + R2_DIVISOR) / R2_DIVISOR)

void adc1_init(void);
void start_conversion(void);
uint32_t adc_read(void);
uint32_t adc_leer_canal(uint32_t canal);

#endif /* ADC_H_ */
