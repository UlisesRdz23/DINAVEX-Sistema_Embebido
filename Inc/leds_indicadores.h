/*
 * leds_indicadores.h
 *
 *  Created on: 15 nov 2026
 *      Author: uliss
 */

#ifndef LEDS_INDICADORES_H_
#define LEDS_INDICADORES_H_

#include "stm32f4xx.h"
#include "systick.h"

void leds_init(void);
void leds_indicadores(char l);
void secuencia_buzzer(void);

#endif /* LEDS_INDICADORES_H_ */
