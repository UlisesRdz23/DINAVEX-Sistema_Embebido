/*
 * systick.c
 *
 *  Created on: 1 nov 2026
 *      Author: uliss
 */

#include "systick.h"

#define SYSTICK_LOAD_VAL	16000 - 1
#define CTRL_ENABLE			(1U<<0)
#define CTRL_CLKSRC			(1U<<2)
#define CTRL_COUNTFLAG		(1U<<16)

void systickDelaysMS(int delay){
	/*Reload with number of clocks per miliseconds*/
	SysTick->LOAD = SYSTICK_LOAD_VAL;
	/*Clear systick current value register*/
	SysTick->VAL = 0;
	/*Enable systick and select the internal clk src*/
	SysTick->CTRL = CTRL_ENABLE | CTRL_CLKSRC;

	for(int i = 0; i<delay; i++){
		/*Wait until the COUNTFLAG is set*/
		while((SysTick->CTRL & CTRL_COUNTFLAG) == 0){}
	}
	SysTick->CTRL = 0;
}
