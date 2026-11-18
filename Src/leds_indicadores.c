/*
 * leds_indicadores.c
 *
 *  Created on: 15 nov 2026
 *      Author: uliss
 */
#include "leds_indicadores.h"

#define GPIOBEN				(1U<<1) // 0b 0000 0000 0000 0000 0000 0000 0000 0001
#define GPIOCEN				(1U<<2)
#define LED_AZUL			(1U<<1) //PB1
#define LED_VERDE 			(1U<<5) //PB5
#define buzzer				(1U<<5) //PC5
#define LED_ROJO			(1U<<7) //PC7

void leds_init(void){
	/* 1. Enable clock access to GPIOA and GPIOB*/
	RCC->AHB1ENR |= GPIOBEN;
	RCC->AHB1ENR |= GPIOCEN;

	/* 2. Set  PB1, PB5, PC7 as output pin*/
	GPIOB->MODER |=	(1U<<2);	//Set bit 10 to 1
	GPIOB->MODER &=~(1U<<3);	//Set bit 11 to 0*/

	GPIOB->MODER |=	(1U<<10);	//Set bit 10 to 1
	GPIOB->MODER &=~(1U<<11);	//Set bit 11 to 0*/

	GPIOC->MODER |=	(1U<<10);	//Set bit 10 to 1
	GPIOC->MODER &=~(1U<<11);	//Set bit 11 to 0*/

	GPIOC->MODER |=	(1U<<14);	//Set bit 10 to 1
	GPIOC->MODER &=~(1U<<15);	//Set bit 11 to 0*/
}

void leds_indicadores(char l){
	if(l == 'a'){
		GPIOB->ODR ^= LED_AZUL;
	}else if(l == 'v'){
		GPIOB->ODR ^= LED_VERDE;
	}else if(l == 'r'){
		GPIOC->ODR ^= LED_ROJO;
	}
}

void secuencia_buzzer(void){
	for(int i = 0; i<20; i++){
		GPIOC->ODR ^= buzzer;
		systickDelaysMS(100);
	}
}
