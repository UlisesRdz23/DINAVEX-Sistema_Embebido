/*
 * adc.c
 *
 *  Created on: 15 oct 2026
 *      Author: uliss
 */

#include "adc.h"

#define GPIOAEN (1U<<0)
#define ADC1ENR (1U<<8)
#define ADC_LEN1 0x00
#define CR2_ADON (1U<<0)
#define CR2_SWSTART (1U<<30)
#define SR_EOC (1U<<1)

/* adc configured with 3 channels
 * PA0 = CH0
 * PA1 = CH1
 * */

void adc1_init(void){
	/***Configure the ADC GPIO pin***/
	/*Enable clock access to GPIOA*/
	RCC->AHB1ENR |= GPIOAEN;
	/*Set the mode of PA0 and PA1 to analog mode*/
	GPIOA->MODER |= (1U<<0);
	GPIOA->MODER |= (1U<<1);

	GPIOA->MODER |= (1U<<2);
	GPIOA->MODER |= (1U<<3);

	/***Configure the ADC mode***/
	/*Enable clock access to ADC*/
	RCC->APB2ENR |= ADC1ENR;

	/*Configure ADC parameters*/
	/*Conversion sequence start*/
	//ADC1->SQR3 = ADC_CH1;
	/*Conversion sequence length*/
	ADC1->SQR1 = ADC_LEN1;
	/*Sampling time 480 cycles for CH0 and CH1 (111 = 480 cycles)*/
	ADC1->SMPR2 |= (7U<<0) | (7U<<3);
	/*Enable ADC module*/
	ADC1->CR2 |= CR2_ADON;
}

void start_conversion(void){
	/*Start ADC conversion*/
	ADC1->CR2 |= CR2_SWSTART;
}

uint32_t adc_read(void){
	/*Wait to conversion to complete*/
	while(! (ADC1->SR & SR_EOC)){}
	/*Read conversion*/
	return ADC1->DR;
}

uint32_t adc_leer_canal(uint32_t canal){
	/*Select the channel to convert*/
	ADC1->SQR3 = canal;
	start_conversion();
	return adc_read();
}
