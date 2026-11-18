#include "tim.h"

#define TIM2EN		(1U<<0)
#define CR1_CEN		(1U<<0)
#define DIER_UIE 	(1U<<0)

volatile uint32_t tim2_ticks = 0;

void tim2_200hz_interrupt_init(void){
	/*Enable clock acces to tim2*/
	RCC->APB1ENR |= TIM2EN;

	/*Set prescaler value*/
	TIM2->PSC = 1600 - 1; //16 000 000 / 1 600 = 10 000

	/*Set auto-reload value*/
	TIM2->ARR = (PERIODO_MS * 10U) - 1; //10 000 / 50 = 200 Hz

	/*Clear counter*/
	TIM2->CNT = 0;

	TIM2->EGR |= TIM_EGR_UG;      // fuerza carga de PSC/ARR
	TIM2->SR &= ~SR_UIF;          // limpia la bandera que genera el UG

	/*Enable TIM interrupt*/
	TIM2->DIER |= DIER_UIE;

	/*Enable TIM interrupt in NVIC*/
	NVIC_EnableIRQ(TIM2_IRQn);

	/*Enable timer2*/
	TIM2->CR1 |= CR1_CEN;
}

//void TIM2_IRQHandler(void){
	//if(TIM2->SR & SR_UIF){
		/*Clear update interrupt flag*/
		//TIM2->SR &= ~SR_UIF;
		/*Do something*/
		//tim2_ticks++;
	//}
//}
