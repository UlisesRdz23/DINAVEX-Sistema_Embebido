/*
 * uart.c
 *
 *  Created on: 12 oct 2026
 *      Author: uliss
 */
#include "uart.h"

#define GPIOAEN (1U<<0)
#define GPIOCEN (1U<<2)
#define UART2EN (1U<<17)
#define UART3EN (1U<<18)

#define CR1_RE (1U<<2)
#define CR1_TE (1U<<3)
#define CR1_UE (1U<<13)
#define CR1_RXNEIE (1U<<5)

#define DMA1EN	(1U<<21)
#define DMA_CR_EN	(1U<<0)
#define CHSEL4	(1U<<27)
#define DMA_MEM_INC (1U<<10)
#define DMA_DIR_MEM_TO_PERIPH (1U<<6)
#define UART_CR3_DMA (1U<<7)

#define DMA_CIRC (1U<<8)
#define UART_CR3_DMAR (1U<<6)
#define RX_DMA_TAM 128

#define SR_TXE	(1U<<7)
#define SR_RNXE (1U<<5)

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ

#define UART_BAUDRATE 115200

int UART_selec;

static volatile char rx_dma_buf[RX_DMA_TAM];
static uint16_t rx_leido = 0;

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk, uint32_t BaudRate);
static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate);
void uart2_write(int ch);

int __io_putchar(int ch){
	if(UART_selec == 2){
			uart2_write(ch);
		} else if (UART_selec == 3){
			uart3_write(ch);
		}
		return ch;
}

void uart2_rxtx_init(void){
	/****Configure uart gpio pin****/
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA2 mode to alternate function mode*/
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/*Set PA2 alterntive function type to UART_TX (AF07)*/
	GPIOA ->AFR[0] |= (1U<<8);
	GPIOA ->AFR[0] |= (1U<<9);
	GPIOA ->AFR[0] |= (1U<<10);
	GPIOA ->AFR[0] &=~ (1U<<11);

	/*Set PA3 mode to alternate function mode*/
	GPIOA->MODER &=~(1U<<6);
	GPIOA->MODER |= (1U<<7);

	/*Set PA3 alterntive function type to UART_RX (AF07)*/
	GPIOA ->AFR[0] |= (1U<<12);
	GPIOA ->AFR[0] |= (1U<<13);
	GPIOA ->AFR[0] |= (1U<<14);
	GPIOA ->AFR[0] &=~ (1U<<15);

	/****Configure uart module****/
	/*Enable clock access to uart2*/
	RCC->APB1ENR |= UART2EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	/*Configure the transfer directio*/
	USART2->CR1 = (CR1_TE | CR1_RE);

	/*Enable uart module*/
	USART2->CR1 |= CR1_UE;
}

void uart2_rx_interrupt_init(void){
	/*Enable RXNE interrupt*/
	USART2->CR1 |= CR1_RXNEIE;
	/*Enable UART2 interrupt in NVIC*/
	NVIC_EnableIRQ(USART2_IRQn);
}

void uart3_rxtx_init(void){
	/****Configure uart gpio pin****/
	/*Enable clock access to gpioc*/
	RCC->AHB1ENR |= GPIOCEN;

	/*Set PC10 mode to alternate function mode*/
	GPIOC->MODER &=~(1U<<20);
	GPIOC->MODER |= (1U<<21);

	/*Set PC10 alterntive function type to UART_TX (AF07)*/
	GPIOC ->AFR[1] |= (1U<<8);
	GPIOC ->AFR[1] |= (1U<<9);
	GPIOC ->AFR[1] |= (1U<<10);
	GPIOC ->AFR[1] &=~ (1U<<11);

	/*Set PC11 mode to alternate function mode*/
	GPIOC->MODER &=~(1U<<22);
	GPIOC->MODER |= (1U<<23);

	/*Set PC11 alterntive function type to UART_RX (AF07)*/
	GPIOC ->AFR[1] |= (1U<<12);
	GPIOC ->AFR[1] |= (1U<<13);
	GPIOC ->AFR[1] |= (1U<<14);
	GPIOC ->AFR[1] &=~ (1U<<15);

	/****Configure uart module****/
	/*Enable clock access to uart2*/
	RCC->APB1ENR |= UART3EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART3, APB1_CLK, UART_BAUDRATE);

	/*Configure the transfer direction*/
	USART3->CR1 = (CR1_TE | CR1_RE);

	/*Enable uart module*/
	USART3->CR1 |= CR1_UE;
}

void uart3_rx_interrupt_init(void){
	/*Enable RXNE interrupt*/
	USART3->CR1 |= CR1_RXNEIE;
	/*Enable UART2 interrupt in NVIC*/
	NVIC_EnableIRQ(USART3_IRQn);
}

void uart2_tx_init(void){
	/****Configure uart gpio pin****/
	/*Enable clock access to gpioa*/
	RCC->AHB1ENR |= GPIOAEN;

	/*Set PA2 mode to alternate function mode*/
	GPIOA->MODER &=~(1U<<4);
	GPIOA->MODER |= (1U<<5);

	/*Set PA2 alterntive function type to UART_TX (AF07)*/
	GPIOA ->AFR[0] |= (1U<<8);
	GPIOA ->AFR[0] |= (1U<<9);
	GPIOA ->AFR[0] |= (1U<<10);
	GPIOA ->AFR[0] &=~ (1U<<11);

	/****Configure uart module****/
	/*Enable clock access to uart2*/
	RCC->APB1ENR |= UART2EN;

	/*Configure baudrate*/
	uart_set_baudrate(USART2, APB1_CLK, UART_BAUDRATE);

	/*Configure the transfer directio*/
	USART2->CR1 = CR1_TE;

	/*Enable uart module*/
	USART2->CR1 |= CR1_UE;
}

char uart2_read(void){
	/*Make sure the receive data register is empty*/
	while (!(USART2->SR & SR_RNXE)){}
	/*Read data*/
	return USART2->DR;
}

char uart3_read(void){
	/*Make sure the receive data register is empty*/
	while (!(USART3->SR & SR_RNXE)){}
	/*Read data*/
	return USART3->DR;
}

void uart2_write(int ch){
	/*Make sure the transmit data register is empty*/
	while (!(USART2->SR & SR_TXE)){}
	/*Write to transmit data register*/
	USART2->DR = (ch & 0xFF);
}

void uart3_write(int ch){
	/*Make sure the transmit data register is empty*/
	while (!(USART3->SR & SR_TXE)){}
	/*Write to transmit data register*/
	USART3->DR = (ch & 0xFF);
}

void uart_printf(int UARTx, const char *mensaje){
	UART_selec = UARTx;
	printf("%s", mensaje);
}

/***TELEMETRIA***/
void dma1_stream6_init(void){
	/*Enable clock accesto DMA*/
	RCC->AHB1ENR |= DMA1EN;

	/*Disable DMA stream6*/
	DMA1_Stream6->CR &= ~DMA_CR_EN;
	while(DMA1_Stream6->CR & DMA_CR_EN){}

	/*Clear all interrupt flags of stream6*/
	DMA1->HIFCR |= (1U<<16);
	DMA1->HIFCR |= (1U<<18);
	DMA1->HIFCR |= (1U<<19);
	DMA1->HIFCR |= (1U<<20);
	DMA1->HIFCR |= (1U<<21);

	/*Set the destination buffer*/
	DMA1_Stream6->PAR = (uint32_t)&USART2->DR;

	/*Select Stream6 CH4*/
	DMA1_Stream6->CR = CHSEL4;

	/*Enable memory increment*/
	DMA1_Stream6->CR |= DMA_MEM_INC;

	/*Configure transfer direction*/
	DMA1_Stream6->CR |= DMA_DIR_MEM_TO_PERIPH;

	/*Enable direct mode and disable FIFO*/
	DMA1_Stream6->FCR = 0;

	/*Enable UART2 transmitter DMA*/
	USART2->CR3 |= UART_CR3_DMA;

}

//Función para comprobar si se esta enviando el mensaje
bool dma1_stream6_ocupado(void){
	return (DMA1_Stream6->CR & DMA_CR_EN) != 0;
}

//Función enviar mensaje,  false si se esta enviando ya un mensaje
bool dma1_stream6_enviar(const char *buf, uint16_t len){
	if(dma1_stream6_ocupado() || len == 0){
		return false;
	}

	/*Clear all interrupt flags of stream6*/
	DMA1->HIFCR |= (1U<<16);
	DMA1->HIFCR |= (1U<<18);
	DMA1->HIFCR |= (1U<<19);
	DMA1->HIFCR |= (1U<<20);
	DMA1->HIFCR |= (1U<<21);

	/*Set the source buffer*/
	DMA1_Stream6->M0AR = (uint32_t)buf;
	/*Set length*/
	DMA1_Stream6->NDTR = len;
	DMA1_Stream6->CR |= DMA_CR_EN;
	return true;
}

void dma1_stream5_rx_init(void){
	/*Enable clock accesto DMA*/
	RCC->AHB1ENR |= DMA1EN;

	/*Disable DMA stream5*/
	DMA1_Stream5->CR &= ~DMA_CR_EN;
	while(DMA1_Stream5->CR & DMA_CR_EN){}

	/*Clear all interrupt flags of stream5*/
	DMA1->HIFCR |= (1U<<6);
	DMA1->HIFCR |= (1U<<8);
	DMA1->HIFCR |= (1U<<9);
	DMA1->HIFCR |= (1U<<10);
	DMA1->HIFCR |= (1U<<11);

	/*Source UART2 data register*/
	DMA1_Stream5->PAR = (uint32_t)&USART2->DR;
	/*Destination circular buffer*/
	DMA1_Stream5->M0AR = (uint32_t)rx_dma_buf;
	DMA1_Stream5->NDTR = RX_DMA_TAM;

	/*Select Stream5 CH4*/
	DMA1_Stream5->CR = CHSEL4;

	/*Enable memory increment*/
	DMA1_Stream5->CR |= DMA_MEM_INC;

	/*Circular*/
	DMA1_Stream5->CR |= DMA_CIRC;

	/*Enable direct mode and disable FIFO*/
	DMA1_Stream5->FCR = 0;

	/*Enable UART2 receive DMA*/
	USART2->CR3 |= UART_CR3_DMAR;

	/*Start: from here the DMA receives*/
	DMA1_Stream5->CR |= DMA_CR_EN;
}

int uart2_rx_leer(void){
	/*Posicion donde escribe el DMA*/
	uint16_t escrito = RX_DMA_TAM - DMA1_Stream5->NDTR;

	if(escrito >= RX_DMA_TAM) escrito = 0;


	if(rx_leido == escrito){
		return -1;
	}

	char c = rx_dma_buf[rx_leido];
	rx_leido++;
	if(rx_leido >= RX_DMA_TAM) rx_leido = 0;

	return (unsigned char)c;
}

/***Configuracion***/
static void uart_set_baudrate(USART_TypeDef *USARTx,uint32_t PeriphClk, uint32_t BaudRate){
	USARTx->BRR = compute_uart_bd(PeriphClk,BaudRate);
}

static uint16_t compute_uart_bd(uint32_t PeriphClk, uint32_t BaudRate){
	return (PeriphClk + (BaudRate/2U)) / BaudRate;
}
