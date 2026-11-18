/*
 * uart.h
 *
 *  Created on: 12 oct 2026
 *      Author: uliss
 */

#ifndef UART_H_
#define UART_H_
#include "stm32f4xx.h"
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

#define SR_RNXE (1U<<5)
#define HISR_TCIF6 (1U<<21)
#define HIFCR_CTCIF6 (1U<<21)

void uart2_tx_init(void);
void uart2_rxtx_init(void);
char uart2_read(void);
void uart2_write(int ch);
void uart2_rx_interrupt_init(void);

void uart3_rxtx_init(void);
void uart3_write(int ch);
char uart3_read(void);
void uart3_rx_interrupt_init(void);

void uart_printf(int UARTx, const char *mensaje);

void dma1_stream6_init(void);
bool dma1_stream6_ocupado(void);
bool dma1_stream6_enviar(const char *buf, uint16_t len);
void dma1_stream5_rx_init(void);
int uart2_rx_leer(void);

#endif /* UART_H_ */
