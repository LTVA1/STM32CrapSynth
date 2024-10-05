/*
 * uart.c
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include "uart.h"
#include "stm32f3xx.h"
#include "ring_buf.h"

extern RingBuffer rx_ring_buf;

uint8_t tx_buf[TX_BUF_SIZE];

uint8_t rx_buf[64];

void USART1_IRQHandler()
{
	if (USART1->ISR & USART_ISR_TC)
	{
		USART1->ICR |= USART_ICR_TCCF;
		//usart_ready = 1;
	}

	if (USART1->ISR & USART_ISR_RXNE)
	{
		buffer_put_to_end(&rx_ring_buf, USART1->RDR);
	}
}

void uart_init()
{
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;	//enable transmit and receive

	USART1->BRR = SystemCoreClock/256000; //256000 baud rate

	USART1->CR1 |= USART_CR1_TCIE | USART_CR1_RXNEIE; //interruption
	NVIC_SetPriority(USART1_IRQn, 3);

	USART1->CR3 |= USART_CR3_DMAT; //enable DMA tx mode

	USART1->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);
}
