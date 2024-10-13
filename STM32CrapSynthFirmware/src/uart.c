/*
 * uart.c
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include "uart.h"
#include "stm32f3xx.h"
#include "ring_buf.h"
#include "commands.h"

extern RingBuffer rx_ring_buf;

extern uint16_t manufacturer_id;
extern uint64_t device_uid;
extern uint32_t memory_size;

uint8_t tx_buf[TX_BUF_SIZE];
uint8_t rx_buf[RX_BUF_SIZE];

uint8_t usart_ready;

uint8_t our_xor;

void USART1_IRQHandler()
{
	if (USART1->ISR & USART_ISR_RXNE)
	{
		buffer_put_to_end(&rx_ring_buf, USART1->RDR);
	}
}

void DMA1_Channel4_IRQHandler()
{
	DMA1->IFCR |= DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN; //stop DMA

	usart_ready = 1;
}

void uart_send_via_dma(uint16_t size)
{
	while(!usart_ready) { asm("nop"); }
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CMAR = (uint32_t)(&tx_buf[0]);
	usart_ready = 0;
	DMA1_Channel4->CCR |= DMA_CCR_EN;
}

void uart_init()
{
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;	//enable transmit and receive and interruption

	USART1->BRR = SystemCoreClock/256000; //256000 baud rate
	NVIC_SetPriority(USART1_IRQn, 3);

	USART1->CR3 |= USART_CR3_DMAT; //enable DMA tx mode

	USART1->CR1 |= USART_CR1_UE;
	NVIC_EnableIRQ(USART1_IRQn);

	buffer_init(&rx_ring_buf, rx_buf, RX_BUF_SIZE);
	usart_ready = 1;
}

void uart_send_comms_establish_packet()
{
	our_xor = 0;

	tx_buf[0] = SYNTH_SYNC_BYTE;
	tx_buf[1] = SYNTH_RESPONSE_READY;

	tx_buf[2] = MCU_FIRMWARE_SIZE >> 24;
	tx_buf[3] = (MCU_FIRMWARE_SIZE >> 16) & 0xff;
	tx_buf[4] = (MCU_FIRMWARE_SIZE >> 8) & 0xff;
	tx_buf[5] = MCU_FIRMWARE_SIZE & 0xff;

	tx_buf[6] = BASE_ADDR_FLASH >> 24;
	tx_buf[7] = (BASE_ADDR_FLASH >> 16) & 0xff;
	tx_buf[8] = (BASE_ADDR_FLASH >> 8) & 0xff;
	tx_buf[9] = BASE_ADDR_FLASH & 0xff;

	tx_buf[10] = BASE_ADDR_RAM >> 24;
	tx_buf[11] = (BASE_ADDR_RAM >> 16) & 0xff;
	tx_buf[12] = (BASE_ADDR_RAM >> 8) & 0xff;
	tx_buf[13] = BASE_ADDR_RAM & 0xff;

	tx_buf[14] = SUPPORTED_FILE_VERSION >> 24;
	tx_buf[15] = (SUPPORTED_FILE_VERSION >> 16) & 0xff;
	tx_buf[16] = (SUPPORTED_FILE_VERSION >> 8) & 0xff;
	tx_buf[17] = SUPPORTED_FILE_VERSION & 0xff;

	tx_buf[18] = FIRMWARE_VERSION >> 24;
	tx_buf[19] = (FIRMWARE_VERSION >> 16) & 0xff;
	tx_buf[20] = (FIRMWARE_VERSION >> 8) & 0xff;
	tx_buf[21] = FIRMWARE_VERSION & 0xff;

	tx_buf[22] = memory_size >> 24;
	tx_buf[23] = (memory_size >> 16) & 0xff;
	tx_buf[24] = (memory_size >> 8) & 0xff;
	tx_buf[25] = memory_size & 0xff;

	tx_buf[26] = device_uid >> 50;
	tx_buf[27] = (device_uid >> 48) & 0xff;
	tx_buf[28] = (device_uid >> 40) & 0xff;
	tx_buf[29] = (device_uid >> 32) & 0xff;
	tx_buf[30] = (device_uid >> 24) & 0xff;
	tx_buf[31] = (device_uid >> 16) & 0xff;
	tx_buf[32] = (device_uid >> 8) & 0xff;
	tx_buf[33] = device_uid & 0xff;

	tx_buf[34] = manufacturer_id;

	for(int i = 35; i < SYNTH_RESPONSE_SIZE; i++)
	{
		tx_buf[i] = 0;
	}
	for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
	{
		our_xor ^= tx_buf[i];
	}

	tx_buf[SYNTH_RESPONSE_SIZE - 1] = our_xor;

	uart_send_via_dma(SYNTH_RESPONSE_SIZE);
}
