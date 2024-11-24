/*
 * uart.c
 *
 *  Created on: 5 ���. 2024 �.
 *      Author: Georg
 */

#include "uart.h"
#include "stm32f3xx.h"
#include "ring_buf.h"
#include "commands.h"
#include "main.h"
#include "gpio.h"

extern RingBuffer rx_ring_buf;

extern uint16_t manufacturer_id;
extern uint64_t device_uid;
extern uint32_t memory_size;
extern Program_state_ccm state_ccm;

uint8_t tx_buf[TX_BUF_SIZE];
uint8_t rx_buf[RX_BUF_SIZE];

uint8_t usart_ready;

uint8_t our_xor;

__attribute__((section (".ccmram")))
void UART4_IRQHandler()
{
	if (UART4->ISR & USART_ISR_RXNE)
	{
		//CS_NOISE_HIGH
		buffer_put_to_end(&rx_ring_buf, UART4->RDR);
		//CS_NOISE_LOW
	}
}

/*
__attribute__((section (".ccmram")))
void DMA1_Channel4_IRQHandler()
{
	DMA1->IFCR |= DMA_IFCR_CTCIF4;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN; //stop DMA

	usart_ready = 1;
}
*/

__attribute__((section (".ccmram")))
void DMA2_Channel5_IRQHandler()
{
	DMA2->IFCR |= DMA_IFCR_CTCIF5;
	DMA2_Channel5->CCR &= ~DMA_CCR_EN; //stop DMA

	usart_ready = 1;
}

void uart_send_via_dma(uint16_t size)
{
	/*while(!usart_ready) { asm("nop"); }
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel4->CNDTR = size;
	DMA1_Channel4->CMAR = (uint32_t)(&tx_buf[0]);
	usart_ready = 0;
	DMA1_Channel4->CCR |= DMA_CCR_EN;*/

	while(!usart_ready) { asm("nop"); }
	DMA2_Channel5->CCR &= ~DMA_CCR_EN;
	DMA2_Channel5->CNDTR = size;
	DMA2_Channel5->CMAR = (uint32_t)(&tx_buf[0]);
	usart_ready = 0;
	DMA2_Channel5->CCR |= DMA_CCR_EN;
}

void uart_init()
{
	//UARTs used as phase reset timers

	USART1->CR1 |= USART_CR1_TE | USART_CR1_TXEIE | USART_CR1_M1; //9 data bits
	USART1->CR2 |= USART_CR2_STOP_1; //2 stop bits

	USART1->BRR = 1000;

	USART1->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(USART1_IRQn, 8);
	//NVIC_EnableIRQ(USART1_IRQn);

	USART2->CR1 |= USART_CR1_TE | USART_CR1_TCIE | USART_CR1_M1; //9 data bits
	USART2->CR2 |= USART_CR2_STOP_1; //2 stop bits

	USART2->BRR = 1000;

	USART2->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(USART2_IRQn, 8);
	//NVIC_EnableIRQ(USART2_IRQn);

	USART3->CR1 |= USART_CR1_TE | USART_CR1_TCIE | USART_CR1_M1; //9 data bits
	USART3->CR2 |= USART_CR2_STOP_1; //2 stop bits

	USART3->BRR = 1000;

	USART3->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(USART3_IRQn, 8);
	//NVIC_EnableIRQ(USART3_IRQn);

	UART5->CR1 |= USART_CR1_TE | USART_CR1_TCIE | USART_CR1_M1; //9 data bits
	UART5->CR2 |= USART_CR2_STOP_1; //2 stop bits

	UART5->BRR = 1000;

	UART5->CR1 |= USART_CR1_UE;

	NVIC_SetPriority(UART5_IRQn, 8);
	//NVIC_EnableIRQ(UART5_IRQn);


	//================================================
	//UART

	UART4->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;	//enable transmit and receive and interruption

	UART4->BRR = SystemCoreClock/2/(2000000); //2M baud rate

	UART4->CR3 |= USART_CR3_DMAT; //enable DMA tx mode

	UART4->CR1 |= USART_CR1_UE;
	NVIC_SetPriority(UART4_IRQn, 3);
	NVIC_EnableIRQ(UART4_IRQn);

	//UART4->TDR = 0xdd;

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

	tx_buf[26] = device_uid >> 56;
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

void uart_send_response()
{
	our_xor = 0;

	tx_buf[0] = SYNTH_SYNC_BYTE;

	switch(state_ccm.state)
	{
		case STATE_PROG_INTERNAL_FLASH:
		{
			tx_buf[1] = SYNTH_RESPONSE_FLASH_BLOCK;
			break;
		}
		case STATE_PROG_EXTERNAL_FLASH:
		{
			tx_buf[1] = SYNTH_RESPONSE_EXTERNAL_FLASH_BLOCK;
			break;
		}
		case STATE_PROG_RAM:
		{
			tx_buf[1] = SYNTH_RESPONSE_RAM_BLOCK;
			break;
		}
		default: break;
	}

	tx_buf[2] = state_ccm.last_packet_xor;

	tx_buf[3] = state_ccm.block_start_offset & 0xff;
	tx_buf[4] = (state_ccm.block_start_offset >> 8) & 0xff;
	tx_buf[5] = (state_ccm.block_start_offset >> 16) & 0xff;
	tx_buf[6] = (state_ccm.block_start_offset >> 24) & 0xff;

	tx_buf[7] = state_ccm.block_length & 0xff;
	tx_buf[8] = (state_ccm.block_length >> 8) & 0xff;

	for(int i = 9; i < SYNTH_RESPONSE_SIZE; i++)
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
