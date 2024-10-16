/*
 * spi.c
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include "spi.h"
#include "stm32f3xx.h"

uint8_t spi1_ready_tx;
uint8_t spi1_ready_rx;
uint8_t spi1_rxtx;
uint8_t spi2_ready;

void DMA1_Channel2_IRQHandler() //receive
{
	DMA1->IFCR |= DMA_IFCR_CTCIF2;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN; //stop DMA

	spi1_ready_rx = 1;
}

void DMA1_Channel3_IRQHandler() //send
{
	DMA1->IFCR |= DMA_IFCR_CTCIF3;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN; //stop DMA

	spi1_ready_tx = 1;

	if(!spi1_rxtx)
	{
		spi1_ready_rx = 1; //if we don't receive something, hopefully prevents new transaction when one of DMA channels is still working on old one
	}
}

void DMA1_Channel5_IRQHandler() //receive
{
	DMA1->IFCR |= DMA_IFCR_CTCIF5;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN; //stop DMA

	spi2_ready = 1;
}

void spi1_send_via_dma(uint8_t* sendbuf, uint16_t size)
{
	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CMAR = (uint32_t)sendbuf;
	DMA1_Channel3->CNDTR = size;
	spi1_ready_tx = 0;
	spi1_ready_rx = 0;
	spi1_rxtx = 0;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void spi1_receive_via_dma(uint8_t* sendbuf, uint8_t* receivebuf, uint16_t size)
{
	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	//we need to specify dummy send buffer for DMA "TX" channel to force SPI clock

	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;

	DMA1_Channel3->CMAR = (uint32_t)sendbuf;
	DMA1_Channel3->CNDTR = size;

	DMA1_Channel2->CMAR = (uint32_t)receivebuf;
	DMA1_Channel2->CNDTR = size;

	spi1_ready_tx = 0;
	spi1_ready_rx = 0;
	spi1_rxtx = 1;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}

void spi2_send_via_dma(uint16_t* sendbuf, uint16_t size)
{
	while(!spi2_ready) { asm("nop"); }

	DMA1_Channel5->CCR &= ~DMA_CCR_EN;
	DMA1_Channel5->CMAR = (uint32_t)sendbuf;
	DMA1_Channel5->CNDTR = size;
	spi2_ready = 0;
	DMA1_Channel5->CCR |= DMA_CCR_EN;
}

void spi_init()
{
	SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; //fclk / 2, master mode, software slave management
	//without software slave management does not work!
	SPI1->CR2 = SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN;   //  8 bit, enable DMA TX & RX
	SPI1->CR1 |= SPI_CR1_SPE; // enable

	SPI2->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; //fclk / 2, master mode, software slave management
	//without software slave management does not work!
	SPI2->CR2 = SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0 | SPI_CR2_TXDMAEN;   // 16 bit, DMA TX
	SPI2->CR1 |= SPI_CR1_SPE; // enable

	spi1_ready_tx = 1;
	spi1_ready_rx = 1;
	spi1_rxtx = 0;
}
