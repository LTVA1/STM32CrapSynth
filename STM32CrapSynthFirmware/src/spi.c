/*
 * spi.c
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include "spi.h"
#include "stm32f3xx.h"

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
}
