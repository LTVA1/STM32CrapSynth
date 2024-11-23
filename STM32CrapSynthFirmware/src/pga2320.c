/*
 * pga2320.c
 *
 *  Created on: 16 окт. 2024 г.
 *      Author: georg
 */

#include "pga2320.h"
#include "stm32f3xx.h"
#include "spi.h"
#include "gpio.h"

uint8_t spi_tx_buf[ATT_TX_BUF_SIZE]; //SPI will send 16 bits at a time but each 16-bit section holds volumes for 2 channels
uint8_t att_need_write;

extern uint8_t spi2_ready;

void att_write_filter_resonance(uint8_t resonance)
{
	spi_tx_buf[3] = resonance;
	att_need_write = 1;
}

void att_write_vol(uint8_t ch, uint8_t vol)
{
	switch(ch)
	{
		case 0:
		{
			spi_tx_buf[6] = vol;
			break;
		}
		case 1:
		{
			spi_tx_buf[7] = vol;
			break;
		}
		case 2:
		{
			spi_tx_buf[4] = vol;
			break;
		}
		case 3:
		{
			spi_tx_buf[5] = vol;
			break;
		}
		case 4:
		{
			spi_tx_buf[3] = vol;
			break;
		}
		case 5:
		{
			spi_tx_buf[0] = vol;
			break;
		}
		case 6:
		{
			spi_tx_buf[1] = vol;
			break;
		}
		default: break;
	}
	att_need_write = 1;
}

void att_flush()
{
	while(!spi2_ready) { asm("nop"); }
	while(!(SPI2->SR & SPI_SR_TXE) || (SPI2->SR & SPI_SR_BSY)) { asm("nop"); }

	SPI2->CR1 &= ~SPI_CR1_SPE;
	SPI2->CR1 |= SPI_CR1_CPHA; //for PGA2320
	SPI2->CR1 |= SPI_CR1_SPE;

	CS_ATTEN_LOW
	spi2_send_via_dma((uint16_t*)&spi_tx_buf[0], ATT_TX_BUF_SIZE / 2);

	while(!spi2_ready) { asm("nop"); }
	while(!(SPI2->SR & SPI_SR_TXE) || (SPI2->SR & SPI_SR_BSY)) { asm("nop"); }

	CS_ATTEN_HIGH
}

void att_init_all()
{
	att_need_write = 0;

	for(int i = 0; i < ATT_TX_BUF_SIZE; i++)
	{
		spi_tx_buf[i] = 0;
	}

	att_flush();
}
