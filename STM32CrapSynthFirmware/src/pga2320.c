/*
 * pga2320.c
 *
 *  Created on: 16 ���. 2024 �.
 *      Author: georg
 */

#include "pga2320.h"
#include "stm32f3xx.h"
#include "spi.h"
#include "gpio.h"
#include "main.h"

#define my_min(a, b)  (((a)<(b))?(a):(b))

uint8_t spi_tx_buf_vol[ATT_TX_BUF_SIZE] __attribute__ ((aligned(2))); //SPI will send 16 bits at a time but each 16-bit section holds volumes for 2 channels
uint8_t att_need_write;

extern uint8_t spi2_ready;
extern Program_state_ram state_ram;

__attribute__((section (".ccmram")))
void att_write_vol(uint8_t ch, uint8_t vol)
{
	switch(ch)
	{
		case 0:
		{
			spi_tx_buf_vol[6] = vol;
			break;
		}
		case 1:
		{
			spi_tx_buf_vol[7] = vol;
			break;
		}
		case 2:
		{
			spi_tx_buf_vol[4] = vol;
			break;
		}
		case 3:
		{
			spi_tx_buf_vol[5] = vol;
			break;
		}
		case 4:
		{
			spi_tx_buf_vol[3] = vol;
			break;
		}
		case 5:
		{
			spi_tx_buf_vol[0] = vol;
			break;
		}
		case 6:
		{
			spi_tx_buf_vol[1] = vol;
			break;
		}
		default: break;
	}
	att_need_write = 1;
}

__attribute__((section (".ccmram")))
void att_flush()
{
	while(!spi2_ready) { asm("nop"); }
	while(!(SPI2->SR & SPI_SR_TXE) || (SPI2->SR & SPI_SR_BSY)) { asm("nop"); }

	SPI2->CR1 &= ~SPI_CR1_SPE;

	SPI2->CR1 |= (SPI_CR1_CPHA | SPI_CR1_BR_1); //for PGA2320 slower speed
	SPI2->CR1 |= SPI_CR1_SPE;

	CS_ATTEN_LOW
	spi2_send_via_dma((uint16_t*)&spi_tx_buf_vol[0], ATT_TX_BUF_SIZE / 2);

	while(!spi2_ready) { asm("nop"); }
	while(!(SPI2->SR & SPI_SR_TXE) || (SPI2->SR & SPI_SR_BSY)) { asm("nop"); }

	CS_ATTEN_HIGH
}

void att_init_all()
{
	att_need_write = 0;

	for(int i = 0; i < ATT_TX_BUF_SIZE; i++)
	{
		spi_tx_buf_vol[i] = 0;
	}

	att_flush();
}
