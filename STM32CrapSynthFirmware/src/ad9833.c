/*
 * ad9833.c
 *
 *  Created on: 16 окт. 2024 г.
 *      Author: Georg
 */

#include "ad9833.h"
#include "stm32f3xx.h"
#include "spi.h"
#include "gpio.h"

extern uint8_t spi2_ready;

uint16_t spi_tx_buf[AD9833_NUM][AD9833_TX_BUF_SIZE];

inline void ad9833_cs_low(uint8_t dds)
{
	switch(dds)
	{
		case 0:
		{
			CS_AD1_LOW
			break;
		}
		case 1:
		{
			CS_AD2_LOW
			break;
		}
		case 2:
		{
			CS_AD3_LOW
			break;
		}
		case 3:
		{
			CS_AD4_LOW
			break;
		}
		default: break;
	}
}

inline void ad9833_cs_high(uint8_t dds)
{
	switch(dds)
	{
		case 0:
		{
			CS_AD1_HIGH
			break;
		}
		case 1:
		{
			CS_AD2_HIGH
			break;
		}
		case 2:
		{
			CS_AD3_HIGH
			break;
		}
		case 3:
		{
			CS_AD4_HIGH
			break;
		}
		default: break;
	}
}

void ad9833_reset(uint8_t dds)
{
	ad9833_cs_low(dds);

	spi_tx_buf[dds][0] |= RESET_BIT; //reset bit on

	spi2_send_via_dma(&spi_tx_buf[dds][0], 1);
	while(!spi2_ready) { asm("nop"); }

	spi_tx_buf[dds][0] &= ~RESET_BIT; //reset bit off

	ad9833_cs_high(dds);
}

void ad9833_write_freq(uint8_t dds, uint32_t freq)
{
	ad9833_cs_low(dds);

	spi_tx_buf[dds][1] = FREQ0_REG_ADDRESS | (freq & 0x3fff);
	spi_tx_buf[dds][2] = FREQ0_REG_ADDRESS | ((freq >> 14) & 0x3fff);

	spi2_send_via_dma(&spi_tx_buf[dds][1], 2);
	while(!spi2_ready) { asm("nop"); }

	ad9833_cs_high(dds);
}

void ad9833_init(uint8_t dds)
{
	spi_tx_buf[dds][0] |= B28;
	ad9833_reset(dds);
	ad9833_write_freq(dds, 0);
}

void ad9833_init_all()
{
	for(int i = 0; i < AD9833_NUM; i++)
	{
		ad9833_init(i);
	}
}
