/*
******************************************************************************
File:     main.c
Info:     Generated by Atollic TrueSTUDIO(R) 9.2.0   2024-10-02

The MIT License (MIT)
Copyright (c) 2018 STMicroelectronics

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

******************************************************************************
*/

/* Includes */
#include "stm32f3xx.h"

#include "systick.h"
#include "internal_flash.h"
#include "tim.h"
#include "spi.h"
#include "uart.h"
#include "dac.h"
#include "dma.h"
#include "gpio.h"
#include "external_flash.h"
#include "main.h"
#include "commands.h"
#include "ad9833.h"
#include "pga2320.h"
#include "clocks_interrupts.h"

__attribute__((section (".ccmram_variables")))
Program_state_ccm state_ccm;
__attribute__((section (".ccmram_variables")))
Program_state_ram state_ram;

extern uint8_t spi_tx_buf[];

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/

void state_init()
{
	state_ccm.state = STATE_IDLE;
}

int main(void)
{
	remap_and_place_interrupt_vectors_table_to_ccmram();
	enable_all_clocks(); // in that order because http://efton.sk/STM32/gotcha/g183.html
	set_72MHz();
	unlock_flash();
	systick_init();
	timers_all_init();
	dac_init();
	dma_init();
	gpio_init();
	state_init();
	spi_init();
	ad9833_init_all();
	att_init_all();
	external_flash_init_and_request_info();
	playback_init();
	uart_init();
	uart_send_comms_establish_packet();

	/*for(int i = 0; i < 100000; i++) { asm("nop"); }

	NVIC_DisableIRQ(UART5_IRQn);
	UART5->BRR = 0xfff7;
	NVIC_EnableIRQ(UART5_IRQn);

	for(int i = 0; i < 400000; i++) { asm("nop"); }

	NVIC_DisableIRQ(UART5_IRQn);*/

	//ad9833_write_freq(2, 5000);

	//TIM20->ARR = 7200;
	//TIM20->PSC = 10000;

	//TIM20->CR1 = TIM_CR1_CEN;

	//DAC1->CR |= DAC_CR_WAVE1_0 | DAC_CR_MAMP1_3 | DAC_CR_MAMP1_0;
	//DAC1->CR |= DAC_CR_WAVE2_1 | DAC_CR_MAMP2_3;

	//test_play_wavetable();
	//test_play_sample();

	/*ZCEN_DAC_ENABLE

	for(int j = 0; j < 80; j++)
	{
		spi_tx_buf[0] = (uint8_t)(120 + j);

		att_flush();

		for(int i = 0; i < 100000; i++) { asm("nop"); }
	}

	for(int j = 0; j < 80; j++)
	{
		spi_tx_buf[0] = (uint8_t)(120 + 80 - j);

		att_flush();

		for(int i = 0; i < 100000; i++) { asm("nop"); }
	}

	spi_tx_buf[0] = 0;

	for(int j = 0; j < 80; j++)
	{
		spi_tx_buf[1] = (uint8_t)(120 + j);

		att_flush();

		for(int i = 0; i < 100000; i++) { asm("nop"); }
	}

	for(int j = 0; j < 80; j++)
	{
		spi_tx_buf[1] = (uint8_t)(120 + 80 - j);

		att_flush();

		for(int i = 0; i < 100000; i++) { asm("nop"); }
	}*/

	//spi_tx_buf[6] = 220;
	//spi_tx_buf[7] = 220;

	//att_flush();

	//ad9833_write_freq(0, 5000);
	//ad9833_change_wave(0, 2);

	/*ad9833_cs_low(0);
	spi2_send_16bits(0x2100);
	ad9833_cs_high(0);
	ad9833_cs_low(0);
	spi2_send_16bits(0x50C7);
	ad9833_cs_high(0);
	ad9833_cs_low(0);
	spi2_send_16bits(0x4000);
	ad9833_cs_high(0);
	ad9833_cs_low(0);
	spi2_send_16bits(0x2000);
	ad9833_cs_high(0);

	ad9833_change_wave(0, 1);*/

	//CH1_CONNECT_DDS

	//TIM3->ARR = 65000;
	//TIM3->PSC = 10;
	//TIM3->CCR3 = 32000;
	//TIM3->CR1 |= TIM_CR1_CEN;



	/*uint8_t temp[3];

	uint32_t lfsr_state = 0;
	uint32_t divider = 6550;

	//CS_EXT_FLASH_LOW

	extern uint8_t spi1_ready_tx;
	extern uint8_t spi1_ready_rx;

	for(int i = 0; i < 200; i++)
	{
		temp[0] = ((lfsr_state >> 16) & 0x7f) | 0x80;
		temp[1] = (lfsr_state >> 8) & 0xff;
		temp[2] = lfsr_state;

		CS_EXT_FLASH_LOW

		extern uint8_t spi1_ready_tx;
		extern uint8_t spi1_ready_rx;

		spi1_send_via_dma(&temp[0], 3);
		while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }
		while(!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY)) { asm("nop"); }

		CS_EXT_FLASH_HIGH

		//divider--;

		for(int i = 0; i < 9800; i++) { asm("nop"); }
	}

	for(int i = 0; i < 10000 && divider > 100; i++)
	{
		temp[0] = ((divider >> 16) & 1);
		temp[1] = (divider >> 8) & 0xff;
		temp[2] = divider;

		CS_EXT_FLASH_LOW

		extern uint8_t spi1_ready_tx;
		extern uint8_t spi1_ready_rx;

		spi1_send_via_dma(&temp[0], 3);
		while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }
		while(!(SPI1->SR & SPI_SR_TXE) || (SPI1->SR & SPI_SR_BSY)) { asm("nop"); }

		CS_EXT_FLASH_HIGH

		divider--;

		for(int i = 0; i < 9800; i++) { asm("nop"); }
	}*/

	while(1)
	{
		decode_command();
		write_packet_to_flash();
		external_flash_write_page_task();
		execute_commands();

		/*for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 2200000; j++) { asm("nop"); }

			ad9833_change_wave(0, i);
			ad9833_write_freq(0, 600 + 300 * i);
		}*/
	}

	return 0;
}
