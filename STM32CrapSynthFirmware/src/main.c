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
#include "rtc.h"
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

__attribute__((section (".ccmram")))
Program_state_ccm state_ccm;
Program_state_ram state_ram;

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
	rtc_init();
	timers_all_init();
	dac_init();
	dma_init();
	gpio_init();
	state_init();
	spi_init();
	ad9833_init_all();
	att_init_all();
	external_flash_init_and_request_info();
	uart_init();
	uart_send_comms_establish_packet();

	ad9833_write_freq(2, 5000);

	//TIM20->ARR = 7200;
	//TIM20->PSC = 10000;

	//TIM20->CR1 = TIM_CR1_CEN;

	while(1)
	{
		decode_command();
		write_packet_to_flash();
		external_flash_write_page_task();

		for(int i = 0; i < 4; i++)
		{
			for(int j = 0; j < 7200000; j++) { asm("nop"); }

			ad9833_change_wave(2, i);
		}
	}

	return 0;
}
