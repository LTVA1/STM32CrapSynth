/*
 * playback.c
 *
 *  Created on: 9 окт. 2024 г.
 *      Author: georg
 */

#include "playback.h"
#include "stm32f3xx.h"
#include "main.h"
#include "commands.h"
#include "external_flash.h"
#include "gpio.h"

#include <string.h>

uint8_t wavetable_array[2][WAVETABLE_SIZE];
uint32_t noise_lfsr_load[NOISE_LFSR_LENGTH * 2];
uint8_t sample_mem_ram[SAMPLE_MEM_RAM_SIZE];

uint16_t curr_buf_pos;
uint16_t curr_read_buf_pos;
uint32_t curr_dump_pos;
uint8_t new_tick;

uint8_t chan_base_addr[13];

uint8_t curr_command;
uint8_t curr_chan;

uint32_t dummy;

extern Program_state_ccm state_ccm;
extern Program_state_ram state_ram;
extern uint8_t spi_rx_double_buf[];

__attribute__((section (".ccmram")))
void DMA2_Channel3_IRQHandler()
{
	DMA2->IFCR |= DMA_IFCR_CTCIF3;
	DMA2_Channel3->CCR &= ~DMA_CCR_EN; //stop DMA
	TIM6->CR1 &= ~TIM_CR1_CEN;

	Sample_state* ss = &state_ram.dac[0];

	ss->curr_pos += ss->curr_portion_size;

	if(ss->curr_pos < ss->length)
	{
		DMA2_Channel3->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->curr_pos;
		DMA2_Channel3->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel3->CCR |= DMA_CCR_EN;
		TIM6->CR1 = TIM_CR1_CEN;

		return;
	}

	if(ss->curr_pos == ss->length && ss->loop)
	{
		ss->curr_pos = ss->loop_point;
		DMA2_Channel3->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->loop_point;
		DMA2_Channel3->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel3->CCR |= DMA_CCR_EN;
		TIM6->CR1 = TIM_CR1_CEN;

		return;
	}
}

__attribute__((section (".ccmram")))
void DMA2_Channel4_IRQHandler()
{
	DMA2->IFCR |= DMA_IFCR_CTCIF4;
	DMA2_Channel4->CCR &= ~DMA_CCR_EN; //stop DMA
	TIM7->CR1 &= ~TIM_CR1_CEN;

	Sample_state* ss = &state_ram.dac[1];

	ss->curr_pos += ss->curr_portion_size;

	if(ss->curr_pos < ss->length)
	{
		DMA2_Channel4->CMAR = BASE_ADDR_FLASH + ss->start_offset + ss->curr_pos;
		DMA2_Channel4->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel3->CCR |= DMA_CCR_EN;
		TIM7->CR1 = TIM_CR1_CEN;

		return;
	}

	if(ss->curr_pos == ss->length && ss->loop)
	{
		ss->curr_pos = ss->loop_point;
		DMA2_Channel4->CMAR = BASE_ADDR_FLASH + ss->start_offset + ss->loop_point;
		DMA2_Channel4->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel4->CCR |= DMA_CCR_EN;
		TIM7->CR1 = TIM_CR1_CEN;

		return;
	}
}

__attribute__((section (".ccmram")))
void TIM2_IRQHandler()
{
	if(TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		new_tick = 1;
	}
}

__attribute__((section (".ccmram")))
void read_reg_dump(uint8_t half, uint8_t start)
{
	external_flash_read_data(curr_dump_pos, &spi_rx_double_buf[half ? (EXT_FLASH_RX_BUF_SIZE / 2) : 0], (EXT_FLASH_RX_BUF_SIZE / 2), start);

	curr_dump_pos += EXT_FLASH_RX_BUF_SIZE / 2;
	curr_buf_pos += EXT_FLASH_RX_BUF_SIZE / 2;

	if(curr_buf_pos == EXT_FLASH_RX_BUF_SIZE)
	{
		curr_buf_pos = 0;
	}
}

__attribute__((section (".ccmram")))
uint8_t reg_dump_read_byte()
{
	uint8_t data = spi_rx_double_buf[curr_read_buf_pos];
	curr_read_buf_pos++;

	if(curr_read_buf_pos == EXT_FLASH_RX_BUF_SIZE / 2)
	{
		read_reg_dump(0, 0);
	}
	if(curr_read_buf_pos == EXT_FLASH_RX_BUF_SIZE)
	{
		read_reg_dump(1, 0);
		curr_read_buf_pos = 0;
	}

	return data;
}

__attribute__((section (".ccmram")))
uint16_t reg_dump_read_two_bytes()
{
	return ((uint16_t)reg_dump_read_byte() | ((uint16_t)reg_dump_read_byte() << 8));
}

__attribute__((section (".ccmram")))
uint32_t reg_dump_read_three_bytes()
{
	return ((uint32_t)reg_dump_read_byte() | ((uint32_t)reg_dump_read_byte() << 8) | ((uint32_t)reg_dump_read_byte() << 16));
}

__attribute__((section (".ccmram")))
uint32_t reg_dump_read_four_bytes()
{
	return ((uint32_t)reg_dump_read_byte() | ((uint32_t)reg_dump_read_byte() << 8) | ((uint32_t)reg_dump_read_byte() << 16) | ((uint32_t)reg_dump_read_byte() << 24));
}

void playback_init()
{
	curr_buf_pos = 0;
	curr_dump_pos = 0;
	new_tick = 0;
	curr_read_buf_pos = 0;

	memset(&state_ram, 0, sizeof(Program_state_ram));

	for(int i = 0; i < 5; i++) //AD9833 chans & noise chan
	{
		chan_base_addr[i] = 8 * i;
	}
	for(int i = 5; i < 7; i++) //DAC chans
	{
		chan_base_addr[i] = 8 * 5 + 32 * (i - 5);
	}
	for(int i = 7; i < 12; i++) //phase reset timer chans
	{
		chan_base_addr[i] = 8 * 5 + 32 * 2 + 8 * (i - 7);
	}

	chan_base_addr[12] = 0xff;
}

DMA_Channel_TypeDef* samp_chans_dma[2] = { DMA2_Channel3, DMA2_Channel4 };
DMA_Channel_TypeDef* wave_copy_chans_dma[2] = { DMA1_Channel6, DMA1_Channel7 };
TIM_TypeDef* samp_chans_timers[2] = { TIM6, TIM7 };
IRQn_Type samp_chans_IRQ[2] = { DMA2_Channel3_IRQn, DMA2_Channel4_IRQn };

void set_playback_rate()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;

	TIM2->ARR = reg_dump_read_four_bytes();
	TIM2->PSC = 0;

	TIM2->CR1 |= TIM_CR1_CEN;
}

void start_playback()
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
	TIM2->CNT = 0;

	playback_init();

	read_reg_dump(0, 1);
	read_reg_dump(1, 0);

	TIM2->ARR = reg_dump_read_four_bytes();
	TIM2->PSC = 0;

	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
}

void stop_playback()
{
	NVIC_DisableIRQ(TIM2_IRQn);

	TIM2->CR1 &= ~TIM_CR1_CEN;

	for(int i = 0; i < 2; i++)
	{
		samp_chans_dma[i]->CCR &= ~DMA_CCR_EN;
		samp_chans_timers[i]->CR1 &= ~TIM_CR1_CEN;
		NVIC_DisableIRQ(samp_chans_IRQ[i]);
	}

	//TODO: stop noise generator and AD9833's

	CS_EXT_FLASH_HIGH
}

__attribute__((section (".ccmram")))
void execute_dac_command(uint8_t chan, uint8_t command)
{
	Sample_state* ss = &state_ram.dac[chan];

	switch(command)
	{
		case CMD_DAC_VOLUME:
		{
			dummy = reg_dump_read_byte();
			break;
		}
		case CMD_DAC_PLAY_SAMPLE:
		case CMD_DAC_PLAY_SAMPLE_LOOPED:
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;
			samp_chans_dma[chan]->CCR &= ~DMA_CCR_EN;
			ss->curr_pos = 0;
			ss->wavetable = 0;
			samp_chans_dma[chan]->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset;
			samp_chans_dma[chan]->CNDTR = my_min(0xffff, ss->length);

			ss->curr_portion_size = my_min(0xffff, ss->length);
			ss->loop = (command == CMD_DAC_PLAY_SAMPLE_LOOPED ? 1 : 0);

			NVIC_EnableIRQ(samp_chans_IRQ[chan]);

			samp_chans_dma[chan]->CCR |= DMA_CCR_EN;
			samp_chans_timers[chan]->CR1 |= TIM_CR1_CEN;
			break;
		}
		case CMD_DAC_PLAY_WAVETABLE:
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;
			samp_chans_dma[chan]->CCR &= ~DMA_CCR_EN;
			ss->wavetable = 1;

			NVIC_DisableIRQ(samp_chans_IRQ[chan]);

			samp_chans_dma[chan]->CMAR = (uint32_t)&wavetable_array[chan][0];
			samp_chans_dma[chan]->CNDTR = WAVETABLE_SIZE;
			samp_chans_dma[chan]->CCR |= DMA_CCR_CIRC;

			samp_chans_dma[chan]->CCR |= DMA_CCR_EN;

			samp_chans_timers[chan]->CR1 |= TIM_CR1_CEN;
			break;
		}
		case CMD_DAC_STOP:
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;
			samp_chans_dma[chan]->CCR &= ~DMA_CCR_EN;
			break;
		}
		case CMD_DAC_START_ADDR_FLASH:
		{
			ss->in_ram = 0;
			ss->start_offset = reg_dump_read_three_bytes();
			break;
		}
		case CMD_DAC_START_ADDR_RAM:
		{
			ss->in_ram = 1;
			ss->start_offset = reg_dump_read_two_bytes();
			break;
		}
		case CMD_DAC_RESET: //TODO: separate into function for phase reset timer?
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;
			samp_chans_dma[chan]->CCR &= ~DMA_CCR_EN;

			ss->curr_pos = 0;

			samp_chans_dma[chan]->CNDTR = (ss->wavetable ? WAVETABLE_SIZE : my_min(0xffff, ss->length));
			ss->curr_portion_size = my_min(0xffff, ss->length);

			samp_chans_dma[chan]->CCR |= DMA_CCR_EN;
			samp_chans_timers[chan]->CR1 |= TIM_CR1_CEN;
			break;
		}
		case CMD_DAC_TIMER_FREQ:
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;

			samp_chans_timers[chan]->PSC = reg_dump_read_byte();
			samp_chans_timers[chan]->ARR = reg_dump_read_two_bytes();

			if(samp_chans_timers[chan]->CNT >= samp_chans_timers[chan]->ARR)
			{
				samp_chans_timers[chan]->CNT = samp_chans_timers[chan]->ARR - 3;
			}

			samp_chans_timers[chan]->CR1 |= TIM_CR1_CEN;
			break;
		}
		case CMD_DAC_ZERO_CROSS_ENABLE:
		{
			ZCEN_DAC_ENABLE
			break;
		}
		case CMD_DAC_ZERO_CROSS_DISABLE:
		{
			ZCEN_DAC_DISABLE
			break;
		}
		case CMD_DAC_LOOP_POINT_FLASH:
		{
			ss->in_ram = 0;
			ss->loop_point = reg_dump_read_three_bytes();
			break;
		}
		case CMD_DAC_LOOP_POINT_RAM:
		{
			ss->in_ram = 1;
			ss->loop_point = reg_dump_read_two_bytes();
			break;
		}
		case CMD_DAC_LENGTH_FLASH:
		{
			ss->in_ram = 0;
			ss->length = reg_dump_read_three_bytes();
			break;
		}
		case CMD_DAC_LENGTH_RAM:
		{
			ss->in_ram = 1;
			ss->length = reg_dump_read_two_bytes();
			break;
		}
		case CMD_DAC_WAVE_TYPE:
		{
			ss->wave_type = reg_dump_read_byte();

			if(ss->wave_type > 5)
			{
				ss->wavetable = 1;
			}
			else
			{
				ss->wavetable = 0;
			}

			//TODO: support hardware triangle & noise

			if(ss->wave_type == 6 && ss->wavetable)
			{
				for(int i = 0; i < WAVETABLE_SIZE; i++)
				{
					wavetable_array[chan][i] = i < ss->duty ? 255 : 0;
				}
			}
			if(ss->wave_type == 7 && ss->wavetable)
			{
				for(int i = 0; i < WAVETABLE_SIZE; i++)
				{
					wavetable_array[chan][i] = i;
				}
			}
			break;
		}
		case CMD_DAC_DUTY:
		{
			//ZCEN_2_ENABLE
			ss->duty = reg_dump_read_byte();

			if(ss->wave_type == 6 && ss->wavetable)
			{
				for(int i = 0; i < WAVETABLE_SIZE; i++)
				{
					wavetable_array[chan][i] = ((i < ss->duty) ? 255 : 0);
				}
			}
			//ZCEN_2_DISABLE
			break;
		}
		case CMD_DAC_WAVETABLE_DATA:
		{
			wave_copy_chans_dma[chan]->CCR &= ~DMA_CCR_EN;
			wave_copy_chans_dma[chan]->CMAR = (uint32_t)&spi_rx_double_buf[curr_read_buf_pos];
			wave_copy_chans_dma[chan]->CNDTR = WAVETABLE_SIZE;
			wave_copy_chans_dma[chan]->CCR |= DMA_CCR_EN;
			curr_read_buf_pos += 256;
			/*for(int i = 0; i < WAVETABLE_SIZE; i++)
			{
				wavetable_array[chan][i] = reg_dump_read_byte();
			}*/
			break;
		}
		case CMD_DAC_NOISE_TRI_AMP:
		{
			DAC->CR &= (chan ? (~DAC_CR_MAMP2) : (~DAC_CR_MAMP1));
			DAC->CR |= (chan ? ((uint32_t)reg_dump_read_byte() << 8) : ((uint32_t)reg_dump_read_byte() << 24));
			break;
		}
		default: break;
	}
}

__attribute__((section (".ccmram")))
void execute_commands()
{
	if(!new_tick) return;

	//ZCEN_DAC_ENABLE

	while(1)
	{
		curr_command = reg_dump_read_byte();
		curr_chan = 0xff;

		if(curr_command == CMD_NOP)
		{
			while(curr_command == CMD_NOP)
			{
				curr_command = reg_dump_read_byte();
			}
		}

		if(curr_command != CMD_NEXT_FRAME && curr_command != CMD_NOP)
		{
			for(int i = 0; i < 12; i++)
			{
				if(curr_command >= chan_base_addr[i] && curr_command < chan_base_addr[i + 1])
				{
					curr_chan = i;
					break;
				}
			}

			switch(curr_chan)
			{
				case 5:
				case 6:
				{
					execute_dac_command(curr_chan - 5, curr_command - chan_base_addr[curr_chan]);
					break;
				}
				default: break;
			}
		}

		if(curr_command == CMD_NEXT_FRAME) break;

		if(curr_command == CMD_SET_RATE)
		{
			set_playback_rate();
		}

		if(curr_command == CMD_END)
		{
			stop_playback();
		}

		if(curr_command == CMD_LOOP_POINT)
		{
			//stop_playback();

			//TIM2->CR1 &= ~TIM_CR1_CEN;
			//TIM2->CNT = 0;
			curr_buf_pos = 0;
			curr_dump_pos = reg_dump_read_four_bytes();

			CS_EXT_FLASH_HIGH
			//__disable_irq();
			//TIM2->CR1 &= ~TIM_CR1_CEN;
			curr_read_buf_pos = 0;

			read_reg_dump(0, 1);
			//__enable_irq();
			read_reg_dump(1, 0);

			//TIM2->CR1 |= TIM_CR1_CEN;

			//NVIC_EnableIRQ(TIM2_IRQn);
			//TIM2->CR1 |= TIM_CR1_CEN;
		}
	}

	new_tick = 0;

	//ZCEN_DAC_DISABLE
}





































void test_play_wavetable()
{
	for(int i = 0; i < WAVETABLE_SIZE; i++)
	{
		wavetable_array[0][i] = i;
	}

	//wavetable_array[0][200] = 0xff;

	for(int i = 0; i < WAVETABLE_SIZE; i++)
	{
		wavetable_array[1][i] = (i > 127) ? ~(i * 2) : (i * 2);
	}

	//wavetable_array[1][210] = 0xff;

	//TIM6->ARR = 300;
	//TIM6->PSC = 0;

	TIM7->ARR = 300;
	TIM7->PSC = 0;

	//NVIC_DisableIRQ(DMA2_Channel3_IRQn);

	//DMA2_Channel3->CMAR = (uint32_t)&wavetable_array[0][0];
	//DMA2_Channel3->CNDTR = WAVETABLE_SIZE;
	//DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	//DMA2_Channel3->CCR |= DMA_CCR_EN;

	NVIC_DisableIRQ(DMA2_Channel4_IRQn);

	DMA2_Channel4->CMAR = (uint32_t)&wavetable_array[1][0];
	DMA2_Channel4->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel4->CCR |= DMA_CCR_CIRC;

	DMA2_Channel4->CCR |= DMA_CCR_EN;

	//TIM6->CR1 = TIM_CR1_CEN;
	TIM7->CR1 = TIM_CR1_CEN;
}

void test_play_sample()
{
	TIM6->ARR = 1580;
	TIM6->PSC = 0;

	NVIC_EnableIRQ(DMA2_Channel3_IRQn);

	DMA2_Channel3->CMAR = BASE_ADDR_FLASH;
	DMA2_Channel3->CNDTR = 0xffff;
	state_ram.dac[0].curr_portion_size = 0xffff;
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	state_ram.dac[0].start_offset = 0;
	state_ram.dac[0].length = 130897;
	state_ram.dac[0].loop = 1;
	state_ram.dac[0].loop_point = 0;
	state_ram.dac[0].prescaler = 0;
	state_ram.dac[0].volume = 0xff;
	state_ram.dac[0].wavetable = 0;
	state_ram.dac[0].curr_pos = 0;

	DMA2_Channel3->CCR |= DMA_CCR_EN;

	TIM6->CR1 = TIM_CR1_CEN;
}
