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
#include "pga2320.h"
#include "ad9833.h"

#include <string.h>

uint8_t wavetable_array[3][WAVETABLE_SIZE];
//uint32_t noise_lfsr_load[NOISE_LFSR_LENGTH * 2];
uint8_t sample_mem_ram[SAMPLE_MEM_RAM_SIZE];

uint16_t curr_buf_pos;
uint16_t curr_read_buf_pos;
uint32_t curr_dump_pos;
uint8_t new_tick;

uint8_t chan_base_addr[20];

uint8_t curr_command;
uint8_t curr_chan;

uint32_t dummy;

extern Program_state_ccm state_ccm;
extern Program_state_ram state_ram;
extern uint8_t spi_rx_double_buf[];

extern uint8_t att_need_write;

TIM_TypeDef* pwm_timers[4] = { TIM3, TIM4, TIM8, TIM15 };
TIM_TypeDef* noise_clock_timer = TIM17;

USART_TypeDef* phase_reset_uarts[4] = { USART1, USART2, USART3, UART5 };
IRQn_Type phase_reset_interrupts[6] = { TIM20_UP_IRQn, 0, /*SysTick*/ USART1_IRQn, USART2_IRQn, USART3_IRQn, UART5_IRQn };

DMA_Channel_TypeDef* samp_chans_dma[3] = { DMA2_Channel3, DMA2_Channel4, DMA1_Channel6 };
DMA_Channel_TypeDef* wave_copy_chans_dma[3] = { DMA2_Channel1, DMA1_Channel7, DMA1_Channel1 };
TIM_TypeDef* samp_chans_timers[3] = { TIM6, TIM7, TIM16 };
IRQn_Type samp_chans_IRQ[3] = { DMA2_Channel3_IRQn, DMA2_Channel4_IRQn, DMA1_Channel6_IRQn };

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
		DMA2_Channel4->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->curr_pos;
		DMA2_Channel4->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel4->CCR |= DMA_CCR_EN;
		TIM7->CR1 = TIM_CR1_CEN;

		return;
	}

	if(ss->curr_pos == ss->length && ss->loop)
	{
		ss->curr_pos = ss->loop_point;
		DMA2_Channel4->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->loop_point;
		DMA2_Channel4->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA2_Channel4->CCR |= DMA_CCR_EN;
		TIM7->CR1 = TIM_CR1_CEN;

		return;
	}
}

__attribute__((section (".ccmram")))
void DMA1_Channel6_IRQHandler()
{
	DMA1->IFCR |= DMA_IFCR_CTCIF6;
	DMA1_Channel6->CCR &= ~DMA_CCR_EN; //stop DMA
	TIM16->CR1 &= ~TIM_CR1_CEN;

	Sample_state* ss = &state_ram.dac[2];

	ss->curr_pos += ss->curr_portion_size;

	if(ss->curr_pos < ss->length)
	{
		DMA1_Channel6->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->curr_pos;
		DMA1_Channel6->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA1_Channel6->CCR |= DMA_CCR_EN;
		TIM16->CR1 = TIM_CR1_CEN;

		return;
	}

	if(ss->curr_pos == ss->length && ss->loop)
	{
		ss->curr_pos = ss->loop_point;
		DMA1_Channel6->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset + ss->loop_point;
		DMA1_Channel6->CNDTR = my_min(0xffff, ss->length - ss->curr_pos);
		ss->curr_portion_size = my_min(0xffff, ss->length - ss->curr_pos);

		DMA1_Channel6->CCR |= DMA_CCR_EN;
		TIM16->CR1 = TIM_CR1_CEN;

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
void load_noise_lfsr()
{
	Noise_state* noise = &state_ram.noise;

	//switch clock to GPIO mode
	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_0; //GPIO output

	if(noise->clock_source == 0)
	{
		NOISE_CLOCK_EXTERNAL
	}

	SCK_NOISE_HIGH

	CS_NOISE_LOW

	for(int i = 0; i < 23; i++)
	{
		if(noise->lfsr & (1 << i))
		{
			DATA_NOISE_HIGH
			SCK_NOISE_LOW
		}
		else
		{
			DATA_NOISE_LOW
			SCK_NOISE_LOW
		}

		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");

		SCK_NOISE_HIGH

		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
	}

	CS_NOISE_HIGH
	DATA_NOISE_LOW

	if(noise->clock_source == 0)
	{
		NOISE_CLOCK_INTERNAL
	}

	GPIOB->MODER &= ~GPIO_MODER_MODER7;
	GPIOB->MODER |= GPIO_MODER_MODER7_1; //altfunc output (clock from timer)
}

__attribute__((section (".ccmram")))
inline void phase_reset(uint8_t channel)
{
	switch(channel)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		{
			ad9833_reset(channel);
			break;
		}
		case 4:
		{
			load_noise_lfsr();
			break;
		}
		case 5:
		case 6:
		case 7:
		{
			Sample_state* ss = &state_ram.dac[channel - 5];

			if(ss->wavetable)
			{
				samp_chans_timers[channel - 5]->CR1 &= ~TIM_CR1_CEN;
				samp_chans_dma[channel - 5]->CCR &= ~DMA_CCR_EN;

				NVIC_DisableIRQ(samp_chans_IRQ[channel - 5]);

				samp_chans_dma[channel - 5]->CMAR = (uint32_t)&wavetable_array[channel - 5][0];
				samp_chans_dma[channel - 5]->CNDTR = WAVETABLE_SIZE;
				samp_chans_dma[channel - 5]->CCR |= DMA_CCR_CIRC;

				samp_chans_dma[channel - 5]->CCR |= DMA_CCR_EN;

				samp_chans_timers[channel - 5]->CR1 |= TIM_CR1_CEN;
			}
			else
			{
				samp_chans_timers[channel - 5]->CR1 &= ~TIM_CR1_CEN;
				samp_chans_dma[channel - 5]->CCR &= ~(DMA_CCR_EN);
				samp_chans_dma[channel - 5]->CCR &= ~(DMA_CCR_CIRC);
				ss->curr_pos = 0;
				samp_chans_dma[channel - 5]->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset;
				samp_chans_dma[channel - 5]->CNDTR = my_min(0xffff, ss->length);

				ss->curr_portion_size = my_min(0xffff, ss->length);
				//ss->loop = (command == CMD_DAC_PLAY_SAMPLE_LOOPED ? 1 : 0);

				NVIC_EnableIRQ(samp_chans_IRQ[channel - 5]);

				samp_chans_dma[channel - 5]->CCR |= DMA_CCR_TCIE;

				samp_chans_dma[channel - 5]->CCR |= DMA_CCR_EN;
				samp_chans_timers[channel - 5]->CR1 |= TIM_CR1_CEN;
			}
			break;
		}
		default: break;
	}
}

//phase reset timer 1
__attribute__((section (".ccmram")))
void TIM20_UP_IRQHandler()
{
	CS_NOISE_HIGH
	if(TIM20->SR & TIM_SR_UIF)
	{
		TIM20->SR = ~(TIM_SR_UIF); //clear interrupt

		for(uint8_t i = 0; i < 8; i++)
		{
			if(state_ram.timer[0].chan_bitmask & (1 << i))
			{
				phase_reset(i);
			}
		}
	}
	CS_NOISE_LOW
}

//phase reset timer 2
__attribute__((section (".ccmram")))
void SysTick_Handler()
{
	ZCEN_1_ENABLE
	for(uint8_t i = 0; i < 8; i++)
	{
		if(state_ram.timer[1].chan_bitmask & (1 << i))
		{
			phase_reset(i);
		}
	}
	ZCEN_1_DISABLE
}

//phase reset timer 3
__attribute__((section (".ccmram")))
void USART1_IRQHandler()
{
	ZCEN_2_ENABLE
	if (USART1->ISR & USART_ISR_TXE)
	{
		USART1->TDR = 0; //clear interrupt

		for(uint8_t i = 0; i < 8; i++)
		{
			if(state_ram.timer[2].chan_bitmask & (1 << i))
			{
				phase_reset(i);
			}
		}
	}
	ZCEN_2_DISABLE
}

//phase reset timer 4
__attribute__((section (".ccmram")))
void USART2_IRQHandler()
{
	ZCEN_3_ENABLE
	if (USART2->ISR & USART_ISR_TXE)
	{
		USART2->TDR = 0; //clear interrupt

		for(uint8_t i = 0; i < 8; i++)
		{
			if(state_ram.timer[3].chan_bitmask & (1 << i))
			{
				phase_reset(i);
			}
		}
	}
	ZCEN_3_DISABLE
}

//phase reset timer 5
__attribute__((section (".ccmram")))
void USART3_IRQHandler()
{
	ZCEN_DAC_ENABLE
	if (USART3->ISR & USART_ISR_TXE)
	{
		USART3->TDR = 0; //clear interrupt

		for(uint8_t i = 0; i < 8; i++)
		{
			if(state_ram.timer[4].chan_bitmask & (1 << i))
			{
				phase_reset(i);
			}
		}
	}
	ZCEN_DAC_DISABLE
}

//phase reset timer 6
__attribute__((section (".ccmram")))
void UART5_IRQHandler()
{
	NOISE_CLOCK_EXTERNAL
	if (UART5->ISR & USART_ISR_TXE)
	{
		UART5->TDR = 0; //clear interrupt

		for(uint8_t i = 0; i < 8; i++)
		{
			if(state_ram.timer[5].chan_bitmask & (1 << i))
			{
				phase_reset(i);
			}
		}
	}
	NOISE_CLOCK_INTERNAL
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
	if(curr_read_buf_pos >= EXT_FLASH_RX_BUF_SIZE)
	{
		read_reg_dump(1, 0);
		curr_read_buf_pos = 0;
	}

	if(curr_read_buf_pos == EXT_FLASH_RX_BUF_SIZE / 2)
	{
		read_reg_dump(0, 0);
	}

	uint8_t data = spi_rx_double_buf[curr_read_buf_pos];
	curr_read_buf_pos++;

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

__attribute__((section (".ccmram")))
void pwm_set_duty(uint8_t chan, uint16_t duty)
{
	PSG_state* psg = &state_ram.psg[chan];
	psg->duty = duty;

	switch(chan)
	{
		case 0:
		{
			TIM3->CCR3 = duty * TIM3->ARR / 0xffff;
			break;
		}
		case 1:
		{
			TIM4->CCR1 = duty * TIM4->ARR / 0xffff;
			break;
		}
		case 2:
		{
			TIM8->CCR3 = duty * TIM8->ARR / 0xffff;
			break;
		}
		case 3:
		{
			TIM15->CCR1 = duty * TIM15->ARR / 0xffff;
			break;
		}
		default: break;
	}
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
	for(int i = 5; i < 8; i++) //DAC chans
	{
		chan_base_addr[i] = 8 * 5 + 32 * (i - 5);
	}
	for(int i = 8; i < 14; i++) //phase reset timer chans
	{
		chan_base_addr[i] = 8 * 5 + 32 * 3 + 8 * (i - 8);
	}

	chan_base_addr[14] = 0xff;
}

__attribute__((section (".ccmram")))
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

	for(int i = 0; i < 4; i++)
	{
		pwm_set_duty(i, 0);
		ad9833_write_freq(i, 0);
		ad9833_reset(i);
		connect_dds(i);
	}

	NOISE_CLOCK_EXTERNAL

	TIM20->CR1 &= ~TIM_CR1_CEN;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	for(int i = 0; i < 4; i++)
	{
		NVIC_DisableIRQ(phase_reset_interrupts[i + 2]);
		phase_reset_uarts[i]->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
	}

	NVIC_EnableIRQ(TIM2_IRQn);
	TIM2->CR1 |= TIM_CR1_CEN;
}

void stop_playback()
{
	NVIC_DisableIRQ(TIM2_IRQn);

	TIM2->CR1 &= ~TIM_CR1_CEN;

	for(int i = 0; i < 3; i++)
	{
		samp_chans_dma[i]->CCR &= ~DMA_CCR_EN;
		wave_copy_chans_dma[i]->CCR &= ~DMA_CCR_EN;
		samp_chans_timers[i]->CR1 &= ~TIM_CR1_CEN;
		NVIC_DisableIRQ(samp_chans_IRQ[i]);
	}

	for(int i = 0; i < 4; i++)
	{
		pwm_set_duty(i, 0);
		ad9833_write_freq(i, 0);
		ad9833_reset(i);
		connect_dds(i);
	}

	TIM20->CR1 &= ~TIM_CR1_CEN;
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

	for(int i = 0; i < 4; i++)
	{
		NVIC_DisableIRQ(phase_reset_interrupts[i + 2]);
		phase_reset_uarts[i]->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
	}

	CS_EXT_FLASH_HIGH
}

__attribute__((section (".ccmram")))
void execute_dds_pwm_command(uint8_t chan, uint8_t command)
{
	PSG_state* psg = &state_ram.psg[chan];

	switch(command)
	{
		case CMD_AD9833_VOL:
		{
			psg->volume = reg_dump_read_byte();
			att_write_vol(chan, psg->volume);
			att_need_write = 1;
			break;
		}
		case CMD_AD9833_WAVE_TYPE:
		{
			psg->wave = reg_dump_read_byte();

			if(psg->wave == 5)
			{
				connect_pwm(chan);
			}
			if(psg->wave < 5 && psg->wave > 0)
			{
				ad9833_change_wave(chan, psg->wave - 1);
				connect_dds(chan);
			}
			if(psg->wave == 0)
			{
				ad9833_write_freq(chan, 0);
				ad9833_reset(chan);
				connect_dds(chan);
			}
			break;
		}
		case CMD_AD9833_FREQ:
		{
			ad9833_write_freq(chan, reg_dump_read_four_bytes());
			break;
		}
		case CMD_AD9833_PHASE_RESET:
		{
			if(psg->wave == 5)
			{
				pwm_timers[chan]->CNT = 0;
			}
			if(psg->wave < 5 && psg->wave > 0)
			{
				ad9833_reset(chan);
			}
			break;
		}
		case CMD_AD9833_PWM_FREQ:
		{
			pwm_timers[chan]->CR1 &= ~TIM_CR1_CEN;

			pwm_timers[chan]->PSC = reg_dump_read_byte();
			pwm_timers[chan]->ARR = reg_dump_read_two_bytes();

			if(pwm_timers[chan]->CNT >= pwm_timers[chan]->ARR)
			{
				pwm_timers[chan]->CNT = pwm_timers[chan]->ARR - 3;
			}

			pwm_timers[chan]->CR1 |= TIM_CR1_CEN;

			pwm_set_duty(chan, psg->duty);
			break;
		}
		case CMD_AD9833_PWM_DUTY:
		{
			pwm_set_duty(chan, reg_dump_read_two_bytes());
			break;
		}
		case CMD_AD9833_ZERO_CROSS_ENABLE:
		{
			switch(chan)
			{
				case 0:
				case 1:
				{
					ZCEN_1_ENABLE
					break;
				}
				case 2:
				case 3:
				{
					ZCEN_2_ENABLE
					break;
				}
				default: break;
			}
			break;
		}
		case CMD_AD9833_ZERO_CROSS_DISABLE:
		{
			switch(chan)
			{
				case 0:
				case 1:
				{
					ZCEN_1_DISABLE
					break;
				}
				case 2:
				case 3:
				{
					ZCEN_2_DISABLE
					break;
				}
				default: break;
			}
			break;
		}
		default: break;
	}
}

__attribute__((section (".ccmram")))
void execute_noise_command(uint8_t command)
{
	Noise_state* noise = &state_ram.noise;

	switch(command)
	{
		case CMD_NOISE_VOL:
		{
			noise->volume = reg_dump_read_byte();
			att_write_vol(4, noise->volume);
			att_need_write = 1;
			break;
		}
		case CMD_NOISE_CLOCK_INTERNAL:
		{
			NOISE_CLOCK_INTERNAL
			noise->clock_source = 0;
			break;
		}
		case CMD_NOISE_CLOCK_EXTERNAL:
		{
			NOISE_CLOCK_EXTERNAL
			noise->clock_source = 1;
			break;
		}
		case CMD_NOISE_ZERO_CROSS_ENABLE:
		{
			ZCEN_3_ENABLE
			break;
		}
		case CMD_NOISE_ZERO_CROSS_DISABLE:
		{
			ZCEN_3_DISABLE
			break;
		}
		case CMD_NOISE_FREQ:
		{
			noise_clock_timer->CR1 &= ~TIM_CR1_CEN;

			noise_clock_timer->PSC = reg_dump_read_byte();
			noise_clock_timer->ARR = reg_dump_read_two_bytes();

			if(noise_clock_timer->CNT >= noise_clock_timer->ARR)
			{
				noise_clock_timer->CNT = noise_clock_timer->ARR - 3;
			}

			noise_clock_timer->CCR1 = noise_clock_timer->ARR >> 1; //50% duty cycle clock

			noise_clock_timer->CR1 |= TIM_CR1_CEN;
			break;
		}

		case CMD_NOISE_RESET:
		case CMD_NOISE_LOAD_LFSR:
		{
			if(command == CMD_NOISE_LOAD_LFSR)
			{
				noise->lfsr = reg_dump_read_three_bytes();
			}

			load_noise_lfsr();
			break;
		}

		default: break;
	}
}

__attribute__((section (".ccmram")))
void execute_dac_command(uint8_t chan, uint8_t command)
{
	Sample_state* ss = &state_ram.dac[chan];

	switch(command)
	{
		case CMD_DAC_VOLUME:
		{
			ss->volume = reg_dump_read_byte();

			if(chan < 2)
			{
				att_write_vol(5 + chan, ss->volume);
				att_need_write = 1;
			}
			if(chan == 2)
			{
				TIM1->ARR = 1023 - ss->volume * 3;
			}

			break;
		}
		case CMD_DAC_PLAY_SAMPLE:
		case CMD_DAC_PLAY_SAMPLE_LOOPED:
		{
			samp_chans_timers[chan]->CR1 &= ~TIM_CR1_CEN;
			samp_chans_dma[chan]->CCR &= ~(DMA_CCR_EN);
			samp_chans_dma[chan]->CCR &= ~(DMA_CCR_CIRC);
			ss->curr_pos = 0;
			ss->wavetable = 0;
			samp_chans_dma[chan]->CMAR = (ss->in_ram ? (uint32_t)&sample_mem_ram[0] : BASE_ADDR_FLASH) + ss->start_offset;
			samp_chans_dma[chan]->CNDTR = my_min(0xffff, ss->length);

			ss->curr_portion_size = my_min(0xffff, ss->length);
			ss->loop = (command == CMD_DAC_PLAY_SAMPLE_LOOPED ? 1 : 0);

			NVIC_EnableIRQ(samp_chans_IRQ[chan]);

			samp_chans_dma[chan]->CCR |= DMA_CCR_TCIE;

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

			if(curr_read_buf_pos >= EXT_FLASH_RX_BUF_SIZE / 2 && curr_read_buf_pos - 256 < EXT_FLASH_RX_BUF_SIZE / 2)
			{
				read_reg_dump(0, 0);
			}
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
void execute_phase_reset_timer_command(uint8_t channel, uint8_t command)
{
	Timer_state* timer = &state_ram.timer[channel];

	switch(command)
	{
		case CMD_TIMER_CHANNEL_BITMASK:
		{
			timer->chan_bitmask = reg_dump_read_byte();
			break;
		}
		case CMD_TIMER_FREQ:
		{
			if(channel == 0)
			{
				TIM20->CR1 &= ~TIM_CR1_CEN;

				TIM20->PSC = reg_dump_read_byte();
				TIM20->ARR = reg_dump_read_two_bytes();

				if(TIM20->CNT >= TIM20->ARR)
				{
					TIM20->CNT = TIM20->ARR - 3;
				}

				TIM20->CR1 |= TIM_CR1_CEN;
			}
			break;
		}
		case CMD_TIMER_RESET:
		{
			if(channel == 0)
			{
				TIM20->CR1 &= ~TIM_CR1_CEN;

				TIM20->CNT = 0;

				TIM20->CR1 |= TIM_CR1_CEN;
			}

			if(channel == 1)
			{
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
				SysTick->VAL = SysTick->LOAD;
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
			}

			if(channel > 1 && channel < 6) //UARTs
			{
				NVIC_DisableIRQ(phase_reset_interrupts[channel]);
				phase_reset_uarts[channel - 2]->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
				phase_reset_uarts[channel - 2]->CR1 |= (USART_CR1_TE | USART_CR1_UE);
				//phase_reset_uarts[channel - 2]->TDR = 0;
				NVIC_EnableIRQ(phase_reset_interrupts[channel]);
			}
			break;
		}
		case CMD_TIMER_ENABLE:
		{
			if(channel == 0)
			{
				TIM20->CR1 |= TIM_CR1_CEN;
			}

			if(channel == 1)
			{
				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
			}

			if(channel > 1 && channel < 6) //UARTs
			{
				NVIC_DisableIRQ(phase_reset_interrupts[channel]);
				phase_reset_uarts[channel - 2]->CR1 |= (USART_CR1_TE | USART_CR1_UE);
				NVIC_EnableIRQ(phase_reset_interrupts[channel]);
				//phase_reset_uarts[channel - 2]->TDR = 0;
			}
			break;
		}
		case CMD_TIMER_DISABLE:
		{
			if(channel == 0)
			{
				TIM20->CR1 &= ~TIM_CR1_CEN;
			}

			if(channel == 1)
			{
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
			}

			if(channel > 1 && channel < 6) //UARTs
			{
				NVIC_DisableIRQ(phase_reset_interrupts[channel]);
				phase_reset_uarts[channel - 2]->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
			}
			break;
		}
		case CMD_TIMER_FREQ_SYSTICK:
		{
			if(channel == 1)
			{
				SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
				SysTick->LOAD = ((uint32_t)(reg_dump_read_byte() << 16) | (uint32_t)(reg_dump_read_byte() << 8) | (uint32_t)reg_dump_read_byte());

				if(SysTick->VAL > SysTick->LOAD) SysTick->VAL = SysTick->LOAD;

				SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
			}
			break;
		}
		case CMD_TIMER_FREQ_UART:
		{
			if(channel > 1 && channel < 6) //UARTs
			{
				NVIC_DisableIRQ(phase_reset_interrupts[channel]);
				//phase_reset_uarts[channel - 2]->CR1 &= ~(USART_CR1_TE | USART_CR1_UE);
				phase_reset_uarts[channel - 2]->BRR = reg_dump_read_two_bytes();
				//phase_reset_uarts[channel - 2]->CR1 |= (USART_CR1_TE | USART_CR1_UE);
				//phase_reset_uarts[channel - 2]->TDR = 0;
				NVIC_EnableIRQ(phase_reset_interrupts[channel]);
			}
			break;
		}

		default: break;
	}
}

__attribute__((section (".ccmram")))
void execute_commands()
{
	if(!new_tick) return;

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

		if(curr_command != CMD_NEXT_FRAME)
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
				case 0:
				case 1:
				case 2:
				case 3:
				{
					execute_dds_pwm_command(curr_chan, curr_command - chan_base_addr[curr_chan]);
					break;
				}
				case 4:
				{
					execute_noise_command(curr_command - chan_base_addr[curr_chan]);
					break;
				}
				case 5:
				case 6:
				case 7:
				{
					execute_dac_command(curr_chan - 5, curr_command - chan_base_addr[curr_chan]);
					break;
				}
				case 8:
				case 9:
				case 10:
				case 11:
				case 12:
				case 13:
				{
					execute_phase_reset_timer_command(curr_chan - 8, curr_command - chan_base_addr[curr_chan]);
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
			curr_buf_pos = 0;
			curr_dump_pos = reg_dump_read_four_bytes();
			uint32_t temp = curr_dump_pos;

			CS_EXT_FLASH_HIGH
			curr_read_buf_pos = 0;

			read_reg_dump(0, 1);
			read_reg_dump(1, 0);

			curr_dump_pos = temp;

			if(curr_dump_pos == 0)
			{
				TIM2->ARR = reg_dump_read_four_bytes();
				TIM2->PSC = 0;
			}
		}
	}

	new_tick = 0;

	if(att_need_write)
	{
		att_flush();
	}
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

	TIM6->ARR = 601;
	TIM6->PSC = 0;

	TIM7->ARR = 600;
	TIM7->PSC = 0;

	TIM16->ARR = 500;
	TIM16->PSC = 0;

	/*NVIC_DisableIRQ(DMA2_Channel3_IRQn);

	DMA2_Channel3->CMAR = (uint32_t)&wavetable_array[0][0];
	DMA2_Channel3->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	DMA2_Channel3->CCR |= DMA_CCR_EN;

	NVIC_DisableIRQ(DMA2_Channel4_IRQn);

	DMA2_Channel4->CMAR = (uint32_t)&wavetable_array[1][0];
	DMA2_Channel4->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel4->CCR |= DMA_CCR_CIRC;

	DMA2_Channel4->CCR |= DMA_CCR_EN;*/

	DMA1_Channel6->CMAR = (uint32_t)&wavetable_array[1][0];
	DMA1_Channel6->CNDTR = WAVETABLE_SIZE;
	DMA1_Channel6->CCR |= DMA_CCR_CIRC;

	DMA1_Channel6->CCR |= DMA_CCR_EN;

	NVIC_DisableIRQ(DMA1_Channel6_IRQn);

	//TIM6->CR1 = TIM_CR1_CEN;
	//TIM7->CR1 = TIM_CR1_CEN;

	TIM1->ARR = 255;
	TIM16->CR1 |= TIM_CR1_CEN;
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
	state_ram.dac[0].loop_point = 50000;
	state_ram.dac[0].prescaler = 0;
	state_ram.dac[0].volume = 0xff;
	state_ram.dac[0].wavetable = 0;
	state_ram.dac[0].curr_pos = 0;

	DMA2_Channel3->CCR |= DMA_CCR_EN;

	TIM6->CR1 = TIM_CR1_CEN;
}
