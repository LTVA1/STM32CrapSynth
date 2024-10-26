/*
 * playback.c
 *
 *  Created on: 9 окт. 2024 г.
 *      Author: georg
 */

#include "playback.h"
#include "stm32f3xx.h"
#include "main.h"

uint8_t wavetable_array[2][WAVETABLE_SIZE];
uint32_t noise_lfsr_load[NOISE_LFSR_LENGTH * 2];
uint8_t sample_mem_ram[SAMPLE_MEM_RAM_SIZE];

extern Program_state_ccm state_ccm;
extern Program_state_ram state_ram;

void play_wavetable(uint8_t channel)
{
	for(int i = 0; i < WAVETABLE_SIZE; i++)
	{
		wavetable_array[channel][i] = i;
	}

	wavetable_array[channel][200] = 0xff;

	TIM6->ARR = 720;
	TIM6->PSC = 0;

	DMA2_Channel3->CMAR = &wavetable_array[0][0];
	DMA2_Channel3->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	DMA2_Channel3->CCR |= DMA_CCR_EN;

	TIM6->CR1 = TIM_CR1_CEN;
}
