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
		wavetable_array[0][i] = i;
	}

	//wavetable_array[0][200] = 0xff;

	for(int i = 0; i < WAVETABLE_SIZE; i++)
	{
		wavetable_array[1][i] = (i > 127) ? ~(i * 2) : (i * 2);
	}

	//wavetable_array[1][210] = 0xff;

	TIM6->ARR = 300;
	TIM6->PSC = 0;

	TIM7->ARR = 300;
	TIM7->PSC = 0;

	NVIC_DisableIRQ(DMA2_Channel3_IRQn);

	DMA2_Channel3->CMAR = (uint32_t)&wavetable_array[0][0];
	DMA2_Channel3->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel3->CCR |= DMA_CCR_CIRC;

	DMA2_Channel3->CCR |= DMA_CCR_EN;

	NVIC_DisableIRQ(DMA2_Channel4_IRQn);

	DMA2_Channel4->CMAR = (uint32_t)&wavetable_array[1][0];
	DMA2_Channel4->CNDTR = WAVETABLE_SIZE;
	DMA2_Channel4->CCR |= DMA_CCR_CIRC;

	DMA2_Channel4->CCR |= DMA_CCR_EN;

	TIM6->CR1 = TIM_CR1_CEN;
	TIM7->CR1 = TIM_CR1_CEN;
}
