/*
 * dma.c
 *
 *  Created on: 9 ���. 2024 �.
 *      Author: georg
 */

#include "dma.h"
#include "stm32f3xx.h"
#include "playback.h"

extern uint8_t wavetable_array[2][WAVETABLE_SIZE];
extern uint32_t noise_lfsr_load[];

void dma_init()
{
	//some of the used DMA channels' settings aren't going to change throughout the whole program,
	//so it's optimal to define them during initialization

	//DMA channels mapping:

	//DMA1_Channel1 - TIM17-driven noise chip LFSR load ("SPI")
	//DMA1_Channel2 - SPI1 RX
	//DMA1_Channel3 - SPI1 TX
	//DMA1_Channel4 - USART1 TX
	//DMA1_Channel5 - SPI2 TX (16 bits)
	//DMA1_Channel6 - Copy wavetable data for DAC ch.1 (MEM2MEM)
	//DMA1_Channel7 - Copy wavetable data for DAC ch.2 (MEM2MEM)

	//DMA2_Channel3 - DAC ch.1 request
	//DMA2_Channel4 - DAC ch.2 request

	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_MSIZE_1; //32 bits
	DMA1_Channel1->CPAR = (uint32_t)(&(GPIOB->BSRRL));
	DMA1_Channel1->CMAR = (uint32_t)(&noise_lfsr_load[0]);
	NVIC_SetPriority(DMA1_Channel1_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_TCIE;
	DMA1_Channel2->CPAR = (uint32_t)(&(SPI1->DR));
	NVIC_SetPriority(DMA1_Channel2_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);

	DMA1_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
	DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR));
	NVIC_SetPriority(DMA1_Channel3_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);

	DMA1_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
	DMA1_Channel4->CPAR = (uint32_t)(&(USART1->TDR));
	NVIC_SetPriority(DMA1_Channel4_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel4_IRQn);

	DMA1_Channel5->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_DIR | DMA_CCR_TCIE; //16 bits
	DMA1_Channel5->CPAR = (uint32_t)(&(SPI2->DR));
	NVIC_SetPriority(DMA1_Channel5_IRQn, 8);
	NVIC_EnableIRQ(DMA1_Channel5_IRQn);

	DMA1_Channel6->CCR |= DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_MEM2MEM;
	DMA1_Channel6->CPAR = (uint32_t)(&wavetable_array[0][0]);

	DMA1_Channel7->CCR |= DMA_CCR_MINC | DMA_CCR_PINC | DMA_CCR_MEM2MEM;
	DMA1_Channel7->CPAR = (uint32_t)(&wavetable_array[1][0]);

	DMA2_Channel3->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE; //here we may disable interrupt later if it's looped sample without custom loop point or wavetable
	DMA2_Channel3->CPAR = (uint32_t)(&DAC1->DHR8R1);
	NVIC_SetPriority(DMA2_Channel3_IRQn, 8);
	NVIC_EnableIRQ(DMA2_Channel3_IRQn);

	DMA2_Channel4->CCR |= DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;
	DMA2_Channel4->CPAR = (uint32_t)(&DAC1->DHR8R2);
	NVIC_SetPriority(DMA2_Channel4_IRQn, 8);
	NVIC_EnableIRQ(DMA2_Channel4_IRQn);
}
