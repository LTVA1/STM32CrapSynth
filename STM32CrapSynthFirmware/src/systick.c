/*
 * systick.c
 *
 *  Created on: 2 ���. 2024 �.
 *      Author: georg
 */

#include "stm32f3xx.h"

__attribute__((section (".ccmram")))
void SysTick_Handler()
{
	//GPIOA->ODR ^= 1 << 10;
}

void systick_init()
{
	//phase reset timer 3
	SysTick->CTRL = 0; // disable running timer
	SysTick->LOAD = 72000000;
	SysTick->VAL = 72000000;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

	//NVIC_SetPriority(SysTick_IRQn, 8);
	//NVIC_EnableIRQ(SysTick_IRQn);

	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
