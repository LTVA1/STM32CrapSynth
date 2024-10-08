/*
 * systick.c
 *
 *  Created on: 2 ���. 2024 �.
 *      Author: georg
 */

#include "stm32f3xx.h"

void SysTick_Handler()
{
	//...
}

void systick_init()
{
	//phase reset timer 3
	SysTick->CTRL = 0; // disable running timer
	SysTick->LOAD = 1;
	SysTick->VAL = 1;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;

	NVIC_SetPriority(SysTick_IRQn, 8);
	NVIC_EnableIRQ(SysTick_IRQn);

	//SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}
