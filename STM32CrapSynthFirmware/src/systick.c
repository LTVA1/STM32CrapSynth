/*
 * systick.c
 *
 *  Created on: 2 окт. 2024 г.
 *      Author: georg
 */

#include "stm32f3xx.h"

void systick_init()
{
	//phase reset timer 3
	SysTick->CTRL = 0; // disable running timer
	SysTick->LOAD = 7200000;
	SysTick->VAL = 7200000;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk;
}
