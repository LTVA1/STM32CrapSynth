/*
 * rtc.c
 *
 *  Created on: 2 окт. 2024 г.
 *      Author: georg
 */

#include "rtc.h"
#include "stm32f3xx.h"

void RTC_WKUP_IRQHandler()
{
	RTC->ISR = ~(RTC_ISR_WUTF);
	EXTI->PR = EXTI_PR_PR20;

	//...
}

void rtc_init()
{
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	RTC->CR &= RTC_CR_WUTE;

	while(!(RTC->ISR & RTC_ISR_WUTWF)) {}

	RTC->CR &= RTC_CR_WUCKSEL;
	RTC->CR |= RTC_CR_WUCKSEL_2;
	RTC->CR |= RTC_CR_WUTIE;

	RTC->PRER = 0; // disable prescalers

	EXTI->IMR |= EXTI_IMR_MR20;
	EXTI->RTSR |= EXTI_RTSR_TR20;

	NVIC_SetPriority(RTC_WKUP_IRQn, 8);
	NVIC_EnableIRQ(RTC_WKUP_IRQn);

	//RTC_CR_WUCKSEL_1 is used as 17th bit in autoreload value. RTC->WUTR is the autorealod reg
	//RTC->CR |= RTC_CR_WUTE;
}
