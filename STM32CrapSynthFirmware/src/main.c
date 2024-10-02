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

void enable_all_clocks()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN | RCC_AHBENR_FLITFEN |
			RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
			RCC_AHBENR_GPIODEN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
			RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_SPI2EN |
			RCC_APB1ENR_PWREN | RCC_APB1ENR_DAC1EN | RCC_APB1ENR_PWREN;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM20EN |
			RCC_APB2ENR_TIM8EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN |
			RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN | RCC_APB2ENR_SYSCFGEN;

	PWR->CR |= PWR_CR_DBP; // allow writes to RTC registers...

	RCC->BDCR |= RCC_BDCR_RTCSEL_HSE; // in advance
	RCC->BDCR |= RCC_BDCR_RTCEN;
}

void set_72MHz()
{
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* ���� ���� HSE �� �������� ��� ���������� */
	while(!(RCC->CR & RCC_CR_HSERDY)) {}

	/* ������������� Flash �� 2 ����� �������� */
	/* Flash �� ����� �������� �� ������� ������� */
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	/* HCLK = SYSCLK */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK */
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/* PCLK1 = HCLK */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	/* ������������� ��������� PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
	/* ��� �������, ��� ����� �� 8���! */
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9;

	/* �������� PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* �������, ���� PLL �������� ��� ���������� */
	while((RCC->CR & RCC_CR_PLLRDY) == 0) {}

	/* �������� PLL ��� �������� ��������� ������� */
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* �������, ���� PLL ��������� ��� �������� ��������� ������� */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {}

	SystemCoreClockUpdate();
}

int main(void)
{
	enable_all_clocks(); // in that order because http://efton.sk/STM32/gotcha/g183.html
	set_72MHz();
	unlock_flash();
	systick_init();
	rtc_init();

	while(1)
	{

	}

  return 0;
}
