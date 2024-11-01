/*
 * clocks_interrupts.c
 *
 *  Created on: 26 окт. 2024 г.
 *      Author: georg
 */

#include "stm32f3xx.h"

void remap_and_place_interrupt_vectors_table_to_ccmram()
{
	for(int i = 0; i < 404; i += 4)
	{
		*((uint32_t*)(0x10000000 + i)) = *((uint32_t*)(0x08000000 + i)); //dumb copy data (vector interrupt table) to CCMRAM from Flash
	}

	SCB->VTOR = 0x10000000;
}

void enable_all_clocks()
{
	RCC->AHBENR |= RCC_AHBENR_DMA1EN | RCC_AHBENR_DMA2EN | RCC_AHBENR_FLITFEN |
			RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
			RCC_AHBENR_GPIODEN | RCC_AHBENR_ADC12EN | RCC_AHBENR_ADC34EN;

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM3EN | RCC_APB1ENR_TIM4EN |
			RCC_APB1ENR_TIM6EN | RCC_APB1ENR_TIM7EN | RCC_APB1ENR_SPI2EN |
			RCC_APB1ENR_PWREN | RCC_APB1ENR_DAC1EN | RCC_APB1ENR_PWREN | RCC_APB1ENR_UART4EN;

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_SPI1EN | RCC_APB2ENR_TIM20EN |
			RCC_APB2ENR_TIM8EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_TIM15EN |
			RCC_APB2ENR_TIM16EN | RCC_APB2ENR_TIM17EN | RCC_APB2ENR_SYSCFGEN;

	RCC->APB2RSTR |= RCC_APB2RSTR_SPI1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

	PWR->CR |= PWR_CR_DBP; // allow writes to RTC registers...

	RCC->BDCR |= RCC_BDCR_RTCSEL_HSE; // in advance
	RCC->BDCR |= RCC_BDCR_RTCEN;

	/*DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM2_STOP | DBGMCU_APB1_FZ_DBG_TIM3_STOP |
			DBGMCU_APB1_FZ_DBG_TIM4_STOP | DBGMCU_APB1_FZ_DBG_TIM6_STOP |
			DBGMCU_APB1_FZ_DBG_TIM7_STOP | DBGMCU_APB1_FZ_DBG_RTC_STOP;

	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM1_STOP | DBGMCU_APB2_FZ_DBG_TIM8_STOP |
			DBGMCU_APB2_FZ_DBG_TIM15_STOP | DBGMCU_APB2_FZ_DBG_TIM16_STOP |
			DBGMCU_APB2_FZ_DBG_TIM17_STOP | DBGMCU_APB2_FZ_DBG_TIM20_STOP;*/
}

void set_72MHz()
{
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Ждем пока HSE не выставит бит готовности */
	while(!(RCC->CR & RCC_CR_HSERDY)) {}

	/* Конфигурируем Flash на 2 цикла ожидания */
	/* Flash не может работать на высокой частоте */
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= FLASH_ACR_LATENCY_2;

	/* HCLK = SYSCLK */
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	/* PCLK2 = HCLK */
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	/* PCLK1 = HCLK */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	/* Конфигурируем множитель PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
	/* При условии, что кварц на 8МГц! */
	RCC->CFGR &= ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMUL);
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE_PREDIV | RCC_CFGR_PLLMUL9;

	/* Включаем PLL */
	RCC->CR |= RCC_CR_PLLON;

	/* Ожидаем, пока PLL выставит бит готовности */
	while((RCC->CR & RCC_CR_PLLRDY) == 0) { asm("nop"); }

	/* Выбираем PLL как источник системной частоты */
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	/* Ожидаем, пока PLL выберется как источник системной частоты */
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) { asm("nop"); }

	SystemCoreClockUpdate();
}
