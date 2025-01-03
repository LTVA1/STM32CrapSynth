/*
 * gpio.c
 *
 *  Created on: 12 ���. 2024 �.
 *      Author: georg
 */

#include "gpio.h"
#include "stm32f3xx.h"

void gpio_init()
{
	//pull the chip select pins high in advance
	*(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (2)) | (1 << (3)) | (1 << (9)) | (1 << (10)) | (1 << (15));
	*(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (12));
	*(volatile uint32_t*)&(GPIOD->BSRRL) = (1 << (2));

	//PA0 output, PA1 altfun 9, PA2&PA3 output, PA4&PA5 analog, PA9&PA10 output, PA15 output
	//last minute addition: PA8 altfun 6 (PWM DAC TIM1 ch1 output)
	GPIOA->MODER &= ~GPIO_MODER_MODER15;
	GPIOA->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER2_0 | GPIO_MODER_MODER3_0 |
			GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER8_1 | GPIO_MODER_MODER9_0 | GPIO_MODER_MODER10_0 |
			GPIO_MODER_MODER15_0;
	GPIOA->AFR[0] |= 9 << 4;
	GPIOA->AFR[1] |= 6;

	//PB0 altfun 2, PB1 altfun 4, PB3 altfun 5, PB4 altfun 5, PB5 altfun 5, PB6 altfun 2, PB7 altfun 1 (or output),
	//PB8 altfun 1, PB9 output, PB13 altfun 5, PB15 altfun 5
	GPIOB->MODER |= GPIO_MODER_MODER0_1 | GPIO_MODER_MODER1_1 | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 |
			GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1 |
			GPIO_MODER_MODER9_0 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1;
	GPIOB->AFR[0] |= 2 | (4 << 4) | (5 << (4*3)) | (5 << (4*4)) | (5 << (4*5)) | (2 << (4*6)) | (1 << (4*7));
	GPIOB->AFR[1] |= 1 | (5 << (4*5)) | (5 << (4*7));

	//PB4 flash MISO, pull down?
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_1;

	//PC0 output, PC1 output, PC3 output, PC4 altfun 7, PC5 altfun 7, PC6&PC7&PC8&PC9 output, PC12&PC13 output
	GPIOC->MODER |= GPIO_MODER_MODER0_0 | GPIO_MODER_MODER1_0 | GPIO_MODER_MODER3_0 | GPIO_MODER_MODER4_1 |
			 GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER7_0 | GPIO_MODER_MODER8_0 | GPIO_MODER_MODER9_0 |
			 GPIO_MODER_MODER12_0 | GPIO_MODER_MODER13_0;
	GPIOC->AFR[0] |= (7 << (4*4)) | (7 << (4*5));

	//UART4 RX&TX
	GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
	GPIOC->MODER |= GPIO_MODER_MODER10_1 | GPIO_MODER_MODER11_1;
	GPIOC->AFR[1] |= (5 << (4*2)) | (5 << (4*3));

	//PD2 output
	GPIOD->MODER |= GPIO_MODER_MODER2_0;
}

__attribute__((section (".ccmram")))
void connect_dds(uint8_t chan)
{
	switch(chan)
	{
		case 0:
		{
			CH1_CONNECT_DDS
			break;
		}
		case 1:
		{
			CH2_CONNECT_DDS
			break;
		}
		case 2:
		{
			CH3_CONNECT_DDS
			break;
		}
		case 3:
		{
			CH4_CONNECT_DDS
			break;
		}
		default: break;
	}
}

__attribute__((section (".ccmram")))
void connect_pwm(uint8_t chan)
{
	switch(chan)
	{
		case 0:
		{
			CH1_CONNECT_PWM
			break;
		}
		case 1:
		{
			CH2_CONNECT_PWM
			break;
		}
		case 2:
		{
			CH3_CONNECT_PWM
			break;
		}
		case 3:
		{
			CH4_CONNECT_PWM
			break;
		}
		default: break;
	}
}
