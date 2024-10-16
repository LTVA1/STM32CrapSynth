/*
 * gpio.h
 *
 *  Created on: 12 ���. 2024 �.
 *      Author: georg
 */

#ifndef GPIO_H_
#define GPIO_H_

#define CS_AD1_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (3 + 16));
#define CS_AD1_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (3));
#define CS_AD2_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (9 + 16));
#define CS_AD2_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (9));
#define CS_AD3_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (10 + 16));
#define CS_AD3_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (10));
#define CS_AD4_LOW *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (12 + 16));
#define CS_AD4_HIGH *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (12));

#define CS_EXT_FLASH_LOW *(volatile uint32_t*)&(GPIOD->BSRRL) = (1 << (2 + 16));
#define CS_EXT_FLASH_HIGH *(volatile uint32_t*)&(GPIOD->BSRRL) = (1 << (2));

#define CS_NOISE_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (15 + 16));
#define CS_NOISE_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (15));

#define CS_ATTEN_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (2 + 16));
#define CS_ATTEN_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (2));

void gpio_init();

#endif /* GPIO_H_ */
