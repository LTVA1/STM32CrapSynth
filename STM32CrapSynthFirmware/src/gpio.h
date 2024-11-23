/*
 * gpio.h
 *
 *  Created on: 12 окт. 2024 г.
 *      Author: georg
 */

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

#define CS_AD1_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (3 + 16));
#define CS_AD1_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (3));
#define CS_AD2_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (9 + 16));
#define CS_AD2_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (9));
#define CS_AD3_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (10 + 16));
#define CS_AD3_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (10));
#define CS_AD4_LOW *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (12 + 16));
#define CS_AD4_HIGH *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (12));

#define CH1_CONNECT_DDS *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (0 + 16));
#define CH1_CONNECT_PWM *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (0));
#define CH2_CONNECT_DDS *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (3 + 16));
#define CH2_CONNECT_PWM *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (3));
#define CH3_CONNECT_DDS *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (13 + 16));
#define CH3_CONNECT_PWM *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (13));
#define CH4_CONNECT_DDS *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (0 + 16));
#define CH4_CONNECT_PWM *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (0));

#define NOISE_CLOCK_INTERNAL *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (1));
#define NOISE_CLOCK_EXTERNAL *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (1 + 16));

#define CS_EXT_FLASH_LOW *(volatile uint32_t*)&(GPIOD->BSRRL) = (1 << (2 + 16));
#define CS_EXT_FLASH_HIGH *(volatile uint32_t*)&(GPIOD->BSRRL) = (1 << (2));

#define CS_NOISE_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (15 + 16));
#define CS_NOISE_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (15));

#define SCK_NOISE_LOW *(volatile uint32_t*)&(GPIOB->BSRRL) = (1 << (7 + 16));
#define SCK_NOISE_HIGH *(volatile uint32_t*)&(GPIOB->BSRRL) = (1 << (7));

#define DATA_NOISE_LOW *(volatile uint32_t*)&(GPIOB->BSRRL) = (1 << (9 + 16));
#define DATA_NOISE_HIGH *(volatile uint32_t*)&(GPIOB->BSRRL) = (1 << (9));

#define CS_ATTEN_LOW *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (2 + 16));
#define CS_ATTEN_HIGH *(volatile uint32_t*)&(GPIOA->BSRRL) = (1 << (2));

#define ZCEN_1_ENABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (6));
#define ZCEN_1_DISABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (6 + 16));

#define ZCEN_2_ENABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (7));
#define ZCEN_2_DISABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (7 + 16));

#define ZCEN_3_ENABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (8));
#define ZCEN_3_DISABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (8 + 16));

#define ZCEN_DAC_ENABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (9));
#define ZCEN_DAC_DISABLE *(volatile uint32_t*)&(GPIOC->BSRRL) = (1 << (9 + 16));

void gpio_init();
void connect_dds(uint8_t chan);
void connect_pwm(uint8_t chan);

#endif /* GPIO_H_ */
