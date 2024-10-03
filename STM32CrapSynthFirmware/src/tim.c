/*
 * tim.c
 *
 *  Created on: 3 окт. 2024 г.
 *      Author: Georg
 */

#include "tim.h"
#include "stm32f3xx.h"

void TIM1_UP_TIM16_IRQHandler()
{

}

void TIM2_IRQHandler()
{

}

void TIM20_UP_IRQHandler()
{

}

void TIM1_TRG_COM_TIM17_IRQHandler()
{

}

void timers_all_init()
{
	//phase reset timer 1
	TIM1->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 8);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);

	//engine tick rate
	TIM2->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM2_IRQn, 8);
	NVIC_EnableIRQ(TIM2_IRQn);

	//PWM ch. 1 -> TIM3 ch.3
	TIM3->CCMR2 |= TIM_CCMR2_OC3CE | TIM_CCMR2_OC3PE;
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;	//<-- pwm mode 2
	TIM3->CCR3 = 0;	//<-- here need put pwm value...
	TIM3->CCER |= TIM_CCER_CC3E;

	//PWM ch. 2 -> TIM4 ch.1
	TIM4->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE;
	TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;	//<-- pwm mode 2
	TIM4->CCR1 = 0;	//<-- here need put pwm value...
	TIM4->CCER |= TIM_CCER_CC1E;

	//DAC ch1 driver
	TIM6->CR2 |= TIM_CR2_MMS_1;	//generate TRG0 after overflow TIM6_CNT

	//DAC ch2 driver
	TIM7->CR2 |= TIM_CR2_MMS_1;	//generate TRG0 after overflow TIM6_CNT

	//PWM ch. 3 -> TIM8 ch.3 complementary
	TIM8->CCMR2 |= TIM_CCMR2_OC3CE | TIM_CCMR2_OC3PE;
	TIM8->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_0;	//<-- pwm mode 2
	TIM8->CCR3 = 0;	//<-- here need put pwm value...
	TIM8->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NP; //TIM_CCER_CC3NP - change polarity (since complementary channel is used)
	TIM8->BDTR |= TIM_BDTR_MOE; //won't work without it

	//PWM ch. 4 -> TIM15 ch.1 complementary
	TIM15->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE;
	TIM15->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;	//<-- pwm mode 2
	TIM15->CCR1 = 0;	//<-- here need put pwm value...
	TIM15->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NP; //TIM_CCER_CC1NP - change polarity (since complementary channel is used)

	//filter clock -> TIM16 ch.1 complementary
	TIM16->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE;
	TIM16->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;	//<-- pwm mode 2
	TIM16->CCR1 = 0;	//<-- here need put pwm value...
	TIM16->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NP; //TIM_CCER_CC1NP - change polarity (since complementary channel is used)

	//noise clock -> TIM17 ch.1 complementary
	TIM17->CCMR1 |= TIM_CCMR1_OC1CE | TIM_CCMR1_OC1PE;
	TIM17->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_0;	//<-- pwm mode 2
	TIM17->CCR1 = 0;	//<-- here need put pwm value...
	TIM17->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NP; //TIM_CCER_CC1NP - change polarity (since complementary channel is used)

	//TIM17->DIER |= TIM_DIER_UIE;
	//NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 8);
	//NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);

	//phase reset timer 2
	TIM20->DIER |= TIM_DIER_UIE;
	NVIC_SetPriority(TIM20_UP_IRQn, 8);
	NVIC_EnableIRQ(TIM20_UP_IRQn);
}
