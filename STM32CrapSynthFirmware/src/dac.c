/*
 * dac.c
 *
 *  Created on: 8 окт. 2024 г.
 *      Author: Georg
 */

#include "dac.h"
#include "stm32f3xx.h"

void dac_init()
{
	DAC->CR |= DAC_CR_BOFF1 | DAC_CR_BOFF2 |
			DAC_CR_DMAEN1 | DAC_CR_DMAEN2 |
			DAC_CR_TSEL2_1; // timer 7, ch1 timer 6
	DAC->CR |= DAC_CR_EN1 | DAC_CR_EN2;
}
