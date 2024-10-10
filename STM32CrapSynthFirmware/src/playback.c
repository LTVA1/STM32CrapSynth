/*
 * playback.c
 *
 *  Created on: 9 окт. 2024 г.
 *      Author: georg
 */

#include "playback.h"
#include "stm32f3xx.h"

uint8_t wavetable_array[2][WAVETABLE_SIZE];
uint32_t noise_lfsr_load[NOISE_LFSR_LENGTH * 2];
