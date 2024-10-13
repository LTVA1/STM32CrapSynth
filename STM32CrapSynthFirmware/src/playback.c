/*
 * playback.c
 *
 *  Created on: 9 окт. 2024 г.
 *      Author: georg
 */

#include "playback.h"
#include "stm32f3xx.h"
#include "main.h"

uint8_t wavetable_array[2][WAVETABLE_SIZE];
uint32_t noise_lfsr_load[NOISE_LFSR_LENGTH * 2];
uint8_t sample_mem_ram[SAMPLE_MEM_RAM_SIZE];

extern Program_state_ccm state_ccm;
extern Program_state_ram state_ram;
