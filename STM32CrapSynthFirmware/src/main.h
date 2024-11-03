/*
 * main.h
 *
 *  Created on: 13 окт. 2024 г.
 *      Author: georg
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdint.h>
#include "playback.h"

#define STATE_IDLE 0
#define STATE_PROG_INTERNAL_FLASH 1
#define STATE_PROG_EXTERNAL_FLASH 2
#define STATE_PROG_RAM 3

#define my_min(a,b) (((a)<(b))?(a):(b))
#define my_max(a,b) (((a)>(b))?(a):(b))

typedef struct
{
	uint8_t state;
	uint32_t block_start_offset; // shared between writing to flash (internal and external) and writing to RAM
	uint16_t block_length;
	uint8_t* data_pointer;
	uint8_t last_packet_xor;
} Program_state_ccm; //in CCM for faster code execution

typedef struct
{
	uint8_t wave;
	uint16_t autoreload;
	uint16_t prescaler;
	uint8_t enable;
	uint8_t volume;
	uint16_t duty;
} PSG_state;

typedef struct
{
	uint32_t lfsr;
	uint16_t autoreload;
	uint16_t prescaler;
	uint8_t volume;
	uint8_t enable;
	uint8_t clock_source; //internal or external
} Noise_state;

typedef struct
{
	uint8_t wavetable;
	uint8_t in_ram; // if sample in RAM
	uint8_t loop; // if sample looped
	uint32_t start_offset;
	uint32_t length;
	uint32_t loop_point;
	uint32_t curr_pos;
	uint8_t volume;
	uint16_t autoreload;
	uint16_t prescaler;
	uint16_t curr_portion_size;
	uint8_t wave_type;
	uint8_t duty;
} Sample_state;

typedef struct
{
	uint32_t autoreload; //RTC wakeup timer effectively has 17-bit autoreload
	uint16_t prescaler;
	uint8_t enable;
	uint8_t chan_bitmask;
} Timer_state;

typedef struct
{
	PSG_state psg[4];
	Noise_state noise;
	Sample_state dac[2];
	Timer_state timer[5]; //last timer shared between phase reset function and external noise clock source function
} Program_state_ram; //in RAM so that it would be sent to PC via UART DMA

#endif /* MAIN_H_ */
