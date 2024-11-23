/*
 * playback.h
 *
 *  Created on: 9 окт. 2024 г.
 *      Author: georg
 */

#ifndef PLAYBACK_H_
#define PLAYBACK_H_

#include <stdio.h>
#include <stdint.h>

#define WAVETABLE_SIZE 256
#define NOISE_LFSR_LENGTH 23
#define SAMPLE_MEM_RAM_SIZE (50 * 1024)

//commands
#define CMD_AD9833_VOL 0
#define CMD_AD9833_WAVE_TYPE 1
#define CMD_AD9833_FREQ 2 /*28 bits*/
#define CMD_AD9833_PHASE_RESET 3
#define CMD_AD9833_PWM_FREQ 4 /*8 bits prescaler and 16 bits autoreload*/
#define CMD_AD9833_PWM_DUTY 5
#define CMD_AD9833_ZERO_CROSS_ENABLE 6
#define CMD_AD9833_ZERO_CROSS_DISABLE 7

#define CMD_NOISE_VOL 0
#define CMD_NOISE_CLOCK_INTERNAL 1
#define CMD_NOISE_CLOCK_EXTERNAL 2
#define CMD_NOISE_ZERO_CROSS_ENABLE 3
#define CMD_NOISE_ZERO_CROSS_DISABLE 4
#define CMD_NOISE_FREQ 5
#define CMD_NOISE_RESET 6
#define CMD_NOISE_LOAD_LFSR 7

#define CMD_DAC_VOLUME 0
#define CMD_DAC_PLAY_SAMPLE 1
#define CMD_DAC_PLAY_SAMPLE_LOOPED 2
#define CMD_DAC_PLAY_WAVETABLE 3
#define CMD_DAC_STOP 4
#define CMD_DAC_START_ADDR_FLASH 5
#define CMD_DAC_START_ADDR_RAM 6
#define CMD_DAC_RESET 7
#define CMD_DAC_TIMER_FREQ 8
#define CMD_DAC_ZERO_CROSS_ENABLE 9
#define CMD_DAC_ZERO_CROSS_DISABLE 10
#define CMD_DAC_LOOP_POINT_FLASH 11
#define CMD_DAC_LOOP_POINT_RAM 12
#define CMD_DAC_LENGTH_FLASH 13
#define CMD_DAC_LENGTH_RAM 14
#define CMD_DAC_WAVE_TYPE 15
#define CMD_DAC_DUTY 16
#define CMD_DAC_WAVETABLE_DATA 17
#define CMD_DAC_NOISE_TRI_AMP 18

#define CMD_TIMER_CHANNEL_BITMASK 0
#define CMD_TIMER_FREQ 1
#define CMD_TIMER_RESET 2
#define CMD_TIMER_ENABLE 3
#define CMD_TIMER_DISABLE 4

#define CMD_NEXT_FRAME 0xfb
#define CMD_SET_RATE 0xfc
#define CMD_LOOP_POINT 0xfd
#define CMD_END 0xfe
#define CMD_NOP 0xff

void playback_init();
void start_playback();
void stop_playback();
void execute_commands();

#endif /* PLAYBACK_H_ */
