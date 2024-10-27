/*
 * commands.c
 *
 *  Created on: 13 окт. 2024 г.
 *      Author: georg
 */

#include "commands.h"
#include "main.h"
#include "stm32f3xx.h"
#include "uart.h"
#include "ring_buf.h"
#include <string.h>

extern Program_state_ccm state_ccm;
extern RingBuffer rx_ring_buf;
extern uint8_t tx_buf[];
extern uint8_t sample_mem_ram[];

void decode_command()
{
	static uint8_t state = CMD_DECODE_STATE_BEGIN;
	static uint8_t cmd_buf[RX_BUF_SIZE];
	static uint16_t cmd_buf_pointer;
	static uint8_t curr_byte;
	static uint8_t buf_error;
	static uint8_t our_xor;
	static uint16_t size;

	buf_error = buffer_get_from_front(&rx_ring_buf, &curr_byte);

	if(buf_error) return; //empty buffer or whatever

	switch(state)
	{
		case CMD_DECODE_STATE_BEGIN:
		{
			if(curr_byte == SYNTH_SYNC_BYTE)
			{
				cmd_buf_pointer = 0;
				cmd_buf[cmd_buf_pointer] = curr_byte;
				++cmd_buf_pointer;
				our_xor = 0;

				our_xor ^= curr_byte;

				state = CMD_DECODE_STATE_READ_SIZE;
				size = 0;
			}
			break;
		}

		case CMD_DECODE_STATE_READ_SIZE:
		{
			size <<= 8;
			cmd_buf[cmd_buf_pointer] = curr_byte;
			size += curr_byte;

			++cmd_buf_pointer;
			our_xor ^= curr_byte;

			if(cmd_buf_pointer == 3)
			{
				state = CMD_DECODE_STATE_READ_CMD;
			}
			break;
		}

		case CMD_DECODE_STATE_READ_CMD:
		{
			cmd_buf[cmd_buf_pointer] = curr_byte;
			++cmd_buf_pointer;
			our_xor ^= curr_byte;

			if(cmd_buf_pointer == size) //size + sync byte = 3 bytes
			{
				state = CMD_DECODE_STATE_READ_XOR;
			}
			break;
		}

		case CMD_DECODE_STATE_READ_XOR:
		{
			cmd_buf[cmd_buf_pointer] = curr_byte;
			cmd_buf_pointer = 0;

			if(our_xor == curr_byte && cmd_buf[3] < SYNTH_CMD_MAX)
			{
				if(cmd_buf[3] == SYNTH_CMD_LOAD_FLASH || cmd_buf[3] == SYNTH_CMD_LOAD_RAM ||
						cmd_buf[3] == SYNTH_CMD_LOAD_EXT_FLASH)
				{
					state_ccm.block_start_offset = cmd_buf[4] + ((uint32_t)cmd_buf[5] << 8) +
							((uint32_t)cmd_buf[6] << 16) + ((uint32_t)cmd_buf[7] << 24);
					state_ccm.block_length = size - 8;
					state_ccm.data_pointer = &cmd_buf[8];
					state_ccm.last_packet_xor = our_xor;

					if(cmd_buf[3] == SYNTH_CMD_LOAD_RAM)
					{
						memcpy(&sample_mem_ram[state_ccm.block_start_offset], state_ccm.data_pointer, state_ccm.block_length);
						state_ccm.state = STATE_PROG_RAM;
						uart_send_response();
						state_ccm.state = STATE_IDLE;

						//play_sample();
					}
					if(cmd_buf[3] == SYNTH_CMD_LOAD_FLASH)
					{
						state_ccm.state = STATE_PROG_INTERNAL_FLASH;
					}
					if(cmd_buf[3] == SYNTH_CMD_LOAD_EXT_FLASH)
					{
						state_ccm.state = STATE_PROG_EXTERNAL_FLASH;
					}
				}
			}

			if(our_xor != curr_byte && cmd_buf[3] < SYNTH_CMD_MAX)
			{
				our_xor = 0;
				tx_buf[0] = SYNTH_SYNC_BYTE;
				tx_buf[1] = SYNTH_RESPONSE_BAD_XOR;

				for(int i = 2; i < SYNTH_RESPONSE_SIZE; i++)
				{
					tx_buf[i] = 0;
				}
				for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
				{
					our_xor ^= tx_buf[i];
				}

				tx_buf[SYNTH_RESPONSE_SIZE - 1] = our_xor;
				uart_send_via_dma(SYNTH_RESPONSE_SIZE);
			}

			if(our_xor == curr_byte && cmd_buf[3] >= SYNTH_CMD_MAX)
			{
				our_xor = 0;
				tx_buf[0] = SYNTH_SYNC_BYTE;
				tx_buf[1] = SYNTH_RESPONSE_UNKNOWN_COMMAND;

				for(int i = 2; i < SYNTH_RESPONSE_SIZE; i++)
				{
					tx_buf[i] = 0;
				}
				for(int i = 0; i < SYNTH_RESPONSE_SIZE - 1; i++)
				{
					our_xor ^= tx_buf[i];
				}

				tx_buf[SYNTH_RESPONSE_SIZE - 1] = our_xor;
				uart_send_via_dma(SYNTH_RESPONSE_SIZE);
			}

			state = CMD_DECODE_STATE_BEGIN;
			our_xor = 0;
			size = 0;
			break;
		}
		default: break;
	}
}
