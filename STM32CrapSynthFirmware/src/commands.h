/*
 * commands.h
 *
 *  Created on: 13 ���. 2024 �.
 *      Author: georg
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#define FIRMWARE_VERSION 0x00000001

#define MCU_FIRMWARE_SIZE (1024 * 16) /*16 KiB*/
#define BASE_ADDR_FLASH (0x08000000 + MCU_FIRMWARE_SIZE)
#define SUPPORTED_FILE_VERSION 1

#define SYNTH_SYNC_BYTE 0x69

#define SYNTH_CMD_LOAD_FLASH 0
#define SYNTH_CMD_LOAD_RAM 1
#define SYNTH_CMD_LOAD_EXT_FLASH 2
#define SYNTH_CMD_PLAYBACK_START 3
#define SYNTH_CMD_PLAYBACK_STOP 4
#define SYNTH_CMD_MAX 5

#define SYNTH_RESPONSE_SIZE 48

#define SYNTH_RESPONSE_READY 0
#define SYNTH_RESPONSE_FLASH_BLOCK 1
#define SYNTH_RESPONSE_RAM_BLOCK 2
#define SYNTH_RESPONSE_EXTERNAL_FLASH_BLOCK 3
#define SYNTH_RESPONSE_BAD_XOR 4
#define SYNTH_RESPONSE_UNKNOWN_COMMAND 5

#define CMD_DECODE_STATE_BEGIN 0
#define CMD_DECODE_STATE_READ_SIZE 1
#define CMD_DECODE_STATE_READ_CMD 2
#define CMD_DECODE_STATE_READ_XOR 3

void decode_command();

#endif /* COMMANDS_H_ */
