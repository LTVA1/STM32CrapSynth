/*
 * external_flash.c
 *
 *  Created on: 12 окт. 2024 г.
 *      Author: georg
 */

#include "external_flash.h"
#include "spi.h"
#include "gpio.h"
#include "stm32f3xx.h"

uint8_t spi_rx_double_buf[EXT_FLASH_RX_BUF_SIZE];
uint8_t spi_tx_buf[EXT_FLASH_TX_BUF_SIZE];

extern uint8_t spi1_ready_tx;
extern uint8_t spi1_ready_rx;

uint16_t manufacturer_id;
uint16_t device_id;
uint64_t device_uid;
uint32_t memory_size;

void external_flash_init_and_request_info()
{
	CS_EXT_FLASH_LOW
	//we assume that the memory we are working with is either w25q80, w25q16, w25q32,  w25q64  or  w25q128
	//                                                          1MiB    2MiB    4MiB    8MiB        16MiB
	//                                                    ID: 0x4014  0x4015  0x4016   0x6017      0x4018 OR 0x7018?
	//                                                                                OR 0x4017?
	//reset
	spi_tx_buf[0] = 0x66;
	spi_tx_buf[1] = 0x99;

	spi1_send_via_dma(&spi_tx_buf[0], 2);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	CS_EXT_FLASH_HIGH

	//100 ms delay
	for(int i = 0; i < 7200000; i++) { asm("nop"); }

	CS_EXT_FLASH_LOW
	//request manufacturer/device ID
	//0x9f, manufacturer ID, device ID MSB, device ID LSB
	spi_tx_buf[0] = 0x9f;

	spi1_send_via_dma(&spi_tx_buf[0], 1);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	spi_tx_buf[0] = 0; //just in case

	spi1_receive_via_dma(&spi_tx_buf[0], &spi_rx_double_buf[0], 3);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	CS_EXT_FLASH_HIGH

	manufacturer_id = spi_rx_double_buf[0];
	device_id = (spi_rx_double_buf[1] << 8) | spi_rx_double_buf[2];

	if(device_id == 0x4014) memory_size = 1024 * 1024;
	if(device_id == 0x4015) memory_size = 1024 * 1024 * 2;
	if(device_id == 0x4016) memory_size = 1024 * 1024 * 4;
	if(device_id == 0x4017 || device_id == 0x6017) memory_size = 1024 * 1024 * 8;
	if(device_id == 0x4018 || device_id == 0x7018) memory_size = 1024 * 1024 * 16;

	CS_EXT_FLASH_LOW
	//write enable
	//0x06
	spi_tx_buf[0] = 0x06;

	spi1_send_via_dma(&spi_tx_buf[0], 1);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	CS_EXT_FLASH_HIGH

	//device unique ID
	//0x4B, dummy, dummy, dummy, dummy, UID MSB to LSB (8 bytes)
	spi_tx_buf[0] = 0x4B;
	spi_tx_buf[1] = 0; //just in case
	spi_tx_buf[2] = 0;
	spi_tx_buf[3] = 0;
	spi_tx_buf[4] = 0;
	spi_tx_buf[5] = 0;
	spi_tx_buf[6] = 0;
	spi_tx_buf[7] = 0;

	CS_EXT_FLASH_LOW

	spi1_send_via_dma(&spi_tx_buf[0], 5);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	spi_tx_buf[0] = 0;

	spi1_receive_via_dma(&spi_tx_buf[0], &spi_rx_double_buf[0], 8);

	while(!spi1_ready_tx || !spi1_ready_rx) { asm("nop"); }

	device_uid =
			((uint64_t)spi_rx_double_buf[0] << 60) |
			((uint64_t)spi_rx_double_buf[1] << 54) |
			((uint64_t)spi_rx_double_buf[2] << 48) |
			((uint64_t)spi_rx_double_buf[3] << 32) |
			((uint64_t)spi_rx_double_buf[4] << 24) |
			((uint64_t)spi_rx_double_buf[5] << 16) |
			((uint64_t)spi_rx_double_buf[6] << 8) |
			(uint64_t)spi_rx_double_buf[7];

	CS_EXT_FLASH_HIGH
}
