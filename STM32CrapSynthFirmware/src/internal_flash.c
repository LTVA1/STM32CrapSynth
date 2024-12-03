/*
 * internal_flash.c
 *
 *  Created on: 2 окт. 2024 г.
 *      Author: georg
 */

#include "internal_flash.h"
#include "main.h"
#include "stm32f3xx.h"
#include "uart.h"
#include "commands.h"

extern Program_state_ccm state_ccm;

void unlock_flash()
{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}

void flash_erase(uint32_t page_address) //pageAddress - any address on the page we erase
{
	while (FLASH->SR & FLASH_SR_BSY) { asm("nop"); }

	if (FLASH->SR & FLASH_SR_EOP)
	{
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = page_address;
	FLASH->CR |= FLASH_CR_STRT;
	while (!(FLASH->SR & FLASH_SR_EOP)) { asm("nop"); }
	FLASH->SR = FLASH_SR_EOP;
	FLASH->CR &= ~FLASH_CR_PER;
}

//data - block data pointer
//address - flash page start address
//count - number of bytes, divisible by 2

void flash_write(uint8_t* data, uint32_t address, uint32_t count)
{
	while (FLASH->SR & FLASH_SR_BSY) { asm("nop"); }

	if (FLASH->SR & FLASH_SR_EOP)
	{
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR |= FLASH_CR_PG;

	for (uint32_t i = 0; i < count; i += 2)
	{
		*(volatile uint16_t*)(address + i) = (((uint16_t)data[i + 1]) << 8) + data[i];
		while (!(FLASH->SR & FLASH_SR_EOP)) { asm("nop"); }
		FLASH->SR = FLASH_SR_EOP;
	}

	FLASH->CR &= ~(FLASH_CR_PG);
}

void write_packet_to_flash()
{
	if(state_ccm.state != STATE_PROG_INTERNAL_FLASH) return;

	flash_erase(state_ccm.block_start_offset + BASE_ADDR_FLASH);
	flash_write(state_ccm.data_pointer, state_ccm.block_start_offset + BASE_ADDR_FLASH, state_ccm.block_length);

	uart_send_response();

	state_ccm.state = STATE_IDLE;
}
