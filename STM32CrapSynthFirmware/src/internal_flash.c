/*
 * internal_flash.c
 *
 *  Created on: 2 ���. 2024 �.
 *      Author: georg
 */

#include "internal_flash.h"
#include "stm32f3xx.h"

void unlock_flash()
{
	FLASH->KEYR = 0x45670123;
	FLASH->KEYR = 0xCDEF89AB;
}
