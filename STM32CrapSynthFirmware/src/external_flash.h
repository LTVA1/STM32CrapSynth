/*
 * external_flash.h
 *
 *  Created on: 12 окт. 2024 г.
 *      Author: georg
 */

#ifndef EXTERNAL_FLASH_H_
#define EXTERNAL_FLASH_H_

#include <stdint.h>

#define EXT_FLASH_RX_BUF_SIZE 8192
#define EXT_FLASH_RX_BUF_SIZE_HALF (EXT_FLASH_RX_BUF_SIZE / 2)
#define EXT_FLASH_TX_BUF_SIZE 32

void external_flash_init_and_request_info();
void external_flash_write_page_task();
void external_flash_read_data(uint32_t address, uint8_t* data, uint16_t size, uint8_t start);

#endif /* EXTERNAL_FLASH_H_ */
