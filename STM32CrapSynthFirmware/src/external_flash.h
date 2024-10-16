/*
 * external_flash.h
 *
 *  Created on: 12 окт. 2024 г.
 *      Author: georg
 */

#ifndef EXTERNAL_FLASH_H_
#define EXTERNAL_FLASH_H_

#define EXT_FLASH_RX_BUF_SIZE 8192
#define EXT_FLASH_RX_BUF_SIZE_HALF (EXT_FLASH_RX_BUF_SIZE / 2)
#define EXT_FLASH_TX_BUF_SIZE 32

void external_flash_init_and_request_info();
void external_flash_write_page_task();

#endif /* EXTERNAL_FLASH_H_ */
