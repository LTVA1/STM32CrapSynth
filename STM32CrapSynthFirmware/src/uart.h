/*
 * uart.h
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

#define TX_BUF_SIZE 64
#define RX_BUF_SIZE (2048 + 32)

void uart_init();
void uart_send_via_dma(uint16_t size);
void uart_send_comms_establish_packet();

#endif /* UART_H_ */
