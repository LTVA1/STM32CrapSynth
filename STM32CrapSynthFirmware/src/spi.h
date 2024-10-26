/*
 * spi.h
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

void spi_init();
void spi1_send_via_dma(uint8_t* sendbuf, uint16_t size);
void spi2_send_via_dma(uint16_t* sendbuf, uint16_t size);
void spi2_send_16bits(uint16_t data);
void spi1_receive_via_dma(uint8_t* sendbuf, uint8_t* receivebuf, uint16_t size);

#endif /* SPI_H_ */
