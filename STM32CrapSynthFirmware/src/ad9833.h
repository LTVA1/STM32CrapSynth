/*
 * ad9833.h
 *
 *  Created on: 16 окт. 2024 г.
 *      Author: Georg
 */

#ifndef AD9833_H_
#define AD9833_H_

#include <stdint.h>

#define AD9833_TX_BUF_SIZE 3
#define AD9833_NUM 4

#define FREQ0_REG_ADDRESS (0b01 << 14)

#define RESET_BIT (1 << 8)
#define B28 (1 << 12) /* two 14-bit writes, update of internal reg after 2nd write */

void ad9833_reset(uint8_t dds);
void ad9833_write_freq(uint8_t dds, uint32_t freq);
void ad9833_init(uint8_t dds);
void ad9833_init_all();

#endif /* AD9833_H_ */
