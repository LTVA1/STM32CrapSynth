/*
 * pga2320.h
 *
 *  Created on: 16 окт. 2024 г.
 *      Author: georg
 */

#ifndef PGA2320_H_
#define PGA2320_H_

#include <stdint.h>

#define ATT_TX_BUF_SIZE 8

void att_write_filter_resonance(uint8_t resonance);
void att_write_vol(uint8_t ch, uint8_t vol);
void att_flush();
void att_init_all();

#endif /* PGA2320_H_ */
