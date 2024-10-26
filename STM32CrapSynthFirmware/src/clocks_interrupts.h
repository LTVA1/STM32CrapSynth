/*
 * clocks_interrupts.h
 *
 *  Created on: 26 окт. 2024 г.
 *      Author: georg
 */

#ifndef CLOCKS_INTERRUPTS_H_
#define CLOCKS_INTERRUPTS_H_

void remap_interrupt_vectors_table_to_ccmram();
void enable_all_clocks();
void set_72MHz();

#endif /* CLOCKS_INTERRUPTS_H_ */
