/*
 * ring_buf.h
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include <stdio.h>
#include <stdint.h>

typedef struct
{
	uint8_t* buffer;
	uint16_t size;
	uint16_t data_size;
	uint16_t ptr_begin;
	uint16_t ptr_end;
} RingBuffer;

void buffer_init(RingBuffer* buf, uint8_t* buf_data, uint16_t size_buf);
uint8_t buffer_put_to_end(RingBuffer* buf, uint8_t data);
uint8_t buffer_get_from_front(RingBuffer* buf, uint8_t* data);
