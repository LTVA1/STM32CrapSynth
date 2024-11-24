/*
 * ring_buf.c
 *
 *  Created on: 5 окт. 2024 г.
 *      Author: Georg
 */

#include "ring_buf.h"

RingBuffer rx_ring_buf;

void buffer_init(RingBuffer* buf, uint8_t* buf_data, uint16_t size_buf)
{
	buf->buffer = buf_data;
	buf->size = size_buf;
	buf->ptr_begin = 0;
	buf->ptr_end = 0;
	buf->data_size = 0;
}

__attribute__((section (".ccmram")))
uint8_t buffer_put_to_end(RingBuffer* buf, uint8_t data)
{
	if(buf->data_size >= buf->size) return 1; //buffer overflow

	buf->buffer[buf->ptr_begin] = data;
	++buf->ptr_begin;
	++buf->data_size;

	if(buf->data_size >= buf->size)
	{
		buf->ptr_begin = 0;
	}

	if(buf->ptr_begin >= buf->size)
	{
		buf->ptr_begin = 0;
	}

	return 0; //success
}

__attribute__((section (".ccmram")))
uint8_t buffer_get_from_front(RingBuffer* buf, uint8_t* data)
{
	if(buf->data_size <= 0)
	{
		return 1; //buffer empty
	}

	*data = buf->buffer[buf->ptr_end];
	++buf->ptr_end;

	if(buf->data_size > 0)
	{
		--buf->data_size;
	}

	if(buf->ptr_end >= buf->size)
	{
		buf->ptr_end = 0; //variable overflows, back to 0
	}

	return 0;
}

