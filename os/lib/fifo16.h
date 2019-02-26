/*
 * Copyright (c) 2015, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Reto Da Forno
 */

/**
 * @addtogroup  lib
 * @{
 *
 * @defgroup    fifo First-in, first-out queue (16-bit implementation)
 * @{
 * 
 * @file
 *
 * @brief First-in, first-out queue based on a linear data array. All elements
 * have the same maximum size.
 * This lib provides address management only, no actual memory allocation.
 * Therefore, it is suitable for any type of memory with 16-bit addresses.
 */

#ifndef FIFO16_H_
#define FIFO16_H_

#include <string.h>

#define FIFO16_ERROR      0xffff

/**
 * @brief declare a FIFO
 * @param start start address of the data array
 * @param elem_size size of one data element
 * @param num number of data elements in this memory block
 * @note It is the users responsibility to allocate a memory block of the
 * size 'elem_size * num' starting at the address 'start_addr'. The FIFO
 * itself uses 14 bytes.
 */
#define FIFO16(name, elem_size, num) \
  static struct fifo16 name = { 0, elem_size, num - 1, 0, 0, 0 }

struct fifo16 {
  uint16_t start;     /* start address of the array */
  uint16_t elem_size; /* size of one data unit */
  uint16_t num_elem;  /* number of data units in this memory block */
  uint16_t count;     /* number of occupied queue spaces */
  uint16_t read;      /* the read index */
  uint16_t write;     /* the write index */
};

#define FIFO16_RESET(f)       ((f)->read = (f)->write = (f)->count = 0)
#define FIFO16_EMPTY(f)       ((f)->count == 0)
#define FIFO16_FULL(f)        ((f)->count == (f)->num_elem)
#define FIFO16_FREE_SPACE(f)  ((f)->num_elem - (f)->count)
#define FIFO16_READ_ADDR(f)   ((f)->start + ((f)->read * (f)->elem_size))
#define FIFO16_WRITE_ADDR(f)  ((f)->start + ((f)->write * (f)->elem_size))
/* increment the read index */
#define FIFO16_INCR_READ(f)   { \
  (f)->read++; \
  if((f)->read == (f)->num_elem) { \
    (f)->read = 0; \
  } \
}
/* increment the write index */
#define FIFO16_INCR_WRITE(f)  { \
  (f)->write++; \
  if((f)->write == (f)->num_elem) { \
    (f)->write = 0; \
  } \
}

/**
 * @brief initializes the FIFO queue (set start address and reset counters)
 */
static inline void
fifo16_init(struct fifo16 * const f, uint16_t start_addr)
{
  f->start = start_addr;
  FIFO16_RESET(f);
}

/**
 * @brief delivers the address of the next element to read and shifts the 
 * read pointer forward by 1 element
 * @return the address of the next element to read or FIFO16_ERROR if
 * the queue is empty
 */
static inline uint32_t
fifo16_get(struct fifo16 * const f) 
{
  if(FIFO16_EMPTY(f)) { return FIFO16_ERROR; }
  uint32_t next_read = FIFO16_READ_ADDR(f);
  FIFO16_INCR_READ(f);
  f->count--;
  return next_read;
}

/**
 * @brief shifts the write pointer to the next position
 * @return the address of the next free element or FIFO16_ERROR if the
 * queue is full
 */
static inline uint32_t
fifo16_put(struct fifo16 * const f) 
{
  if(FIFO16_FULL(f)) { return FIFO16_ERROR; }
  uint32_t next_write = FIFO16_WRITE_ADDR(f);
  FIFO16_INCR_WRITE(f);
  f->count++;
  return next_write;
}

#endif /* FIFO16_H_ */
