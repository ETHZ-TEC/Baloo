/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Romain Jacob
 */

#include "contiki.h"
#include "random.h"
#include "memb.h"
#include "queue.h"
#include "rtimer-ext.h"
#include "node-id.h"

/**
 * @brief     Struct to store the generated packets
 */
typedef struct packet_info {
  struct packet_info *next;
  uint16_t        src_id;
  uint8_t         seqn;
  uint16_t        payload;
} lpsd_packet_queue_t;

/**
 * @brief     Generates and stores a new packet
 */
void generate_new_data(void);

/**
 * @brief     Initializes the data structures required
 *            for the data packet generation
 */
void data_generation_init(void);

/**
 * @brief     Looks at the next data packet
 *            (does not remove it from the queue)
 * @return    A pointer to the first packet in queue
 */
void* get_data(void);

/**
 * @brief     Removes the next data packet
 * @return    A pointer to the first packet in queue
 */
void* pop_data(void);

/**
 * @brief     Checks if a queue is empty
 * @return    True    if there is packets in the queue
 *            False   if the queue is empty
 */
uint8_t is_data_in_queue(void);
