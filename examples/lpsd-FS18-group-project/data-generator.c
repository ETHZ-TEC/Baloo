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

#include "../lpsd-FS18-group-project/data-generator.h"

#include "sys/log.h"
#define LOG_MODULE    "DesignProjectApp"
#define LOG_LEVEL LOG_LEVEL_MAIN
#include <stdio.h> /* For printf() */
/*---------------------------------------------------------------------------*/
extern uint16_t datarate;
extern uint16_t randomseed;
QUEUE(packet_queue);
MEMB(packet_memb, lpsd_packet_queue_t, PACKET_QUEUE_SIZE);
/*---------------------------------------------------------------------------*/
void data_generation_init(void)
{
  /* initialize the packet queue */
  memb_init(&packet_memb);
  queue_init(packet_queue);

  /* initialize the random generator */
  random_init(randomseed);

  /* initialize the timer for data generation */
  rtimer_ext_clock_t start_time = (10000 + (node_id * 111)%500)
                                   * RTIMER_EXT_SECOND_LF
                                   / 1000;
  rtimer_ext_clock_t period     = RTIMER_EXT_SECOND_LF / datarate;
  rtimer_ext_schedule(RTIMER_EXT_LF_1, start_time, period, (rtimer_ext_callback_t) &generate_new_data);
}
/*---------------------------------------------------------------------------*/
void generate_new_data(void)
{
  static uint8_t seqn = 0;

  /* Stop after 200 packets */
  if(seqn >= 200) {
    return;
  } else {
    /* Increment sequence number
     * -> This should happen even if the queue overflows */
    seqn++;
  }

  lpsd_packet_queue_t* pkt = memb_alloc(&packet_memb);
  if(pkt == 0) {
    LOG_INFO("Data queue overflow!\n");
    return;
  }
  pkt->src_id   = node_id;
  pkt->seqn     = seqn;
  pkt->payload  = random_rand();

  /* add packet to the queue */
  queue_enqueue(packet_queue, pkt);
  LOG_INFO("pkt->payload: %u\n", pkt->payload);
}
/*---------------------------------------------------------------------------*/
void* get_data()
{
  /* peek at the first packet */
  lpsd_packet_queue_t* pkt = queue_peek(packet_queue);
  return (&(pkt->src_id));
}
/*---------------------------------------------------------------------------*/
void* pop_data()
{
  /* dequeue the first packet */
  lpsd_packet_queue_t* pkt = queue_dequeue(packet_queue);
  /* free the memory block */
  memb_free(&packet_memb, pkt);
  /* return the corresponding packet pointer */
  return (&(pkt->src_id));
}
/*---------------------------------------------------------------------------*/
uint8_t is_data_in_queue(){
  return *packet_queue == NULL ? 0 : 1;
}
/*---------------------------------------------------------------------------*/
