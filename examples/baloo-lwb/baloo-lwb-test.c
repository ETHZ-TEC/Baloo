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
 * \author
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 */

/**
 * \file
 *         Re-implementation of the LWB protocol using Baloo
 */

#include "contiki.h"
#include "lwb/gmw-lwb.h"
#include "debug-print.h"
#include "dc-stat.h"
#include "gpio.h"
#include "node-id.h"

/*---------------------------------------------------------------------------*/
#ifdef APP_TASK_ACT_PIN
#define TASK_ACTIVE             PIN_SET(APP_TASK_ACT_PIN)
#define TASK_SUSPENDED          PIN_CLR(APP_TASK_ACT_PIN)
#else
#define TASK_ACTIVE
#define TASK_SUSPENDED
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
static const char* lwb_sync_state_to_string[NUM_OF_LWB_SYNC_STATES] =
{ "BOOTSTRAP", "QSYNC", "SYNC", "SYNC2", "MISSED", "USYNC", "USYNC2" };
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data)
{
  static uint8_t  pkt_buffer[LWB_MAX_DATA_PKT_LEN];
  static uint16_t stream_id;

  PROCESS_BEGIN();

  /* start the LWB (GMW thread) */
  lwb_start(0, &app_process);

  /* main loop of this application task */
  while (1) {
    /* this task may only run when polled by the GMW task */
    TASK_SUSPENDED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    TASK_ACTIVE; /* application task runs now */

    if(HOST_ID == node_id) {
      /* we are the host: read the received data packets */
      uint16_t cnt = 0;
      uint16_t pkt_len;
      do {
        cnt++;
        pkt_len = lwb_rcv_pkt(pkt_buffer, 0, 0);
      } while(pkt_len);
      DEBUG_PRINT_INFO("rcvd packets: %u, FSR: %u, PER: %u", //CPU DC: %u, RF DC: %u",
                       cnt - 1,
                       glossy_get_fsr(),
                       glossy_get_per() /*,
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC */);
      DEBUG_PRINT_INFO("Radio DC: %u.%02u", DCSTAT_RF_DC/100, DCSTAT_RF_DC%100);

    } else {
      /* we are a source node */
      if(lwb_stream_get_state(stream_id) < LWB_STREAM_STATE_PENDING) {
        /* request a stream with an IPI (inter packet interval) of 10s */
        stream_id = lwb_stream_request(SOURCE_IPI);
        if(stream_id == LWB_INVALID_STREAM_ID) {
          DEBUG_PRINT_ERROR("stream request failed");
        }
      } else if(lwb_stream_get_state(stream_id) == LWB_STREAM_STATE_ACTIVE) {
        /* make sure the output queue never empties, generate a dummy packet
         * whenever there is space in the queue */
        memset(pkt_buffer, 0xaa, LWB_MAX_PAYLOAD_LEN);
        uint16_t cnt = 0;
        while(lwb_send_pkt(LWB_RECIPIENT_SINK, stream_id, pkt_buffer,
                           LWB_MAX_PAYLOAD_LEN)) cnt++;
        if(cnt) {
          DEBUG_PRINT_INFO("%u LWB packets created", cnt);
        }
      }
      DEBUG_PRINT_INFO("%s time: %lu, FSR: %u, PER: %u", //, CPU DC: %u, RF DC: %u",
                       lwb_sync_state_to_string[lwb_get_sync_state()],
                       lwb_get_time(0),
                       glossy_get_fsr(),
                       glossy_get_per() /*,
                       DCSTAT_CPU_DC,
                       DCSTAT_RF_DC*/);
      DEBUG_PRINT_INFO("Radio DC: %u.%02u", DCSTAT_RF_DC/100, DCSTAT_RF_DC%100);
    }
    debug_print_poll();   /* let the debug print task run */
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
