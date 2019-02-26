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
 *         A Glossy test application
 *
 *         No drift compensation, no state machine. Everything is static.
 *         The host node sends a sync packet every 100ms.
 */

#include "contiki.h"
#include "rtimer-ext.h"
#include "node-id.h"
#include "glossy.h"
#include "gpio.h"
#include "debug-print.h"

/*---------------------------------------------------------------------------*/
#ifndef GLOSSY_UNKNOWN_INITIATOR
#define GLOSSY_UNKNOWN_INITIATOR      0
#endif /* GLOSSY_UNKNOWN_INITIATOR */
#ifndef GLOSSY_WITH_CAL
#define GLOSSY_WITH_CAL               1
#endif /* GLOSSY_WITH_CAL */
#ifndef GLOSSY_WITH_SYNC
#define GLOSSY_WITH_SYNC              0x10
#endif /* GLOSSY_WITH_SYNC */
#ifndef GLOSSY_UNKNOWN_PAYLOAD_LEN
#define GLOSSY_UNKNOWN_PAYLOAD_LEN    0
#endif /* GLOSSY_UNKNOWN_PAYLOAD_LEN */
/*---------------------------------------------------------------------------*/
#define WAIT_UNTIL(time) \
{\
  rtimer_ext_schedule(GLOSSY_RTIMER_ID, (time), 0, glossy_thread);\
  PT_YIELD(&glossy_pt);\
}
/*---------------------------------------------------------------------------*/
#if PLATFORM_DPP_CC430
  #define GLOSSY_SYNC_RECEIVED        glossy_is_t_ref_updated()
  #define GLOSSY_PACKET_RECEIVED      glossy_get_rx_cnt()
  #define GLOSSY_REFERENCE_TIME       glossy_get_t_ref_lf()
#else  /* PLATFORM_DPP_CC430 */
  #define GLOSSY_SYNC_RECEIVED        glossy_is_t_ref_updated()
  #define GLOSSY_PACKET_RECEIVED      glossy_get_rx_cnt()
  #define GLOSSY_REFERENCE_TIME       glossy_get_t_ref()
  #define glossy_get_snr()            0    /* undefined */
#endif /* PLATFORM_DPP_CC430 */
/*---------------------------------------------------------------------------*/
static struct pt glossy_pt; /* glossy protothread */
static uint8_t glossy_payload[RF_CONF_MAX_PKT_LEN];
/*---------------------------------------------------------------------------*/
PT_THREAD(glossy_thread(rtimer_ext_t *rt))
{
  static rtimer_ext_clock_t t_next_glossy_flood;
  static uint16_t bootstrap_cnt = 0;
  static uint16_t pkt_cnt       = 0;
  static uint16_t miss_cnt      = 0;
  static uint8_t  sync_state    = 0;

  /* compose the packet */
  memset(glossy_payload, GLOSSY_PAYLOAD_LEN, GLOSSY_PAYLOAD_LEN);

  PT_BEGIN(&glossy_pt);

  /* main loop of this application task */
  while (1) {

    /* HOST NODE */
    if(node_id == HOST_ID) {
      /* send a packet */
      glossy_start(node_id,
                   glossy_payload,
                   GLOSSY_PAYLOAD_LEN,
                   GLOSSY_N_TX,
                   GLOSSY_WITH_SYNC,
                   GLOSSY_WITH_CAL);
      WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
      glossy_stop();
      DEBUG_PRINT_INFO("packet sent, fsr=%u%%",
                       glossy_get_fsr() / 100);

      t_next_glossy_flood = rt->time - GLOSSY_T_SLOT + GLOSSY_PERIOD;

    /* SOURCE NODE */
    } else {
      if(!sync_state) {
        /* synchronize first! wait for a packet... */
        DEBUG_PRINT_INFO("BOOTSTRAP");
        bootstrap_cnt++;
        do {
          glossy_start(GLOSSY_UNKNOWN_INITIATOR,
                       glossy_payload,
                       GLOSSY_UNKNOWN_PAYLOAD_LEN,
                       GLOSSY_N_TX,
                       GLOSSY_WITH_SYNC,
                       GLOSSY_WITH_CAL);
          WAIT_UNTIL(rt->time + GLOSSY_T_SLOT);
          glossy_stop();
        } while (!GLOSSY_SYNC_RECEIVED);
        /* synchronized! */
        sync_state = 1;

      } else {
        /* already synchronized, receive a packet */
        glossy_start(GLOSSY_UNKNOWN_INITIATOR,
                     (uint8_t*) &glossy_payload,
                     GLOSSY_UNKNOWN_PAYLOAD_LEN,
                     GLOSSY_N_TX,
                     GLOSSY_WITH_SYNC,
                     GLOSSY_WITH_CAL);
        WAIT_UNTIL(rt->time + GLOSSY_T_SLOT + GLOSSY_T_GUARD);
        glossy_stop();

        if(GLOSSY_PACKET_RECEIVED) {
          pkt_cnt++;
        } else {
          miss_cnt++;
        }
        /* has the reference time been updated? */
        if(!GLOSSY_SYNC_RECEIVED) {
          sync_state = 0;  /* sync missed */
          continue;
        }
        /* print out some stats */
        DEBUG_PRINT_INFO("rcv=%u miss=%u boot=%u per=%u%% fsr=%u%% snr=%ddBm "
                         "m_hop=%u", pkt_cnt,
                                     miss_cnt,
                                     bootstrap_cnt,
                                     glossy_get_per() / 100,
                                     glossy_get_fsr() / 100,
                                     glossy_get_snr(),
                                     glossy_get_relay_cnt());
      }
      t_next_glossy_flood = GLOSSY_REFERENCE_TIME -
                            GLOSSY_REF_OFS +
                            GLOSSY_PERIOD -
                            GLOSSY_T_GUARD;
    }
    /* wait for the next glossy flood and let the debug print process run */
    debug_print_poll();
    WAIT_UNTIL(t_next_glossy_flood);
  }

  PT_END(&glossy_pt);
}
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data)
{
  PROCESS_BEGIN();

  rtimer_ext_init();
  debug_print_init();

  PIN_CFG_OUT(GLOSSY_START_PIN);
  PIN_CFG_OUT(GLOSSY_TX_PIN);
  PIN_CFG_OUT(GLOSSY_RX_PIN);

  if(HOST_ID == node_id) {
    /* set the content of the payload */
    uint8_t i;
    for(i = 0; i < GLOSSY_PAYLOAD_LEN; i++) {
      glossy_payload[i] = i;
    }
  }
  /* start the glossy thread in 1s */
  rtimer_ext_schedule(GLOSSY_RTIMER_ID,
                      rtimer_ext_now_lf() + RTIMER_EXT_SECOND_LF, 0,
                      glossy_thread);

  PROCESS_END();

  return 0;
}
/*---------------------------------------------------------------------------*/
