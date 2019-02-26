/*
 * Copyright (c) 2018, ETH Zurich.
 * Copyright (c) 2013, Olaf Landsiedel.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * \authors
 *          Federico Ferrari
 *          Olaf Landsiedel
 *          Reto Da Forno
 */

/**
 * \file
 *         A Chaos test application
 */

#include <string.h>
#include "contiki.h"
#include "node-id.h"
#include "chaos.h"
#include "gpio.h"
#include "rtimer-ext.h"

/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "chaos-test"
#define LOG_LEVEL  LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
static struct pt chaos_scheduler_pt;
static uint8_t  chaos_payload[CHAOS_CONF_PAYLOAD_LEN];
static uint16_t packets_received,
                packets_missed;
static uint16_t counter;
/*---------------------------------------------------------------------------*/
PROCESS(chaos_test, "Chaos test");
/*---------------------------------------------------------------------------*/
#define CHAOS_IS_SYNCED           chaos_is_t_ref_l_updated()
#define CHAOS_REFERENCE_TIME      chaos_get_t_ref_lf_ext()
/*---------------------------------------------------------------------------*/
PT_THREAD(chaos_scheduler(rtimer_ext_t *rt))
{
  PT_BEGIN(&chaos_scheduler_pt);

  if(node_id == INITIATOR_NODE_ID) {  // Chaos initiator.
    LOG_INFO("INITIATOR\n");

    while(1) {
      // --- chaos phase begin ---
      memcpy(chaos_payload, &counter, 2);   // dummy data
      chaos_start(chaos_payload, 2, CHAOS_INITIATOR, 
                  CHAOS_CONF_N_TX, 1);
      // Schedule end of Chaos phase based on CHAOS_DURATION.
      rtimer_ext_schedule(CHAOS_RTIMER_ID,
                          rt->time + CHAOS_DURATION,
                          0, chaos_scheduler);
      PT_YIELD(&chaos_scheduler_pt);
      chaos_stop();
      // --- chaos phase end ---
      // Poll the process that prints statistics.
      process_poll(&chaos_test);
      // Schedule begin of next Chaos phase based on CHAOS_PERIOD.
      rtimer_ext_schedule(CHAOS_RTIMER_ID,
                          rt->time + CHAOS_PERIOD - CHAOS_DURATION,
                          0, chaos_scheduler);
      LOG_INFO("chaos round finished\n");
      PT_YIELD(&chaos_scheduler_pt);  // suspend this protothread
    }

  } else {  // Chaos receiver.
    LOG_INFO("RECEIVER\n");

    while(1) {
      // --- chaos phase begin ---
      memcpy(chaos_payload, &counter, 2);   // dummy data
      do {
        chaos_start(chaos_payload, 2, CHAOS_RECEIVER, CHAOS_CONF_N_TX, 1);
        // Chaos has already successfully bootstrapped
        rtimer_ext_schedule(CHAOS_RTIMER_ID,
                            rt->time + CHAOS_GUARD_TIME + CHAOS_DURATION,
                            0, chaos_scheduler);
        PT_YIELD(&chaos_scheduler_pt);
        chaos_stop();
      } while (!CHAOS_IS_SYNCED);
      // --- chaos phase end ---
      // Poll the process that prints statistics.
      process_poll(&chaos_test);
      // Schedule the next round.
      rtimer_ext_schedule(CHAOS_RTIMER_ID,
                          CHAOS_REFERENCE_TIME + CHAOS_PERIOD - 
                          CHAOS_GUARD_TIME,
                          0, chaos_scheduler);
      LOG_INFO("chaos round finished\n");
      PT_YIELD(&chaos_scheduler_pt);  /* suspend this protothread */
    }
  }

  PT_END(&chaos_scheduler_pt);
}
/*---------------------------------------------------------------------------*/
AUTOSTART_PROCESSES(&chaos_test);
PROCESS_THREAD(chaos_test, ev, data)
{
  PROCESS_BEGIN();

  PIN_CLR(ADC0);
  PIN_CFG_OUT(ADC0);
  PIN_CLR(ADC1);
  PIN_CFG_OUT(ADC1);
  PIN_CLR(ADC2);
  PIN_CFG_OUT(ADC2);

  LOG_INFO("chaos test! tx on complete: %d, "
           "payload: %d, period %lu, duration %lu, node count: %u\n",
           CHAOS_CONF_N_TX_COMPLETE, CHAOS_CONF_PAYLOAD_LEN, CHAOS_PERIOD,
           CHAOS_DURATION, CHAOS_CONF_NUM_NODES);

  rtimer_ext_init();

  // Start Chaos in one second.
  rtimer_ext_schedule(CHAOS_RTIMER_ID,
                      rtimer_ext_now_lf() + RTIMER_EXT_SECOND,
                      0, chaos_scheduler);

  // print stats loop
  while(1) {
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    // Print statistics only if Chaos is not still bootstrapping.
    if(chaos_get_rx_cnt()) {  // Packet received at least once.
      // Increment number of successfully received packets.
      packets_received++;
      // Print information about last packet and related latency.
      uint16_t* payload = (uint16_t*)chaos_payload;
      LOG_INFO("received packets: %u, counter: %u %u\n",
               packets_received, payload[0], payload[1]);
    } else {
      // Increment number of missed packets.
      packets_missed++;
      // Print failed reception.
      LOG_INFO("packet missed\n");
    }
#if ENERGEST_CONF_ON
    // Compute average radio-on time, in microseconds.
    unsigned long avg_rf_on = CHAOS_PERIOD * 1e6 / RTIMER_EXT_SECOND *
                              (energest_type_time(ENERGEST_TYPE_LISTEN) +
                                energest_type_time(ENERGEST_TYPE_TRANSMIT)) /
                              (energest_type_time(ENERGEST_TYPE_CPU) +
                                energest_type_time(ENERGEST_TYPE_LPM));
    // Print information about average radio-on time.
    LOG_INFO("average radio-on time %lu.%03lu ms\n",
        avg_rf_on / 1000, avg_rf_on % 1000);
#endif /* ENERGEST_CONF_ON */

    counter += node_id;
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
