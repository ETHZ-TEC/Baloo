/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
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
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         A simple Baloo protocol using the Chaos primitive
 *
 *         In each round, nodes pseudo-randomly generate a boolean value,
 *         which is passed as payload to Chaos.
 *         The custom `chaos_set_payload_cb()` use this boolean value to act
 *         on the actual packet payload: if the boolean is true, the bit
 *         corresponding to the node's position in CHAOS_CONF_NODE_ID_MAPPING
 *         is set to 1 (or 0 otherwise).
 *
 *         This mimics a quick aggregation of ACKs from multiple nodes.
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "gpio.h"
#include "node-id.h"
#include "debug-print.h"
#include "leds.h"
/*---------------------------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;
/*---------------------------------------------------------------------------*/
static uint16_t       counter;
static uint8_t        received_payload[NUM_NODES];
static const uint16_t static_nodes[NUM_NODES] = NODE_LIST;
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/
/* pin for application task activity indication defined? */
#ifdef APP_TASK_ACT_PIN
  #define APP_TASK_ACTIVE       PIN_SET(APP_TASK_ACT_PIN)
  #define APP_TASK_INACTIVE     PIN_CLR(APP_TASK_ACT_PIN)
#else /* APP_TASK_ACT_PIN */
  #define APP_TASK_ACTIVE
  #define APP_TASK_INACTIVE
#endif /* APP_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
#if CHAOS_CONF_SET_CUSTOM_PAYLOAD
/* Implement custom Chaos data processing function:
 * If the payload from the middleware is non-zero, the bit corresponding to the 
 * node_index is set to one.
 * This can be used to quickly aggregate ACKs from multiple nodes, for example.
 * NOTE:  Works fine up to 8 nodes.
 *        Require adaptations to support more nodes.
 */
__attribute__((always_inline))
inline void chaos_set_payload_cb(uint8_t* chaos_payload_field,
                                 uint16_t node_index,
                                 uint8_t* payload)
{
  if(*payload) {
    *chaos_payload_field |= (1 << (uint8_t)node_index);
  }  
}
#endif /* CHAOS_CONF_SET_CUSTOM_PAYLOAD */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{
  PROCESS_BEGIN();

  /* initialization of the GMW structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted
     * permission by receiving a poll event by the GMW task */
    APP_TASK_INACTIVE;    /* application task suspended */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    APP_TASK_ACTIVE;      /* application task runs now */

    DEBUG_PRINT_INFO("payload: %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u %u",
                     received_payload[0], received_payload[1], received_payload[2], received_payload[3],
                     received_payload[4], received_payload[5], received_payload[6], received_payload[7],
                     received_payload[8], received_payload[9], received_payload[10], received_payload[11],
                     received_payload[12], received_payload[13], received_payload[14], received_payload[15]
    );
    DEBUG_PRINT_INFO("round finished");

    //counter += node_id;
    counter += 0;

    /* poll the debug-print task to print out all queued debug messages */
    debug_print_poll();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
host_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                   gmw_sync_event_t sync_event,
                                   gmw_pkt_event_t pkt_event)
{
  leds_on(LEDS_GREEN);
  return GMW_RUNNING;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
host_on_slot_pre_callback(uint8_t slot_index,
                          uint16_t slot_assignee,
                          uint8_t* out_len,
                          uint8_t* out_payload,
                          uint8_t is_initiator,
                          uint8_t is_contention_slot)
{
  /* regardless of whether we are initiator: fill in the payload */
  //*out_payload = (counter/7)%2;
  *out_payload = (uint8_t)(node_id + counter);
  *out_len = 1;
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post_callback(uint8_t slot_index,
                           uint16_t slot_assignee,
                           uint8_t len,
                           uint8_t* payload,
                           uint8_t is_initiator,
                           uint8_t is_contention_slot,
                           gmw_pkt_event_t event)
{
  //Store the received payload
  memcpy(&received_payload, payload, NUM_NODES);
  //DEBUG_PRINT_INFO("payload: %u", payload[0]);
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  //DEBUG_PRINT_INFO("%u %u %u", received_payload[0],received_payload[1],received_payload[2]);
  app_control_update(&control);
  gmw_set_new_control(&control);
  leds_off(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t sync_event,
                                  gmw_pkt_event_t pkt_event)
{
  leds_on(LEDS_GREEN);
  leds_off(LEDS_RED);
  return GMW_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
src_on_slot_pre_callback(uint8_t slot_index,
                         uint16_t slot_assignee,
                         uint8_t* out_len,
                         uint8_t* out_payload,
                         uint8_t is_initiator,
                         uint8_t is_contention_slot)
{
  /* regardless of whether we are initiator: fill in the payload */
  //*out_payload = (counter/7)%2;
  *out_payload = (uint8_t)(node_id + counter);
  *out_len = 1;
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
src_on_slot_post_callback(uint8_t slot_index,
                          uint16_t slot_assignee,
                          uint8_t len,
                          uint8_t* payload,
                          uint8_t is_initiator,
                          uint8_t is_contention_slot,
                          gmw_pkt_event_t event)
{
  //Store the received payload
  memcpy(&received_payload, payload, NUM_NODES);
  //DEBUG_PRINT_INFO("payload: %u", payload[0]);
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  //DEBUG_PRINT_INFO("%u %u %u", received_payload[1],received_payload[5],received_payload[12]);
  leds_off(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  leds_off(LEDS_GREEN);
  leds_on(LEDS_RED);
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  if(HOST_ID == node_id) {
    /*
    uint16_t i;
    for(i = 0; i < NUM_NODES; i++) {
      control->schedule.slot[i] = static_nodes[i];
    }
    */
    control->schedule.n_slots         = 1;    /* 1 slot only */
    control->schedule.slot[0]         = HOST_ID;
    control->schedule.time            = 0;
    control->schedule.period          = 2;
    control->config.primitive         = GMW_PRIM_CHAOS;
    GMW_CONTROL_SET_CONFIG(control);
  }
}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  control->schedule.time += control->schedule.period;
  DEBUG_PRINT_INFO("time: %lu, period: %u", control->schedule.time,
                                            control->schedule.period);
}
/*---------------------------------------------------------------------------*/
void
gmw_init(gmw_protocol_impl_t* host_impl,
         gmw_protocol_impl_t* src_impl,
         gmw_control_t* control)
{
  /* load the host node implementation */
  host_impl->on_control_slot_post   = &host_on_control_slot_post_callback;
  host_impl->on_slot_pre            = &host_on_slot_pre_callback;
  host_impl->on_slot_post           = &host_on_slot_post_callback;
  host_impl->on_round_finished      = &host_on_round_finished;

  /* load the source node implementation */
  src_impl->on_control_slot_post    = &src_on_control_slot_post_callback;
  src_impl->on_slot_pre             = &src_on_slot_pre_callback;
  src_impl->on_slot_post            = &src_on_slot_post_callback;
  src_impl->on_round_finished       = &src_on_round_finished;
  src_impl->on_bootstrap_timeout    = &src_on_bootstrap_timeout;

  gmw_control_init(control);
  app_control_init(control);
  gmw_set_new_control(control);
}
/*---------------------------------------------------------------------------*/
