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
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         Baloo protocol illustrating the utilization
 *         of the static control feature
 */


/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
#include "node-id.h"
#include "dc-stat.h"
#include "glossy.h"
#include "leds.h"
/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;
/*---------------------------------------------------------------------------*/
/*- APP VARIABLES -----------------------------------------------------------*/
static uint8_t counter;

#ifdef FLOCKLAB
/*static const uint16_t static_nodes[GMW_CONF_MAX_SLOTS] =
                                       { 1, 2, 3, 4, 6,
                                         7, 8,10,11,13,
                                        14,15,16,17,18,
                                        19,20,22,23,24,
                                        25,26,27,28,32,
                                        33 };*/
static const uint16_t static_nodes[GMW_CONF_MAX_SLOTS] =
                                       { 11, 10, 9, 7, 6, 5, 4, 2 };
#else
static const uint16_t static_nodes[] = { 1, 1, 2, 2 };
#endif /* FLOCKLAB */

static const uint8_t number_of_static_nodes = sizeof(static_nodes) / 2;
static uint32_t number_of_total_slots;
static uint32_t number_of_received_slots;
static uint8_t is_synced;
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
static void app_control_static_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* --- Application-specific initialization --- */

  is_synced = 0;

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    DEBUG_PRINT_INFO("%lu, %u, %u, %u, %lu, %lu, %u\n",
                     control.schedule.time,
                     DCSTAT_CPU_DC,
                     DCSTAT_RF_DC,
                     debug_print_get_max_stack_size(),
                     number_of_received_slots,
                     number_of_total_slots,
                     glossy_get_fsr());

    debug_print_poll();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
host_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                   gmw_sync_event_t event,
                                   gmw_pkt_event_t  pkt_event)
{
  /* If either the sched or the config is static */
  if(GMW_CONF_USE_STATIC_CONFIG || GMW_CONF_USE_STATIC_SCHED) {
    /* Update the static control
     * -> Happens here as the control update might depend on the info
     *    received in the control slot (for example, current round ID)
     * */
    app_control_static_update(in_out_control);

    /* Save a local copy */
    control = *in_out_control;
  }

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
  if(is_initiator) {
    out_payload[0] = 42;
    out_payload[1] = counter++;
    *out_len = 2;
  }
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
               uint8_t len, uint8_t* payload,
               uint8_t is_initiator, uint8_t is_contention_slot,
               gmw_pkt_event_t event)
{
  if(!is_initiator) {
    if(len == 2) {
      DEBUG_PRINT_INFO("Host received %u %u", payload[0],
                       payload[1]);
      number_of_received_slots++;
    } else {
      DEBUG_PRINT_INFO("Host did not receive, len=%u", len);
    }
    number_of_total_slots++;
  }
  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Update the control */
  app_control_update(&control);
  gmw_set_new_control(&control);

  leds_off(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t event,
                                  gmw_pkt_event_t  pkt_event)
{

  /* If either the sched or the config is static */
  if(GMW_CONF_USE_STATIC_CONFIG || GMW_CONF_USE_STATIC_SCHED) {
    /* The first time a control packet is received,
     * fill the static schedule to send to the middleware
     */
    if(!is_synced) {
      if(GMW_CONF_USE_STATIC_SCHED) {
        in_out_control->schedule = control.schedule;
      }
      if(GMW_CONF_USE_STATIC_CONFIG) {
        in_out_control->config = control.config;
      }
    }
    /* Update the static schedule
     * -> Happens here as the schedule update might depend on the info
     *    received in the control slot (for example, current round ID)
     **/
    app_control_static_update(in_out_control);
  }

  /* Save a local copy of the control*/
  control = *in_out_control;

  is_synced = 1;

  leds_on(LEDS_GREEN);
  leds_off(LEDS_RED);
  return GMW_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
src_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
                         uint8_t* out_len, uint8_t* out_payload,
                         uint8_t is_initiator, uint8_t is_contention_slot)
{
  if(is_initiator) {
    out_payload[0] = counter--;
    out_payload[1] = 0xaa;
    *out_len = 2;
  }
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
src_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
                          uint8_t len, uint8_t* payload,
                          uint8_t is_initiator, uint8_t is_contention_slot,
                          gmw_pkt_event_t event)
{
  if(!is_initiator) {
    if(len == 2) {
      DEBUG_PRINT_INFO("Source received %u %u", payload[0],
               payload[1]);
      number_of_received_slots++;
    } else {
      DEBUG_PRINT_INFO("Source did not receive, len=%u", len);
    }
    number_of_total_slots++;
  }
  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Nothing to do */
  leds_off(LEDS_GREEN);
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  if(!is_synced) {
    DCSTAT_RESET;
  }

  leds_off(LEDS_GREEN);
  leds_on(LEDS_RED);
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  if(GMW_CONF_USE_STATIC_CONFIG ||
     GMW_CONF_USE_STATIC_SCHED  ||
     HOST_ID == node_id){
    gmw_schedule_t* sched = &control->schedule;
    int i;
    for(i=0; i<number_of_static_nodes; i++) {
      sched->slot[i] = static_nodes[i];
    }
    sched->n_slots = number_of_static_nodes;
    sched->time    = 0;
    sched->period  = 2000;

    GMW_CONTROL_SET_CONFIG(control);

    control->config.gap_time = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
    control->config.slot_time = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA);
    control->config.n_retransmissions = GMW_CONF_TX_CNT_DATA;

    /* The user bytes are always sent by the host,
     * They are used (in this example) by all nodes to perform the update of the
     * static schedule and config
     */
    GMW_CONTROL_SET_USER_BYTES(control);
    control->user_bytes[0] = 0;
  }
}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* Dynamic update of the control
   * New values that will be sent by the host at the beginning of the next round
   */

  /* update the schedule if not static*/
  if(!GMW_CONF_USE_STATIC_SCHED) {
    control->schedule.time += control->schedule.period;
  }

  /* update the rest of the control */
  control->user_bytes[0] = !control->user_bytes[0];
  
  /* update the config if not static*/
  if(!GMW_CONF_USE_STATIC_CONFIG) {
    if(control->user_bytes[0]) {
      control->config.n_retransmissions = 4;
    } else {
      control->config.n_retransmissions = 2;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
app_control_static_update(gmw_control_t* control)
{
  /* Static update of the control
   * Call by all nodes in the control slot post callback to set the static
   * schedule and config information, if any.
   */

  /* update the schedule if static*/
  if(GMW_CONF_USE_STATIC_SCHED) {
    control->schedule.time += control->schedule.period;
  }

  /* update the config if static*/
  if(GMW_CONF_USE_STATIC_CONFIG) {
    if(control->user_bytes[0]) {
      control->config.n_retransmissions = 4;
    } else {
      control->config.n_retransmissions = 2;
    }
  }
}
/*---------------------------------------------------------------------------*/
/**
 * GMW initialization function
 */
void gmw_init(gmw_protocol_impl_t* host_impl,
              gmw_protocol_impl_t* src_impl,
              gmw_control_t* control){

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

  /* loads __default__ schedule and config parameters */
  gmw_control_init(control);

  /* loads __application__ initial control parameters */
  app_control_init(control);

  /* notify the middleware that the host-app has a new control */
  gmw_set_new_control(control);
}
/*---------------------------------------------------------------------------*/
