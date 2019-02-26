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
 *         Jonas Baechli   jonas.baechli@bluewin.ch
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         A very simple test protocol using Baloo (using Robust-Glossy)
 *
 *         All nodes have a slot assigned (from the static_nodes array),
 *         where they each send 2 bytes, a counter value and a magic number.
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "node-id.h"
#include "gpio.h"
#include "debug-print.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- APP VARIABLES -----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static uint8_t        counter = 0;
static uint16_t       rcv_cnt = 0;
static const uint16_t static_nodes[GMW_CONF_MAX_SLOTS] =
                                       { 1, 2, 3, 4, 6,
                                         7, 8,10,11,13,
                                        14,15,16,17,18,
                                        19,20,22,23,24,
                                        25,26,27,28,32,
                                        33 };
static const uint8_t number_of_static_nodes = GMW_CONF_MAX_SLOTS;
static const uint8_t payload_length = 2;
static const uint8_t control_length = GMW_CONF_MAX_SLOTS * 2 +
                                      GMW_SCHED_SECTION_HEADER_LEN +
                                      GMW_CONFIG_SECTION_HEADER_LEN;

/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/
/* function specific to Robust Glossy */
void glossy_set_channel_and_payload_len(uint8_t channel_index,
                                        uint8_t payload_length);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* initialization of the GMW structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* application specific initialization */
  
  PIN_SEL_I(5,5);

  /* set the payload length for Glossy (will be used whenever 0 is passed
   * as length argument to glossy_start()) */
  glossy_set_channel_and_payload_len(0, control_length);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted
     * permission by receiving a poll event by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    DEBUG_PRINT_INFO("dummy application task");

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
  /* return GMW_RUNNING in order to participate in this round */
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
  /* NOTE: in order for Robust Glossy to work properly, the payload length
   *       must be known in advance and passed to the middleware, even as a
   *       source node. */
  *out_len = payload_length;
  if(is_initiator) {
    /* if this is our slot, set our payload */
    out_payload[0] = 42;
    out_payload[1] = counter++;
  }

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
  /* if this was not our slot, handle the payload */
  if(!is_initiator) {
    if(len == 2) {
      /* print the two bytes */
      DEBUG_PRINT_VERBOSE("Host received %u %u",
                          payload[0],
                          payload[1]);
      rcv_cnt++;
    }
  }

  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* the round has finished
   * update the control packet
   **/
  app_control_update(&control);

  /* and set it at the GMW */
  gmw_set_new_control(&control);

  DEBUG_PRINT_INFO("received %u of %u packets", rcv_cnt,
                                                number_of_static_nodes);
  rcv_cnt = 0;
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t sync_event,
                                  gmw_pkt_event_t pkt_event)
{
  /* Default state update
   * - suspend execution if control is missed
   * - go to bootstrap after two consecutive missed controls
   * - run normally otherwise
   **/
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
  /* NOTE: in order for Robust Glossy to work properly, the payload length
   *       must be known in advance and passed to the middleware, even as a
   *       source node. */
  *out_len = payload_length;
  if(is_initiator) {
    /* if this is our slot, set our payload */
    out_payload[0] = counter--;
    out_payload[1] = 0xaa;
  }

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
  /* same as for host:
   * if this was not our slot, handle the payload
   */
  if(!is_initiator) {
    if(len == 2) {
      rcv_cnt++;
    }
  }

  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* make sure payload length is set correctly for the control packet of the
   * next round */
  glossy_set_channel_and_payload_len(0, control_length);

  /* do nothing in this case, however this callback could
   * be used to print debug output or to update statistics.
   */
  DEBUG_PRINT_INFO("received %u of %u packets", rcv_cnt,
                                                number_of_static_nodes);
  rcv_cnt = 0;
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  /* NOTE: The application can change the bootstrap channel here by calling
   *       glossy_set_channel_and_payload_len(index, 0), where index is between
   *       0 and GLOSSY_NUM_CHANNELS - 1. By default, Glossy will pick a
   *       channel on which a packet has been received previously or the first
   *       channel if no packet has been received yet. If payload length is
   *       set to zero, Glossy will not do channel hopping, which is beneficial
   *       for bootstrapping. */
  glossy_set_channel_and_payload_len(0, 0);       /* set payload length to 0 */

  /* returning 0 here instructs the GMW to stay in bootstrap */
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  /**
   * Initialization of the application-side control structure
   * - This must always be done of the host node (HOST_ID == node_id)
   * - This must be done for all nodes if
   *    the middleware uses a 'static' configuration, ie,
   *    GMW_CONF_USE_STATIC_SCHED   and/or
   *    GMW_CONF_USE_STATIC_CONFIG  are set to true.
   */

  /* initialize the slot assignment with the given array
   * of nodes
   */

  if(HOST_ID == node_id) {
    uint16_t i;
    for(i = 0; i < number_of_static_nodes; i++) {
      control->schedule.slot[i] = static_nodes[i];
    }

    /* we have number_of_static_nodes nodes in our schedule */
    control->schedule.n_slots = number_of_static_nodes;

    /* we start our local time at 0 */
    control->schedule.time = 0;

    /* the period is 2 seconds */
    control->schedule.period = 2;

    /* we want the control packet to include a
     * config section
     */
    GMW_CONTROL_SET_CONFIG(control);

    /* set the parameters from "project-conf.h" */
    control->config.gap_time            = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
    control->config.slot_time           = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA);
    control->config.n_retransmissions   = GMW_CONF_TX_CNT_DATA;
  }
}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* increment the time by the period */
  control->schedule.time += control->schedule.period;

  /* print debug output */
  DEBUG_PRINT_INFO("time: %lu, period: %u", control->schedule.time,
                                            control->schedule.period);
}
/*---------------------------------------------------------------------------*/
/**
 * GMW initialization function
 */
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

  /* loads __default__ schedule and config parameters */
  gmw_control_init(control);

  /* loads __application__ initial control parameters */
  app_control_init(control);

  /* notify the middleware that the host-app has a new control */
  gmw_set_new_control(control);
}
/*---------------------------------------------------------------------------*/
