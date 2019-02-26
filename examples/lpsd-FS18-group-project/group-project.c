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

/**
 * @brief Glossy Middleware Test Application
 *
 * This is a minimal example using the Glossy Middleware (GMW).
 * All nodes have a slot assigned (from the static_nodes array),
 * where they each send 2 bytes, a counter value and a magic number.
 */

/*---------------------------------------------------------------------------*/
#include "../lpsd-FS18-group-project/data-generator.h"
#include "contiki.h"
#include "gmw.h"
#include "node-id.h"
#include "gpio.h"
#include "debug-print.h"
#include "gpio.h"

/* data generator */

/*---------------------------------------------------------------------------*/
/* Makefile variables */
#ifndef DATARATE
#error No data rate specified. Example: use '-DDATARATE=10' to configure a rate of 10 packets per second.
#endif /* DATARATE */
uint16_t datarate = DATARATE;

#ifndef SINK_ADDRESS
#error No sink address specified. Example: use '-DSINK_ADDRESS=1' to configure the sink node ID.
#endif /* SINK_ADDRESS */
uint16_t sinkaddress = SINK_ADDRESS;

#ifndef RANDOM_SEED
#error No random seed specified. Example: use '-RANDOM_SEED=123' initialize the random number generator.
#endif /* RANDOM_SEED */
uint16_t randomseed = RANDOM_SEED;
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
static const uint16_t static_nodes[GMW_CONF_MAX_SLOTS] =
                                       { 22, 4, 8, 15, 3,
                                         31, 32, 33, 6, 16,
                                         1, 28, 18, 10 };
static const uint8_t number_of_static_nodes = GMW_CONF_MAX_SLOTS;

/**
 * @brief     Struct to store packet information
 */
typedef struct {
  uint16_t        src_id;
  uint8_t         seqn;
  uint16_t        payload;
} lpsd_packet_t;

static lpsd_packet_t  packet_buffer; /* packet buffer */
/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* application specific initialization */
  PIN_CFG_OUT(GLOSSY_START_PIN);
  PIN_CFG_OUT(GLOSSY_RX_PIN);
  PIN_CFG_OUT(GLOSSY_TX_PIN);

  /* initialize the data generator */
  data_generation_init();

  /* initialization of the GMW structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  DEBUG_PRINT_INFO("Sched: %u, config: %u, control: %u, payload: %u",
                   sizeof(gmw_schedule_t),
                   sizeof(gmw_config_t),
                   sizeof(gmw_control_t),
                   sizeof(lpsd_packet_t));
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted
     * permission by receiving a poll event by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

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
  /* nothing to do (we write directly our own packets at the end of the round)*/

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
  /* if something received, write */
  if(len == sizeof(lpsd_packet_t)) {
    memcpy(&packet_buffer, payload, len);
    DEBUG_PRINT_INFO("Pkt:%u,%u,%u", packet_buffer.src_id,packet_buffer.seqn, packet_buffer.payload);
  }

  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Check for packet in queue */
  while(is_data_in_queue()){
    lpsd_packet_t* packet = pop_data();
    packet_buffer = *packet;

    /* Write our own message to serial */
    DEBUG_PRINT_INFO("Pkt:%u,%u,%u", packet_buffer.src_id,packet_buffer.seqn, packet_buffer.payload);
  }

  /* the round has finished
   * update the control packet
   **/
  app_control_update(&control);

  /* and set it at the GMW */
  gmw_set_new_control(&control);
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
  if(is_initiator) {
    while(is_data_in_queue()){
      memcpy(out_payload, pop_data(), sizeof(lpsd_packet_t));
      *out_len = sizeof(lpsd_packet_t);
    }
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
  /* nothing to do */
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* do nothing */
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
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

    control->schedule.period = 1000/datarate;

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
