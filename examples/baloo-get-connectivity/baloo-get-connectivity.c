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
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         An example protocol using Baloo made to estimate
 *         the connectivity of the network
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
static uint8_t        counter                     = 0;
static const uint16_t static_nodes[NUM_NODES]     = NODE_LIST;
static char           topology_info[2*NUM_NODES]  = {};
static uint32_t       period_us                   = TEST_PERIOD;
static uint16_t       period_s                    ;
/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
//static void app_control_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* application specific initialization */
  period_s = (period_us/1000000) + 1;

  /* initialization of the GMW structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted
     * permission by receiving a poll event by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    //DEBUG_PRINT_INFO("dummy application task");

    /* poll the debug-print task to print out all queued debug messages */
    debug_print_poll();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t sync_event,
                                  gmw_pkt_event_t pkt_event)
{
  /* Always run once we got the schedule once. Everything remains the same */

  return GMW_RUNNING;
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
  /* if this is our slot, set our payload */
  if(is_initiator) {
    out_payload[0] = counter--;
    out_payload[1] = 0xaa;
    *out_len = TEST_PAYLOAD_SIZE;
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
    topology_info[2*slot_index] = (char)(strobing_get_rx_cnt() + 48);

  } else {
    topology_info[2*slot_index] = '0';

  }
  topology_info[2*slot_index+1] = ',';

  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* do nothing in this case, however this callback could
   * be used to print debug output or to update statistics.
   */

  /* Terminate the string and print */
  topology_info[2*NUM_NODES-1] = '\0';
  DEBUG_PRINT_INFO("N_rx: %s", topology_info);

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
    for(i = 0; i < NUM_NODES; i++) {
      control->schedule.slot[i] = static_nodes[i];
    }

    /* we have number_of_static_nodes nodes in our schedule */
    control->schedule.n_slots = NUM_NODES;

    /* we start our local time at 0 */
    control->schedule.time = 0;

    /* the period is 2 seconds */
    control->schedule.period = period_s;

    /* we want the control packet to include a
     * config section
     */
    GMW_CONTROL_SET_CONFIG(control);
    control->config.primitive = GMW_PRIM_STROBING;
  }
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
  host_impl->on_control_slot_post   = &src_on_control_slot_post_callback;
  host_impl->on_slot_pre            = &src_on_slot_pre_callback;
  host_impl->on_slot_post           = &src_on_slot_post_callback;
  host_impl->on_round_finished      = &src_on_round_finished;

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
