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
 *         Minimal Baloo protocol
 *         to investigate the latency and energy models of TTW
 */


/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
#include "node-id.h"
#include "dc-stat.h"
#include "glossy.h"
#include "random.h"
#include "bolt.h"
#include "leds.h"
/*---------------------------------------------------------------------------*/

#define RTIMER_EXT_HF_TO_US(t)    ((t * 1000000) / RTIMER_EXT_SECOND_HF)


/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;

/*---------------------------------------------------------------------------*/
/*- XP PARAMETERS -----------------------------------------------------------*/

#ifndef TTW_MAX_SLOTS_PER_ROUND
#error Number of slots per round not specified
#endif /* TTW_MAX_SLOTS_PER_ROUND */
uint16_t slots_per_round = TTW_MAX_SLOTS_PER_ROUND;

#ifndef TTW_MAX_PAYLOAD_LEN
#error The payload length is not specified
#endif /* TTW_MAX_PAYLOAD_LEN */
uint16_t payload_length = TTW_MAX_PAYLOAD_LEN;

#ifndef RANDOM_SEED
#error No random seed specified.
#endif /* RANDOM_SEED */
uint16_t randomseed = RANDOM_SEED;

#ifdef XP_ACT_PIN
  #define XP_GPIO_ON           PIN_SET(XP_ACT_PIN)
  #define XP_GPIO_OFF          PIN_CLR(XP_ACT_PIN)
#else /* XP_ACT_PIN */
  #define XP_GPIO_ON
  #define XP_GPIO_OFF
#endif /* XP_ACT_PIN */

/*---------------------------------------------------------------------------*/
/*- APP VARIABLES -----------------------------------------------------------*/
static uint8_t trigger_counter  = 5;
static uint8_t round_counter    = 0;
static const uint16_t static_nodes_all[NUM_NODES] = NODE_LIST;
static uint16_t static_nodes_used[GMW_CONF_MAX_SLOTS];
static gmw_statistics_t* stats_pt = NULL;
static uint8_t is_synced;
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
static void app_control_static_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
PROCESS(pre_process, "Pre-round Task");
AUTOSTART_PROCESSES(&app_process, &pre_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();


  /* --- Application-specific initialization --- */

  is_synced = (node_id == HOST_ID);

  /* init BOLT */
  bolt_init();

  /* fill the static list randomly */
  random_init(randomseed);
  int i;
  for(i=0; i<slots_per_round; i++) {
    static_nodes_used[i] = static_nodes_all[ random_rand() % NUM_NODES ];
  }

  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(&pre_process, &app_process, &host_impl, &src_impl);

  /* log the node list - 0 means "unused" */
  DEBUG_PRINT_INFO("L: %u %u %u %u %u "
      "%u %u %u %u %u "
      "%u %u %u %u %u "
      "%u %u %u %u %u "
      "%u %u %u %u %u "
      "%u %u %u %u %u ",
      static_nodes_used[0],
      static_nodes_used[1],
      static_nodes_used[2],
      static_nodes_used[3],
      static_nodes_used[4],
      static_nodes_used[5],
      static_nodes_used[6],
      static_nodes_used[7],
      static_nodes_used[8],
      static_nodes_used[9],
      static_nodes_used[10],
      static_nodes_used[11],
      static_nodes_used[12],
      static_nodes_used[13],
      static_nodes_used[14],
      static_nodes_used[15],
      static_nodes_used[16],
      static_nodes_used[17],
      static_nodes_used[18],
      static_nodes_used[19],
      static_nodes_used[20],
      static_nodes_used[21],
      static_nodes_used[22],
      static_nodes_used[23],
      static_nodes_used[24],
      static_nodes_used[25],
      static_nodes_used[26],
      static_nodes_used[27],
      static_nodes_used[28],
      static_nodes_used[29]);

  debug_print_poll();

  /* get pointer to GMW stats */
  stats_pt = gmw_get_stats();

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* log only the relevant rounds
     * -> once the count down reaches 1 (at the end of the last full round) */
    if(control.user_bytes[0] < 2) {
      DEBUG_PRINT_INFO("E: %lu",
                       (uint32_t)GMW_TICKS_TO_US(DCSTAT_RF_SUM));
      DEBUG_PRINT_INFO("T: %lu",
                       (uint32_t)(GMW_TICKS_TO_US(stats_pt->t_round_last)));
    }
    DCSTAT_RESET;

    debug_print_poll();
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(pre_process, ev, data)
{
  PROCESS_BEGIN();

  /* --- Pre-process initialization --- */

  // a priori, nothing to do here

  /* main loop of this pre-processing task */
  while(1) {

    XP_GPIO_OFF; // Pre-process yields
    /* the task should not do anything until it is explicitly granted
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    XP_GPIO_ON; // Pre-process starts
    DEBUG_PRINT_INFO("Round starts");

    /* in "real" TTW, this is where BOLT is flushed
     * and the message buffers filled */

    if( node_id == HOST_ID ){
      /* Update the control */
        app_control_update(&control);
        gmw_set_new_control(&control);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t event,
                                  gmw_pkt_event_t  pkt_event)
{
  XP_GPIO_ON;

  /* The first time a control packet is received,
   * fill the static schedule to send to the middleware
   */
  if(!is_synced && (event == GMW_EVT_CONTROL_RCVD)) {
    in_out_control->schedule = control.schedule;
    in_out_control->config = control.config;
    is_synced = 1;
  }
  /* Update the static schedule
   * -> Happens here as the schedule update might depend on the info
   *    received in the control slot (for example, current round ID)
   **/
  app_control_static_update(in_out_control);

  /* Save a local copy of the control*/
  control = *in_out_control;



  if((node_id != HOST_ID)  && (event == GMW_EVT_CONTROL_RCVD)) {
    //DEBUG_PRINT_INFO("C: %i H %u", HOST_ID, stats_pt->relay_cnt);
    // -> Don't know why, but stats does not return the correct values...
    DEBUG_PRINT_INFO("C: %i H %u", HOST_ID, glossy_get_relay_cnt());
  }

  /* Debugging statements
   *
  DEBUG_PRINT_INFO("t_ref: %llu HF ticks",
                   (uint64_t)glossy_get_t_ref());
  DEBUG_PRINT_INFO("t_ref: %llu us",
                   RTIMER_EXT_HF_TO_US((uint64_t)glossy_get_t_ref()));

  DEBUG_PRINT_INFO("relay_cnt_t_ref: %u",
                   glossy_get_relay_cnt());

  DEBUG_PRINT_INFO("T_slot_sum: %llu HF ticks",
                   (uint64_t)glossy_get_T_slot_sum());
  DEBUG_PRINT_INFO("T_slot_sum: %llu us",
                   RTIMER_EXT_HF_TO_US((uint64_t)glossy_get_T_slot_sum()));

  DEBUG_PRINT_INFO("n_T_slot: %u",
                   glossy_get_n_T_slot());

  DEBUG_PRINT_INFO("T_slot_estimated: %lu HF ticks",
                   glossy_get_T_slot_estimated());

  DEBUG_PRINT_INFO("RX-TX errors: %u ",
                   glossy_get_n_errors());
   */

  leds_on(LEDS_GREEN);
  leds_off(LEDS_RED);

  XP_GPIO_OFF;

  return GMW_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
                         uint8_t* out_len, uint8_t* out_payload,
                         uint8_t is_initiator, uint8_t is_contention_slot)
{
  XP_GPIO_ON;

  if(is_initiator) {
    /* in "real" TTW, memcpy from message buffer */
    memset(out_payload, (uint8_t) node_id, payload_length);
    *out_len = payload_length;
  }

  XP_GPIO_OFF;

  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
on_slot_post_callback(uint8_t   slot_index,
                      uint16_t  slot_assignee,
                      uint8_t   len,
                      uint8_t*  payload,
                      uint8_t   is_initiator,
                      uint8_t   is_contention_slot,
                      gmw_pkt_event_t event)
{
  XP_GPIO_ON;

  /* overwrite slot_index
   * -> for more legible logs in the single-slot rounds */
  if(round_counter != 0) {
    slot_index = round_counter-1;
  }

  if( !is_initiator ) {
    if( event == GMW_EVT_PKT_OK ) {
      XP_GPIO_OFF;
      if ( !bolt_write( payload, 1 ) ){
        /* Don't print error, this fills up the debug buffer... */
        // DEBUG_PRINT_ERROR("ERROR: bolt_write failed");
      }
      //DEBUG_PRINT_INFO("msg: %u", payload[0]);
      XP_GPIO_ON;
      /* delay to account for bolt write */
      //__delay_cycles(MCLK_SPEED/1000000 * 125);   /* wait 125 us */
      DEBUG_PRINT_INFO("S: %i H %u", static_nodes_used[slot_index], glossy_get_relay_cnt());
    } else {
      DEBUG_PRINT_INFO("S: %i F %u", static_nodes_used[slot_index], event);
    }
  } else {
    DEBUG_PRINT_INFO("S: %i I (%u)", static_nodes_used[slot_index], payload[0]);
  }

  XP_GPIO_OFF;

  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  XP_GPIO_ON;

  /* Nothing to do */
  leds_off(LEDS_GREEN);

  XP_GPIO_OFF;
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  XP_GPIO_ON;

  /* reset energy stats whenever we go back to bootstrap */
  DCSTAT_RESET;

  leds_off(LEDS_GREEN);
  leds_on(LEDS_RED);

  XP_GPIO_OFF;

  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
    gmw_schedule_t* sched = &control->schedule;
    int i;
    for(i=0; i<slots_per_round; i++) {
      sched->slot[i] = (uint8_t) static_nodes_used[i];
    }
    sched->n_slots = slots_per_round;
    sched->time    = 0;
    sched->period  = 1000;

    GMW_CONTROL_SET_CONFIG(control);
    GMW_CONTROL_SET_USER_BYTES(control);

    if(node_id == HOST_ID) {
      /* The user bytes are always sent by the host,
       * They are used (in this example) by all nodes to perform the update of the
       * static schedule and config
       */
      control->user_bytes[0] = trigger_counter;
      control->user_bytes[1] = 255; // dummy byte
    }
}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* Dynamic update of the control
   * New values that will be sent by the host at the beginning of the next round
   */

  /* update the rest of the control
   * -> while count down hasn't reach zero, decrease it */
  if(control->user_bytes[0]) {
    control->user_bytes[0] -= 1;
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

  /* once the count down reached zero,
   * which to the single-slot rounds */
  if(!control->user_bytes[0]) {
    control->schedule.n_slots = 1;
    GMW_CONTROL_SET_CONFIG(control);
    control->schedule.slot[0] = static_nodes_used[round_counter];
    GMW_CONTROL_SET_USER_BYTES(control);
    round_counter = (round_counter + 1) % slots_per_round;
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
  host_impl->on_control_slot_post   = &on_control_slot_post_callback;
  host_impl->on_slot_pre            = &on_slot_pre_callback;
  host_impl->on_slot_post           = &on_slot_post_callback;
  host_impl->on_round_finished      = &on_round_finished;

  /* load the source node implementation */
  src_impl->on_control_slot_post    = &on_control_slot_post_callback;
  src_impl->on_slot_pre             = &on_slot_pre_callback;
  src_impl->on_slot_post            = &on_slot_post_callback;
  src_impl->on_round_finished       = &on_round_finished;
  src_impl->on_bootstrap_timeout    = &src_on_bootstrap_timeout;

  /* loads __default__ schedule and config parameters */
  gmw_control_init(control);

  /* loads __application__ initial control parameters */
  app_control_init(control);

  /* notify the middleware that the host-app has a new control */
  gmw_set_new_control(control);
}
/*---------------------------------------------------------------------------*/
