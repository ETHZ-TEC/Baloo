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
 *         Link quality estimation test
 *         ===
 *
 *         *** GENERAL DESCRIPTION ***
 *
 *         The test consists in one main Baloo round, where each node in the
 *         network is assigned one data slot.
 *         In these slots, nodes send a fixed number of strobes
 *         (GMW_CONF_TX_CNT_DATA, defined to 100 by default ).
 *         The assignment of nodes to the slots is pseudo-random (controlled
 *         by "randomseed").
 *
 *         All other nodes listen to the strobes and log the reception data
 *         over serial. The program execution is also monitored using GPIOs.
 *
 *         The TX power is set to 0dBm for all transmissions.
 *
 *         The default frequency channel used is:
 *         - Channel 26 (2.48 GHz) for the sky mote
 *         - Channel  5 (869  MHz) for the DPP-cc430 mote
 *         This can be modified after compilation by binary patching (see below)
 *
 *         *** CODE INSTRUMENTATION ***
 *
 *         The serial log is produced at the end of the test. They are formatted
 *         as follows:
 *
 *         " Log:X:Y:Z[0].Z[1]. (...) .Z[n] "
 *           - X    is the node_id of the initiator
 *           - Y    is the number of strobes successfully received
 *           - Z[i] is the i-th byte of the bit-stream of reception events;
 *           i.e., Z[0] contains the binary reception information of the first
 *           8 strobes.
 *           For example: Z[0] = 23 = 0b 0001 0111, means
 *            + the first 3 strobes were lost
 *            + the 4-th was received
 *            + the 5-th was lost
 *            + the last 3 strobes were received
 *
 *         For the slot where the node is initiator, the log simply says:
 *
 *         " Log:slot_assignee:Strobing "
 *
 *         If a data slot is "missed" or "skipped" by Baloo, a notification
 *         is written in the serial log:
 *
 *         " Missed/Skipped:slot_index,slot_assignee "
 *
 *         The corresponding line of results is then:
 *         " Log:slot_assignee:Err! "
 *
 *         In addition, the code is instrumented with three GPIO pin lines
 *         to track the program execution (PRIM = {GLOSSY, STROBING})
 *           +  PRIM_START_PIN track the execution of
 *           the primitive; i.e., roughly from the slot start till slot end
 *           +  PRIM_TX_PIN track the TX phases of the primitives
 *           +  PRIM_RX_PIN track the RX phases of the primitives
 *         For the strobing primitive, another pin marks every successful
 *         strobe reception, mapped to the line as the TX_PIN;
 *         i.e., successful strobe receptions are followed by a raising and
 *         falling edge of the TX_PIN line.
 *
 *         *** TIME SYNCHRONIZATION ***
 *
 *         Before the main Baloo round, a few empty rounds are run. These give
 *         time to all nodes to synchronize with the HOST, and to learn their
 *         internal clock drift.
 *         There are "trigger_counter" such empty rounds.
 *         The round period is to 10 seconds to enable a good estimation of
 *         the drift (the longer the period, the better).
 *
 *         *** PROGRAM INTERFACE ***
 *
 *         The code is written to enable binary patching after compilation.
 *         Currently, this is done via the "tos-set-symbol" utility,
 *         and the user is free to patch
 *
 *         - the strobe payload length,
 *           by patching "payload_length"
 *
 *         - the random generator seed,
 *           by patching "randomseed"
 *
 *         - the HOST_ID,
 *           by patching "host_id"
 *
 *         - the radio frequency channel,
 *           by patching "rf_channel"
 *
*          TODO: modify this with the patching of an entire struct (method from
*          Markus). This will allow to have everything in one place, and to
*          extend to patching to the node list.
*          This would be nice, as the firmware could be then distributed and
*          used even without requiring anyone to recompile, just patch it to fit
*          your testbed!
 */


/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
#include "node-id.h"
#include "glossy.h"
#include "random.h"
#include "leds.h"
#include "strobing.h"
#include "stdio.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;

/*---------------------------------------------------------------------------*/
/*- XP PARAMETERS -----------------------------------------------------------*/

/**
 * The following variables are included in the code
 * such that they can be patched in the firmware, after compilation.
 */

#ifndef PAYLOAD_LEN
#error The payload length is not specified
#endif /* PAYLOAD_LEN */
uint16_t payload_length = PAYLOAD_LEN;

#ifndef RANDOM_SEED
#error No random seed specified.
#endif /* RANDOM_SEED */
uint16_t randomseed = RANDOM_SEED;

#ifndef HOST_ID_MAKEFILE
#error No host id specified.
#endif /* HOST_ID_MAKEFILE */
uint16_t host_id = HOST_ID_MAKEFILE;

#ifndef RF_CHANNEL
#error No frequency channel specified.
#endif /* RF_CHANNEL */
uint8_t rf_channel = RF_CHANNEL;

/*---------------------------------------------------------------------------*/
/*- GPIO --------------------------------------------------------------------*/

#ifdef XP_ACT_PIN
  #define XP_GPIO_ON           PIN_SET(XP_ACT_PIN)
  #define XP_GPIO_OFF          PIN_CLR(XP_ACT_PIN)
#else /* XP_ACT_PIN */
  #define XP_GPIO_ON
  #define XP_GPIO_OFF
#endif /* XP_ACT_PIN */

/*---------------------------------------------------------------------------*/
/*- APP VARIABLES -----------------------------------------------------------*/
#define NUM_BYTES     GMW_CONF_TX_CNT_DATA/8 \
                      + (GMW_CONF_TX_CNT_DATA%8 != 0)
#define SLOT_FAILED   255

static uint16_t static_nodes_all[NUM_NODES] = NODE_LIST;
static uint16_t static_nodes_ordered[NUM_NODES];
static uint8_t  test_results_bitstream[NUM_NODES][NUM_BYTES];
static uint8_t  test_results_total[NUM_NODES];
static uint8_t  is_synced;
/* Number of rounds at start-up,
 * used for learning clock drifts
 * */
static uint8_t trigger_counter  = 5;
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

  /* --- Application-specific initialization --- */

  is_synced = (node_id == HOST_ID);

  /* Fill the static list randomly
   * -> Each node appears once in the list, but the ordering is random
   * */
  random_init(randomseed);
  int i;
  int n_id = 0;
  for(i=0; i<NUM_NODES; i++) {
    /* find a node id not yet assigned a slot */
    do
      n_id = random_rand() % NUM_NODES;
    while(static_nodes_all[n_id] == 0);
    /* assign the node the i-th slot */
    static_nodes_ordered[i] = static_nodes_all[ n_id ];
    /* mark the node assigned */
    static_nodes_all[n_id] = 0;
  }

  /* initialized result array with SLOT_FAILED
   * -> This is useful in case a node would not participate
   * in the main round (e.g., because control packet is missed)
   * */
  memset(test_results_total, SLOT_FAILED, NUM_NODES);

  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);

  DEBUG_PRINT_INFO("=== Link quality test ===");
  DEBUG_PRINT_INFO("Payload: %u bytes", payload_length);
  DEBUG_PRINT_INFO("Seed: %u", randomseed);

  debug_print_poll();

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* print the test results */
    if(control.user_bytes[0] == 0) {
      DEBUG_PRINT_MSG_NOW("=== Results ===");
      int i,j;
      /* make sure 'to_print' is big enough (given the number of strobes)
       * - Log tag (Log:)                 -> 4 char
       * - initiator ID (uint16_t)        -> 5 char
       * - separator (:)                  -> 1 char
       * - number of receptions (0-100)   -> 3 char
       * - separator (:)                  -> 1 char
       * - reception bit stream: x*(NUM_BYTES)
       *  - 8-bit stream (0-255)  -> 3 char
       *  - separator (.)         -> 1 char
       *  - eol                   -> 1 char
       * - eol                            -> 1 char
       * */
      char to_print[4+5+1+3+1+(3+1)*(NUM_BYTES)+1];
      char to_add[5];
      for(i=0;i<NUM_NODES; i++) {

        /* nothing to log when we are the initiator */
        if( node_id == static_nodes_ordered[i] ) {
          DEBUG_PRINT_MSG_NOW("Log:%u:Strobing", node_id);
          continue;
        }

        /* clean the strings */
        memset(to_print,  0, sizeof(to_print));
        memset(to_add,    0, sizeof(to_add));

        /* log the initiator node id */
        sprintf(to_print, "Log:%u:", static_nodes_ordered[i]);

        /* log the total number of receptions */
        if(test_results_total[i] == SLOT_FAILED) {

          /* slot failed */
          sprintf(to_add, "Err!");
          strcat(to_print, to_add);

        } else {

          /* slot executed normally */
          sprintf(to_add, "%u:", test_results_total[i]);
          strcat(to_print, to_add);

          /* log the reception bit stream */
          for(j=0;j<(NUM_BYTES);j++) {
            memset( to_add, 0, sizeof(to_add));
            sprintf(to_add, "%u.", test_results_bitstream[i][j]);
            strcat(to_print, to_add);
          }
        }
        DEBUG_PRINT_MSG_NOW("%s", to_print);
      }
      DEBUG_PRINT_MSG_NOW("=== Test completed ===");
      DEBUG_PRINT_MSG_NOW("=== Error logs ===");

#if TEST_CONF_ONE_ROUND
      /* stop Baloo */
      GMW_RTIMER_STOP();
#else
      /* reset the logging arrays */
      memset(test_results_total, SLOT_FAILED, NUM_NODES);
      memset(test_results_bitstream, 0, sizeof(test_results_bitstream));
#endif /* TEST_CONF_ONE_ROUND */

    } else if (control.user_bytes[0] == 1) {
      DEBUG_PRINT_MSG_NOW("1 round  left for synchronization...");
    } else {
      DEBUG_PRINT_MSG_NOW("%u rounds left for synchronization...",
                       control.user_bytes[0]);
    }

    if(node_id == HOST_ID) {
      app_control_update(&control);
      gmw_set_new_control(&control);
    }

    debug_print_poll();
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
  if(is_synced && (event != GMW_EVT_CONTROL_RCVD)
     && (in_out_control->user_bytes[0] > 0)){
    /* Manually update the user_byte */
    in_out_control->user_bytes[0] -= 1;
  }
  app_control_static_update(in_out_control);

  /* Save a local copy of the control*/
  control = *in_out_control;

  if(event == GMW_EVT_CONTROL_RCVD){
    leds_on(LEDS_GREEN);
    leds_off(LEDS_RED);
  } else {
    leds_off(LEDS_GREEN);
    leds_on(LEDS_RED);
  }

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
    /* random payload */
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

  /* Log erroneous receptions
   * E: slot_index, slot_assignee, error code */
  if(event == GMW_EVT_PKT_MISSED) {
    DEBUG_PRINT_INFO("Missed:%u:%u", slot_index, slot_assignee);
    test_results_total[slot_index] = SLOT_FAILED;

  } else if(event == GMW_EVT_PKT_SKIPPED) {
    DEBUG_PRINT_INFO("Skipped:%u:%u", slot_index, slot_assignee);
    test_results_total[slot_index] = SLOT_FAILED;

  } else if( !is_initiator ) {
    /* log the total number of receptions */
    test_results_total[slot_index] = GMW_GET_N_RX_PRIM2();
    /* log bit stream of receptions */
    memcpy(&(test_results_bitstream[slot_index]),
           strobing_get_rx_binary(),
           (NUM_BYTES) );
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

  leds_off(LEDS_GREEN);
  leds_on(LEDS_RED);

  XP_GPIO_OFF;

  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
    /* The first few rounds have no data slots,
     * they are used solely for time synchronization:
     * - sync with the HOST node
     * - estimate the clock drifts
     * */
    gmw_schedule_t* sched = &control->schedule;
    int i;
    for(i=0; i<NUM_NODES; i++) {
      sched->slot[i] = (uint8_t) static_nodes_ordered[i];
    }
    sched->n_slots = 0;
    sched->time    = 0;
#ifdef FLOCKLAB
    sched->period  = 10; /* seconds */
#else
    sched->period  = 1; /* speed-up for testing on the desk */
#endif

    control->config.primitive         = GMW_PRIM_STROBING;
    control->config.max_packet_length = STROBING_CONF_PAYLOAD_LEN;

    GMW_CONTROL_SET_CONFIG(control);
    GMW_CONTROL_SET_USER_BYTES(control);

    if(node_id == HOST_ID) {
      /* The user bytes are always sent by the host,
       * They are used (in this example) by all nodes to perform the update of the
       * static schedule and config
       */
      control->user_bytes[0] = trigger_counter;
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
   * triggers the main round */
  if(!control->user_bytes[0]) {
    /* You may need to increase the period in case of many nodes */
    control->schedule.period = 10;
    control->schedule.n_slots = NUM_NODES;
    GMW_CONTROL_SET_CONFIG(control);
    /* keep the user bytes, otherwise control has 0 bytes of payload...*/
    GMW_CONTROL_SET_USER_BYTES(control);
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
