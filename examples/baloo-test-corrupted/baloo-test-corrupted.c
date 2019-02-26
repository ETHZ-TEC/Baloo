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
 *         Baloo protocol illustrating the utilization of the interference 
 *         detection feature
 */


/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "node-id.h"
#include "debug-print.h"
#include "random.h"
#include "leds.h"
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
static uint8_t              counter = 0;

static uint32_t number_of_total_slots = 0;
static uint32_t number_of_received_slots = 0;
static uint32_t number_of_corrupted_slots = 0;
static uint32_t number_of_garbage_slots = 0;
static uint32_t number_of_silence_slots = 0;
static uint32_t number_of_total_controls = 0;
static uint32_t number_of_received_controls = 0;

static uint8_t is_synced = 0;
static uint8_t is_sender = 0;

/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
static void app_control_static_update(gmw_control_t* control);
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
  // Initialize pseudo-random generator based on the node id
  random_init(node_id);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
    DEBUG_PRINT_INFO("%lu,%lu,%lu,%lu,%lu,%lu,%lu,%u\r\n",
           number_of_received_slots,
           number_of_corrupted_slots,
           number_of_garbage_slots,
           number_of_silence_slots,
           number_of_total_slots,
           number_of_received_controls,
           number_of_total_controls,
           glossy_get_fsr());

    debug_print_poll();

#if GMW_CONF_USE_LF_FOR_WAKEUP
    GMW_BEFORE_DEEPSLEEP();
#endif /* GMW_CONF_USE_LF_FOR_WAKEUP */
    
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
host_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                   gmw_sync_event_t sync_event,
                                   gmw_pkt_event_t pkt_event)
{
  /* If either the sched or the config is static */
  if (GMW_CONF_USE_STATIC_CONFIG ||
      GMW_CONF_USE_STATIC_SCHED) {

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
host_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
              uint8_t* out_len, uint8_t* out_payload,
              uint8_t is_initiator, uint8_t is_contention_slot)
{
  static uint16_t rand_draw;

  if(is_initiator) {
    rand_draw = random_rand();
    DEBUG_PRINT_VERBOSE("random number: %u", rand_draw);

    if (rand_draw > 10000) { /* roughly, 5 out of 6 chance */
      out_payload[0] = 42;
      out_payload[1] = counter;
      *out_len = 2;
    }
    counter++;
  }
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
               uint8_t len, uint8_t* payload,
               uint8_t is_initiator, uint8_t is_contention_slot,
               gmw_pkt_event_t event)
{
  if(!is_initiator) {

    if (event == GMW_EVT_PKT_OK){
      DEBUG_PRINT_VERBOSE("Host received %u bytes of payload ", len);
      DEBUG_PRINT_VERBOSE("%u %u", payload[0], payload[1]);
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_INFO("RSSI: %idBm over %u samples, slot: %u",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples, slot: %u",
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif /* GMW_CONF_USE_NOISE_DETECTION */
      number_of_received_slots++;
    }

    else if (event == GMW_EVT_PKT_CORRUPTED){
      DEBUG_PRINT_VERBOSE("Host received a corrupted packet in slot %u", slot_index);
#if GMW_CONF_USE_NOISE_DETECTION
      DEBUG_PRINT_VERBOSE("RSSI: %idBm over %u samples, slot: %u",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
#endif
      number_of_corrupted_slots++;
    }

    else if (event == GMW_EVT_PKT_GARBAGE){
      DEBUG_PRINT_INFO("Host received garbage in slot %u", slot_index);
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_VERBOSE("RSSI: %idBm over %u samples, slot: %u",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
        DEBUG_PRINT_VERBOSE("Noisy channel: %idBm over %u samples, slot: %u",
                       gmw_get_rssi_high_noise_average(),
                       gmw_get_noise_measurement_high_noise_samples(),
                       slot_index
                       );
#
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples, slot: %u",
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif
      number_of_garbage_slots++;
    }

    else if (event == GMW_EVT_PKT_SILENCE){
      DEBUG_PRINT_VERBOSE("Host didn't receive anything from node %u", slot_assignee);
#if GMW_CONF_USE_NOISE_DETECTION
      DEBUG_PRINT_VERBOSE("RSSI: %idBm over %u samples, slot: %u",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples(),
                       slot_index
                       );
#endif
      number_of_silence_slots++;
    }

	number_of_total_slots++;


  }
  return GMW_EVT_REPEAT_DEFAULT;
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
                                  gmw_sync_event_t sync_event,
                                  gmw_pkt_event_t pkt_event)
{

  /* If either the sched or the config is static */
  if (GMW_CONF_USE_STATIC_CONFIG ||
      GMW_CONF_USE_STATIC_SCHED) {

    /* The first time a control packet is received,
     * fill the static schedule to send to the middleware
     */
    if (!is_synced) {
      if (GMW_CONF_USE_STATIC_SCHED) {
        in_out_control->schedule = control.schedule;
      }
      if (GMW_CONF_USE_STATIC_CONFIG) {
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

  if (sync_event == GMW_EVT_CONTROL_RCVD){
    number_of_received_controls++;
    number_of_received_slots++;
  }
  else { /* Control missed */
    if (pkt_event == GMW_EVT_PKT_CORRUPTED){
      DEBUG_PRINT_INFO("Control packet corrupted");
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_INFO("RSSI: %idBm over %u samples",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples()
                       );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples",
                       gmw_get_noise_measurement_total_samples()
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif /* GMW_CONF_USE_NOISE_DETECTION */
      number_of_corrupted_slots++;
    }
    else if (pkt_event == GMW_EVT_PKT_GARBAGE){
      DEBUG_PRINT_INFO("Control packet is garbage");
#if GMW_CONF_USE_NOISE_DETECTION

#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_VERBOSE("Noisy RSSI: %idBm over %u samples",
                      gmw_get_rssi_high_noise_average(),
                      gmw_get_noise_measurement_high_noise_samples()
                      );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples",
                       gmw_get_noise_measurement_total_samples()
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif
      number_of_garbage_slots++;
    }
    else if (pkt_event == GMW_EVT_PKT_SILENCE){
      DEBUG_PRINT_INFO("No control packet detected");
#if GMW_CONF_USE_NOISE_DETECTION
      DEBUG_PRINT_VERBOSE("RSSI: %idBm over %u samples",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples()
                       );
#endif
      number_of_silence_slots++;
    }

  }

  number_of_total_slots++;
  number_of_total_controls++;
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
  static uint16_t rand_draw;

  if(is_contention_slot) {
    rand_draw = random_rand();
    DEBUG_PRINT_VERBOSE("random number: %u", rand_draw);

    if (rand_draw > 30000) { /* roughly, 50% chance */
      out_payload[0] = counter;
      out_payload[1] = (uint8_t) node_id;
      *out_len = 2;

      is_sender = 1;
    }
    else {
      is_sender = 0;
      *out_len = 0;
    }
    counter--;
  }
  else if (!is_initiator){
    is_sender = 0;
  }

  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
src_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
              uint8_t len, uint8_t* payload,
              uint8_t is_initiator, uint8_t is_contention_slot,
              gmw_pkt_event_t event)
{
  if(!is_sender) {
	if(len == 2) { /* this is the length we expect */
	  DEBUG_PRINT_VERBOSE("Src received %u %u", payload[0],
			   payload[1]);
	  number_of_received_slots++;
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_INFO("RSSI: %idBm over %u samples",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples()
                       );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples",
                       gmw_get_noise_measurement_total_samples()
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif /* GMW_CONF_USE_NOISE_DETECTION */
	} else { /* In all those case, len=0 */
	  if (event == GMW_EVT_PKT_CORRUPTED){
	    DEBUG_PRINT_INFO("packet corrupted (slot %u).", slot_index);
        number_of_corrupted_slots++;
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_INFO("Noisy RSSI: %idBm over %u samples",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples()
                       );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples",
                       gmw_get_noise_measurement_total_samples()
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif /* GMW_CONF_USE_NOISE_DETECTION */
	  }
	  else if (event == GMW_EVT_PKT_GARBAGE){
        DEBUG_PRINT_INFO("Interference, flood missed. Packet from slot %u", slot_index);
        number_of_garbage_slots++;
#if GMW_CONF_USE_NOISE_DETECTION
#ifdef PLATFORM_DPP_CC430
        DEBUG_PRINT_INFO("RSSI: %idBm over %u samples",
                       gmw_get_rssi_average(),
                       gmw_get_noise_measurement_total_samples()
                       );
#elif defined PLATFORM_SKY
        DEBUG_PRINT_INFO("Signal Strength: %u samples",
                       gmw_get_noise_measurement_total_samples()
                       );
        DEBUG_PRINT_INFO("%u samples above threshold (%idBm)",
                       gmw_get_noise_measurement_high_noise_samples(),
                       GMW_CONF_HIGH_NOISE_THRESHOLD
                       );
#endif /* PLATFORM_SKY */
#endif /* GMW_CONF_USE_NOISE_DETECTION */
	  }
	  else if (event == GMW_EVT_PKT_SILENCE){
        DEBUG_PRINT_INFO("No transmission detected. Silence in slot %u", slot_index);
        number_of_silence_slots++;
	  }
	  else{
	    DEBUG_PRINT_INFO("Wrong number of Bytes received (len=%u)", len);
	  }
	}
	number_of_total_slots++;
  }
  return GMW_EVT_REPEAT_DEFAULT;
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
  // Nothing to do
  leds_off(LEDS_GREEN);
  leds_on(LEDS_RED);
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  if (GMW_CONF_USE_STATIC_CONFIG || GMW_CONF_USE_STATIC_SCHED || HOST_ID == node_id){

    gmw_schedule_t* sched = &control->schedule;
    int i;
    sched->slot[0] = HOST_ID;                     /* first slot for the host */
    for (i=1; i<GMW_CONF_MAX_SLOTS; i++) {
      sched->slot[i] = 0xffff;                    /* all others are contention slots */
    }
    sched->n_slots = GMW_CONF_MAX_SLOTS;
    sched->time = 0;
    sched->period = 2000;

    GMW_CONTROL_SET_CONFIG(control);

    control->config.gap_time            = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
    control->config.slot_time           = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA);
    control->config.n_retransmissions   = GMW_CONF_TX_CNT_DATA;

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
  if (!GMW_CONF_USE_STATIC_SCHED) {
    control->schedule.time += control->schedule.period;
  }

  /* update the config if not static*/
  if (!GMW_CONF_USE_STATIC_CONFIG) {
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
  if (GMW_CONF_USE_STATIC_SCHED) {
    control->schedule.time += control->schedule.period;
  }

  /* update the config if static*/
  if (GMW_CONF_USE_STATIC_CONFIG) {
  }
}
/*---------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/**
 * GMW initialization function
 *
 * /!\/!\/!\ DO NOT MODIFY /!\/!\/!\
 *
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

