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
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         Implementation of TTnet using Baloo
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
#include "ttnet.h"
#include "ttnet_nodes.h"
#include "ttnet_messages.h"
/*---------------------------------------------------------------------------*/

/*- GMW VARIABLES -----------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;
/*---------------------------------------------------------------------------*/
/*- TTW VARIABLES -----------------------------------------------------------*/
static uint8_t dummy_counter;
static uint8_t mode_change_counter = 0; /* dummy for emulating mode changes */

static uint8_t is_synced = 0;
static ttw_role_t node_role; /* sender, receiver, forwarder*/

// rounds
static uint8_t  current_round_id  = 0;
static uint8_t  next_round_id     = 0;

// modes
static uint8_t  current_mode_id; // current mode   (associated to the round)
static uint8_t  next_mode_id;    // announced mode (sent in the beacon)
static uint8_t  switching_bit;

// scheduling tables

#define TTW_STARTING_MODE               1
#define TTW_NUMBER_MODES                3
#define TTW_NUMBER_ROUNDS               7

static ttw_mode_t   mode_array[TTW_NUMBER_MODES];
static ttw_round_t  round_array[TTW_NUMBER_ROUNDS];
static ttw_round_t* current_round_pt;
static int16_t      current_message_id;

//static uint8_t message_buffer[TTW_NUMBER_MESSAGES][TTW_MAX_PAYLOAD_LEN];

static int16_t sched_table[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND];

static gmw_statistics_t* stats_pt = NULL;
/*

TTW_ROUND_TIME
 *
 */


/*---------------------------------------------------------------------------*/
/*- TTW PROTOTYPES ----------------------------------------------------------*/
//static uint16_t get_round_period(uint16_t round_id);
static void load_sched_table();

/*---------------------------------------------------------------------------*/
/*- STANDARD PROTOTYPES -----------------------------------------------------*/
static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
static void app_control_static_update(gmw_control_t* control);
/*---------------------------------------------------------------------------*/


PROCESS(app_process, "Application Task");
PROCESS(pre_process, "Pre-round Task");
AUTOSTART_PROCESSES(&app_process,&pre_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* --- Application-specific initialization --- */

  /* load the scheduling table in each node's memory */
  load_sched_table();

  /* get pointer to GMW stats */
  stats_pt = gmw_get_stats();


  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(&pre_process, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  int i = 0;
  while(1) {
    /* the app task should not do anything until it is explicitly granted
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    /* Check*/
    for(i=0;i<4;i++){
      DEBUG_PRINT_INFO("round %i: %i %i %i %i", i, sched_table[i][0],
                       sched_table[i][1],
                       sched_table[i][2],
                       sched_table[i][3]);
    }

    DEBUG_PRINT_INFO("Round finished");
    DEBUG_PRINT_INFO("round period: (%lums)", GMW_PERIOD_TO_MS(control.schedule.period));
    DEBUG_PRINT_INFO("round "
        "period: (%lums)",
        GMW_PERIOD_TO_MS(mode_array[TTW_STARTING_MODE].hyperperiod));

//    DEBUG_PRINT_WARNING("B: %u, T: %lu",
//    									round_array[current_round_id].n_slots,
//                     (uint32_t)(GMW_TICKS_TO_US(stats_pt->t_round_last)));
//    DEBUG_PRINT_WARNING("slot time %lu", GMW_SLOT_TIME_TO_US(control.config.slot_time));
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

  /* main loop of this application task */
  while(1) {
    /* the task should not do anything until it is explicitly granted
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    DEBUG_PRINT_INFO("Pre-process runs");

    /* Flush Bolt */
    // to be implemented -> for real

    // to be implemented -> mock-up:
    //    packets are always there, filled with dummy payload

    /* Update the beacon information */
    if(node_id == HOST_ID) {
      app_control_update(&control);
      gmw_set_new_control(&control);
      DEBUG_PRINT_INFO("Current round: %i", TTW_GET_BEACON_ROUND(&control));
      DEBUG_PRINT_INFO("Sched_table: %i %i %i", sched_table[TTW_GET_BEACON_ROUND(&control)][0],
                     sched_table[TTW_GET_BEACON_ROUND(&control)][1],
                     sched_table[TTW_GET_BEACON_ROUND(&control)][2]);
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
host_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                   gmw_sync_event_t event,
                                   gmw_pkt_event_t  pkt_event)
{
  /* The host always has the latest beacon information
   * - Re-construct the host's schedule
   * - Store in your local control copy
   * - Set your state as running
   */

  /* Update the static control
   * -> Happens here as the control update might depend on the info
   *    received in the control slot (for example, current round ID)
   * */
  app_control_static_update(in_out_control);

  /* Save a local copy */
  control = *in_out_control;

  leds_on(LEDS_GREEN);
  return GMW_RUNNING;
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t event,
                                  gmw_pkt_event_t  pkt_event)
{
  /* The source nodes need to successfully receive beacons
   * - Upon reception of the first beacon, send the local control info to gmw
   * - Re-construct the node schedule
   * - Store in your local control copy
   * - Set your state as running
   */

  //TODO: think about the desired behavior when beacon is missed (Default or else)

  if(event == GMW_EVT_CONTROL_RCVD){

    /* The first time a control packet is received,
     * fill the static schedule to send to the middleware
     */
    if(!is_synced) {
      in_out_control->schedule  = control.schedule;
      in_out_control->config    = control.config;
    }
    is_synced = 1;

    /* Update the static schedule
     * -> Happens here as the schedule update might depend on the info
     *    received in the control slot (for example, current round ID)
     **/
    app_control_static_update(in_out_control);

    /* Save a local copy of the control*/
    control = *in_out_control;

    leds_on(LEDS_GREEN);
    leds_off(LEDS_RED);
    return GMW_RUNNING;

  } else {
    /*
    // Notify the APP and return
    if ( !bolt_write("Control packet missed", 22) )
    {
      DEBUG_PRINT_ERROR("ERROR bolt_write failed");
    }
    */
    leds_on(LEDS_RED);
    return GMW_DEFAULT;
  }
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
on_slot_pre_callback(uint8_t slot_index,
                          uint16_t slot_assignee,
                          uint8_t* out_len,
                          uint8_t* out_payload,
                          uint8_t is_initiator,
                          uint8_t is_contention_slot)
{
  /* Don't use the is_initiator info, use the info comprised in the
   * round array schedule instead
   * -> all nodes already know what to do based on that info */

  /* Skip unused slots */
  if(slot_index >= round_array[current_round_id].n_slots) {
    return GMW_EVT_SKIP_ROUND;
  }

  /* Read the current message ID */
  current_message_id = sched_table[current_round_id][slot_index];

  /* == Initiator == */
  if(current_message_id > 0) {

    // Set role
    node_role = TTW_SENDER;

    // Copy the payload
    // -> eventually, will be a memcpy, done manually for now
    out_payload[0] = (uint8_t) node_id;
    out_payload[1] = dummy_counter++;
    *out_len = TTW_MAX_PAYLOAD_LEN;

  /* == Receiver == */
  } else if(current_message_id < 0) {
    // Set role
    node_role = TTW_RECEIVER;

  /* == Forwarder == */
  } else {
    // Set role
    node_role = TTW_FORWARDER;
  }

  return GMW_EVT_SKIP_DEFAULT;
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
/*
 * Eventually, might look something like this:
  if( len ){
    if ( !bolt_write( glossy_payload, len ) ){
    DEBUG_PRINT_ERROR("ERROR: bolt_write failed");
    }
  }
 */


  if(node_role == TTW_SENDER) {

    if(event == GMW_EVT_PKT_OK){
      DEBUG_PRINT_INFO("Slot %i: Send Suc.", slot_index);
    } else {
      DEBUG_PRINT_INFO("Slot %i: Send Fail (%u)", slot_index, event);
    }

  } else if(node_role == TTW_RECEIVER) {

    if(event == GMW_EVT_PKT_OK) {
      DEBUG_PRINT_INFO("Slot %i: Rcv Suc. (%u %u)", slot_index, payload[0], payload[1]);
    } else {
      DEBUG_PRINT_INFO("Slot %i: Rcv Fail (%u)", slot_index, event);
    }

  } else if(node_role == TTW_FORWARDER){

    // Still good to know if we missed the slot...
    if(event == GMW_EVT_PKT_OK) {
      DEBUG_PRINT_INFO("Slot %i: Fwd Suc. (%u)", slot_index, event);
    } else {
      DEBUG_PRINT_INFO("Slot %i: Fwd Fail (%u)", slot_index, event);
    }
  }

  /* Notify the middleware when the last slot is reached */
  if(slot_index+1 == round_array[current_round_id].n_slots) {
    return GMW_EVT_END_ROUND;
  } else {
    return GMW_EVT_NO_REPEAT;
  }

}
/*---------------------------------------------------------------------------*/
static void
on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Update the control
   * -> done in the pre-process instead
   * -> Why? To run as late as possible before the next round */

  // app_control_update(&control);
  // gmw_set_new_control(&control);

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

  /* schedule */
  control->schedule.n_slots = TTW_MAX_SLOTS_PER_ROUND;
  // Set all slots as contention slots (see on_slot_pre_cb for details)
  int i;
  for(i=0;i<TTW_MAX_SLOTS_PER_ROUND;i++){
    control->schedule.slot[i] = GMW_SLOT_CONTENTION;
  }

  /* config
   * -> Use Baloo's default settings */
  GMW_CONTROL_SET_CONFIG(control);

  /* user bytes
   * -> Needed for all nodes (otherwise not decompiled from buffer) */
  GMW_CONTROL_SET_USER_BYTES(control);

  /* Add host-specific information */
  if(HOST_ID == node_id){

  	/* Set the round period for the first round */
    uint8_t first_round = mode_array[TTW_STARTING_MODE].first_round_id;
    if(round_array[first_round].mode_id != round_array[first_round+1].mode_id) {
      // if the starting mode has only one round, use the mode hyperperiod
    	control->schedule.period = mode_array[TTW_STARTING_MODE].hyperperiod;
    } else {
    	// else, take the difference of offsets of the first two rounds
    	control->schedule.period = round_array[first_round+1].start_time_offset
																- round_array[first_round].start_time_offset;
    }

    /* user bytes
     * They are used to encode the TTW beacon informations
     */
    TTW_SET_BEACON_MODE(control, TTW_STARTING_MODE);
    TTW_SET_BEACON_ROUND(control, mode_array[TTW_STARTING_MODE].first_round_id);
    TTW_CLR_BEACON_SB(control);
  }
}

/*---------------------------------------------------------------------------*/
static void
load_sched_table()
{
  /* Parameter points to an array of the following shape
   *    sched_table[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND]
   *
   * This function would eventually implement the automatic conversion from the
   * schedule synthesis to the respective node table.
   *
   * For now, done manually.
   * */

  /* At this stage, it is not clear yet how we are going to handle the
   * filling of the schedule info in memory.
   * -> For now, we do everything by hand at initialization (aka, here)
   */

	/* Modes */
  mode_array[0].hyperperiod     = 1000LU; /* ms */
  mode_array[0].first_round_id  = 0;
  mode_array[1].hyperperiod     = 1000LU; /* ms */
  mode_array[1].first_round_id  = 4;
  mode_array[2].hyperperiod     = 50LU; /* ms */
  mode_array[2].first_round_id  = 6;

  /* Rounds */
  // Mode 0
  round_array[0].mode_id = 0;
  round_array[0].n_slots = 2;
  round_array[0].start_time_offset = 5LU;

  round_array[1].mode_id = 0;
  round_array[1].n_slots = 3;
  round_array[1].start_time_offset = 32LU;
//  round_array[1].start_time_offset = 250LU; //32LU;

  round_array[2].mode_id = 0;
  round_array[2].n_slots = 1;
  round_array[2].start_time_offset = 510LU;

  round_array[3].mode_id = 0;
  round_array[3].n_slots = 4;
  round_array[3].start_time_offset = 539LU;
//  round_array[3].start_time_offset = 729LU;
  // --

  // Mode 1
	round_array[4].mode_id = 1;
  round_array[4].n_slots = 2;
  round_array[4].start_time_offset = 5LU;

  round_array[5].mode_id = 1;
  round_array[5].n_slots = 2;
  round_array[5].start_time_offset = 505LU;
  // --

  // Mode 2
	round_array[6].mode_id = 2;
  round_array[6].n_slots = 1;
  round_array[6].start_time_offset = 5LU;
  // --

  /* Scheduling tables */
  if(node_id == SENSOR1) {
    int16_t sched_table_instance[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND] =  {
			// M_ACT_1 M_BROADCAST
      { M_ACK1, -M_BROADCAST } ,

      // M_SENS_2, M_ACK1, M_ACK2
      { 0, M_ACK1, 0} ,

      // M_BROADCAST
			{ -M_BROADCAST } ,

			// M_SENS_1, M_SENS_2, M_ACK1, M_ACK2
			{ M_SENS_1, 0, M_ACK1, 0} ,

      // M_ACT_1, M_SENS_2
      { 0, 0} ,

			// M_SENS_1, M_SENS_2
      { M_SENS_1, 0} ,

      // M_ALERT
      { M_ALERT } ,
    };

    /* Store in external variable */
    memcpy(&sched_table, &sched_table_instance,
           2*TTW_NUMBER_ROUNDS*TTW_MAX_SLOTS_PER_ROUND);

  } else if(node_id == SENSOR2) {
    int16_t sched_table_instance[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND] =  {
			// M_ACT_1, M_BROADCAST
			{ 0, -M_BROADCAST  } ,

			// M_SENS_2, M_ACK1, M_ACK2
			{ M_SENS_2, 0, M_ACK2} ,

			// M_BROADCAST
			{ -M_BROADCAST } ,

			// M_SENS_1, M_SENS_2, M_ACK1, M_ACK2
			{ 0, M_SENS_2, 0, M_ACK2} ,

			// M_ACT_1, M_SENS_2
			{ 0, M_SENS_2} ,

			// M_SENS_1, M_SENS_2
			{ 0, M_SENS_2} ,

			// M_ALERT
      { 0 } ,
    };

    /* Store in external variable */
    memcpy(&sched_table, &sched_table_instance,
           2*TTW_NUMBER_ROUNDS*TTW_MAX_SLOTS_PER_ROUND);

  } else if(node_id == ACTUATOR) {
    int16_t sched_table_instance[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND] =  {
    	// M_ACT_1, M_BROADCAST
			{ -M_ACT_1, 0 } ,

			// M_SENS_2, M_ACK1, M_ACK2
			{ 0, 0, 0} ,

			// M_BROADCAST
			{ 0 } ,

			// M_SENS_1, M_SENS_2, M_ACK1, M_ACK2
			{ 0, 0, 0, 0} ,

			// M_ACT_1, M_SENS_2
			{ -M_ACT_1, 0} ,

			// M_SENS_1, M_SENS_2
			{ 0, 0} ,

			// M_ALERT
      { 0 } ,
    };

    /* Store in external variable */
    memcpy(&sched_table, &sched_table_instance,
           2*TTW_NUMBER_ROUNDS*TTW_MAX_SLOTS_PER_ROUND);

  } else if(node_id == CONTROLLER) {
    int16_t sched_table_instance[TTW_NUMBER_ROUNDS][TTW_MAX_SLOTS_PER_ROUND] =  {
			// M_ACT_1, M_BROADCAST
			{ 0, M_BROADCAST } ,

			// M_SENS_2, M_ACK1, M_ACK2
			{ -M_SENS_2, -M_ACK1, -M_ACK2} ,

			// M_BROADCAST
			{ M_BROADCAST } ,

			// M_SENS_1, M_SENS_2, M_ACK1, M_ACK2
			{ -M_SENS_1, -M_SENS_2, -M_ACK1, -M_ACK2} ,

			// M_ACT_1, M_SENS_2
			{ M_ACT_1, -M_SENS_2} ,

			// M_SENS_1, M_SENS_2
			{ -M_SENS_1, -M_SENS_2} ,

			// M_ALERT
      { -M_ALERT } ,
    };

    /* Store in external variable */
    memcpy(&sched_table, &sched_table_instance,
           2*TTW_NUMBER_ROUNDS*TTW_MAX_SLOTS_PER_ROUND);

  }
}

/*---------------------------------------------------------------------------
static uint16_t
get_round_period(uint16_t roundID)
{
  return 0;
}*/
/*---------------------------------------------------------------------------
static uint8_t
is_my_message(uint16_t message_id){
  // Implementation of the local look-up
  // might not be needed eventually (just look for message_id != 0

  return (message_id == node_id);
}
*/

/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* Dynamic update of the control
   * -> Update of the beacon information -- host only
   */

	/* Basic mode change: switch statically:
	 * start on mode 1 (normal)
	 * mode_change_counter == 10 -> switch to mode 2 (emergency)
	 * mode_change_counter == 20 -> switch to mode 1 (normal)
	 * mode_change_counter == 30 -> switch to mode 0 (normal + update)
	 */
	mode_change_counter++;
	if(mode_change_counter == 10) {

		// Switch to mode 2
		TTW_SET_BEACON_MODE(control, 2);
		// Set the SB
		TTW_SET_BEACON_SB(control);
		// Set the new round id
		TTW_SET_BEACON_ROUND(control, next_round_id);

	} else if(mode_change_counter == 20) {

			// Switch to mode 1
			TTW_SET_BEACON_MODE(control, 1);
			// Set the SB
			TTW_SET_BEACON_SB(control);
			// Set the new round id
			TTW_SET_BEACON_ROUND(control, next_round_id);

	} else if(mode_change_counter == 30) {

			// Switch to mode 0
			TTW_SET_BEACON_MODE(control, 0);
			// Set the SB
			TTW_SET_BEACON_SB(control);
			// Set the new round id
			TTW_SET_BEACON_ROUND(control, next_round_id);

	} else {
		// No mode-change for the moment, just set the round id
		// (already computed on the on_control_slot_post of the previous round)
		TTW_CLR_BEACON_SB(control);
		TTW_SET_BEACON_ROUND(control, next_round_id);
	}
}
/*---------------------------------------------------------------------------*/
static void
app_control_static_update(gmw_control_t* control)
{
  /* Static update of the control
   * -> Locally reconstruct the control information based on the beacon
   * -> Call by all nodes in the on_control_slot_post_callback()
   */

  /* Extract beacon information */
  current_round_id  = TTW_GET_BEACON_ROUND(control);
  next_mode_id      = TTW_GET_BEACON_MODE(control);
  switching_bit     = TTW_GET_BEACON_SB(control);

  /* Handle mode switches
   * -> change right away for now, which means current_mode_id always hold the latest
   * mode information */
  if(switching_bit) {
  	DEBUG_PRINT_WARNING("mode change!")
  	next_round_id = mode_array[next_mode_id].first_round_id;

  	/* Set the start of the new mode conservatively!
  	 * -> One must make sure that the previous round is finished by the time the new round is scheduled.
  	 * We do things (very) conservatively here, but at least it's safe
  	 * (all rounds are shorter than the round hyperperiod)
  	 */
  	control->schedule.period = round_array[next_round_id].start_time_offset
  															+ mode_array[current_mode_id].hyperperiod; 	// safety margin
  } else {
		/* Assuming no mode change, compute:
		 * - the next round ID
		 * - the corresponding round period
		 * */

		//TODO this needs to be written down VERY carefully. Might buggy right now...
		next_round_id = (current_round_id + 1)%TTW_NUMBER_ROUNDS;
		if((next_round_id == current_round_id) ||
			 (round_array[next_round_id].mode_id != next_mode_id) ||
			 (TTW_NUMBER_MODES == 1 && next_round_id < current_round_id)){
			/* Either:
			 * - There is only one round in the mode (cond. 1), or
			 * - The last round in the mode schedule is reached (cond. 2 and 3)
			 * Overwrite the next_round_id and compute the corresponding period
			 */
			next_round_id = mode_array[current_mode_id].first_round_id;
			control->schedule.period = round_array[next_round_id].start_time_offset
															- round_array[current_round_id].start_time_offset
															+ mode_array[current_mode_id].hyperperiod;
		} else {
			control->schedule.period = round_array[next_round_id].start_time_offset
															- round_array[current_round_id].start_time_offset;

		}
  }

  /* Load the current round schedule information */
  current_round_pt = &(round_array[current_round_id]);

  /* update the mode information */
  current_mode_id = next_mode_id;

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
  host_impl->on_slot_pre            = &on_slot_pre_callback;
  host_impl->on_slot_post           = &on_slot_post_callback;
  host_impl->on_round_finished      = &on_round_finished;

  /* load the source node implementation */
  src_impl->on_control_slot_post    = &src_on_control_slot_post_callback;
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
