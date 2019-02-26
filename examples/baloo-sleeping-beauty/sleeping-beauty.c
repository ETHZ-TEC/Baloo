/*
 * Copyright (c) 2018-2019, Swiss Federal Institute of Technology (ETH Zurich).
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
 *         Re-implementation of the Sleeping Beauty protocol using Baloo
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "node-id.h"
#include "debug-print.h"

/* for stats */
#include "sys/energest.h"
#include "sys/dc-stat.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;

// Needed for the static control copy
static uint8_t              is_synced;

/* for stats */
static uint16_t     RF_DC               = 0;
static uint8_t      first_reset         = 0;
rtimer_ext_clock_t  stat_resettime      = 0;
static uint16_t     received_pkt_cnt  = 0;
static uint16_t     total_pkt_cnt  = 0;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- SLEEPING-BEAUTY PROTOTYPES ----------------------------------------------*/
/*---------------------------------------------------------------------------*/

static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);

/* Additional function required to use the 'static control' feature of Baloo*/
static void app_control_static_update(gmw_control_t* control);

static uint64_t set_all_nodes_active(void);
static uint64_t select_active_nodes(void);
static uint8_t is_in_parent_list(uint8_t);
static uint8_t find_in_parent_list(uint8_t);
static uint8_t get_assignment(uint16_t);

/*---------------------------------------------------------------------------*/
/*- SLEEPING-BEAUTY VARIABLES -----------------------------------------------*/
/*---------------------------------------------------------------------------*/

#include "sleeping-beauty.h"

/* Packet buffer */

union {
  sb_req_struct     req;     /* Request  packet */
  sb_ack_struct     ack;     /* Ack      packet */
  sb_strobe_struct  strobe;  /* Strobe   packet */
  sb_data_struct    data;    /* Data     packet */
} sb_packet_buffer;

#define sb_req      ((sb_req_struct*)&sb_packet_buffer)
#define sb_ack      ((sb_ack_struct*)&sb_packet_buffer)
#define sb_strobe   ((sb_strobe_struct*)&sb_packet_buffer)
#define sb_data     ((sb_data_struct*)&sb_packet_buffer)

/* Tracking variables */

static uint8_t      sync_bit;
static uint8_t      strobe_bit;

static uint8_t      current_number_nodes;
static uint8_t      previous_number_nodes;
static uint8_t      current_rr_pairs;


static sb_topology_struct   sb_topology;
static etx_value_t          new_etx_value;
static uint8_t              topology_index;

static uint64_t     active_node_bitmap;
static uint64_t     potential_parent_bitmap;
static uint8_t      assigned_slot;
static uint8_t      currently_assigned_slot;


static uint8_t      is_active;
static uint8_t      in_bootstrap;
static uint8_t      superframe_counter;
static uint16_t     counter=0;      // dummy data payload
static uint8_t      received_request_counter;
static uint8_t      timeout_counter;
static uint8_t      last_control_missed;

static uint16_t     assignment_list[SB_MAX_NUMBER_NODES];
static const uint16_t static_source_list[NUM_SOURCES] = SOURCES_LIST;
/*---------------------------------------------------------------------------*/

#include "gpio.h"
#ifdef GMW_CONF_DEBUG_PIN
#define GMW_CONF_DEBUG_PIN_ON     PIN_SET(GMW_CONF_DEBUG_PIN)
#define GMW_CONF_DEBUG_PIN_OFF    PIN_CLR(GMW_CONF_DEBUG_PIN)
#else
#define GMW_CONF_DEBUG_PIN_ON
#define GMW_CONF_DEBUG_PIN_OFF
#endif


/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* ------------------------------------------- */
  /* --- Application-specific initialization --- */

  if (node_id == HOST_ID )
  {
    current_number_nodes = 1; // Only the host
    current_rr_pairs = SB_MAX_RR_PAIRS;

    /* Set the first round to be with strobing */
    sync_bit    = 1;
    strobe_bit  = 1;
    superframe_counter = SB_NUMBER_SUPERFRAME - 2;

    /* In bootstrapping, all nodes participates */
    in_bootstrap = 1;
    active_node_bitmap = set_all_nodes_active(); // all ones

    assigned_slot           = 0; // Host does not send data
    is_active               = 1; // Host always active

    timeout_counter = 0;
  }
  else {

    in_bootstrap            = 1;
    current_number_nodes    = 0; // Unknown
    current_rr_pairs        = 0; // Unknown;
    assigned_slot           = 0; // Src does not have a slot yet
    is_synced               = 0; // No control packet received thus far
    is_active               = 1; // Node active until set otherwise

    memset(&sb_topology, 0xff, sizeof(sb_topology_struct));
    potential_parent_bitmap = 0; // Until bootstrapping, we don't know who might be our parents

  }

  /* ------------------------------------------- */


  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* start the GMW thread */
  gmw_start(NULL, &app_process, &host_impl, &src_impl);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

#if ENERGEST_CONF_ON && defined PLATFORM_SKY

    RF_DC = (energest_type_time(ENERGEST_TYPE_LISTEN)
            + energest_type_time(ENERGEST_TYPE_TRANSMIT)) * 10000 /
                (ENERGEST_GET_TOTAL_TIME() - stat_resettime);
    DEBUG_PRINT_INFO("Radio DC: %u.%02u", RF_DC/100, RF_DC%100);

    if(!in_bootstrap) {

      /* Reset stats when you bootstrap the first time */
      energest_init();
      stat_resettime = ENERGEST_GET_TOTAL_TIME();

      first_reset = 1;
    }

#elif defined PLATFORM_DPP_CC430

    RF_DC = DCSTAT_RF_DC; //dummy use to avoid compiler errors
    DEBUG_PRINT_INFO("Radio DC: %u.%02u", RF_DC/100, RF_DC%100);
    if(!in_bootstrap) {

      /* Reset stats when you bootstrap the first time */
      DCSTAT_RESET;
      first_reset = 1;
    }
#endif /* ENERGEST_CONF_ON */

    if(node_id==HOST_ID) {
      DEBUG_PRINT_INFO("PRR: %lu RCVD:%u TOT:%u Missed: %u",
                       ((uint32_t)received_pkt_cnt*10000)/total_pkt_cnt,
                       received_pkt_cnt,
                       total_pkt_cnt,
                       total_pkt_cnt-received_pkt_cnt);
    }
    /*
    if(node_id != HOST_ID) {
      DEBUG_PRINT_INFO("min-max: %u - %u", sb_topology.min_etx, sb_topology.max_etx);
      DEBUG_PRINT_INFO("\n%u \t %u\n%u \t %u\n%u \t %u\n%u \t %u\n%u \t %u\n",
                       sb_topology.parent_info[0].assigned_slot,
                       sb_topology.parent_info[0].etx_value,
                       sb_topology.parent_info[1].assigned_slot,
                       sb_topology.parent_info[1].etx_value,
                       sb_topology.parent_info[2].assigned_slot,
                       sb_topology.parent_info[2].etx_value,
                       sb_topology.parent_info[3].assigned_slot,
                       sb_topology.parent_info[3].etx_value,
                       sb_topology.parent_info[4].assigned_slot,
                       sb_topology.parent_info[4].etx_value)
    }
    */

    /* poll the debug-print task, this will output
     * the prepared debug-print messages
     */
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
  /* save current_number_nodes at the beginning of the round */
  previous_number_nodes = current_number_nodes;

  /* Update the static control */
  app_control_static_update(in_out_control);
  /* Save a local copy */
  control = *in_out_control;


  /* initialize tracking variables for the current round */
  received_request_counter = 0;

  return GMW_RUNNING;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
host_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
              uint8_t* out_len, uint8_t* out_payload,
              uint8_t is_initiator, uint8_t is_contention_slot)
{
  if(strobe_bit) {
    /* Full round with RR pairs and strobes */

    if((slot_index < 2*current_rr_pairs)  &&
       (is_initiator)) {

      /*
       * Acq slot
       */

      // Send the ack that has been prepared at the end of the previous slot
      memcpy(out_payload, sb_ack, SB_ACK_PKT_LEN);
      *out_len = SB_ACK_PKT_LEN;

    } else if(is_initiator) {

      /*
       * Strobe slot
       */

      // Host is the sink: ETX is zero
      sb_strobe->etx_value = 0;
      memcpy(out_payload, sb_strobe, SB_STROBE_PKT_LEN);
      *out_len = SB_STROBE_PKT_LEN;
    }

  } else {
    uint8_t active = IS_ACTIVE_SLOT(slot_index);
    //DEBUG_PRINT_INFO("active %u", active);
    if(!active) {
      return GMW_EVT_SKIP_SLOT;
    }
  }

  // Host always participate is all active slots
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
               uint8_t len, uint8_t* payload,
               uint8_t is_initiator, uint8_t is_contention_slot,
               gmw_pkt_event_t event)
{
  if(strobe_bit) {
    /* Full round with RR pairs and strobes */

    if(slot_index < 2*current_rr_pairs) {
      if(!is_initiator) {

        /*
         * Req slot
         */

        if(len == 2) {

          /* A request has been received, prepare the ack */

          // Get source nodeID and fill the ack
          sb_ack->ack_src = ((payload[1] << 8) | payload[0]);

          // Check if the source node has already been assigned a slot
          currently_assigned_slot = get_assignment(sb_ack->ack_src);
          if((currently_assigned_slot == NOT_ASSIGNED) &&
             (current_number_nodes < SB_MAX_NUMBER_NODES) ){
            // Add to the list
            assignment_list[current_number_nodes-1] = sb_ack->ack_src;
            // Send assignment and increment the number of nodes
            sb_ack->assigned_slot = current_number_nodes++;
          } else if(currently_assigned_slot != NOT_ASSIGNED) {
            // Re-send the same assignment
            sb_ack->assigned_slot = currently_assigned_slot;
          } else {
            // Network is full. Send a non-ack.
            sb_ack->assigned_slot = 0;
            // Decrement counter to eventually exit bootstrap
            received_request_counter--;
          }

          /* Keep track of received requests */
          received_request_counter++;

        } else if(len == 0) {
          // No request sent or capture effect failed
          sb_ack->ack_src       = SB_NO_ACK_SRC;
          sb_ack->assigned_slot = 0;

          //TODO: if we use the noise detection, we can count the failed capture
          //effect as a request. Might be a good idea to speed up the bootstrap

        } else {
          DEBUG_PRINT_ERROR("Wrong packet length. Expect a REQ (len=2B), got %uB", len);
        }

        DEBUG_PRINT_INFO("Sent Ack(id, slot): %u %u",sb_ack->ack_src, sb_ack->assigned_slot);


      } else {

        /*
         * Ack slot
         */

        // Nothing to do

      }



    } else if(is_initiator) {

      /*
       * Strobe slot
       */

      // Nothing to do
    }

  }

  if(!strobe_bit || (slot_index >= 2*current_rr_pairs + previous_number_nodes)) {

    /*
     * Data slot
     */

    // Log received data packets
    if(event == GMW_EVT_PKT_OK) {
      memcpy(sb_data, payload, len);
      GMW_CONF_DEBUG_PIN_ON;
      DEBUG_PRINT_MSG_NOW("R: %u %u %u",sb_data->payload[0], sb_data->payload[1], sb_data->payload[2]);
      received_pkt_cnt++;
      GMW_CONF_DEBUG_PIN_OFF;
      /*
      if(strobe_bit){
        DEBUG_PRINT_INFO("P:%u: %u %u %u %u %u",
                       sb_data->payload[0],
                       sb_data->parents[0],
                       sb_data->parents[1],
                       sb_data->parents[2],
                       sb_data->parents[3],
                       sb_data->parents[4]);
      }*/
    }
    uint8_t active = IS_ACTIVE_SLOT(slot_index);
    //DEBUG_PRINT_INFO("active %u", active);
    if(active) {
      total_pkt_cnt++;
    }
  }

  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{

  if(in_bootstrap) {
    // Decide here if it is time to leave the bootstrap primitive

    /* If not all requests where used, reduce their number by one in the next round
    if(current_rr_pairs - received_request_counter) {
      current_rr_pairs = MAX(current_rr_pairs-1 , 1);
    }*/

    // Check for the end of the bootstrapping phase
    if(received_request_counter == 0) {
      timeout_counter++;
    } else {
      timeout_counter = 0;
    }

    if(timeout_counter >= SB_BOOT_TIMEOUT_COUNT) {
      in_bootstrap = 0;
      current_rr_pairs = 1;
      superframe_counter = (superframe_counter+1)%SB_NUMBER_SUPERFRAME;
      DEBUG_PRINT_INFO("Bootstrap phase ends.")
    }

  } else {
    superframe_counter = (superframe_counter+1)% SB_NUMBER_SUPERFRAME;
  }

  /* Update the control */
  app_control_update(&control);
  gmw_set_new_control(&control);
}
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post_callback(gmw_control_t* in_out_control,
                                  gmw_sync_event_t sync_event,
                                  gmw_pkt_event_t pkt_event)
{
  uint8_t i;

  if (sync_event == GMW_EVT_CONTROL_MISSED)
  {
    /* Notify the app layer, log, etc... */

    last_control_missed = 1;
    DEBUG_PRINT_MSG_NOW("Control missed");

    /* Default state update
     * - suspend execution if control is missed
     * - go to bootstrap after two consecutive missed controls
     * - run normally otherwise
     * */
    return GMW_DEFAULT;

  } else { // Control packet successfully received

    DEBUG_PRINT_VERBOSE("Control received");

    /* The first time a control packet is received,
     * fill the static schedule to send to the middleware
     */

    if (!is_synced) {
        uint8_t i;
        in_out_control->schedule = control.schedule;
        in_out_control->config = control.config;
        for (i=0; i<GMW_SCHED_N_SLOTS(&control.schedule); i++){
          in_out_control->slot_config[i] = control.slot_config[i];
        }

        /* Mark that the middleware now has all the static control information*/
        is_synced = 1;
    }

    /* Extract information from the received control */
    sync_bit    = (in_out_control->user_bytes[0] & SYNC_BIT_MASK);
    strobe_bit  = (in_out_control->user_bytes[0] & STROBE_BIT_MASK) >> 1;
    current_number_nodes = (in_out_control->user_bytes[0] >> 2);
    current_rr_pairs = (in_out_control->user_bytes[1]);

    /* Flag the end of bootstrapping */
    if( !strobe_bit && in_bootstrap ) {
      in_bootstrap = 0;
      DEBUG_PRINT_INFO("Bootstrap phase ends.");
    }

    //DEBUG_PRINT_INFO("current_number_nodes: %u", current_number_nodes);

    if(sync_bit || last_control_missed ) {
      /* Update the active node bitmap */
      active_node_bitmap = 0;
      for(i=2; i<GMW_CONF_CONTROL_USER_BYTES; i++) {
        active_node_bitmap |= ((uint64_t) in_out_control->user_bytes[i])  << (8*(i-2));

        //DEBUG_PRINT_INFO("%llx", active_node_bitmap);
        //DEBUG_PRINT_INFO("%x", in_out_control->user_bytes[i] );
      }
      //DEBUG_PRINT_INFO("%llx", active_node_bitmap);

      //DEBUG_PRINT_INFO("is_active pre: %u", is_active)
      /* Check if we are active or not */
      if(assigned_slot != 0) {
        //DEBUG_PRINT_INFO("%u", assigned_slot );
        is_active = (active_node_bitmap & ((uint64_t)1 << (assigned_slot-1))) >> (assigned_slot-1);
      }
      //DEBUG_PRINT_INFO("is_active post: %u", is_active)
      last_control_missed = 0;
    }

    /* Update the static control */
    app_control_static_update(in_out_control);
    /* Save a local copy */
    control = *in_out_control;

    if(is_active) {
      return GMW_RUNNING;
    } else {
      // Suspend execution and resume before the last superframe
      DEBUG_PRINT_INFO("Set as inactive, goes to sleep.")
      return GMW_SUSPENDED;
    }
  }
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
src_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
             uint8_t* out_len, uint8_t* out_payload,
             uint8_t is_initiator, uint8_t is_contention_slot)
{
  if(strobe_bit) {
    /* Full round with RR pairs and strobes */

    if(is_contention_slot) {

      /*
       * REQ slots
       */

      if(assigned_slot == 0) {
        /* No slot assigned yet, request one */
        sb_req->req_src = node_id;
        memcpy(out_payload, sb_req, SB_REQ_PKT_LEN);
        *out_len = SB_REQ_PKT_LEN;
      }

    } else if((slot_index >= 2*current_rr_pairs) &&
              (slot_index < (2*current_rr_pairs + current_number_nodes))) {

      /*
       * Strobe slots
       */

      if(is_initiator) {
        /* my strobe slot */
        sb_strobe->etx_value = sb_topology.min_etx;
        memcpy(out_payload, sb_strobe, SB_STROBE_PKT_LEN);
        *out_len = SB_STROBE_PKT_LEN;

      } else if(!in_bootstrap) {
        uint8_t is_parent = IS_POTENTIAL_PARENT(slot_index);
        if(!is_parent) {
          /* strobe from a node not being of our potential parents, skip */
          return GMW_EVT_SKIP_SLOT;
        }
      }

    } else {

      /*
       * Data slots
       */

      if(is_initiator) {
        /* my data slot */
        sb_data->payload[0] = (uint8_t) ++counter;
        sb_data->payload[1] = (counter >> 8);
        sb_data->payload[2] = (uint8_t) node_id;
        memcpy(out_payload, sb_data, SB_PAYLOAD_LENGTH);
        uint8_t i;
        for(i=0; i<SB_NUMBER_PARENTS; i++) {
          out_payload[SB_PAYLOAD_LENGTH + i] = sb_topology.parent_info[i].assigned_slot;
        }
        *out_len = SB_PAYLOAD_LENGTH + SB_NUMBER_PARENTS;
        DEBUG_PRINT_INFO("S: %u %u %u", sb_data->payload[0], sb_data->payload[1], sb_data->payload[2]);
      }
    }

  } else {

    /* Normal round, with data slots only */

    if(is_initiator) {
      /* my data slot */
      sb_data->payload[0] = (uint8_t) ++counter;
      sb_data->payload[1] = (counter >> 8);
      sb_data->payload[2] = (uint8_t) node_id;
      memcpy(out_payload, sb_data, SB_PAYLOAD_LENGTH);
      *out_len = SB_PAYLOAD_LENGTH;
      DEBUG_PRINT_INFO("S: %u %u %u", sb_data->payload[0], sb_data->payload[1], sb_data->payload[2]);
    /*} else if(!(IS_ACTIVE_SLOT(slot_index))) {
       Data slot of an inactive node, skip
      DEBUG_PRINT_INFO("test works")
      return GMW_EVT_SKIP_SLOT;*/
    } else {
      uint8_t active = IS_ACTIVE_SLOT(slot_index);
      //DEBUG_PRINT_INFO("active %u", active);
      if(!active) {
        return GMW_EVT_SKIP_SLOT;
      }
    }
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
  if(in_bootstrap) {

    /* register all potential parents */
    if((slot_index >= 2*current_rr_pairs) &&
       (slot_index <  2*current_rr_pairs + current_number_nodes) &&
       (event == GMW_EVT_PKT_OK)) {
      /* A strobe was received, save the initiator as a potential parent */
      potential_parent_bitmap |= (1 << (slot_index - 2*current_rr_pairs));
    }
  }

  if(strobe_bit) {
    /* Full round with RR pairs and strobes */

    if((slot_index < 2*current_rr_pairs)  &&
       (!is_contention_slot)) {

      /*
       * Acq slot
       */

      if(event == GMW_EVT_PKT_OK) {
        //TODO think about the danger of empty payload
        if(len != SB_ACK_PKT_LEN) {
          DEBUG_PRINT_ERROR("Wrong packet length. Expect a ACK (len=%uB), got %uB", SB_ACK_PKT_LEN, len);
        } else {
          memcpy(sb_ack, payload, SB_ACK_PKT_LEN);
          if(sb_ack->ack_src == node_id) {
            /* I have been acked, store the slot assignment */
            assigned_slot = sb_ack->assigned_slot;
            if( assigned_slot == 0 ) {
              DEBUG_PRINT_INFO("Network is full.");
            }
          }
        }
      }

    } else if((slot_index >= 2*current_rr_pairs) &&
              (slot_index <  2*current_rr_pairs + current_number_nodes) &&
              (in_bootstrap || (IS_POTENTIAL_PARENT(slot_index)) )) {

      /*
       * Strobe slots
       */

      if(event == GMW_EVT_PKT_OK) {
        //TODO think about the danger of empty payload
        /* Store received ETX value */
        if(len != SB_STROBE_PKT_LEN) {
          DEBUG_PRINT_ERROR("Wrong packet length. Expect a STROBE (len=%uB), got %uB", SB_STROBE_PKT_LEN, len);
        } else {
          // One should be careful about overflows of the ETX values,
          // but it should not be a problem unless the network becomes __really__ big

          /* Compute received ETX value */
          new_etx_value =  ((SB_NUMBER_STROBES * 100) / GMW_GET_N_RX_PRIM2())      // one-hop ETX value
                          + ((payload[1] << 8) | payload[0]    );   // neighbor's ETX value

          /* If necessary, update the topology info */
          topology_index = is_in_parent_list(slot_index);

          if((topology_index == NOT_IN_LIST) &&
             (topology_index != MYSELF) &&
             (new_etx_value  <  sb_topology.max_etx)) {
            /* parent is not yet in list, find worst one and exchange */
            topology_index = find_in_parent_list(0);
            sb_topology.parent_info[topology_index].etx_value     = new_etx_value;
            sb_topology.parent_info[topology_index].assigned_slot = slot_index - 2*current_rr_pairs;

          } else if(topology_index != NOT_IN_LIST) {
            /* parent already in the short list, update its ETX value */
            sb_topology.parent_info[topology_index].etx_value = new_etx_value;
          }

          /* Update local max_etx */
          topology_index = find_in_parent_list(0);
          sb_topology.max_etx = sb_topology.parent_info[topology_index].etx_value;

          /* Update local min_etx */
          topology_index = find_in_parent_list(1);
          sb_topology.min_etx = sb_topology.parent_info[topology_index].etx_value;

        }
      }
    }
  }
  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Nothing to do
  int8_t i;
  for(i=0;i<4;i++) {
    DEBUG_PRINT_INFO("%u %u %u %u", control.schedule.slot[0+i*4],
                     control.schedule.slot[1+i*4],
                     control.schedule.slot[2+i*4],
                     control.schedule.slot[3+i*4]);
  }*/
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  /* Nothing to do */
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  /**
   *  Initialization of the application-side control structure
   * - This must always be done of the host node (HOST_ID == node_id)
   * - This must be done for all nodes if
   *    the middleware uses a 'static' configuration, ie,
   *    GMW_CONF_USE_STATIC_SCHED   and/or
   *    GMW_CONF_USE_STATIC_CONFIG  are set to true.
   */

  /*
   * Schedule
   */

  control->schedule.time    = 0;
  control->schedule.period  = SB_IPI; //ms
  control->schedule.n_slots = MAX( 0 ,                    // for source nodes, which start with no info
                              2*current_rr_pairs          // RR pairs
                            + current_number_nodes        // strobes from all nodes
                            + current_number_nodes - 1 ); // data from all nodes but the host

  /* RR slots */
  uint8_t i;
  for (i=0; i < current_rr_pairs; i++){
    control->schedule.slot[2*i]   = GMW_SLOT_CONTENTION;    /* All nodes can request to join */
    control->schedule.slot[2*i+1] = HOST_ID;                /* Ack from the host */
  }

  /* Strobe slots */
  if (assigned_slot != 0 || node_id == HOST_ID){
    control->schedule.slot[2*current_rr_pairs
                           + assigned_slot] = node_id;  /* my strobe slot */
  }

  /* Data slots*/
  if (assigned_slot != 0){
    // if assigned_slot != 0, we have data to send
    control->schedule.slot[2*current_rr_pairs
                         + current_number_nodes
                         + assigned_slot - 1] = node_id; /* my data slot*/
  }

  /*
   * Config
   */

  GMW_CONTROL_SET_CONFIG(control);

  control->config.n_retransmissions = SB_NUMBER_RETRANSMISSONS_DATA;
  control->config.gap_time = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
  control->config.slot_time = GMW_US_TO_SLOT_TIME(SB_DATA_PKT_NORM_T);

  /*
   * Slot Config
   */

  GMW_CONTROL_SET_SLOT_CONFIG(control);

  /* RR slots */
  for (i=0; i < current_rr_pairs; i++){
    /* REQ slots */
    control->slot_config[2*i  ].n_retransmissions = SB_NUMBER_RETRANSMISSONS_REQ;
    // slot_time is already defined by GMW_CONF_T_CONT
    /* ACQ slots */
    control->slot_config[2*i+1].n_retransmissions = SB_NUMBER_RETRANSMISSONS_ACQ;
    control->slot_config[2*i+1].primitive       = 0; // glossy primitive
    control->slot_config[2*i+1].slot_time_select  = SB_ACK_SLOT_TIME_SELECT;
  }

  /* Strobe slots */
  for (i=2*current_rr_pairs ;
        i < (2*current_rr_pairs + current_number_nodes) ;
        i++){
    control->slot_config[i].n_retransmissions = SB_NUMBER_STROBES;
    control->slot_config[i].primitive       = 2; // strobe primitive
    control->slot_config[i].slot_time_select  = SB_STROBE_SLOT_TIME_SELECT;
  }

  /* Data slots */
  for (i=2*current_rr_pairs + current_number_nodes ;
        i < 2*current_rr_pairs
            + current_number_nodes
            + current_number_nodes - 1 ;
        i++){
    control->slot_config[i].n_retransmissions = SB_NUMBER_RETRANSMISSONS_DATA;
    control->slot_config[i].primitive       = 0; // glossy primitive
    control->slot_config[i].slot_time_select  = SB_FULL_DATA_SLOT_TIME_SELECT;
  }

  /*
   * User bytes
   */

  GMW_CONTROL_SET_USER_BYTES(control);

  /* Set the current size of the bit-map and sync and strobe bits */
  control->user_bytes[0] = ( (sync_bit)
                             | (strobe_bit << 1)
                             | (current_number_nodes << 2));
  control->user_bytes[1] = current_rr_pairs;
  /* Fill the bit-map */
  for (i=2; i<GMW_CONF_CONTROL_USER_BYTES; i++){
    control->user_bytes[i] = (active_node_bitmap >> (8*(i-2)));
  }

}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* Dynamic update of the control
   * New values that will be sent by the host at the beginning of the next round
   */
  uint8_t i;

  if(superframe_counter == 0) {
    sync_bit    = 0;
    strobe_bit  = 0;

  } else if(superframe_counter == (SB_NUMBER_SUPERFRAME-2)) {
    sync_bit    = 1;
    strobe_bit  = 1;
    active_node_bitmap = set_all_nodes_active();
    /* Update the bit-map */
    for(i=2; i<GMW_CONF_CONTROL_USER_BYTES; i++) {
      control->user_bytes[i] = (active_node_bitmap >> (8*(i-2)));
      //DEBUG_PRINT_INFO("%x", control->user_bytes[i]);
    }

  } else if(superframe_counter == SB_NUMBER_SUPERFRAME-1) {
    sync_bit    = 1;
    strobe_bit  = 0;
    active_node_bitmap = select_active_nodes();
    /* Update the bit-map */
    for(i=2; i<GMW_CONF_CONTROL_USER_BYTES; i++) {
      control->user_bytes[i] = (active_node_bitmap >> (8*(i-2)));
      //DEBUG_PRINT_INFO("%x", control->user_bytes[i]);
    }
  }

  /* Update the current size of the bit-map and sync and strobe bits */
  control->user_bytes[0] = ( (sync_bit)
                             | (strobe_bit << 1)
                             | (current_number_nodes << 2));
  control->user_bytes[1] = current_rr_pairs;


}
/*---------------------------------------------------------------------------*/
static void
app_control_static_update(gmw_control_t* control)
{
  /* Static update of the control
   * Call by all nodes in the control slot post callback to set the static
   * schedule and config information, if any.
   */

  /* Update the schedule and config whenever necessary */

  uint8_t i;

  if(!is_active && sync_bit && !strobe_bit) {
    /* Node about to go to sleep. Adjust the period to wake-up later.
     * Do so only if you learnt you were inactive in the 'correct' round.
     * This avoids a node wrongly scheduling wake-up intervals after going back to bootstrap. */
    control->schedule.period = SB_IPI * (SB_NUMBER_SUPERFRAME-1);
  } else {
    /* Node active, possibly dormant before. */
    control->schedule.period = SB_IPI;
  }

  if (strobe_bit){

    /* Update slot config and schedule */

    /*
     * Schedule
     */

    control->schedule.n_slots = 2*current_rr_pairs          // RR pairs
                              + current_number_nodes        // strobes from all nodes
                              + current_number_nodes - 1 ;  // data from all nodes but the host

    // n_slot has changed, re-set the flags for the config and user-bytes
    GMW_CONTROL_SET_CONFIG(control);
    GMW_CONTROL_SET_SLOT_CONFIG(control);
    GMW_CONTROL_SET_USER_BYTES(control);

    /* Clean the schedule */
    //memset(&(control->schedule.slot[0]), 0, GMW_CONF_MAX_SLOTS);
    memset(control->schedule.slot, 0, GMW_CONF_MAX_SLOTS);

    /* RR slots */
    for(i=0; i < current_rr_pairs; i++) {
      control->schedule.slot[2*i]   = GMW_SLOT_CONTENTION;    /* All nodes can request to join */
      control->schedule.slot[2*i+1] = HOST_ID;                /* Ack from the host */
    }

    /* Strobe slots */
    if(assigned_slot != 0 || node_id == HOST_ID) {
      control->schedule.slot[2*current_rr_pairs
                             + assigned_slot] = node_id;  /* my strobe slot */
    }

    /* Data slots*/
    if(assigned_slot != 0) {
      // if assigned_slot != 0, we have data to send
    control->schedule.slot[2*current_rr_pairs
                           + current_number_nodes
                           + assigned_slot - 1] = node_id; /* my data slot*/
    }

    /*
     * Slot Config
     */

    /* RR slots */
    for(i=0; i < current_rr_pairs; i++) {
      /* REQ slots */
      control->slot_config[2*i  ].n_retransmissions = SB_NUMBER_RETRANSMISSONS_REQ;
      /* ACQ slots */
      control->slot_config[2*i+1].n_retransmissions = SB_NUMBER_RETRANSMISSONS_ACQ;
      control->slot_config[2*i+1].primitive       = 0; // glossy primitive
      control->slot_config[2*i+1].slot_time_select  = SB_ACK_SLOT_TIME_SELECT;
    }

    /* Strobe slots */
    for(i=2*current_rr_pairs ;
          i < (2*current_rr_pairs + current_number_nodes) ;
          i++) {
      control->slot_config[i].n_retransmissions = SB_NUMBER_STROBES;
      control->slot_config[i].primitive       = 2; // strobe primitive
      control->slot_config[i].slot_time_select  = SB_STROBE_SLOT_TIME_SELECT;
    }

    /* Data slots */
    for(i=2*current_rr_pairs + current_number_nodes ;
          i < 2*current_rr_pairs
              + current_number_nodes
              + current_number_nodes - 1 ;
          i++) {
      control->slot_config[i].n_retransmissions = SB_NUMBER_RETRANSMISSONS_DATA;
      control->slot_config[i].primitive       = 0; // glossy primitive
      control->slot_config[i].slot_time_select  = SB_FULL_DATA_SLOT_TIME_SELECT;
    }

  } else {

    /* Clean the schedule */
    memset(control->schedule.slot, 0, GMW_CONF_MAX_SLOTS);
    //memset(&(control->schedule.slot[0]), 0, GMW_CONF_MAX_SLOTS);

    /* Data slots*/
    control->schedule.slot[assigned_slot - 1] = node_id; /* my data slot*/
    control->schedule.n_slots = current_number_nodes -1 ;
    control->config.slot_time = GMW_US_TO_SLOT_TIME(SB_DATA_PKT_NORM_T);

    GMW_CONTROL_SET_CONFIG(control);
    GMW_CONTROL_SET_USER_BYTES(control);

  }
}

/*---------------------------------------------------------------------------*/




static uint64_t
set_all_nodes_active(void)
{
  return 0xffffffffffffffff;
}

static uint64_t select_active_nodes(void)
{
  /* Arbitrary policy: put one out of two nodes to sleep
   * Note: In practice, one would use an algorithm that accounts for the
   * node identities and the reported parents and ETX values.
   */
#ifdef NUM_SOURCES_CASE
  uint8_t i;
  uint64_t assignment;
  uint64_t new_active_nodes_map = 0;

  for(i=0; i < NUM_SOURCES_CASE; i++) {
    assignment = get_assignment(static_source_list[i]);
    new_active_nodes_map |= (((uint64_t) 1) << (assignment-1));
  }

  //DEBUG_PRINT_INFO("new_active_nodes_map: %u", (uint16_t) new_active_nodes_map);
  return new_active_nodes_map;
#endif

  return 0xaaaaaaaaaaaaaaaa;
}

static uint8_t is_in_parent_list(uint8_t current_slot_index)
{
  uint8_t parent_slot_assigned = current_slot_index - 2*current_rr_pairs;
  uint8_t i;
  if(parent_slot_assigned == assigned_slot){
    /* we have received our own strobes (although that should not happen...) */
    return MYSELF;
  }
  for(i=0 ; i < SB_NUMBER_PARENTS ; i++) {
    if(sb_topology.parent_info[i].assigned_slot == parent_slot_assigned) {
      /* parent is already in list, returns the index from the parent list */
      return i;
    }
  }
  /* parent not found in list */
  return NOT_IN_LIST;
}

static uint8_t find_in_parent_list(uint8_t direction)
{
  /* direction = 0 -> find worst parent
   * direction = 1 -> find best parent
   */

  uint8_t       current_index   = 0xff;
  etx_value_t   current_etx     = (0xffff)*direction;
  uint8_t i;

  for(i=0 ; i < SB_NUMBER_PARENTS ; i++) {
    if(direction) {
      // find best parent
      if(sb_topology.parent_info[i].etx_value < current_etx) {
        current_etx     = sb_topology.parent_info[i].etx_value;
        current_index   = i;
      }

    } else {
      // find worst parent
      if(sb_topology.parent_info[i].etx_value > current_etx) {
        current_etx     = sb_topology.parent_info[i].etx_value;
        current_index   = i;
      }
    }
  }
  return current_index;
}

static uint8_t get_assignment(uint16_t node_id)
{
  uint8_t i;
  for (i=0; i < current_number_nodes-1; i++){
    if(assignment_list[i] == node_id){
      return (i+1);
    }
  }
  return NOT_ASSIGNED;
}


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
