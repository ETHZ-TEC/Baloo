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
 *         Re-implementation of the Crystal protocol using Baloo
 *         -> Adapted to have a number of parameters directly patchable
 *            in the binary (see the list in custom.h)
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "gpio.h"
#include "node-id.h"
#include "debug-print.h"

/* for stats */
#include "sys/energest.h"
#include "sys/dc-stat.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GRAZ VARIABLES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#include "my2c.h"
#include "testbed.h"
/* Generic configuration */
volatile config_t __attribute((section (".testbedConfigSection"))) cfg
#ifdef GRAZ
  ={{{3,{119,222,220,207,212,0,0,0},{202,0,0,0,0,0,0,0},8,0,0,5000,0,0}}};
#elif defined FLOCKLAB
  ={{{3,{3,10,4,6,7,17,33,24},{2,0,0,0,0,0,0,0},8,0,0,2000,0,0}}};
#endif /* GRAZ */
/* Custom configuration */
volatile custom_config_t __attribute((section (".customConfigSection")))
  custom_cfg = {5,2,31};
/*
{
  "n_tx_max":3,
  "n_empty_ta":2
}
*/
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GMW VARIABLES -----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static gmw_protocol_impl_t  host_impl;
static gmw_protocol_impl_t  src_impl;
static gmw_control_t        control;
static uint8_t              is_synced = 0;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- APP VARIABLES -----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef DEPCOMP
static const uint8_t sndtbl[] = {0} ;
#endif
static uint8_t is_source    = 0;
static uint8_t is_data_to_read = 0;
static uint8_t is_data_to_send = 0;
static uint8_t send_pkt_buffer[GMW_CONF_MAX_DATA_PKT_LEN] = {};
/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static void app_control_init(gmw_control_t* control);
static void app_control_update(gmw_control_t* control);
static uint8_t data_to_send(uint16_t node_id, uint16_t epoch);

/* Additional function required to use the 'static control' feature of Baloo*/
static void app_control_static_update(gmw_control_t* control);

/* Additional functions used to implement channel hopping */
static inline uint8_t get_num_channels();
static inline uint8_t get_channel_epoch(uint16_t epoch);
static inline uint8_t get_channel_epoch_ta(uint16_t epoch, uint8_t ta);

/*---------------------------------------------------------------------------*/
/*- CRYSTAL-SPECIFIC VARIABLES ----------------------------------------------*/
/*---------------------------------------------------------------------------*/

/* Tracking variables */
static uint16_t     epoch           = 0;
static uint8_t      sleep_order     = 0;
static uint8_t      n_ta            = 0;
static uint16_t     src_to_ack      = 0;
static uint16_t     seqn_to_ack     = 1;
static uint8_t      n_empty_ts      = 0;
static uint8_t      n_high_noise    = 0;
static uint8_t      n_noacks        = 0;
static uint8_t      app_has_data_to_send = 0;

/* stats */
//TODO: optimize state keeping
static uint16_t     last_rcvd_seqn[226]  = { 0 }; // 226: maximal node id
static uint16_t     received_packets    = 0;
static uint16_t     expected_packets    = 0;
static uint16_t     sent_ack            = 0;

//static uint16_t     RF_DC               = 0;
//static uint8_t      first_reset         = 0;
rtimer_ext_clock_t  stat_resettime      = 0;


/* Packet buffer */
#include "crystal-packet-types.h"

static crystal_data_struct    crystal_data;
static crystal_ack_struct     crystal_ack;

/* RF channels */
static uint8_t      rf_channel      = GMW_CONF_RF_TX_CHANNEL;      /* Initial RF channel */

#ifdef PLATFORM_SKY
/* static uint8_t      channel_array[] = {GMW_CONF_RF_TX_CHANNEL}; */

static uint8_t      channel_array[] = {11,12,13,14,15,
                                       16,17,18,19,20,
                                       21,22,23,24,26};   /*Available channels to hop */
#elif defined PLATFORM_DPP_CC430
/* static uint8_t      channel_array[] = {0,1,2,3,4,
                                       5,6,7,8,9,
                                       10};
channels 0 and 10 are used by other networks */
/* static uint8_t      channel_array[] = {3};*/
static uint8_t      channel_array[] = {1,2,3,4,
                                       5,6,7,8,9
                                       };  /* Available channels to hop */
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
#ifdef EEPROM_PIN
#define EEPROM_STARTED      PIN_SET(EEPROM_PIN)
#define EEPROM_STOPPED      PIN_CLR(EEPROM_PIN)
#define EEPROM_TOGGLE       PIN_XOR(EEPROM_PIN)
#else
#define EEPROM_STARTED
#define EEPROM_STOPPED
#define EEPROM_TOGGLE
#endif /* EEPROM_PIN */

#ifdef ACT_PIN
#define ACT_PIN_ON      PIN_SET(ACT_PIN)
#define ACT_PIN_OFF      PIN_CLR(ACT_PIN)
#else
#define ACT_PIN_ON
#define ACT_PIN_OFF
#endif /* ACT_PIN */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
PROCESS(app_process, "Application Task");
PROCESS(pre_process, "EEPROM API");
AUTOSTART_PROCESSES(&app_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- PREPROCESS --------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef GRAZ
  static void     my_eeprom_write      (uint8_t* msg);
  static uint8_t  my_eeprom_read       (uint8_t* msg_buffer);
#else
  #include "random.h"
  static uint16_t rand_draw;
#endif /* GRAZ */

PROCESS_THREAD(pre_process, ev, data)
{
  PROCESS_BEGIN();

#ifndef GRAZ
  random_init(node_id);
#endif /* GRAZ */

#ifdef GRAZ
  //the software my2c may be stuck for a long time
  //watchdog_stop();

  //enable and stop the my2c (otherwise it would block the bus)

  my2c_enable();
  my2c_stop();
#endif /* GRAZ */

  /* configure the event pin as desired */
  if(is_source){
    //configure pin to input with interrupt
    PIN_CFG_IN(EVENT_PIN);
    PIN_UNSEL(EVENT_PIN);
    PIN_IES_FALLING(EVENT_PIN);
    PIN_CLR(EVENT_PIN);
    PIN_INT_EN(EVENT_PIN);

    //wait until the pin is initially settled
    clock_delay(10000);
    while( (PIN_GET(EVENT_PIN)) != 0);
    /*while( (P2IN & BIT6) != 0);*/

  } else {
    //configure pin to output
    /*
     * Important: On Flocklab, EVENT_PIN is configured as an input by the observer.
     * Don't set it as output! Use another (output) pin.
     *
     * For now, we use rtimer timestamps instead.
     */
#ifdef GRAZ
    PIN_UNSEL(EVENT_PIN);
    PIN_CFG_OUT(EVENT_PIN);
    PIN_CLR(EVENT_PIN);
#endif /* GRAZ */
  }

  /* Reset is_data_to_read
   * -> This should not be necessary, but can't find another solution for now
   */
  is_data_to_read = 0;

  /* main loop of this application task */
  while(1) {

    /* the app task should not do anything until it is explicitly granted
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    if(is_source) {
#ifdef GRAZ
      /* Read data from EEPROM */
      if(is_data_to_read) {
        if(my_eeprom_read((uint8_t*)&send_pkt_buffer)) {
          DEBUG_PRINT_MSG_NOW("Pkt to send!");
          is_data_to_send = 1;
          is_data_to_read = 0;
        } else {
          DEBUG_PRINT_MSG_NOW("No pkt available yet...");
          is_data_to_send = 0;
          is_data_to_read = 0;
        }
      } else {
        DEBUG_PRINT_MSG_NOW("No pkt available yet...");
        is_data_to_send = 0;
        is_data_to_read = 0;
      }
#else /* not GRAZ */
      /* Or generate random payload */
      uint8_t i;
      for(i=0; i<CRYSTAL_PAYLOAD_LENGTH; i++) {
        rand_draw           = random_rand();
        send_pkt_buffer[i]  = (uint8_t)rand_draw;
      }
      is_data_to_send = 1;
      DEBUG_PRINT_INFO("Pkt to send!");
#endif /* GRAZ */
    }
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* ------------------------------------------- */
  /* --- Application-specific initialization --- */

  int i;

  /* Reset the rf channel to desired value */
  GMW_SET_RF_CHANNEL(rf_channel);
  epoch = 1;
  seqn_to_ack = 1; // seqn_to_ack == 0 is used to flag an error

  /* identify my role */
  if(node_id == HOST_ID) {
    printf("I'm the host!\n");
  } else {
    for(i=0; i<TB_NUMNODES; i++) {
      if(node_id == cfg.p[0].source_id[i]){
        is_source = 1;
        printf("I'm a source!\n");
        break;
      }
    }
    if(!is_source) {
      printf("I have no specific role...\n");
    }
  }

  printf("custom_cfg: %u %u", custom_cfg.n_tx_max, custom_cfg.n_empty_ta);

  /* ------------------------------------------- */
  /* MAKE SURE THE INITIALIZATION OF CONTROL VARIABLES IS DONE BEFORE CALLING gmw_init()
   * ------------------------------------------- */
  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  /* ------------------------------------------- */

  /* start the pre-process */
  process_start(&pre_process,NULL);

  /* start the GMW thread */
  gmw_start(&pre_process, &app_process, &host_impl, &src_impl);
  
  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    
    DEBUG_PRINT_INFO("Round ended.");

#if ENERGEST_CONF_ON && defined PLATFORM_SKY

    RF_DC = (energest_type_time(ENERGEST_TYPE_LISTEN)
            + energest_type_time(ENERGEST_TYPE_TRANSMIT)) * 10000 /
                (ENERGEST_GET_TOTAL_TIME() - stat_resettime);
    DEBUG_PRINT_INFO("Radio DC: %u.%02u", RF_DC/100, RF_DC%100);

    if(!first_reset) {

      /* Reset stats when you bootstrap the first time */
      energest_init();
      stat_resettime = ENERGEST_GET_TOTAL_TIME();

      first_reset = 1;
    }

#elif defined PLATFORM_DPP_CC430

    RF_DC = DCSTAT_RF_DC; //dummy use to avoid compiler errors
    DEBUG_PRINT_INFO("Radio DC: %u.%02u", RF_DC/100, RF_DC%100);
    if(!first_reset) {

      /* Reset stats when you bootstrap the first time */
      DCSTAT_RESET;
      first_reset = 1;
    }
#endif /* ENERGEST_CONF_ON */

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
  /* Update the static control */
  app_control_static_update(in_out_control);
  /* Save a local copy */
  control = *in_out_control;

  DEBUG_PRINT_INFO("control sent on channel %u", rf_channel);

  /* initialize tracking variables for the current round */
  n_empty_ts    = 0;
  n_high_noise  = 0;
  n_ta          = 0;
  sleep_order   = 0;

  return GMW_RUNNING;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
host_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
              uint8_t* out_len, uint8_t* out_payload,
              uint8_t is_initiator, uint8_t is_contention_slot)
{
  /* -------- T slot -------- */
  if(is_contention_slot) {

    if( epoch > CRYSTAL_START_EPOCH) {
      /* set the right channel based on the epoch number and TA counter */
      rf_channel = get_channel_epoch_ta(epoch, n_ta);
      GMW_SET_RF_CHANNEL(rf_channel);
    }

  /* -------- A slot -------- */
  } else {
    sleep_order =
        epoch >= N_FULL_EPOCHS && (
            (n_ta         >= CRYSTAL_MAX_TAS-1) ||
            (n_empty_ts   >= CRYSTAL_SINK_MAX_EMPTY_TS_DYNAMIC(n_ta)) ||
            (CRYSTAL_SINK_MAX_NOISY_TS
                && n_high_noise >= CRYSTAL_SINK_MAX_NOISY_TS)
            );


    /* Fill the ACK packet
     * The values of src_to_ack and seqn_to_ack
     * have already been updated in the slot_post_callback
     * */
    crystal_ack.epoch  = epoch;        // Send current epoch
    crystal_ack.n_ta   = n_ta;         // Current n_ta (or do we increment it already?
    crystal_ack.cmd    = sleep_order;  // Sleep command
    crystal_ack.src    = src_to_ack;   // ID of the acknowledged src node
    crystal_ack.seqn   = seqn_to_ack;  // seqn of the acknowledged packet

    DEBUG_PRINT_VERBOSE("send ack: %u %u %u %u %u",
                     epoch, n_ta, sleep_order, src_to_ack, seqn_to_ack);
    /* Pass the info payload to the middleware */
    memcpy(out_payload, &crystal_ack, CRYSTAL_ACK_PKT_LEN);
    *out_len = CRYSTAL_ACK_PKT_LEN;
    sent_ack++;
  }

  return GMW_EVT_NO_SKIP;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post_callback(uint8_t slot_index, uint16_t slot_assignee,
               uint8_t len, uint8_t* payload,
               uint8_t is_initiator, uint8_t is_contention_slot,
               gmw_pkt_event_t event)
{
  /* -------- T slot -------- */
  if(is_contention_slot) {
    /* some logging */

    /* Update tracking variables */
    if( event == GMW_EVT_PKT_OK ) {
      n_empty_ts = 0;
      n_high_noise = 0;

    } else if( event == GMW_EVT_PKT_CORRUPTED ) {
      n_empty_ts = 0;
      n_high_noise = 0;

    } else if( event == GMW_EVT_PKT_GARBAGE ) {
      n_empty_ts = 0;
      n_high_noise++;

    } else if( event == GMW_EVT_PKT_SILENCE ) {
      n_empty_ts++;
      n_high_noise = 0;

    } else if( event == GMW_EVT_PKT_MISSED ) {
      /* Don't do anything*/

    } else {
      DEBUG_PRINT_ERROR("Packet reception event is buggy.")
    }

    /* Prepare the ACK */
    if(event == GMW_EVT_PKT_OK) {
      memcpy(&crystal_data, payload, CRYSTAL_DATA_PKT_LEN); // copy received bytes in the crystal data packet
      src_to_ack    = crystal_data.src;
      seqn_to_ack   = crystal_data.seqn;

      /* Check is the payload is not empty
       * -> Bug from Glossy... to be solved */
      if(src_to_ack != 0) {

        /* Check if it is a newly received packet and log */
        if(last_rcvd_seqn[src_to_ack] != seqn_to_ack) {
          last_rcvd_seqn[src_to_ack] = seqn_to_ack;
          received_packets++;

          #ifdef GRAZ
              /* Write to EEPROM*/
              my_eeprom_write(crystal_data.payload);
          #endif /* GRAZ */
          DEBUG_PRINT_INFO("R: %x %x", crystal_data.payload[0], crystal_data.payload[1])
        }
      }

    } else {
      /* No packet properly received, send a nACK */
      src_to_ack    = CRYSTAL_NACK_NODE;
      seqn_to_ack   = CRYSTAL_NACK_SEQN;
    }

    return GMW_EVT_NO_REPEAT;

  /* -------- A slot -------- */
  } else {
    /* some logging */

    /* Increment n_ta counter */
    n_ta++;

    /* Check for the end of round */
    if(!sleep_order && n_ta < CRYSTAL_MAX_TAS) {
      /* We are not done yet, continue */
      return GMW_EVT_REPEAT_ROUND;

    } else {
      /* Stop the round */

      /* Debug logs */
      if(sleep_order) {
        DEBUG_PRINT_VERBOSE("Host stops the round - sleep order.");
        DEBUG_PRINT_VERBOSE("n_ta: %u ; n_empty_ts: %u ; n_high_noise %u",
                         n_ta,
                         n_empty_ts,
                         n_high_noise);

      } else {
        DEBUG_PRINT_VERBOSE("Host stops the round - Max TA reached")
      }

      return GMW_EVT_NO_REPEAT;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Network management after bootstrapping */
  if(epoch >= CRYSTAL_START_EPOCH) {

    /* Log the current PRR */
    expected_packets += CRYSTAL_NB_CONCURRENT_SENDER;

    DEBUG_PRINT_INFO("Received %u out of %u packets. PRR: %lu",
                     received_packets,
                     expected_packets,
                     (uint32_t) 10000LU*received_packets/expected_packets);

    /* Set the radio channel for the next control packet */
    rf_channel = get_channel_epoch(epoch + 1);
    GMW_SET_RF_CHANNEL(rf_channel);
  }

  /* Log number of ack sent (to gauge the impact of noise detect on the DC) */
  DEBUG_PRINT_INFO("Sent acks: %u", sent_ack);
  sent_ack = 0;

  /* Increment the local copy of the epoch counter */
  epoch++;

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

  DEBUG_PRINT_MSG_NOW("control post callback");

  if(sync_event == GMW_EVT_CONTROL_MISSED) {

    /* Notify the app layer, log, etc... */

    /* Opportunistically increment the epoch
     * -> Gives a chance to be on the right channel for this round */
    epoch++;
    DEBUG_PRINT_INFO("Control missed on channel: %u", rf_channel);

  } else { // Control packet successfully received

    //DEBUG_PRINT_INFO("Control received on channel: %u", rf_channel);

    /* The first time a control packet is received,
     * fill the static schedule to send to the middleware
     */
    if (!is_synced) {
        in_out_control->schedule = control.schedule;
        in_out_control->config = control.config;
        in_out_control->slot_config[0] = control.slot_config[0];
        in_out_control->slot_config[1] = control.slot_config[1];

        /* Mark that the middleware now has all the static control information*/
        is_synced = 1;
    }

    /* Extract the epoch counter value */
    epoch = (           in_out_control->user_bytes[0]       ) |
            ((uint16_t)(in_out_control->user_bytes[1]) << 8 );
  }

  /* Control only serves to update the time ref
   * Never suspends nor go back to bootstrap and hope for the best
   */

  /* initialize tracking variables for the round
   * (different from host: see which ones I really need) */
  n_empty_ts      = 0;
  n_noacks        = 0;
  n_high_noise    = 0;
  sleep_order     = 0;
  n_ta            = 0;

  /* Update the static control */
  app_control_static_update(in_out_control);

  /* Save a local copy of the control*/
  control = *in_out_control;

  /* look if there is data to be sent in this epoch
   * -> it is not clear yet how we want to implement this function.
   * It will highly depend on how we trigger the packet sending, eventually.*/
  app_has_data_to_send = data_to_send(node_id, epoch);
  DEBUG_PRINT_INFO("app_has_data_to_send = %u", app_has_data_to_send);

  return GMW_RUNNING;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
src_on_slot_pre_callback(uint8_t slot_index, uint16_t slot_assignee,
             uint8_t* out_len, uint8_t* out_payload,
             uint8_t is_initiator, uint8_t is_contention_slot)
{
  /* -------- T slot -------- */
  if(is_contention_slot) {

    if( epoch > CRYSTAL_START_EPOCH) {
      /* set the right channel based on the epoch number and TA counter */
      rf_channel = get_channel_epoch_ta(epoch, n_ta);
      GMW_SET_RF_CHANNEL(rf_channel);
    }

    /* if you are supposed to send, prepare your packet */
    if( app_has_data_to_send ) {
      crystal_data.src         = node_id;
      crystal_data.seqn        = seqn_to_ack;

      /* memcpy the src and seqn */
      memcpy(out_payload, &crystal_data, 4);
      /* memcpy the payload */
      memcpy(out_payload+2+2, &send_pkt_buffer, CRYSTAL_PAYLOAD_LENGTH);
      *out_len = CRYSTAL_DATA_PKT_LEN;

      DEBUG_PRINT_INFO("S: %x %x", send_pkt_buffer[0], send_pkt_buffer[1]);
    }
  }

  /* -------- A slot -------- */
  else {
    /* Nothing to do */
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
  /* -------- T slot -------- */
  if(is_contention_slot) {
    /* some logging */

    /* Update tracking variables */
    if ( event == GMW_EVT_PKT_OK ) {
      n_empty_ts = 0;
    }
    else if ( event == GMW_EVT_PKT_CORRUPTED ) {
      n_empty_ts = 0;
    }
    else if ( (event == GMW_EVT_PKT_GARBAGE) ||
              (event == GMW_EVT_PKT_MISSED )    ) {
      /* Don't touch n_empty_ts, as we are not sure what's going on:
       * - maybe there is just too much noise, but other nodes are still sending
       * - maybe no one for your network is sending anymore, but you can't be sure
       * In doubt: stay on */
    }
    else if ( event == GMW_EVT_PKT_SILENCE ) {
      n_empty_ts++;
    }
    else {
      DEBUG_PRINT_ERROR("Packet reception event is buggy.")
    }


    return GMW_EVT_NO_REPEAT;
  }

  /* -------- A slot -------- */
  else {
  /* some logging */

    /* Process received packet */
    if (event == GMW_EVT_PKT_OK){

      /* Update tracking variables */
      n_noacks = 0;
      n_high_noise = 0;

      // copy received bytes in the crystal ack packet
      memcpy(&crystal_ack, payload, CRYSTAL_ACK_PKT_LEN);

      /* Check is the payload is not empty
       * -> Bug from Glossy... to be solved */
      if(crystal_ack.seqn == 0) {
        /* Glossy bug, count packet as corrupted */
        n_noacks = 0;
        n_high_noise = 0;

      } else {

        /* Sanity checks */
        /* If one of these test fails, it is likely that the packet has been wrongly
         * identified as valid while it is in fact corrupted...
         */
        if( epoch != crystal_ack.epoch ) {
          DEBUG_PRINT_ERROR("Buggy: Received (%u) and local (%u) epoch values don't match.",
                            (crystal_ack.epoch),
                            epoch);
        }
        if( n_ta != crystal_ack.n_ta ) {
          DEBUG_PRINT_ERROR("Buggy: Received (%u) and local (%u) n_ta values don't match.",
                            crystal_ack.n_ta,
                            n_ta);
        }

        /* Save the sleep command from the host */
        sleep_order = crystal_ack.cmd;

        /* If we sent a packet, look at the ACK */
        if( app_has_data_to_send ) {
          if( node_id == crystal_ack.src ) {
            /* Host ACKed one of our packet */
            if( seqn_to_ack == crystal_ack.seqn ) {
              /* Our packet has been received and ACKed. Perfect. */
              //DEBUG_PRINT_INFO("Packet has been acked.");
              seqn_to_ack++;
              // some logging
              app_has_data_to_send = 0;

            }  else {
              DEBUG_PRINT_ERROR("Buggy: Host ACKed an old packet.");
              //DEBUG_PRINT_INFO("Packet to resent");
            }

          } else {
            /* Host ACKed someone else's packet. Re-send.*/
            //DEBUG_PRINT_INFO("Packet to resent");
            // some logging
          }
        }
      }

    } else if( event == GMW_EVT_PKT_CORRUPTED ) {
      /* Don't touch the n_noacks counter
       * -> don't make sense. I almost know for sure I received an ACK!..*/
      n_noacks = 0;
      n_high_noise = 0;

    } else if( event == GMW_EVT_PKT_GARBAGE ) {
      n_high_noise ++;
      if(n_high_noise > CRYSTAL_MAX_NOISY_AS) {
        n_noacks ++;
      }

    } else if( event == GMW_EVT_PKT_SILENCE ) {
      n_noacks++;
      n_high_noise = 0;

    } else {
      DEBUG_PRINT_ERROR("Packet reception is buggy.")
    }

    /* Increment n_ta counter */
    n_ta++;

    /* Evaluate the stop condition */
    if(sleep_order || (n_ta >= CRYSTAL_MAX_TAS) || // always stop when ordered or max is reached
        (epoch >= N_FULL_EPOCHS && (
            (  app_has_data_to_send  && (n_noacks >= CRYSTAL_MAX_MISSING_ACKS)) ||
            ((!app_has_data_to_send) && (n_noacks >= CRYSTAL_MAX_SILENT_TAS) && n_empty_ts >= CRYSTAL_MAX_SILENT_TAS)
          )
        )
      ) {

      /* Some debug logs
      if(sleep_order) {
        DEBUG_PRINT_INFO("Src stops the round - sleep order.");

      } else if(n_ta >= CRYSTAL_MAX_TAS) {
        DEBUG_PRINT_INFO("Src stops the round - Max TA reached");

      } else {
        DEBUG_PRINT_INFO("Src stops the round - Own decision");
        DEBUG_PRINT_INFO("n_noacks: %u ; n_empty_ts: %u ; n_high_noise %u",
                         n_noacks,
                         n_empty_ts,
                         n_noacks);
      }
      */

      if(app_has_data_to_send) {
        DEBUG_PRINT_INFO("App has unsent data when the round ends.");
        seqn_to_ack++;  // packet is lost
      }

      return GMW_EVT_NO_REPEAT;

    } else {
      return GMW_EVT_REPEAT_ROUND;
    }
  }
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  if(epoch >= CRYSTAL_START_EPOCH) {
    /* Set the radio channel for the next control packet */
    rf_channel = get_channel_epoch(epoch+1);
    GMW_SET_RF_CHANNEL(rf_channel);
  }
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

  gmw_schedule_t* sched = &control->schedule;

  /* Schedule */
  sched->slot[0]  = GMW_SLOT_CONTENTION;     /* Data slot: all nodes contend */
  sched->slot[1]  = HOST_ID;                 /* Ack  slot: host replies */
  sched->n_slots  = GMW_CONF_MAX_SLOTS;
  sched->time     = 0;
  sched->period   = CRYSTAL_PERIOD; //ms

  /* Config */
  GMW_CONTROL_SET_CONFIG(control);
  control->config.n_retransmissions = 0; /* Specified in the slot config */
  control->config.gap_time  = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
  control->config.slot_time = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA);

  /* Slot Config */
  GMW_CONTROL_SET_SLOT_CONFIG(control);
  control->slot_config[0].n_retransmissions = GMW_CONF_TX_CNT_DATA; /* T slot */
  control->slot_config[1].n_retransmissions = 4; /* A slot */

  /* User bytes */
  /* The user bytes are always sent by the host,
   * They are used by Crystal to send the epoch counter to all nodes
   */
  GMW_CONTROL_SET_USER_BYTES(control);
  control->user_bytes[0] = 0x00ff & epoch;  /* epoch counter (lower bits)  */
  control->user_bytes[1] = epoch >> 8;      /* epoch counter (higher bits) */

}
/*---------------------------------------------------------------------------*/
static void
app_control_update(gmw_control_t* control)
{
  /* Dynamic update of the control
   * New values that will be sent by the host at the beginning of the next round
   */

  /* Only user bytes are sent by the host */
  control->user_bytes[0] = 0x00ff & epoch;   /* epoch counter (lower bits)  */
  control->user_bytes[1] = epoch >> 8;       /* epoch counter (higher bits) */
}
/*---------------------------------------------------------------------------*/
static void
app_control_static_update(gmw_control_t* control)
{
  /* Static update of the control
   * Call by all nodes in the control slot post callback to set the static
   * schedule and config information, if any.
   */

  /* Only update the global time info. All the rest remains the same.*/
  control->schedule.time += control->schedule.period;
}
/*---------------------------------------------------------------------------*/

static uint8_t
data_to_send(uint16_t node_id, uint16_t epoch)
{

  DEBUG_PRINT_INFO("this should always print");

#ifdef DEPCOMP

  /* We don't need the sndtbl, send event-based */
  DEBUG_PRINT_INFO("is_data_to_send: %u", is_data_to_send);
  return is_data_to_send;

#else

#ifndef FLOCKLAB

  DEBUG_PRINT_INFO("FLOCKLAB undefined");
  /* to test on the desk */
  if(epoch >= CRYSTAL_START_EPOCH) {
    return 1;
  }
  return 0;
#endif /* FLOCKLAB */

  /* Check if a node has data to send in a given epoch */

  DEBUG_PRINT_INFO("this should not print");
  if(CRYSTAL_NB_CONCURRENT_SENDER > 0) {
    int i;
    int cur_idx;
    if (epoch >= CRYSTAL_START_EPOCH) {
      cur_idx = ((epoch - CRYSTAL_START_EPOCH) % CRYSTAL_ACTIVE_EPOCHS) * CRYSTAL_NB_CONCURRENT_SENDER;
      for (i=0; i<CRYSTAL_NB_CONCURRENT_SENDER; i++) {
        if (node_id == sndtbl[cur_idx + i]) {
          return 1;
        }
      }
    }
  }

  return 0;

#endif /* DEPCOMP */
}

/*---------------------------------------------------------------------------*/

static inline uint8_t get_num_channels() {
  return (sizeof(channel_array)/sizeof(channel_array[0]));
}

static inline uint8_t get_channel_epoch(uint16_t epoch) {
  return channel_array[(epoch*7) % get_num_channels()];
}

static inline uint8_t get_channel_epoch_ta(uint16_t epoch, uint8_t ta) {
  return channel_array[((7*epoch + ta + 1)*7)% get_num_channels()];
}
/*---------------------------------------------------------------------------*/


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
#ifdef GRAZ
static void
my_eeprom_write(uint8_t* msg)
{
  EEPROM_STARTED;
  DEBUG_PRINT_MSG_NOW("eeprom write");
  int i;
  //raise gpio pin to indicate write operation
  PIN_SET(EVENT_PIN);

  //start my2c communication
  my2c_start();
  //write address on the bus
  my2c_write(0x50<<1);
  //write memory address (2 bytes)
  my2c_write(cfg.p[0].msg_offsetH);
  my2c_write(cfg.p[0].msg_offsetL);

  //write messages up to the length in the struct
  for(i=0;i<CRYSTAL_PAYLOAD_LENGTH;i++){
      my2c_write(msg[i]);
  }
  //stop my2c communication
  my2c_stop();

  //lower gpio pin to indicate finished write
  PIN_CLR(EVENT_PIN);
  EEPROM_STOPPED;
}
/*---------------------------------------------------------------------------*/
static uint8_t
my_eeprom_read(uint8_t* msg_buffer)
{
  EEPROM_STARTED;
  //double-check if there is data to read
  if(!is_data_to_read) {
    //nothing to read, abort
    DEBUG_PRINT_MSG_NOW("Nothing to read");
    EEPROM_STOPPED;
    return 0;

  } else {
    int8_t i;
    //my2c start
    my2c_start();
    //write address on the bus
    my2c_write(0x50<<1);
    //write memory address (2 bytes)
    my2c_write(cfg.p[0].msg_offsetH);
    my2c_write(cfg.p[0].msg_offsetL);
    //disable the bus and wait a bit
    my2c_stop();
    clock_delay(100);
    my2c_start();
    //write address on the bus and set read bit
    my2c_write((0x50<<1)|1);
    //read back all the data, on the last byte indicate end
    for(i=0;i<(CRYSTAL_PAYLOAD_LENGTH);i++) {
      if(i==((CRYSTAL_PAYLOAD_LENGTH)-1)) {
        msg_buffer[i]=my2c_read(0);
      } else {
        msg_buffer[i]=my2c_read(1);
      }
    }
    //my2c stop
    my2c_stop();

    //raise EVENT_PIN
    PIN_SET(EVENT_PIN);
  }
  EEPROM_STOPPED;
  return 1;
}
#endif /* GRAZ */
/*---------------------------------------------------------------------------*/
#include "isr_compat.h"
// Interrupt service routine
ISR(PORT2, __eeprom_isr)
{
  EEPROM_TOGGLE;
  /* clear interrupt on event pin*/
  PIN_CLR_IFG(EVENT_PIN);
  is_data_to_read = 1;
  DEBUG_PRINT_INFO("Time:%llu, Epoch:%u",(GMW_TICKS_TO_US(GMW_RTIMER_NOW()))
                                        , epoch)
}
/*---------------------------------------------------------------------------*/
