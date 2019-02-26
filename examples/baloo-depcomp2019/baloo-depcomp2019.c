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
 *         An example protocol using Baloo, compatible with the collection
 *         scenarios of the 2019 Dependability Competition
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
#include "node-id.h"
#include "gpio.h"
#include "watchdog.h"
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*- GRAZ VARIABLES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
#include "my2c.h"
#include "testbed.h"
volatile config_t __attribute((section (".testbedConfigSection"))) cfg
={{{1,{16,10,20,07,40,0,0,0},{2,0,0,0,0,0,0,0},24,0,0,1000,500,30000}}};
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
static uint8_t is_synced    = 0;
static uint8_t is_source    = 0;
static uint8_t is_data_to_read = 0;
static uint8_t is_data_to_send = 0;
static uint8_t send_pkt_buffer[GMW_CONF_MAX_DATA_PKT_LEN] = {};
/*---------------------------------------------------------------------------*/
/*- APP PROTOTYPES ----------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
static void app_control_init  (gmw_control_t* control);
//static void app_control_update(gmw_control_t* control);
//static void app_control_static_update(gmw_control_t* control);
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
#endif

  /* configure the event pin as desired */
  if(is_source){
    //configure pin to input with interrupt
    P2DIR &= ~BV(EVENT_PIN);
    P2SEL &= ~BV(EVENT_PIN);
    //P2IES&=~BV(EVENT_PIN);
    P2IES |=  BV(EVENT_PIN);
    P2IFG &= ~BV(EVENT_PIN);
    P2IE  |=  BV(EVENT_PIN);

    //wait until the pin is initially settled
    clock_delay(10000);
    while( (P2IN & BIT6) != 0);

  } else {
    //configure pin to output
    P2SEL &= ~BV(EVENT_PIN);
    P2DIR |=  BV(EVENT_PIN);
    P2OUT &= ~BV(EVENT_PIN);
  }



  /* main loop of this application task */
  while(1) {

    /* the app task should not do anything until it is explicitly granted
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

    if(is_source) {
#ifdef GRAZ
      /* Read data from EEPROM */
      if(is_data_to_read) {
        uint64_t t_now = GMW_RTIMER_NOW();
        if(my_eeprom_read((uint8_t*)&send_pkt_buffer)) {
          DEBUG_PRINT_INFO("Read:%llu", GMW_TICKS_TO_US(GMW_RTIMER_NOW() - t_now));
          //DEBUG_PRINT_MSG_NOW("Pkt to send!");
          is_data_to_send = 1;
          is_data_to_read = 0;
        } else {
          //DEBUG_PRINT_MSG_NOW("No pkt available yet...");
          is_data_to_send = 0;
        }
      }
#else
      /*... for now, we generate random payload */
      uint8_t i;
      for(i=0; i<MSG_LENGTH; i++) {
        rand_draw           = random_rand();
        send_pkt_buffer[i]  = (uint8_t)rand_draw;
        is_data_to_send = 1;
      }
#endif /* GRAZ */

    } else {
      //DEBUG_PRINT_MSG_NOW("Nothing to do before the round.");
    }

  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(app_process, ev, data) 
{ 
  PROCESS_BEGIN();

  /* --- Application-specific initialization --- */
  is_synced = 0;

  //Romain: not needed. Done by default in platform_init_stage_two()
  //nodeid_restore();

  /* identify my role */
  if(node_id == HOST_ID) {
    printf("I'm the host!\n");
  } else {
    int i;
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

  /* initialization of the application structures */
  gmw_init(&host_impl, &src_impl, &control);

  //print_testbed_config((config_t*)(&cfg));

  /* start the pre-process */
  process_start(&pre_process,NULL);

  /* start the GMW thread */
  gmw_start(&pre_process, &app_process, &host_impl, &src_impl);

  /* main loop of this application task */
  while(1) {
    /* the app task should not do anything until it is explicitly granted 
     * permission (by receiving a poll event) by the GMW task */
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);

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
  return GMW_EVT_REPEAT_DEFAULT;
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
  if(!is_initiator && event == GMW_EVT_PKT_OK && len) {
#ifdef GRAZ
    /* Write to EEPROM*/
    uint64_t t_now = GMW_RTIMER_NOW();
    my_eeprom_write(payload);
    DEBUG_PRINT_INFO("Write:%llu", GMW_TICKS_TO_US(GMW_RTIMER_NOW() - t_now));
    //DEBUG_PRINT_INFO("Wrote pkt from node %u", slot_assignee);
#else
    /* Write to serial */
    //DEBUG_PRINT_INFO("Rcv %uB, first: %u", len, payload[0]);
#endif /* GRAZ */
  }
  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Nothing to do */
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
  }
  is_synced = 1;
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

    if(is_data_to_send) {
      memcpy(out_payload, send_pkt_buffer, MSG_LENGTH);
      *out_len = MSG_LENGTH;
      //DEBUG_PRINT_INFO("Sending %uB, first: %u", *out_len, out_payload[0]);
      /* Mark that the data has been sent */
      is_data_to_send = 0;

    } else {
      /* Nothing to send, skip the slot */
      *out_len = 0;
      return GMW_EVT_SKIP_SLOT;
    }
  }

  return GMW_EVT_REPEAT_DEFAULT;
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
  return GMW_EVT_NO_REPEAT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_processes)
{
  /* Nothing to do */
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  return 0;
}
/*---------------------------------------------------------------------------*/
static void
app_control_init(gmw_control_t* control)
{
  /* we know that only p[0] is used */
  // defined destination as the GMW host

  // fill the schedule and number of source nodes
  control->schedule.n_slots = 0;
  int i;
  for(i=0; i<8; i++) {
    if(cfg.p[0].source_id[i]) {
      control->schedule.slot[i] = cfg.p[0].source_id[i];
      control->schedule.n_slots++;
    }
  }
  control->schedule.period = PERIOD_S; // convert to seconds

  GMW_CONTROL_SET_CONFIG(control);

  control->config.gap_time = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
  control->config.slot_time = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA); // set random value for now
  control->config.n_retransmissions = GMW_CONF_TX_CNT_DATA;
  //control->config.max_packet_length = pkt_len;
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
#ifdef GRAZ
static void
my_eeprom_write(uint8_t* msg)
{
  EEPROM_STARTED;
  DEBUG_PRINT_MSG_NOW("eeprom write");
  int i;
  //raise gpio pin to indicate write operation
  P2OUT |= BV(EVENT_PIN);

  //start my2c communication
  my2c_start();
  //write address on the bus
  my2c_write(0x50<<1);
  //write memory address (2 bytes)
  my2c_write(cfg.p[0].msg_offsetH);
  my2c_write(cfg.p[0].msg_offsetL);

  //write messages up to the length in the struct
  for(i=0;i<MSG_LENGTH;i++){
      my2c_write(msg[i]);
  }
  //stop my2c communication
  my2c_stop();

  //lower gpio pin to indicate finished write
  P2OUT &= ~BV(EVENT_PIN);
  EEPROM_STOPPED;
}
/*---------------------------------------------------------------------------*/
static uint8_t
my_eeprom_read(uint8_t* msg_buffer)
{
  EEPROM_STARTED;
  //DEBUG_PRINT_MSG_NOW("eeprom read, pin: %u", P2IN & BV(EVENT_PIN));
  //check if EVENT_PIN is high
  if(P2IN & BV(EVENT_PIN)) {
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
    for(i=0;i<(MSG_LENGTH);i++) {
      if(i==((MSG_LENGTH)-1)) {
        msg_buffer[i]=my2c_read(0);
      } else {
        msg_buffer[i]=my2c_read(1);
      }
    }
    //my2c stop
    my2c_stop();

    //raise EVENT_PIN
    P2OUT |= BV(EVENT_PIN);
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
  //ACT_PIN_ON;
  EEPROM_TOGGLE;
  P2IFG&=~BV(EVENT_PIN);
  is_data_to_read = 1;

  //ACT_PIN_OFF;
}
/*---------------------------------------------------------------------------*/
