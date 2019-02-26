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
 *         Jonas Baechli   jonas.baechli@bluewin.ch
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */

/**
 * \file
 *         Glossy MiddleWare (GMW) implementation.
 *
 * Dependencies: (not sure if that is the right place for this...)
 *  - Glossy implementation.
 *  - Platform port for GMW (gmw-platform.c)
 *  - Extended RTimer (rtimer) implementation with support for at least one
 *      low-frequency and one high-frequency timer.
 *  - GMW_T_HOP
 *  - gmw_rtimer_t => typedef with used timer implementation
 *  - gmw_rtimer_clock_t => typedef for timestamps
 */

/**
 * \addtogroup gmw
 * @{
*/

#include <string.h>
#include "contiki.h"
#include "gmw.h"
#include "node-id.h"
#include "debug-print.h"
#include "gpio.h"

#if DEPCOMP
#include "config.h"
#endif

/*---------------------------------------------------------------------------*/
/* some defines to show activity */
#ifdef GMW_CONF_SLOT_ACT_PIN
 #define GMW_GPIO_CONTROL_SEND_START()  PIN_SET(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_CONTROL_SEND_END()    PIN_CLR(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_CONTROL_RECV_START()  PIN_SET(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_CONTROL_RECV_END()    PIN_CLR(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_PACKET_SEND_START()   PIN_SET(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_PACKET_SEND_END()     PIN_CLR(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_PACKET_RECV_START()   PIN_SET(GMW_CONF_SLOT_ACT_PIN)
 #define GMW_GPIO_PACKET_RECV_END()     PIN_CLR(GMW_CONF_SLOT_ACT_PIN)
#else /* GMW_CONF_SLOT_ACT_PIN */
 #define GMW_GPIO_CONTROL_SEND_START()
 #define GMW_GPIO_CONTROL_SEND_END()
 #define GMW_GPIO_CONTROL_RECV_START()
 #define GMW_GPIO_CONTROL_RECV_END()
 #define GMW_GPIO_PACKET_SEND_START()
 #define GMW_GPIO_PACKET_SEND_END()
 #define GMW_GPIO_PACKET_RECV_START()
 #define GMW_GPIO_PACKET_RECV_END()
#endif /* GMW_CONF_SLOT_ACT_PIN */

#ifdef GMW_CONF_TASK_ACT_PIN
 #define GMW_TASK_RESUMED        PIN_SET(GMW_CONF_TASK_ACT_PIN)
 #define GMW_TASK_SUSPENDED      PIN_CLR(GMW_CONF_TASK_ACT_PIN)
#else /* GMW_CONF_TASK_ACT_PIN */
 #define GMW_TASK_RESUMED
 #define GMW_TASK_SUSPENDED
#endif /* GMW_CONF_TASK_ACT_PIN */
/*---------------------------------------------------------------------------*/
/**
 * @brief     the finite state machine for the time synchronization on a source
 *            node the next state can be retrieved from the current state
 *            (column) and the latest event (row)
 * @note      undefined transitions force the SM to go back into bootstrap!
 */
#define NUM_OF_SYNC_EVENTS      (2)
#define NUM_OF_SYNC_STATES      (3)

static const
gmw_sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] =
{/* STATES:                                              EVENTS:         */
 /* BOOTSTRAP,      RUNNING,        SUSPENDED,                           */
  { GMW_RUNNING,    GMW_RUNNING,    GMW_RUNNING,   }, /* schedule rcvd   */
  { GMW_BOOTSTRAP,  GMW_SUSPENDED,  GMW_BOOTSTRAP, }  /* schedule missed */
};
/*---------------------------------------------------------------------------*/
/* @brief     Macros assessing the outcome of a transmission */
#define GMW_CORRUPTED_PACKET_RECEIVED   \
                              ((n_rx == 0) && (n_rx_started > 0))
#define GMW_PKT_RCVD          (n_rx > 0)
#define GMW_PKT_CORRUPTED     (GMW_CORRUPTED_PACKET_RECEIVED)
                               /* A flood was detected
                                * but not successfully received */
#define GMW_PKT_GARBAGE       ((!GMW_CORRUPTED_PACKET_RECEIVED) && \
                               (GMW_HIGH_NOISE_DETECTED()) )
                               /* Something received but not a valid flood */
#define GMW_PKT_SILENCE       ((!GMW_CORRUPTED_PACKET_RECEIVED) && \
                               (!GMW_HIGH_NOISE_DETECTED()) )
                               /* Nothing received at all */
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macro used to perform an overrun check after each slot in the
 *            round. If the elapsed time since the beginning of the data slots
 *            (t1 - t2) gets too big, the round operation in interrupted.
 */
#define GMW_CHECK_ROUND_OVERRUN(t1, t2, c)  \
  ((GMW_TICKS_TO_MS(t1) - GMW_TICKS_TO_MS(t2)) > \
   (GMW_PERIOD_TO_MS(c.schedule.period) - GMW_CONF_T_PREPROCESS \
                                        - GMW_CONF_T_POSTPROCESS_MIN))
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macros extracting the desired number of retransmissions and slot
 *            time out of the control structure
 */
#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  /* macro to retrieve number of retransmissions of slot i (c => control) */
  #define GMW_CONTROL_GET_SLOT_CONFIG_N_RETRANS(c, i) \
    (GMW_CONTROL_HAS_SLOT_CONFIG(c)) ? \
    ((c)->slot_config[i].n_retransmissions) : \
    (current_config->n_retransmissions)
  /* macro to retrieve slot time of slot i (c => control)*/
  #define GMW_CONTROL_GET_SLOT_CONFIG_TIME(c, i) \
    (IS_CONTENTION_SLOT) ? \
    (GMW_US_TO_TICKS(GMW_CONF_T_CONT)) : \
      ((GMW_CONTROL_HAS_SLOT_CONFIG(c)) ? \
      (GMW_SLOT_TIME_TO_TICKS( \
          (c)->slot_time_list[(c)->slot_config[i].slot_time_select])) : \
      (GMW_SLOT_TIME_TO_TICKS(current_config->slot_time)))
#else /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
  #define GMW_CONTROL_GET_SLOT_CONFIG_N_RETRANS(c, i) \
    current_config->n_retransmissions
  #define GMW_CONTROL_GET_SLOT_CONFIG_TIME(c, i) \
    (IS_CONTENTION_SLOT) ? \
        GMW_US_TO_TICKS(GMW_CONF_T_CONT) : \
        GMW_SLOT_TIME_TO_TICKS(current_config->slot_time)
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macro polling the noise detection process
 */
#ifndef GMW_NOISE_DETECTION
  #if GMW_CONF_USE_NOISE_DETECTION
    #define GMW_NOISE_DETECTION()           gmw_noise_detection_poll()
  #else /* GMW_CONF_USE_NOISE_DETECTION */
    #define GMW_NOISE_DETECTION()
  #endif /* GMW_CONF_USE_NOISE_DETECTION */
#endif /* GMW_NOISE_DETECTION */
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macros to send and receive control and data slots.
 */
#define GMW_SEND_CONTROL() \
{\
  GMW_GPIO_CONTROL_SEND_START();\
  GMW_START(node_id, gmw_payload, control_len, \
            GMW_CONF_TX_CNT_CONTROL, GMW_WITH_SYNC,\
            GMW_WITH_RF_CAL);\
  GMW_NOISE_DETECTION();\
  GMW_WAIT_UNTIL(rt->time + GMW_US_TO_TICKS(GMW_CONF_T_CONTROL));\
  GMW_STOP();\
  GMW_GPIO_CONTROL_SEND_END(); \
}

#define GMW_RCV_CONTROL() \
{\
  GMW_GPIO_CONTROL_RECV_START(); \
  GMW_START(GMW_UNKNOWN_INITIATOR, gmw_payload, 0, \
            GMW_CONF_TX_CNT_CONTROL, GMW_WITH_SYNC, \
            GMW_WITH_RF_CAL);\
  GMW_NOISE_DETECTION();\
  GMW_WAIT_UNTIL(rt->time + \
              GMW_US_TO_TICKS(GMW_CONF_T_CONTROL + GMW_CONF_T_GUARD_ROUND)); \
  GMW_STOP();\
  GMW_GPIO_CONTROL_RECV_END(); \
}

#define GMW_SEND_PACKET() \
{\
  GMW_GPIO_PACKET_SEND_START(); \
  GMW_START_PRIM(node_id, gmw_payload, payload_len, \
                 GMW_CONTROL_GET_SLOT_CONFIG_N_RETRANS(&control, slot_idx), \
                 GMW_WITHOUT_SYNC, GMW_WITHOUT_RF_CAL);\
  GMW_NOISE_DETECTION();\
  GMW_WAIT_UNTIL(rt->time + current_slot_time);\
  GMW_STOP_PRIM();\
  GMW_GPIO_PACKET_SEND_END(); \
}

#define GMW_RCV_PACKET() \
{\
  GMW_GPIO_PACKET_RECV_START(); \
  GMW_START_PRIM(GMW_UNKNOWN_INITIATOR, gmw_payload, \
                 payload_len, \
                 GMW_CONTROL_GET_SLOT_CONFIG_N_RETRANS(&control, slot_idx), \
                 GMW_WITHOUT_SYNC, GMW_WITHOUT_RF_CAL);\
  GMW_NOISE_DETECTION();\
  GMW_WAIT_UNTIL(rt->time + current_slot_time + \
                 GMW_US_TO_TICKS(GMW_CONF_T_GUARD_SLOT));\
  GMW_STOP_PRIM();\
  GMW_GPIO_PACKET_RECV_END(); \
}
/*---------------------------------------------------------------------------*/
/**
 * @brief     Suspend the GMW proto-thread until the rtimer reaches
 *            the specified time.
 */
#define GMW_WAIT_UNTIL(time) \
{\
  GMW_RTIMER_SCHEDULE(time, gmw_thread);\
  GMW_TASK_SUSPENDED;\
  PT_YIELD(&gmw_pt);\
  GMW_TASK_RESUMED;\
}
#ifndef GMW_BEFORE_DEEPSLEEP
#define GMW_BEFORE_DEEPSLEEP()
#endif /* GMW_PREPARE_DEEPSLEEP */
#ifndef GMW_AFTER_DEEPSLEEP
#define GMW_AFTER_DEEPSLEEP()
#endif /* GMW_AFTER_DEEPSLEEP */
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macros used to support different synchronous transmission
 *            primitives
 */
#if GMW_CONF_USE_MULTI_PRIMITIVES
  #ifndef GMW_START_PRIM0
    #define GMW_START_PRIM0(a, b, c, d, e, f)   GMW_START(a, b, c, d, e, f)
    #define GMW_STOP_PRIM0()                    GMW_STOP()
    #define GMW_GET_PAYLOAD_LEN_PRIM0()         GMW_GET_PAYLOAD_LEN()
    #define GMW_GET_N_RX_PRIM0()                GMW_GET_N_RX()
    #define GMW_GET_N_RX_STARTED_PRIM0()        GMW_GET_N_RX_STARTED()
  #endif /* GMW_START_PRIM0 */
  #ifndef GMW_START_PRIM1
    #define GMW_START_PRIM1(a, b, c, d, e, f)
    #define GMW_STOP_PRIM1()
    #define GMW_GET_PAYLOAD_LEN_PRIM1()         0
    #define GMW_GET_N_RX_PRIM1()                0
    #define GMW_GET_N_RX_STARTED_PRIM1()        0
  #endif /* GMW_START_PRIM1 */
  #ifndef GMW_START_PRIM2
    #define GMW_START_PRIM2(a, b, c, d, e, f)
    #define GMW_STOP_PRIM2()
    #define GMW_GET_PAYLOAD_LEN_PRIM2()         0
    #define GMW_GET_N_RX_PRIM2()                0
    #define GMW_GET_N_RX_STARTED_PRIM2()        0
  #endif /* GMW_START_PRIM2 */
  #ifndef GMW_START_PRIM3
    #define GMW_START_PRIM3(a, b, c, d, e, f)
    #define GMW_STOP_PRIM3()
    #define GMW_GET_PAYLOAD_LEN_PRIM3()         0
    #define GMW_GET_N_RX_PRIM3()                0
    #define GMW_GET_N_RX_STARTED_PRIM3()        0
  #endif /* GMW_START_PRIM3 */

  #if GMW_CONF_USE_CONTROL_SLOT_CONFIG
    #define GET_PRIMTITIVE()   (GMW_CONTROL_HAS_SLOT_CONFIG(&control) ? \
                                 (control.slot_config[slot_idx].primitive) :\
                                 (current_config->primitive))
  #else /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
    #define GET_PRIMTITIVE()   (current_config->primitive)
  #endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */

  #define GMW_START_PRIM(a, b, c, d, e, f) { \
    gmw_primitive = GET_PRIMTITIVE(); \
    if(gmw_primitive == 0) { \
      GMW_START_PRIM0(a, b, c, d, e, f); \
    } else if(gmw_primitive == 1) { \
      GMW_START_PRIM1(a, b, c, d, e, f); \
    } else if(gmw_primitive == 2) { \
      GMW_START_PRIM2(a, b, c, d, e, f); \
    } else if(gmw_primitive == 3) { \
      GMW_START_PRIM3(a, b, c, d, e, f); \
    } \
  }
  #define GMW_STOP_PRIM() { \
    if(gmw_primitive == 0) { \
      GMW_STOP_PRIM0(); \
      payload_len   = GMW_GET_PAYLOAD_LEN_PRIM0(); \
      n_rx          = GMW_GET_N_RX_PRIM0(); \
      n_rx_started  = GMW_GET_N_RX_STARTED_PRIM0(); \
    } else if(gmw_primitive == 1) { \
      GMW_STOP_PRIM1(); \
      payload_len   = GMW_GET_PAYLOAD_LEN_PRIM1(); \
      n_rx          = GMW_GET_N_RX_PRIM1(); \
      n_rx_started  = GMW_GET_N_RX_STARTED_PRIM1(); \
    } else if(gmw_primitive == 2) { \
      GMW_STOP_PRIM2(); \
      payload_len   = GMW_GET_PAYLOAD_LEN_PRIM2(); \
      n_rx          = GMW_GET_N_RX_PRIM2(); \
      n_rx_started  = GMW_GET_N_RX_STARTED_PRIM2(); \
    } else if(gmw_primitive == 3) { \
      GMW_STOP_PRIM3(); \
      payload_len   = GMW_GET_PAYLOAD_LEN_PRIM3(); \
      n_rx          = GMW_GET_N_RX_PRIM3(); \
      n_rx_started  = GMW_GET_N_RX_STARTED_PRIM3(); \
    } else { \
      payload_len   = 0; \
      n_rx          = 0; \
      n_rx_started  = 0; \
    } \
    gmw_primitive = 0; /* back to default primitive */ \
  }
#else  /* GMW_CONF_USE_MULTI_PRIMITIVES */
  /* just use the default primitive */
  #define GMW_START_PRIM        GMW_START
  #define GMW_STOP_PRIM()       GMW_STOP(); \
                                payload_len   = GMW_GET_PAYLOAD_LEN(); \
                                n_rx          = GMW_GET_N_RX(); \
                                n_rx_started  = GMW_GET_N_RX_STARTED();
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
/*---------------------------------------------------------------------------*/
#define GMW_IS_HOST             (node_id == HOST_ID)
/*---------------------------------------------------------------------------*/
static struct pt                gmw_pt;
static gmw_protocol_impl_t*     host_impl;
static gmw_protocol_impl_t*     src_impl;
static gmw_protocol_impl_t*     gmw_impl;   /* currently used implementation */
static gmw_pre_post_processes_t pre_post_proc;
static gmw_control_t            control;
static gmw_control_t*           new_control;
static gmw_sync_state_t         sync_state;
static gmw_rtimer_clock_t       rx_timestamp;
static gmw_statistics_t         stats = { 0 };
static uint32_t                 global_time;
static uint8_t                  gmw_payload[GMW_MAX_PKT_LEN];
static uint8_t                  control_len;
#if GMW_CONF_USE_MULTI_PRIMITIVES
uint8_t                         gmw_primitive;
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
/*---------------------------------------------------------------------------*/
/**
 * @brief     Check if new control information has been send by the application.
 *            If so, copies the application-side control info into the
 *            middleware-side control structure.
 */
static void
copy_control_if_updated(void);
/*---------------------------------------------------------------------------*/
const gmw_statistics_t * const
gmw_get_stats(void)
{
  return &stats;
}
/*---------------------------------------------------------------------------*/
gmw_sync_state_t
gmw_get_state(void)
{
  return sync_state;
}
/*---------------------------------------------------------------------------*/
/**
 * @brief     GMW protothread.
 *            It implements the generic round structure, schedule the execution
 *            of the primitives, and executes the callback functions.
 */
PT_THREAD(gmw_thread(gmw_rtimer_t *rt))
{
  /* all variables must be static due to protothread implementation */

  static gmw_config_t           current_config_instance;
  static gmw_config_t*          current_config;

  static gmw_rtimer_clock_t     t_start;
  static gmw_rtimer_clock_t     t_ref;
  static gmw_rtimer_clock_t     t_now;
  static gmw_rtimer_clock_t     start_of_next_round;
  static gmw_rtimer_clock_t     slot_start;
  static gmw_rtimer_clock_t     current_slot_time;

  static uint32_t               pre_process_offset;
  static uint32_t               missed_slots;
  static uint8_t                payload_len;
  static uint8_t                n_rx;
  static uint8_t                n_rx_started;
  static uint8_t                n_missed_slots;
  static uint8_t                is_current_config_valid;

  static gmw_pkt_event_t        pkt_event;
  static gmw_sync_state_t       impl_sync_state;
  static gmw_sync_event_t       sync_event;

#if GMW_CONF_USE_DRIFT_COMPENSATION
  static gmw_rtimer_clock_t     t_ref_last;
  static uint16_t               period_last;
#endif /* GMW_CONF_USE_DRIFT_COMPENSATION */

  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */

  PT_BEGIN(&gmw_pt);   /* declare variables before this statement! */

  /* ---------------------------- INITIALIZATION ---------------------------- */
  if(GMW_IS_HOST) {
    /* HOST NODE */
    current_config      = &control.config;
    gmw_impl            = host_impl;
    stats.t_slack_min   = 0xffffffff;
    sync_state          = GMW_RUNNING; /* 'RUNNING' is default state of host */
    start_of_next_round = rt->time +
                          GMW_CONF_T_PREPROCESS * GMW_RTIMER_SECOND / 1000 +
                          GMW_US_TO_TICKS(GMW_CONF_PREROUND_SETUP_TIME) +
                          GMW_RTIMER_SECOND / 100;

  } else {
    /* SOURCE NODE */
    current_config          = &current_config_instance;
    gmw_impl                = src_impl;
    is_current_config_valid = 0;
    sync_state              = GMW_BOOTSTRAP;
    memset(current_config, 0, sizeof(gmw_config_t));
  }
  memset(&control, 0, sizeof(control));
  gmw_control_init(&control);
  pre_process_offset  = 0;
  start_of_next_round = 0;
  GMW_RTIMER_RESET();
  rt->time = 0;

  if(!gmw_impl) {
    DEBUG_PRINT_FATAL("No callback implementation!");
    while (1);
  }

  /* ------------------------------ MAIN LOOP ------------------------------ */
  while(1) {

    /* poll the pre process if applicable */
  #if GMW_CONF_T_PREPROCESS
    if(pre_post_proc.pre_process_next_round && pre_process_offset) {
      process_poll(pre_post_proc.pre_process_next_round);
      start_of_next_round += pre_process_offset;
      GMW_WAIT_UNTIL(start_of_next_round);
    }
  #endif /* GMW_CONF_T_PREPROCESS */

    /* --- COMMUNICATION ROUND STARTS --- */

    /* the control packet might be much longer than other packets */
    gmw_set_maximum_packet_length(GMW_CONF_MAX_CONTROL_PKT_LEN +
                                  GMW_CONF_RF_OVERHEAD);

    if(GMW_IS_HOST) {
      /* prepare control packet */
      copy_control_if_updated();
      control_len = gmw_control_compile_to_buffer(&control, gmw_payload,
                                                  GMW_MAX_PKT_LEN);
      if(control_len == 0) {
        DEBUG_PRINT_MSG_NOW("Packet buffer too small to send the control.");
        break;
      }

      /* busy wait until for round start */
      start_of_next_round += GMW_US_TO_TICKS(GMW_CONF_PREROUND_SETUP_TIME);

      if(start_of_next_round < GMW_RTIMER_NOW()) {
        DEBUG_PRINT_ERROR("GMW t_start overrun by %ld ticks",
                          (uint32_t)(GMW_RTIMER_NOW() - start_of_next_round));
      }
      GMW_WAIT_UNTIL(start_of_next_round);

      /* set the time stamp for the start of this round */
      t_start = start_of_next_round;

      /* send the actual control slot */
      GMW_SEND_CONTROL();

      global_time     = control.schedule.time;
      rx_timestamp    = GMW_GET_T_REF();
      stats.relay_cnt = GMW_GET_RELAY_CNT_FIRST_RX();

      /* inform implementation about control slot and update internal state */
      sync_state = gmw_impl->on_control_slot_post(&control,
                                                  GMW_EVT_CONTROL_RCVD,
                                                  GMW_EVT_PKT_OK);
      if(!(GMW_RUNNING == sync_state || GMW_SUSPENDED == sync_state)) {
        DEBUG_PRINT_ERROR("host on_control_slot_post returned invalid state!");
        while(1);
      }

    } else { /* SOURCE NODE */

      if(sync_state == GMW_BOOTSTRAP) {
BOOTSTRAP_MODE:
  #if GMW_CONF_USE_DRIFT_COMPENSATION
        period_last = 0;
  #endif /* GMW_CONF_USE_DRIFT_COMPENSATION */
        /* Reset the part of the control that is supposed to be received.
         * Right now, does not erase anything if static, can be extended 
         * to clean the user bytes */
        if((!GMW_CONF_USE_STATIC_SCHED) && (!GMW_CONF_USE_STATIC_CONFIG)) {
          memset(&control, 0, sizeof(control));
        }
        stats.bootstrap_cnt++;
        is_current_config_valid = 0;
        /* synchronize first! wait for the first control packet... */
        do {
          /* try to receive a control packet */
          GMW_RCV_CONTROL();

          /* did we receive a synced (setting t_ref) glossy packet? */
          if(!GMW_IS_T_REF_UPDATED()) {
            uint32_t time_to_sleep_in_ms = gmw_impl->on_bootstrap_timeout();
            if(time_to_sleep_in_ms != 0) {
              /* we go to sleep */
              DEBUG_PRINT_MSG_NOW("going to sleep for %lums",
                                  time_to_sleep_in_ms);
              stats.sleep_cnt++;
              GMW_BEFORE_DEEPSLEEP();
              GMW_WAIT_UNTIL(GMW_RTIMER_NOW() +
                             GMW_MS_TO_TICKS(time_to_sleep_in_ms));
              GMW_AFTER_DEEPSLEEP();
            }
          } else {
            /* we received something, try to interpret it as a control pkt */
            break;
          }
        } while(1);
        /* schedule received! */
      } else {
        /* max packet length was set at end of last round */
        GMW_RCV_CONTROL();
      }

      /* did we receive a synced glossy packet?
       * If so, we know for sure we successfully received a control packet
       * -> True only assuming there are no other Glossy network running in 
       *    parallel!
       * Also check the magic number after decompiling the buffer
       * to avoid sync with another network. */
      if(GMW_IS_T_REF_UPDATED()) {
        /* HF timestamp of first RX; subtract a constant offset */
        t_ref        = GMW_GET_T_REF() - 
                       GMW_US_TO_TICKS(GMW_CONF_T_REF_OFS);
        global_time  = control.schedule.time;
        rx_timestamp = t_ref;
        sync_event   = GMW_EVT_CONTROL_RCVD;

        /* reconstruct the control struct */
        if(!gmw_control_decompile_from_buffer(&control, gmw_payload,
                                              GMW_GET_PAYLOAD_LEN())) {
          DEBUG_PRINT_MSG_NOW("Reception of control buggy. "
                              "Back to bootstrap.");
          goto BOOTSTRAP_MODE;
        }

  #if GMW_CONF_USE_MAGIC_NUMBER
        /* check the magic number */
        if(control.magic_number != GMW_CONF_CONTROL_MAGIC_NUMBER) {
          DEBUG_PRINT_MSG_NOW("Received packet is not a valid control packet. "
                              "Back to bootstrap.");
          goto BOOTSTRAP_MODE;
        }
  #endif /*GMW_CONF_USE_MAGIC_NUMBER*/

        /* store current config if received */
        if(GMW_CONTROL_HAS_CONFIG(&control)) {
          /**
           *  Postpone the memcpy of current_config
           * till after the control_slot_post callback
           * (for compatibility with the 'static' configuration of the
           *  middleware)
           */
          is_current_config_valid = 1;
        } else if(!is_current_config_valid) {
          /* we need a valid config */
          goto BOOTSTRAP_MODE;
        }
      } else {
        DEBUG_PRINT_WARNING("Schedule missed or corrupted.");
        /* we can only estimate t_ref */
        t_ref += ((int32_t)control.schedule.period * GMW_RTIMER_SECOND +
           GMW_PPM_TO_TICKS((uint32_t)control.schedule.period * stats.drift)) /
           GMW_CONF_TIME_SCALE;
  #if GMW_CONF_USE_DRIFT_COMPENSATION
        /* make sure t_ref is not used for drift compensation in this round */
        period_last = 0;
  #endif /* GMW_CONF_USE_DRIFT_COMPENSATION */

        sync_event = GMW_EVT_CONTROL_MISSED;

  #if GMW_CONF_USE_NOISE_DETECTION
        /* What went wrong with the control packet reception?
         * Only valid if the noise detection feature is enabled, i.e.
         * GMW_CONF_USE_NOISE_DETECTION is set to 1 */
        if(GMW_PKT_CORRUPTED) {
          pkt_event = GMW_EVT_PKT_CORRUPTED;
          DEBUG_PRINT_VERBOSE("Control corrupted");

        } else if(GMW_PKT_GARBAGE) {
          pkt_event = GMW_EVT_PKT_GARBAGE;
          DEBUG_PRINT_VERBOSE("Control screwed up");

        } else if(GMW_PKT_SILENCE) {
          pkt_event = GMW_EVT_PKT_SILENCE;
          DEBUG_PRINT_VERBOSE("Control missing");

        } else {
          DEBUG_PRINT_WARNING("Packet reception event is buggy...");
        }
  #endif /* GMW_CONF_USE_NOISE_DETECTION */
      }

      /* inform higher layer about control slot, update state machine */
      impl_sync_state = gmw_impl->on_control_slot_post(&control,
                                                       sync_event,
                                                       pkt_event);
      if(GMW_DEFAULT == impl_sync_state) {
        sync_state = next_state[sync_event][sync_state];
      } else {
        sync_state = impl_sync_state;
      }
      if(GMW_BOOTSTRAP == sync_state) {
        goto BOOTSTRAP_MODE;
      }

      /* store current config if received */
      if(GMW_CONTROL_HAS_CONFIG(&control)) {
        memcpy(current_config, &control.config,
               sizeof(gmw_config_t));
      }

      t_start = t_ref;
    } /* end of !GMW_IS_HOST */

    /* --- PREPARE FOR DATA SLOTS ---*/

    /* reset variables */
    n_missed_slots = 0;
    missed_slots = 0;

  #if GMW_CONF_USE_AUTOCLEAN
    /* if AUTOCLEAN is set, zero-ed the middleware send/receive buffer */
    memset(gmw_payload, 0, GMW_MAX_PKT_LEN);
  #endif /* GMW_CONF_USE_AUTOCLEAN */

    /* setup radio for data packets */
    gmw_set_maximum_packet_length(current_config->max_packet_length +
                                  GMW_CONF_RF_OVERHEAD);

    /* start time of first data slot */
    slot_start = t_start + GMW_US_TO_TICKS(GMW_CONF_T_CONTROL) +
                           GMW_US_TO_TICKS(GMW_CONF_T_GAP_CONTROL);

    /* permission to participate in this round? */
    if(GMW_RUNNING == sync_state) {

      /* --- DATA SLOTS --- */
      static uint8_t slot_idx;
      for(slot_idx = 0; slot_idx < GMW_SCHED_N_SLOTS(&control.schedule);
          slot_idx++) {

        /* prepare variables for current slot */
      #define IS_INITIATOR        (control.schedule.slot[slot_idx] == node_id)
      #define IS_CONTENTION_SLOT  (GMW_SLOT_CONTENTION == \
                                   control.schedule.slot[slot_idx])
        payload_len         = 0;
        n_rx                = 0;
        n_rx_started        = 0;
        current_slot_time   = GMW_CONTROL_GET_SLOT_CONFIG_TIME(&control,
                                                              slot_idx);

        /* on_slot_pre_callback */
        gmw_skip_event_t skip_event;  /* no need to make this static */
        skip_event = gmw_impl->on_slot_pre(slot_idx,
                                           control.schedule.slot[slot_idx],
                                           &payload_len,
                                           gmw_payload,
                                           IS_INITIATOR,
                                           IS_CONTENTION_SLOT);

        /* set t_now, this allows us to determine if we missed the slot */
        t_now = GMW_RTIMER_NOW();
        if(skip_event == GMW_EVT_SKIP_SLOT) {
          /* Instruction to skip the slot, but we have payload to send... */
          pkt_event   = GMW_EVT_PKT_SKIPPED;
          if(payload_len) {
            DEBUG_PRINT_WARNING("slot %u skipped (app flag)", slot_idx);
          }
          payload_len = 0;

        /* did we miss the slot? */
        } else if(slot_start <= t_now) {
          n_missed_slots++;
          missed_slots |= (1 << slot_idx);
          /* we are too late, abort the flood */
          pkt_event   = GMW_EVT_PKT_MISSED;
          payload_len = 0;
          DEBUG_PRINT_VERBOSE("slot %u skipped (missed by %ld ticks)",
                              slot_idx, (int32_t)(t_now - slot_start));

        } else if(IS_INITIATOR || (IS_CONTENTION_SLOT && payload_len)) {
          /* INITIATE the flood */
          GMW_WAIT_UNTIL(slot_start);
          GMW_SEND_PACKET();
          pkt_event = GMW_EVT_PKT_OK;
          DEBUG_PRINT_VERBOSE("packet sent (%ub)", payload_len);

        } else {
          /* RECEIVER */
          gmw_rtimer_clock_t slot_start_with_guard = slot_start -
                                        GMW_US_TO_TICKS(GMW_CONF_T_GUARD_SLOT);
          if(t_now >= slot_start_with_guard) {
            slot_start_with_guard = slot_start;
          }
          GMW_WAIT_UNTIL(slot_start_with_guard);
          GMW_RCV_PACKET();

          /* --- SLOT ENDED --- */

          /* track the slot-post processing time */
          t_now = GMW_RTIMER_NOW();

          /* did we receive data? */
          if(GMW_PKT_RCVD) {
            pkt_event = GMW_EVT_PKT_OK;
            DEBUG_PRINT_VERBOSE("received packet, len=%u", payload_len);
            stats.pkt_rcvd_cnt++;

          } else if(GMW_PKT_CORRUPTED) {
            pkt_event = GMW_EVT_PKT_CORRUPTED;
            DEBUG_PRINT_VERBOSE("corrupted packet received in slot %u",
                                slot_idx);
            stats.pkt_corr_cnt++;

          } else if(GMW_PKT_GARBAGE) {
            pkt_event = GMW_EVT_PKT_GARBAGE;
            DEBUG_PRINT_VERBOSE("garbage received in slot %u", slot_idx);
            stats.pkt_interf_cnt++;

          } else if(GMW_PKT_SILENCE) {
            pkt_event = GMW_EVT_PKT_SILENCE;
            DEBUG_PRINT_VERBOSE("no data received in slot %u from node %u",
                                slot_idx, control.schedule.slot[slot_idx]);
            stats.pkt_silence_cnt++;

          } else {
            DEBUG_PRINT_ERROR("invalid packet reception event");
            pkt_event = GMW_EVT_PKT_SILENCE;
          }
        }

        /* on_slot_post_callback */
        static gmw_repeat_event_t repeat_event;
        repeat_event = gmw_impl->on_slot_post(slot_idx,
                                              control.schedule.slot[slot_idx],
                                              payload_len,
                                              gmw_payload,
                                              IS_INITIATOR,
                                              IS_CONTENTION_SLOT,
                                              pkt_event);

        /* store post-precessing time */
        stats.t_proc_max = MAX((uint16_t)
                               GMW_TICKS_TO_US(GMW_RTIMER_NOW() - t_now),
                               stats.t_proc_max);

        /* update the start time of the next slot */
        slot_start += current_slot_time + 
                      GMW_GAP_TIME_TO_TICKS(current_config->gap_time);

  #if GMW_CONF_USE_AUTOCLEAN
        /* if AUTOCLEAN is set, zero-ed the middleware send/receive buffer */
        memset(gmw_payload, 0, GMW_MAX_PKT_LEN);
  #endif /* GMW_CONF_USE_AUTOCLEAN */

        /* check if the round is not getting too long */
        if(GMW_CHECK_ROUND_OVERRUN(slot_start, t_start, control)) {
          DEBUG_PRINT_ERROR("Round overruns. Aborted after slot %u", slot_idx);
          break;
        }

        /* look at the repeat flag */
        if(repeat_event == GMW_EVT_REPEAT_SLOT) {
          slot_idx -= 1;
        }
        if(repeat_event == GMW_EVT_REPEAT_ROUND) {
          /* when next incremented by the middleware, slot_idx becomes 0 */
          slot_idx = 255;
        }
      }
    }

    gmw_impl->on_round_finished(&pre_post_proc);

    /* --- COMMUNICATION ROUND ENDS --- */

    if(!GMW_IS_HOST) {
  #if GMW_CONF_USE_DRIFT_COMPENSATION
      /* only update drift compensation if synced */
      if(GMW_EVT_CONTROL_RCVD == sync_event) {
        if(period_last) {
          /* estimate the clock drift in ppm */
          /* convert from ticks to us, then normalize (divide by the period) to
          * get ppm */
          int32_t diff = (int32_t)(t_ref - t_ref_last) -
                         (int32_t)GMW_PERIOD_TO_TICKS((uint32_t)period_last);
          int32_t drift = GMW_TICKS_TO_PPM(diff * GMW_CONF_TIME_SCALE) /
                          (int32_t)period_last;
          if((drift < GMW_CONF_MAX_CLOCK_DEV) &&
            (drift > -GMW_CONF_MAX_CLOCK_DEV)) {
            stats.drift = (stats.drift + drift) / 2;      /* low-pass filter */
          }
        }
        t_ref_last  = t_ref;
        period_last = control.schedule.period;
      }
  #endif /* GMW_CONF_USE_DRIFT_COMPENSATION */

    }

    /* state keeping */
    if(sync_state == GMW_SUSPENDED) {
      stats.suspended_cnt++;
    }

    /* print missed slot count and bitshifted 1 marking each missed slot */
    if(n_missed_slots) {
      DEBUG_PRINT_WARNING("Missed %u slots! Binary: %" PRIX32, n_missed_slots,
                          missed_slots);
    }

    /* poll the post process */
    if(pre_post_proc.post_process_current_round) {
      process_poll(pre_post_proc.post_process_current_round);
    }

    /* wake-up earlier to execute the pre-process (if one is defined) */
    pre_process_offset = (!pre_post_proc.pre_process_next_round) ? 0 :
                         GMW_MS_TO_TICKS(GMW_CONF_T_PREPROCESS);

    /* schedule the next wake-up*/
    if(GMW_IS_HOST) {
      /* start of next round in clock ticks */
      start_of_next_round = t_start +
          (gmw_rtimer_clock_t)control.schedule.period * GMW_RTIMER_SECOND /
          GMW_CONF_TIME_SCALE -
          GMW_US_TO_TICKS(GMW_CONF_PREROUND_SETUP_TIME) -
          pre_process_offset;
    } else {
      /* start of next round in clock ticks */
      start_of_next_round = t_ref +
          ((gmw_rtimer_clock_t)control.schedule.period * GMW_RTIMER_SECOND +
           GMW_PPM_TO_TICKS((uint32_t)control.schedule.period * stats.drift)) /
           GMW_CONF_TIME_SCALE -
          GMW_US_TO_TICKS(GMW_CONF_T_GUARD_ROUND) -
          pre_process_offset;
    }

    /* state keeping */
    uint32_t measured_round_time_ms =
                         (uint32_t)GMW_TICKS_TO_MS(GMW_RTIMER_NOW() - t_start);
    /* sanity check */
    if(GMW_PERIOD_TO_MS(control.schedule.period) < measured_round_time_ms) {
      DEBUG_PRINT_WARNING("Total measured round time %lums exceeds the round "
                        "period!", measured_round_time_ms);
    }
    stats.t_round_max = MAX(measured_round_time_ms, stats.t_round_max);

    /* suspend this task */
    GMW_WAIT_UNTIL(start_of_next_round);

  } /* main loop */

  PT_END(&gmw_pt);
}
/*---------------------------------------------------------------------------*/
void
gmw_set_new_control(gmw_control_t* control)
{
  new_control = control;
}
/*---------------------------------------------------------------------------*/
void
gmw_start(struct process* pre_gmw_proc, 
          struct process *post_gmw_proc,
          gmw_protocol_impl_t* host_protocol_impl,
          gmw_protocol_impl_t* src_protocol_impl)
{
  pre_post_proc.post_process_current_round = post_gmw_proc;
  pre_post_proc.pre_process_next_round     = pre_gmw_proc;
  host_impl = host_protocol_impl;
  src_impl  = src_protocol_impl;

  /* check if pre_process is correctly set */
  if(((pre_gmw_proc != NULL) && (GMW_CONF_T_PREPROCESS == 0)) ||
     ((pre_gmw_proc == NULL) && (GMW_CONF_T_PREPROCESS != 0))) {
    DEBUG_PRINT_ERROR("A pre-processing task is set, but GMW_CONF_T_PREPROCESS"
                      " is zero or vice-versa.");
  }

  /* debug-print is required for gmw */
  debug_print_init();

#if GMW_CONF_USE_NOISE_DETECTION
  gmw_noise_detection_init();
#endif /* GMW_CONF_USE_NOISE_DETECTION */

  /* set RF channel and transmit power */
  gmw_set_rf_channel(GMW_CONF_RF_TX_CHANNEL);
  gmw_set_tx_power(GMW_CONF_RF_TX_POWER);

  /* allow platform dependent initialization procedure */
  gmw_platform_init();

  DEBUG_PRINT_INFO("Starting GMW protothread (%s node)", 
                   GMW_IS_HOST ? "host" : "source");
  DEBUG_PRINT_INFO("t_ctrl=%ums, t_data=%ums, t_cont=%ums, "
                   "data=%ub, slots=%u, tx=%u, hop=%u, scale=%u",
                   (uint16_t)(GMW_CONF_T_CONTROL / 1000),
                   (uint16_t)(GMW_CONF_T_DATA / 1000),
                   (uint16_t)(GMW_CONF_T_CONT / 1000),
                   GMW_CONF_MAX_DATA_PKT_LEN,
                   GMW_CONF_MAX_SLOTS,
                   GMW_CONF_TX_CNT_DATA,
                   GMW_CONF_MAX_HOPS,
                   (uint16_t)GMW_CONF_TIME_SCALE);

  /* start the GMW protothread in 100ms */
  GMW_RTIMER_SCHEDULE(GMW_RTIMER_NOW() + GMW_RTIMER_SECOND / 10, &gmw_thread);

  /* poll the debug print process */
  debug_print_poll();
}
/*---------------------------------------------------------------------------*/
static void
copy_control_if_updated(void)
{
  /* copy the updated control packet */
  if(NULL != new_control) {
    memcpy(&control, new_control, sizeof(gmw_control_t));
    new_control = NULL;
  }
}
/*---------------------------------------------------------------------------*/

/** @} */
