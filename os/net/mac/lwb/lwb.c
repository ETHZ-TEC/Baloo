/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Reto Da Forno
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @file
 *
 * an implementation of the Low-Power Wireless Bus
 */

#include "lwb.h"
#include "fifo16.h"
#include "debug-print.h"
#include "gpio.h"
#include "node-id.h"
#include "random.h"
#include "glossy.h"

/*---------------------------------------------------------------------------*/
#if LWB_CONF_HEADER_LEN != 3
#error "LWB_CONF_HEADER_LEN must be 3!"
#endif
#define LWB_DATA_PKT_PAYLOAD_LEN    (LWB_CONF_MAX_DATA_PKT_LEN - \
                                     LWB_CONF_HEADER_LEN)

/* indicates when this node is about to send a request */
#ifdef LWB_REQ_IND_PIN
  #define LWB_REQ_IND               { PIN_SET(LWB_REQ_IND_PIN); \
                                      PIN_CLR(LWB_REQ_IND_PIN); }
#else /* LWB_CONF_REQ_SENT_PIN */
  #define LWB_REQ_IND
#endif /* LWB_CONF_REQ_SENT_PIN */

/* code that is executed upon detection of a contention */
#ifndef LWB_REQ_DETECTED
#define LWB_REQ_DETECTED
#endif /* LWB_REQ_DETECTED */

/* indicates when this node is about to send a data packet */ 
#ifdef LWB_DATA_IND_PIN
  #define LWB_DATA_IND              { PIN_SET(LWB_DATA_IND_PIN); \
                                      PIN_CLR(LWB_DATA_IND_PIN); }
#else /* LWB_CONF_DATA_IND_PIN */
  #define LWB_DATA_IND
#endif /* LWB_CONF_DATA_IND_PIN */

/* is executed before each data slot */
#ifndef LWB_DATA_SLOT_STARTS
#define LWB_DATA_SLOT_STARTS
#endif /* LWB_DATA_SLOT_STARTS */
/*---------------------------------------------------------------------------*/
/* internal sync state of the LWB on the source node */
typedef enum {
  LWB_STATE_BOOTSTRAP = 0,
  LWB_STATE_QUASI_SYNCED,
  LWB_STATE_SYNCED,
  LWB_STATE_SYNCED_2,
  LWB_STATE_MISSED,
  LWB_STATE_UNSYNCED,
  LWB_STATE_UNSYNCED2,
  NUM_OF_SYNC_STATES
} lwb_sync_state_t;
/*---------------------------------------------------------------------------*/
typedef enum {
  EVT_1ST_SCHED_RCVD = 0,
  EVT_2ND_SCHED_RCVD = 1,
  EVT_SCHED_LWB_STATE_MISSED = 2,
  NUM_OF_SYNC_EVENTS
} lwb_sync_event_t;
/*---------------------------------------------------------------------------*/
typedef struct {
  uint16_t recipient;     /* target node ID */
  uint8_t  stream_id;     /* message type and connection ID (used as stream ID
                             in LWB); first bit is msg type */
  uint8_t  payload[LWB_DATA_PKT_PAYLOAD_LEN];       
} lwb_data_pkt_t;
/*---------------------------------------------------------------------------*/
typedef struct {  
  /* be aware of structure alignment (8-bit are aligned to 8-bit, 16 to 16 etc)
   * the first 3 bytes of a glossy packet are always node_id and stream_id */
  union {
    struct {
      lwb_data_pkt_t data_pkt;
      uint8_t reserved1[LWB_CONF_MAX_PKT_LEN - LWB_CONF_MAX_DATA_PKT_LEN];
    };
    struct {
      lwb_stream_req_t srq_pkt;
      uint8_t reserved2[LWB_CONF_MAX_PKT_LEN - LWB_STREAM_REQ_PKT_LEN];
    };
    lwb_stream_ack_t sack_pkt;
    uint8_t raw_data[LWB_CONF_MAX_PKT_LEN];
  };
} glossy_payload_t;
/*---------------------------------------------------------------------------*/
/**
 * @brief the finite state machine for the time synchronization on a source 
 * node the next state can be retrieved from the current state (column) and
 * the latest event (row)
 * @note  undefined transitions force the SM to go back into bootstrap
 */
#if LWB_CONF_SKIP_QUASI_SYNCED
#define AFTER_BOOTSTRAP	    LWB_STATE_SYNCED
#else
#define AFTER_BOOTSTRAP     LWB_STATE_QUASI_SYNCED
#endif
static const 
lwb_sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_SYNC_STATES] = 
{
/* EVENTS:   \ STATES ->  LWB_STATE_BOOTSTRAP, LWB_STATE_QUASI_SYNCED, LWB_STATE_SYNCED,    LWB_STATE_SYNCED2,   LWB_STATE_MISSED,    LWB_STATE_UNSYNCED,  LWB_STATE_UNSYNCED2                         */
/* 1st schedule rcvd */ { AFTER_BOOTSTRAP,     LWB_STATE_SYNCED,       LWB_STATE_BOOTSTRAP, LWB_STATE_SYNCED,    LWB_STATE_SYNCED,    LWB_STATE_BOOTSTRAP, LWB_STATE_SYNCED    },
/* 2nd schedule rcvd */ { LWB_STATE_BOOTSTRAP, LWB_STATE_QUASI_SYNCED, LWB_STATE_SYNCED_2,  LWB_STATE_BOOTSTRAP, LWB_STATE_BOOTSTRAP, LWB_STATE_SYNCED_2,  LWB_STATE_BOOTSTRAP },
/* schedule missed   */ { LWB_STATE_BOOTSTRAP, LWB_STATE_BOOTSTRAP,    LWB_STATE_MISSED,    LWB_STATE_UNSYNCED,  LWB_STATE_UNSYNCED,  LWB_STATE_UNSYNCED2, LWB_STATE_BOOTSTRAP }
};
/* note: syn2 = already synced */
static const char* lwb_sync_state_to_string[NUM_OF_SYNC_STATES] = 
{ "BOOTSTRAP", "QSYN", "SYN", "SYN2", "MISS", "USYN", "USYN2" };
static const uint32_t guard_time[NUM_OF_SYNC_STATES] = {
/* STATE:      LWB_STATE_BOOTSTRAP, LWB_STATE_QUASI_SYNCED, LWB_STATE_SYNCED,   LWB_STATE_SYNCED_2, LWB_STATE_MISSED,   LWB_STATE_UNSYNCED, LWB_STATE_UNSYNCED2 */
/* T_GUARD: */ LWB_CONF_T_GUARD,    LWB_CONF_T_GUARD_2,     LWB_CONF_T_GUARD_1, LWB_CONF_T_GUARD_1, LWB_CONF_T_GUARD_2, LWB_CONF_T_GUARD_3, LWB_CONF_T_GUARD_3
};
/*---------------------------------------------------------------------------*/
#ifdef LWB_CONF_TASK_ACT_PIN
  #define LWB_TASK_RESUMED        PIN_SET(LWB_CONF_TASK_ACT_PIN)
  #define LWB_TASK_SUSPENDED      PIN_CLR(LWB_CONF_TASK_ACT_PIN)
#else
  #define LWB_TASK_RESUMED     
  #define LWB_TASK_SUSPENDED  
#endif
/*---------------------------------------------------------------------------*/
#define LWB_T_SLOT_START(i)       ((LWB_CONF_T_SCHED + LWB_CONF_T_GAP) + \
                                   (LWB_CONF_T_DATA + LWB_CONF_T_GAP) * i)
#define LWB_DATA_RCVD             (glossy_get_rx_cnt() > 0)
#define RTIMER_CAPTURE            (t_now = rtimer_ext_now_hf())
#define RTIMER_ELAPSED            ((rtimer_ext_now_hf() - t_now) * 1000 / 3250)
/*---------------------------------------------------------------------------*/
#define LWB_SEND_SCHED() \
{\
  glossy_start(node_id, (uint8_t *)&schedule, schedule_len, \
               LWB_CONF_TX_CNT_SCHED, 1, 1);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED);\
  glossy_stop();\
}   
#define LWB_RCV_SCHED() \
{\
  glossy_start(0, (uint8_t *)&schedule, 0, LWB_CONF_TX_CNT_SCHED, 1, 1);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_SCHED + t_guard);\
  glossy_stop();\
}   
#define LWB_SEND_PACKET() \
{\
  glossy_start(node_id, (uint8_t*)&glossy_payload, payload_len, \
               LWB_CONF_TX_CNT_DATA, 0, 0);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_DATA);\
  glossy_stop();\
}
#define LWB_RCV_PACKET() \
{\
  glossy_start(0, (uint8_t*)&glossy_payload, 0, LWB_CONF_TX_CNT_DATA, 0, 0);\
  /* always use smallest guard time for data slots! */\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_DATA + LWB_CONF_T_GUARD);\
  glossy_stop();\
}
#define LWB_SEND_SRQ() \
{\
  glossy_start(node_id, (uint8_t*)&glossy_payload, payload_len, \
               LWB_CONF_TX_CNT_DATA, 0, 0);\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_CONT);\
  glossy_stop();\
}
#define LWB_RCV_SRQ() \
{\
  glossy_start(0, (uint8_t*)&glossy_payload, 0, LWB_CONF_TX_CNT_DATA, 0, 0);\
  /* always use smallest guard time for contention slot! */\
  LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_CONT + LWB_CONF_T_GUARD);\
  glossy_stop();\
}
/*---------------------------------------------------------------------------*/
/* suspend the LWB proto-thread until the rtimer reaches the specified time */
#define LWB_WAIT_UNTIL(time) \
{\
  rtimer_ext_schedule(LWB_CONF_RTIMER_ID, time, 0, callback_func);\
  LWB_TASK_SUSPENDED;\
  PT_YIELD(&lwb_pt);\
  LWB_TASK_RESUMED;\
}
/* same as LWB_WAIT_UNTIL, but use the LF timer to schedule the wake-up */
#define LWB_LF_WAIT_UNTIL(time) \
{\
  rtimer_ext_schedule(LWB_CONF_LF_RTIMER_ID, time, 0, callback_func);\
  LWB_TASK_SUSPENDED;\
  PT_YIELD(&lwb_pt);\
  LWB_TASK_RESUMED;\
  LWB_AFTER_DEEPSLEEP();\
}
#ifndef LWB_BEFORE_DEEPSLEEP
#define LWB_BEFORE_DEEPSLEEP() 
#endif /* LWB_PREPARE_DEEPSLEEP */
#ifndef LWB_AFTER_DEEPSLEEP
#define LWB_AFTER_DEEPSLEEP()
#endif /* LWB_AFTER_DEEPSLEEP */
/*---------------------------------------------------------------------------*/
static struct pt        lwb_pt;
static struct process*  pre_proc;
static struct process*  post_proc;
static lwb_sync_state_t sync_state;
static rtimer_ext_clock_t   rx_timestamp;
#if LWB_CONF_USE_LF_FOR_WAKEUP
static rtimer_ext_clock_t   rx_timestamp_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
static uint32_t         global_time;
static lwb_statistics_t stats = { 0 };
static uint8_t          urgent_stream_req = 0;
/* allocate memory in the SRAM (+1 to store the message length) */
static uint8_t          in_buffer_mem[LWB_CONF_IN_BUFFER_SIZE * 
                                      (LWB_CONF_MAX_DATA_PKT_LEN + 1)];  
static uint8_t          out_buffer_mem[LWB_CONF_OUT_BUFFER_SIZE * 
                                       (LWB_CONF_MAX_DATA_PKT_LEN + 1)]; 
FIFO16(in_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_IN_BUFFER_SIZE);
FIFO16(out_buffer, LWB_CONF_MAX_DATA_PKT_LEN + 1, LWB_CONF_OUT_BUFFER_SIZE);
/*---------------------------------------------------------------------------*/
/* store a received message in the incoming queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t 
lwb_in_buffer_put(const uint8_t * const data, uint8_t len)
{  
  if(!len) {
    return 0;
  }
  if(len > LWB_CONF_MAX_DATA_PKT_LEN) {
    len = LWB_CONF_MAX_DATA_PKT_LEN;
    DEBUG_PRINT_WARNING("received data packet is too big"); 
  }
  /* received messages will have the max. length LWB_CONF_MAX_DATA_PKT_LEN */
  uint32_t pkt_addr = fifo16_put(&in_buffer);
  if(FIFO16_ERROR != pkt_addr) {
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    /* copy the data into the queue */
    memcpy(next_msg, data, len);
    /* last byte holds the payload length */
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len;
    return 1;
  }
  stats.rxbuf_drop++;
  DEBUG_PRINT_WARNING("lwb rx queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns the message length in bytes */
uint8_t 
lwb_out_buffer_get(uint8_t* out_data)
{
  /* messages have the max. length LWB_CONF_MAX_DATA_PKT_LEN and are already
   * formatted according to glossy_payload_t */
  uint32_t pkt_addr = fifo16_get(&out_buffer);
  if(FIFO16_ERROR != pkt_addr) {
    /* assume pointers are always 16-bit */
    uint8_t* next_msg = (uint8_t*)((uint16_t)pkt_addr);
    /* check the length */
    uint8_t len = *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN);
    if(len > LWB_CONF_MAX_DATA_PKT_LEN) {
      DEBUG_PRINT_WARNING("invalid message length detected");
      len = LWB_CONF_MAX_DATA_PKT_LEN;  /* truncate */
    }
    memcpy(out_data, next_msg, len);
    return len;
  }
  DEBUG_PRINT_VERBOSE("lwb tx queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* puts a message into the outgoing queue, returns 1 if successful, 
 * 0 otherwise */
uint8_t
lwb_send_pkt(uint16_t recipient,
             uint8_t stream_id, 
             const uint8_t * const data, 
             uint8_t len)
{
  /* data has the max. length LWB_DATA_PKT_PAYLOAD_LEN, lwb header needs 
   * to be added before the data is inserted into the queue */
  if(len > LWB_DATA_PKT_PAYLOAD_LEN || !data) {
    DEBUG_PRINT_WARNING("invalid payload length");
    return 0;
  }
  uint32_t pkt_addr = fifo16_put(&out_buffer);
  if(FIFO16_ERROR != pkt_addr) {
    /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)(uint16_t)pkt_addr;  
    *(next_msg) = (uint8_t)recipient;   /* recipient L */  
    *(next_msg + 1) = recipient >> 8;   /* recipient H */  
    *(next_msg + 2) = stream_id; 
    *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) = len + LWB_CONF_HEADER_LEN;
    memcpy(next_msg + LWB_CONF_HEADER_LEN, data, len);
    return 1;
  }
  stats.txbuf_drop++;
  DEBUG_PRINT_VERBOSE("lwb tx queue full");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* copies the oldest received message in the queue into out_data and returns 
 * the message size (in bytes) */
uint8_t
lwb_rcv_pkt(uint8_t* out_data,
             uint16_t * const out_node_id, 
             uint8_t * const out_stream_id)
{ 
  if(!out_data) { return 0; }
  /* messages in the queue have the max. length LWB_CONF_MAX_DATA_PKT_LEN, 
   * lwb header needs to be stripped off; payload has max. length
   * LWB_DATA_PKT_PAYLOAD_LEN */
  uint32_t pkt_addr = fifo16_get(&in_buffer);
  if(FIFO16_ERROR != pkt_addr) {
   /* assume pointers are 16-bit */
    uint8_t* next_msg = (uint8_t*)(uint16_t)pkt_addr; 
    uint8_t msg_len = *(next_msg + LWB_CONF_MAX_DATA_PKT_LEN) -
                      LWB_CONF_HEADER_LEN;
    if(msg_len > LWB_CONF_MAX_DATA_PKT_LEN) {
      msg_len = LWB_CONF_MAX_DATA_PKT_LEN;
    }
    memcpy(out_data, next_msg + LWB_CONF_HEADER_LEN, msg_len);
    if(out_node_id) {
      /* cant just treat next_msg as 16-bit value due to misalignment */
      *out_node_id = (uint16_t)next_msg[1] << 8 | next_msg[0];
    }
    if(out_stream_id) {
      *out_stream_id = next_msg[2];
    }
    return msg_len;
  }
  DEBUG_PRINT_VERBOSE("lwb rx queue empty");
  return 0;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_rcv_buffer_state(void)
{
  return in_buffer.count;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_get_send_buffer_state(void)
{
  return out_buffer.count;
}
/*---------------------------------------------------------------------------*/
const lwb_statistics_t * const
lwb_get_stats(void)
{
  return &stats;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_request_stream(lwb_stream_req_t* stream_request, uint8_t urgent)
{
  if(!stream_request) { return 0; }
  if(urgent) {
    urgent_stream_req = stream_request->stream_id;
  }
  return lwb_stream_add(stream_request);
}
/*---------------------------------------------------------------------------*/
lwb_conn_state_t
lwb_get_state(void)
{
  if(sync_state < LWB_STATE_SYNCED) { return LWB_STATE_INIT; }
  else if(sync_state < LWB_STATE_MISSED) { return LWB_STATE_CONNECTED; }
  return LWB_STATE_CONN_LOST;
}
/*---------------------------------------------------------------------------*/
uint32_t 
lwb_get_time(rtimer_ext_clock_t* reception_time)
{
  if(reception_time) {
    *reception_time = rx_timestamp;
  }
  return global_time;
}
/*---------------------------------------------------------------------------*/
uint64_t
lwb_get_timestamp(void)
{
  /* convert to microseconds */
  uint64_t timestamp = (uint64_t)global_time * 1000000;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  if(sync_state <= LWB_STATE_SYNCED_2) {
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    return timestamp + /* convert to microseconds */
           (rtimer_ext_now_hf() - rx_timestamp) * 1000000 / RTIMER_EXT_SECOND_HF;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  }
  /* not synced! */
  return timestamp +
         (rtimer_ext_now_lf() - rx_timestamp_lf) * 1000000 / RTIMER_EXT_SECOND_LF;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
}
/*---------------------------------------------------------------------------*/
/* update the sync state machine based on the schedule slot event */
lwb_sync_state_t
lwb_update_sync_state(const lwb_sync_state_t curr_state,
                      const lwb_schedule_t const * sched)
{
  lwb_sync_event_t evt;
  if(glossy_is_t_ref_updated()) {
    if(LWB_SCHED_IS_1ST(sched)) {
      evt = EVT_1ST_SCHED_RCVD;
    } else {
      evt = EVT_2ND_SCHED_RCVD;
    }
  } else {
    evt = EVT_SCHED_LWB_STATE_MISSED;
  }
  return next_state[evt][sync_state];
}
/*---------------------------------------------------------------------------*/
/**
 * @brief thread of the host node
 * @note runs in an interrupt context; don't use switch-case statements here
 */
PT_THREAD(lwb_thread_host(rtimer_ext_t *rt)) 
{  
  /* all variables must be static */
  static lwb_schedule_t schedule;
  static rtimer_ext_clock_t t_start; 
  static rtimer_ext_clock_t t_now;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_ext_clock_t t_start_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  /* constant guard time for the host */
  static uint8_t slot_idx;
  static uint8_t streams_to_update[LWB_CONF_MAX_DATA_SLOTS];
  static uint8_t schedule_len,
                 payload_len;
  static uint8_t rcvd_data_pkts;
  static const void* callback_func = lwb_thread_host;

  /* note: all statements above PT_BEGIN() will be executed each time the 
   * protothread is scheduled */
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
    
  memset(&schedule, 0, sizeof(schedule));
  
  /* initialization specific to the host node */
  schedule_len = lwb_sched_init(&schedule);
  sync_state = LWB_STATE_SYNCED;  /* the host is always 'synced' */
  
  rtimer_ext_reset();
  rt->time = 0;
  
  while(1) {
#if LWB_CONF_T_PREPROCESS
    if(pre_proc) {
      process_poll(pre_proc);
    }
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS *
                      RTIMER_EXT_SECOND_LF / 1000);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(rt->time + LWB_CONF_T_PREPROCESS *
                   RTIMER_EXT_SECOND_HF / 1000)
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#endif /* LWB_CONF_T_PREPROCESS */

#if LWB_CONF_USE_LF_FOR_WAKEUP 
    t_start_lf = rt->time; 
    rt->time = rtimer_ext_now_hf();
#endif  /* LWB_CONF_USE_LF_FOR_WAKEUP */
    /* set the start time of the round to the expiration time of the last 
     * scheduled timeout */
    t_start = rt->time;
        
    /* --- COMMUNICATION ROUND STARTS --- */
    
    LWB_SCHED_SET_AS_1ST(&schedule);          /* mark this schedule as first */
    LWB_SEND_SCHED();            /* send the previously computed schedule */

    global_time  = schedule.time;
    rx_timestamp = glossy_get_t_ref();

    slot_idx = 0;     /* reset the packet counter */

    /* uncompress the schedule */
#if LWB_CONF_SCHED_COMPRESS
    lwb_sched_uncompress((uint8_t*)schedule.slot, 
                         LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_CONF_SCHED_COMPRESS */
    
    /* --- S-ACK SLOT --- */
    
    if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {
      payload_len = lwb_sched_prepare_sack(&glossy_payload.sack_pkt); 
      /* wait for the slot to start */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(0));            
      LWB_SEND_PACKET();   /* transmit s-ack */
      DEBUG_PRINT_VERBOSE("S-ACK sent");
      slot_idx++;   /* increment the packet counter */
    } else {
      DEBUG_PRINT_VERBOSE("no S-ACK slot");
    }
         
    /* --- DATA SLOTS --- */
    
    rcvd_data_pkts = 0;    /* number of received data packets in this round */
    if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
      static uint8_t i = 0;
      for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
        streams_to_update[i] = LWB_INVALID_STREAM_ID;
        /* is this our slot? Note: slots assigned to node ID 0 always belong 
         * to the host */
        if(schedule.slot[i] == 0 || schedule.slot[i] == node_id) {
          /* send a data packet (if there is any) */
          payload_len = lwb_out_buffer_get(glossy_payload.raw_data);
          if(payload_len) { 
            /* note: stream ID is irrelevant here */
            /* wait until the data slot starts */
            LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx));  
            LWB_SEND_PACKET();
            DEBUG_PRINT_VERBOSE("data packet sent (%ub)", payload_len);
          } else {
            DEBUG_PRINT_VERBOSE("no data to send!");
          }
        } else {        
          /* wait until the data slot starts */
          LWB_DATA_SLOT_STARTS;
          LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) -
                         LWB_CONF_T_GUARD);
          LWB_RCV_PACKET();  /* receive a data packet */
          payload_len = glossy_get_payload_len();
          if(LWB_DATA_RCVD && payload_len) {
            /* measure the time it takes to process the received message */
            RTIMER_CAPTURE;   
            if(glossy_payload.data_pkt.recipient == node_id || 
               glossy_payload.data_pkt.recipient == LWB_RECIPIENT_SINK ||
               glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST) {
              /* is it a stream request? (piggyback on data packet) */
              if(LWB_INVALID_STREAM_ID == glossy_payload.data_pkt.stream_id) {
                DEBUG_PRINT_VERBOSE("piggyback stream request from node %u", 
                                 glossy_payload.srq_pkt.id);
                lwb_sched_proc_srq((lwb_stream_req_t*)
                                   &glossy_payload.raw_data[3]);
              } else 
              {
                streams_to_update[i] = glossy_payload.data_pkt.stream_id;
                DEBUG_PRINT_VERBOSE("data received (s=%u.%u l=%u)", 
                                    schedule.slot[i], 
                                    glossy_payload.data_pkt.stream_id, 
                                    payload_len);
                /* replace target node ID by sender node ID */
                glossy_payload.data_pkt.recipient = schedule.slot[i];
                lwb_in_buffer_put(glossy_payload.raw_data, 
                                  payload_len);
              }
            } else {
              DEBUG_PRINT_VERBOSE("packet dropped, target_id != node_id");
            }
            /* update statistics */
            stats.rx_total += payload_len;
            stats.pkt_cnt++;
            rcvd_data_pkts++;
            /* measure time (must always be smaller than LWB_CONF_T_GAP!) */
            stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_proc_max);
          } else {
            DEBUG_PRINT_VERBOSE("no data received from node %u", 
                                schedule.slot[i]);
          }
        }
      }
    }
    
    /* --- CONTENTION SLOT --- */
    
    if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
      /* wait until the slot starts, then receive the packet */
      LWB_WAIT_UNTIL(t_start + LWB_T_SLOT_START(slot_idx) - LWB_CONF_T_GUARD);
      LWB_RCV_SRQ();
      if(LWB_DATA_RCVD) {
        LWB_REQ_DETECTED;
        lwb_sched_proc_srq(&glossy_payload.srq_pkt);
      }
    }

    /* compute the new schedule */
    RTIMER_CAPTURE;
    schedule_len = lwb_sched_compute(&schedule, 
                                     streams_to_update, 
                                     lwb_get_send_buffer_state());
    stats.t_sched_max = MAX((uint16_t)RTIMER_ELAPSED, stats.t_sched_max);

    LWB_WAIT_UNTIL(t_start + LWB_CONF_T_SCHED2_START);
    LWB_SEND_SCHED();    /* send the schedule for the next round */
    
    /* --- COMMUNICATION ROUND ENDS --- */
    /* time for other computations */
    
    /* print out some stats */
    DEBUG_PRINT_INFO("t=%lu ts=%u td=%u dp=%u p=%u per=%u fsr=%u", 
                     global_time,
                     stats.t_sched_max,
                     stats.t_proc_max,
                     rcvd_data_pkts,
                     stats.pkt_cnt,
                     glossy_get_per(),
                     glossy_get_fsr());
  
    /* poll the other processes to allow them to run after the LWB task was 
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    debug_print_poll();
    if(post_proc) {
      /* will be executed before the debug print task */
      process_poll(post_proc);    
    }
    
    /* suspend this task and wait for the next round */
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_start_lf + 
                      (rtimer_ext_clock_t)schedule.period * RTIMER_EXT_SECOND_LF / 
                      LWB_CONF_TIME_SCALE - LWB_CONF_T_PREPROCESS *
                      RTIMER_EXT_SECOND_LF / 1000);
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_start + 
                   (rtimer_ext_clock_t)schedule.period * RTIMER_EXT_SECOND_HF /
                   LWB_CONF_TIME_SCALE - 
                   LWB_CONF_T_PREPROCESS * RTIMER_EXT_SECOND_HF / 1000);
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }
  
  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
/**
 * @brief declaration of the protothread (source node)
 * @note runs in an interrupt context; don't use switch-case statements here
 */
PT_THREAD(lwb_thread_src(rtimer_ext_t *rt)) 
{  
  /* all variables must be static */
  static glossy_payload_t glossy_payload;                   /* packet buffer */
  static lwb_schedule_t schedule;
  static rtimer_ext_clock_t t_ref,
                        t_ref_last;
#if LWB_CONF_USE_LF_FOR_WAKEUP
  static rtimer_ext_clock_t t_ref_lf;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  static rtimer_ext_clock_t t_now;
  static uint32_t t_guard;                  /* 32-bit is enough for t_guard! */
  static int32_t  drift;
  static uint16_t period_last;
  static uint8_t  slot_idx;
  static uint8_t  relay_cnt_first_rx;
  static uint8_t  payload_len;
  static uint8_t  rounds_to_wait;
  static const void* callback_func = lwb_thread_src;
  
  PT_BEGIN(&lwb_pt);   /* declare variables before this statement! */
  
  memset(&schedule, 0, sizeof(schedule)); 
  
  /* initialization specific to the source node */
  lwb_stream_init();
  sync_state  = LWB_STATE_BOOTSTRAP;
  period_last = LWB_CONF_SCHED_PERIOD_MIN;
  
  while(1) {
      
#if LWB_CONF_T_PREPROCESS
    if(pre_proc) {
      process_poll(pre_proc);
    }
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(rt->time + 
                      LWB_CONF_T_PREPROCESS * RTIMER_EXT_SECOND_LF / 1000);
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(rt->time + 
                   LWB_CONF_T_PREPROCESS * RTIMER_EXT_SECOND_HF / 1000)
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
#endif /* LWB_CONF_T_PREPROCESS */
    
    /* --- COMMUNICATION ROUND STARTS --- */
    
#if LWB_CONF_USE_LF_FOR_WAKEUP
    rt->time = rtimer_ext_now_hf();        /* overwrite LF with HF timestamp */
    t_ref = rt->time + t_guard;        /* in case the schedule is missed */
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
    if(sync_state == LWB_STATE_BOOTSTRAP) {
BOOTSTRAP:
      DEBUG_PRINT_MSG_NOW("BOOTSTRAP ");
      stats.bootstrap_cnt++;
      lwb_stream_rejoin();  /* rejoin all (active) streams */
      /* synchronize first! wait for the first schedule... */
      do {
        LWB_RCV_SCHED();
        if((rtimer_ext_now_hf() - t_ref) > LWB_CONF_T_SILENT) {
          DEBUG_PRINT_MSG_NOW("communication timeout, going to sleep...");
          stats.sleep_cnt++;
          LWB_BEFORE_DEEPSLEEP();
          LWB_LF_WAIT_UNTIL(rtimer_ext_now_lf() + LWB_CONF_T_DEEPSLEEP);
          t_ref = rtimer_ext_now_hf();
          /* alternative: implement a host failover policy */
        }
      } while(!glossy_is_t_ref_updated() || !LWB_SCHED_IS_1ST(&schedule));
      /* schedule received! */
    } else {
      LWB_RCV_SCHED();  
    }
    /* compute new sync state and update t_guard */
    sync_state = lwb_update_sync_state(sync_state, &schedule);
    t_guard    = guard_time[sync_state];            /* adjust the guard time */
    if(LWB_STATE_BOOTSTRAP == sync_state) {
      goto BOOTSTRAP;         /* wrong schedule received > back to bootstrap */
    }
    
    LWB_SCHED_SET_AS_2ND(&schedule);    /* clear the last bit of 'period' */
    if(glossy_is_t_ref_updated()) {
      /* HF timestamp of first RX; subtract a constant offset */
      t_ref = glossy_get_t_ref() - LWB_CONF_T_REF_OFS;           
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* estimate t_ref_lf by subtracting the elapsed time since t_ref: */
      rtimer_ext_clock_t hf_now;
      rtimer_ext_now(&hf_now, &t_ref_lf);
      t_ref_lf -= (uint32_t)(hf_now - t_ref) / (uint32_t)RTIMER_EXT_HF_LF_RATIO;
      rx_timestamp_lf = t_ref_lf;
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      global_time = schedule.time;
      rx_timestamp = t_ref;
    } else {
      DEBUG_PRINT_WARNING("schedule missed");
      /* we can only estimate t_ref and t_ref_lf */
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* since HF clock was off, we need a new timestamp; subtract a const.
       * processing offset to adjust (if needed) */
      t_ref_lf += ((rtimer_ext_clock_t)period_last * RTIMER_EXT_SECOND_LF +
                  ((int32_t)period_last * stats.drift / 256)) /
                  LWB_CONF_TIME_SCALE;
      /* do NOT update t_ref here! */
  #else  /* LWB_CONF_USE_LF_FOR_WAKEUP */
      t_ref += (rtimer_ext_clock_t)period_last *
               (RTIMER_EXT_SECOND_HF + stats.drift / 16) / LWB_CONF_TIME_SCALE;
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
      /* don't update schedule.time here! */
    }

    /* permission to participate in this round? */
    if(sync_state == LWB_STATE_SYNCED || sync_state == LWB_STATE_UNSYNCED) {
        
      static uint8_t i;  /* must be static */
      slot_idx = 0;   /* reset the packet counter */
      relay_cnt_first_rx = glossy_get_relay_cnt();
#if LWB_CONF_SCHED_COMPRESS
      lwb_sched_uncompress((uint8_t*)schedule.slot, 
                           LWB_SCHED_N_SLOTS(&schedule));
#endif /* LWB_CONF_SCHED_COMPRESS */
      
      /* --- S-ACK SLOT --- */

      if(LWB_SCHED_HAS_SACK_SLOT(&schedule)) {   
        /* wait for the slot to start */
        LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(0) - LWB_CONF_T_GUARD);
        LWB_RCV_PACKET();                 /* receive s-ack */
        if(LWB_DATA_RCVD) {
          i = 0;
          DEBUG_PRINT_VERBOSE("S-ACK packet received (%u stream acks)", 
                              glossy_payload.sack_pkt.n_extra + 1);
          do {
            /* note: potential cause of problems (but: structure glossy_payload
             * should be aligned to an even address!) */
            if(*(uint16_t*)(glossy_payload.raw_data + (i * 4)) == node_id) {     
              uint8_t stream_id = *(uint8_t*)(glossy_payload.raw_data + 
                                  (i * 4 + 2));
              stats.t_slot_last = schedule.time;
              rounds_to_wait = 0;
              if(lwb_stream_update_state(stream_id)) {
                DEBUG_PRINT_INFO("S-ACK received for stream %u (joined)", 
                                 stream_id);
              } else {
                DEBUG_PRINT_INFO("S-ACK received for stream %u (removed)", 
                                 stream_id);
              }
            } 
            i++;
          } while(i <= glossy_payload.sack_pkt.n_extra);
        } else {
          DEBUG_PRINT_VERBOSE("no data received in SACK SLOT");
        }
        slot_idx++;   /* increment the packet counter */
      }
      
      /* --- DATA SLOTS --- */

      if(LWB_SCHED_HAS_DATA_SLOT(&schedule)) {
        for(i = 0; i < LWB_SCHED_N_SLOTS(&schedule); i++, slot_idx++) {
          if(schedule.slot[i] == node_id) {
            stats.t_slot_last = schedule.time;
            /* this is our data slot, send a data packet */
            payload_len = 0;
            /* is there an 'urgent' stream request? -> if so, piggyback it 
             * onto the data packet */
            if(urgent_stream_req) {
              glossy_payload.data_pkt.recipient = LWB_RECIPIENT_SINK;
              glossy_payload.data_pkt.stream_id = LWB_INVALID_STREAM_ID;
              if(lwb_stream_prepare_req((lwb_stream_req_t*)
                                        &glossy_payload.raw_data[3], 
                                        urgent_stream_req)) {
                payload_len = sizeof(lwb_stream_req_t) + 3;
              }
              DEBUG_PRINT_VERBOSE("piggyback stream request prepared");
            } else {
              /* fetch the next 'ready-to-send' packet */
              payload_len = lwb_out_buffer_get(glossy_payload.raw_data);
            }
            if(payload_len) {
              LWB_DATA_IND;
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_PACKET();
              DEBUG_PRINT_INFO("data packet sent (%ub)", payload_len);
            } else {
              DEBUG_PRINT_VERBOSE("no message to send (data slot ignored)");
            }
          } else
          {
            /* receive a data packet */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) - 
                           LWB_CONF_T_GUARD);
            LWB_RCV_PACKET();
            payload_len = glossy_get_payload_len();
            /* process the received data */
            if(LWB_DATA_RCVD && payload_len) {
              /* measure the time it takes to process the received data */
              RTIMER_CAPTURE;     
              /* only forward packets that are destined for this node */
              if(glossy_payload.data_pkt.recipient == node_id || 
                glossy_payload.data_pkt.recipient == LWB_RECIPIENT_BROADCAST) {
                DEBUG_PRINT_VERBOSE("data received");
                /* replace target node ID by sender node ID */
                glossy_payload.data_pkt.recipient = schedule.slot[i];
                lwb_in_buffer_put(glossy_payload.raw_data, 
                                  payload_len);
              } else {
                DEBUG_PRINT_VERBOSE("received packet dropped");      
              }
              /* must always be smaller than LWB_CONF_T_GAP */
              stats.t_proc_max = MAX((uint16_t)RTIMER_ELAPSED, 
                                     stats.t_proc_max); 
            } /* else: no data received */   
            stats.rx_total += payload_len;
            stats.pkt_cnt++;
          }
        }
      }

      /* --- CONTENTION SLOT --- */

      /* is there a contention slot in this round? */
      if(LWB_SCHED_HAS_CONT_SLOT(&schedule)) {
        /* does this node have pending stream requests? */
        if(LWB_STREAM_REQ_PENDING) {
          if(!rounds_to_wait) {              /* allowed to send the request? */
            /* try to join, send the stream request */
            if(lwb_stream_prepare_req(&glossy_payload.srq_pkt, 
                                      LWB_INVALID_STREAM_ID)) {
      #if LWB_CONF_MAX_CONT_BACKOFF
              /* wait between 1 and LWB_CONF_MAX_CONT_BACKOFF rounds */
              rounds_to_wait = (random_rand() >> 1) % 
                               LWB_CONF_MAX_CONT_BACKOFF + 1;
      #endif /* LWB_CONF_MAX_CONT_BACKOFF */
              payload_len = sizeof(lwb_stream_req_t);
              /* wait until the contention slot starts */
              LWB_REQ_IND;
              LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx));
              LWB_SEND_SRQ();  
              DEBUG_PRINT_INFO("request for stream %u sent", 
                               glossy_payload.srq_pkt.stream_id);
            } /* else {
              DEBUG_PRINT_ERROR("failed to prepare stream request packet");
            }*/
          } else {
            DEBUG_PRINT_VERBOSE("must wait %u rounds", rounds_to_wait);
            /* keep waiting and just relay incoming packets */
            rounds_to_wait--;       /* decrease the number of rounds to wait */
            /* wait until the contention slot starts */
            LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) -
                           LWB_CONF_T_GUARD);
            LWB_RCV_SRQ();
          }
          DEBUG_PRINT_VERBOSE("pending stream requests: 0x%x", 
                              LWB_STREAM_REQ_PENDING);
        } else {
          /* no request pending -> just receive / relay packets */
          LWB_WAIT_UNTIL(t_ref + LWB_T_SLOT_START(slot_idx) -
                         LWB_CONF_T_GUARD);
          LWB_RCV_SRQ();
        }
      }
    }
    
    /* --- 2ND SCHEDULE --- */

    LWB_WAIT_UNTIL(t_ref + LWB_CONF_T_SCHED2_START - t_guard);
    LWB_RCV_SCHED();
  
    /* compute new sync state and update t_guard */
    sync_state = lwb_update_sync_state(sync_state, &schedule);
    t_guard    = guard_time[sync_state];            /* adjust the guard time */
    if(LWB_STATE_BOOTSTRAP == sync_state) {
      goto BOOTSTRAP;         /* wrong schedule received > back to bootstrap */
    }
    
    /* --- COMMUNICATION ROUND ENDS --- */    
    /* time for other computations */

    /* check joining state */
    if(LWB_STREAMS_ACTIVE && ((schedule.time - stats.t_slot_last) > 
                              (LWB_CONF_SCHED_PERIOD_MAX << 2))) {
      DEBUG_PRINT_WARNING("no-slot timeout, requesting new stream...");
      lwb_stream_rejoin();  /* re-join (all streams) */
      stats.t_slot_last = schedule.time;
    }
    
    /* estimate the clock drift */
#if (LWB_CONF_TIME_SCALE == 1)  /* only calc drift if time scale is not used */
  #if LWB_CONF_USE_LF_FOR_WAKEUP
    /* t_ref can't be used in this case -> use t_ref_lf instead */
    drift = ((int32_t)((t_ref_lf - t_ref_last) - ((int32_t)period_last *
                       RTIMER_EXT_SECOND_LF)) << 8) / (int32_t)period_last;
    t_ref_last = t_ref_lf;     
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    drift = ((int32_t)((t_ref - t_ref_last) - ((int32_t)period_last *
                       RTIMER_EXT_SECOND_HF)) << 4) / (int32_t)period_last;
    t_ref_last = t_ref; 
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */

    if(sync_state <= LWB_STATE_MISSED) {
  #if LWB_CONF_USE_LF_FOR_WAKEUP
      /* convert LWB_CONF_MAX_CLOCK_DEV into clock ticks per second */
      if((drift < (int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_EXT_SECOND_LF * 256 /
                            1000000)) &&
         (drift > -(int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_EXT_SECOND_LF * 256 /
                             1000000))) {
  #else /* LWB_CONF_USE_LF_FOR_WAKEUP */
      if((drift < (int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_EXT_SECOND_HF * 16 /
                            1000000)) &&
         (drift > -(int16_t)(LWB_CONF_MAX_CLOCK_DEV * RTIMER_EXT_SECOND_HF * 16 /
                             1000000))) {
  #endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
        stats.drift = (stats.drift + drift) / 2;  /* low-pass filter */
      }
    }
#endif /* LWB_CONF_TIME_SCALE */

    period_last = schedule.period;
    if(sync_state > LWB_STATE_SYNCED_2) {
      stats.unsynced_cnt++;
    }
    /* print out some stats (note: takes approx. 2ms to compose this string) */
    DEBUG_PRINT_INFO("%s %lu T=%u n=%u s=%u tp=%u p=%u r=%u b=%u "
                     "u=%u dr=%d per=%u fsr=%u",
                     lwb_sync_state_to_string[sync_state], 
                     schedule.time, 
                     schedule.period, 
                     LWB_SCHED_N_SLOTS(&schedule), 
                     LWB_STREAMS_ACTIVE, 
                     stats.t_proc_max,
                     stats.pkt_cnt,
                     relay_cnt_first_rx,
                     stats.bootstrap_cnt, 
                     stats.unsynced_cnt, 
                     stats.drift, /* in clock ticks per second (x16 or x256) */
                     glossy_get_per(),
                     glossy_get_fsr());

    /* check processing time */
    if(stats.t_proc_max >= LWB_CONF_T_GAP) {
      DEBUG_PRINT_ERROR("LWB gap time overrun detected");
    }

    /* erase the schedule (slot allocations only) */
    memset(&schedule.slot, 0, sizeof(schedule.slot));

    /* poll the other processes to allow them to run after the LWB task was
     * suspended (note: the polled processes will be executed in the inverse
     * order they were started/created) */
    debug_print_poll();
    if(post_proc) {
      process_poll(post_proc);
    }
    
#if LWB_CONF_USE_LF_FOR_WAKEUP
    LWB_LF_WAIT_UNTIL(t_ref_lf + 
                      ((rtimer_ext_clock_t)schedule.period * RTIMER_EXT_SECOND_LF + 
                       (((int32_t)schedule.period * stats.drift) / 256)) /
                      LWB_CONF_TIME_SCALE - 
                      t_guard / (uint32_t)RTIMER_EXT_HF_LF_RATIO - 
                      LWB_CONF_T_PREPROCESS * RTIMER_EXT_SECOND_LF / 1000);
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
    LWB_WAIT_UNTIL(t_ref + 
                   ((rtimer_ext_clock_t)schedule.period * RTIMER_EXT_SECOND_HF +
                    (((int32_t)schedule.period * stats.drift) / 16)) /
                   LWB_CONF_TIME_SCALE - t_guard - 
                   LWB_CONF_T_PREPROCESS * RTIMER_EXT_SECOND_HF / 1000);
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
  }

  PT_END(&lwb_pt);
}
/*---------------------------------------------------------------------------*/
void
lwb_pause(void)
{
  rtimer_ext_stop(LWB_CONF_RTIMER_ID);    /* cancel the rtimer */
  rtimer_ext_stop(LWB_CONF_LF_RTIMER_ID);
}
/*---------------------------------------------------------------------------*/
void
lwb_resume(void)
{
#if LWB_CONF_USE_LF_FOR_WAKEUP
  rtimer_ext_clock_t start_time = rtimer_ext_now_lf() + RTIMER_EXT_SECOND_LF / 10;
  rtimer_ext_id_t timer_id = LWB_CONF_LF_RTIMER_ID;
#else /* LWB_CONF_USE_LF_FOR_WAKEUP */
  rtimer_ext_clock_t start_time = rtimer_ext_now_hf() + RTIMER_EXT_SECOND_HF / 10;
  rtimer_ext_id_t timer_id = LWB_CONF_RTIMER_ID;
#endif /* LWB_CONF_USE_LF_FOR_WAKEUP */
    
  if(node_id == HOST_ID) {
    /* note: must add at least some clock ticks! */
    rtimer_ext_schedule(timer_id, start_time, 0, lwb_thread_host);
  } else {
    rtimer_ext_schedule(timer_id, start_time, 0, lwb_thread_src);
  }
}
/*---------------------------------------------------------------------------*/
/* define the process control block */
PROCESS(lwb_process, "Communication Task (LWB)");
/*---------------------------------------------------------------------------*/
/* define the body (protothread) of a process */
PROCESS_THREAD(lwb_process, ev, data) 
{
  PROCESS_BEGIN();
  
  /* pass the start addresses of the memory blocks holding the queues */
  fifo16_init(&in_buffer, (uint16_t)in_buffer_mem);
  fifo16_init(&out_buffer, (uint16_t)out_buffer_mem); 
  
#ifdef LWB_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(LWB_CONF_TASK_ACT_PIN);
  PIN_CLR(LWB_CONF_TASK_ACT_PIN);
#endif /* LWB_CONF_TASK_ACT_PIN */
  
  PT_INIT(&lwb_pt); /* initialize the protothread */

  lwb_resume();

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
void
lwb_start(struct process* pre_lwb_proc, struct process *post_lwb_proc)
{
  pre_proc  = (struct process*)pre_lwb_proc;
  post_proc = (struct process*)post_lwb_proc;
  
  DEBUG_PRINT_INFO("Starting '%s'\r\n", lwb_process.name);
  DEBUG_PRINT_INFO("T=%ums T_s=%ums T_d=%ums T_c=%ums l=%ub N_s=%u N_tx=%u N_h=%u\r\n",
                   (uint16_t)RTIMER_EXT_HF_TO_MS(LWB_T_ROUND_MAX),
                   (uint16_t)RTIMER_EXT_HF_TO_MS(LWB_CONF_T_SCHED),
                   (uint16_t)RTIMER_EXT_HF_TO_MS(LWB_CONF_T_DATA),
                   (uint16_t)RTIMER_EXT_HF_TO_MS(LWB_CONF_T_CONT),
                   LWB_CONF_MAX_DATA_PKT_LEN, 
                   LWB_CONF_MAX_DATA_SLOTS, 
                   LWB_CONF_TX_CNT_DATA, 
                   LWB_CONF_MAX_HOPS);
  /* check validity of round length */
  if((LWB_CONF_T_SCHED2_START + LWB_CONF_T_SCHED) > 
     (RTIMER_EXT_SECOND_HF / LWB_CONF_TIME_SCALE)) {
    DEBUG_PRINT_INFO("WARNING: LWB_CONF_T_SCHED2_START > min round period\r\n");
  }
  if(LWB_CONF_T_SCHED2_START < (LWB_T_ROUND_MAX / LWB_CONF_TIME_SCALE)) {
    DEBUG_PRINT_INFO("WARNING: LWB_CONF_T_SCHED2_START < LWB_T_ROUND_MAX!");
  }
  process_start(&lwb_process, NULL);
}
/*---------------------------------------------------------------------------*/
