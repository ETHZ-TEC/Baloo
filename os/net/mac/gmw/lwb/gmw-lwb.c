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
 * Author:  Jonas Baechli
 *          Reto Da Forno
 */

#include "gmw-lwb.h"
#include "node-id.h"
#include "random.h"
#include "debug-print.h"
#include "fifo16.h"

/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
typedef enum {
  LWB_EVENT_RCVD_1ST_SCHED = 0,
  LWB_EVENT_RCVD_2ND_SCHED,
  EVT_SCHED_LWB_SYNC_STATE_MISSED,
  NUM_OF_SYNC_EVENTS
} lwb_sync_event_t;

typedef struct {
  uint8_t    len;
  lwb_data_t data;
} lwb_queue_elem_t;

typedef struct {
  /* index in the stream list is the ID, no need to store the stream ID here */
  lwb_stream_state_t state : 8;    /* 1 byte only */
  uint16_t           ipi;          /* inter-packet interval */
} lwb_stream_t;

typedef enum lwb_host_round {
  LWB_ROUND_TYPE_MAIN,        /* main round */
  LWB_ROUND_TYPE_SCHED_ONLY,  /* send schedule only (2nd schedule)*/
} lwb_round_type_t;
/*---------------------------------------------------------------------------*/
static uint8_t input_queue_buffer[LWB_CONF_INPUT_QUEUE_SIZE * sizeof(lwb_queue_elem_t)];
static uint8_t output_queue_buffer[LWB_CONF_OUTPUT_QUEUE_SIZE * sizeof(lwb_queue_elem_t)];
FIFO16(input_queue,  sizeof(lwb_queue_elem_t), LWB_CONF_INPUT_QUEUE_SIZE);
FIFO16(output_queue, sizeof(lwb_queue_elem_t), LWB_CONF_OUTPUT_QUEUE_SIZE);
static struct process*     pre_proc;
static struct process*     post_proc;
static lwb_round_type_t    current_round_type;
static gmw_control_t       control;

/* --- variables for the HOST node --- */
static gmw_protocol_impl_t host_impl;
static uint8_t             slot_streams[LWB_MAX_DATA_SLOTS + 1];

/* --- variables for the SOURCE node --- */
static gmw_protocol_impl_t src_impl;
static lwb_stream_t        streams[LWB_CONF_MAX_N_STREAMS_PER_NODE];
static lwb_sync_state_t    sync_state;
static uint8_t             stream_request_pending;
static uint8_t             rounds_to_wait;
static uint16_t            previous_periods[2];
static uint32_t            sync_time;
static uint64_t            sync_timestamp;
/*---------------------------------------------------------------------------*/
/**
 * @brief state transition matrix for the source node; the next state can be
 * retrieved from the current state (column) and the latest event (row)
 * @note undefined transitions force the SM to go back to bootstrap
 */
static const lwb_sync_state_t next_state[NUM_OF_SYNC_EVENTS][NUM_OF_LWB_SYNC_STATES] =
{
  { LWB_SYNC_STATE_QSYNCED,   LWB_SYNC_STATE_SYNCED,    LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_SYNCED,    LWB_SYNC_STATE_SYNCED,    LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_SYNCED    },
  { LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_QSYNCED,   LWB_SYNC_STATE_SYNCED2,   LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_SYNCED2,   LWB_SYNC_STATE_BOOTSTRAP },
  { LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_BOOTSTRAP, LWB_SYNC_STATE_MISSED,    LWB_SYNC_STATE_UNSYNCED,  LWB_SYNC_STATE_UNSYNCED,  LWB_SYNC_STATE_UNSYNCED2, LWB_SYNC_STATE_BOOTSTRAP }
};
/* mapping from LWB states to GMW superstates */
static const gmw_sync_state_t gmw_superstate[NUM_OF_LWB_SYNC_STATES] =
{
  GMW_BOOTSTRAP,    /* LWB_SYNC_STATE_BOOTSTRAP,    */
  GMW_SUSPENDED,    /* LWB_SYNC_STATE_QSYNCED,      */
  GMW_RUNNING,      /* LWB_SYNC_STATE_SYNCED,       */
  GMW_SUSPENDED,    /* LWB_SYNC_STATE_SYNCED2,      */
  GMW_SUSPENDED,    /* LWB_SYNC_STATE_MISSED,       */
  GMW_RUNNING,      /* LWB_SYNC_STATE_UNSYNCED,     */
  GMW_SUSPENDED,    /* LWB_SYNC_STATE_UNSYNCED2,    */
};
/*---------------------------------------------------------------------------*/
/*------------------------------ prototypes ---------------------------------*/
/*---------------------------------------------------------------------------*/
void    lwb_init(void);
uint8_t output_queue_get(uint8_t* out_data);
uint8_t input_queue_put(const uint8_t * const data, uint8_t len);
uint8_t stream_prepare_req(lwb_stream_req_t* const out_srq);
uint8_t stream_update_state(uint16_t stream_id);
/*---------------------------------------------------------------------------*/
/*---------------------------- main interface -------------------------------*/
/*---------------------------------------------------------------------------*/
void
lwb_start(struct process* pre_lwb_proc, struct process *post_lwb_proc)
{
  /* pass the start addresses of the memory blocks holding the queues */
  fifo16_init(&input_queue, (uint16_t)input_queue_buffer);
  fifo16_init(&output_queue, (uint16_t)output_queue_buffer);

  pre_proc  = pre_lwb_proc;
  post_proc = post_lwb_proc;

  lwb_init();   /* init data structures and callbacks */

  DEBUG_PRINT_INFO("LWB initialized");

  gmw_start(pre_lwb_proc, post_lwb_proc, &host_impl, &src_impl);
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_send_pkt(uint16_t recipient,
             uint8_t stream_id,
             const uint8_t* const data,
             uint8_t len)
{
  /* data has the max. length LWB_MAX_PAYLOAD_LEN, LWB header needs
   * to be added before the data is inserted into the queue */
  if(len == 0 || len > LWB_MAX_PAYLOAD_LEN || !data) {
    return 0;
  }
  uint16_t addr = fifo16_put(&output_queue);
  if(FIFO16_ERROR != addr) {
    lwb_queue_elem_t* elem         = (lwb_queue_elem_t*)addr;
    elem->data.header.recipient_id = recipient;
    elem->data.header.type         = LWB_PACKET_TYPE_DATA;
    elem->data.header.stream_id    = stream_id;
    elem->len                      = len + sizeof(lwb_header_t);
    memcpy(elem->data.payload, data, len);
    return 1;   /* success */
  }
  return 0;     /* failed */
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_rcv_pkt(uint8_t* out_data,
            uint16_t* const out_sender_id,
            uint8_t* const out_stream_id)
{
  if(!out_data) {
    return 0;   /* invalid argument */
  }
  /* messages in the queue have the max. length LWB_MAX_DATA_PKT_LEN,
   * lwb header needs to be stripped off; payload has max. length
   * LWB_MAX_PAYLOAD_LEN */
  uint16_t addr = fifo16_get(&input_queue);
  if(FIFO16_ERROR != addr) {
    /* assume pointers are 16-bit */
    lwb_queue_elem_t* elem = (lwb_queue_elem_t*)addr;
    /* make sure the message length doesn't exceed the buffer limits */
    if(elem->len > LWB_MAX_DATA_PKT_LEN) {
      elem->len = LWB_MAX_DATA_PKT_LEN;     /* truncate */
    }
    memcpy(out_data, elem->data.payload, elem->len);
    if(out_sender_id) {
      *out_sender_id = elem->data.header.recipient_id;
    }
    if(out_stream_id) {
      *out_stream_id = elem->data.header.stream_id;
    }
    return elem->len - sizeof(lwb_header_t);
  }
  return 0;   /* queue empty */
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_rcv_buffer_state(void)
{
  return input_queue.count;
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_send_buffer_state(void)
{
  return output_queue.count;
}
/*---------------------------------------------------------------------------*/
lwb_sync_state_t
lwb_get_sync_state(void)
{
  return sync_state;
}
/*---------------------------------------------------------------------------*/
uint32_t
lwb_get_time(rtimer_ext_clock_t* const out_timestamp)
{
  if(out_timestamp) {
    *out_timestamp = sync_timestamp;
  }
  return sync_time;
}
/*---------------------------------------------------------------------------*/
/*------------------------- HOST callback functions -------------------------*/
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
host_on_control_slot_post(gmw_control_t* in_out_control,
                          gmw_sync_event_t event,
                          gmw_pkt_event_t pkt_event)
{
  if(LWB_ROUND_TYPE_MAIN == current_round_type) {
    sync_time      = in_out_control->schedule.time;
    sync_timestamp = GMW_GET_T_REF();
    return GMW_RUNNING;
  } else {
    return GMW_SUSPENDED;
  }
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
host_on_slot_pre(uint8_t slot_index,
                 uint16_t slot_assignee,
                 uint8_t* out_len,
                 uint8_t* out_payload,
                 uint8_t is_initiator,
                 uint8_t is_contention_slot)
{
  if(is_initiator) {
    /* check if there is an S-ACK pending */
    *out_len = lwb_sched_prepare_sack((lwb_stream_ack_t*)out_payload);
    if(*out_len == 0) {
      /* no S-ACK is pending: see if there is some data to send */
      *out_len = output_queue_get(out_payload);
      if(*out_len == 0) {
        return GMW_EVT_SKIP_SLOT;
      }
    }
  } else if(is_initiator && !is_contention_slot) {
    /* normal data slot */
    *out_len = output_queue_get(out_payload);
  }
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
host_on_slot_post(uint8_t slot_index,
                  uint16_t slot_assignee,
                  uint8_t len,
                  uint8_t* payload,
                  uint8_t is_initiator,
                  uint8_t is_contention_slot,
                  gmw_pkt_event_t event)
{
  if(!is_initiator && (len >= sizeof(lwb_header_t))) {
    /* not initiator and we have received some data */
    lwb_pkt_t* pkt = (lwb_pkt_t*)payload;
    /* filter packet based on recipient ID */
    if(pkt->header.recipient_id == node_id ||
       pkt->header.recipient_id == LWB_RECIPIENT_SINK ||
       pkt->header.recipient_id == LWB_RECIPIENT_BROADCAST) {
      /* check the packet type */
      if(pkt->header.type == LWB_PACKET_TYPE_REQ) {
        /* stream request */
        lwb_sched_process_stream_req(&pkt->srq);

      } else if(pkt->header.type == LWB_PACKET_TYPE_DATA) {
        /* normal data packet */
        slot_streams[slot_index] = pkt->header.stream_id;
        /* replace recipient node ID by sender node ID */
        pkt->data.header.recipient_id = slot_assignee;
        input_queue_put(pkt->raw, len);
        DEBUG_PRINT_VERBOSE("data received (s=%u.%u l=%u)", slot_assignee,
                            pkt->data.header.stream_id, len);
      }
    } // else: ignore packet
  }
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
host_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_proc)
{
  if(LWB_ROUND_TYPE_MAIN == current_round_type) {
    /* next round will be schedule only */
    /* calculate the new schedule based on the registered streams */
    lwb_sched_compute(&control.schedule, slot_streams, output_queue.count);
    /* adjust the period */
    control.schedule.period -= LWB_CONF_SCHED2_OFFSET;
    GMW_LWB_SET_SECOND_CONTROL(&control);            /* mark as 2nd schedule */
    GMW_CONTROL_CLR_CONFIG(&control);               /* don't send the config */
    current_round_type = LWB_ROUND_TYPE_SCHED_ONLY;
    /* no pre or post process for the next round */
    in_out_pre_post_proc->post_process_current_round = NULL;
    in_out_pre_post_proc->pre_process_next_round     = NULL;
  } else {
    /* next round will be a main round */
    /* adjust period and time */
    control.schedule.period = LWB_CONF_SCHED2_OFFSET;
    GMW_LWB_SET_FIRST_CONTROL(&control);           /* mark as first schedule */
    GMW_CONTROL_SET_CONFIG(&control);                     /* send the config */
    current_round_type = LWB_ROUND_TYPE_MAIN;
    /* enable pre and post process */
    in_out_pre_post_proc->post_process_current_round = post_proc;
    in_out_pre_post_proc->pre_process_next_round     = pre_proc;
  }
  GMW_CONTROL_SET_USER_BYTES(&control);                 /* enable user bytes */
  gmw_set_new_control(&control);                /* notify GMW of new control */
}
/*---------------------------------------------------------------------------*/
/*------------------------ SOURCE callback functions ------------------------*/
/*---------------------------------------------------------------------------*/
static gmw_sync_state_t
src_on_control_slot_post(gmw_control_t* in_out_control,
                         gmw_sync_event_t event,
                         gmw_pkt_event_t pkt_event)
{
  /* default case */
  lwb_sync_event_t lwb_sync_event = EVT_SCHED_LWB_SYNC_STATE_MISSED;
  gmw_sync_state_t gmw_sync_state;
  if(GMW_EVT_CONTROL_RCVD == event) {
    /* control packet received! */
    /* toggle round type */
    if(GMW_LWB_IS_FIRST_CONTROL(in_out_control)) {
      current_round_type = LWB_ROUND_TYPE_MAIN;
      lwb_sync_event     = LWB_EVENT_RCVD_1ST_SCHED;
      /* store the synchronization point */
      sync_time          = in_out_control->schedule.time;
      sync_timestamp     = GMW_GET_T_REF();
    } else {
      current_round_type = LWB_ROUND_TYPE_SCHED_ONLY;
      lwb_sync_event     = LWB_EVENT_RCVD_2ND_SCHED;
    }
  } else {
    /* control packet missed */
    /* toggle round type */
    if(LWB_ROUND_TYPE_MAIN == current_round_type) {
      current_round_type = LWB_ROUND_TYPE_SCHED_ONLY;
    } else {
      current_round_type = LWB_ROUND_TYPE_MAIN;
    }
    /* manually update schedule */
    in_out_control->schedule.time  += in_out_control->schedule.period;
    in_out_control->schedule.period = previous_periods[1];
  }
  /* keep track of the round period */
  previous_periods[1] = previous_periods[0];
  previous_periods[0] = in_out_control->schedule.period;
  /* update the sync state */
  sync_state          = next_state[lwb_sync_event][sync_state];
  gmw_sync_state      = gmw_superstate[sync_state];

  return gmw_sync_state;
}
/*---------------------------------------------------------------------------*/
static gmw_skip_event_t
src_on_slot_pre(uint8_t slot_index,
                uint16_t slot_assignee,
                uint8_t* out_len,
                uint8_t* out_payload,
                uint8_t is_initiator,
                uint8_t is_contention_slot)
{
  lwb_pkt_t* lwb_pkt = (lwb_pkt_t*)out_payload;
  if(is_initiator) {
    /* this is our slot, send a data packet */
    *out_len = output_queue_get(lwb_pkt->raw);
    if(*out_len == 0) {
      DEBUG_PRINT_WARNING("no data to send, slot skipped");
      return GMW_EVT_SKIP_SLOT;
    }
  } else if(is_contention_slot) {
    /* contention slot */
    if(stream_request_pending) {
      /* allowed to send the request? */
      if(rounds_to_wait == 0) {
        /* send the stream request */
        *out_len = stream_prepare_req(&lwb_pkt->srq);
        if(*out_len) {
  #if LWB_CONF_CONT_BACKOFF
          /* wait between 1 and LWB_CONF_CONT_BACKOFF rounds */
          rounds_to_wait = (random_rand() >> 1) %
                           LWB_CONF_CONT_BACKOFF + 1;
  #endif /* LWB_CONF_CONT_BACKOFF */
          DEBUG_PRINT_INFO("stream request sent");
        }
      } else {
        DEBUG_PRINT_VERBOSE("must wait %u rounds", rounds_to_wait);
        rounds_to_wait--; /* decrease the number of rounds to wait */
      }
      DEBUG_PRINT_VERBOSE("%u pending stream requests",
                          stream_request_pending);
    }
  }
  return GMW_EVT_SKIP_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static gmw_repeat_event_t
src_on_slot_post(uint8_t slot_index,
                 uint16_t slot_assignee,
                 uint8_t len,
                 uint8_t* payload,
                 uint8_t is_initiator,
                 uint8_t is_contention_slot,
                 gmw_pkt_event_t event)
{
  lwb_pkt_t* lwb_pkt = (lwb_pkt_t*)payload;

  if(!is_initiator && (len >= sizeof(lwb_header_t))) {
    /* filter packet by recipient ID */
    if(lwb_pkt->header.recipient_id == node_id ||
       lwb_pkt->header.recipient_id == LWB_RECIPIENT_BROADCAST) {
      /* received a packet, check the type */
      if(LWB_PACKET_TYPE_DATA == lwb_pkt->header.type) {
        /* replace target node ID by sender node ID */
        lwb_pkt->header.recipient_id = slot_assignee;
        input_queue_put(lwb_pkt->raw, len);

      } else if(LWB_PACKET_TYPE_ACK == lwb_pkt->header.type) {
        /* acknowledgement for stream request received */
        stream_update_state(lwb_pkt->sack.stream_id);
        DEBUG_PRINT_INFO("ACK for stream %u received",
                         lwb_pkt->sack.stream_id);
        rounds_to_wait = 0;
      } else {
        DEBUG_PRINT_INFO("received packet of type %u ignored",
                         lwb_pkt->header.type);
      }
    } // else: ignore packet
  }
  return GMW_EVT_REPEAT_DEFAULT;
}
/*---------------------------------------------------------------------------*/
static void
src_on_round_finished(gmw_pre_post_processes_t* in_out_pre_post_proc)
{
  if(LWB_ROUND_TYPE_MAIN == current_round_type) {
    /* no pre or post process in the next round */
    in_out_pre_post_proc->post_process_current_round  = NULL;
    in_out_pre_post_proc->pre_process_next_round      = NULL;
  } else {
    /* enable pre and post process */
    in_out_pre_post_proc->post_process_current_round = post_proc;
    in_out_pre_post_proc->pre_process_next_round     = pre_proc;
  }
}
/*---------------------------------------------------------------------------*/
static uint32_t
src_on_bootstrap_timeout(void)
{
  /* stay in bootstrap */
  return 0;
}
/*---------------------------------------------------------------------------*/
/*---------------------------- stream handling ------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
lwb_stream_request(uint16_t ipi)
{
  uint16_t idx;
  /* look for an empty spot in the list */
  for(idx = 0; idx < LWB_CONF_MAX_N_STREAMS_PER_NODE; idx++) {
    if(streams[idx].state <= LWB_STREAM_STATE_INACTIVE) {
      break;  /* this stream is not being used -> take this slot */
    }
  }
  /* add the new stream */
  if(idx < LWB_CONF_MAX_N_STREAMS_PER_NODE) {
    streams[idx].ipi   = ipi;
    streams[idx].state = LWB_STREAM_STATE_PENDING;
    stream_request_pending++;
    DEBUG_PRINT_INFO("stream %u added (IPI: %u)", 
                     idx, streams[idx].ipi);
    return idx;     /* return the new stream ID (= index in the stream list) */
  }
  return LWB_INVALID_STREAM_ID;
}
/*---------------------------------------------------------------------------*/
void
lwb_stream_remove(uint8_t stream_id)
{
  if(stream_id < LWB_CONF_MAX_N_STREAMS_PER_NODE) {
    if((streams[stream_id].state == LWB_STREAM_STATE_PENDING) &&
       stream_request_pending) {
      stream_request_pending--;
    }
    streams[stream_id].ipi   = 0;
    streams[stream_id].state = LWB_STREAM_STATE_INACTIVE;
    DEBUG_PRINT_INFO("stream with ID %u removed", stream_id);
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
stream_update_state(uint16_t stream_id)
{
  if(stream_id < LWB_CONF_MAX_N_STREAMS_PER_NODE) {
    if((streams[stream_id].state == LWB_STREAM_STATE_PENDING) &&
       stream_request_pending) {
      stream_request_pending--;
    }
    if(streams[stream_id].ipi) {
      /* make sure the stream is in active state */
      streams[stream_id].state = LWB_STREAM_STATE_ACTIVE;
      return 1;   /* stream is active */

    } else {
      /* IPI is 0 -> make sure the stream is marked as 'inactive' */
      streams[stream_id].state = LWB_STREAM_STATE_INACTIVE;
      return 0;   /* stream removed */
    }
  }
  return 0;  /* invalid stream ID */
}
/*---------------------------------------------------------------------------*/
uint8_t
stream_prepare_req(lwb_stream_req_t* const out_srq)
{
  /* get the first stream request in the list */
  uint16_t idx = LWB_INVALID_STREAM_ID;
  for(idx = 0; idx < LWB_CONF_MAX_N_STREAMS_PER_NODE; idx++) {
    if(streams[idx].state == LWB_STREAM_STATE_PENDING) {
      break;
    }
  }
  if(idx < LWB_CONF_MAX_N_STREAMS_PER_NODE) {
    /* compose the packet */
    out_srq->header.recipient_id = LWB_RECIPIENT_SINK;
    out_srq->header.type         = LWB_PACKET_TYPE_REQ;
    out_srq->header.stream_id    = idx;
    out_srq->sender_id           = node_id;
    out_srq->ipi                 = streams[idx].ipi;
    return sizeof(lwb_stream_req_t);  /* success */
  }
  return 0;   /* no stream request to send */
}
/*---------------------------------------------------------------------------*/
lwb_stream_state_t
lwb_stream_get_state(uint8_t stream_id)
{
  if(stream_id < LWB_CONF_MAX_N_STREAMS_PER_NODE) {
    return streams[stream_id].state;
  }
  /* stream does not exist */
  return LWB_STREAM_STATE_INVALID;
}
/*---------------------------------------------------------------------------*/
/*-------------------------- helper functions -------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
input_queue_put(const uint8_t* const data, uint8_t len)
{
  if(len == 0 || !data) {
    return 0;
  }
  if(len > LWB_MAX_DATA_PKT_LEN) {
    len = LWB_MAX_DATA_PKT_LEN;
    DEBUG_PRINT_WARNING("received data packet is too big");
  }
  uint16_t addr = fifo16_put(&input_queue);
  if(FIFO16_ERROR != addr) {
    lwb_queue_elem_t* elem = (lwb_queue_elem_t*)addr;
    memcpy(&elem->data, data, len);    /* includes LWB header */
    elem->len = len;
    return 1;
  }
  DEBUG_PRINT_WARNING("LWB rx queue full, packet dropped");
  return 0;
}
/*---------------------------------------------------------------------------*/
/* fetch the next 'ready-to-send' message from the outgoing queue
 * returns the message length in bytes */
uint8_t
output_queue_get(uint8_t* const out_data)
{
  /* messages have the max. length LWB_MAX_DATA_PKT_LEN and are already
   * formatted according to glossy_payload_t */
  uint16_t addr = fifo16_get(&output_queue);
  if(FIFO16_ERROR != addr) {
    lwb_queue_elem_t* elem = (lwb_queue_elem_t*)addr;
    /* check the length */
    if(elem->len > LWB_MAX_DATA_PKT_LEN || elem->len == 0) {
      DEBUG_PRINT_WARNING("invalid message length detected");
      return 0;
    }
    memcpy(out_data, &elem->data, elem->len);
    return elem->len;   /* success */
  }
  return 0;  /* queue empty */
}
/*---------------------------------------------------------------------------*/
void
lwb_init(void)
{
  /* assign the callback functions and initialize the data structures for host
   * and source node */
  if(node_id == HOST_ID) {
    /* HOST */
    host_impl.on_control_slot_post = &host_on_control_slot_post;
    host_impl.on_slot_pre          = &host_on_slot_pre;
    host_impl.on_slot_post         = &host_on_slot_post;
    host_impl.on_round_finished    = &host_on_round_finished;
    gmw_control_init(&control);
    lwb_sched_init(&control.schedule);
    GMW_LWB_SET_FIRST_CONTROL(&control);
    GMW_CONTROL_SET_USER_BYTES(&control);
    GMW_CONTROL_SET_CONFIG(&control);
    gmw_set_new_control(&control);
    sync_state = LWB_SYNC_STATE_SYNCED;

  } else {
    /* SOURCE */
    src_impl.on_control_slot_post = &src_on_control_slot_post;
    src_impl.on_slot_pre          = &src_on_slot_pre;
    src_impl.on_slot_post         = &src_on_slot_post;
    src_impl.on_round_finished    = &src_on_round_finished;
    src_impl.on_bootstrap_timeout = &src_on_bootstrap_timeout;
    sync_state = LWB_SYNC_STATE_BOOTSTRAP;
    /* stream list */
    memset(streams, 0, LWB_CONF_MAX_N_STREAMS_PER_NODE * sizeof(lwb_stream_t));
  }
}
/*---------------------------------------------------------------------------*/
