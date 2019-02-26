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

#ifndef GMW_LWB_H_
#define GMW_LWB_H_

#include "contiki.h"
#include "gmw.h"

/*--------------------------- default config --------------------------------*/

/* note: parameters only for LWB specific config, otherwise use the define
 *       from GMW! */

#ifndef LWB_CONF_INPUT_QUEUE_SIZE
#define LWB_CONF_INPUT_QUEUE_SIZE       5
#endif /* LWB_CONF_INPUT_QUEUE_SIZE */

#ifndef LWB_CONF_OUTPUT_QUEUE_SIZE
#define LWB_CONF_OUTPUT_QUEUE_SIZE      5
#endif /* LWB_CONF_INPUT_QUEUE_SIZE */

#if !LWB_CONF_OUTPUT_QUEUE_SIZE || !LWB_CONF_INPUT_QUEUE_SIZE
#error "LWB_CONF_OUTPUT_QUEUE_SIZE and LWB_CONF_INPUT_QUEUE_SIZE can't be 0"
#endif

#ifndef LWB_CONF_MAX_N_STREAMS_PER_NODE
/* this value may not be higher than 32! */
#define LWB_CONF_MAX_N_STREAMS_PER_NODE 5
#endif /* LWB_CONF_MAX_STREAM_CNT_PER_NODE */

#ifndef LWB_CONF_MAX_N_STREAMS
/* max. number of streams (bounds the required memory on the host) */
#define LWB_CONF_MAX_N_STREAMS          30
#endif /* N_STREAMS_MAX */

/* max. number of rounds a node backs off after sending a stream request
 * before it tries again */
#ifndef LWB_CONF_CONT_BACKOFF
#define LWB_CONF_CONT_BACKOFF           8
#endif /* LWB_CONF_CONT_BACKOFF */

#if GMW_CONF_CONTROL_USER_BYTES != 1
#error "GMW_CONF_CONTROL_USER_BYTES must be set to 1"
#endif /* GMW_CONF_CONTROL_USER_BYTES */


/* --- SCHEDULER --- */

#ifndef LWB_CONF_SCHED_PERIOD_MAX
/* max. assignable round period in seconds, must not exceed 2^15 - 1 seconds!*/
#define LWB_CONF_SCHED_PERIOD_MAX           30
#endif /* LWB_CONF_SCHED_PERIOD_MAX */

#ifndef LWB_CONF_SCHED_PERIOD_MIN
/* minimum round period, must be higher than T_ROUND_MAX */
#define LWB_CONF_SCHED_PERIOD_MIN           2
#endif /* LWB_CONF_SCHED_PERIOD_MIN */

#ifndef LWB_CONF_SCHED_PERIOD_DEFAULT
/* default period */
#define LWB_CONF_SCHED_PERIOD_DEFAULT       10
#endif /* LWB_CONF_SCHED_PERIOD_IDLE */

/* when to send the 2nd schedule, relative to the first (main) schedule */
#ifndef LWB_CONF_SCHED2_OFFSET
#define LWB_CONF_SCHED2_OFFSET              1     /* in seconds */
#endif /* LWB_CONF_SCHED2_OFFSET */

#ifndef LWB_CONF_SCHED_STREAM_TIMEOUT
/* threshold / timeout for the stream removal (max. number of consecutive
 * misses) */
#define LWB_CONF_SCHED_STREAM_TIMEOUT       10    /* # rounds */
#endif /* LWB_CONF_SCHED_STREAM_TIMEOUT */

#ifndef LWB_CONF_SCHED_MAX_PENDING_SACK
/* the max. number of pending stream ACKs (buffer size on HOST) */
#define LWB_CONF_SCHED_MAX_PENDING_SACK     4
#endif /* LWB_CONF_SCHED_MAX_PENDING_SACK */

#if LWB_CONF_SCHED2_OFFSET >= LWB_CONF_SCHED_PERIOD_MIN
#error "LWB_CONF_SCHED2_OFFSET >= LWB_CONF_SCHED_PERIOD_MIN"
#endif

/*-------------------------------- defines ----------------------------------*/

#define LWB_RECIPIENT_SINK          0x0000  /* to all sinks and the host */
#define LWB_RECIPIENT_BROADCAST     0xffff  /* to all nodes / sinks */

#define LWB_SCHEDULER_STATIC        1
#define LWB_SCHEDULER_MIN_ENERGY    2

/* use GMW_CONF_... defines to set the max. packet length! */
#define LWB_MAX_PAYLOAD_LEN         (GMW_CONF_MAX_DATA_PKT_LEN - \
                                     LWB_HEADER_LEN)
#define LWB_MAX_DATA_PKT_LEN        GMW_CONF_MAX_DATA_PKT_LEN
#define LWB_HEADER_LEN              4

#define GMW_LWB_CONTROL_MASK          (0x80)
#define GMW_LWB_SET_FIRST_CONTROL(c)  ((c)->user_bytes[0] |= GMW_LWB_CONTROL_MASK)
#define GMW_LWB_SET_SECOND_CONTROL(c) ((c)->user_bytes[0] &= ~GMW_LWB_CONTROL_MASK)
#define GMW_LWB_IS_FIRST_CONTROL(c)   (((c)->user_bytes[0] & GMW_LWB_CONTROL_MASK) > 0)
#define GMW_LWB_IS_SECOND_CONTROL(c)  (((c)->user_bytes[0] & GMW_LWB_CONTROL_MASK) == 0)

#define LWB_INVALID_STREAM_ID       0xff

/* max. allowed data slots (-1 due to contention slot) */
#define LWB_MAX_DATA_SLOTS          (GMW_CONF_MAX_SLOTS - 1)

#if LWB_MAX_DATA_SLOTS == 0
#error "GMW_CONF_MAX_SLOTS must be > 1"
#endif

#if GMW_CONF_TIME_SCALE != 1
#error "GMW_CONF_TIME_SCALE must be set to 1"
#endif

/*------------------------------- typedefs ----------------------------------*/

typedef enum {
  LWB_SYNC_STATE_BOOTSTRAP = 0,
  LWB_SYNC_STATE_QSYNCED,
  LWB_SYNC_STATE_SYNCED,
  LWB_SYNC_STATE_SYNCED2,
  LWB_SYNC_STATE_MISSED,
  LWB_SYNC_STATE_UNSYNCED,
  LWB_SYNC_STATE_UNSYNCED2,
  NUM_OF_LWB_SYNC_STATES
} lwb_sync_state_t;

typedef enum {
  LWB_PACKET_TYPE_DATA = 0,
  LWB_PACKET_TYPE_REQ,
  LWB_PACKET_TYPE_ACK,
} lwb_packet_type_t;

typedef enum {
  LWB_STREAM_STATE_INVALID = 0, /* invalid state or stream does not exist */
  LWB_STREAM_STATE_INACTIVE,    /* suspended */
  LWB_STREAM_STATE_PENDING,     /* request pending */
  LWB_STREAM_STATE_ACTIVE,      /* active, in use */
} lwb_stream_state_t;

typedef struct __attribute__((packed)) lwb_header {
  uint16_t          recipient_id;
  lwb_packet_type_t type : 8;   /* 1 byte only */
  uint8_t           stream_id;
} lwb_header_t;

typedef struct __attribute__((packed)) {
  lwb_header_t header;
  uint8_t      payload[LWB_MAX_PAYLOAD_LEN];
} lwb_data_t;

typedef struct __attribute__((packed)) lwb_stream_req {
  lwb_header_t header;
  uint16_t     sender_id;
  uint16_t     ipi;
  int16_t      offset;    /* optional offset parameter */
} lwb_stream_req_t;

typedef lwb_header_t lwb_stream_ack_t;    /* same data structure */

typedef struct __attribute__((packed)) lwb_pkt {
  /* be aware of structure alignment! */
  union __attribute__((packed)) {
    lwb_header_t     header;
    lwb_data_t       data;
    lwb_stream_req_t srq;
    lwb_stream_ack_t sack;
    uint8_t          raw[LWB_MAX_DATA_PKT_LEN];
  };
} lwb_pkt_t;

typedef gmw_schedule_t lwb_schedule_t;

/*----------------------------- main interface ------------------------------*/

/**
 * @brief start the Low-Power Wireless Bus
 * @param pre_lwb_proc a pointer to a preprocess task process that runs
 * before an LWB round. Set GMW_CONF_T_PREPROCESS to the worst-case
 * execution time of this function.
 * @param post_lwb_proc a pointer to the application task process control
 * block (struct process), this process will be called (polled) at the end
 * of an LWB round
 */
void lwb_start(struct process* pre_lwb_proc, struct process* post_lwb_proc);

/**
 * @brief schedule a packet for transmission over the LWB
 * @param data a pointer to the data packet to send
 * @param len the length of the data packet (must be less or equal
 * LWB_MAX_DATA_PKT_LEN)
 * @return 1 if successful, 0 otherwise (queue full or invalid parameters)
 */
uint8_t lwb_send_pkt(uint16_t recipient,
                     uint8_t stream_id,
                     const uint8_t * const data,
                     uint8_t len);

/**
 * @brief get a data packet that have been received during the previous LWB
 * rounds
 * @param out_data A valid memory block that can hold one data packet. The
 * buffer must be at least LWB_MAX_DATA_PKT_LEN bytes long.
 * @param out_sender_id the ID of the node that sent the message (optional)
 * @param out_stream_id the stream ID (optional)
 * @return the length of the data packet in bytes or 0 if the queue is empty
 * @note once a data packet was requested, it will be removed from the internal
 * buffer
 */
uint8_t lwb_rcv_pkt(uint8_t* out_data,
                    uint16_t * const out_sender_id,
                    uint8_t * const out_stream_id);

/**
 * @brief check the status of the receive buffer (incoming messages)
 * @return the number of packets in the queue
 */
uint8_t lwb_rcv_buffer_state(void);

/**
 * @brief check the status of the send buffer (outgoing messages)
 * @return the number of remaining packets in the queue
 */
uint8_t lwb_send_buffer_state(void);

/**
 * @brief schedules a stream request to be sent during the contention slot
 * @param ipi inter-packet interval in seconds
 * @return the ID of the new stream if successful, LWB_INVALID_STREAM_ID
 * otherwise (stream list full)
 * @note: the stream info data to which stream_request points will be copied
 * into an internal buffer and may be deleted after calling
 * lwb_request_stream().
 */
uint8_t lwb_stream_request(uint16_t ipi);

/**
 * @brief removes a stream from the list
 * @param stream_id ID of the stream
 */
void lwb_stream_remove(uint8_t stream_id);

/**
 * @brief query the state of a stream
 * @param stream_id ID of the stream
 * @return the stream state
 */
lwb_stream_state_t lwb_stream_get_state(uint8_t stream_id);

/**
 * @brief query the synchronization state
 * @return the current sync state
 */
lwb_sync_state_t lwb_get_sync_state(void);

/**
 * @brief get the last sync time
 * @param out_timestamp [optional] timestamp in ticks of the last sync point
 * @return the time in seconds
 */
uint32_t lwb_get_time(rtimer_ext_clock_t* const out_timestamp);


/*-------------------------- scheduler interface ----------------------------*/
/* do not call these functions directly */

uint16_t lwb_sched_init(lwb_schedule_t* const out_sched);
uint16_t lwb_sched_compute(gmw_schedule_t* const in_out_sched,
                           const uint8_t* const slot_streams,
                           uint8_t n_slots_host);
uint8_t  lwb_sched_prepare_sack(lwb_stream_ack_t* const out_sack);
void     lwb_sched_process_stream_req(const lwb_stream_req_t* req);


#endif /* GMW_LWB_H_ */
