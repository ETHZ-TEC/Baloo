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
 * Author:  Reto Da Forno
 *          Federico Ferrari
 *          Marco Zimmerling
 */

/**
 * @brief
 * a very basic scheduler implementation for the LWB
 *
 * The scheduler operates with a constant period and only accepts new
 * stream requests if the network is not yet saturated.
 * There is exactly one contention slot per round.
 */

#include "gmw-lwb.h"
#include "list.h"
#include "memb.h"
#include "node-id.h"
#include "random.h"
#include "debug-print.h"

/*---------------------------------------------------------------------------*/
typedef struct lwb_stream_list {
  struct lwb_stream_list *next;
  uint16_t node_id;
  uint16_t ipi;
  uint32_t last_assigned;
  uint8_t  stream_id;
  uint8_t  n_cons_missed;
} lwb_stream_list_t;
/*---------------------------------------------------------------------------*/
static uint16_t          period;
static uint32_t          time;               /* global time */
static uint16_t          n_streams;          /* # streams */
static uint8_t           saturated;
static uint32_t          data_cnt;
static uint16_t          data_ipi;
static uint8_t           n_pending_sack;
static uint8_t           stream_req_in_last_round;
static lwb_stream_ack_t  pending_sack[LWB_CONF_SCHED_MAX_PENDING_SACK];
LIST(streams_list);
MEMB(streams_memb, lwb_stream_list_t, LWB_CONF_MAX_N_STREAMS);
/*---------------------------------------------------------------------------*/
static inline uint16_t gcd(uint16_t u, uint16_t v);
/*---------------------------------------------------------------------------*/
static inline void
lwb_sched_del_stream(lwb_stream_list_t* stream) 
{
  if(0 == stream) {
    return;  /* entry not found, don't do anything */
  }
  uint16_t device_id  = stream->node_id;
  uint8_t  stream_id  = stream->stream_id;
  list_remove(streams_list, stream);
  memb_free(&streams_memb, stream);
  n_streams--;
  DEBUG_PRINT_INFO("stream %u of node %u removed", stream_id, device_id);
}
/*---------------------------------------------------------------------------*/
uint8_t
lwb_sched_prepare_sack(lwb_stream_ack_t* const out_sack)
{
  if(n_pending_sack) {
    n_pending_sack--;
    memcpy(out_sack, &pending_sack[n_pending_sack], sizeof(lwb_stream_ack_t));
    return sizeof(lwb_stream_ack_t);
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
void
lwb_sched_process_stream_req(const lwb_stream_req_t* req)
{
  if(LWB_INVALID_STREAM_ID == req->header.stream_id ||
     n_pending_sack >= LWB_CONF_SCHED_MAX_PENDING_SACK) {
    DEBUG_PRINT_WARNING("can't process stream request");
    return;
  }
  stream_req_in_last_round = 1;
  /* check if stream already exists */
  lwb_stream_list_t *stream = 0;
  for(stream = list_head(streams_list); stream != 0; stream = stream->next) {
    if(req->sender_id == stream->node_id &&
       req->header.stream_id == stream->stream_id) {
      break;
    }
  }
  /* add or remove is implicitly given by the IPI (0 implies 'remove') */
  if(req->ipi > 0) {
    /* add or adjust a stream */
    uint32_t last_assigned = 0;
    if(((int32_t)time + req->offset) > 0) {
      last_assigned = (int32_t)time + req->offset;
    }
    if(stream) {
      /* already exists -> update the IPI */
      stream->ipi           = req->ipi;
      stream->last_assigned = last_assigned;
      stream->n_cons_missed = 0;         /* reset this counter */
      DEBUG_PRINT_INFO("stream %u of node %u updated",
                       stream->stream_id,
                       stream->node_id);
    } else {
      /* does not exist: add the new stream */
      stream = memb_alloc(&streams_memb);
      if(stream == 0) {
        DEBUG_PRINT_ERROR("out of memory: stream request dropped");
        return;
      }
      stream->node_id       = req->sender_id;
      stream->ipi           = req->ipi;
      stream->last_assigned = last_assigned;
      stream->stream_id     = req->header.stream_id;
      stream->n_cons_missed = 0;
      /* insert the stream into the list, ordered by node id */
      lwb_stream_list_t *prev;
      for(prev = list_head(streams_list); prev != 0; prev = prev->next) {
        if((stream->node_id >= prev->node_id) &&
           ((prev->next == 0) || (stream->node_id < prev->next->node_id))) {
          break;
        }
      }
      list_insert(streams_list, prev, stream);
      n_streams++;
      DEBUG_PRINT_INFO("stream %u of node %u registered",
                       stream->stream_id,
                       stream->node_id);
    }
    /* insert into the list of pending S-ACKs */
    pending_sack[n_pending_sack].recipient_id = req->sender_id;
    pending_sack[n_pending_sack].type         = LWB_PACKET_TYPE_ACK;
    pending_sack[n_pending_sack].stream_id    = req->header.stream_id;
    n_pending_sack++;

  } else {
    /* remove stream */
    lwb_sched_del_stream(stream);
    /* no ACK for stream removal */
    DEBUG_PRINT_INFO("stream %u of node %u removed",
                      stream->stream_id,
                      stream->node_id);
  }
}
/*---------------------------------------------------------------------------*/
static inline uint16_t
adapt_period(void)
{
  if(stream_req_in_last_round) {
    /* set period to the minimal allowed value */
    stream_req_in_last_round = 0;
    return LWB_CONF_SCHED_PERIOD_MIN;
  }
  stream_req_in_last_round = 0;
  if(!data_cnt) {
    return LWB_CONF_SCHED_PERIOD_DEFAULT;     /* no streams */
  }
  saturated = 0;
  uint16_t new_period = (uint16_t)(((uint32_t)data_ipi *
                                    LWB_MAX_DATA_SLOTS) / data_cnt);
  /* check for saturation */
  if(new_period < LWB_CONF_SCHED_PERIOD_MIN) {
    /* T_opt is smaller than LWB_CONF_SCHED_PERIOD_MIN */
    DEBUG_PRINT_WARNING("network saturated!");
    saturated = 1;
    return LWB_CONF_SCHED_PERIOD_MIN;
  }
  /* limit the period */
  if(new_period > LWB_CONF_SCHED_PERIOD_MAX) {
    return LWB_CONF_SCHED_PERIOD_MAX;
  }
  return new_period;
}
/*---------------------------------------------------------------------------*/
uint16_t
lwb_sched_compute(lwb_schedule_t* const in_out_sched,
                  const uint8_t* const slot_streams,
                  uint8_t n_slots_host)
{
  static uint16_t slots_tmp[LWB_MAX_DATA_SLOTS + 1];
  uint16_t n_slots_assigned = 0;
  uint16_t i;

  /* reset values */
  data_ipi = 1;
  data_cnt = 0;

  /* loop through all the streams in the list */
  lwb_stream_list_t *curr_stream = list_head(streams_list);
  while(curr_stream != NULL) {
    /* check if the stream is in 'slot_streams' */
    uint16_t list_len = GMW_SCHED_N_SLOTS(in_out_sched);
    uint16_t idx = 0;
    /* slow search, should be optimized */
    while(idx < list_len) {
      if(in_out_sched->slot[idx] == curr_stream->node_id &&
         slot_streams[idx] == curr_stream->stream_id) {
        curr_stream->n_cons_missed = 0;   /* reset */
        break;
      }
      idx++;
    }
    if(curr_stream->n_cons_missed & 0x80) {
      /* no packet received from this stream */
      curr_stream->n_cons_missed &= 0x7f;              /* clear the last bit */
      curr_stream->n_cons_missed++;
    }
    if(curr_stream->n_cons_missed > LWB_CONF_SCHED_STREAM_TIMEOUT) {
      /* too many consecutive slots without reception: delete this stream */
      lwb_stream_list_t *stream_to_remove = curr_stream;
      curr_stream = curr_stream->next;
      lwb_sched_del_stream(stream_to_remove);
    } else {
      uint16_t curr_gcd = gcd(data_ipi, curr_stream->ipi);
      uint16_t k1       = curr_stream->ipi / curr_gcd;
      uint16_t k2       = data_ipi / curr_gcd;
      data_cnt          = data_cnt * k1 + k2;
      data_ipi          = data_ipi * k1;
      curr_stream = curr_stream->next;
    }
  }

  /* now we can clear content of the schedule */
  memset(in_out_sched->slot, 0, sizeof(in_out_sched->slot));
  /* stream ACK slot? */
  if(n_pending_sack) {
    n_slots_host++;
  }
  /* assign slots to the host */
  n_slots_host = MIN(LWB_MAX_DATA_SLOTS, n_slots_host);
  i = n_slots_host;
  while(i) {
    in_out_sched->slot[n_slots_assigned++] = node_id;
    i--;
  }
  period = adapt_period();                         /* adapt the round period */
  time  += period;                   /* increment time by the current period */

  /* assign slots to the source nodes */
  if(n_streams > 0) {
    /* random initial position in the list */
    uint16_t first_idx = 0;
    uint16_t rand_init_pos = (random_rand() % n_streams);
    curr_stream = list_head(streams_list);
    for(i = 0; i < rand_init_pos; i++) {
      curr_stream = curr_stream->next;
    }
    lwb_stream_list_t *first_stream = curr_stream;
    do {
      /* assign slots for this stream, if possible */
      if((n_slots_assigned < LWB_MAX_DATA_SLOTS) && 
        (time >= (curr_stream->ipi + curr_stream->last_assigned))) {
        /* the number of slots to assign to curr_stream */
        uint16_t to_assign = (time - curr_stream->last_assigned) /
                             curr_stream->ipi;
        if(saturated) {
          if((curr_stream->next == first_stream) || 
            (curr_stream->next == NULL && rand_init_pos == 0)) {
            /* last random stream: assign all possible slots */
          } else {
            /* ensure fairness among source nodes when the bandwidth saturates
            * assigned a number of slots proportional to (1/IPI) */
            uint16_t slots_ipi = period / curr_stream->ipi;
            if(to_assign > slots_ipi) {
              to_assign = slots_ipi;
              if(to_assign == 0 && curr_stream == first_stream) {
                /* first random stream: assign one slot to it, even if it has 
                * very long IPI */
                to_assign = 1;
              }
            }
          }
        }
        if(to_assign > (LWB_MAX_DATA_SLOTS - n_slots_assigned)) {
          to_assign = LWB_MAX_DATA_SLOTS - n_slots_assigned;
        }
        curr_stream->last_assigned += to_assign * curr_stream->ipi;
        while(to_assign > 0) {
          slots_tmp[n_slots_assigned] = curr_stream->node_id;
          to_assign--;
          n_slots_assigned++;
        }
        /* set the last bit, we are expecting a packet from this stream in the
         * next round */
        curr_stream->n_cons_missed |= 0x80; 
      }
      /* go to the next stream in the list */
      curr_stream = curr_stream->next;
      if(curr_stream == NULL) {
        /* end of the list: start again from the head of the list */
        curr_stream = list_head(streams_list);
        first_idx   = n_slots_assigned;
      }
    } while(curr_stream != first_stream);

    /* copy into new data structure to keep the node IDs ordered */
    memcpy(&in_out_sched->slot[n_slots_host], &slots_tmp[first_idx],
           (n_slots_assigned - first_idx) * 2);
    memcpy(&in_out_sched->slot[n_slots_assigned - first_idx + n_slots_host],
           slots_tmp, first_idx * 2);
  }

  /* add a contention slot at the end */
  in_out_sched->slot[n_slots_assigned++] = GMW_SLOT_CONTENTION;
  in_out_sched->n_slots = n_slots_assigned;

  /* this schedule is sent at the end of a round: do not communicate
   * (i.e. do not set the first bit of period) */
  in_out_sched->period = period;   /* no need to clear the last bit */
  in_out_sched->time   = time;
  /* log the parameters of the new schedule */
  DEBUG_PRINT_INFO("schedule updated (period: %u, slots: %u, streams: %u)", 
                   in_out_sched->period, n_slots_assigned, n_streams);

  return (n_slots_assigned * 2) + GMW_SCHED_SECTION_HEADER_LEN;
}
/*---------------------------------------------------------------------------*/
uint16_t
lwb_sched_init(lwb_schedule_t* const out_sched)
{
  memb_init(&streams_memb);
  list_init(streams_list);

  n_streams          = 0;
  n_pending_sack     = 0;
  time               = 0;
  period             = LWB_CONF_SCHED2_OFFSET;
  out_sched->time    = time;
  out_sched->period  = period;
  out_sched->n_slots = 1;
  out_sched->slot[0] = GMW_SLOT_CONTENTION;  /* always incl. contention slot */

  DEBUG_PRINT_INFO("LWB scheduler initialized (max streams: %u)",
                   LWB_CONF_MAX_N_STREAMS);

  return GMW_SCHED_SECTION_HEADER_LEN + 2;    /* 2 bytes for contention slot */
}
/*---------------------------------------------------------------------------*/

/**
 * @brief implementation of the binary GCD algorithm
 * @param[in] u an unsigned integer
 * @param[in] v an unsigned integer
 * @return the greatest common divider of u and v
 */
static inline uint16_t
gcd(uint16_t u, uint16_t v)
{
  uint16_t shift;

  /* GCD(0,x) := x */
  if(u == 0 || v == 0) {
    return u | v;
  }
  /* Let shift := lg K, where K is the greatest power of 2
     dividing both u and v. */
  for(shift = 0; ((u | v) & 1) == 0; ++shift) {
    u >>= 1;
    v >>= 1;
  }
  while((u & 1) == 0) {
    u >>= 1;
  }
  /* From here on, u is always odd. */
  do {
    while((v & 1) == 0) {  /* Loop X */
      v >>= 1;
    }
    /* Now u and v are both odd, so diff(u, v) is even.
       Let u = min(u, v), v = diff(u, v)/2. */
    if(u < v) {
      v -= u;
    } else {
      uint16_t diff = u - v;
      u = v;
      v = diff;
    }
    v >>= 1;
  } while(v != 0);

  return u << shift;
}
/*---------------------------------------------------------------------------*/
