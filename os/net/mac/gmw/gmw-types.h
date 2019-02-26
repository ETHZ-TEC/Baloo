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
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *            This file contains all GMW-specific data types and
 *            the callback prototypes.
 */

#ifndef GMW_TYPES_H_
#define GMW_TYPES_H_
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- Control structure definition ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief                       Schedule section structure.
 *
 * @note                        n_slots encodes additional information:
 *                              which sections are included in the current
 *                              control (see gmw-control.h).
 */
#define GMW_SCHED_SECTION_HEADER_LEN    (8)
typedef struct __attribute__((packed)) gmw_schedule {
  uint32_t time;                  /* multiple of GMW_CONF_PERIOD_TIME_BASE */
  uint16_t period;                /* multiple of GMW_CONF_PERIOD_TIME_BASE */
  uint16_t n_slots;
  uint16_t slot[TOGMW_CONF_MAX_SLOTS];
} gmw_schedule_t;

/**
 * @brief                       Config section structure.
 */
#define GMW_CONFIG_SECTION_HEADER_LEN   (6)
typedef struct __attribute__((packed)) gmw_config {
  uint8_t n_retransmissions;
  uint8_t channel_hopping_mode; /*
                                 * not, per slot, or in flood
                                 * (see gmw_hopping_mode_t)
                                 * /!\ NOT IMPLEMENTED YET /!\
                                 */
  uint8_t max_packet_length;    /*
                                 * Max packet size (at rf level)
                                 * for data packets
                                 */
  uint8_t primitive;            /* ID of the primitive to use (0 is Glossy) */
  uint8_t gap_time;             /* Gap time in increments of 100 us */
  uint8_t slot_time;            /* Slot time in increments of 500 us */
} gmw_config_t;

/**
 * @brief                       Slot_config config structure
 */
#define GMW_CONTROL_SLOT_CONFIG_TYPE_SIZE       1
#define GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE   8
typedef struct __attribute__((packed)) gmw_slot_config {
  uint8_t n_retransmissions : 3;
  uint8_t slot_time_select  : 3; /* ID of desired slot time (see slot_times)*/
  uint8_t primitive         : 2; /* ID of the primitive to use (0 is Glossy) */
} gmw_slot_config_t;

/**
 * @brief                       Control packet structure
 *
 * @note                        The size and content of the control structure
 *                              is flexibly set based on configuration defines:
 *                              - GMW_CONF_USE_CONTROL_SLOT_CONFIG
 *                              - GMW_CONF_CONTROL_USER_BYTES
 *                              - GMW_CONF_USE_MAGIC_NUMBER
 *                              - GMW_CONF_MAX_SLOTS
 *                              - GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE
 */
typedef struct __attribute__((packed)) gmw_control {
  gmw_schedule_t schedule;
  gmw_config_t config;
#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  gmw_slot_config_t   slot_config[GMW_CONF_MAX_SLOTS];
  uint8_t             slot_time_list[GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE];
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
#if GMW_CONF_CONTROL_USER_BYTES
  uint8_t user_bytes[GMW_CONF_CONTROL_USER_BYTES];
#endif /* GMW_CONF_CONTROL_USER_BYTES */
#if GMW_CONF_USE_MAGIC_NUMBER
  uint8_t magic_number;
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/
} gmw_control_t;

/**
 * @brief                       Struct used to hold pointers to the pre- and
 *                              post process to run before/after GMW rounds
 */
typedef struct gmw_pre_post_processes {
  struct process* post_process_current_round;
  struct process* pre_process_next_round;
} gmw_pre_post_processes_t;
/*---------------------------------------------------------------------------*/

/**
 * @brief                       Statistics structure
 */
typedef struct {
  uint8_t  relay_cnt;       /* number of retransmission the host received
                             * for control slot */
  uint8_t  suspended_cnt;   /* #rounds spent in suspended state */
  uint8_t  bootstrap_cnt;   /* # bootstrap attempts */
  uint8_t  sleep_cnt;       /* #times node went into LPM due to rf silence */
  int16_t  drift;           /* drift versus the reference time in ppm */
  uint16_t pkt_rcvd_cnt;    /* total number of received packets */
  uint16_t pkt_corr_cnt;    /* total number of corrupted packets
                             * (unsuccessful floods) */
  uint16_t pkt_interf_cnt;  /* total number of interfered packets
                             * (something on the channel,
                             * but no flood detected) */
  uint16_t pkt_silence_cnt; /* total number of slots with expected packets
                             * but nothing on the channel */
  uint16_t t_proc_max;      /* max. time needed to process rcvd data pkts */
  uint32_t t_round_max;     /* longest round duration in ms */
  uint32_t t_slack_min;     /* shortest slack time (end of current round to
                             * start of next round) in ms */
  /* crc must be the last element! */
  uint16_t crc;             /* crc of this struct (without the crc) */
} gmw_statistics_t;
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- States and events ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief                       State of GMW
 *                              Used as return type of the control_post callback
 *                              for the application to inform the middleware
 *                              of the desired state for the coming round.
 */
typedef enum {
  GMW_BOOTSTRAP = 0,
  GMW_RUNNING,
  GMW_SUSPENDED,
  GMW_DEFAULT,              /* State is decided on following the standard GMW
                             * state-machine */
} gmw_sync_state_t;

/**
 * @brief                       GMW control packet reception events
 */
typedef enum {
  GMW_EVT_CONTROL_RCVD = 0,
  GMW_EVT_CONTROL_MISSED,
  GMW_NUM_OF_SYNC_EVENTS
} gmw_sync_event_t;

/**
 * @brief                       GMW data packet reception events
 */
typedef enum {
  GMW_EVT_PKT_OK = 0,          /* Successful Glossy flood (send or receive) */
  GMW_EVT_PKT_CORRUPTED = 1,   /* A Glossy flood detected but corrupted */
  GMW_EVT_PKT_GARBAGE = 2,     /* Something received but not a Glossy flood */
  GMW_EVT_PKT_SILENCE = 3,     /* Nothing received at all */
  GMW_EVT_PKT_MISSED = 4,      /* Packet missed (bad timing) */
  GMW_EVT_PKT_SKIPPED = 5      /* Slot skipped */
} gmw_pkt_event_t;

/**
 * @brief                       GMW slot skip event
 *                              Used as return type of the slot_pre callback
 *                              for the application to inform the middleware
 *                              if the coming slot should be executed or skipped
 */
typedef enum {
  GMW_EVT_NO_SKIP = 0,         /* The slot is executed normally */
  GMW_EVT_SKIP_SLOT = 1,       /* The slot is skipped:
                                * the node turns off its radio */
  GMW_EVT_SKIP_DEFAULT = 0     /* Default = no skip */
} gmw_skip_event_t;

/**
 * @brief                       GMW slot and round repeat events
 *                              Used as return type of the slot_post callback
 *                              for the application to inform the middleware
 *                              if the past slot (or the whole round) should be
 *                              repeated.
 */
typedef enum {
  GMW_EVT_NO_REPEAT = 0,       /* No repetition */
  GMW_EVT_REPEAT_SLOT = 1,     /* Repeat the last data slot */
  GMW_EVT_REPEAT_ROUND = 2,    /* Smart over the complete round */
  GMW_EVT_REPEAT_DEFAULT = 0   /* Set default same as 'NO_REPEAT' */
} gmw_repeat_event_t;
/*---------------------------------------------------------------------------*/

/**
 * @brief                       GMW channel hopping mode
 *
 * @note                        Not implemented so far.
 */
typedef enum {
  GMW_NO_HOPPING = 0,          /* No channel hopping */
  GMW_PER_SLOT_HOPPING = 1,    /* Channel is fixed for a given flood */
  GMW_IN_FLOOD_HOPPING = 2     /* Channel can hop during a Glossy flood
                                * -> Not yet implemented */
} gmw_hopping_mode_t;

/*---------------------------------------------------------------------------*/
/*--- Callback functions ---*/
/*---------------------------------------------------------------------------*/

/**
 * @brief                       Called after sending or receiving/missing
 *                              a schedule/control packet
 * @param in_out_schedule       Pointer to the sent/received control packet.
 * @param sync_event            Event tagging success or failure of control
 *                              packet reception.
 * @param pkt_event             Information on the success or the (source of)
 *                              failure of control packet reception.
 * @return gmw_sync_state_t     Desired middleware state for the coming round.
 */
typedef gmw_sync_state_t (*gmw_on_control_slot_post_callback)(
              gmw_control_t*    in_out_control,
              gmw_sync_event_t  sync_event,
              gmw_pkt_event_t   pkt_event);

/**
 * @brief                       Called before a data slot to prepare for the
 *                              upcoming communication.
 * @param slot_index            Slot index of the upcoming data slot,
 *                              starting at 0.
 * @param slot_assignee         Node ID of the assigned node to this slot
 *                              (or GMW_SLOT_CONTENTION)
 * @param out_len               Size of the payload to send (if any)
 * @param out_payload           Pointer to the payload to send (if any).
 * @param is_initiator          Helper variable:
 *                              1 if (node_id==slot_assignee), 0 otherwise
 * @param is_contention_slot    Helper variable:
 *                              1 if (GMW_SLOT_CONTENTION==slot_assignee),
 *                              0 otherwise
 * @return gmw_skip_event_t     Flag to inform the middleware if the coming slot
 *                              should be executed or skipped/
 */
typedef gmw_skip_event_t (*gmw_on_slot_pre_callback)(
                      uint8_t   slot_index,
                      uint16_t  slot_assignee,
                      uint8_t*  out_len,
                      uint8_t*  out_payload,
                      uint8_t   is_initiator,
                      uint8_t   is_contention_slot);

/**
 * @brief                       Called after a data slot to process the
 *                              received data.
 * @param slot_index            Slot index of the last data slot,
 *                              starting at 0.
 * @param slot_assignee         Node ID of the assigned node to this slot
 *                              (or GMW_SLOT_CONTENTION)
 * @param len                   Size of the received payload (if any)
 * @param payload               Pointer to the received payload (if any).
 * @param is_initiator          Helper variable:
 *                              1 if (node_id==slot_assignee), 0 otherwise
 * @param is_contention_slot    Helper variable:
 *                              1 if (GMW_SLOT_CONTENTION==slot_assignee),
 *                              0 otherwise
 * @param pkt_event             Information on the success or the (source of)
 *                              failure of control packet reception.
 * @return gmw_repeat_event_t   Flag for the middleware to know whether
 *                              the last slot or the whole round should be
 *                              repeated after this slot.
 */
typedef gmw_repeat_event_t (*gmw_on_slot_post_callback)(
                       uint8_t  slot_index,
                       uint16_t slot_assignee,
                       uint8_t  len,
                       uint8_t* payload,
                       uint8_t  is_initiator,
                       uint8_t  is_contention_slot,
                gmw_pkt_event_t pkt_event);

/**
 * @brief                       Called after the last slot has been handled
 *                              (i.e., after the last on_slot_post callback).
 * @param pre_post_processes    Pointer to the pre-/post-processes to excute
 *                              after this round and before the next one.
 *                              Set to NULL to disable either process.
 */
typedef void (*gmw_on_round_finish_callback)(
     gmw_pre_post_processes_t* in_out_pre_post_processes);
/**
 * @brief                       Called each time a source node timed out while
 *                              trying to receive a control packet.
 *                              This lets the application layer decide whether
 *                              a node should keep trying to bootstrap.
 * @return                      A time to wait (in ms) until we try again
 *                              to receive a control packet.
 *                              If 0 is returned, we try receiving again right
 *                              away; otherwise, the radio is turned off for the
 *                              returned time.
 */
typedef uint32_t (*gmw_on_bootstrap_timeout_callback)(void);

/**
 * @brief                       Node implementation structure,
 *                              holding pointers to all callback functions.
 */
typedef struct gmw_protocol_impl {
  gmw_on_control_slot_post_callback     on_control_slot_post;
  gmw_on_slot_pre_callback              on_slot_pre;
  gmw_on_slot_post_callback             on_slot_post;
  gmw_on_round_finish_callback          on_round_finished;
  gmw_on_bootstrap_timeout_callback     on_bootstrap_timeout;
} gmw_protocol_impl_t;

/** @} */

#endif /* GMW_TYPES_H_ */
