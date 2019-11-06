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
 * @addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *            Implementation of control-related functions.
 */

/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gmw.h"
#include "debug-print.h"
/*---------------------------------------------------------------------------*/
#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
static const uint8_t slot_times[GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE] = {
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_0),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_1),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_2),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_3),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_4),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_5),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_6),
  GMW_US_TO_SLOT_TIME(GMW_SLOT_TIME_7)
};
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */
/*---------------------------------------------------------------------------*/
static void config_init(  gmw_config_t*   config  );
static void schedule_init(gmw_schedule_t* schedule);
/*---------------------------------------------------------------------------*/
void
gmw_control_init(gmw_control_t *control)
{
  schedule_init(&control->schedule);
  config_init(&control->config);
  GMW_CONTROL_SET_CONFIG(control);

#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  GMW_CONTROL_SET_SLOT_CONFIG(control);
  memcpy(control->slot_time_list, slot_times, sizeof(slot_times));
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */

#if GMW_CONF_CONTROL_USER_BYTES
  GMW_CONTROL_SET_USER_BYTES(control);
#endif /* GMW_CONF_CONTROL_USER_BYTES */

  if(sizeof(gmw_slot_config_t) != GMW_CONTROL_SLOT_CONFIG_TYPE_SIZE) {
    DEBUG_PRINT_ERROR(
      "sizeof(gmw_slot_config_t) != GMW_CONTROL_SLOT_CONFIG_TYPE_SIZE");
  }

  if(sizeof(gmw_control_t) > GMW_MAX_PKT_LEN) {
    DEBUG_PRINT_WARNING("A full control structure does not fit in a packet!");
    DEBUG_PRINT_WARNING("Reduce control size or use static schedule/config.");
  }

#if GMW_CONF_USE_MAGIC_NUMBER
  control->magic_number = GMW_CONF_CONTROL_MAGIC_NUMBER;
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_control_compile_to_buffer(const gmw_control_t* control,
                              uint8_t* buffer,
                              uint8_t len)
{
  uint16_t start_address = (uint16_t)buffer;
  uint16_t n_slots       = GMW_SCHED_N_SLOTS(&control->schedule);

  if(( sizeof(gmw_control_t)            // size of full control
      - GMW_CONF_USE_STATIC_SCHED *     // if static sched substract
        sizeof(gmw_schedule_t)          // size of schedule
      - GMW_CONF_USE_STATIC_CONFIG *    // if static config substract
       (sizeof(gmw_config_t)            // size of config plus
        + (sizeof(gmw_slot_config_t)
          * GMW_CONF_MAX_SLOTS          // size of slot_config
          + GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE)  // size of slot_time_list
        * GMW_CONF_USE_CONTROL_SLOT_CONFIG)) > len) {
    DEBUG_PRINT_WARNING("Packet buffer too small to send the required control "
                        "information.");
    return 0;
  }

  /* schedule section */
  if(!GMW_CONF_USE_STATIC_SCHED) {

    if(GMW_CONF_MAX_SLOTS < n_slots) {
      DEBUG_PRINT_ERROR("n_slots is too large (>= GMW_CONF_MAX_SLOTS)");
    }
    memcpy(buffer, &control->schedule, GMW_SCHED_SECTION_HEADER_LEN);
    buffer += GMW_SCHED_SECTION_HEADER_LEN;
    memcpy(buffer, control->schedule.slot, n_slots * 2);
    buffer += n_slots * 2;
  }

  /* config section */
  if(GMW_CONTROL_HAS_CONFIG(control)) {
    if(!GMW_CONF_USE_STATIC_CONFIG) {
      memcpy(buffer, &control->config, GMW_CONFIG_SECTION_HEADER_LEN);
      buffer += GMW_CONFIG_SECTION_HEADER_LEN;
    }
  }

#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  /* slot config section */
  if(GMW_CONTROL_HAS_SLOT_CONFIG(control)) {
    if(!GMW_CONF_USE_STATIC_CONFIG) {
      /* If static config, the slot config is also static*/
      memcpy(buffer, &control->slot_config    , n_slots);
      buffer += n_slots;
      memcpy(buffer, &control->slot_time_list , 8);
      buffer += 8;
    }
  }
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */

  /* user bytes section */
#if GMW_CONF_CONTROL_USER_BYTES
  if(GMW_CONTROL_HAS_USER_BYTES(control)) {
    memcpy(buffer, &control->user_bytes, GMW_CONF_CONTROL_USER_BYTES);
    buffer += GMW_CONF_CONTROL_USER_BYTES;
  }
#endif /* GMW_CONF_CONTROL_USER_BYTES */

#if GMW_CONF_USE_MAGIC_NUMBER
  /* magic number */
  *buffer = control->magic_number; /* Magic number is one byte */
  buffer++;
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/

  if(!((uint16_t)buffer - start_address)) {
    DEBUG_PRINT_WARNING("Control packet contains no payload! Glossy behavior "
                        "undefined.");
    return 1;

  } else {
    return ((uint16_t)buffer - start_address);
  }
}
/*---------------------------------------------------------------------------*/
/* macro used to decompile the control structure from array */
#define ITERATE_BUFFER(f, s, loc) \
{ \
  if(len < s) { \
    DEBUG_PRINT_ERROR("received buffer len too small for control packet @" loc\
                      " (expect=%u receive=%u)", s, len); \
    return 0xff; \
  } \
  memcpy(f, buffer, s); \
  len -= s; \
  buffer += s; \
}

/*---------------------------------------------------------------------------*/
uint8_t
gmw_control_decompile_from_buffer(gmw_control_t* control,
                                  uint8_t* buffer,
                                  uint8_t len)
{
  uint8_t n_slots = 0;

  if(( sizeof(gmw_control_t)            // size of full control
      - GMW_CONF_USE_STATIC_SCHED *     // if static sched substract
        sizeof(gmw_schedule_t)          // size of schedule
      - GMW_CONF_USE_STATIC_CONFIG *    // if static config substract
       (sizeof(gmw_config_t)            // size of config plus
       + (sizeof(gmw_slot_config_t)
         * GMW_CONF_MAX_SLOTS           // size of slot_config
         + GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE)  // size of slot_time_list
       * GMW_CONF_USE_CONTROL_SLOT_CONFIG)) < len) {
    DEBUG_PRINT_WARNING("Received packet bigger than maximal expected control size.");
    DEBUG_PRINT_MSG_NOW("exp %u, rcv %u",
                        ( sizeof(gmw_control_t)            // size of full control
                              - GMW_CONF_USE_STATIC_SCHED *     // if static sched substract
                                sizeof(gmw_schedule_t)          // size of schedule
                              - GMW_CONF_USE_STATIC_CONFIG *    // if static config substract
                               (sizeof(gmw_config_t)            // size of config plus
                               + (sizeof(gmw_slot_config_t)
                                 * GMW_CONF_MAX_SLOTS           // size of slot_config
                                 + GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE)  // size of slot_time_list
                               * GMW_CONF_USE_CONTROL_SLOT_CONFIG)), len );
    return 0;
  }

  /* schedule section */
  if(!GMW_CONF_USE_STATIC_SCHED) {
    /*Like this, nothing at all will be sent (not even time and period)*/
    ITERATE_BUFFER(&control->schedule, GMW_SCHED_SECTION_HEADER_LEN, "sched");
    n_slots = GMW_SCHED_N_SLOTS(&control->schedule);;
    ITERATE_BUFFER(&control->schedule.slot[0], 2*n_slots, "n_slots");
  }

  /* optional config section */
  if(GMW_CONTROL_HAS_CONFIG(control)) {
    if(!GMW_CONF_USE_STATIC_CONFIG) {
      ITERATE_BUFFER(&control->config, GMW_CONFIG_SECTION_HEADER_LEN, "config");
    }
  }

#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  /* optional slot config section */
  if(GMW_CONTROL_HAS_SLOT_CONFIG(control)) {
    if(!GMW_CONF_USE_STATIC_CONFIG) {
      /*If static config, the slot config is also static*/
      ITERATE_BUFFER(&control->slot_config    , n_slots, "sconfig");
      ITERATE_BUFFER(&control->slot_time_list , 8, "slot_time_list");
    }
  }
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */

#if GMW_CONF_CONTROL_USER_BYTES
  /* optional user bytes section */
  if(GMW_CONTROL_HAS_USER_BYTES(control)) {
    ITERATE_BUFFER(&control->user_bytes, GMW_CONF_CONTROL_USER_BYTES, "user");
  }
#endif /* GMW_CONF_CONTROL_USER_BYTES */

#if GMW_CONF_USE_MAGIC_NUMBER
  /* magic number */
  ITERATE_BUFFER(&control->magic_number, 1, "magicNb");
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/

  return 1;
}
/*---------------------------------------------------------------------------*/
static void
config_init(gmw_config_t* config)
{
  config->n_retransmissions    = GMW_CONF_TX_CNT_DATA;
  config->channel_hopping_mode = 0;
  config->max_packet_length    = GMW_CONF_MAX_DATA_PKT_LEN +
                                 GMW_CONF_RF_OVERHEAD;
  config->primitive            = 0;
  config->gap_time             = GMW_US_TO_GAP_TIME(GMW_CONF_T_GAP);
  config->slot_time            = GMW_US_TO_SLOT_TIME(GMW_CONF_T_DATA);
}
/*---------------------------------------------------------------------------*/
static void
schedule_init(gmw_schedule_t* schedule)
{
  schedule->n_slots             = 0;
  schedule->time                = 0;
  schedule->period              = 5;
}
/*---------------------------------------------------------------------------*/
/** @} */
