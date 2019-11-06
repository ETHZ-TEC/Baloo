/*
 * Copyright (c) 2011, ETH Zurich.
 * Copyright (c) 2013, Olaf Landsiedel.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Authors: Federico Ferrari
 *          Olaf Landsiedel
 *          Reto Da Forno
 */

/**
 * \file
 *         Chaos core, header file.
 * \author
 *         Olaf Landsiedel <olafl@chalmers.se>
 * \author
 *         Federico Ferrari <ferrari@tik.ee.ethz.ch>
 */

#ifndef CHAOS_H_
#define CHAOS_H_

#include <string.h>
#include "contiki.h"
#include "rtimer-ext.h"

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

#ifndef CHAOS_CONF_NUM_NODES
#define CHAOS_CONF_NUM_NODES         3
#endif /* CHAOS_CONF_NUM_NODES */

#ifndef CHAOS_CONF_NODE_ID_MAPPING
#define CHAOS_CONF_NODE_ID_MAPPING   { 1, 2, 3 }
#endif /* CHAOS_CONF_NODE_ID_MAPPING */

#ifndef CHAOS_CONF_FINAL_FLOOD_ON
#define CHAOS_CONF_FINAL_FLOOD_ON   1 // are the final Chaos floods enabled?
#endif /* CHAOS_CONF_FINAL_FLOOD_ON */

#ifndef CHAOS_CONF_TIMEOUT_ON
#define CHAOS_CONF_TIMEOUT_ON       1 // are timeouts enabled?
#endif /* CHAOS_CONF_TIMEOUT_ON */

/* total payload length */
#ifndef CHAOS_CONF_PAYLOAD_LEN
#define CHAOS_CONF_PAYLOAD_LEN      100
#endif /* CHAOS_CONF_PAYLOAD_LEN */

#ifndef CHAOS_CONF_SET_CUSTOM_PAYLOAD
#define CHAOS_CONF_SET_CUSTOM_PAYLOAD   0
#endif /* CHAOS_CONF_SET_CUSTOM_PAYLOAD */

#ifndef CHAOS_CONF_SHARED_PAYLOAD
#define CHAOS_CONF_SHARED_PAYLOAD       0
#endif /* CHAOS_CONF_SHARED_PAYLOAD */

/* payload per node */
#if CHAOS_CONF_SHARED_PAYLOAD
  #define CHAOS_PAYLOAD_PER_NODE  CHAOS_CONF_PAYLOAD_LEN
#else 
  #define CHAOS_PAYLOAD_PER_NODE  (CHAOS_CONF_PAYLOAD_LEN / CHAOS_CONF_NUM_NODES)
#endif /* CHAOS_CONF_SHARED_PAYLOAD */ 

#ifndef CHAOS_CONF_N_TX
#define CHAOS_CONF_N_TX             255
#endif /* CHAOS_CONF_N_TX */

#ifndef CHAOS_CONF_N_TX_COMPLETE
#define CHAOS_CONF_N_TX_COMPLETE    5
#endif /* CHAOS_CONF_N_TX_COMPLETE */

#ifndef CHAOS_CONF_USE_TIMER_ISR
/* if set to 0, a callback function (chaos_timer_int_cb) will be defined
 * instead of the timer interrupt service routine */
#if BALOO
#define CHAOS_CONF_USE_TIMER_ISR    !GMW_CONF_USE_MULTI_PRIMITIVES
#else
#define CHAOS_CONF_USE_TIMER_ISR    1
#endif /* BALOO */
#endif /* CHAOS_CONF_USE_TIMER_ISR */


enum {
  CHAOS_RECEIVER = 0, CHAOS_INITIATOR = 1
};

/**
 * \defgroup chaos_API General Chaos Interface
 * @{
 */

/**
 * \brief               Start Chaos and stall all other application tasks.
 *
 * \param payload       A pointer to the data.
 *                      At the initiator, Glossy reads from the given memory
 *                      location data provided by the application.
 *                      At a receiver, Glossy writes to the given memory
 *                      location data for the application.
 *                      NOTE: Buffer must be large enough to hold
 *                            CHAOS_CONF_PAYLOAD_LEN bytes.
 * \param payload_len   Length of the flooding data, in bytes.
 * \param is_initiator  1 if initiator, 0 otherwise
 * \param n_tx_max      Maximum number of transmissions (N).
 * \param dco_cal       Non-zero value => do DCO calibration.
 */
void chaos_start(uint8_t *payload,
                 uint8_t payload_len,
                 uint8_t is_initiator,
                 uint8_t n_tx_max,
                 uint8_t dco_cal);

/**
 * \brief            Stop Chaos and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Chaos phase.
 *                   If it is zero, the packet was not successfully received.
 * \sa               get_rx_cnt
 */
uint8_t chaos_stop(void);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Chaos phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t chaos_get_rx_cnt(void);

/**
 * \brief            Get the last data processing counter.
 * \returns          Number of times we execute the data processing function during
 *                   last Chaos phase.
 */
uint8_t chaos_get_processing_cnt(void);

/**
 * \brief            Get the current Chaos state.
 * \return           Current Chaos state, one of the possible values
 *                   of \link chaos_state \endlink.
 */
uint8_t chaos_get_state(void);

/**
 * \brief            Get low-frequency time of first packet reception
 *                   during the last Chaos phase.
 * \returns          Low-frequency time of first packet reception
 *                   during the last Chaos phase.
 */
rtimer_clock_t chaos_get_t_first_rx_l(void);

/**
 * \brief               Callback function for Choas data processing
 *
 * \param payload_field Pointer to the Chaos payload field.
 * \param node_index    Static index of a choas node.
 * \param payload       Pointer to the node payload.
 */
inline void chaos_set_payload_cb(uint8_t* payload_field,
                                 uint16_t node_index,
                                 uint8_t* payload);

/** @} */


#if !CHAOS_CONF_USE_TIMER_ISR
void chaos_timer_int_cb(void);
#endif /* CHAOS_CONF_USE_TIMER_ISR */

/**
 * \defgroup chaos_sync Interface related to time synchronization
 * @{
 */

/**
 * \brief            Get the last relay counter.
 * \returns          Value of the relay counter embedded in the first packet
 *                   received during the last Chaos phase.
 */
uint8_t chaos_get_relay_cnt(void);

/**
 * \brief            Get the local estimation of T_slot, in DCO clock ticks.
 * \returns          Local estimation of T_slot.
 */
rtimer_clock_t chaos_get_T_slot_h(void);

/**
 * \brief            Get low-frequency synchronization reference time.
 * \returns          Low-frequency reference time
 *                   (i.e., time at which the initiator started the flood).
 */
rtimer_clock_t chaos_get_t_ref_l(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Chaos phase, zero otherwise.
 */
uint8_t chaos_is_t_ref_l_updated(void);

rtimer_ext_clock_t chaos_get_t_ref_lf_ext(void);

/** @} */

#endif /* CHAOS_H_ */

