/*
 * Copyright (c) 2011, ETH Zurich.
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
 *          Reto Da Forno
 *
 */

#ifndef GLOSSY_H_
#define GLOSSY_H_

#include "contiki.h"
#include "rtimer-ext.h"

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

#ifndef GLOSSY_CONF_PAYLOAD_LEN
#define GLOSSY_CONF_PAYLOAD_LEN         100
#endif /* GLOSSY_CONF_PACKET_SIZE */

/* magic number / identifier for a Glossy packet (4 bits only) */
#ifndef GLOSSY_CONF_HEADER_BYTE
#define GLOSSY_CONF_HEADER_BYTE         0x0a
#endif /* GLOSSY_CONF_HEADER_BYTE */

/* Try to keep the setup time in glossy_start() constant to allow for precide
 * drift compensation on the source nodes. The flood initiator will wait until
 * the specified amount has passed before issuing the start_tx command. 
 * Note: This define only has an effect on the initiator if sync is enabled.
 *       This feature can be disabled by setting the value to zero. */
#ifndef GLOSSY_CONF_SETUPTIME_WITH_SYNC
#define GLOSSY_CONF_SETUPTIME_WITH_SYNC 1000UL    /* in us */
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */

#ifndef GLOSSY_CONF_USE_TIMER_ISR
/* if set to 0, a callback function (glossy_timer_int_cb) will be defined
 * instead of the timer interrupt service routine */
#define GLOSSY_CONF_USE_TIMER_ISR       1
#endif /* CHAOS_CONF_USE_TIMER_ISR */

/* do not change */
#define GLOSSY_MAX_HEADER_LEN               4


/* ----------------------- Application interface -------------------- */

/**
 * \brief               Start Glossy and stall all other application tasks.
 *
 * \param initiator_id  Node ID of the flood initiator.
 * \param payload       A pointer to the data.
 *                      At the initiator, Glossy reads from the given memory
 *                      location data provided by the application.
 *                      At a receiver, Glossy writes to the given memory
 *                      location data for the application.
 * \param payload_len   Length of the flooding data, in bytes.
 * \param n_tx_max      Maximum number of transmissions (N).
 * \param with_sync     Not zero if Glossy must provide time synchronization,
 *                      zero otherwise.
 * \param dco_cal       Non-zero value => do DCO calibration.
 */
void glossy_start(uint16_t initiator_id,
                  uint8_t *payload,
                  uint8_t payload_len,
                  uint8_t n_tx_max,
                  uint8_t with_sync,
                  uint8_t dco_cal);

/**
 * \brief            Stop Glossy and resume all other application tasks.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t glossy_stop(void);

/**
 * \brief            Get the last received counter.
 * \returns          Number of times the packet has been received during
 *                   last Glossy phase.
 *                   If it is zero, the packet was not successfully received.
 */
uint8_t glossy_get_rx_cnt(void);

/**
 * \brief            Get the number of started receptions
 * \returns          Number of times (preamble + sync detection)
 *                   has been received during the last flood
 */
uint8_t glossy_get_rx_try_cnt(void);

/**
 * \brief            Get the current Glossy state.
 * \return           Current Glossy state, one of the possible values
 *                   of \link glossy_state \endlink.
 */
uint8_t glossy_get_state(void);

/**
 * \brief            Get the last relay counter.
 * \returns          Value of the relay counter embedded in the first packet
 *                   received during the last Glossy phase.
 */
uint8_t glossy_get_relay_cnt(void);

/**
 * \brief            Provide information about current synchronization status.
 * \returns          Not zero if the synchronization reference time was
 *                   updated during the last Glossy phase, zero otherwise.
 */
uint8_t glossy_is_t_ref_updated(void);

/**
 * \brief            Get the sync reference time.
 * \returns          Timestamp in rtimer-ext clock ticks
 */
rtimer_ext_clock_t glossy_get_t_ref(void);

/**
 * \brief get the received payload length of the last flood
 */
uint8_t glossy_get_payload_len(void);

/**
 * \brief            get the average flood success rate
 * \returns          percentage * 100 of successful floods, i.e. at least 1
 *                   packet received
 */
uint16_t glossy_get_fsr(void);

/**
 * \brief            get the average packet error rate
 * \return           percentage * 100 of successfully received packets
 */
uint16_t glossy_get_per(void);

/**
 * \brief timer B callback function for RF interrupt handling
 */
void glossy_timer_int_cb(void);


#endif /* GLOSSY_H_ */

