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
 */

#ifndef STROBING_H_
#define STROBING_H_

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

/* magic ID to identify a packet */
#ifndef STROBING_CONF_HEADER_BYTE
#define STROBING_CONF_HEADER_BYTE             0x44
#endif /* STROBING_CONF_HEADER_BYTE */

/* define the rf1a_cb_... functions within strobing.c? if disabled, callback
 * functions will be named strobing_...() instead */
#ifndef STROBING_CONF_USE_RF1A_CALLBACKS
#if BALOO
#define STROBING_CONF_USE_RF1A_CALLBACKS      !GMW_CONF_USE_MULTI_PRIMITIVES
#else
#define STROBING_CONF_USE_RF1A_CALLBACKS      1
#endif /* BALOO */
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */

#ifndef STROBING_CONF_TX_TO_TX_DELAY
#define STROBING_CONF_TX_TO_TX_DELAY          300   /* MCLK ticks */
#endif /* STROBING_CONF_TX_TO_TX_DELAY */


/**
 * @brief       start strobing
 * @param[in]   is_initiator whether or not this node is initiator
 * @param[in]   payload pointer to the packet data
 * @param[in]   payload_len length of the data, must not exceed
 *                          STROBING_CONF_PAYLOAD_LEN
 * @param[in]   n_tx number of retransmissions
 */
void strobing_start(uint8_t is_initiator,
                    uint8_t* payload,
                    uint8_t payload_len,
                    uint8_t n_tx);

/**
 * @brief stop strobing
 */
uint8_t strobing_stop(void);

/**
 * @brief query activity of strobing
 * @return the number of received bytes since strobing_start was called
 */
uint8_t strobing_is_active(void);

/**
 * @brief get the number of received packets during the last slot
 */
uint8_t strobing_get_rx_cnt(void);

/**
 * @brief get the number of received packets during the last slot
 */
uint8_t strobing_get_rx_try_cnt(void);

/**
 * @brief get the length of the payload of the received/transmitted packet
 */
uint8_t strobing_get_payload_len(void);

#if !STROBING_CONF_USE_RF1A_CALLBACKS
/**
 * @brief callback functions
 */
void strobing_rx_started(rtimer_ext_clock_t *timestamp);
void strobing_tx_started(rtimer_ext_clock_t *timestamp);
void strobing_header_received(rtimer_ext_clock_t *timestamp, uint8_t *header, uint8_t packet_len);
void strobing_rx_ended(rtimer_ext_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len);
void strobing_tx_ended(rtimer_ext_clock_t *timestamp);
void strobing_rx_failed(rtimer_ext_clock_t *timestamp);
void strobing_rx_tx_error(rtimer_ext_clock_t *timestamp);
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */

#endif /* STROBING_H_ */
