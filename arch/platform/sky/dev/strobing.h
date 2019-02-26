/*
 * Copyright (c) 2018, ETH Zurich.
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
 * Authors: Reto Da Forno
 *
 */

#ifndef STROBING_H_
#define STROBING_H_

#include "contiki.h"
#include "rtimer-ext.h"

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

#ifndef STROBING_CONF_PAYLOAD_LEN
#define STROBING_CONF_PAYLOAD_LEN       100
#endif /* STROBING_CONF_PACKET_SIZE */

/* magic ID to identify a packet */
#ifndef STROBING_CONF_HEADER_BYTE
#define STROBING_CONF_HEADER_BYTE       0xab
#endif /* STROBING_CONF_HEADER_BYTE */

#ifndef STROBING_CONF_USE_TIMER_ISR
/* if set to 0, a callback function (strobing_timer_int_cb) will be defined
 * instead of the timer interrupt service routine */
#define STROBING_CONF_USE_TIMER_ISR     1
#endif /* STROBING_CONF_USE_TIMER_ISR */

#ifndef STROBING_CONF_TX_TO_TX_DELAY
#define STROBING_CONF_TX_TO_TX_DELAY    500   /* MCLK ticks */
#endif /* STROBING_CONF_TX_TO_TX_DELAY */


/* ----------------------- Application interface -------------------- */

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
 * @brief how many times the packet reception has been started in the last slot
 */
uint8_t strobing_get_rx_try_cnt(void);

/**
 * @brief get the length of the payload of the received/transmitted packet
 */
uint8_t strobing_get_payload_len(void);


void strobing_timer_int_cb(void);


#endif /* STROBING_H_ */

