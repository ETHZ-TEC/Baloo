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
 *          Romain Jacob
 */

#ifndef BASIC_RADIO_H_
#define BASIC_RADIO_H_

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

#ifndef RADIO_CONF_PAYLOAD_LEN
#define RADIO_CONF_PAYLOAD_LEN       100
#endif /* RADIO_CONF_PACKET_SIZE */

/* magic ID to identify a packet */
#ifndef RADIO_CONF_HEADER_BYTE
#define RADIO_CONF_HEADER_BYTE       0xaa
#endif /* RADIO_CONF_HEADER_BYTE */


/* ----------------------- Application interface -------------------- */

/**
 * @brief                     send a data packet
 * @param[in]   payload       pointer to the packet data
 * @param[in]   payload_len   length of the data
 * @param[in]   wait_until_sent set to a non-zero value to block until
 *                              the packet has been sent
 * @return 1 if successful, 0 otherwise
 * @note: do not call this function from an interrupt! */
uint8_t radio_send(uint8_t* payload,
                   uint8_t payload_len,
                  uint8_t wait_until_sent);


/**
 * @brief                     receive a data packet
 * @param[in]   payload       pointer to the receive packet buffer
 * @param[in]   timeout_ms    maximal time spent on idle listening
 *                            for a incoming packet, in ms (< 2000ms)
 * @return      payload length if successful,
 *              0 otherwise
 * @note: do not call this function from an interrupt! */
uint8_t radio_rcv(uint8_t* payload,
                  uint16_t timeout_ms);

/**
 * @brief       get the length of the payload of
 *              the received/transmitted packet
 */
uint8_t radio_get_payload_len(void);


#endif /* BASIC_RADIO_H_ */

