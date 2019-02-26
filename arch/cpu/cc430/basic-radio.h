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

#ifndef BASIC_RADIO_H_
#define BASIC_RADIO_H_

/* include the Baloo configuration to overwrite the default settings */
#if BALOO
#include "gmw.h"
#endif

/* magic ID to identify a packet */
#ifndef BASIC_RADIO_CONF_HEADER_BYTE
#define BASIC_RADIO_CONF_HEADER_BYTE      0x76
#endif /* BASIC_RADIO_CONF_HEADER_BYTE */

#ifndef BASIC_RADIO_CONF_DO_CALIBRATION
#define BASIC_RADIO_CONF_DO_CALIBRATION   0
#endif /* BASIC_RADIO_CONF_DO_CALIBRATION */

/**
 * @brief send a data packet
 * @param payload         pointer to the data to send
 * @param payload_len     number of bytes to send
 * @param wait_until_sent set to non-zero value to block until TX finished
 * @return 1 if successful, 0 otherwise
 * @note: do not call this function from an interrupt! */
uint8_t radio_send(uint8_t* payload,
                   uint8_t payload_len,
                   uint8_t wait_until_sent);


/**
 * @brief receive a data packet
 * @param out_payload output buffer to hold the received data
 * @param timeout_ms  timeout in milliseconds (max: 1999)
 * @return payload length if successful, 0 otherwise
 * @note: do not call this function from an interrupt! */
uint8_t radio_rcv(uint8_t* out_payload,
                  uint16_t timeout_ms);


#endif /* BASIC_RADIO_H_ */
