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
 */

#ifndef CHAOS_TEST_H_
#define CHAOS_TEST_H_


#ifndef INITIATOR_NODE_ID
#define INITIATOR_NODE_ID         1
#endif

#define CHAOS_CONF_NUM_NODES        2
#define CHAOS_CONF_NODE_ID_MAPPING  { 1, 2 }

#define CHAOS_CONF_PAYLOAD_LEN    (CHAOS_CONF_NUM_NODES * 2)

#define CHAOS_RTIMER_ID           RTIMER_EXT_LF_0
#define RTIMER_EXT_SECOND         RTIMER_EXT_SECOND_LF // same as RTIMER_SECOND

#define CHAOS_PERIOD              ((uint32_t)RTIMER_EXT_SECOND)     // 1s
#define CHAOS_DURATION            ((uint32_t)RTIMER_EXT_SECOND / 2) // 0.5s

/* Guard-time at receivers. */
#define CHAOS_GUARD_TIME          (RTIMER_EXT_SECOND / 333)    // 3ms

/* Number of consecutive Chaos phases with successful computation of reference
 * time required to exit from bootstrapping. Default value: 3. */
#define CHAOS_BOOTSTRAP_PERIODS   3

/* cc2420 config, don't change! */
#define CC2420_CONF_AUTOACK         0
#define CC2420_CONF_ADDRDECODE      0
#define CC2420_CONF_SFD_TIMESTAMPS  0
//#define CC2420_TXPOWER              CC2420_TXPOWER_MAX
#define RF_CHANNEL                  15

/* GPIO config */
#define CHAOS_START_PIN             ADC0
#define CHAOS_RX_PIN                ADC1
#define CHAOS_TX_PIN                ADC2


#endif /* CHAOS_TEST_H_ */
