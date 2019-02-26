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

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

#define HOST_ID                         1

/*
 * NOTE: works 'as is' for up to 8 nodes in NODE_LIST
 * Adding more node requires some adaptations to `baloo-chaos-test.c`
 */
/*
#define NODE_LIST  \
 { 1,2,3,4,6,7,8,10,11,13,14,15,16,17,18,19,20,22,23,24,25,26,27,28,32,33 }
#define NUM_NODES                       26
*/
#define NODE_LIST  \
 { 2,7 }
#define NUM_NODES                       2

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
  #define CHAOS_START_PIN               FLOCKLAB_LED1
  #define CHAOS_TX_PIN                  FLOCKLAB_INT1
  #define CHAOS_RX_PIN                  FLOCKLAB_INT2
#else
  #define GLOSSY_START_PIN              ADC0
  #define GLOSSY_TX_PIN                 ADC1
  #define GLOSSY_RX_PIN                 ADC2
  #define CHAOS_START_PIN               ADC0
  #define CHAOS_TX_PIN                  ADC1
  #define CHAOS_RX_PIN                  ADC2
#endif /* FLOCKLAB */


/* Chaos config */
#define CHAOS_CONF_NUM_NODES            NUM_NODES
#define CHAOS_CONF_NODE_ID_MAPPING      NODE_LIST
#define CHAOS_CONF_PAYLOAD_LEN          1
/* Set to one to specify a custom agregation function.
 * If set to 0, Chaos simply copy each node's payload in the payload, in their
 * respective place (according to CHAOS_CONF_NODE_ID_MAPPING).
 */
#define CHAOS_CONF_SET_CUSTOM_PAYLOAD   1
/* If set to one, the chaos payload (size CHAOS_CONF_PAYLOAD_LEN) is shared
 * among all nodes. Otherwise, each node
 * (CHAOS_CONF_PAYLOAD_LEN / CHAOS_CONF_NUM_NODES) bytes of payload.
 */
#define CHAOS_CONF_SHARED_PAYLOAD       1
#if CHAOS_CONF_SHARED_PAYLOAD && !CHAOS_CONF_SET_CUSTOM_PAYLOAD
#warning "If payload is shared, a custom agregation function is probably needed!"
#endif


/* GMW config */
#define GMW_CONF_USE_MULTI_PRIMITIVES       1
#define GMW_PRIM2_ENABLE                0     /* disable strobing mode */
/* make sure none of the used communication primitives defines the timer b1
  * interrupt service routine */
#define CHAOS_CONF_USE_TIMER_ISR        0
#define GLOSSY_CONF_USE_TIMER_ISR       0
#define GMW_CONF_MAX_DATA_PKT_LEN       CHAOS_CONF_PAYLOAD_LEN
#define GMW_CONF_MAX_SLOTS              1
//#define GMW_CONF_T_DATA                 50000
#define GMW_CONF_RF_TX_CHANNEL          GMW_RF_TX_CHANNEL_2405_MHz


/* config for cc2420 radio */
#define CC2420_CONF_AUTOACK             0
#define CC2420_CONF_ADDRDECODE          0
#define CC2420_CONF_SFD_TIMESTAMPS      0

/* CPU frequency */
#define F_CPU                           4194304UL


#endif /* PROJECT_CONF_H_ */
