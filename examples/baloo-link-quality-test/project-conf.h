/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Romain Jacob
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

/*
 * application specific config file to override default settings
 */

/* Test configuration */

#define HOST_ID                         host_id
/* If set, only one measurement round; otherwise continuous */
#define TEST_CONF_ONE_ROUND             1
#define GMW_CONF_RF_TX_CHANNEL          rf_channel

#ifdef FLOCKLAB
#define NODE_LIST  \
 { 1,2,3,4,6,7,8,10,11,13,14,15,16,17,18,19,20,22,23,24,25,26,27,28,31,32,33 }
#define NUM_NODES                       27
#else
#define NODE_LIST  \
 { 2,7,16,2,7,16,2,7,16,2,7,16,2,7,16,7}
#define NUM_NODES                       16
#endif

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */

#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"

  #define GLOSSY_START_PIN                FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                   FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                   FLOCKLAB_INT2
  #define STROBING_START_PIN              FLOCKLAB_LED1
  #define STROBING_TX_PIN                 FLOCKLAB_INT1
  #define STROBING_RX_PIN                 FLOCKLAB_INT2
  #define STROBING_RX_SUCCESS_PIN         FLOCKLAB_INT1

/* for debubbing */
  //#define DEBUG_PIN                         FLOCKLAB_LED1
  //#define CB_PIN                          FLOCKLAB_INT2

#endif /* FLOCKLAB */

/* Platform-specific configuration */
#ifdef PLATFORM_SKY

  /* GPIO */
#ifndef FLOCKLAB
  #define GLOSSY_START_PIN              ADC0
  #define GLOSSY_TX_PIN                 ADC1
  #define GLOSSY_RX_PIN                 ADC2
  #define STROBING_START_PIN            ADC0
  #define STROBING_TX_PIN               ADC1
  #define STROBING_RX_PIN               ADC2
  #define STROBING_RX_SUCCESS_PIN       ADC1
#endif /* FLOCKLAB */

  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

  /* communication primitives */
  #define GMW_PRIM1_ENABLE              0  /* don't use chaos */

#elif defined PLATFORM_DPP_CC430

  /* RF1A settings */

  /* GPIO */
#ifndef FLOCKLAB
  #define GLOSSY_START_PIN              COM_GPIO1
  #define GLOSSY_TX_PIN                 COM_GPIO2
  #define GLOSSY_RX_PIN                 COM_GPIO3
  #define STROBING_START_PIN            COM_GPIO1
  #define STROBING_TX_PIN               COM_GPIO2
  #define STROBING_RX_PIN               COM_GPIO3
  #define STROBING_RX_SUCCESS_PIN       COM_GPIO2

  /* for debubbing */
  //#define ISR_PIN                       COM_GPIO1
  //#define CB_PIN                        COM_GPIO3

#endif /* FLOCKLAB */

#else  /* PLATFORM_ */
  #error "unknown target platform!"
#endif /* PLATFORM_ */

/* Strobing config */
#define STROBING_CONF_PAYLOAD_LEN       payload_length
#define STROBING_CONF_FIRST_BYTE_AS_COUNTER 1

/* GMW configuration */
#define GMW_CONF_USE_MULTI_PRIMITIVES   1
#define GMW_CONF_MAX_SLOTS              NUM_NODES
#define GMW_CONF_TX_CNT_CONTROL         6
#define GMW_CONF_TX_CNT_DATA            100 // Number of strobes
#define GMW_CONF_T_DATA                 GMW_T_SLOT_STROBE_MIN(payload_length + \
                                          GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_DATA) /* us */


#define GMW_CONF_PERIOD_TIME_BASE       GMW_CONF_PERIOD_TIME_BASE_1s
/* increase the slot time base */
#define GMW_CONF_SLOT_TIME_BASE         1000LU /*us*/
/* set generous gap and guard times */
#define GMW_CONF_T_GUARD_SLOT           1000LU /*us*/
#define GMW_CONF_T_GAP                  5000LU /*us*/
/* the pre-round setup fits in the guard time */
#define GMW_CONF_PREROUND_SETUP_TIME    GMW_CONF_T_GUARD_ROUND

#define GMW_CONF_USE_STATIC_SCHED       1
#define GMW_CONF_USE_STATIC_CONFIG      1
#define GMW_CONF_CONTROL_USER_BYTES     1

#define GMW_CONF_USE_AUTOCLEAN          1

/* enable LEDs */
#define LED_CONF_ON                     1

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_BUFFER_SIZE    1024

#endif /* __CONFIG_H__ */
