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

#define HOST_ID                         4

#define NODE_LIST  \
 { 1,2,3,4,6, 8,10,11,13,15,16,17,18,19,20, 22,23,24,25,26, 27,28,31,32,33 }
// { 1,2,3,4,6,7,8,10,11,13,14,15,16,17,18,19,20,22,23,24,25,26,27,28,32,33 }
//#define NUM_NODES                       26
#define NUM_NODES                       25

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */

#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"

  #define GMW_CONF_ROUND_ACT_PIN        FLOCKLAB_LED1   // Baloo rounds
  #define DCSTAT_RF_ON_PIN              FLOCKLAB_INT1   // radio-on
  #define GMW_CONF_SLOT_ACT_PIN         FLOCKLAB_INT2   // Baloo slots
  #define XP_ACT_PIN                    FLOCKLAB_INT2   // Baloo pre-process & CB

  /* for debugging */
  //#define GLOSSY_RX_PIN                 FLOCKLAB_LED1
  //#define GLOSSY_TX_PIN                 FLOCKLAB_INT2

#endif /* FLOCKLAB */

/* Platform-specific configuration */
#ifdef PLATFORM_SKY

  /* GPIO */
#ifndef FLOCKLAB
  #define GMW_CONF_ROUND_ACT_PIN        ADC0   // Baloo rounds
  #define DCSTAT_RF_ON_PIN              ADC1   // radio-on
  #define GMW_CONF_SLOT_ACT_PIN         ADC2   // Baloo slots
  #define XP_ACT_PIN                    ADC2   // Baloo pre-process & CB
#endif /* FLOCKLAB */

  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2405_MHz
  /* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

#elif defined PLATFORM_DPP_CC430

  /* GPIO */
#ifndef FLOCKLAB
  #define GMW_CONF_ROUND_ACT_PIN        COM_GPIO1   // Baloo rounds
  #define DCSTAT_RF_ON_PIN              COM_GPIO2   // radio-on
  #define GMW_CONF_SLOT_ACT_PIN         COM_GPIO3   // Baloo slots
  #define XP_ACT_PIN                    COM_GPIO3   // Baloo pre-process & CB

  /* for debugging */
  //#define GLOSSY_RX_PIN                 COM_GPIO1
  //#define GLOSSY_TX_PIN                 COM_GPIO3

#endif /* FLOCKLAB */

  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_869_0_MHz

#else  /* PLATFORM_ */
  #error "unknown target platform!"
#endif /* PLATFORM_ */

/* GMW configuration */
#define GMW_CONF_MAX_DATA_PKT_LEN       64
#define GMW_CONF_MAX_SLOTS              30

#define GMW_CONF_MAX_HOPS               4 // H
#define GMW_CONF_TX_CNT_DATA            2 // N
#define GMW_CONF_TX_CNT_CONTROL         GMW_CONF_TX_CNT_DATA

#define GMW_CONF_PERIOD_TIME_BASE       GMW_CONF_PERIOD_TIME_BASE_1ms

#define GMW_CONF_T_GAP                  1500LU  /* us */
#define GMW_CONF_T_GAP_CONTROL          1500LU  /* us */
#define GMW_CONF_T_GUARD_SLOT           100LU   /* us */
#define GMW_CONF_T_GUARD_ROUND          GMW_CONF_T_GUARD_SLOT
/* the pre-round setup fits in the guard time - it takes ~67us */
#define GMW_CONF_PREROUND_SETUP_TIME    GMW_CONF_T_GUARD_ROUND

#define GMW_CONF_T_PREPROCESS           2LU /* ms */
#define GMW_CONF_T_DATA                 GMW_T_SLOT_MIN(payload_length + \
                                          GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_DATA, \
                                          GMW_CONF_MAX_HOPS) /* us */

#define GMW_CONF_USE_STATIC_SCHED       1
#define GMW_CONF_USE_STATIC_CONFIG      1
#define GMW_CONF_CONTROL_USER_BYTES     2
#define GMW_CONF_USE_AUTOCLEAN          1

/* always send the relay counter in Glossy header */
#define GLOSSY_CONF_ALWAYS_RELAY_CNT    1

/* enable dc-stat to measure the radio-on time */
#define DCSTAT_CONF_ON                  1

/* disable LEDs */
#define LED_CONF_ON                     0

/* enable Bolt */
#define BOLT_CONF_ON                    1

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_STACK_GUARD    (SRAM_START + SRAM_SIZE - 0x0200)
#define DEBUG_PRINT_CONF_BUFFER_SIZE    1024

#endif /* __CONFIG_H__ */
