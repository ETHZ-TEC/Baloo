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
 *          Romain Jacob
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to override default settings
 */

#define HOST_ID                         1

#define NODE_LIST  \
 { 1,2,3,4,6,7,8,10,11,13,14,15,16,17,18,19,20,22,23,24,25,26,27,28,32,33 }
#define NUM_NODES                       26

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */#ifdef FLOCKLAB
  #include "../../tools/flocklab/flocklab.h"
  #define GLOSSY_START_PIN                FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                   FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                   FLOCKLAB_INT2
  #define STROBING_START_PIN              FLOCKLAB_LED1
  #define STROBING_TX_PIN                 FLOCKLAB_INT1
  #define STROBING_RX_PIN                 FLOCKLAB_INT2
#endif /* FLOCKLAB */

/* platform dependent config */
#ifdef PLATFORM_SKY
  /* RF channel */
  #define GMW_CONF_RF_TX_CHANNEL          GMW_RF_TX_CHANNEL_2405_MHz
  /* radio config */
  #define CC2420_CONF_AUTOACK             0
  #define CC2420_CONF_ADDRDECODE          0
  #define CC2420_CONF_SFD_TIMESTAMPS      0
  /* GPIO */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN              ADC0
    #define GLOSSY_TX_PIN                 ADC1
    #define GLOSSY_RX_PIN                 ADC2
    #define STROBING_START_PIN            ADC0
    #define STROBING_TX_PIN               ADC1
    #define STROBING_RX_PIN               ADC2
  #endif /* FLOCKLAB */
  /* communication primitives */
  #define GLOSSY_CONF_USE_TIMER_ISR       !GMW_CONF_USE_MULTI_PRIMITIVES
  #define CHAOS_CONF_USE_TIMER_ISR        !GMW_CONF_USE_MULTI_PRIMITIVES
  #define STROBING_CONF_USE_TIMER_ISR     0
  #define GMW_PRIM1_ENABLE                0               /* don't use chaos */

#elif defined PLATFORM_DPP_CC430
  /* RF channel */
  #define GMW_CONF_RF_TX_CHANNEL          GMW_RF_TX_CHANNEL_868_6_MHz
  /* GPIO */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN              COM_GPIO1
    #define GLOSSY_TX_PIN                 COM_GPIO2
    #define GLOSSY_RX_PIN                 COM_GPIO3
    #define STROBING_START_PIN            COM_GPIO1
    #define STROBING_TX_PIN               COM_GPIO2
    #define STROBING_RX_PIN               COM_GPIO3
    #define GMW_CONF_RF_TX_POWER          GMW_RF_TX_POWER_MINUS_30_dBm
  #else
    #define GMW_CONF_RF_TX_POWER          GMW_RF_TX_POWER_PLUS_10_dBm
  #endif /* FLOCKLAB */

  /* communication primitives */
  #define GLOSSY_CONF_USE_RF1A_CALLBACKS    !GMW_CONF_USE_MULTI_PRIMITIVES
  #define STROBING_CONF_USE_RF1A_CALLBACKS  !GMW_CONF_USE_MULTI_PRIMITIVES

#else
  #error "unknown target platform"
#endif

/* Strobing config */
#define STROBING_CONF_PAYLOAD_LEN         GMW_CONF_MAX_DATA_PKT_LEN

/* GMW config */
#define GMW_CONF_USE_MULTI_PRIMITIVES     1
#define GMW_CONF_MAX_DATA_PKT_LEN         2
#define GMW_CONF_MAX_SLOTS                NUM_NODES

#define GMW_CONF_T_DATA                   15000

#define GMW_CONF_USE_CONTROL_SLOT_CONFIG  1
/* Define the slot time available in the slot_config, in us */
#define GMW_SLOT_TIME_0                   8000
#define GMW_SLOT_TIME_1                   7000
#define GMW_SLOT_TIME_2                   6000
#define GMW_SLOT_TIME_3                   5000
#define GMW_SLOT_TIME_4                   4000
#define GMW_SLOT_TIME_5                   3000
#define GMW_SLOT_TIME_6                   2000

#endif /* PROJECT_CONF_H_ */
