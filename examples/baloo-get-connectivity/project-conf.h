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
 * Author:  Jonas Bächli
 *          Romain Jacob
 *          Reto Da Forno
 */

#ifndef PROJECT_CONF_H_
#define PROJECT_CONF_H_

/*
 * application specific config file to overwrite default settings
 */

/* --- definitions for FLOCKLAB --- */

/* to compile for flocklab, uncomment the following define or pass FLOCKLAB=1
 * to the make command */
//#define FLOCKLAB
#ifdef FLOCKLAB

  #include "../../tools/flocklab/flocklab.h"
  
  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
  #define STROBING_START_PIN            FLOCKLAB_LED1
  #define STROBING_TX_PIN               FLOCKLAB_INT1
  #define STROBING_RX_PIN               FLOCKLAB_INT2
#endif /* FLOCKLAB */


/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
    #define STROBING_START_PIN          ADC0
    #define STROBING_TX_PIN             ADC1
    #define STROBING_RX_PIN             ADC2
  #endif /* FLOCKLAB */
  /* RF - defined in the Makefile */
  //#define GMW_CONF_RF_TX_CHANNEL        15
  /* cc2420 specific config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL
  /* TX power setting */
  #define GMW_CONF_RF_TX_POWER            GMW_RF_TX_PWR_0_dBm

  /* communication primitives */
  #define GLOSSY_CONF_USE_TIMER_ISR       !GMW_CONF_USE_MULTI_PRIMITIVES
  #define CHAOS_CONF_USE_TIMER_ISR        !GMW_CONF_USE_MULTI_PRIMITIVES
  #define STROBING_CONF_USE_TIMER_ISR     0
  #define GMW_PRIM1_ENABLE                0               /* don't use chaos */

#elif defined PLATFORM_DPP_CC430
  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    //#define RF_GDO2_PIN                 COM_GPIO2
    #define GLOSSY_RX_PIN               COM_GPIO2
    #define GLOSSY_TX_PIN               COM_GPIO3
    /* lower TX power when running tests on the desk */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_MINUS_30_dBm
  #endif /* FLOCKLAB */
  /* RF */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_868_6_MHz

  /* communication primitives */
  #define GLOSSY_CONF_USE_RF1A_CALLBACKS    !GMW_CONF_USE_MULTI_PRIMITIVES
  #define STROBING_CONF_USE_RF1A_CALLBACKS  !GMW_CONF_USE_MULTI_PRIMITIVES

#else
  #error "unknown target platform"
#endif


/* --- GENERAL definitions --- */

#define GMW_CONF_TX_CNT_DATA            10
#define GMW_CONF_MAX_HOPS               10 /*no idea how big Graz is for now */

#ifdef FLOCKLAB
  #define HOST_ID                       1
  #define NUM_NODES                     26
  #define NODE_LIST                    { 1, 2, 3, 4, 6,\
                                         7, 8,10,11,13,\
                                        14,15,16,17,18,\
                                        19,20,22,23,24,\
                                        25,26,27,28,32,\
                                        33}
#elif defined GRAZ
  #define HOST_ID                       100
  #define NUM_NODES                     51
  #define NODE_LIST                    {100,  104,  108,  112,  116,  150,  200,  204,  208,  212,  216,  220,  224, \
101,  105,  109,  113,  117,  151,  201,  205,  209,  213,  217,  221,  225, \
102,  106,  110,  114,  118,  152,  202,  206,  210,  214,  218,  222,  226, \
103,  107,  111,  115,  119,  153,  203,  207,  211,  215,  219,  223}
#else
  #define HOST_ID                       2
  #define NUM_NODES                     3
  #define NODE_LIST                    {16,2,7}
#endif

/* Defined in the Makefile */
//#define TEST_PAYLOAD_SIZE               8
#define TEST_PERIOD                     (uint32_t)(GMW_T_SLOT_MIN(         \
                                        TEST_PAYLOAD_SIZE          \
                                        + GMW_RF_OVERHEAD_STROBING, \
                                        GMW_CONF_TX_CNT_DATA,   \
                                        GMW_CONF_MAX_HOPS) \
                                          + GMW_CONF_T_GAP)\
                                        * NUM_NODES \
                                        + 30000

/* GMW configuration */
#define GMW_CONF_USE_MULTI_PRIMITIVES       1
#define GMW_CONF_MAX_DATA_PKT_LEN       TEST_PAYLOAD_SIZE
#define GMW_CONF_MAX_SLOTS              NUM_NODES
#define GMW_CONF_MAX_CONTROL_PKT_LEN    120

/* Strobing config */
#define STROBING_CONF_PAYLOAD_LEN       GMW_CONF_MAX_DATA_PKT_LEN

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 1
#define DEBUG_PRINT_CONF_MSG_LEN        120

#endif /* PROJECT_CONF_H_ */
