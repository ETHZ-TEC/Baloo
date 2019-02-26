/*
 * Copyright (c) 2018-2019, Swiss Federal Institute of Technology (ETH Zurich).
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

#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_

/*
 * application specific config file to override default settings
 */

/* --- definitions for FLOCKLAB --- */

/* to compile for flocklab , pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB

  #include "../../tools/flocklab/flocklab.h"

  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
  #define STROBING_TX_PIN               FLOCKLAB_INT1
  #define STROBING_RX_PIN               FLOCKLAB_INT2
  #define DCSTAT_RF_ON_PIN              FLOCKLAB_LED1

#ifdef PLATFORM_SKY
  #define GLOSSY_START_PIN              FLOCKLAB_LED2
  #define GMW_CONF_DEBUG_PIN            FLOCKLAB_LED3
#endif /* PLATFORM_SKY */

#endif /* FLOCKLAB */

/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY

/* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

  /* radio configuration */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2480_MHz

  /* GPIO config */
  #ifndef FLOCKLAB
    //#define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
    #define STROBING_TX_PIN             ADC1
    #define STROBING_RX_PIN             ADC2
    #define DCSTAT_RF_ON_PIN            ADC0
    #define GMW_CONF_DEBUG_PIN         ADC7
  #endif /* FLOCKLAB */

  /* communication primitives */
  #define GLOSSY_CONF_USE_TIMER_ISR       0
  #define CHAOS_CONF_USE_TIMER_ISR        0
  #define STROBING_CONF_USE_TIMER_ISR     0
  #define GMW_PRIM1_ENABLE                0  /* don't use chaos */


#elif defined PLATFORM_DPP_CC430

  /* radio configuration */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_868_6_MHz

  /* stats */
  #define DCSTAT_CONF_ON                1

  /* GPIO config */
  #ifndef FLOCKLAB

    #define DCSTAT_RF_ON_PIN            COM_GPIO1
    #define GLOSSY_TX_PIN               COM_GPIO2
    #define GLOSSY_RX_PIN               COM_GPIO3
    #define STROBING_TX_PIN             COM_GPIO2
    #define STROBING_RX_PIN             COM_GPIO3

    /* lower TX power when running tests on the desk */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_MINUS_30_dBm
  #else /* FLOCKLAB */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_PLUS_10_dBm
  #endif /* FLOCKLAB */

  /* communication primitives */
  #define GLOSSY_CONF_USE_RF1A_CALLBACKS    0
  #define STROBING_CONF_USE_RF1A_CALLBACKS  0

#else
  #error "unknown target platform"
#endif


/* --- GENERAL definitions --- */

#define HOST_ID                         1

/* stats */
#define ENERGEST_CONF_ON                1
#define ENERGEST_CONF_GET_TOTAL_TIME    rtimer_ext_now_lf


/* Sleeping-Beauty configuration */
#define SB_PAYLOAD_LENGTH               3   /* Data payload  */
#define SB_MAX_NUMBER_NODES             26  /* max is 64 */
#define SB_IPI                          10  /* seconds */
#define SB_NUMBER_SUPERFRAME            10  /* max is 255 */
#define SB_SCHED_PERIOD                 SB_NUMBER_SUPERFRAME * SB_IPI   /* seconds */
#define SB_BOOT_TIMEOUT_COUNT           3  /* num of SB_IPI before exiting bootstrap */
#define SB_MAX_RR_PAIRS                 10
#define SB_MIN_RR_PAIRS                 1
#define SB_NUMBER_STROBES               5   /* num of packets sent per node for strobing */
#define SB_NUMBER_PARENTS               5   /* num of parents in list */
#define SB_NO_ACK_SRC                   0xffff

#define SB_NUMBER_RETRANSMISSONS_DATA    2
#define SB_NUMBER_RETRANSMISSONS_REQ     2
#define SB_NUMBER_RETRANSMISSONS_ACQ     2


/* minimal time for a regular data slot in us */
#define SB_DATA_PKT_NORM_T              GMW_T_SLOT_MIN(         \
                                        SB_PAYLOAD_LENGTH       \
                                        + GMW_RF_OVERHEAD_GLOSSY, \
                                        SB_NUMBER_RETRANSMISSONS_DATA,   \
                                        GMW_CONF_MAX_HOPS)

/* minimal time for a full data slot, in us */
#define SB_DATA_PKT_FULL_T              GMW_T_SLOT_MIN(         \
                                        SB_PAYLOAD_LENGTH       \
                                        + SB_NUMBER_PARENTS     \
                                        + GMW_RF_OVERHEAD_GLOSSY, \
                                        SB_NUMBER_RETRANSMISSONS_DATA,   \
                                        GMW_CONF_MAX_HOPS)


#define SB_STROBE_PKT_LEN               2   /* sizeof(sb_strobe_struct) */

/* minimal time for a strobe slot, in us */
#define SB_STROBE_T                     GMW_T_SLOT_STROBE_MIN(  \
                                        SB_STROBE_PKT_LEN       \
                                        + GMW_RF_OVERHEAD_STROBING, \
                                        SB_NUMBER_STROBES)

#define SB_ACK_PKT_LEN                  3   /* sizeof(sb_ack_struct) */
#define SB_ACK_T                        GMW_T_SLOT_MIN(         \
                                        SB_ACK_PKT_LEN          \
                                        + GMW_RF_OVERHEAD_GLOSSY, \
                                        SB_NUMBER_RETRANSMISSONS_ACQ,   \
                                        GMW_CONF_MAX_HOPS)

#define SB_MAX_PKT_T                    MAX(MAX(SB_ACK_T,SB_STROBE_T),SB_DATA_PKT_FULL_T)


#define SB_REQ_PKT_LEN                  2   /* sizeof(sb_req_struct) */
#define SB_REQ_T                        GMW_T_SLOT_MIN(         \
                                        SB_REQ_PKT_LEN          \
                                        + GMW_RF_OVERHEAD_GLOSSY, \
                                        SB_NUMBER_RETRANSMISSONS_REQ,   \
                                        GMW_CONF_MAX_HOPS)

// Set the desired contention slot time to the middleware
#define GMW_CONF_T_CONT                 SB_REQ_T

/* Control packet configuration*/
#define GMW_CONF_USE_STATIC_SCHED           1   /* Use static sched */
#define GMW_CONF_USE_STATIC_CONFIG          1   /* Use static config */
#define GMW_CONF_USE_CONTROL_SLOT_CONFIG    1
#define GMW_CONF_USE_MAGIC_NUMBER           1
#ifdef FLOCKLAB
#define GMW_CONF_CONTROL_MAGIC_NUMBER       10
#else
#define GMW_CONF_CONTROL_MAGIC_NUMBER       39
#endif
#define GMW_CONF_TX_CNT_CONTROL             3

#define GMW_SLOT_TIME_0                     SB_ACK_T
#define GMW_SLOT_TIME_1                     SB_STROBE_T
#define GMW_SLOT_TIME_2                     SB_DATA_PKT_NORM_T
#define GMW_SLOT_TIME_3                     SB_DATA_PKT_FULL_T

#define SB_ACK_SLOT_TIME_SELECT             0x0
#define SB_STROBE_SLOT_TIME_SELECT          0x1
#define SB_NORM_DATA_SLOT_TIME_SELECT       0x2
#define SB_FULL_DATA_SLOT_TIME_SELECT       0x3



#define GMW_CONF_CONTROL_USER_BYTES         1 + 1 + (SB_MAX_NUMBER_NODES / 8) \
                                                  + ((SB_MAX_NUMBER_NODES % 8) != 0)
/*
 * sync and strobe bit + 6-bit for the current network size
 * 1 byte for the current_rr_pairs
 * n bits for the active node bit-mask
 * Round-up to get the final number of bytes
 */

#define GMW_CONF_MAX_SLOTS             2*SB_MAX_RR_PAIRS \
                                            + SB_MAX_NUMBER_NODES \
                                            + SB_MAX_NUMBER_NODES - 1

#define GMW_CONF_MAX_DATA_PKT_LEN       SB_PAYLOAD_LENGTH       \
                                        + SB_NUMBER_PARENTS

/* GMW config */
#define GMW_CONF_USE_MULTI_PRIMITIVES   1

#define GMW_CONF_PERIOD_TIME_BASE		    GMW_CONF_PERIOD_TIME_BASE_1s

#define GMW_CONF_T_GAP_CONTROL          15000 //us
#define GMW_CONF_T_GAP                  10000 //us

#define GMW_CONF_T_POSTPROCESS_MIN      10 // ms

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 1
#define DEBUG_PRINT_CONF_BUFFER_SIZE    1024

#endif /* __CONFIG_H__ */
