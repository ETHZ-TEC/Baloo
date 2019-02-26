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
 * application specific config file to overwrite default settings
 */

/* --- definitions for FLOCKLAB --- */
#ifdef FLOCKLAB

  #include "../../tools/flocklab/flocklab.h"

  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
  #define DCSTAT_RF_ON_PIN              FLOCKLAB_LED1

#ifdef PLATFORM_SKY
  #define GLOSSY_START_PIN              FLOCKLAB_LED2
  #define EVENT_PIN                     FLOCKLAB_SIG1

#endif /* PLATFORM_SKY */

#endif /* FLOCKLAB */

/* --- definitions for GRAZ --- */
#ifdef GRAZ
  #include "../../tools/competition/graz.h"
  #define GLOSSY_START_PIN              GPIO17
  #define GLOSSY_TX_PIN                 GPIO4
  #define GLOSSY_RX_PIN                 GPIO18
  #define EEPROM_PIN                    GPIO25
  //#define ACT_PIN                       GPIO4

  #define EVENT_PIN                     PORT2, PIN6

#endif /* GRAZ */
/* Mapping GPIO-bits for visualization in Grafana
  # GPIO17   // 0x80
  # GPIO4    // 0x40
  # GPIO18   // 0x20
  # GPIO27   // 0x10
  # GPIO22   // 0x08
  # GPIO23   // 0x04
  # GPIO24   // 0x02
  # GPIO25   // 0x01
 */

/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY

/* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

  /* radio configuration */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_2480_MHz //26

  /* GPIO config */
  #if !defined FLOCKLAB && !defined GRAZ
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
  #endif /* FLOCKLAB */

#elif defined PLATFORM_DPP_CC430

  /* radio configuration */
  #define GMW_CONF_RF_TX_CHANNEL        GMW_RF_TX_CHANNEL_868_6_MHz

  /* stats */
  #define DCSTAT_CONF_ON                0

/* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    #define GLOSSY_RX_PIN               COM_GPIO2
    #define GLOSSY_TX_PIN               COM_GPIO3
    //#define GMW_NOISE_DETECT_PIN      COM_GPIO2
    //#define GMW_GLOSSY_DETECT_PIN     COM_GPIO3
    //#define DCSTAT_RF_ON_PIN          COM_GPIO1

    /* lower TX power when running tests on the desk */
    #define GMW_CONF_RF_TX_POWER        GMW_RF_TX_POWER_MINUS_30_dBm

  #endif /* FLOCKLAB */

#else
  #error "unknown target platform"
#endif

/* Parse configuration */
#define HOST_ID                         cfg.p[0].destination_id[0]
#define CRYSTAL_PERIOD                  MAX((cfg.p[0].periodicity), 1000)
#define CRYSTAL_PAYLOAD_LENGTH          cfg.p[0].msg_length
#define CRYSTAL_SINK_MAX_EMPTY_TS       custom_cfg.n_empty_ta
#define GMW_CONF_TX_CNT_DATA            custom_cfg.n_tx_max
#define GMW_CONF_RF_TX_POWER            custom_cfg.tx_power

/* --- GENERAL definitions --- */

/* stats */
#define ENERGEST_CONF_ON                0
#define ENERGEST_CONF_GET_TOTAL_TIME    rtimer_ext_now_lf

/* Crystal configuration */
//#define CRYSTAL_PAYLOAD_LENGTH          0   /* Data payload is one byte */
//#define CRYSTAL_PERIOD                  2000   // ms
#define CRYSTAL_NACK_NODE               0
#define CRYSTAL_NACK_SEQN               65535

#define CRYSTAL_NB_CONCURRENT_SENDER    8
#define CRYSTAL_START_EPOCH             10  // epochs before we start hopping channels
#define CRYSTAL_ACTIVE_EPOCHS           1   // Number of 'different' epochs

#define CRYSTAL_DATA_PKT_LEN            (2+2+ CRYSTAL_PAYLOAD_LENGTH) // payload
#define CRYSTAL_ACK_PKT_LEN             (2+1+1+2+2)


#define CRYSTAL_TA_DURATION         ((GMW_CONF_T_CONT + GMW_CONF_T_DATA \
                                    + (2 * GMW_CONF_T_GAP)) / 1000LU)    // in ms

// max number of TA pairs (to avoid rounds overruning)
#define CRYSTAL_MAX_TAS             (CRYSTAL_PERIOD - 100 ) \
                                    / CRYSTAL_TA_DURATION
                                    // Rounds will automatically end
                                    // approx. 100ms before the next one.

#define N_FULL_EPOCHS               0   // number of rounds where nodes
                                        // don't go to sleep... (why?)

// default values from the Crystal repo
//#define CRYSTAL_SINK_MAX_EMPTY_TS   2   // R
#define CRYSTAL_SINK_MAX_NOISY_TS   6   // X
#define CRYSTAL_MAX_SILENT_TAS      2   // Y
#define CRYSTAL_MAX_MISSING_ACKS    4   // Z
#define CRYSTAL_MAX_NOISY_AS        CRYSTAL_SINK_MAX_NOISY_TS
                                    // node: max allowed number of noisy empty As

// additional feature we don't use for now
// (modify the sleep test at the host)
#define CRYSTAL_USE_DYNAMIC_NEMPTY  0   // dyn_nempty: not sure what that is...

#if CRYSTAL_USE_DYNAMIC_NEMPTY
#define CRYSTAL_SINK_MAX_EMPTY_TS_DYNAMIC(n_ta_) (((n_ta_)>1)?(CRYSTAL_SINK_MAX_EMPTY_TS):1)
#warning ------------- !!! USING DYNAMIC N_EMPTY !!! -------------
#else
#define CRYSTAL_SINK_MAX_EMPTY_TS_DYNAMIC(n_ta_) CRYSTAL_SINK_MAX_EMPTY_TS
#endif

/* GMW configuration */

/* Control packet configuration*/
#define GMW_CONF_USE_STATIC_SCHED           1   /* Use static sched */
#define GMW_CONF_USE_STATIC_CONFIG          1   /* Use static config */
#define GMW_CONF_USE_CONTROL_SLOT_CONFIG    1   /* Use slot config */
#define GMW_CONF_CONTROL_USER_BYTES         2   /* Two bytes of 'user bytes' */

#define GMW_CONF_USE_AUTOCLEAN              1
#define GMW_CONF_USE_MAGIC_NUMBER           0
#define GMW_CONF_CONTROL_MAGIC_NUMBER       66

#if GMW_CONF_USE_MAGIC_NUMBER
  #define GMW_CONF_MAX_CONTROL_PKT_LEN    (GMW_CONF_CONTROL_USER_BYTES + 1)
                                          /* Control packet is very small
                                           * since both schedule and config are static
                                           */
#else
  #define GMW_CONF_MAX_CONTROL_PKT_LEN    (GMW_CONF_CONTROL_USER_BYTES)
                                          /* Control packet is very small
                                           * since both schedule and config are static
                                           */
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/

#define GMW_CONF_MAX_HOPS               5
#define GMW_CONF_MAX_DATA_PKT_LEN       68 // 64 Bytes payload + 4B Crystal header
/* Set the length of the contention slot to the min value
 * required to send a **data** packet */
#define GMW_CONF_T_CONT                 GMW_T_SLOT_MIN(\
                                          2+2+CRYSTAL_PAYLOAD_LENGTH \
                                          + GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_DATA, \
                                          GMW_CONF_MAX_HOPS)
/* Set the length of the data slot to the min value
 * required to send an **acknowledgment** packet */
#define GMW_CONF_T_DATA                 GMW_T_SLOT_MIN(\
                                          CRYSTAL_ACK_PKT_LEN \
                                          + GMW_CONF_RF_OVERHEAD, \
                                          4, \
                                          GMW_CONF_MAX_HOPS)
// We write packets at the destination in the slot_post_cb
// -> Make sure gap is big enough such that observer has time to read
//    data between to write operations.
#define T_EEPROM_WRITE                  2500 // us, arbitrary value for now
#define T_EEPROM_READ                   2    // ms, arbitrary value for now
#define GMW_CONF_T_GAP                  MAX( (20000LU - GMW_CONF_T_DATA), \
                                             T_EEPROM_WRITE)

#define GMW_CONF_MAX_SLOTS              2 /* one TA pair, that eventually gets repeated */

#define GMW_CONF_PERIOD_TIME_BASE		    GMW_CONF_PERIOD_TIME_BASE_1ms

#define GMW_CONF_T_PREPROCESS           T_EEPROM_READ //ms
#define GMW_CONF_T_POSTPROCESS_MIN      10            // ms

#define GMW_CONF_USE_NOISE_DETECTION    0
/* A reasonable threshold is 3dB above the expected
 * sensitivity of the radio... */
#define GMW_CONF_HIGH_NOISE_THRESHOLD   -60
#define GMW_CONF_HIGH_NOISE_MIN_COUNT   80

/* debug config */
#define DEBUG_PRINT_CONF_BUFFER_SIZE    700
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_EMERGENCY
#define DEBUG_PRINT_CONF_PRINT_DBGLEVEL 1

/* energy optimization */
#define UART1_CONF_RX_WITH_DMA          0   /* DMA requires DCO, hence cannot enter LPM3 -> disable DMA */
#define RTIMER_EXT_CONF_USE_ETIMER      0

#endif /* __CONFIG_H__ */
