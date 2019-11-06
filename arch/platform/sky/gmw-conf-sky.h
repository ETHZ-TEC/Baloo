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
 * \author
 *         Jonas Baechli   jonas.baechli@bluewin.ch
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup gmw-platform
 * @{
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *
 *            Platform-specific GMW Configuration
 *            Platform: sky
 */
/*---------------------------------------------------------------------------*/

#ifndef GMW_CONF_SKY_H_
#define GMW_CONF_SKY_H_

/* config for cc2420 radio (
 * -> These defines must be added in project-conf.h!
 * -> Cause: file inclusion order
 */
#define CC2420_CONF_AUTOACK             0
#define CC2420_CONF_ADDRDECODE          0
#define CC2420_CONF_SFD_TIMESTAMPS      0

/* CPU frequency (define this in project-conf.h!) */
//#define F_CPU                           4194304UL

#define RTIMER_NOW_DCO()                rtimer_arch_now_dco()
#define rtimer_arch_now_dco()           (TBR)

/* Make sure that Glossy in configured to be able to send a control packet */
#ifndef GLOSSY_CONF_PAYLOAD_LEN
#define GLOSSY_CONF_PAYLOAD_LEN         GMW_MAX_PKT_LEN
#endif /* GLOSSY_CONF_PAYLOAD_LEN */

#ifndef GMW_CONF_RF_TX_POWER
#define GMW_CONF_RF_TX_POWER            GMW_RF_TX_PWR_0_dBm
#endif /* GMW_CONF_RF_TX_POWER */

#ifndef GMW_CONF_RF_TX_CHANNEL
#define GMW_CONF_RF_TX_CHANNEL          GMW_RF_TX_CHANNEL_2480_MHz
#endif /* GMW_CONF_RF_TX_CHANNEL */

#define GMW_CONF_RF_TX_BITRATE          250000 /* kbps */
#ifndef GMW_CONF_T_REF_OFS
#define GMW_CONF_T_REF_OFS              (GLOSSY_CONF_SETUPTIME_WITH_SYNC + 420LU)    /* us */
#endif /* GMW_CONF_T_REF_OFS */
#define GMW_RF_OVERHEAD_GLOSSY          5       /* see glossy.c */
#define GMW_RF_OVERHEAD_STROBING        4
#define GMW_CONF_RF_OVERHEAD            MAX(GMW_RF_OVERHEAD_GLOSSY, GMW_RF_OVERHEAD_STROBING)

/* min. duration of 1 packet transmission with Glossy in us for TelosB */
#define GMW_T_HOP(len)        (3 + 24 + 192 + 192 + ((len) * 8 * 1000000UL / \
                               GMW_CONF_RF_TX_BITRATE))

#define GMW_START(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal) glossy_start(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal)
#define GMW_STOP()                   glossy_stop()
#define GMW_GET_T_REF()              glossy_get_t_ref()
#define GMW_IS_T_REF_UPDATED()       glossy_is_t_ref_updated()
#define GMW_GET_N_RX()               glossy_get_rx_cnt()
#define GMW_GET_N_RX_STARTED()       glossy_get_rx_try_cnt()
#define GMW_GET_PAYLOAD_LEN()        glossy_get_payload_len()
#define GMW_GET_RELAY_CNT_FIRST_RX() glossy_get_relay_cnt()

#if GMW_CONF_USE_MULTI_PRIMITIVES
  /* all modes enabled if USE_GLOSSY_MODES (note: mode 0 is always enabled) */
  #ifndef GMW_PRIM1_ENABLE
  #define GMW_PRIM1_ENABLE           1
  #endif /* GMW_PRIM1_ENABLE */
  #ifndef GMW_PRIM2_ENABLE
  #define GMW_PRIM2_ENABLE           1
  #endif /* GMW_PRIM2_ENABLE */
  #ifndef GMW_PRIM3_ENABLE
  #define GMW_PRIM3_ENABLE           1
  #endif /* GMW_PRIM3_ENABLE */

  #define GMW_PRIM_DEFAULT           GMW_PRIM_GLOSSY     /* default Glossy */
  #define GMW_PRIM_GLOSSY            0
  #define GMW_PRIM_CHAOS             1
  #define GMW_PRIM_STROBING          2

  #if GMW_PRIM1_ENABLE
  #define GMW_START_PRIM1(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal) chaos_start(payload, payload_len, (initiator_id == node_id), n_tx_max, rf_cal)
  #define GMW_STOP_PRIM1()                    chaos_stop()
  #define GMW_GET_PAYLOAD_LEN_PRIM1()         CHAOS_CONF_PAYLOAD_LEN
  #define GMW_GET_N_RX_PRIM1()                chaos_get_rx_cnt()
  #define GMW_GET_N_RX_STARTED_PRIM1()        0
  #define GMW_GET_RELAY_CNT_FIRST_RX_PRIM1()  GMW_RELAY_COUNT_UNDEF
  #endif /* GMW_PRIM1_ENABLE */

  #if GMW_PRIM2_ENABLE
  #define GMW_START_PRIM2(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal) strobing_start((initiator_id == node_id), payload, payload_len, n_tx_max)
  #define GMW_STOP_PRIM2()                    strobing_stop()
  #define GMW_GET_PAYLOAD_LEN_PRIM2()         strobing_get_payload_len()
  #define GMW_GET_N_RX_PRIM2()                strobing_get_rx_cnt()
  #define GMW_GET_N_RX_STARTED_PRIM2()        strobing_get_rx_try_cnt()
  #define GMW_GET_RELAY_CNT_FIRST_RX_PRIM2()  GMW_RELAY_COUNT_UNDEF
  #endif /* GMW_PRIM2_ENABLE */
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */

#define GMW_CONF_RTIMER_ID           RTIMER_EXT_LF_0

#include "rtimer-ext.h"
#include "glossy.h"
#include "chaos.h"
#include "strobing.h"

typedef rtimer_ext_t        gmw_rtimer_t;
typedef rtimer_ext_clock_t  gmw_rtimer_clock_t;

typedef enum
{
  GMW_RF_TX_PWR_0_dBm = 31,
  GMW_RF_TX_PWR_MINUS_1_dBm = 27,
  GMW_RF_TX_PWR_MINUS_3_dBm = 23,
  GMW_RF_TX_PWR_MINUS_5_dBm = 19,
  GMW_RF_TX_PWR_MINUS_7_dBm = 15,
  GMW_RF_TX_PWR_MINUS_10_dBm = 11,
  GMW_RF_TX_PWR_MINUS_15_dBm = 7,
  GMW_RF_TX_PWR_MINUS_25_dBm = 3
} gmw_rf_tx_power_t;

/* 2.4GHz ZigBee channels */
typedef enum
{
  GMW_RF_TX_CHANNEL_2405_MHz = 11,
  GMW_RF_TX_CHANNEL_2410_MHz = 12,
  GMW_RF_TX_CHANNEL_2415_MHz = 13,
  GMW_RF_TX_CHANNEL_2420_MHz = 14,
  GMW_RF_TX_CHANNEL_2425_MHz = 15,
  GMW_RF_TX_CHANNEL_2430_MHz = 16,
  GMW_RF_TX_CHANNEL_2435_MHz = 17,
  GMW_RF_TX_CHANNEL_2440_MHz = 18,
  GMW_RF_TX_CHANNEL_2445_MHz = 19,
  GMW_RF_TX_CHANNEL_2450_MHz = 20,
  GMW_RF_TX_CHANNEL_2455_MHz = 21,
  GMW_RF_TX_CHANNEL_2460_MHz = 22,
  GMW_RF_TX_CHANNEL_2465_MHz = 23,
  GMW_RF_TX_CHANNEL_2470_MHz = 24,
  GMW_RF_TX_CHANNEL_2475_MHz = 25,
  GMW_RF_TX_CHANNEL_2480_MHz = 26
} gmw_rf_tx_channel_t;


/* config check */
#if GLOSSY_CONF_PAYLOAD_SIZE < GMW_MAX_PKT_LEN
#error "GLOSSY_CONF_PAYLOAD_SIZE too small"
#endif

#endif /* GMW_CONF_SKY_H_ */
