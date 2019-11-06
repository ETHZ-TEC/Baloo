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
 *            Platform: dpp-cc430
 */
/*---------------------------------------------------------------------------*/

#ifndef GMW_CONF_DPP_H_
#define GMW_CONF_DPP_H_


#ifndef GLOSSY_CONF_PAYLOAD_LEN
#define GLOSSY_CONF_PAYLOAD_LEN         GMW_MAX_PKT_LEN
#endif /* GLOSSY_CONF_PAYLOAD_LEN */

/* the RF overhead introduced by glossy and the radio platform.
 * 2 bytes for glossy header + length byte + 16-bit crc
 */
#define GMW_RF_OVERHEAD_GLOSSY          5       /* see glossy.c */
#define GMW_RF_OVERHEAD_STROBING        4
#define GMW_CONF_RF_OVERHEAD            MAX(GMW_RF_OVERHEAD_GLOSSY, GMW_RF_OVERHEAD_STROBING)

#define GMW_CONF_T_REF_OFS              (GLOSSY_CONF_SETUPTIME_WITH_SYNC + 300)    /* us */

#ifndef GMW_CONF_T_GUARD_ROUND
#define GMW_CONF_T_GUARD_ROUND          1000      /* us */
#endif /* GMW_CONF_T_GUARD_ROUND */

#define GMW_CONF_RF_TX_BITRATE          250000    /* kbps */

#ifndef GMW_CONF_RF_TX_POWER
#define GMW_CONF_RF_TX_POWER            GMW_RF_TX_POWER_0_dBm
#endif /* GMW_CONF_RF_TX_POWER */

#ifndef GMW_CONF_RF_TX_CHANNEL
#define GMW_CONF_RF_TX_CHANNEL          GMW_RF_TX_CHANNEL_870_0_MHz
#endif /* GMW_CONF_RF_TX_CHANNEL */

#define GMW_CONF_RTIMER_ID              RTIMER_EXT_LF_0

/* min. duration of 1 packet transmission with Glossy in us
 * note: TX to RX switch takes ~313us, RX to TX switch ~287us -> constant
 *       overhead is ~300us per hop, which already includes the transmission
 *       of 4 preamble bytes and the sync word (4 bytes) -> the actual 
 *       switching time is therefore approx. 44us */
#define GMW_T_HOP(len)   (300 + (len) * 8 * 1000000UL / GMW_CONF_RF_TX_BITRATE)

#define GMW_START(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal) glossy_start(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal)
#define GMW_STOP()                   glossy_stop()
#define GMW_GET_T_REF()              glossy_get_t_ref_lf()
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

  #define GMW_PRIM_DEFAULT           0     /* default Glossy */
  #define GMW_PRIM_CHAOS             1
  #define GMW_PRIM_STROBING          2

  #if GMW_PRIM2_ENABLE
  #define GMW_START_PRIM2(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal)  strobing_start((initiator_id == node_id), payload, payload_len, n_tx_max)
  #define GMW_STOP_PRIM2()                    strobing_stop()
  #define GMW_GET_PAYLOAD_LEN_PRIM2()         strobing_get_payload_len()
  #define GMW_GET_N_RX_PRIM2()                strobing_get_rx_cnt()
  #define GMW_GET_N_RX_STARTED_PRIM2()        strobing_get_rx_try_cnt()
  #define GMW_GET_RELAY_CNT_FIRST_RX_PRIM2()  GMW_RELAY_COUNT_UNDEF
  #endif /* GMW_PRIM2_ENABLE */

  /* make sure the RF1A callback functions are not defined multiple times (put this into project-conf.h!) */
  //#define GLOSSY_CONF_USE_RF1A_CALLBACKS    0
  //#define STROBING_CONF_USE_RF1A_CALLBACKS  0
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */

#ifndef RF_CONF_MAX_PKT_LEN
#define RF_CONF_MAX_PKT_LEN                 GMW_MAX_PKT_LEN
#endif /* RF_CONF_MAX_PKT_LEN */


#include "rtimer-ext.h"
#include "glossy.h"
#include "strobing.h"

typedef rtimer_ext_t        gmw_rtimer_t;
typedef rtimer_ext_clock_t  gmw_rtimer_clock_t;

typedef enum
{
  GMW_RF_TX_CHANNEL_868_0_MHz = 0,
  GMW_RF_TX_CHANNEL_868_2_MHz = 1,
  GMW_RF_TX_CHANNEL_868_4_MHz = 2,
  GMW_RF_TX_CHANNEL_868_6_MHz = 3,
  GMW_RF_TX_CHANNEL_868_8_MHz = 4,
  GMW_RF_TX_CHANNEL_869_0_MHz = 5,
  GMW_RF_TX_CHANNEL_869_2_MHz = 6,
  GMW_RF_TX_CHANNEL_869_4_MHz = 7,
  GMW_RF_TX_CHANNEL_869_6_MHz = 8,
  GMW_RF_TX_CHANNEL_869_8_MHz = 9,
  GMW_RF_TX_CHANNEL_870_0_MHz = 10
} gmw_rf_tx_channel_t;

typedef enum
{
  GMW_RF_TX_POWER_MINUS_30_dBm = 0,
  GMW_RF_TX_POWER_MINUS_12_dBm =  1,
  GMW_RF_TX_POWER_MINUS_6_dBm =  2,
  GMW_RF_TX_POWER_0_dBm =  3,
  GMW_RF_TX_POWER_PLUS_10_dBm =  4,
  GMW_RF_TX_POWER_MAX =  5
} gmw_rf_tx_power_t;


#endif /* GMW_CONF_DPP_H_ */
