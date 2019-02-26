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
 *            Platform dependent implementation for GMW
 *            Platform: sky mote
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "cc2420.h"
#include "gpio.h"
#include "debug-print.h"
#include "gmw.h"
#include "glossy.h"
/*---------------------------------------------------------------------------*/
void
gmw_platform_init(void)
{
#if GMW_CONF_USE_NOISE_DETECTION
  /*
   * the cc2420 has ~45 dB offset on the RSSI threshold setting
   * (see Data Sheet Sec.23 RSSI / Energy Detection)
   */
  cc2420_set_cca_threshold(GMW_CONF_HIGH_NOISE_THRESHOLD + 45);
#endif /* GMW_CONF_USE_NOISE_DETECTION */

#ifdef DCSTAT_RF_ON_PIN
  PIN_UNSEL(DCSTAT_RF_ON_PIN);
  PIN_CLR(DCSTAT_RF_ON_PIN);
  PIN_CFG_OUT(DCSTAT_RF_ON_PIN);
#endif /* DCSTAT_RF_ON_PIN */

#ifdef GLOSSY_START_PIN
  PIN_UNSEL(GLOSSY_START_PIN);
  PIN_CLR(GLOSSY_START_PIN);
  PIN_CFG_OUT(GLOSSY_START_PIN);
#endif /* GLOSSY_START_PIN */

#ifdef GLOSSY_TX_PIN
  PIN_UNSEL(GLOSSY_TX_PIN);
  PIN_CLR(GLOSSY_TX_PIN);
  PIN_CFG_OUT(GLOSSY_TX_PIN);
#endif /* GLOSSY_TX_PIN */

#ifdef GLOSSY_RX_PIN
  PIN_UNSEL(GLOSSY_RX_PIN);
  PIN_CLR(GLOSSY_RX_PIN);
  PIN_CFG_OUT(GLOSSY_RX_PIN);
#endif /* GLOSSY_RX_PIN */

#ifdef GMW_CONF_TASK_ACT_PIN
  PIN_CFG_OUT(GMW_CONF_TASK_ACT_PIN);
  PIN_CLR(GMW_CONF_TASK_ACT_PIN);
#endif /* GMW_CONF_TASK_ACT_PIN */

#ifdef GMW_CONF_SLOT_ACT_PIN
  PIN_CFG_OUT(GMW_CONF_SLOT_ACT_PIN);
  PIN_CLR(GMW_CONF_SLOT_ACT_PIN);
#endif /* GMW_CONF_SLOT_ACT_PIN */

#ifdef GMW_NOISE_DETECT_PIN
  PIN_UNSEL(GMW_NOISE_DETECT_PIN);
  PIN_CFG_OUT(GMW_NOISE_DETECT_PIN);
  PIN_CLR(GMW_NOISE_DETECT_PIN);
#endif /* GMW_NOISE_DETECT_PIN */

#ifdef GMW_GLOSSY_DETECT_PIN
  PIN_UNSEL(GMW_GLOSSY_DETECT_PIN);
  PIN_CFG_OUT(GMW_GLOSSY_DETECT_PIN);
  PIN_CLR(GMW_GLOSSY_DETECT_PIN);
#endif /* GMW_GLOSSY_DETECT_PIN */

#ifdef GMW_CONF_DEBUG_PIN
  PIN_UNSEL(GMW_CONF_DEBUG_PIN);
  PIN_CFG_OUT(GMW_CONF_DEBUG_PIN);
  PIN_CLR(GMW_CONF_DEBUG_PIN);
#endif /* GMW_CONF_DEBUG_PIN */

  rtimer_ext_init();
}
/*---------------------------------------------------------------------------*/
void
gmw_set_maximum_packet_length(uint8_t length)
{
  /* TODO if possible */
}
/*---------------------------------------------------------------------------*/
void
gmw_set_rf_channel(gmw_rf_tx_channel_t channel)
{
  if(channel > 26 || channel < 11) {
    DEBUG_PRINT_MSG_NOW("Wrong channel setting for the tmote sky (%u)", channel);
    DEBUG_PRINT_MSG_NOW("Select RF channel between 10 and 26! Set to 26.");
    channel = 26;
  }
  cc2420_set_channel(channel);
}
/*---------------------------------------------------------------------------*/
void
gmw_set_tx_power(gmw_rf_tx_power_t power)
{
/* Available power levels:
 * 0 -1 -3 -5 -7 -10 -15 and -25 dBm
 */
  /*if(power > 0 || power < -25) {
    DEBUG_PRINT_MSG_NOW("Wrong TX power setting for the tmote sky");
    DEBUG_PRINT_MSG_NOW("Select TX power between 0 and -25 dBm! Set to 0dBm.");
    power = 0;
  }*/
  cc2420_set_txpower(power);
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_high_noise_detected(void)
{
#if GMW_CONF_USE_NOISE_DETECTION
  return gmw_high_noise_test();
#else
  return 0;
#endif
}
/*---------------------------------------------------------------------------*/
uint8_t
gmw_communication_active(void)
{
#if GMW_CONF_USE_MULTI_PRIMITIVES
  DEBUG_PRINT_VERBOSE("Noise detection not implemented for multiple modes.");
  return 0;
#else  /* GMW_CONF_USE_MULTI_PRIMITIVES */
  return (glossy_get_state());
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
}
/*---------------------------------------------------------------------------*/
int8_t
gmw_get_rssi_last(void)
{
  return ((int8_t)cc2420_rssi());
}
/*---------------------------------------------------------------------------*/
#if GMW_CONF_USE_MULTI_PRIMITIVES
/* if different communication primitives are to be used, then
 * the timer interrupt needs to be defined here */
interrupt(TIMERB1_VECTOR) // __attribute__ ((section(".chaos")))
timerb1_interrupt(void)
{
#if GMW_PRIM1_ENABLE
  if(gmw_primitive == 1) {    /* Chaos */
    chaos_timer_int_cb();
  } else
#endif /* GMW_PRIM1_ENABLE */
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {
    strobing_timer_int_cb();
  } else
#endif /* GMW_PRIM2_ENABLE */
  {                             /* default (Glossy) */
    glossy_timer_int_cb();
  }
}
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
/*---------------------------------------------------------------------------*/

/** @} */
