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
 *            Platform: dpp-cc430
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "gpio.h"
#include "rf1a.h"
#include "glossy.h"
#include "debug-print.h"
#include "gmw.h"
/*---------------------------------------------------------------------------*/
void
gmw_platform_init(void)
{
#ifdef GLOSSY_RF_PIN
  PIN_UNSEL(GLOSSY_RF_PIN);
  PIN_CLR(GLOSSY_RF_PIN);
  PIN_CFG_OUT(GLOSSY_RF_PIN);
#endif /* GLOSSY_RF_PIN */

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

  //rtimer_ext_init();    -> is already called in platform.c
}
/*---------------------------------------------------------------------------*/
void
gmw_set_maximum_packet_length(uint8_t length)
{
  rf1a_set_maximum_packet_length(length);
}
/*---------------------------------------------------------------------------*/
void
gmw_set_rf_channel(gmw_rf_tx_channel_t channel)
{
  if(channel > 10) {
    DEBUG_PRINT_ERROR("Wrong channel setting for cc430 platform");
    DEBUG_PRINT_ERROR("Select RF channel between 0 and 10! Set to 0.");
    channel = 0;
  }
  rf1a_set_channel(channel);
}
/*---------------------------------------------------------------------------*/
void
gmw_set_tx_power(gmw_rf_tx_power_t power)
{
  /* register settings corresponding to TX power levels
   * {-30, -12, -6, 0, +10, max} dBm
   * */
  DEBUG_PRINT_VERBOSE("TX power set to %ddBm", rf1a_tx_power_val[power]);
  rf1a_set_tx_power(power);
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
  return glossy_is_active();
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
}
/*---------------------------------------------------------------------------*/
int8_t
gmw_get_rssi_last(void)
{
  return rf1a_get_rssi();
}
/*---------------------------------------------------------------------------*/
#if GMW_CONF_USE_MULTI_PRIMITIVES
/*---------------------- RF1A callback implementation -----------------------*/
void
rf1a_cb_rx_started(rtimer_ext_clock_t *timestamp)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_rx_started(timestamp);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_rx_started(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_started(rtimer_ext_clock_t *timestamp)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_tx_started(timestamp);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_tx_started(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_header_received(rtimer_ext_clock_t *timestamp,
                        uint8_t *header,
                        uint8_t packet_len)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_header_received(timestamp, header, packet_len);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_header_received(timestamp, header, packet_len);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_ended(rtimer_ext_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_rx_ended(timestamp, pkt, pkt_len);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_rx_ended(timestamp, pkt, pkt_len);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_ended(rtimer_ext_clock_t *timestamp)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_tx_ended(timestamp);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_tx_ended(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_failed(rtimer_ext_clock_t *timestamp)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_rx_failed(timestamp);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_rx_failed(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_tx_error(rtimer_ext_clock_t *timestamp)
{
#if GMW_PRIM2_ENABLE
  if(gmw_primitive == 2) {    /* strobing */
    strobing_rx_tx_error(timestamp);
  } else
#endif /* GMW_PRIM2_ENABLE */
  {
    glossy_rx_tx_error(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */
/*---------------------------------------------------------------------------*/


/** @} */
