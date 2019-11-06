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
 */

#include "contiki.h"
#include "strobing.h"
#include "rf1a.h"
#include "gpio.h"
#include "dc-stat.h"

/*---------------------------------------------------------------------------*/
#define STROBING_HEADER_LEN   1

/* mainly for debugging purposes */
#ifdef STROBING_START_PIN
#define STROBING_STARTED      PIN_SET(STROBING_START_PIN)
#define STROBING_STOPPED      PIN_CLR(STROBING_START_PIN)
#else
#define STROBING_STARTED
#define STROBING_STOPPED
#endif

#ifdef STROBING_RX_PIN
#define STROBING_RX_STARTED   PIN_SET(STROBING_RX_PIN)
#define STROBING_RX_STOPPED   PIN_CLR(STROBING_RX_PIN)
#else
#define STROBING_RX_STARTED
#define STROBING_RX_STOPPED
#endif

#ifdef STROBING_TX_PIN
#define STROBING_TX_STARTED   PIN_SET(STROBING_TX_PIN)
#define STROBING_TX_STOPPED   PIN_CLR(STROBING_TX_PIN)
#else
#define STROBING_TX_STARTED
#define STROBING_TX_STOPPED
#endif

#ifdef STROBING_RX_FAIL_PIN
#define STROBING_RX_FAIL_ON     PIN_SET(STROBING_RX_FAIL_PIN)
#define STROBING_RX_FAIL_OFF    PIN_CLR(STROBING_RX_FAIL_PIN)
#else
#define STROBING_RX_FAIL_ON
#define STROBING_RX_FAIL_OFF
#endif

#ifdef STROBING_RX_SUCCESS_PIN
#define STROBING_RX_SUCCESS_ON    PIN_SET(STROBING_RX_SUCCESS_PIN)
#define STROBING_RX_SUCCESS_OFF   PIN_CLR(STROBING_RX_SUCCESS_PIN)
#else
#define STROBING_RX_SUCCESS_ON
#define STROBING_RX_SUCCESS_OFF
#endif

/*---------------------------------------------------------------------------*/
typedef struct {
  uint8_t* payload;
  uint8_t  payload_len;
  uint8_t  active;
  uint8_t  is_initiator;
  uint8_t  header_ok;
  uint8_t  header;
  uint8_t  n_rx;
  uint8_t  n_rx_started;
  uint8_t  n_tx;
  uint8_t  n_tx_max;
  uint8_t  rx_binary[STROBING_CONF_MAX_TX_CNT/8
                     + (STROBING_CONF_MAX_TX_CNT%8 != 0)];
} strobing_state_t;
/*---------------------------------------------------------------------------*/
static strobing_state_t cfg;

/*----------------------------- main interface ------------------------------*/
void
strobing_start(uint8_t is_initiator,
               uint8_t* payload,
               uint8_t payload_len,
               uint8_t n_tx)
{
  STROBING_STARTED;

  /* reset the data structure */
  cfg.active       = 1;
  cfg.payload      = payload;
  cfg.payload_len  = payload_len;
  cfg.n_rx         = 0;
  cfg.n_rx_started = 0;
  cfg.n_tx         = 0;
  cfg.n_tx_max     = n_tx;
  cfg.header       = STROBING_CONF_HEADER_BYTE;
  memset(cfg.rx_binary, 0, sizeof(cfg.rx_binary));

  /* wake-up the radio core */
  rf1a_go_to_idle();
  /* go to TX mode at the end of RX (RX -> RX doesn't work for some reason 
   * and RX -> Idle -> RX is slower than RX -> TX -> RX) */
  rf1a_set_rxoff_mode(RF1A_OFF_MODE_FSTXON);
  /* stay in TX mode at the end of TX (but do not yet transmit the preamble) */
  rf1a_set_txoff_mode(RF1A_OFF_MODE_FSTXON);
  /* reconfigure lost registers */
  rf1a_reconfig_after_sleep();
  /* set header length */
  rf1a_set_header_len_rx(STROBING_HEADER_LEN);

  volatile uint16_t timeout;
  if(is_initiator) {
    /* INITIATOR */
    if(!cfg.payload_len) {
      strobing_stop();
      return;
    }

    cfg.is_initiator = 1;

    if(STROBING_CONF_FIRST_BYTE_AS_COUNTER) {
      cfg.payload[0] = cfg.n_tx;
    }

    rf1a_start_tx();
    rf1a_write_to_tx_fifo((uint8_t*)&cfg.header,
                          STROBING_HEADER_LEN,
                          (uint8_t*)cfg.payload,
                          cfg.payload_len);
  } else {
    /* RECEIVER */
    cfg.is_initiator = 0;
    rf1a_start_rx();
  }
  /* note: RF_RDY bit must be cleared by the radio core before entering LPM
   * after a transition from idle to RX or TX. Either poll the status of the
   * radio core (SNOP strobe) or read the GDOx signal assigned to RF_RDY */
  timeout = 500;                                   /* ~500us @13MHz (MSP430) */
  while((RF1AIN & BIT0) && timeout) timeout--;          /* check GDO0 signal */
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_stop(void)
{
  if(cfg.active) {
    /* flush both RX FIFO and TX FIFO and go to sleep */
    rf1a_flush_rx_fifo();
    rf1a_flush_tx_fifo();
    rf1a_go_to_sleep();
    rf1a_clear_pending_interrupts();
    STROBING_RX_STOPPED;
    STROBING_TX_STOPPED;
    STROBING_STOPPED;
    cfg.active = 0;
  }

  return cfg.n_rx;
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_is_active(void)
{
  return cfg.active;
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_rx_cnt(void)
{
  return cfg.n_rx;
}
/*---------------------------------------------------------------------------*/
uint8_t*
strobing_get_rx_binary(void)
{
  return (uint8_t*)(&(cfg.rx_binary));
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_rx_try_cnt(void)
{
  return cfg.n_rx_started;
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_payload_len(void)
{
  return cfg.payload_len;
}
/*---------------------- RF1A callback implementation -----------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_rx_started(rtimer_ext_clock_t *timestamp)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_rx_started(rtimer_ext_clock_t *timestamp)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  cfg.n_rx_started++;
  STROBING_RX_STARTED;
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_tx_started(rtimer_ext_clock_t *timestamp)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS*/
strobing_tx_started(rtimer_ext_clock_t *timestamp)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  STROBING_TX_STARTED;
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_header_received(rtimer_ext_clock_t *timestamp,
                        uint8_t *header,
                        uint8_t packet_len)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_header_received(rtimer_ext_clock_t *timestamp,
                         uint8_t *header,
                         uint8_t packet_len)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  /* process the header */
  if(header[0] != STROBING_CONF_HEADER_BYTE ||
     (cfg.payload_len &&
      ((cfg.payload_len + STROBING_HEADER_LEN) != packet_len)) ||
      (packet_len > STROBING_CONF_PAYLOAD_LEN + STROBING_HEADER_LEN)
      ) {
    /* the header is not ok (either wrong header signature or invalid length):
     * interrupt the reception and start a new attempt */
    rf1a_cb_rx_failed(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_rx_ended(rtimer_ext_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_rx_ended(rtimer_ext_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  STROBING_RX_STOPPED;

  /* double-check that the pkt_len value, abort if
   * - if it does not match what you expect, or
   * - if it is larger than the maximal expected size
   * */
  if(((cfg.payload_len) && ((cfg.payload_len + STROBING_HEADER_LEN) != pkt_len)) ||
     (pkt_len > STROBING_CONF_PAYLOAD_LEN + STROBING_HEADER_LEN)
     ){
    rf1a_cb_rx_failed(timestamp);
    return;
  }

  STROBING_RX_SUCCESS_ON;
  /* we have received a packet and the CRC is correct */
  cfg.payload_len = pkt_len - STROBING_HEADER_LEN;
  uint8_t* payload = pkt + STROBING_HEADER_LEN;

  if(STROBING_CONF_FIRST_BYTE_AS_COUNTER) {
    /* log a bit stream of packet reception events */
    cfg.rx_binary[payload[0]/8] |= 1 << (7-(payload[0]%8));
  }

  /* increment the reception counter */
  cfg.n_rx++;

  if(cfg.n_rx == 1) {
    /* we are a receiver and this was our first packet reception: */
    /* store the payload for the application */
    memcpy((uint8_t *)cfg.payload, payload, cfg.payload_len);
  }
  /* restart RX mode */
  rf1a_start_rx();
  STROBING_RX_SUCCESS_OFF;
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_tx_ended(rtimer_ext_clock_t *timestamp)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_tx_ended(rtimer_ext_clock_t *timestamp)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  STROBING_TX_STOPPED;

  /* increment the transmission counter */
  cfg.n_tx++;

  if(cfg.n_tx && (cfg.n_tx == cfg.n_tx_max)) {
    strobing_stop();    /* goal reached, done! */
  } else {

    if(STROBING_CONF_FIRST_BYTE_AS_COUNTER) {
      cfg.payload[0] = cfg.n_tx;
    }

    /* add some artificial delay, otherwise the receivers can't keep up */
    __delay_cycles(STROBING_CONF_TX_TO_TX_DELAY);
    rf1a_start_tx();
    rf1a_write_to_tx_fifo((uint8_t*)&cfg.header,
                          STROBING_HEADER_LEN,
                          (uint8_t*)cfg.payload, cfg.payload_len);
  }
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_rx_failed(rtimer_ext_clock_t *timestamp)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_rx_failed(rtimer_ext_clock_t *timestamp)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  /* RX has failed due to invalid CRC or invalid header */
  STROBING_RX_FAIL_ON;
  STROBING_RX_STOPPED;

  /* restart RX mode */
  rf1a_start_rx();
  STROBING_RX_FAIL_OFF;
}
/*---------------------------------------------------------------------------*/
void
#if STROBING_CONF_USE_RF1A_CALLBACKS
rf1a_cb_rx_tx_error(rtimer_ext_clock_t *timestamp)
#else /* STROBING_CONF_USE_RF1A_CALLBACKS */
strobing_rx_tx_error(rtimer_ext_clock_t *timestamp)
#endif /* STROBING_CONF_USE_RF1A_CALLBACKS */
{
  /* unspecified RX/TX error */
  if(cfg.active) {

    /* TX failed
     * -> This happens sometimes due to TX FIFO underflow. */
    if(cfg.is_initiator) {
      STROBING_TX_STOPPED;
      /* add some artificial delay, otherwise the receivers can't keep up */
      __delay_cycles(STROBING_CONF_TX_TO_TX_DELAY);
      rf1a_flush_rx_fifo();
      rf1a_flush_tx_fifo();
      rf1a_start_tx();
      if(STROBING_CONF_FIRST_BYTE_AS_COUNTER) {
        cfg.payload[0] = cfg.n_tx;
      }
      rf1a_write_to_tx_fifo((uint8_t*)&cfg.header,
                            STROBING_HEADER_LEN,
                            (uint8_t*)cfg.payload, cfg.payload_len);

    /* RX failed */
    } else {
      STROBING_RX_FAIL_ON;
      STROBING_RX_STOPPED;
      rf1a_flush_rx_fifo();
      rf1a_flush_tx_fifo();
      rf1a_start_rx();
      STROBING_RX_FAIL_OFF;
    }
  }
}
/*---------------------------------------------------------------------------*/
