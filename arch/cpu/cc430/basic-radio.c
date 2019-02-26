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
#include "basic-radio.h"
#include "rf1a.h"

/*---------------------------------------------------------------------------*/
#define RADIO_HEADER_LEN   1

/* mainly for debugging purposes */
#ifdef RADIO_START_PIN
#define RADIO_STARTED      PIN_SET(RADIO_START_PIN)
#define RADIO_STOPPED      PIN_CLR(RADIO_START_PIN)
#else
#define RADIO_STARTED
#define RADIO_STOPPED
#endif

#ifdef RADIO_RX_PIN
#define RADIO_RX_STARTED   PIN_SET(RADIO_RX_PIN)
#define RADIO_RX_STOPPED   PIN_CLR(RADIO_RX_PIN)
#else
#define RADIO_RX_STARTED
#define RADIO_RX_STOPPED
#endif

#ifdef RADIO_TX_PIN
#define RADIO_TX_STARTED   PIN_SET(RADIO_TX_PIN)
#define RADIO_TX_STOPPED   PIN_CLR(RADIO_TX_PIN)
#else
#define RADIO_TX_STARTED
#define RADIO_TX_STOPPED
#endif

/*---------------------------------------------------------------------------*/
enum {
  RADIO_STATE_OFF = 0,
  RADIO_STATE_IDLE = 1,
  RADIO_STATE_RECEIVING = 2
};
/*---------------------------------------------------------------------------*/
static uint8_t*          radio_payload;
static volatile uint8_t  radio_payload_len;
static volatile uint8_t  radio_state = RADIO_STATE_OFF;
/*---------------------------------------------------------------------------*/
void
radio_stop(void)
{
  if(radio_state != RADIO_STATE_OFF) {
    /* flush both RX FIFO and TX FIFO and go to sleep */
    rf1a_flush_rx_fifo();
    rf1a_flush_tx_fifo();
    rf1a_go_to_sleep();
    rf1a_clear_pending_interrupts();
    RADIO_RX_STOPPED;
    RADIO_TX_STOPPED;
    RADIO_STOPPED;
    radio_state = RADIO_STATE_OFF;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
radio_send(uint8_t* payload,
           uint8_t payload_len,
           uint8_t wait_until_finished)
{
  if(!payload || !payload_len || payload_len > RADIO_CONF_PAYLOAD_LEN) {
    return 0;
  }
  if(radio_state != RADIO_STATE_OFF) {
    radio_stop();   /* force off */
    return 0;       /* indicate error condition */
  }
  RADIO_STARTED;

  /* reset the data structure */
  radio_state       = RADIO_STATE_IDLE;
  radio_payload     = payload;
  radio_payload_len = payload_len;
  uint8_t header    = BASIC_RADIO_CONF_HEADER_BYTE;

  /* wake-up the radio core */
  rf1a_go_to_idle();
  rf1a_set_rxoff_mode(RF1A_OFF_MODE_IDLE);
  rf1a_set_txoff_mode(RF1A_OFF_MODE_IDLE);
#if BASIC_RADIO_CONF_DO_CALIBRATION
  /* do not calibrate automatically */
  rf1a_set_calibration_mode(RF1A_CALIBRATION_MODE_MANUAL);
  /* perform a manual calibration */
  rf1a_manual_calibration();
#endif /* BASIC_RADIO_CONF_DO_CALIBRATION */
  /* reconfigure lost registers */
  rf1a_reconfig_after_sleep();
  /* set header length */
  rf1a_set_header_len_rx(RADIO_HEADER_LEN);

  /* SENDER: start TX */
  rf1a_start_tx();
  rf1a_write_to_tx_fifo((uint8_t*)&header,
                        RADIO_HEADER_LEN,
                        (uint8_t*)radio_payload,
                        radio_payload_len);

  if(wait_until_finished) {
    while(radio_state != RADIO_STATE_OFF);

  } else {
    /* RF_RDY bit must be cleared by the radio core before entering LPM
     * after a transition from idle to RX or TX. Either poll the status of the
     * radio core (SNOP strobe) or read the GDOx signal assigned to RF_RDY */
    volatile uint16_t timeout = 500;               /* ~500us @13MHz (MSP430) */
    while((RF1AIN & BIT0) && timeout) timeout--;        /* check GDO0 signal */
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
radio_rcv(uint8_t* out_payload,
          uint16_t timeout_ms)
{
  if(!out_payload) {
    return 0;
  }
  if(radio_state != RADIO_STATE_OFF) {
    radio_stop();   /* force off */
    return 0;       /* indicate error condition */
  }
  if(timeout_ms >= 2000) {
    timeout_ms = 1999;
  }
  RADIO_STARTED;

  /* reset the data structure */
  radio_state       = RADIO_STATE_IDLE;
  radio_payload     = out_payload;
  radio_payload_len = 0;

  /* wake-up the radio core */
  rf1a_go_to_idle();
  rf1a_set_rxoff_mode(RF1A_OFF_MODE_IDLE);
  rf1a_set_txoff_mode(RF1A_OFF_MODE_IDLE);
#if BASIC_RADIO_CONF_DO_CALIBRATION
  /* do not calibrate automatically */
  rf1a_set_calibration_mode(RF1A_CALIBRATION_MODE_MANUAL);
  /* perform a manual calibration */
  rf1a_manual_calibration();
#endif /* BASIC_RADIO_CONF_DO_CALIBRATION */
  /* reconfigure lost registers */
  rf1a_reconfig_after_sleep();
  /* set header length */
  rf1a_set_header_len_rx(RADIO_HEADER_LEN);

  /* RECEIVER */
  rf1a_start_rx();

  uint16_t timeout_ticks = (uint16_t)((uint32_t)timeout_ms *
                                      RTIMER_SECOND / 1000);
  uint16_t t_start = RTIMER_NOW();
  while((timeout_ms == 0 || (RTIMER_NOW() - t_start) < timeout_ticks)
        && radio_state);

  /* timeout has expired */
  /* is an RX in progress? if so, wait until it is finished */
  while(radio_state == RADIO_STATE_RECEIVING);

  /* make sure the radio is turned off */
  radio_stop();

  return radio_payload_len;
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_started(rtimer_ext_clock_t *timestamp)
{
  radio_state = RADIO_STATE_RECEIVING;
  RADIO_RX_STARTED;
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_started(rtimer_ext_clock_t *timestamp)
{
  RADIO_TX_STARTED;
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_header_received(rtimer_ext_clock_t *timestamp,
                        uint8_t *header,
                        uint8_t packet_len)
{
  /* process the header */
  if(header[0] != BASIC_RADIO_CONF_HEADER_BYTE ||
     (radio_payload_len &&
      ((radio_payload_len + RADIO_HEADER_LEN) != packet_len))) {
    /* the header is not ok (either wrong header signature or invalid length):
     * interrupt the reception and start a new attempt */
    rf1a_cb_rx_failed(timestamp);
  }
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_ended(rtimer_ext_clock_t *timestamp, uint8_t *pkt, uint8_t pkt_len)
{
  RADIO_RX_STOPPED;

  /* we have received a packet and the CRC is correct */
  radio_payload_len = pkt_len - RADIO_HEADER_LEN;
  /* copy the received payload */
  memcpy(radio_payload, pkt + RADIO_HEADER_LEN, radio_payload_len);

  radio_stop();
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_tx_ended(rtimer_ext_clock_t *timestamp)
{
  RADIO_TX_STOPPED;

  radio_stop();
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_failed(rtimer_ext_clock_t *timestamp)
{
  RADIO_RX_STOPPED;
  /* RX has failed due to invalid CRC or invalid header */
  radio_stop();
}
/*---------------------------------------------------------------------------*/
void
rf1a_cb_rx_tx_error(rtimer_ext_clock_t *timestamp)
{
  RADIO_RX_STOPPED;
  RADIO_TX_STOPPED;
  radio_stop();
}
/*---------------------------------------------------------------------------*/
