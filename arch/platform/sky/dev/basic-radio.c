/*
 * Copyright (c) 2018, ETH Zurich.
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Authors: Reto Da Forno
 *          Romain Jacob
 */

#include <string.h>
#include "contiki.h"
#include "basic-radio.h"
#include "cc2420_const.h"
#include "spi-glossy.h"
#include "gpio.h"
#include "watchdog.h"
#include "energest.h"
#include "dc-stat.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Default config ------------------------------*/
/*---------------------------------------------------------------------------*/

#ifdef DCSTAT_RF_ON_PIN
#define RADIO_ON        PIN_SET(DCSTAT_RF_ON_PIN)
#define RADIO_OFF       PIN_CLR(DCSTAT_RF_ON_PIN)
#else /* DCSTAT_RF_ON_PIN */
#define RADIO_ON
#define RADIO_OFF
#endif /* DCSTAT_RF_ON_PIN */

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
/* --------------------------- Macros and defines -------------------------- */
/*---------------------------------------------------------------------------*/

#define RADIO_CRC_OK                 0x80

/* do not change */
#define RADIO_HEADER_LEN             3     /* header + footer (RSSI/CRC) */

#define RADIO_MAX_PACKET_LEN         (RADIO_CONF_PAYLOAD_LEN + \
                                      RADIO_HEADER_LEN)

#if RADIO_MAX_PACKET_LEN > 255
#error "invalid packet length for radio"
#endif /* RADIO_MAX_PACKET_LEN */

#define RADIO_LEN_FIELD              radio_buffer[0]
#define RADIO_HEADER_FIELD           radio_buffer[1]
#define RADIO_DATA_FIELD             radio_buffer[2]
#define RADIO_RSSI_FIELD             radio_buffer[packet_len - 1]
#define RADIO_CRC_FIELD              radio_buffer[packet_len]

#define RTIMER_NOW_DCO()             (TBR)

#define ENABLE_TIMERB() {\
  if((TBCTL & MC_3) == 0) {\
    TBCTL |= TBSSEL_2 | MC_2 | ID_0; /* SMCLK, continuous, divider 1 */\
  }\
}

/* Capture instants of SFD events on timer B1. */
#define SFD_CAP_INIT(edge) {\
  P4SEL |= BV(SFD);\
  TBCCTL1 = CM_3 | CAP | SCS; /* both edges */\
}

/* Enable generation of interrupts due to SFD events. */
#define ENABLE_SFD_INT()      (TBCCTL1 |= CCIE)

/* Disable generation of interrupts due to SFD events. */
#define DISABLE_SFD_INT()     (TBCCTL1 &= ~CCIE)

/* Clear interrupt flag due to SFD events. */
#define CLEAR_SFD_INT()       (TBCCTL1 &= ~CCIFG)

/*---------------------------------------------------------------------------*/
/*---------------------------- Enums and typedefs ---------------------------*/
/*---------------------------------------------------------------------------*/

/* List of possible states. */
enum {
  RADIO_STATE_OFF = 0,
  RADIO_STATE_LISTENING,
  RADIO_STATE_RECEIVING,
  RADIO_STATE_READY_FOR_TX,
  RADIO_STATE_TRANSMITTING,
};


/*---------------------------------------------------------------------------*/
/*-------------------------------- Variables --------------------------------*/
/*---------------------------------------------------------------------------*/

static uint8_t  radio_buffer[RADIO_MAX_PACKET_LEN + 1];
static uint8_t* radio_payload;
static volatile uint8_t radio_payload_len;
static volatile uint8_t radio_state = RADIO_STATE_OFF;
static volatile uint8_t packet_len,
                        bytes_read;
static volatile rtimer_clock_t t_rx_timeout;

/*---------------------------------------------------------------------------*/
/*---------------------------- Function prototypes --------------------------*/
/*---------------------------------------------------------------------------*/
inline void radio_begin_rx(void);
inline void radio_end_rx(void);
inline void radio_begin_tx(void);
inline void radio_end_tx(void);

/*---------------------------------------------------------------------------*/
/*------------------------------ Radio functions ----------------------------*/
/*---------------------------------------------------------------------------*/
static inline void
radio_flush_tx(void)
{
  FASTSPI_STROBE(CC2420_SFLUSHTX);
}
/*---------------------------------------------------------------------------*/
static inline uint8_t
radio_status(void)
{
  uint8_t status;
  FASTSPI_UPD_STATUS(status);
  return status;
}
/*---------------------------------------------------------------------------*/
static inline void
radio_start_rx(void)
{
  FASTSPI_STROBE(CC2420_SRXON);
  while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  RADIO_ON;
  DCSTAT_RF_ON;
}
/*---------------------------------------------------------------------------*/
static inline void
radio_off(void)
{
#if ENERGEST_CONF_ON
  if(energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  }
  if(energest_current_mode[ENERGEST_TYPE_LISTEN]) {
    ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  }
#endif /* ENERGEST_CONF_ON */
  FASTSPI_STROBE(CC2420_SRFOFF);
  RADIO_TX_STOPPED;
  RADIO_RX_STOPPED;
  RADIO_OFF;
  DCSTAT_RF_OFF;
}
/*---------------------------------------------------------------------------*/
static inline void
radio_flush_rx(void)
{
  uint8_t dummy;
  FASTSPI_READ_FIFO_BYTE(dummy);
  (void) dummy;   /* ignore compiler error */
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
}
/*---------------------------------------------------------------------------*/
static inline void
radio_restart_rx(void)
{
  RADIO_RX_STOPPED;
  radio_flush_rx();
  radio_state = RADIO_STATE_LISTENING;
}
/*---------------------------------------------------------------------------*/
static inline void
radio_start_tx(void)
{
  FASTSPI_STROBE(CC2420_STXON);
  RADIO_ON;
  DCSTAT_RF_ON;
  RADIO_RX_STOPPED;
  RADIO_TX_STARTED;

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}
/*---------------------------------------------------------------------------*/
static inline void
radio_write_tx(void)
{
  FASTSPI_WRITE_FIFO(radio_buffer, packet_len - 1);
}
/*---------------------------------------------------------------------------*/
/*------------------------------- Main interface ----------------------------*/
/*---------------------------------------------------------------------------*/
void
radio_stop(void)
{
  if(radio_state != RADIO_STATE_OFF) {
    // turn radio off and flush buffers
    radio_off();
    radio_flush_rx();
    radio_flush_tx();
    radio_state = RADIO_STATE_OFF;

    DISABLE_SFD_INT();
    CLEAR_SFD_INT();
    CC2420_FIFOP_INT_INIT();
    CC2420_ENABLE_FIFOP_INT();

    RADIO_STOPPED;
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
radio_send(uint8_t *payload,
           uint8_t payload_len,
           uint8_t wait_until_sent)
{
  if(!payload ||
     payload_len == 0 || payload_len > RADIO_CONF_PAYLOAD_LEN ||
     radio_state != RADIO_STATE_OFF) {
    return 0;
  }
  RADIO_STARTED;

  /* reset the data structure */
  radio_state        = RADIO_STATE_LISTENING;
  radio_payload      = payload;
  radio_payload_len  = payload_len;

  ENABLE_TIMERB();  /* make sure the timer TB is running! */
  CC2420_DISABLE_FIFOP_INT();
  CC2420_CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();

  // set packet length
  packet_len         = radio_payload_len + RADIO_HEADER_LEN;
  RADIO_LEN_FIELD    = packet_len;
  RADIO_HEADER_FIELD = RADIO_CONF_HEADER_BYTE;

  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  // initiator: copy the application data to the radio_payload field
  memcpy(&RADIO_DATA_FIELD, radio_payload, radio_payload_len);
  // set state
  radio_state = RADIO_STATE_READY_FOR_TX;
  // write the packet to the TXFIFO
  radio_write_tx();
  // start the first transmission
  radio_start_tx();

  // wait until the packet has been sent?
  if(wait_until_sent) {
    while(radio_state != RADIO_STATE_OFF) {
      watchdog_periodic();
    }
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
uint8_t
radio_rcv(uint8_t *payload,
          uint16_t timeout_ms)
{
  if(!payload || radio_state != RADIO_STATE_OFF) {
    return 0;
  }
  RADIO_STARTED;

  ENABLE_TIMERB();  /* make sure the timer TB is running! */
  CC2420_DISABLE_FIFOP_INT();
  CC2420_CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();

  // receiver: set state
  radio_state       = RADIO_STATE_LISTENING;
  // set packet length
  radio_payload_len = 0;
  radio_payload     = payload;

  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  // turn on the radio (RX mode)
  radio_start_rx();

  uint16_t timeout_ticks = (uint16_t)((uint32_t)timeout_ms *
                                      RTIMER_SECOND / 1000);
  uint16_t t_start = RTIMER_NOW();
  while((timeout_ms == 0 || (RTIMER_NOW() - t_start) < timeout_ticks)
        && radio_state) {
    // Poll the watchdog timer to prevent a reset of the node
    watchdog_periodic();
  }

  while(radio_state == RADIO_STATE_RECEIVING) {
    // Poll the watchdog timer to prevent a reset of the node
    watchdog_periodic();
  }

  radio_stop();

  return radio_payload_len;
}
/*---------------------------------------------------------------------------*/
/* ------------------------------ SFD interrupt ---------------------------- */
/*---------------------------------------------------------------------------*/
interrupt(TIMERB1_VECTOR)
timerb1_interrupt(void)
{
  DCSTAT_CPU_ON;
  uint16_t tbiv = TBIV;    // read TBIV to clear IFG
  if(radio_state == RADIO_STATE_RECEIVING && !SFD_IS_1) {
    // packet reception has finished
    radio_end_rx();

  } else {
    if(radio_state == RADIO_STATE_LISTENING && SFD_IS_1) {
      // packet reception has started
      radio_begin_rx();

    } else {
      if(radio_state == RADIO_STATE_READY_FOR_TX && SFD_IS_1) {
        // packet transmission has started
        radio_begin_tx();

      } else {
        if(radio_state == RADIO_STATE_TRANSMITTING && !SFD_IS_1) {
          // packet transmission has finished
          radio_end_tx();

        } else {
          if(tbiv == TBIV_TBCCR5) {
            // rx timeout
            if(radio_state == RADIO_STATE_RECEIVING) {
              // we are still trying to receive a packet: abort the reception
              radio_restart_rx();
            }
            // stop the RX timeout
            TBCCTL5 = 0;

          } else {
            if(radio_state != RADIO_STATE_OFF) {
              // something strange is going on: go back to the waiting state
              radio_flush_rx();
              radio_state = RADIO_STATE_LISTENING;
            }
          }
        }
      }
    }
  }
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
/*------------------------- Radio interrupt functions -----------------------*/
/*---------------------------------------------------------------------------*/
inline void
radio_begin_rx(void)
{
  RADIO_RX_STARTED;
  rtimer_clock_t t_rx_start = TBCCR1;
  radio_state = RADIO_STATE_RECEIVING;

  // wait until the FIFO pin is 1 (i.e., until the first byte is received)
  while(!FIFO_IS_1);

  // read the first byte (i.e., the len field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(RADIO_LEN_FIELD);
  // keep receiving only if it has the right length
  if((RADIO_LEN_FIELD < RADIO_HEADER_LEN) ||
     (RADIO_LEN_FIELD > RADIO_MAX_PACKET_LEN)) {
    // packet with a wrong length: abort packet reception
    radio_stop();
    return;
  }
  bytes_read = 1;
  packet_len = RADIO_LEN_FIELD;
  t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len * 35 + 200) *4;

#if !COOJA
  // wait until the FIFO pin is 1 (i.e., until the second byte is received)
  while(!FIFO_IS_1) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_stop();
      return;
    }
  }
  // read the second byte (i.e., the header field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(RADIO_HEADER_FIELD);
  // keep receiving only if it has the right header
  if(RADIO_HEADER_FIELD != RADIO_CONF_HEADER_BYTE) {
    // packet with a wrong header: abort packet reception
    radio_stop();
    return;
  }
  bytes_read = 2;
  if(packet_len > 8) {
    // if packet is longer than 8 bytes, read all bytes but the last 8
    while(bytes_read <= packet_len - 8) {
      // wait until the FIFO pin is 1 (until one more byte is received)
      while(!FIFO_IS_1) {
        if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
          radio_stop();
          return;
        }
      }
      // read another byte from the RXFIFO
      FASTSPI_READ_FIFO_BYTE(radio_buffer[bytes_read]);
      bytes_read++;
    }
  }
#endif /* COOJA */
  // schedule RX timeout
  TBCCR5  = t_rx_timeout;
  TBCCTL5 = CCIE;
}
/*---------------------------------------------------------------------------*/
inline void
radio_end_rx(void)
{
  RADIO_RX_STOPPED;
  // read the remaining bytes from the RXFIFO
  FASTSPI_READ_FIFO_NO_WAIT(&radio_buffer[bytes_read],
                            packet_len - bytes_read + 1);
  bytes_read = packet_len + 1;
#if COOJA
  if((RADIO_CRC_FIELD & RADIO_CRC_OK) &&
     (RADIO_HEADER_FIELD == RADIO_CONF_HEADER_BYTE)) {
#else
  if(RADIO_CRC_FIELD & RADIO_CRC_OK) {
#endif /* COOJA */
    // packet correctly received
    radio_payload_len = packet_len - RADIO_HEADER_LEN;
    // copy the application data from the radio_payload field
    memcpy(radio_payload, &RADIO_DATA_FIELD, radio_payload_len);

    radio_stop();

  } else {
    radio_restart_rx();
  }
}
/*---------------------------------------------------------------------------*/
inline void
radio_begin_tx(void)
{
  RADIO_TX_STARTED;
  radio_state = RADIO_STATE_TRANSMITTING;
}
/*---------------------------------------------------------------------------*/
inline void
radio_end_tx(void)
{
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  RADIO_TX_STOPPED;
  radio_stop();
}
/*---------------------------------------------------------------------------*/
/*----------------------------- Get functions -------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
radio_get_payload_len(void)
{
  return radio_payload_len;
}
/*---------------------------------------------------------------------------*/
