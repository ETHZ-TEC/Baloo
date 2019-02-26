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
 */

#include <string.h>
#include "contiki.h"
#include "strobing.h"
#include "cc2420_const.h"
#include "rtimer-ext.h"
#include "spi-glossy.h"
#include "watchdog.h"
#include "node-id.h"
#include "gpio.h"
#include "energest.h"
#include "dc-stat.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Default config ------------------------------*/
/*---------------------------------------------------------------------------*/

#ifdef DCSTAT_RF_ON_PIN
#define STROBING_RF_ON        PIN_SET(DCSTAT_RF_ON_PIN)
#define STROBING_RF_OFF       PIN_CLR(DCSTAT_RF_ON_PIN)
#else /* DCSTAT_RF_ON_PIN */
#define STROBING_RF_ON
#define STROBING_RF_OFF
#endif /* DCSTAT_RF_ON_PIN */

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

/*---------------------------------------------------------------------------*/
/* --------------------------- Macros and defines -------------------------- */
/*---------------------------------------------------------------------------*/

#define CM_POS                          CM_1
#define CM_NEG                          CM_2
#define CM_BOTH                         CM_3

#define STROBING_CRC_OK                 0x80

/* do not change */
#define STROBING_HEADER_LEN             3     /* header + footer (RSSI/CRC) */

#define STROBING_MAX_PACKET_LEN         (STROBING_CONF_PAYLOAD_LEN + \
                                         STROBING_HEADER_LEN)

#if STROBING_MAX_PACKET_LEN > 255
#error "invalid packet length for strobing"
#endif /* STROBING_MAX_PACKET_LEN */

#define STROBING_LEN_FIELD              strobing_buffer[0]
#define STROBING_HEADER_FIELD           strobing_buffer[1]
#define STROBING_DATA_FIELD             strobing_buffer[2]
#define STROBING_RSSI_FIELD             strobing_buffer[packet_len_tmp - 1]
#define STROBING_CRC_FIELD              strobing_buffer[packet_len_tmp]

#define RTIMER_NOW_DCO()                rtimer_arch_now_dco()
#define rtimer_arch_now_dco()           (TBR)

/* Capture instants of SFD events on timer B1. */
#define SFD_CAP_INIT(edge) {\
  P4SEL |= BV(SFD);\
  TBCCTL1 = edge | CAP | SCS;\
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
enum strobing_state {
  STROBING_STATE_OFF = 0,      /**< is not executing */
  STROBING_STATE_WAITING,      /**< is waiting for a packet being flooded */
  STROBING_STATE_RECEIVING,    /**< is receiving a packet */
  STROBING_STATE_READY_FOR_TX, /**< ready to transmit a packet */
  STROBING_STATE_TRANSMITTING, /**< is transmitting a packet */
  STROBING_STATE_TRANSMITTED,  /**< has just finished transmitting a packet */
  STROBING_STATE_ABORTED       /**< has just aborted a packet reception */
};


/*---------------------------------------------------------------------------*/
/*-------------------------------- Variables --------------------------------*/
/*---------------------------------------------------------------------------*/

static uint8_t strobing_buffer[STROBING_MAX_PACKET_LEN + 1];
static uint8_t *strobing_payload;
static uint8_t strobing_payload_len,
               packet_len,
               packet_len_tmp,
               rx_cnt,
               rx_try_cnt,
               tx_cnt,
               tx_max,
               bytes_read;
static rtimer_clock_t t_rx_timeout;
static volatile uint8_t state;
static uint16_t rx_timeout,
                bad_length,
                bad_header,
                bad_crc;

/*---------------------------------------------------------------------------*/
/*---------------------------- Function prototypes --------------------------*/
/*---------------------------------------------------------------------------*/
inline void strobing_begin_rx(void);
inline void strobing_end_rx(void);
inline void strobing_begin_tx(void);
inline void strobing_end_tx(void);

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
  STROBING_RF_ON;
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
  STROBING_TX_STOPPED;
  STROBING_RX_STOPPED;
  STROBING_RF_OFF;
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
  STROBING_RX_STOPPED;
  state = STROBING_STATE_ABORTED;
  radio_flush_rx();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_abort_tx(void)
{
  FASTSPI_STROBE(CC2420_SRXON); // start listening (RX)
  STROBING_TX_STOPPED;
#if ENERGEST_CONF_ON
  if(energest_current_mode[ENERGEST_TYPE_TRANSMIT]) {
    ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
    ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  }
#endif /* ENERGEST_CONF_ON */
  radio_flush_rx();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_start_tx(void)
{
  FASTSPI_STROBE(CC2420_STXON);
  STROBING_RF_ON;
  DCSTAT_RF_ON;
  STROBING_RX_STOPPED;
  STROBING_TX_STARTED;

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}
/*---------------------------------------------------------------------------*/
static inline void
radio_write_tx(void)
{
  FASTSPI_WRITE_FIFO(strobing_buffer, packet_len_tmp - 1);
}
/*---------------------------------------------------------------------------*/
/*------------------------------- Main interface ----------------------------*/
/*---------------------------------------------------------------------------*/
void
strobing_start(uint8_t is_initiator,
               uint8_t *payload,
               uint8_t payload_len,
               uint8_t n_tx_max)
{
  STROBING_STARTED;

  strobing_payload     = payload;
  strobing_payload_len = payload_len;
  tx_max               = n_tx_max;
  tx_cnt = 0;
  rx_cnt = 0;

  CC2420_DISABLE_FIFOP_INT();
  CC2420_CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();

  // set packet length
  if(strobing_payload_len) {
    packet_len_tmp        = strobing_payload_len + STROBING_HEADER_LEN;
    packet_len            = packet_len_tmp;
    STROBING_LEN_FIELD    = packet_len_tmp;
    STROBING_HEADER_FIELD = STROBING_CONF_HEADER_BYTE;
  } else {
    // packet length not known yet (only for receivers)
    packet_len = 0;
  }
  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  if(is_initiator && strobing_payload_len) {
    // initiator: copy the application data to the strobing_payload field
    memcpy(&STROBING_DATA_FIELD, strobing_payload, strobing_payload_len);
    // set state
    state = STROBING_STATE_READY_FOR_TX;
    // write the packet to the TXFIFO
    radio_write_tx();
    // start the first transmission
    radio_start_tx();
  } else {
    // receiver: set state
    state = STROBING_STATE_WAITING;
    // turn on the radio (RX mode)
    radio_start_rx();
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_stop(void)
{
  // turn radio off and flush buffers
  radio_off();
  radio_flush_rx();
  radio_flush_tx();
  state = STROBING_STATE_OFF;

  DISABLE_SFD_INT();
  CLEAR_SFD_INT();
  CC2420_FIFOP_INT_INIT();
  CC2420_ENABLE_FIFOP_INT();

  STROBING_STOPPED;

  // return the number of times the packet has been received
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
/* ------------------------------ SFD interrupt ---------------------------- */
/*---------------------------------------------------------------------------*/
#if STROBING_CONF_USE_TIMER_ISR
interrupt(TIMERB1_VECTOR)
timerb1_interrupt(void)
#else /* STROBING_CONF_USE_TIMER_ISR */
void
strobing_timer_int_cb(void)
#endif /* STROBING_CONF_USE_TIMER_ISR */
{
  DCSTAT_CPU_ON;
  uint16_t tbiv = TBIV;    // read TBIV to clear IFG
  if(state == STROBING_STATE_RECEIVING && !SFD_IS_1) {
    // packet reception has finished
    strobing_end_rx();

  } else {
#if RTIMER_EXT_CONF_HF_ENABLE
    if(tbiv == TBIV_TBIFG) {
      rtimer_ext_notify_hf_timer_overflow();

    } else
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
    if(state == STROBING_STATE_WAITING && SFD_IS_1) {
      // packet reception has started
      strobing_begin_rx();

    } else {
      if(state == STROBING_STATE_READY_FOR_TX && SFD_IS_1) {
        // packet transmission has started
        strobing_begin_tx();

      } else {
        if(state == STROBING_STATE_TRANSMITTING && !SFD_IS_1) {
          // packet transmission has finished
          strobing_end_tx();

        } else {
          if(state == STROBING_STATE_ABORTED) {
            // packet reception has been aborted
            state = STROBING_STATE_WAITING;

          } else {
            if(tbiv == TBIV_TBCCR5) {
              // rx timeout
              if(state == STROBING_STATE_RECEIVING) {
                // we are still trying to receive a packet: abort the reception
                radio_restart_rx();
                rx_timeout++;
              }
              // stop the RX timeout
              TBCCTL5 = 0;

            } else {
              if(state != STROBING_STATE_OFF) {
                // something strange is going on: go back to the waiting state
                radio_flush_rx();
                state = STROBING_STATE_WAITING;
              }
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
strobing_begin_rx(void)
{
  STROBING_RX_STARTED;
  rtimer_clock_t t_rx_start = TBCCR1;
  rx_try_cnt++;
  state = STROBING_STATE_RECEIVING;
  if(packet_len) {
    // Rx timeout: packet duration + 200 us
    // (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
    t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len_tmp * 35 + 200) * 4;
  }
  // wait until the FIFO pin is 1 (i.e., until the first byte is received)
  while(!FIFO_IS_1) {
    if(packet_len && !RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_restart_rx();
      rx_timeout++;
      return;
    }
  }
  // read the first byte (i.e., the len field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(STROBING_LEN_FIELD);
  // keep receiving only if it has the right length
  if((packet_len && (STROBING_LEN_FIELD != packet_len_tmp)) ||
     (STROBING_LEN_FIELD < STROBING_HEADER_LEN) ||
     (STROBING_LEN_FIELD > STROBING_MAX_PACKET_LEN)) {
    // packet with a wrong length: abort packet reception
    radio_restart_rx();
    bad_length++;
    return;
  }
  bytes_read = 1;
  if(!packet_len) {
    packet_len_tmp = STROBING_LEN_FIELD;
    t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len_tmp * 35 + 200) *4;
  }

#if !COOJA
  // wait until the FIFO pin is 1 (i.e., until the second byte is received)
  while(!FIFO_IS_1) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_restart_rx();
      rx_timeout++;
      return;
    }
  }
  // read the second byte (i.e., the header field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(STROBING_HEADER_FIELD);
  // keep receiving only if it has the right header
  if(STROBING_HEADER_FIELD != STROBING_CONF_HEADER_BYTE) {
    // packet with a wrong header: abort packet reception
    radio_restart_rx();
    bad_header++;
    return;
  }
  bytes_read = 2;
  if(packet_len_tmp > 8) {
    // if packet is longer than 8 bytes, read all bytes but the last 8
    while(bytes_read <= packet_len_tmp - 8) {
      // wait until the FIFO pin is 1 (until one more byte is received)
      while(!FIFO_IS_1) {
        if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
          radio_restart_rx();
          rx_timeout++;
          return;
        }
      }
      // read another byte from the RXFIFO
      FASTSPI_READ_FIFO_BYTE(strobing_buffer[bytes_read]);
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
strobing_end_rx(void)
{
  STROBING_RX_STOPPED;
  // read the remaining bytes from the RXFIFO
  FASTSPI_READ_FIFO_NO_WAIT(&strobing_buffer[bytes_read],
                            packet_len_tmp - bytes_read + 1);
  bytes_read = packet_len_tmp + 1;
#if COOJA
  if((STROBING_CRC_FIELD & STROBING_CRC_OK) &&
     (STROBING_HEADER_FIELD == STROBING_CONF_HEADER_BYTE)) {
#else
  if(STROBING_CRC_FIELD & STROBING_CRC_OK) {
#endif /* COOJA */
    // packet correctly received
    rx_cnt++;
    if(!packet_len) {
      packet_len           = packet_len_tmp;
      strobing_payload_len = packet_len_tmp - STROBING_HEADER_LEN;
    }
    if(rx_cnt == 1) {
      // copy the application data from the strobing_payload field
      memcpy(strobing_payload, &STROBING_DATA_FIELD, strobing_payload_len);
    }
    state = STROBING_STATE_WAITING;
    radio_flush_rx();
    radio_start_rx();

  } else {
    bad_crc++;
    // packet corrupted, abort the transmission before it actually starts
  }
}
/*---------------------------------------------------------------------------*/
inline void
strobing_begin_tx(void)
{
  STROBING_TX_STARTED;
  state = STROBING_STATE_TRANSMITTING;
}
/*---------------------------------------------------------------------------*/
inline void
strobing_end_tx(void)
{
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  STROBING_TX_STOPPED;
  tx_cnt++;
  if(tx_cnt == tx_max) {
    // stop
    radio_off();
    state = STROBING_STATE_OFF;

  } else {
    // restart TX with a delay
    state = STROBING_STATE_READY_FOR_TX;
    __delay_cycles(STROBING_CONF_TX_TO_TX_DELAY);
    radio_flush_tx();
    radio_write_tx();
    radio_start_tx();
  }
}
/*---------------------------------------------------------------------------*/
/*----------------------------- Get functions -------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_rx_cnt(void)
{
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_rx_try_cnt(void)
{
  return rx_try_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
strobing_get_payload_len(void)
{
  return strobing_payload_len;
}
/*---------------------------------------------------------------------------*/
