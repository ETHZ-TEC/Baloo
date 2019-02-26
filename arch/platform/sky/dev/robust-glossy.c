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
 * Authors: Federico Ferrari
 *          Reto Da Forno
 */

/*
 * Notes:
 *
 * - In order to use channel hopping, the payload length of the packet must be known in advance.
 *   Therefore, not only the transmitter, but also receivers must pass the correct payload length to glossy_start().
 *   If the payload length is set to zero, the receiver will not do channel hopping (stays on a preferred channel).
 * - Use glossy_set_channel_and_payload_len() to set the payload length manually if you can't pass it via glossy_start()
 *   or to set the preferred channel.
 * - GLOSSY_CONF_SETUPTIME_WITH_SYNC must be set to a valid value (non-zero)
 * - GLOSSY_CONF_GUARD_TIME must be set to a reasonable value
 * - The upper layer must ensure that sender and receiver call glossy_start() at the exact same time,
 *   therefore the upper layer is not supposed to add additional guard times of any kind
 */

#include <string.h>
#include "glossy.h"
#include "cc2420.h"
#include "spi-glossy.h"
#include "watchdog.h"
#include "node-id.h"
#include "gpio.h"
#include "energest.h"
#include "random.h"
#include "dc-stat.h"
#include "debug-print.h"

/*---------------------------------------------------------------------------*/
/*----------------------------- Default config ------------------------------*/
/*---------------------------------------------------------------------------*/

#ifdef DCSTAT_RF_ON_PIN
#define GLOSSY_RF_ON        PIN_SET(DCSTAT_RF_ON_PIN)
#define GLOSSY_RF_OFF       PIN_CLR(DCSTAT_RF_ON_PIN)
#else /* DCSTAT_RF_ON_PIN */
#define GLOSSY_RF_ON
#define GLOSSY_RF_OFF
#endif /* DCSTAT_RF_ON_PIN */

/* mainly for debugging purposes */
#ifdef GLOSSY_START_PIN
#define GLOSSY_STARTED      PIN_SET(GLOSSY_START_PIN)
#define GLOSSY_STOPPED      PIN_CLR(GLOSSY_START_PIN)
#define GLOSSY_START_TOGGLE PIN_XOR(GLOSSY_START_PIN)
#else
#define GLOSSY_STARTED
#define GLOSSY_STOPPED
#define GLOSSY_START_TOGGLE
#endif

#ifdef GLOSSY_RX_PIN
#define GLOSSY_RX_STARTED   PIN_SET(GLOSSY_RX_PIN)
#define GLOSSY_RX_STOPPED   PIN_CLR(GLOSSY_RX_PIN)
#else
#define GLOSSY_RX_STARTED
#define GLOSSY_RX_STOPPED
#endif

#ifdef GLOSSY_TX_PIN
#define GLOSSY_TX_STARTED   PIN_SET(GLOSSY_TX_PIN)
#define GLOSSY_TX_STOPPED   PIN_CLR(GLOSSY_TX_PIN)
#else
#define GLOSSY_TX_STARTED
#define GLOSSY_TX_STOPPED
#endif

#ifdef GLOSSY_DEBUG_PIN
#define GLOSSY_DEBUG_ON     PIN_SET(GLOSSY_DEBUG_PIN)
#define GLOSSY_DEBUG_OFF    PIN_CLR(GLOSSY_DEBUG_PIN)
#define GLOSSY_DEBUG_TOGGLE PIN_XOR(GLOSSY_DEBUG_PIN)
#else
#define GLOSSY_DEBUG_ON
#define GLOSSY_DEBUG_OFF
#define GLOSSY_DEBUG_TOGGLE
#endif /* GLOSSY_DEBUG_PIN */

/* The busy process is required to prevent the MCU from entering LPM3, which
 * would effectively disable the DCO (required for Glossy to work). */
#define GLOSSY_USE_BUSY_PROCESS       1

#define GLOSSY_USE_CHANNEL_HOPPING    1

#define GLOSSY_NUM_CHANNELS           4   /* preferably a power of two */

/* Size of the window used to average estimations of slot lengths. */
#define GLOSSY_SYNC_WINDOW            64

/* Initiator timeout for packet retransmission, in number of Glossy slots. */
#define GLOSSY_INITIATOR_TIMEOUT      3

#define DELAY_CYCLES_FOR_CH_SWITCH    340

#if !GLOSSY_CONF_SETUPTIME_WITH_SYNC
#error "invalid value for GLOSSY_CONF_SETUPTIME_WITH_SYNC"
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */

#ifndef GLOSSY_CONF_GUARD_TIME
#define GLOSSY_CONF_GUARD_TIME        0
#endif /* GLOSSY_CONF_GUARD_TIME */

/*---------------------------------------------------------------------------*/
/* --------------------------- Macros and defines -------------------------- */
/*---------------------------------------------------------------------------*/

#define CM_POS                        CM_1
#define CM_NEG                        CM_2
#define CM_BOTH                       CM_3

#if COOJA
#define CLOCK_PHI                     (4194304uL / RTIMER_SECOND)
#else
#define CLOCK_PHI                     (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#define GLOSSY_SYNC_SETUP_TICKS       (uint16_t)(F_CPU / 1000 * GLOSSY_CONF_SETUPTIME_WITH_SYNC / 1000)
#define GLOSSY_GUARD_TIME             (uint16_t)(F_CPU / 1000 * GLOSSY_CONF_GUARD_TIME / 1000)

#define GLOSSY_HEADER_BYTE_MASK       0x0f    /* 4 bits */
#define GLOSSY_HEADER_BYTE_LEN        1
#define GLOSSY_HEADER_BIT_SYNC        0x80    /* sync bit */
#define GLOSSY_HEADER_BIT_CHHOP       0x40    /* 1 bit used to indicate whether or not a channel hop is required */
#define GLOSSY_RELAY_CNT_LEN          1
#define FOOTER_LEN                    2
#define FOOTER1_CRC_OK                0x80
#define FOOTER1_CORRELATION           0x7f

#define GLOSSY_MAX_PACKET_LEN         (GLOSSY_CONF_PAYLOAD_LEN + GLOSSY_HEADER_LEN)

#if GLOSSY_MAX_PACKET_LEN > 255
#error "invalid packet length for Glossy"
#endif /* GLOSSY_MAX_PACKET_LEN */

#define GLOSSY_LEN_FIELD              glossy_buffer[0]
#define GLOSSY_HEADER_FIELD           glossy_buffer[1]
#define GLOSSY_DATA_FIELD             glossy_buffer[2]
#define GLOSSY_RELAY_CNT_FIELD        glossy_buffer[packet_len_tmp - FOOTER_LEN]
#define GLOSSY_RSSI_FIELD             glossy_buffer[packet_len_tmp - 1]
#define GLOSSY_CRC_FIELD              glossy_buffer[packet_len_tmp]

/* Capture next low-frequency clock tick and DCO clock value at that instant. */
#define CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l)  {\
    /* Enable capture mode for timers B6 and A2 (ACLK) */\
    TBCCTL6 = CCIS0 | CM_POS | CAP | SCS; \
    /* backup registers */ \
    uint16_t taccr2 = TACCR2; \
    uint16_t tacctl2 = TACCTL2; \
    TACCTL2 = CCIS0 | CM_POS | CAP | SCS; \
    /* Wait until both timers capture the next clock tick */\
    while (!((TBCCTL6 & CCIFG) && (TACCTL2 & CCIFG))); \
    /* Store the capture timer values */\
    t_cap_h = TBCCR6; \
    t_cap_l = TACCR2; \
    /* Disable capture mode and restore register values */\
    TBCCTL6 = 0; \
    TACCTL2 = tacctl2; \
    TACCR2 = taccr2; \
}

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


#define RTIMER_NOW_DCO()      (TBR)


/*---------------------------------------------------------------------------*/
/*---------------------------- Enums and typedefs ---------------------------*/
/*---------------------------------------------------------------------------*/

/* List of possible Glossy glossy_states. */
enum glossy_glossy_state {
  GLOSSY_STATE_OFF,          /**< Glossy is not executing */
  GLOSSY_STATE_WAITING,      /**< Glossy is waiting for a packet being flooded */
  GLOSSY_STATE_RECEIVING,    /**< Glossy is receiving a packet */
  GLOSSY_STATE_RECEIVED,     /**< Glossy has just finished receiving a packet */
  GLOSSY_STATE_TRANSMITTING, /**< Glossy is transmitting a packet */
  GLOSSY_STATE_TRANSMITTED,  /**< Glossy has just finished transmitting a packet */
  GLOSSY_STATE_ABORTED       /**< Glossy has just aborted a packet reception */
};

typedef struct {
  uint16_t ie1;
  uint16_t ie2;
  uint16_t p1ie;
  uint16_t p2ie;
  uint16_t taie;
  uint16_t tbie;
  uint16_t tbctl;
} interrupt_state_t;


/*---------------------------------------------------------------------------*/
/*-------------------------------- Variables --------------------------------*/
/*---------------------------------------------------------------------------*/

static uint8_t glossy_buffer[GLOSSY_MAX_PACKET_LEN + 1];
static uint8_t *glossy_payload;
static uint8_t initiator,
               sync,
               glossy_payload_len,
               packet_len,
               packet_len_tmp,
               rx_cnt,
               rx_try_cnt,
               tx_cnt,
               tx_max,
               relay_cnt,
               tx_relay_cnt_last,
               bytes_read,
               n_timeouts,
               t_ref_l_updated,
               header_byte;

static volatile uint8_t glossy_state = GLOSSY_STATE_OFF;

static rtimer_clock_t t_start,
                      t_rx_start,
                      t_rx_stop,
                      t_tx_start,
                      t_tx_stop,
                      t_rx_timeout,
                      t_ref_l,
                      t_first_rx_l,
                      T_slot_h,
                      T_rx_h,
                      T_w_rt_h,
                      T_tx_h,
                      T_w_tr_h,
                      T_offset_h;

static rtimer_ext_clock_t t_ref_lf_ext;

static uint16_t flood_cnt,
                flood_cnt_success,
                rx_timeout,
                bad_length,
                bad_header,
                bad_crc;

static uint32_t total_rx_success_cnt;
static uint32_t total_rx_try_cnt;

#if GLOSSY_SYNC_WINDOW
static uint32_t T_slot_h_sum;
static uint8_t win_cnt;
#endif /* GLOSSY_SYNC_WINDOW */

volatile static uint16_t ch_idx;
volatile static uint8_t  switch_ch;
static uint16_t t_irq,
                t_irq_high_cnt;
static uint8_t  cfg_preferred_ch,
                cfg_payload_len;
#if GLOSSY_USE_BUSY_PROCESS
static uint8_t glossy_process_started = 0;
#endif /* GLOSSY_USE_BUSY_PROCESS */

/*---------------------------------------------------------------------------*/

/* allowed channels: 11 (2405) - 26 (2480) */
static const uint16_t rf_channels[GLOSSY_NUM_CHANNELS] = { 26, 11, 20, 16 };

/*---------------------------------------------------------------------------*/
/*---------------------------- Function prototypes --------------------------*/
/*---------------------------------------------------------------------------*/

inline void glossy_begin_rx(void);
inline void glossy_end_rx(void);
inline void glossy_begin_tx(void);
inline void glossy_end_tx(void);
inline void schedule_rx_timeout(void);
inline void stop_rx_timeout(void);
inline void schedule_initiator_timeout(void);
inline void stop_initiator_timeout(void);
#if GLOSSY_USE_CHANNEL_HOPPING
inline void schedule_channel_timeout(uint16_t extraticks);
inline void stop_channel_timeout(void);
#endif /* GLOSSY_USE_CHANNEL_HOPPING */

void enable_other_interrupts(uint8_t enable);
void glossy_init_hf_timer(void);

/*---------------------------------------------------------------------------*/
/*---------------------------- Glossy busy process --------------------------*/
/*---------------------------------------------------------------------------*/
#if GLOSSY_USE_BUSY_PROCESS
PROCESS(glossy_process, "Glossy Busy Task");
PROCESS_THREAD(glossy_process, ev, data)
{
  /* The purpose of this task is to prevent the MCU from entering LPM3.
   * The DCO must stay active while Glossy is active! */
  PROCESS_BEGIN();

#ifdef GLOSSY_START_PIN
  PIN_CFG_OUT(GLOSSY_START_PIN);
  PIN_CLR(GLOSSY_START_PIN);
#endif /* GLOSSY_START_PIN */
#ifdef GLOSSY_TX_PIN
  PIN_CFG_OUT(GLOSSY_TX_PIN);
  PIN_CLR(GLOSSY_TX_PIN);
#endif /* GLOSSY_TX_PIN */
#ifdef GLOSSY_RX_PIN
  PIN_CFG_OUT(GLOSSY_RX_PIN);
  PIN_CLR(GLOSSY_RX_PIN);
#endif /* GLOSSY_RX_PIN */
#ifdef GLOSSY_DEBUG_PIN
  PIN_CFG_OUT(GLOSSY_DEBUG_PIN);
  PIN_CLR(GLOSSY_DEBUG_PIN);
#endif /* GLOSSY_DEBUG_PIN */

  while(1) {
    GLOSSY_STOPPED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* busy wait until Glossy has stopped */
    while(glossy_state != GLOSSY_STATE_OFF) GLOSSY_STARTED;
  }

  PROCESS_END();

  return 0;
}
#endif /* GLOSSY_USE_BUSY_PROCESS */

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
  GLOSSY_RF_ON;
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
  GLOSSY_TX_STOPPED;
  GLOSSY_RX_STOPPED;
  GLOSSY_RF_OFF;
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
  GLOSSY_RX_STOPPED;
  glossy_state = GLOSSY_STATE_ABORTED;
  radio_flush_rx();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_abort_tx(void)
{
  FASTSPI_STROBE(CC2420_SRXON); // start listening (RX)
  GLOSSY_TX_STOPPED;
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
  GLOSSY_RF_ON;
  DCSTAT_RF_ON;
  GLOSSY_RX_STOPPED;
  GLOSSY_TX_STARTED;
  FASTSPI_STROBE(CC2420_STXON);

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
}
/*---------------------------------------------------------------------------*/
static inline void
radio_write_tx(void)
{
  FASTSPI_WRITE_FIFO(glossy_buffer, packet_len_tmp - 1);
}
/*---------------------------------------------------------------------------*/
static inline void
radio_set_channel(uint8_t idx)
{
  /* set radio channel, this will only work if radio is in off state */
  idx = idx % GLOSSY_NUM_CHANNELS;

  uint16_t chval = 357 + 5 * (rf_channels[idx] - 11) + 0x4000;
  FASTSPI_SETREG(CC2420_FSCTRL, chval);

  /* indicate current channel with pin */
#ifdef GLOSSY_CHANNEL_PIN
  idx++;
  while (idx) {
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    idx--;
  }
#endif /* GLOSSY_CHANNEL_PIN */
}
/*---------------------------------------------------------------------------*/
static inline void
radio_set_channel_off_on(uint8_t idx)
{
  /* sets the radio channel, turns radio off (idle) and back on into RX mode */
  idx = idx % GLOSSY_NUM_CHANNELS;

  SPI_ENABLE();
  FASTSPI_TX_NOP(CC2420_SRFOFF);
  FASTSPI_TX_NOP(CC2420_FSCTRL);
  uint16_t chval = 357 + 5 * (rf_channels[idx] - 11) + 0x4000;
  FASTSPI_TX_NOP((u8_t)(chval >> 8));
  FASTSPI_TX_NOP((u8_t)chval);
  FASTSPI_TX_NOP(CC2420_SRXON);
  SPI_WAITFORTx_ENDED();
  SPI_DISABLE();

  /* indicate current channel with pin */
#ifdef GLOSSY_CHANNEL_PIN
  idx++;
  while (idx) {
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    idx--;
  }
#endif /* GLOSSY_CHANNEL_PIN */
}
/*---------------------------------------------------------------------------*/
inline void
radio_set_channel_off(uint8_t idx)
{
  /* sets the radio channel, leaves the radio off */
  idx = idx % GLOSSY_NUM_CHANNELS;

  SPI_ENABLE();
  FASTSPI_TX_NOP(CC2420_SRFOFF);
  FASTSPI_TX_NOP(CC2420_FSCTRL);
  uint16_t chval = 357 + 5 * (rf_channels[idx] - 11) + 0x4000;
  FASTSPI_TX_NOP((uint8_t)(chval >> 8));
  FASTSPI_TX_NOP((uint8_t)chval);
  SPI_WAITFORTx_ENDED();
  SPI_DISABLE();

  /* indicate current channel with pin */
#ifdef GLOSSY_CHANNEL_PIN
  idx++;
  while (idx) {
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    PIN_XOR(GLOSSY_CHANNEL_PIN);
    idx--;
  }
#endif /* GLOSSY_CHANNEL_PIN */
}

/*---------------------------------------------------------------------------*/
/*------------------------------- Main interface ----------------------------*/
/*---------------------------------------------------------------------------*/
void
glossy_start(uint16_t initiator_id,
             uint8_t *payload,
             uint8_t payload_len,
             uint8_t n_tx_max,
             uint8_t with_sync,
             uint8_t dco_cal)
{
  /* NOTE: channel hopping only works properly if the packet length is known in advance!
   *       If the payload length is 0, then the node is bootstrapping and will not switch channels. */
  if(!payload || (initiator_id == node_id && payload_len == 0)) {
    return;
  }

  GLOSSY_STARTED;

  // disable all interrupts that may interfere with Glossy and ensure that TB is running and sourced by the DCO
  enable_other_interrupts(0);

  uint16_t setup_time_start = RTIMER_NOW_DCO();

#if GLOSSY_USE_BUSY_PROCESS
  // make sure the glossy process is running
  if(!glossy_process_started) {
    process_start(&glossy_process, NULL);
    glossy_process_started = 1;
  }
  process_poll(&glossy_process);
#endif /* GLOSSY_USE_BUSY_PROCESS */

  // copy function arguments
  glossy_payload     = payload;
  glossy_payload_len = payload_len;
  initiator          = (initiator_id == node_id);
  sync               = with_sync;
  tx_max             = n_tx_max;

  // initialize Glossy variables
  switch_ch       = 0;
  ch_idx          = 0;         // start with first radio channel
  tx_cnt          = 0;
  rx_cnt          = 0;
  rx_try_cnt      = 0;
  t_irq_high_cnt  = 0;
  t_ref_l_updated = 0;
  header_byte     = (((sync << 7) & GLOSSY_HEADER_BIT_SYNC) | (GLOSSY_CONF_HEADER_BYTE & GLOSSY_HEADER_BYTE_MASK));

#if GLOSSY_USE_CHANNEL_HOPPING
  if(!glossy_payload_len) {
    /* use the payload length that was manually set */
    glossy_payload_len = cfg_payload_len;
  }
#endif /* GLOSSY_USE_CHANNEL_HOPPING */

  // set Glossy packet length, with or without relay counter depending on the sync flag value
  if(glossy_payload_len) {
    packet_len_tmp      = ((sync) ? GLOSSY_RELAY_CNT_LEN : 0) + glossy_payload_len + FOOTER_LEN +  GLOSSY_HEADER_BYTE_LEN;
    packet_len          = packet_len_tmp;
    GLOSSY_LEN_FIELD    = packet_len_tmp;
    GLOSSY_HEADER_FIELD = header_byte;
  } else {
    // payload length is unknown: do not perform channel hopping
    packet_len = 0;
#if GLOSSY_USE_CHANNEL_HOPPING
    ch_idx = cfg_preferred_ch;
#endif /* GLOSSY_USE_CHANNEL_HOPPING*/
  }
  if(sync) {
    GLOSSY_RELAY_CNT_FIELD = 0;
  }

#if !COOJA
  // resynchronize the DCO
  if(dco_cal) {
    msp430_sync_dco();
  }
#endif /* COOJA */

  // set the radio channel and TX power
  radio_set_channel(ch_idx);              // or use: cc2420_set_channel(rf_channels[ch_idx])
  // clear radio buffers
  radio_flush_rx();
  radio_flush_tx();

  if(initiator) {
    // INITIATOR
    // copy the application data to the glossy_payload field
    memcpy(&GLOSSY_DATA_FIELD, glossy_payload, glossy_payload_len);
    // set Glossy glossy_state
    glossy_state = GLOSSY_STATE_RECEIVED;
    radio_write_tx();                   // write the packet to the TXFIFO
    // busy wait for the setup time to pass
    GLOSSY_START_TOGGLE;
    while((uint16_t)(RTIMER_NOW_DCO() - setup_time_start) < GLOSSY_SYNC_SETUP_TICKS);
    GLOSSY_START_TOGGLE;
    // start the first transmission
    t_start = RTIMER_NOW_DCO();
#if GLOSSY_USE_CHANNEL_HOPPING
    schedule_channel_timeout(0);
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    radio_start_tx();
    // schedule the initiator timeout
    if((!sync) || T_slot_h) {
      n_timeouts = 0;
      schedule_initiator_timeout();
    }
  } else {
    // RECEIVER
    glossy_state = GLOSSY_STATE_WAITING;
    // busy wait for the setup time to pass
    GLOSSY_START_TOGGLE;
    while((uint16_t)(RTIMER_NOW_DCO() - setup_time_start) < (GLOSSY_SYNC_SETUP_TICKS - GLOSSY_GUARD_TIME));
    GLOSSY_START_TOGGLE;
    // turn on the radio (RX mode)
    t_start = RTIMER_NOW_DCO();
#if GLOSSY_USE_CHANNEL_HOPPING
    schedule_channel_timeout(GLOSSY_GUARD_TIME);
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    radio_start_rx();
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_stop(void)
{
  if(glossy_state != GLOSSY_STATE_OFF) {
#if GLOSSY_USE_CHANNEL_HOPPING
    // stop channel timeout
    stop_channel_timeout();
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    // stop the initiator timeout, in case it is still active
    stop_initiator_timeout();
    // turn off the radio
    radio_off();
    // flush radio buffers
    radio_flush_rx();
    radio_flush_tx();
    // re-enable non Glossy-related interrupts
    enable_other_interrupts(1);

    // stats
    if(!initiator) {
      if(rx_try_cnt) {
        flood_cnt++;
      }
      if(rx_cnt) {
        flood_cnt_success++;
      } else {
        glossy_payload_len = 0;   /* nothing received */
      }
    }
    total_rx_success_cnt += rx_cnt;
    total_rx_try_cnt += rx_try_cnt;

    if(t_irq_high_cnt) {
      DEBUG_PRINT_WARNING("t_irq: %u, skipped slots: %u", t_irq, t_irq_high_cnt);
    }

    glossy_state = GLOSSY_STATE_OFF;
    GLOSSY_STOPPED;
  }

  // return the number of times the packet has been received
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
/*----------------------------- Get functions -------------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_rx_cnt(void)
{
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_rx_try_cnt(void)
{
  return rx_try_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_relay_cnt(void)
{
  return relay_cnt;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
glossy_get_T_slot_h(void)
{
  return T_slot_h;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_is_t_ref_updated(void)
{
  return t_ref_l_updated;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
glossy_get_t_first_rx_l(void)
{
  return t_first_rx_l;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
glossy_get_t_ref_l(void)
{
  return t_ref_l;
}
/*---------------------------------------------------------------------------*/
rtimer_ext_clock_t
glossy_get_t_ref(void)
{
  return t_ref_lf_ext;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_payload_len(void)
{
  return glossy_payload_len;
}
/*---------------------------------------------------------------------------*/
uint16_t
glossy_get_fsr(void)
{
  if(flood_cnt) {
    return (uint16_t) ((uint64_t) flood_cnt_success * 10000 / (uint64_t) flood_cnt);
  }
  return 10000;
}
/*---------------------------------------------------------------------------*/
uint16_t
glossy_get_per(void)
{
  if(total_rx_try_cnt) {
    return (uint16_t)((total_rx_try_cnt - total_rx_success_cnt) * 100 / total_rx_try_cnt);
  }
  return 10000;
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_get_glossy_state(void)
{
  return glossy_state;
}
/*---------------------------------------------------------------------------*/
void
glossy_set_channel_and_payload_len(uint8_t channel_index, uint8_t payload_length)
{
  /* NOTE: the set radio channel will only be used for bootstrapping, i.e.
   *       if the payload length is unknown (set to zero) */
  cfg_preferred_ch = channel_index % GLOSSY_NUM_CHANNELS;
  /* NOTE: payload_len will only be used if 0 is passed to glossy_start() */
  if(payload_length <= GLOSSY_CONF_PAYLOAD_LEN) {
    cfg_payload_len = payload_length;
  }
}
/*---------------------------------------------------------------------------*/
/*------------------------ Internal helper functions ------------------------*/
/*---------------------------------------------------------------------------*/
void
enable_other_interrupts(uint8_t enable)
{
  static interrupt_state_t int_state;

  int s = splhigh();
  if(enable) {
    /* RE-ENABLE interrupts */
    IE1 = int_state.ie1;
    IE2 = int_state.ie2;
    P1IE = int_state.p1ie;
    P2IE = int_state.p2ie;
    //TACTL |= int_state.taie & TAIE;
    TBCTL |= int_state.tbie & TBIE;
    TACCTL2 |= int_state.taie & CCIE;
    TBCCTL0 |= int_state.tbie & CCIE;
  #if COOJA
    if(TACCTL2 & CCIFG) {   /* etimer is on CCR2 */
      etimer_interrupt();
    }
  #endif
    DISABLE_SFD_INT();
    CLEAR_SFD_INT();
    CC2420_FIFOP_INT_INIT();
    CC2420_ENABLE_FIFOP_INT();
    if(int_state.tbctl != 0xffff) {
      /* restore the previous timer B config */
      TBCTL = 0;
      TBCTL = int_state.tbctl;
      int_state.tbctl = 0xffff;
    }
    watchdog_start();

  } else {
    /* DISABLE interrupts */
    /* note: user is responsible to disable other interrupts that might be
    *       active (such as rtimer CCI or DMA) */
    int_state.ie1 = IE1;
    int_state.ie2 = IE2;
    int_state.p1ie = P1IE;
    int_state.p2ie = P2IE;
    /* TAIFG is bit 0, TACCTLx CCIE is bit 4 */
    int_state.taie = (TACTL & TAIE) | (TACCTL2 & CCIE);
    int_state.tbie = (TBCTL & TBIE) | (TBCCTL0 & CCIE);
    int_state.tbctl = 0xffff;
    IE1 = 0;
    IE2 = 0;
    P1IE = 0;
    P2IE = 0;
    //TACTL &= ~TAIE;
    TBCTL &= ~TBIE;
    /* only disable the etimer and user defined rtimer-ext CCI */
    TACCTL2 &= ~CCIE;
    TBCCTL0 &= ~CCIE;
    CC2420_DISABLE_FIFOP_INT();
    CC2420_CLEAR_FIFOP_INT();
    SFD_CAP_INIT(CM_BOTH);
    ENABLE_SFD_INT();
    /* make sure TB is sourced by the DCO (SMCLK) and running */
    if((TBCTL & MC_3) != MC_2 || (TBCTL & TBSSEL_3) != TBSSEL_2) {
      /* stop, configure and restart the timer B */
      int_state.tbctl = TBCTL;
      TBCTL = 0;
      TBCTL = TBSSEL_2 | MC_2;
    }
    watchdog_stop();
  }
  splx(s);
}
/*---------------------------------------------------------------------------*/
static inline void
estimate_slot_length(rtimer_clock_t t_rx_stop_tmp)
{
  // estimate slot length if rx_cnt > 1
  // and we have received a packet immediately after our last transmission
  if((rx_cnt > 1) && (GLOSSY_RELAY_CNT_FIELD == (tx_relay_cnt_last + 2))) {
    T_w_rt_h = t_tx_start - t_rx_stop;
    T_tx_h = t_tx_stop - t_tx_start;
    T_w_tr_h = t_rx_start - t_tx_stop;
    T_rx_h = t_rx_stop_tmp - t_rx_start;
    rtimer_clock_t T_slot_h_tmp = (T_tx_h + T_w_tr_h + T_rx_h + T_w_rt_h) / 2 - (packet_len * F_CPU) / 31250;
#if GLOSSY_SYNC_WINDOW
    T_slot_h_sum += T_slot_h_tmp;
    if((++win_cnt) == GLOSSY_SYNC_WINDOW) {
      // update the slot length estimation
      T_slot_h = T_slot_h_sum / GLOSSY_SYNC_WINDOW;
      // halve the counters
      T_slot_h_sum /= 2;
      win_cnt /= 2;
    } else {
      if(win_cnt == 1) {
        // at the beginning, use the first estimation of the slot length
        T_slot_h = T_slot_h_tmp;
      }
    }
#else
    T_slot_h = T_slot_h_tmp;
#endif /* GLOSSY_SYNC_WINDOW */
  }
}
/*---------------------------------------------------------------------------*/
static inline void
compute_sync_reference_time(void)
{
#if COOJA
  rtimer_clock_t t_cap_l = RTIMER_NOW();
  rtimer_clock_t t_cap_h = RTIMER_NOW_DCO();
#else
  // capture the next low-frequency clock tick
  rtimer_clock_t t_cap_h, t_cap_l;
  CAPTURE_NEXT_CLOCK_TICK(t_cap_h, t_cap_l);
#endif /* COOJA */
  rtimer_clock_t T_rx_to_cap_h = t_cap_h - t_rx_start;
  unsigned long T_ref_to_rx_h = (GLOSSY_RELAY_CNT_FIELD - 1) * ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250);
  unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
  rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
  // high-resolution offset of the reference time
  T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
  // low-resolution value of the reference time
  t_ref_l = t_cap_l - T_ref_to_cap_l;
  t_ref_lf_ext = rtimer_ext_now_lf();
  t_ref_lf_ext -= ((uint16_t)(t_ref_lf_ext & 0xffff) - (uint16_t)t_ref_l);
  // the reference time has been updated
  t_ref_l_updated = 1;
}
/*---------------------------------------------------------------------------*/
/*--------------------------------- Timeouts --------------------------------*/
/*---------------------------------------------------------------------------*/
inline void
schedule_rx_timeout(void)
{
  TBCCR5  = t_rx_timeout;
  TBCCTL5 = CCIE;
}
/*---------------------------------------------------------------------------*/
inline void
stop_rx_timeout(void)
{
  TBCCTL5 = 0;
}
/*---------------------------------------------------------------------------*/
inline void
schedule_initiator_timeout(void)
{
#if !COOJA
  if(sync) {
    TBCCR4 = t_start + (n_timeouts + 1) * GLOSSY_INITIATOR_TIMEOUT * ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250);
  } else {
    TBCCR4 = t_start + (n_timeouts + 1) * GLOSSY_INITIATOR_TIMEOUT * (((rtimer_clock_t)packet_len * 33 + 380) * 4 + DELAY_CYCLES_FOR_CH_SWITCH);
  }
  TBCCTL4 = CCIE;
#endif
}
/*---------------------------------------------------------------------------*/
inline void
stop_initiator_timeout(void)
{
  TBCCTL4 = 0;
}
/*---------------------------------------------------------------------------*/
inline void
initiator_timeout(void)
{
  /* retransmission timeout on the initiator */  
  /* no packets received so far: send the packet again */
  tx_cnt = 0;
  // set the packet length field to the appropriate value
  GLOSSY_LEN_FIELD = packet_len_tmp;
  // set the header field
  GLOSSY_HEADER_FIELD = header_byte;
  if(sync) {
    GLOSSY_RELAY_CNT_FIELD = n_timeouts * GLOSSY_INITIATOR_TIMEOUT;
  }
  if((n_timeouts * GLOSSY_INITIATOR_TIMEOUT) & 1) {
    GLOSSY_HEADER_FIELD |= GLOSSY_HEADER_BIT_CHHOP;
  }
  // copy the application data to the glossy_payload field
  memcpy(&GLOSSY_DATA_FIELD, glossy_payload, glossy_payload_len);
  // set Glossy glossy_state
  glossy_state = GLOSSY_STATE_RECEIVED;
  // write the packet to the TXFIFO
  radio_write_tx();
  // start another transmission
  radio_start_tx();
  // schedule the timeout again
  schedule_initiator_timeout();
}
/*---------------------------------------------------------------------------*/
#if GLOSSY_USE_CHANNEL_HOPPING
inline void
schedule_channel_timeout(uint16_t extraticks)
{
  // NOTE: if the payload length is 0, then the node is bootstrapping: stay in the base channel!
  if(packet_len) {
    /* packet duration: 32us * packet_len (= payload length + glossy header byte + length byte + RSSI + CRC)
     * + takes ~380us from STOBE_STXON to the SFD high interrupt on receiver (subtract some slack time)
     * 1 DCO tick ~ 0.23 us */
    TBCCR3  = RTIMER_NOW_DCO() + ((uint16_t)packet_len * 32 + 350) * 4 * 2 + DELAY_CYCLES_FOR_CH_SWITCH + extraticks;
    TBCCTL3 = CCIE;
  }
}
/*---------------------------------------------------------------------------*/
inline void
stop_channel_timeout(void)
{
  TBCCTL3 = 0;
}
/*---------------------------------------------------------------------------*/
inline void
channel_timeout(void)
{
#ifdef GLOSSY_CHANNEL_PIN
  PIN_XOR(GLOSSY_CHANNEL_PIN);
#endif /* GLOSSY_CHANNEL_PIN */

  ch_idx++;

  /* switch now if no reception is ongoing */
  if(glossy_state == GLOSSY_STATE_WAITING) {
    /* put radio in idle state, change channel and put it back in RX mode */
    radio_set_channel_off_on(ch_idx);
    /* schedule the next timeout */
    schedule_channel_timeout(DELAY_CYCLES_FOR_CH_SWITCH + 320 + packet_len * 13);
  } else {
    /* do the channel switch at the end of RX or TX */
    switch_ch = 1;
  }

#ifdef GLOSSY_CHANNEL_PIN
  PIN_XOR(GLOSSY_CHANNEL_PIN);
#endif /* GLOSSY_CHANNEL_PIN */
}
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
/*---------------------------------------------------------------------------*/
/*------------------------- Radio interrupt functions -----------------------*/
/*---------------------------------------------------------------------------*/
inline void
glossy_begin_rx(void)
{
  GLOSSY_RX_STARTED;
  t_rx_start = TBCCR1;
  rx_try_cnt++;
  glossy_state = GLOSSY_STATE_RECEIVING;
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
  FASTSPI_READ_FIFO_BYTE(GLOSSY_LEN_FIELD);
  // keep receiving only if it has the right length
  if((packet_len && (GLOSSY_LEN_FIELD != packet_len_tmp)) ||
     (GLOSSY_LEN_FIELD < FOOTER_LEN) || (GLOSSY_LEN_FIELD > GLOSSY_MAX_PACKET_LEN)) {
    // packet with a wrong length: abort packet reception
    radio_restart_rx();
    bad_length++;
    return;
  }
  bytes_read = 1;
  if(!packet_len) {
    packet_len_tmp = GLOSSY_LEN_FIELD;
    t_rx_timeout = t_rx_start + ((rtimer_clock_t)packet_len_tmp * 35 + 200) * 4;
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
  FASTSPI_READ_FIFO_BYTE(GLOSSY_HEADER_FIELD);
  // keep receiving only if it has the right header (ignore the CHHOP bit) */
  if((GLOSSY_HEADER_FIELD & ~GLOSSY_HEADER_BIT_CHHOP) != header_byte) {
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
      FASTSPI_READ_FIFO_BYTE(glossy_buffer[bytes_read]);
      bytes_read++;
    }
  }
#endif /* COOJA */
  schedule_rx_timeout();
  
#if GLOSSY_USE_CHANNEL_HOPPING
  if(rx_cnt == 0) {
    cfg_preferred_ch = ch_idx;
  }
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
}
/*---------------------------------------------------------------------------*/
inline void
glossy_end_rx(void)
{
  GLOSSY_RX_STOPPED;
  rtimer_clock_t t_rx_stop_tmp = TBCCR1;
  // read the remaining bytes from the RXFIFO
  FASTSPI_READ_FIFO_NO_WAIT(&glossy_buffer[bytes_read], packet_len_tmp - bytes_read + 1);
  bytes_read = packet_len_tmp + 1;
  // stop the timeout
  stop_rx_timeout();
#if COOJA
  if((GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) && (GLOSSY_HEADER_FIELD == header_byte)) {
#else
  if(GLOSSY_CRC_FIELD & FOOTER1_CRC_OK) {
#endif /* COOJA */
    // packet correctly received
    if(sync) {
      // increment relay_cnt field
      GLOSSY_RELAY_CNT_FIELD++;
    }
    // toggle the CHHOP bit
    GLOSSY_HEADER_FIELD ^= GLOSSY_HEADER_BIT_CHHOP;
    // write Glossy packet to the TXFIFO
    radio_write_tx();
    glossy_state = GLOSSY_STATE_RECEIVED;
    if(rx_cnt == 0) {
      // first successful reception:
      // store current time and received relay counter
      t_first_rx_l = RTIMER_NOW();
      if(sync) {
        relay_cnt = GLOSSY_RELAY_CNT_FIELD - 1;
      }
    }
    rx_cnt++;
    if(sync) {
      estimate_slot_length(t_rx_stop_tmp);
    }
    t_rx_stop = t_rx_stop_tmp;
    if(initiator) {
      // a packet has been successfully received: stop the initiator timeout
      stop_initiator_timeout();
    }
    if(!packet_len) {
      packet_len = packet_len_tmp;
      glossy_payload_len = packet_len_tmp - FOOTER_LEN - GLOSSY_HEADER_BYTE_LEN - (sync ? GLOSSY_RELAY_CNT_LEN : 0);
#if GLOSSY_USE_CHANNEL_HOPPING
      // 1st RX: now we know the actual packet length
      if((relay_cnt & 1) == 0) {
        // we need to switch the frequency at the end of the current TX phase
        ch_idx++;
        switch_ch = 1;
      }
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    }
  } else {
    bad_crc++;
    // packet corrupted, abort the transmission before it actually starts
    radio_abort_tx();
    glossy_state = GLOSSY_STATE_WAITING;
  }
}
/*---------------------------------------------------------------------------*/
inline void
glossy_begin_tx(void)
{
  /* TX_STARTED has already been set at this point, therefore just toggle the
   * pin to indicate SFD interrupt */
  GLOSSY_TX_STOPPED;
  GLOSSY_TX_STARTED;
  t_tx_start = TBCCR1;
  glossy_state = GLOSSY_STATE_TRANSMITTING;
  tx_relay_cnt_last = GLOSSY_RELAY_CNT_FIELD;
  if((!initiator) && (rx_cnt == 1)) {
    // copy the application data from the glossy_payload field
    memcpy(glossy_payload, &GLOSSY_DATA_FIELD, glossy_payload_len);
  }
  if((sync) && (T_slot_h) && (!t_ref_l_updated) && (rx_cnt)) {
    // compute the reference time after the first reception (higher accuracy)
    compute_sync_reference_time();
  }
}
/*---------------------------------------------------------------------------*/
inline void
glossy_end_tx(void)
{
  GLOSSY_TX_STOPPED;
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  t_tx_stop = TBCCR1;
  // stop Glossy if tx_cnt reached tx_max (and tx_max > 1 at the initiator)
  if((++tx_cnt == tx_max) && ((tx_max - initiator) > 0)) {
    //radio_off();
    //glossy_state = GLOSSY_STATE_OFF;
    glossy_stop();
  } else {
    glossy_state = GLOSSY_STATE_WAITING;
    // automatically switches to RX mode
#if GLOSSY_USE_CHANNEL_HOPPING
    if(switch_ch) {
      // switch channel before entering RX mode
      radio_set_channel_off_on(ch_idx);
      schedule_channel_timeout(0);
      switch_ch = 0;
    }
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    radio_flush_tx();
  }
}
/*---------------------------------------------------------------------------*/
/* ------------------------------ SFD interrupt ---------------------------- */
/*---------------------------------------------------------------------------*/
#if GLOSSY_CONF_USE_TIMER_ISR
interrupt(TIMERB1_VECTOR)
timerb1_interrupt(void)
#else /* GLOSSY_CONF_USE_TIMER_ISR */
void
glossy_timer_int_cb(void)
#endif /* GLOSSY_CONF_USE_TIMER_ISR */
{
  uint16_t tbiv = TBIV;   /* read reset flag (clears highest pending bit) */
  GLOSSY_DEBUG_ON;

  if(glossy_state == GLOSSY_STATE_RECEIVING && !SFD_IS_1) {
    GLOSSY_RX_STOPPED;
    // packet reception has finished
#if GLOSSY_USE_CHANNEL_HOPPING
    stop_rx_timeout();
    if(!packet_len && (GLOSSY_HEADER_FIELD & GLOSSY_HEADER_BIT_CHHOP)) {
      switch_ch = 1;
      ch_idx++;
      /* temporarily set packet_len; workaround to schedule a channel timeout */
      packet_len = packet_len_tmp;
      schedule_channel_timeout(0);
      packet_len = 0;
    }
    if(switch_ch) {
      radio_set_channel_off(ch_idx);
      schedule_channel_timeout(0);
      switch_ch = 0;
    }
    // wait until PROCESSING_CYCLES cycles occurred since the last SFD event
    TBCCR4 = TBCCR1 + DELAY_CYCLES_FOR_CH_SWITCH;
    TBCCTL4 &= ~(CCIE | CCIFG);
  #if COOJA
    while (TBCCR4 - RTIMER_NOW_DCO() < 0x8000);
  #else
    while (!(TBCCTL4 & CCIFG));
  #endif

    t_irq = ((RTIMER_NOW_DCO() - TBCCR1) - (DELAY_CYCLES_FOR_CH_SWITCH)) * 2;
#else /* GLOSSY_USE_CHANNEL_HOPPING */
    t_irq = ((RTIMER_NOW_DCO() - TBCCR1) - 40) * 2;
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
    if (t_irq <= 34) {
      // NOPs (variable number) to compensate for the interrupt service and the busy waiting delay
      asm volatile("add %[d], r0" : : [d] "m" (t_irq));
      asm volatile("nop");            // irq_delay = 0
      asm volatile("nop");            // irq_delay = 2
      asm volatile("nop");            // irq_delay = 4
      asm volatile("nop");            // irq_delay = 6
      asm volatile("nop");            // irq_delay = 8
      asm volatile("nop");            // irq_delay = 10
      asm volatile("nop");            // irq_delay = 12
      asm volatile("nop");            // irq_delay = 14
      asm volatile("nop");            // irq_delay = 16
      asm volatile("nop");            // irq_delay = 18
      asm volatile("nop");            // irq_delay = 20
      asm volatile("nop");            // irq_delay = 22
      asm volatile("nop");            // irq_delay = 24
      asm volatile("nop");            // irq_delay = 26
      asm volatile("nop");            // irq_delay = 28
      asm volatile("nop");            // irq_delay = 30
      asm volatile("nop");            // irq_delay = 32
      asm volatile("nop");            // irq_delay = 34

      // relay the packet
      radio_start_tx();
      DCSTAT_CPU_ON;
      glossy_end_rx();

    } else {
      DCSTAT_CPU_ON;
      // interrupt service delay is too high: do not relay the packet
      radio_flush_rx();
      glossy_state = GLOSSY_STATE_WAITING;
      t_irq_high_cnt++;
    }

  } else {
    DCSTAT_CPU_ON;
#if GLOSSY_USE_CHANNEL_HOPPING
    if(tbiv == TBIV_TBCCR3) {
      channel_timeout();
    } else
#endif /* GLOSSY_USE_CHANNEL_HOPPING */
#if RTIMER_EXT_CONF_HF_ENABLE
    if(tbiv == TBIV_TBIFG) {
      rtimer_ext_notify_hf_timer_overflow();

    } else
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
    if(glossy_state == GLOSSY_STATE_WAITING && SFD_IS_1) {
      // packet reception has started
      glossy_begin_rx();
    } else {

      if(glossy_state == GLOSSY_STATE_RECEIVED && SFD_IS_1) {
        // packet transmission has started
        glossy_begin_tx();

      } else {

        if(glossy_state == GLOSSY_STATE_TRANSMITTING && !SFD_IS_1) {
          // packet transmission has finished
          glossy_end_tx();

        } else {
          if(glossy_state == GLOSSY_STATE_ABORTED) {
            // packet reception has been aborted
            glossy_state = GLOSSY_STATE_WAITING;

          } else {
            if((glossy_state == GLOSSY_STATE_WAITING) && (tbiv == TBIV_TBCCR4)) {
              // initiator timeout
              n_timeouts++;

              if(rx_cnt == 0) {
                initiator_timeout();

              } else {
                // at least one packet has been received: just stop the timeout
                stop_initiator_timeout();
              }

            } else {
              if(tbiv == TBIV_TBCCR5) {
                // rx timeout
                if(glossy_state == GLOSSY_STATE_RECEIVING) {
                  // we are still trying to receive a packet: abort the reception
                  radio_restart_rx();
                  rx_timeout++;
                }
                // stop the timeout
                stop_rx_timeout();

              } else {
                if(glossy_state != GLOSSY_STATE_OFF) {
                  // something strange is going on: go back to the waiting glossy_state
                  radio_flush_rx();
                  glossy_state = GLOSSY_STATE_WAITING;
                }
              }
            }
          }
        }
      }
    }
  }
  DCSTAT_CPU_OFF;
  GLOSSY_DEBUG_OFF;
}
/*---------------------------------------------------------------------------*/
