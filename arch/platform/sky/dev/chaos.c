/*
 * Copyright (c) 2011, ETH Zurich.
 * Copyright (c) 2013, Olaf Landsiedel.
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
 *          Olaf Landsiedel
 *          Reto Da Forno
 */

#include <string.h>
#include "chaos.h"
#include "watchdog.h"
#include "cc2420_const.h"
#include "spi-glossy.h"
#include "gpio.h"
#include "energest.h"
#include "node-id.h"
#include "leds.h"

/*---------------------------------------------------------------------------*/
#include "sys/log.h"
#define LOG_MODULE "chaos"
#define LOG_LEVEL  LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* -------------------------------- defines -------------------------------- */
/*---------------------------------------------------------------------------*/
/* Size of the window used to average estimations of slot lengths. */
#define CHAOS_SYNC_WINDOW             16

#define CHAOS_BYTES_TIMEOUT           32

#define CHAOS_SYNC_MODE CHAOS_SYNC    /* sync enabled */
//#define CHAOS_SYNC_MODE CHAOS_NO_SYNC /* sync disabled */

// minimum slots with no rx before timeout expire, never put below two
#define MIN_SLOTS_TIMEOUT             3
// maximum slots with no rx before timeout expires
#define MAX_SLOTS_TIMEOUT             7

/* Number of clock (DCO) cycles reserved for flags and payload processing. */
#define PROCESSING_CYCLES             4000  // ~1ms

/**
 * If not zero, nodes print additional debug information (disabled by default).
 */
#define CHAOS_DEBUG                   1
#define LOG_FLAGS                     1
//#define LOG_TIRQ                    1
//#define LOG_ALL_FLAGS               1

#ifdef CHAOS_START_PIN
#define CHAOS_STARTED                 PIN_SET(CHAOS_START_PIN)
#define CHAOS_STOPPED                 PIN_CLR(CHAOS_START_PIN)
#else /* CHAOS_START_PIN */
#define CHAOS_STARTED
#define CHAOS_STOPPED
#endif /* CHAOS_START_PIN */

#ifdef CHAOS_RX_PIN
#define CHAOS_RX_STARTED              PIN_SET(CHAOS_RX_PIN)
#define CHAOS_RX_STOPPED              PIN_CLR(CHAOS_RX_PIN)
#else /* CHAOS_RX_PIN */
#define CHAOS_RX_STARTED
#define CHAOS_RX_STOPPED
#endif /* CHAOS_RX_PIN */

#ifdef CHAOS_TX_PIN
#define CHAOS_TX_STARTED              PIN_SET(CHAOS_TX_PIN)
#define CHAOS_TX_STOPPED              PIN_CLR(CHAOS_TX_PIN)
#else /* CHAOS_TX_PIN */
#define CHAOS_TX_STARTED
#define CHAOS_TX_STOPPED
#endif /* CHAOS_TX_PIN */

#ifdef CHAOS_FAILURE_PIN
#define CHAOS_FAILURE_ON              PIN_SET(CHAOS_FAILURE_PIN)
#define CHAOS_FAILURE_OFF             PIN_CLR(CHAOS_FAILURE_PIN)
#else /* CHAOS_FAILURE_PIN */
#define CHAOS_FAILURE_ON
#define CHAOS_FAILURE_OFF
#endif /* CHAOS_FAILURE_PIN */

#ifdef CHAOS_RX_SUCCESS_PIN
#define CHAOS_RX_SUCCESS_ON           PIN_SET(CHAOS_RX_SUCCESS_PIN)
#define CHAOS_RX_SUCCESS_OFF          PIN_CLR(CHAOS_RX_SUCCESS_PIN)
#else /* CHAOS_RX_SUCCESS_PIN */
#define CHAOS_RX_SUCCESS_ON
#define CHAOS_RX_SUCCESS_OFF
#endif /* CHAOS_RX_SUCCESS_PIN */

#ifdef CHAOS_RADIO_ON_PIN
#define CHAOS_RADIO_ON                PIN_SET(CHAOS_RADIO_ON_PIN)
#define CHAOS_RADIO_OFF               PIN_CLR(CHAOS_RADIO_ON_PIN)
#else /* CHAOS_RADIO_ON_PIN */
#define CHAOS_RADIO_ON
#define CHAOS_RADIO_OFF
#endif /* CHAOS_RADIO_ON_PIN */

/**
 * Ratio between the frequencies of the DCO and the low-frequency clocks
 */
#if COOJA
  #define CLOCK_PHI                   (4194304uL / RTIMER_SECOND)
#else
  #define CLOCK_PHI                   (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#define CHAOS_HEADER                  0xfe
#define CHAOS_HEADER_LEN              1       // 1 byte
#define CHAOS_RELAY_CNT_LEN           1
#define CHAOS_IS_ON()                 (chaos_get_state() != CHAOS_STATE_OFF)
#define CHAOS_FOOTER_LEN              2
#define CHAOS_FOOTER1_CRC_OK          0x80
#define CHAOS_FOOTER1_CORRELATION     0x7f

#define CHAOS_PACKET_LEN              (CHAOS_CONF_PAYLOAD_LEN + CHAOS_FLAGS_LEN)

#if CHAOS_SYNC_MODE == CHAOS_SYNC
#define PACKET_LEN (CHAOS_PACKET_LEN + CHAOS_FOOTER_LEN + CHAOS_RELAY_CNT_LEN + CHAOS_HEADER_LEN)
#else
#define PACKET_LEN (CHAOS_PACKET_LEN + CHAOS_FOOTER_LEN + CHAOS_HEADER_LEN)
#endif

/* compute length of the flags array. */
#define CHAOS_FLAGS_LEN               ((CHAOS_CONF_NUM_NODES + 7) / 8)

#define CHAOS_LEN_FIELD               packet[0]
#define CHAOS_HEADER_FIELD            packet[1]
#define CHAOS_DATA_FIELD              packet[2]
#define CHAOS_FLAGS_FIELD             packet[2]
#define CHAOS_PAYLOAD_FIELD           packet[2 + CHAOS_FLAGS_LEN]
#define CHAOS_BYTES_TIMEOUT_FIELD     packet[2+CHAOS_BYTES_TIMEOUT]
#define CHAOS_RELAY_CNT_FIELD         packet[PACKET_LEN - CHAOS_FOOTER_LEN]
#define CHAOS_RSSI_FIELD              packet[PACKET_LEN - 1]
#define CHAOS_CRC_FIELD               packet[PACKET_LEN]

/* compute how the last flag byte looks when we are complete (all other flag bytes are 0xFF) */
#define CHAOS_COMPLETE_FLAG           ((1 << (((CHAOS_CONF_NUM_NODES - 1) % 8) + 1)) - 1)

/**
 * \brief Capture next low-frequency clock tick and DCO clock value at that
 *        instant.
 * \param t_cap_h variable for storing value of DCO clock value
 * \param t_cap_l variable for storing value of low-frequency clock value
 */
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

#define SFD_CAP_INIT(edge)    {\
  P4SEL |= BV(SFD);\
  TBCCTL1 = edge | CAP | SCS;\
}

/* Enable generation of interrupts due to SFD events */
#define ENABLE_SFD_INT()      (TBCCTL1 |= CCIE)

/* Disable generation of interrupts due to SFD events */
#define DISABLE_SFD_INT()     (TBCCTL1 &= ~CCIE)

/* Clear interrupt flag due to SFD events */
#define CLEAR_SFD_INT()       (TBCCTL1 &= ~CCIFG)

#define CM_POS                CM_1
#define CM_NEG                CM_2
#define CM_BOTH               CM_3

#define RTIMER_NOW_DCO()      rtimer_arch_now_dco()
#define rtimer_arch_now_dco() (TBR)

/*---------------------------------------------------------------------------*/
/* --------------------------- enums and typedefs -------------------------- */
/*---------------------------------------------------------------------------*/
enum {
  CHAOS_SYNC = 1, CHAOS_NO_SYNC = 0
};

enum chaos_state {
  CHAOS_STATE_OFF,          /**< Chaos is not executing */
  CHAOS_STATE_WAITING,      /**< Chaos is waiting for a packet being flooded */
  CHAOS_STATE_RECEIVING,    /**< Chaos is receiving a packet */
  CHAOS_STATE_RECEIVED,     /**< Chaos has just finished receiving a packet */
  CHAOS_STATE_TRANSMITTING, /**< Chaos is transmitting a packet */
  CHAOS_STATE_TRANSMITTED,  /**< Chaos has just finished transmitting a packet */
  CHAOS_STATE_ABORTED       /**< Chaos has just aborted a packet reception */
};

/* for internal use only, adds flags as a header to the data packet */
typedef struct {
  uint8_t  flags[CHAOS_FLAGS_LEN];
  uint8_t  payload[CHAOS_CONF_PAYLOAD_LEN];
} chaos_data_t;

/*---------------------------------------------------------------------------*/
/* ------------------------------- variables  ------------------------------ */
/*---------------------------------------------------------------------------*/

static uint8_t packet[PACKET_LEN + 1];
static uint8_t flags[CHAOS_FLAGS_LEN];
static uint8_t my_payload[CHAOS_PAYLOAD_PER_NODE];
static uint8_t* chaos_payload;

static uint8_t  initiator,
                rx_cnt,
                tx_cnt,
                tx_max,
                bytes_read,
                tx_relay_cnt_last,
                tx,
                chaos_complete,
                tx_cnt_complete,
                estimate_length,
                n_slots_timeout,
                relay_cnt_timeout,
                relay_cnt,
                t_ref_l_updated,
                data_processing_cnt;

static volatile uint8_t state;

static rtimer_clock_t t_rx_start,
                      t_rx_stop,
                      t_tx_start,
                      t_tx_stop,
                      t_rx_timeout,
                      t_timeout_start,
                      t_timeout_stop,
                      tbccr1,
                      T_slot_h,
                      T_rx_h,
                      T_w_rt_h,
                      T_tx_h,
                      T_w_tr_h,
                      t_ref_l,
                      T_offset_h,
                      t_first_rx_l;

static rtimer_ext_clock_t t_ref_lf_ext;

static unsigned short ie1,
                      ie2,
                      p1ie,
                      p2ie,
                      tbiv;

static uint32_t T_timeout_h;
static uint16_t n_timeout_wait;

static uint16_t node_index = 0xffff;

#if CHAOS_SYNC_WINDOW
static unsigned long T_slot_h_sum;
static uint8_t win_cnt;
#endif /* CHAOS_SYNC_WINDOW */

#ifdef LOG_TIRQ
#define CHAOS_TIRQ_LOG_SIZE 70
static uint8_t tirq_log_cnt;
static rtimer_clock_t tirq_log[CHAOS_TIRQ_LOG_SIZE];
#endif //LOG_TIRQ

#ifdef LOG_FLAGS
#define CHAOS_FLAGS_LOG_SIZE 70
static uint16_t flags_tx_cnt;
static uint8_t flags_tx[CHAOS_FLAGS_LOG_SIZE*CHAOS_FLAGS_LEN];
static uint8_t relay_counts_tx[CHAOS_FLAGS_LOG_SIZE];
static uint16_t flags_rx_cnt;
static uint8_t flags_rx[CHAOS_FLAGS_LOG_SIZE*CHAOS_FLAGS_LEN];
static uint8_t relay_counts_rx[CHAOS_FLAGS_LOG_SIZE];
#ifdef LOG_ALL_FLAGS
static uint8_t current_flags_rx[CHAOS_FLAGS_LEN];
#endif /* LOG_ALL_FLAGS */
#endif /* LOG_FLAGS */

#if CHAOS_DEBUG
unsigned int high_T_irq,
             rx_timeout,
             bad_length,
             bad_header,
             bad_crc,
             rc_update;
#endif /* CHAOS_DEBUG */

/*---------------------------------------------------------------------------*/
/* -------------------------------- timeouts ------------------------------- */
/*---------------------------------------------------------------------------*/
static inline void
chaos_schedule_timeout(void)
{
#if CHAOS_CONF_TIMEOUT_ON
  if(T_slot_h) {
    // random number between MIN_SLOTS_TIMEOUT and MAX_SLOTS_TIMEOUT
    n_slots_timeout = MIN_SLOTS_TIMEOUT + ((RTIMER_NOW() + RTIMER_NOW_DCO())
                                % (MAX_SLOTS_TIMEOUT - MIN_SLOTS_TIMEOUT + 1));
    T_timeout_h = n_slots_timeout * (uint32_t)T_slot_h;
    t_timeout_stop = t_timeout_start + T_timeout_h;
    if(T_timeout_h >> 16) {
      rtimer_clock_t now = RTIMER_NOW_DCO();
      n_timeout_wait = (T_timeout_h - (now - t_timeout_start)) >> 16;
          // it should never happen, but to be sure...
      if(n_timeout_wait == 0xffff) {
        n_timeout_wait = 0;
      }
    } else {
      n_timeout_wait = 0;
    }
    TBCCR4 = t_timeout_stop;
    TBCCTL4 = CCIE;
    CHAOS_FAILURE_ON;
  }
#endif /* CHAOS_CONF_TIMEOUT_ON */
}
/*---------------------------------------------------------------------------*/
static inline void
chaos_stop_timeout(void)
{
  TBCCTL4 = 0;
  n_timeout_wait = 0;
  CHAOS_FAILURE_OFF;
}
/*---------------------------------------------------------------------------*/
/* ----------------------------- radio functions --------------------------- */
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
radio_on(void)
{
  FASTSPI_STROBE(CC2420_SRXON);
  while(!(radio_status() & (BV(CC2420_XOSC16M_STABLE))));
  CHAOS_RADIO_ON;
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
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
  CHAOS_RADIO_OFF;
  CHAOS_RX_STOPPED;
  CHAOS_TX_STOPPED;
  FASTSPI_STROBE(CC2420_SRFOFF);
  chaos_stop_timeout();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_flush_rx(void)
{
  uint8_t dummy;
  FASTSPI_READ_FIFO_BYTE(dummy);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  FASTSPI_STROBE(CC2420_SFLUSHRX);
  (void)dummy;
}
/*---------------------------------------------------------------------------*/
static inline void
radio_abort_rx(void)
{
  state = CHAOS_STATE_ABORTED;
  CHAOS_RX_STOPPED;
  radio_flush_rx();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_abort_tx(void)
{
  CHAOS_TX_STOPPED;
  FASTSPI_STROBE(CC2420_SRXON);
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
  t_timeout_start = RTIMER_NOW_DCO();
  FASTSPI_STROBE(CC2420_STXON);
  CHAOS_RADIO_ON;
  CHAOS_RX_STOPPED;
#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_LISTEN);
  ENERGEST_ON(ENERGEST_TYPE_TRANSMIT);
#endif /* ENERGEST_CONF_ON */
  chaos_schedule_timeout();
}
/*---------------------------------------------------------------------------*/
static inline void
radio_write_tx(void)
{
  FASTSPI_WRITE_FIFO(packet, PACKET_LEN - 1);
}
/*---------------------------------------------------------------------------*/
/* ----------------------------- helper functions -------------------------- */
/*---------------------------------------------------------------------------*/
void
chaos_data_processing(void)
{
  uint8_t* received_flags = &CHAOS_FLAGS_FIELD;
  uint8_t  complete_temp  = 0xFF;
  uint8_t  is_my_flag_set = received_flags[node_index / 8] 
                            & (1 << (node_index % 8));
  uint16_t i;
  
  for(i = 0; i < CHAOS_FLAGS_LEN-1; i++) {
#if defined LOG_FLAGS && defined LOG_ALL_FLAGS
    current_flags_rx[i] = received_flags[i];
#endif /* LOG_FLAGS */
    tx |= (received_flags[i] != flags[i]);
    received_flags[i] |= flags[i];
    complete_temp &= received_flags[i];
  }
#if defined LOG_FLAGS && defined LOG_ALL_FLAGS
  current_flags_rx[CHAOS_FLAGS_LEN-1] = received_flags[CHAOS_FLAGS_LEN-1];
#endif /* LOG_FLAGS */
  tx |= (received_flags[CHAOS_FLAGS_LEN-1] != flags[CHAOS_FLAGS_LEN-1]);
  received_flags[CHAOS_FLAGS_LEN-1] |= flags[CHAOS_FLAGS_LEN-1];
  chaos_complete = (complete_temp == 0xFF) &&
                   (received_flags[CHAOS_FLAGS_LEN-1] == CHAOS_COMPLETE_FLAG);
                   
  if(!is_my_flag_set) {
    // Add your own payload information to the packet
    chaos_set_payload_cb(&CHAOS_PAYLOAD_FIELD, node_index, my_payload);
    // log data processing
    data_processing_cnt++;
  }
}
/*---------------------------------------------------------------------------*/
static inline void
chaos_disable_other_interrupts(void)
{
  int s = splhigh();
  ie1 = IE1;
  ie2 = IE2;
  p1ie = P1IE;
  p2ie = P2IE;
  IE1 = 0;
  IE2 = 0;
  P1IE = 0;
  P2IE = 0;
  CACTL1 &= ~CAIE;
  DMA0CTL &= ~DMAIE;
  DMA1CTL &= ~DMAIE;
  DMA2CTL &= ~DMAIE;
  // disable etimer interrupts
  //TACCTL1 &= ~CCIE;
  //TBCCTL0 = 0;
  CC2420_DISABLE_FIFOP_INT();
  CC2420_CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();
  // stop Timer B
  //TBCTL = 0;
  // Timer B sourced by the DCO
  //TBCTL = TBSSEL1;
  // start Timer B
  //TBCTL |= MC1;
  //TBCTL &= ~TBIE;
  splx(s);
  watchdog_stop();
}
/*---------------------------------------------------------------------------*/
static inline void
chaos_enable_other_interrupts(void)
{
  int s = splhigh();
  IE1 = ie1;
  IE2 = ie2;
  P1IE = p1ie;
  P2IE = p2ie;
  // enable etimer interrupts
  //TACCTL1 |= CCIE;
#if COOJA
  if(TACCTL1 & CCIFG) {
    etimer_interrupt();
  }
#endif
  DISABLE_SFD_INT();
  CLEAR_SFD_INT();
  CC2420_FIFOP_INT_INIT();
  CC2420_ENABLE_FIFOP_INT();
  // stop Timer B
  //TBCTL = 0;
  // Timer B sourced by the 32 kHz
  //TBCTL = TBSSEL0;
  // start Timer B
  //TBCTL |= MC1;
  splx(s);
  watchdog_start();
}
/*---------------------------------------------------------------------------*/
static inline void
chaos_set_flags(void)
{
  // node index not assigned yet?
  if(node_index == 0xffff) {
    const uint16_t node_id_mapping[CHAOS_CONF_NUM_NODES] = \
                                                    CHAOS_CONF_NODE_ID_MAPPING;
    for(node_index = 0; node_index < CHAOS_CONF_NUM_NODES; node_index++) {
      if(node_id == node_id_mapping[node_index]) {
        break;
      }
    }
    if(node_index == CHAOS_CONF_NUM_NODES) {
      LOG_ERR("invalid node ID mapping!\n");
    }
  }
  if(node_index < CHAOS_CONF_NUM_NODES) {
    // set all flags to zero and the one for this node to one
    memset(flags, 0, CHAOS_FLAGS_LEN);
    flags[node_index / 8] = 1 << (node_index % 8);
  }
  return;
}
/*---------------------------------------------------------------------------*/
/* ------------------------------ main interface --------------------------- */
/*---------------------------------------------------------------------------*/
void
chaos_start(uint8_t *payload,
            uint8_t payload_len,
            uint8_t is_initiator,
            uint8_t n_tx_max,
            uint8_t dco_cal)
{
  chaos_payload  = payload;
  initiator      = is_initiator;
  tx_max         = n_tx_max;

  if(payload_len && payload) {
    memcpy(my_payload, payload, MIN(payload_len, CHAOS_PAYLOAD_PER_NODE));
  }

  // set the 'flags' field in the data struct
  chaos_set_flags();

  // disable all interrupts that may interfere with Chaos
  chaos_disable_other_interrupts();

  // initialize Chaos variables
  tx_cnt = 0;
  rx_cnt = 0;
  chaos_complete  = 0;
  tx_cnt_complete = 0;
  estimate_length = 1;
  data_processing_cnt = 0;
#if CHAOS_DEBUG
  rc_update = 0;
#endif /* CHAOS_DEBUG */

  // set the packet length field to the appropriate value
  CHAOS_LEN_FIELD = PACKET_LEN;
  // set the header field
  CHAOS_HEADER_FIELD = CHAOS_HEADER;
  if(CHAOS_SYNC_MODE) {
    // set the relay_cnt field to 0
    CHAOS_RELAY_CNT_FIELD = 0;
    // the reference time has not been updated yet
    t_ref_l_updated = 0;
  }

#if !COOJA
  if(dco_cal) {
    // resynchronize the DCO
    msp430_sync_dco();
  }
#endif /* COOJA */

  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();

  if(initiator) {
    // initiator: copy the application data to the data field
    memcpy(&CHAOS_FLAGS_FIELD, flags, CHAOS_FLAGS_LEN);
    if(CHAOS_CONF_PAYLOAD_LEN && chaos_payload) {
      // clean the chaos payload field
      memset(&CHAOS_PAYLOAD_FIELD, 0, CHAOS_CONF_PAYLOAD_LEN);
      // add the initiator payload
      chaos_set_payload_cb(&CHAOS_PAYLOAD_FIELD, node_index, my_payload);
      // log
      data_processing_cnt++;
    }
    state = CHAOS_STATE_RECEIVED;
    // write the packet to the TXFIFO
    radio_write_tx();
    // start the first transmission
    radio_start_tx();

  } else {
    state = CHAOS_STATE_WAITING;
    // turn on the radio
    radio_on();
  }
  CHAOS_STARTED;
}
/*---------------------------------------------------------------------------*/
uint8_t
chaos_stop(void)
{
  // turn off the radio
  radio_off();
  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  CHAOS_STOPPED;

  state = CHAOS_STATE_OFF;
  // re-enable non Chaos-related interrupts
  chaos_enable_other_interrupts();
  // return the number of times the packet has been received
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
static inline void
estimate_slot_length(rtimer_clock_t t_rx_stop_tmp)
{
  // estimate slot length if rx_cnt > 1
  // and we have received a packet immediately after our last transmission
  if((rx_cnt > 1) && (CHAOS_RELAY_CNT_FIELD == (tx_relay_cnt_last + 2))) {
    T_w_rt_h = t_tx_start - t_rx_stop;
    T_tx_h = t_tx_stop - t_tx_start;
    T_w_tr_h = t_rx_start - t_tx_stop;
    T_rx_h = t_rx_stop_tmp - t_rx_start;
    uint32_t T_slot_h_tmp = ((uint32_t)T_tx_h +
                             (uint32_t)T_w_tr_h +
                             (uint32_t)T_rx_h +
                             (uint32_t)T_w_rt_h) / 2;
#if CHAOS_SYNC_WINDOW
    T_slot_h_sum += T_slot_h_tmp;
    if((++win_cnt) == CHAOS_SYNC_WINDOW) {
      // update the slot length estimation
      T_slot_h = T_slot_h_sum / CHAOS_SYNC_WINDOW;
      // halve the counters
      T_slot_h_sum /= 2;
      win_cnt /= 2;
    } else {
      if(win_cnt == 1) {
        // at the beginning, use the first estimation of the slot length
        T_slot_h = T_slot_h_tmp;
      }
    }
    estimate_length = 0;
#if CHAOS_DEBUG
    rc_update = CHAOS_RELAY_CNT_FIELD;
#endif /* CHAOS_DEBUG */
#else
    T_slot_h = T_slot_h_tmp;
#endif /* CHAOS_SYNC_WINDOW */
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
  unsigned long T_ref_to_rx_h = (CHAOS_RELAY_CNT_FIELD - 1) * (unsigned long)T_slot_h;
  unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
  rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
  // high-resolution offset of the reference time
  T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
  // low-resolution value of the reference time
  t_ref_l = t_cap_l - T_ref_to_cap_l;
  relay_cnt = CHAOS_RELAY_CNT_FIELD - 1;
  t_ref_lf_ext = rtimer_ext_now_lf();
  t_ref_lf_ext -= ((uint16_t)(t_ref_lf_ext & 0xffff) - (uint16_t)t_ref_l);
  // the reference time has been updated
  t_ref_l_updated = 1;
}
/*---------------------------------------------------------------------------*/
/*------------------------- Radio interrupt functions -----------------------*/
/*---------------------------------------------------------------------------*/
inline void
chaos_begin_rx(void)
{
  CHAOS_RX_STARTED;
  t_rx_start = TBCCR1;
  state = CHAOS_STATE_RECEIVING;
  // Rx timeout: packet duration + 200 us
  // (packet duration: 32 us * packet_length, 1 DCO tick ~ 0.23 us)
  t_rx_timeout = t_rx_start + ((rtimer_clock_t)PACKET_LEN * 35 + 200) * 4;
  tx = 0;

  // wait until the FIFO pin is 1 (i.e., until the first byte is received)
  while(!FIFO_IS_1) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_abort_rx();
#if CHAOS_DEBUG
      rx_timeout++;
#endif /* CHAOS_DEBUG */
      leds_toggle(LEDS_RED);
      return;
    }
  };
#if COOJA
  //OL: do not ask why
  int i;
  for(i = 0; i < 40; i++) {
    asm volatile("nop");
  }
#endif /* COOJA */
  // read the first byte (i.e., the len field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(CHAOS_LEN_FIELD);
  // keep receiving only if it has the right length
  if(CHAOS_LEN_FIELD != PACKET_LEN) {
    // packet with a wrong length: abort packet reception
    radio_abort_rx();
#if CHAOS_DEBUG
    bad_length++;
#endif /* CHAOS_DEBUG */
    leds_toggle(LEDS_RED);
    return;
  }
  bytes_read = 1;

#if CHAOS_CONF_FINAL_FLOOD_ON
  //Chaos mode on completion
  if(chaos_complete && (tx_cnt_complete < CHAOS_CONF_N_TX_COMPLETE)) {
    tx = 1;
  }
#endif /* CHAOS_CONF_FINAL_FLOOD_ON */

  // wait until the FIFO pin is 1 (i.e., until the second byte is received)
  while(!FIFO_IS_1) {
    if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
      radio_abort_rx();
#if CHAOS_DEBUG
      rx_timeout++;
#endif /* CHAOS_DEBUG */
      leds_toggle(LEDS_RED);
      return;
    }
  };
  // read the second byte (i.e., the header field) from the RXFIFO
  FASTSPI_READ_FIFO_BYTE(CHAOS_HEADER_FIELD);
  // keep receiving only if it has the right header
  if(CHAOS_HEADER_FIELD < CHAOS_HEADER) {
    // packet with a wrong header: abort packet reception
    radio_abort_rx();
#if CHAOS_DEBUG
    bad_header++;
#endif /* CHAOS_DEBUG */
    leds_toggle(LEDS_RED);
    return;
  }
  bytes_read = 2;
  if(PACKET_LEN > 8) {
    // if packet is longer than 8 bytes, read all bytes but the last 8
    while(bytes_read <= PACKET_LEN - 8) {
      // wait until the FIFO pin is 1 (until one more byte is received)
      while(!FIFO_IS_1) {
        if(!RTIMER_CLOCK_LT(RTIMER_NOW_DCO(), t_rx_timeout)) {
          radio_abort_rx();
#if CHAOS_DEBUG
          rx_timeout++;
#endif /* CHAOS_DEBUG */
          leds_toggle(LEDS_RED);
          return;
        }
      };
      // read another byte from the RXFIFO
      FASTSPI_READ_FIFO_BYTE(packet[bytes_read]);
      bytes_read++;
    }
  }
}
/*---------------------------------------------------------------------------*/
inline void
chaos_end_rx(void)
{
  CHAOS_RX_STOPPED;
  if(tx) {
    // packet correctly received and tx required
    if(CHAOS_SYNC_MODE) {
      // increment relay_cnt field
      CHAOS_RELAY_CNT_FIELD++;
    }
    if(tx_cnt == tx_max) {
      // no more Tx to perform: stop Chaos
      radio_off();
      state = CHAOS_STATE_OFF;
    } else {
      // write Chaos packet to the TXFIFO
      if(chaos_complete) {
        CHAOS_HEADER_FIELD = CHAOS_HEADER + 1;
      }
      radio_flush_rx();
      radio_write_tx();
      state = CHAOS_STATE_RECEIVED;
    }
    if(rx_cnt == 0) {
      // first successful reception: store current time
      t_first_rx_l = RTIMER_NOW();
    }

#ifdef LOG_TIRQ
    if(tirq_log_cnt < CHAOS_TIRQ_LOG_SIZE) {
      tirq_log[tirq_log_cnt] = T_irq;
      tirq_log_cnt++;
    }
#endif

    rx_cnt++;
    if(CHAOS_SYNC_MODE &&
      estimate_length &&
      rx_cnt > 1) { //CHAOS_HEADER_FIELD == CHAOS_HEADER) {
      estimate_slot_length(tbccr1);
    }
    t_rx_stop = tbccr1;
    /* copy the received data into the local buffer */
    memcpy(flags, &CHAOS_FLAGS_FIELD, CHAOS_FLAGS_LEN);
    memcpy(chaos_payload, &CHAOS_PAYLOAD_FIELD, CHAOS_CONF_PAYLOAD_LEN);
#if CHAOS_CONF_FINAL_FLOOD_ON
    if(chaos_complete) {
      tx_cnt_complete++;
    }
#endif /* CHAOS_CONF_FINAL_FLOOD_ON */
#ifdef LOG_FLAGS
    uint8_t i;
  #ifdef LOG_ALL_FLAGS
    if(flags_rx_cnt < CHAOS_FLAGS_LOG_SIZE*CHAOS_FLAGS_LEN - CHAOS_FLAGS_LEN) {
      relay_counts_rx[flags_rx_cnt / CHAOS_FLAGS_LEN] = CHAOS_RELAY_CNT_FIELD;
      for(i = 0; i < CHAOS_FLAGS_LEN; i++) {
        flags_rx[flags_rx_cnt] = current_flags_rx[i];
        flags_rx_cnt++;
      }
    }
  #else /* LOG_ALL_FLAGS */
    // store only the first complete reception
    if(chaos_complete && (flags_rx_cnt == 0)) {
      relay_counts_rx[flags_rx_cnt / CHAOS_FLAGS_LEN] = CHAOS_RELAY_CNT_FIELD;
      for(i = 0; i < CHAOS_FLAGS_LEN-1; i++) {
        flags_rx[flags_rx_cnt] = 0xff;
        flags_rx_cnt++;
      }
      flags_rx[flags_rx_cnt] = CHAOS_COMPLETE_FLAG;
      flags_rx_cnt++;
    }
  #endif /* LOG_ALL_FLAGS */
#endif /* LOG_FLAGS */
  } else {
    radio_flush_rx();
    state = CHAOS_STATE_WAITING;
  }
}
/*---------------------------------------------------------------------------*/
inline void
chaos_begin_tx(void)
{
  CHAOS_TX_STARTED;
  t_tx_start = TBCCR1;
  state = CHAOS_STATE_TRANSMITTING;
  tx_relay_cnt_last = CHAOS_RELAY_CNT_FIELD;
  // relay counter to be used in case the timeout expires
  relay_cnt_timeout = CHAOS_RELAY_CNT_FIELD + n_slots_timeout;

  if((CHAOS_SYNC_MODE) && (T_slot_h) && (!t_ref_l_updated) && (rx_cnt)) {
    // compute the reference time after the first reception (higher accuracy)
    compute_sync_reference_time();
  }
#ifdef LOG_FLAGS
#ifdef LOG_ALL_FLAGS
  uint8_t i;
  if(flags_tx_cnt < CHAOS_FLAGS_LOG_SIZE*CHAOS_FLAGS_LEN - CHAOS_FLAGS_LEN) {
    relay_counts_tx[flags_tx_cnt / CHAOS_FLAGS_LEN] = CHAOS_RELAY_CNT_FIELD;
    for(i = 0; i < CHAOS_FLAGS_LEN; i++) {
      flags_tx[flags_tx_cnt] = flags[i];
      flags_tx_cnt++;
    }
  }
#else
  // store only the last transmission
  uint8_t i;
  flags_tx_cnt = 0;
  relay_counts_tx[flags_tx_cnt / CHAOS_FLAGS_LEN] = CHAOS_RELAY_CNT_FIELD;
  for(i = 0; i < CHAOS_FLAGS_LEN; i++) {
    flags_tx[flags_tx_cnt] = flags[i];
    flags_tx_cnt++;
  }
#endif /* LOG_ALL_FLAGS */
#endif /* LOG_FLAGS */
}
/*---------------------------------------------------------------------------*/
inline void
chaos_end_tx(void)
{
  CHAOS_TX_STOPPED;
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  t_tx_stop = TBCCR1;
  // stop Chaos if tx_cnt reached tx_max (and tx_max > 1 at the initiator, if sync is enabled)
  if((++tx_cnt == tx_max) && ((!CHAOS_SYNC_MODE) || ((tx_max - initiator) > 0))) {
    radio_off();
    state = CHAOS_STATE_OFF;
#if CHAOS_CONF_FINAL_FLOOD_ON
  } else if(chaos_complete && (tx_cnt_complete >= CHAOS_CONF_N_TX_COMPLETE)) {
    radio_off();
    state = CHAOS_STATE_OFF;
#endif /* CHAOS_CONF_FINAL_FLOOD_ON */
  } else {
    state = CHAOS_STATE_WAITING;
  }
  radio_flush_tx();
}
/*---------------------------------------------------------------------------*/
/* ------------------------------ SFD interrupt ---------------------------- */
/*---------------------------------------------------------------------------*/
#if CHAOS_CONF_USE_TIMER_ISR
interrupt(TIMERB1_VECTOR) // __attribute__ ((section(".chaos")))
timerb1_interrupt(void)
#else /* CHAOS_CONF_USE_TIMER_ISR */
void
chaos_timer_int_cb(void)
#endif /* CHAOS_CONF_USE_TIMER_ISR */
{
  if(state == CHAOS_STATE_RECEIVING && !SFD_IS_1) {
    // packet reception has finished

    // store the time at which the SFD was captured into an ad hoc variable
    // (the value of TBCCR1 might change in case other SFD interrupts occur)
    uint16_t tbccr1 = TBCCR1;

    CHAOS_RX_STOPPED;

    // read the remaining bytes from the RXFIFO
    FASTSPI_READ_FIFO_NO_WAIT(&packet[bytes_read], PACKET_LEN - bytes_read +1);
    bytes_read = PACKET_LEN + 1;

    if(CHAOS_CRC_FIELD & CHAOS_FOOTER1_CRC_OK) {
      // CRC ok: packet successfully received
      CHAOS_RX_SUCCESS_ON;
      // stop the timeout
      chaos_stop_timeout();
      // data processing
      chaos_data_processing();

      CHAOS_RX_SUCCESS_OFF;

      //ok, data processing etc is done and we are ready to transmit a packet
      //now the black magic part starts:
      //we ensure synchronous transmissions on all transmitting Chaos nodes
      //by making them transmit the data packet at exactly the same number of
      //processor cycles after the receive
      //this consists of two steps:
      //1. we wait until a defined time is reached (wait loop)
      //2. we execute a NOP loop to ensure precise synchronization at CPU
      //   instruction cycles.
      //(step two is the same as in Glossy)

      // wait loop
      // wait until PROCESSING_CYCLES cycles occurred since the last SFD event
      TBCCR4   = tbccr1 + PROCESSING_CYCLES;
      TBCCTL4  = CCIE;
      while(!(TBCCTL4 & CCIFG));
      TBCCTL4  = 0;

      // the next instruction is executed at least PROCESSING_CYCLES+12 cycles
      // since the last SFD event
      // ->achieve basic clock synchronization for synchronous TX

      // prepare for NOP loop
      // compute interrupt etc. delay to do get instruction level
      // synchronization for TX
//      rtimer_clock_t T_irq = ((RTIMER_NOW_DCO() - tbccr1) - (PROCESSING_CYCLES+15)) << 1;

      // NOP loop: slip stream!!
      // if delay is within reasonable range: execute NOP loop do ensure 
      // synchronous TX
      // T_irq in [0,...,34]
//      if(T_irq <= 34) {
        if(tx) {
          // NOPs (variable number) to compensate for the interrupt service
          // and the busy waiting delay
/*          asm volatile("add %[d], r0" : : [d] "m" (T_irq));
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
*/          // relay the packet
          //
          // -> all transmitting nodes have instruction level synchronization
          // with
          radio_start_tx();
        }
      //} else {
        // interrupt service delay is too high: do not relay the packet
        // FF: this should never happen!
      //  leds_toggle(LEDS_RED);
#if CHAOS_DEBUG
        if(tx) {
          high_T_irq++;
        }
#endif
      //  tx = 0;
      //}
      // read TBIV to clear IFG
      tbiv = TBIV;
      chaos_end_rx();

    } else {
      // CRC not ok
#if CHAOS_DEBUG
      bad_crc++;
#endif /* CHAOS_DEBUG */
      tx = 0;
      // read TBIV to clear IFG
      tbiv = TBIV;
      chaos_end_rx();
    }

  } else {
    // read TBIV to clear IFG
    tbiv = TBIV;
#if RTIMER_EXT_CONF_HF_ENABLE
    if(tbiv == TBIV_TBIFG) {
      rtimer_ext_notify_hf_timer_overflow();

    } else
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
    if(state == CHAOS_STATE_WAITING && SFD_IS_1) {
      // packet reception has started
      chaos_begin_rx();

    } else {
      if(state == CHAOS_STATE_RECEIVED && SFD_IS_1) {
        // packet transmission has started
        chaos_begin_tx();

      } else {
        if(state == CHAOS_STATE_TRANSMITTING && !SFD_IS_1) {
          // packet transmission has finished
          chaos_end_tx();
        } else {

          if(state == CHAOS_STATE_ABORTED) {
            // packet reception has been aborted
            state = CHAOS_STATE_WAITING;

          } else {
            if(tbiv == TBIV_TBCCR4) {
              // timeout
              if(n_timeout_wait > 0) {
                n_timeout_wait--;

              } else {
                if(state == CHAOS_STATE_WAITING) {
                  // start another transmission
                  radio_start_tx();
                  CHAOS_FAILURE_OFF;
                  if(initiator && rx_cnt == 0) {
                    CHAOS_LEN_FIELD = PACKET_LEN;
                    CHAOS_HEADER_FIELD = CHAOS_HEADER;
                  } else {
                    // stop estimating the slot length during this round (to 
                    // keep maximum precision)
                    estimate_length = 0;
                    CHAOS_LEN_FIELD = PACKET_LEN;
                    CHAOS_HEADER_FIELD = CHAOS_HEADER+1;
                  }
                  CHAOS_RELAY_CNT_FIELD = relay_cnt_timeout;
                  if(CHAOS_PACKET_LEN > CHAOS_BYTES_TIMEOUT) {
                    // first CHAOS_BYTES_TIMEOUT bytes
                    memcpy(&CHAOS_DATA_FIELD, flags, MIN(CHAOS_BYTES_TIMEOUT, CHAOS_FLAGS_LEN));
                    if(CHAOS_BYTES_TIMEOUT > CHAOS_FLAGS_LEN) {
                      memcpy(&CHAOS_DATA_FIELD + CHAOS_FLAGS_LEN, chaos_payload, CHAOS_BYTES_TIMEOUT - CHAOS_FLAGS_LEN);
                    }
                    radio_flush_rx();
                    FASTSPI_WRITE_FIFO(packet, CHAOS_BYTES_TIMEOUT + 1 + CHAOS_HEADER_LEN);
                    // remaining bytes
                    memcpy(&CHAOS_BYTES_TIMEOUT_FIELD, chaos_payload + CHAOS_BYTES_TIMEOUT, CHAOS_PACKET_LEN - CHAOS_BYTES_TIMEOUT);
                    FASTSPI_WRITE_FIFO(&packet[CHAOS_BYTES_TIMEOUT + 1 + CHAOS_HEADER_LEN], PACKET_LEN - CHAOS_BYTES_TIMEOUT - 1 - CHAOS_HEADER_LEN - 1);
                  } else {
                    memcpy(&CHAOS_DATA_FIELD, flags, CHAOS_FLAGS_LEN);
                    memcpy(&CHAOS_DATA_FIELD + CHAOS_FLAGS_LEN, chaos_payload, CHAOS_CONF_PAYLOAD_LEN);
                    // write the packet to the TXFIFO
                    radio_flush_rx();
                    radio_write_tx();
                  }
                  state = CHAOS_STATE_RECEIVED;
                } else {
                  // stop the timeout
                  chaos_stop_timeout();
                }
              }

            } else {
              if(state != CHAOS_STATE_OFF) {
                // something strange is going on: go back to the waiting state
                radio_flush_rx();
                // stop the timeout
                chaos_stop_timeout();
                state = CHAOS_STATE_WAITING;
              }
            }
          }
        }
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
/*-------------------------------- Debug print ------------------------------*/
/*---------------------------------------------------------------------------*/
#ifdef LOG_TIRQ
inline void
print_tirq(void)
{
  LOG_INFO("win_cnt %u, T_slot_tmp %u\n", win_cnt, (uint16_t)(T_slot_h_sum / win_cnt));
  LOG_INFO("tirq %02x:", tirq_log_cnt);
  uint8_t i;
  for(i = 0; i < tirq_log_cnt; i++) {
    LOG_INFO("%04x,",tirq_log[i]);
  }
  LOG_INFO("\n");
  tirq_log_cnt = 0;
  memset(&tirq_log, 0, sizeof(tirq_log));
}
#endif
/*---------------------------------------------------------------------------*/
#ifdef LOG_FLAGS
inline void
print_flags_tx(void)
{
  LOG_INFO("flags_tx %2u:", flags_tx_cnt / CHAOS_FLAGS_LEN);
  uint8_t i;
  int8_t j;
  for(i = 0; i < flags_tx_cnt / CHAOS_FLAGS_LEN; i++) {
    LOG_INFO("0x%02x-0x%02x-0x", i, relay_counts_tx[i]);
    for(j = CHAOS_FLAGS_LEN-1; j >= 0; j--) {
      LOG_INFO("%02x", flags_tx[i*CHAOS_FLAGS_LEN + j]);
    }
    LOG_INFO(",");
  }
  LOG_INFO("\n");
  flags_tx_cnt = 0;
  memset(&flags_tx, 0, sizeof(flags_tx));
  memset(&relay_counts_tx, 0, sizeof(relay_counts_tx));
}
/*---------------------------------------------------------------------------*/
inline void
print_flags_rx(void)
{
  LOG_INFO("flags_rx %2u:", flags_rx_cnt / CHAOS_FLAGS_LEN);
  uint8_t i;
  int8_t j;
  for(i = 0; i < flags_rx_cnt / CHAOS_FLAGS_LEN; i++) {
    LOG_INFO("0x%02x-0x%02x-0x", i, relay_counts_rx[i]);
    for(j = CHAOS_FLAGS_LEN-1; j >= 0; j--) {
      LOG_INFO("%02x", flags_rx[i*CHAOS_FLAGS_LEN + j]);
    }
    LOG_INFO(",");
  }
  LOG_INFO("\n");
  flags_rx_cnt = 0;
  memset(&flags_rx, 0, sizeof(flags_rx));
  memset(&relay_counts_rx, 0, sizeof(relay_counts_rx));
}
#endif /* LOG_FLAGS */
/*---------------------------------------------------------------------------*/
/*--------------------------- get/set functions -----------------------------*/
/*---------------------------------------------------------------------------*/
uint8_t
chaos_get_rx_cnt(void)
{
  return rx_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
chaos_get_processing_cnt(void)
{
  return data_processing_cnt;
}
/*---------------------------------------------------------------------------*/
uint8_t
chaos_get_relay_cnt(void)
{
  return relay_cnt;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
chaos_get_T_slot_h(void)
{
  return T_slot_h;
}
/*---------------------------------------------------------------------------*/
uint8_t
chaos_is_t_ref_l_updated(void)
{
  return t_ref_l_updated;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
chaos_get_t_first_rx_l(void)
{
  return t_first_rx_l;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
chaos_get_t_ref_l(void)
{
  return t_ref_l;
}
/*---------------------------------------------------------------------------*/
rtimer_ext_clock_t
chaos_get_t_ref_lf_ext(void)
{
  return t_ref_lf_ext;
}
/*---------------------------------------------------------------------------*/
void
chaos_set_t_ref_l(rtimer_clock_t t)
{
  t_ref_l = t;
}
/*---------------------------------------------------------------------------*/
void
chaos_set_t_ref_l_updated(uint8_t updated)
{
  t_ref_l_updated = updated;
}
/*---------------------------------------------------------------------------*/
uint8_t
chaos_get_state(void)
{
  return state;
}
/*---------------------------------------------------------------------------*/
#if !CHAOS_CONF_SET_CUSTOM_PAYLOAD
__attribute__((always_inline))
inline void chaos_set_payload_cb(uint8_t* chaos_payload_field,
                                 uint16_t node_index,
                                 uint8_t* payload)
{
  if(payload && (node_index < CHAOS_CONF_NUM_NODES)) {
    memcpy(chaos_payload_field + (node_index * CHAOS_PAYLOAD_PER_NODE),
           payload, CHAOS_PAYLOAD_PER_NODE);
  }
}
#endif /* CHAOS_CONF_SET_CUSTOM_PAYLOAD */
/*---------------------------------------------------------------------------*/
