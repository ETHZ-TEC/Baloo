/*
 * Copyright (c) 2011, ETH Zurich.
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
 *          Jonas Baechli
 *          Reto Da Forno
 */

#include <string.h>
#include "glossy.h"
#include "cc2420_const.h"
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
#else
#define GLOSSY_STARTED
#define GLOSSY_STOPPED
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

/* Size of the window used to average estimations of slot lengths. */
#define GLOSSY_SYNC_WINDOW            64

/* Initiator timeout for packet retransmission, in number of Glossy slots. */
#define GLOSSY_INITIATOR_TIMEOUT      3

/* The busy process is required to prevent the MCU from entering LPM3, which
 * would effectively disable the DCO (required for Glossy to work). */
#define GLOSSY_USE_BUSY_PROCESS       1

/*---------------------------------------------------------------------------*/
/* --------------------------- Macros and defines -------------------------- */
/*---------------------------------------------------------------------------*/

#define CM_POS        CM_1
#define CM_NEG        CM_2
#define CM_BOTH       CM_3

#if COOJA
#define CLOCK_PHI                     (4194304uL / RTIMER_SECOND)
#else
#define CLOCK_PHI                     (F_CPU / RTIMER_SECOND)
#endif /* COOJA */

#if GLOSSY_CONF_SETUPTIME_WITH_SYNC
#define GLOSSY_SYNC_SETUP_TICKS       (uint16_t)(GLOSSY_CONF_SETUPTIME_WITH_SYNC * RTIMER_EXT_SECOND_LF / 1000000)
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */

#define GLOSSY_HEADER_BYTE_MASK       0x0f    /* 4 bits */
#define GLOSSY_HEADER_BYTE_LEN        1
#define GLOSSY_RELAY_CNT_LEN          1
#define GLOSSY_IS_ON()                (get_state() != GLOSSY_STATE_OFF)
#define FOOTER_LEN                    2
#define FOOTER1_CRC_OK                0x80
#define FOOTER1_CORRELATION           0x7f
#define GLOSSY_MAX_PACKET_LEN         (GLOSSY_CONF_PAYLOAD_LEN + GLOSSY_MAX_HEADER_LEN)

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


#define RTIMER_NOW_DCO()      rtimer_arch_now_dco()
#define rtimer_arch_now_dco() (TBR)


/*---------------------------------------------------------------------------*/
/*---------------------------- Enums and typedefs ---------------------------*/
/*---------------------------------------------------------------------------*/

/* List of possible Glossy states. */
enum glossy_state {
  GLOSSY_STATE_OFF,          /**< Glossy is not executing */
  GLOSSY_STATE_WAITING,      /**< Glossy is waiting for a packet being flooded */
  GLOSSY_STATE_RECEIVING,    /**< Glossy is receiving a packet */
  GLOSSY_STATE_RECEIVED,     /**< Glossy has just finished receiving a packet */
  GLOSSY_STATE_TRANSMITTING, /**< Glossy is transmitting a packet */
  GLOSSY_STATE_TRANSMITTED,  /**< Glossy has just finished transmitting a packet */
  GLOSSY_STATE_ABORTED       /**< Glossy has just aborted a packet reception */
};


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

static volatile uint8_t state = GLOSSY_STATE_OFF;

static rtimer_clock_t t_start,
                      t_rx_start,
                      t_rx_stop,
                      t_tx_start,
                      t_tx_stop,
                      t_rx_timeout,
                      t_ref_l,
                      t_first_rx_l,
                      //T_irq,
                      T_slot_h,
                      T_rx_h,
                      T_w_rt_h,
                      T_tx_h,
                      T_w_tr_h,
                      T_offset_h;

static rtimer_ext_clock_t t_ref_lf_ext;

static uint16_t ie1,
                ie2,
                p1ie,
                p2ie,
                tbiv,
                tbcctl0,
                flood_cnt,
                flood_cnt_success,
                //high_T_irq,
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

#if GLOSSY_USE_BUSY_PROCESS
static uint8_t glossy_process_started = 0;
#endif /* GLOSSY_USE_BUSY_PROCESS */

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

void enable_other_interrupts(void);
void disable_other_interrupts(void);
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
    //GLOSSY_STOPPED;
    PROCESS_YIELD_UNTIL(ev == PROCESS_EVENT_POLL);
    /* busy wait until Glossy has stopped */
    while(state != GLOSSY_STATE_OFF); // GLOSSY_STARTED;
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
  state = GLOSSY_STATE_ABORTED;
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
  FASTSPI_STROBE(CC2420_STXON);
  GLOSSY_RF_ON;
  DCSTAT_RF_ON;
  GLOSSY_RX_STOPPED;
  GLOSSY_TX_STARTED;

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
  GLOSSY_STARTED;

#if GLOSSY_CONF_SETUPTIME_WITH_SYNC
  uint16_t setup_time_start = rtimer_ext_now_lf_hw();
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */

  // copy function arguments
  glossy_payload = payload;
  glossy_payload_len = payload_len;
  initiator = (initiator_id == node_id);
  sync = with_sync;
  tx_max = n_tx_max;
  
  // disable all interrupts that may interfere with Glossy
  disable_other_interrupts();
  
#if GLOSSY_USE_BUSY_PROCESS
  // make sure the glossy process is running
  if(!glossy_process_started) {
    process_start(&glossy_process, NULL);
    glossy_process_started = 1;
  }
  process_poll(&glossy_process);
#endif /* GLOSSY_USE_BUSY_PROCESS */
  
  // initialize Glossy variables
  tx_cnt = 0;
  rx_cnt = 0;
  rx_try_cnt = 0;
  header_byte = (((sync << 7) & ~GLOSSY_HEADER_BYTE_MASK) | (GLOSSY_CONF_HEADER_BYTE & GLOSSY_HEADER_BYTE_MASK));

  // set Glossy packet length, with or without relay counter depending on the sync flag value
  if(glossy_payload_len) {
    packet_len_tmp = ((sync) ? GLOSSY_RELAY_CNT_LEN : 0) +
                     glossy_payload_len + FOOTER_LEN +  GLOSSY_HEADER_BYTE_LEN;
    packet_len = packet_len_tmp;
    // set the packet length field to the appropriate value
    GLOSSY_LEN_FIELD = packet_len_tmp;
    // set the header field
    GLOSSY_HEADER_FIELD = header_byte;
  } else {
    // packet length not known yet (only for receivers)
    packet_len = 0;
  }
  if(initiator) {
    // initiator: copy the application data to the glossy_payload field
    memcpy(&GLOSSY_DATA_FIELD, glossy_payload, glossy_payload_len);
    // set Glossy state
    state = GLOSSY_STATE_RECEIVED;
  } else {
    // receiver: set Glossy state
    state = GLOSSY_STATE_WAITING;
  }
  if(sync) {
    // set the relay_cnt field to 0
    GLOSSY_RELAY_CNT_FIELD = 0;
  }

  // the reference time has not been updated yet
  t_ref_l_updated = 0;

#if !COOJA
  // resynchronize the DCO
  if(dco_cal) {
    msp430_sync_dco();
  }
#endif /* COOJA */

  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();
  if(initiator) {
    // write the packet to the TXFIFO
    radio_write_tx();
#if GLOSSY_CONF_SETUPTIME_WITH_SYNC
    // busy wait for the setup time to pass
    if(sync) {
      //GLOSSY_STOPPED;
      while((uint16_t)(rtimer_ext_now_lf_hw() - setup_time_start) < GLOSSY_SYNC_SETUP_TICKS);
      //GLOSSY_STARTED;
    }
#endif /* GLOSSY_CONF_SETUPTIME_WITH_SYNC */
    t_start = RTIMER_NOW_DCO();
    // start the first transmission
    radio_start_tx();
    // schedule the initiator timeout
    if((!sync) || T_slot_h) {
      n_timeouts = 0;
      schedule_initiator_timeout();
    }
  } else {
    // turn on the radio (RX mode)
    radio_start_rx();
  }
}
/*---------------------------------------------------------------------------*/
uint8_t
glossy_stop(void)
{
  // stop the initiator timeout, in case it is still active
  stop_initiator_timeout();
  // turn off the radio
  radio_off();

  // flush radio buffers
  radio_flush_rx();
  radio_flush_tx();

  state = GLOSSY_STATE_OFF;
  // re-enable non Glossy-related interrupts
  enable_other_interrupts();

  // stats
  if(!initiator) {
    if(rx_try_cnt) {
      flood_cnt++;
    }
    if(rx_cnt) {
      flood_cnt_success++;
    }
  }
  total_rx_success_cnt += rx_cnt;
  total_rx_try_cnt += rx_try_cnt;
  
  GLOSSY_STOPPED;

  // return the number of times the packet has been received
  return rx_cnt;
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
  // NOTE: if you modify the code if this function
  // you may need to change the constant part of the interrupt delay (currently 21 DCO ticks),
  // due to possible different compiler optimizations

  // compute the variable part of the delay with which the interrupt has been served
  //T_irq = ((RTIMER_NOW_DCO() - TBCCR1) - (GLOSSY_CONF_USE_TIMER_ISR ? 21 : 32)) << 1;

  if(state == GLOSSY_STATE_RECEIVING && !SFD_IS_1) {
    // packet reception has finished
    // T_irq in [0,...,8]
    /*if(T_irq <= 8) {
      // NOPs (variable number) to compensate for the interrupt service delay (sec. 5.2)
      asm volatile("add %[d], r0" : : [d] "m" (T_irq));
      asm volatile("nop");            // irq_delay = 0
      asm volatile("nop");            // irq_delay = 2
      asm volatile("nop");            // irq_delay = 4
      asm volatile("nop");            // irq_delay = 6
      asm volatile("nop");            // irq_delay = 8
      // NOPs (fixed number) to compensate for HW variations (sec. 5.3)
      // (asynchronous MCU and radio clocks)
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");
      asm volatile("nop");*/
      // relay the packet
      radio_start_tx();
      tbiv = TBIV;        /* read TBIV to clear IFG */
      DCSTAT_CPU_ON;
      glossy_end_rx();
      
    /*} else {
      DCSTAT_CPU_ON;
      // interrupt service delay is too high: do not relay the packet
      radio_flush_rx();
      state = GLOSSY_STATE_WAITING;
      tbiv = TBIV;  // clear IFG
      high_T_irq++;
    }*/
    
  } else {
    DCSTAT_CPU_ON;
    // read TBIV to clear IFG
    tbiv = TBIV;
#if RTIMER_EXT_CONF_HF_ENABLE
    if(tbiv == TBIV_TBIFG) {
      rtimer_ext_notify_hf_timer_overflow();
      
    } else
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
    if(state == GLOSSY_STATE_WAITING && SFD_IS_1) {
      // packet reception has started
      glossy_begin_rx();
    } else {
      
      if(state == GLOSSY_STATE_RECEIVED && SFD_IS_1) {
        // packet transmission has started
        glossy_begin_tx();
        
      } else {
        
        if(state == GLOSSY_STATE_TRANSMITTING && !SFD_IS_1) {
          // packet transmission has finished
          glossy_end_tx();
          
        } else {
          if(state == GLOSSY_STATE_ABORTED) {
            // packet reception has been aborted
            state = GLOSSY_STATE_WAITING;
            
          } else {
            if((state == GLOSSY_STATE_WAITING) && (tbiv == TBIV_TBCCR4)) {
              // initiator timeout
              n_timeouts++;
              
              if(rx_cnt == 0) {
                // no packets received so far: send the packet again
                tx_cnt = 0;
                // set the packet length field to the appropriate value
                GLOSSY_LEN_FIELD = packet_len_tmp;
                // set the header field
                GLOSSY_HEADER_FIELD = header_byte;
                if(sync) {
                  GLOSSY_RELAY_CNT_FIELD = n_timeouts * GLOSSY_INITIATOR_TIMEOUT;
                }
                // copy the application data to the glossy_payload field
                memcpy(&GLOSSY_DATA_FIELD, glossy_payload, glossy_payload_len);
                // set Glossy state
                state = GLOSSY_STATE_RECEIVED;
                // write the packet to the TXFIFO
                radio_write_tx();
                // start another transmission
                radio_start_tx();
                // schedule the timeout again
                schedule_initiator_timeout();
                
              } else {
                // at least one packet has been received: just stop the timeout
                stop_initiator_timeout();
              }
              
            } else {
              if(tbiv == TBIV_TBCCR5) {
                // rx timeout
                if(state == GLOSSY_STATE_RECEIVING) {
                  // we are still trying to receive a packet: abort the reception
                  radio_restart_rx();
                  rx_timeout++;
                }
                // stop the timeout
                stop_rx_timeout();
                
              } else {
                if(state != GLOSSY_STATE_OFF) {
                  // something strange is going on: go back to the waiting state
                  radio_flush_rx();
                  state = GLOSSY_STATE_WAITING;
                }
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
/*------------------------ Internal helper functions ------------------------*/
/*---------------------------------------------------------------------------*/
void
disable_other_interrupts(void)
{
  int s = splhigh();
  ie1 = IE1;
  ie2 = IE2;
  p1ie = P1IE;
  p2ie = P2IE;
  tbcctl0 = TBCCTL0;
  IE1 = 0;
  IE2 = 0;
  P1IE = 0;
  P2IE = 0;
  //CACTL1 &= ~CAIE;
  //DMA0CTL &= ~DMAIE;
  //DMA1CTL &= ~DMAIE;
  //DMA2CTL &= ~DMAIE;
  // disable etimer interrupts
//  TACCTL1 &= ~CCIE;
//  TBCCTL0 &= ~CCIE;
  CC2420_DISABLE_FIFOP_INT();
  CC2420_CLEAR_FIFOP_INT();
  SFD_CAP_INIT(CM_BOTH);
  ENABLE_SFD_INT();
//  // stop Timer B
//  TBCTL = 0;
//  // Timer B sourced by the DCO
//  TBCTL = TBSSEL1;
//  // start Timer B
//  TBCTL |= MC1;
  splx(s);
  watchdog_stop();
}
/*---------------------------------------------------------------------------*/
void
enable_other_interrupts(void)
{
  int s = splhigh();
  IE1 = ie1;
  IE2 = ie2;
  P1IE = p1ie;
  P2IE = p2ie;
  // enable etimer interrupts
//  TACCTL1 |= CCIE;
  TBCCTL0 = tbcctl0;
#if COOJA
  if(TACCTL2 & CCIFG) {   /* etimer is on CCR2 */
    etimer_interrupt();
  }
#endif
  DISABLE_SFD_INT();
  CLEAR_SFD_INT();
  CC2420_FIFOP_INT_INIT();
  CC2420_ENABLE_FIFOP_INT();
//  // stop Timer B
//  TBCTL = 0;
//  // Timer B sourced by the 32 kHz
//  TBCTL = TBSSEL0;
//  // start Timer B
//  TBCTL |= MC1;
  splx(s);
  watchdog_start();
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
  //rtimer_ext_clock_t t_cap_ext_l, t_cap_ext_h;
  //rtimer_ext_now(&t_cap_ext_h, &t_cap_ext_l);
#endif /* COOJA */
  rtimer_clock_t T_rx_to_cap_h = t_cap_h - t_rx_start;
  unsigned long T_ref_to_rx_h = (GLOSSY_RELAY_CNT_FIELD - 1) * ((unsigned long)T_slot_h + (packet_len * F_CPU) / 31250);
  unsigned long T_ref_to_cap_h = T_ref_to_rx_h + (unsigned long)T_rx_to_cap_h;
  rtimer_clock_t T_ref_to_cap_l = 1 + T_ref_to_cap_h / CLOCK_PHI;
  // high-resolution offset of the reference time
  T_offset_h = (CLOCK_PHI - 1) - (T_ref_to_cap_h % CLOCK_PHI);
  // low-resolution value of the reference time
  t_ref_l = t_cap_l - T_ref_to_cap_l;
  //t_ref_lf_ext = t_cap_ext_l - T_ref_to_cap_l;
  t_ref_lf_ext = rtimer_ext_now_lf();
  t_ref_lf_ext -= ((uint16_t)(t_ref_lf_ext & 0xffff) - (uint16_t)t_ref_l);
  //t_ref_hf_ext = t_cap_ext_h - T_ref_to_cap_h;
  // the reference time has been updated
  t_ref_l_updated = 1;
}
/*---------------------------------------------------------------------------*/
/*------------------------- Radio interrupt functions -----------------------*/
/*---------------------------------------------------------------------------*/
inline void
glossy_begin_rx(void)
{
  GLOSSY_RX_STARTED;
  t_rx_start = TBCCR1;
  rx_try_cnt++;
  state = GLOSSY_STATE_RECEIVING;
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
  // keep receiving only if it has the right header
  if(GLOSSY_HEADER_FIELD != header_byte) {
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
    if(tx_cnt >= tx_max) {
      // no more Tx to perform: stop Glossy
      radio_off();
      state = GLOSSY_STATE_OFF;
    } else {
      // write Glossy packet to the TXFIFO
      radio_write_tx();
      state = GLOSSY_STATE_RECEIVED;
    }
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
      glossy_payload_len = packet_len_tmp -
        FOOTER_LEN - GLOSSY_HEADER_BYTE_LEN - (sync ? GLOSSY_RELAY_CNT_LEN : 0);
    }
  } else {
    bad_crc++;
    // packet corrupted, abort the transmission before it actually starts
    radio_abort_tx();
    state = GLOSSY_STATE_WAITING;
  }
}
/*---------------------------------------------------------------------------*/
inline void
glossy_begin_tx(void)
{
  GLOSSY_TX_STARTED;
  t_tx_start = TBCCR1;
  state = GLOSSY_STATE_TRANSMITTING;
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
  ENERGEST_OFF(ENERGEST_TYPE_TRANSMIT);
  ENERGEST_ON(ENERGEST_TYPE_LISTEN);
  GLOSSY_TX_STOPPED;
  t_tx_stop = TBCCR1;
  // stop Glossy if tx_cnt reached tx_max (and tx_max > 1 at the initiator)
  if((++tx_cnt == tx_max) && ((tx_max - initiator) > 0)) {
    radio_off();
    state = GLOSSY_STATE_OFF;
  } else {
    state = GLOSSY_STATE_WAITING;
    // automatically switches to RX mode
  }
  radio_flush_tx();
}
/*---------------------------------------------------------------------------*/
/*--------------------------------- Timeouts --------------------------------*/
/*---------------------------------------------------------------------------*/
inline void
schedule_rx_timeout(void)
{
  TBCCR5 = t_rx_timeout;
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
    TBCCR4 = t_start + (n_timeouts + 1) * GLOSSY_INITIATOR_TIMEOUT *
        ((rtimer_clock_t)packet_len * 35 + 400) * 4;
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
glossy_get_state(void)
{
  return state;
}
/*---------------------------------------------------------------------------*/
