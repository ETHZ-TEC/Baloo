/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Jonas Baechli
 *          Reto Da Forno
 *          Federico Ferrari
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup rtimer_ext
 * @{
 */
/*---------------------------------------------------------------------------*/
/**
 * \file
 *
 * Implements the Contiki rtimer and etimer and provides extended timer
 * functionality.
 * Replaces the files rtimer-arch.c and clock.c.
 *
 * TA0 is a high-frequency (HF) timer with 5 CCRs (SMCLK_SPEED)
 * TA1 is a low-frequency (LF) timer with 3 CCRs (ACLK_SPEED)
 *
 * TA0 runs at 3.25 MHz
 * TA1 runs at 32768 Hz
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "rtimer-ext.h"
#include "sys/energest.h"
#include "sys/etimer.h"
#include "gpio.h"
#include "dc-stat.h"

/* Log configuration */
#include "sys/log.h"
#define LOG_MODULE    "rtimer-ext"
#define LOG_LEVEL LOG_LEVEL_MAIN
/*---------------------------------------------------------------------------*/
static rtimer_ext_t rt[NUM_OF_RTIMER_EXTS]; /* rtimer_ext structs */
volatile rtimer_ext_clock_t ta0_sw_ext; /* SW extension for timer A0 */
volatile rtimer_ext_clock_t ta1_sw_ext;
static volatile clock_time_t clock_count = 0;
static volatile clock_time_t clock_secs = 0;
#define MAX_TICKS         (~((clock_time_t)0) / 2)
#define CLOCK_INTERVAL    (RTIMER_EXT_SECOND_LF / CLOCK_SECOND)
/*---------------------------------------------------------------------------*/
/* implementation of Contiki ETIMER                                          */
/*---------------------------------------------------------------------------*/
void
clock_init()
{
  /* note: rtimer_ext_init() is called in stage one platform init, before this
   * function is called by contiki-main */

  clock_count = 0;

#if RTIMER_EXT_CONF_USE_ETIMER
  #if ETIMER_ARCH_TIMER_ID != RTIMER_EXT_LF_2
  #error "wrong rtimer_ext ID for etimer!"
  #endif
  TA1CCR2   = CLOCK_INTERVAL;
  TA1CCTL2 |= CCIE;
#endif /* RTIMER_EXT_CONF_USE_ETIMER */
}
/*---------------------------------------------------------------------------*/
CCIF unsigned long
clock_seconds(void)
{
  return clock_secs;
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  return clock_count;
}
/*---------------------------------------------------------------------------*/
uint16_t
clock_counter(void)
{
  uint16_t t1, t2;
  do {
    t1 = TA1R;
    t2 = TA1R;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
clock_delay(unsigned int i)
{
  while(i--) {
    _NOP();
  }
}

/*---------------------------------------------------------------------------*/
/* implementation of Contiki RTIMER                                          */
/*---------------------------------------------------------------------------*/
void
rtimer_arch_init(void)
{
  TA1CCTL1 = 0;
}
/*---------------------------------------------------------------------------*/
static char
rtimer_callback(struct rtimer_ext *rt)
{
  rtimer_run_next();
  return 0;
}
/*---------------------------------------------------------------------------*/
void
rtimer_arch_schedule(rtimer_clock_t t)
{
  rtimer_ext_clock_t now = rtimer_ext_now_lf();
  uint16_t diff = t - (now & 0xffff);
  rtimer_ext_schedule(RTIMER_ARCH_TIMER_ID,
                      now + diff,
                      0,
                      rtimer_callback);
}

/*---------------------------------------------------------------------------*/
/* EXTENDED RTIMER functionality                                             */
/*---------------------------------------------------------------------------*/
static inline void
update_rtimer_ext_state(uint16_t timer)
{
  /* update the state only if the rtimer_ext has not been manually */
  /* stopped or re-scheduled by the callback function */
  if(rt[timer].state == RTIMER_EXT_JUST_EXPIRED) {
    if(rt[timer].period > 0) {
      /* if it is periodic, schedule the new expiration */
      rt[timer].time += rt[timer].period;
      if(timer >= RTIMER_EXT_LF_0) {
        *(&TA1CCR0 + (timer - RTIMER_EXT_LF_0)) +=
          (uint16_t) (rt[timer].period);
      } else {
        *(&TA0CCR0 + timer) += (uint16_t) (rt[timer].period);
      }
      rt[timer].state = RTIMER_EXT_SCHEDULED;
    } else {
      if(timer >= RTIMER_EXT_LF_0) {
        /* otherwise, just stop it */
        *(&TA1CCTL0 + (timer - RTIMER_EXT_LF_0)) = 0;
      } else {
        *(&TA0CCTL0 + timer) = 0;
      }
      rt[timer].state = RTIMER_EXT_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
#if !RTIMER_EXT_CONF_LF_UPDATE_INT
#if WATCHDOG_CONF_ON && WATCHDOG_CONF_RESET_ON_TA1IFG
#define RTIMER_EXT_LF_HANDLE_OVF() \
    if(TA1CTL & TAIFG) { \
      ta1_sw_ext++; \
      TA1CTL &= ~TAIFG; \
      watchdog_reset(); \
    }
#else /* WATCHDOG */
#define RTIMER_EXT_LF_HANDLE_OVF() \
    if(TA1CTL & TAIFG) { \
      ta1_sw_ext++; \
      TA1CTL &= ~TAIFG; \
    }
#endif /* WATCHDOG */
#else  /* RTIMER_EXT_CONF_LF_UPDATE_INT */
#define RTIMER_EXT_LF_HANDLE_OVF()
#endif /* RTIMER_EXT_CONF_LF_UPDATE_INT */

#define RTIMER_EXT_HF_CALLBACK(timer) \
  if((rtimer_ext_now_hf() >= rt[timer].time) && \
     (rt[timer].state == RTIMER_EXT_SCHEDULED)) { \
    /* the timer has expired! */ \
    rt[timer].state = RTIMER_EXT_JUST_EXPIRED; \
    /* execute the proper callback function */ \
    rt[timer].func(&rt[timer]); \
    /* update or stop the timer */ \
    update_rtimer_ext_state(timer); \
    if(process_nevents() > 0) { \
      __bic_status_register_on_exit(LPM4_bits); /* LPM4_EXIT; */ \
    } \
  } else if(rt[timer].state == RTIMER_EXT_WFE) { \
    rt[timer].func(&rt[timer]); \
  }

#define RTIMER_EXT_LF_CALLBACK(timer) \
  RTIMER_EXT_LF_HANDLE_OVF(); \
  if((rtimer_ext_now_lf() >= rt[timer].time) && \
     (rt[timer].state == RTIMER_EXT_SCHEDULED)) { \
    /* the timer has expired! */ \
    rt[timer].state = RTIMER_EXT_JUST_EXPIRED; \
    /* execute the proper callback function */ \
    rt[timer].func(&rt[timer]); \
    /* update or stop the timer */ \
    update_rtimer_ext_state(timer); \
    if(process_nevents() > 0) { \
      __bic_status_register_on_exit(LPM4_bits); /* LPM4_EXIT; */ \
    } \
  } else if(rt[timer].state == RTIMER_EXT_WFE) { \
    rt[timer].func(&rt[timer]); \
  }
/*---------------------------------------------------------------------------*/
void
rtimer_ext_init(void)
{
  /* initialize timer A0: */
  /* SMCLK (3.25 MHz), continuous mode, clear TA0R, overflow interrupt */
  ta0_sw_ext = 0;
  /* make sure the input divider expansion is set to 0 before setting the
   * TACLR bit */
  TA0EX0 = 0;
  TA0CTL = TASSEL_2 | MC_2 | ID__1 | TACLR | TAIE; /* SMCLK, input divider 1 */

  /* initialize timer A1: */
  /* ACLK, continuous mode, clear TA1R */
  ta1_sw_ext = 0;
  TA1EX0 = 0;
#if RTIMER_EXT_CONF_LF_UPDATE_INT
  TA1CTL = TASSEL_1 | MC_2 | ID__1 | TACLR | TAIE;
#else  /* RTIMER_EXT_CONF_LF_UPDATE_INT */
  TA1CTL = TASSEL_1 | MC_2 | ID__1 | TACLR;
#endif /* RTIMER_EXT_CONF_LF_UPDATE_INT */
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_schedule(rtimer_ext_id_t timer,
                    rtimer_ext_clock_t start,
                    uint32_t period,
                    rtimer_ext_callback_t func)
{
  if((timer < NUM_OF_RTIMER_EXTS)
      && (rt[timer].state != RTIMER_EXT_SCHEDULED)) {
    rt[timer].func = func;
    rt[timer].period = period;
    rt[timer].time = start + period;
    rt[timer].state = RTIMER_EXT_SCHEDULED;
    if(timer >= RTIMER_EXT_LF_0) {
      *(&TA1CCR0 + (timer - RTIMER_EXT_LF_0)) = (uint16_t) (start + period);
      *(&TA1CCTL0 + (timer - RTIMER_EXT_LF_0)) = CCIE | OUTMOD_4;
      /* if the scheduled time is in the past, then trigger an interrupt now */
      if(rt[timer].time <= rtimer_ext_now_lf()) {
        *(&TA1CCTL0 + (timer - RTIMER_EXT_LF_0)) |= CCIFG;
      }
    } else {
      *(&TA0CCR0 + timer) = (uint16_t) (start + period);
      *(&TA0CCTL0 + timer) = CCIE | OUTMOD_4; /* enable interrupt */
      /* if the scheduled time is in the past, then trigger an interrupt now */
      if(rt[timer].time <= rtimer_ext_now_hf()) {
        *(&TA0CCTL0 + timer) |= CCIFG;
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_wait_for_event(rtimer_ext_id_t timer,
                          rtimer_ext_callback_t func)
{
  /* must be an unscheduled timer */
  if((timer < NUM_OF_RTIMER_EXTS)
      && (rt[timer].state != RTIMER_EXT_SCHEDULED)) {
    rt[timer].func = func;
    rt[timer].state = RTIMER_EXT_WFE;
    if(timer >= RTIMER_EXT_LF_0) {
      /* set the timer to capture mode */
      /* rising edge, synchronize the capture with the next timer clock to
       * prevent race conditions, capture input select */
      *(&TA1CCTL0 + timer - RTIMER_EXT_LF_0) = CM_1 | SCS;
      /* only enable interrupts when a callback function is provided */
      if(func) {
        *(&TA1CCTL0 + timer - RTIMER_EXT_LF_0) |= (CAP | CCIE);
      }
    } else {
      /* set the timer to capture mode */
      *(&TA0CCTL0 + timer) = CM_1 | SCS;
      /* only enable interrupts when a callback function is provided */
      if(func) {
        *(&TA0CCTL0 + timer) |= (CAP | CCIE);
      }
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_stop(rtimer_ext_id_t timer)
{
  if(timer < NUM_OF_RTIMER_EXTS) {
    if(timer >= RTIMER_EXT_LF_0) {
      *(&TA1CCTL0 + timer - RTIMER_EXT_LF_0) = 0;
      rt[timer].state = RTIMER_EXT_INACTIVE;
    } else {
      *(&TA0CCTL0 + timer) = 0;
      rt[timer].state = RTIMER_EXT_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_reset(void)
{
  TA0R = 0;
  TA1R = 0;
  TA0CTL &= ~TAIFG;
  TA1CTL &= ~TAIFG;
  ta0_sw_ext = 0;
  ta1_sw_ext = 0;
  DCSTAT_RESET;
}
/*---------------------------------------------------------------------------*/
inline void
rtimer_ext_update_enable(void)
{
  TA0CTL |= TAIE;
}
/*---------------------------------------------------------------------------*/
inline void
rtimer_ext_update_disable(void)
{
  TA0CTL &= ~TAIE;
}
/*---------------------------------------------------------------------------*/
inline uint8_t
rtimer_ext_update_enabled(void)
{
  return ((TA0CTL & TAIE) > 0);
}
/*---------------------------------------------------------------------------*/
rtimer_ext_clock_t
rtimer_ext_now_hf(void)
{
  /* disable all interrupts */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE;
  /* or use: __get_SR_register()  or  (READ_SR & GIE) */
  __dint();
  __nop();

  /* take a snapshot of both the HW timer and the SW extension */
  rtimer_ext_clock_t sw = ta0_sw_ext;
  uint16_t hw = TA0R;
  if(TA0CTL & TAIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    ta0_sw_ext++;
    TA0CTL &= ~TAIFG;
    /* and take a new snapshot of the HW timer */
    hw = TA0R;
  }
  /* shift the SW extension to the left and append the HW timer */
  rtimer_ext_clock_t time = (sw << 16) | hw;

  /* only enable interrupts if the GIE bit was set before! otherwise interrupt
   * nesting will be enabled if rtimer_ext_now_hf() is called from an ISR! */
  if(interrupt_enabled) {
    __eint();
    __nop();
  }

  return time;
}
/*---------------------------------------------------------------------------*/
uint16_t
rtimer_ext_now_lf_hw(void)
{
  uint16_t hw1, hw2;
  do {
    /* majority vote: loop until both value are the same
     * (necessary because clock sources of the CPU and TA1 are different) */
    hw1 = TA1R;
    hw2 = TA1R;
  } while(hw1 != hw2);

  return hw1;
}
/*---------------------------------------------------------------------------*/
rtimer_ext_clock_t
rtimer_ext_now_lf(void)
{
  /* disable all interrupts */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE; //READ_SR & GIE;
  __dint();
  __nop();

  /* take a snapshot of both the HW timer and the SW extension */
  rtimer_ext_clock_t sw = ta1_sw_ext;
  uint16_t hw = rtimer_ext_now_lf_hw();
  if(TA1CTL & TAIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    ta1_sw_ext++;
    TA1CTL &= ~TAIFG;
    /* and take a new snapshot of the HW timer */
    hw = rtimer_ext_now_lf_hw();
  }
  /* shift the SW extension to the left and append the HW timer */
  rtimer_ext_clock_t time = (sw << 16) | hw;

  /* only enable interrupts if the GIE bit was set before! otherwise interrupt
   * nesting will be enabled if rtimer_ext_now_hf() is called from an ISR! */
  if(interrupt_enabled) {
    __eint();
    __nop();
  }

  return time;
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_now(rtimer_ext_clock_t* const hf_val, 
               rtimer_ext_clock_t* const lf_val)
{
  /* NOTE: This function will only work properly if the CPU and timer TA0 (HF)
   * are running from the same clock source */
  if(hf_val && lf_val) {
    /* disable all interrupts */
    uint16_t interrupt_enabled = __get_interrupt_state() & GIE;//READ_SR & GIE;
    __dint();
    __nop();

    /* take a snapshot of the SW extension */
    rtimer_ext_clock_t sw_hf = ta0_sw_ext;
    rtimer_ext_clock_t sw_lf = ta1_sw_ext;
    uint16_t hw_hf;
    uint16_t hw_lf;
    uint16_t hw_lf2;
    while(1) {
      hw_hf = TA0R;
      hw_lf = TA1R;
      hw_lf2 = TA1R;
      if(hw_lf != hw_lf2) {
        continue;
      }
      if((TA0CTL & TAIFG) && (sw_hf == ta0_sw_ext)) {
        /* in the meantime there has been an overflow of the HW timer: */
        /* manually increment the SW extension and recapture all values */
        sw_hf++;
        continue;
      }
      if((TA1CTL & TAIFG) && (sw_lf == ta1_sw_ext)) {
        /* in the meantime there has been an overflow of the HW timer: */
        /* manually increment the SW extension and recapture all values */
        sw_lf++;
        continue;
      }
      break;
    }
    /* compose the final timestamps (shift the SW extension to the left and
     * append the HW timer */
    *hf_val = (sw_hf << 16) | hw_hf;
    *lf_val = (sw_lf << 16) | hw_lf;

    /* only enable interrupts if the GIE bit was set before! otherwise ISR
     * nesting will be enabled if rtimer_ext_now_hf() is called from an ISR! */
    if(interrupt_enabled) {
      __eint();
      __nop();
    }
  }
}
/*---------------------------------------------------------------------------*/
uint16_t
rtimer_ext_swext_addr(rtimer_ext_id_t timer)
{
  if(timer < RTIMER_EXT_CONF_NUM_HF) {
    return (uint16_t) &ta0_sw_ext;
  }
  return (uint16_t) &ta1_sw_ext;
}
/*---------------------------------------------------------------------------*/
uint8_t
rtimer_ext_next_expiration(rtimer_ext_id_t timer, rtimer_ext_clock_t* exp_time)
{
  if(timer < NUM_OF_RTIMER_EXTS) {
    *exp_time = rt[timer].time;
    return (rt[timer].state == RTIMER_EXT_SCHEDULED);
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/* Timer A0, CCR0 interrupt service routine */
ISR(TIMER0_A0, timer0_a0_interrupt)
{
  DCSTAT_CPU_ON;
#if ENERGEST_CONF_ON
  ENERGEST_ON(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */

  RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_0);

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
/* Timer A0, CCR1-4 interrupt service routine */
ISR(TIMER0_A1, timer0_a1_interrupt)
{
  DCSTAT_CPU_ON;
#if ENERGEST_CONF_ON
  ENERGEST_ON(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */

  switch (TA0IV) {
    case TA0IV_TA0CCR1:
      RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_1);
      break;
    case TA0IV_TA0CCR2:
      RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_2);
      break;
    case TA0IV_TA0CCR3:
      RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_3);
      break;
#if !RF_CONF_ON
    case TA0IV_TA0CCR4:
      RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_4);
      break;
#endif /* RF_CONF_ON */
    case TA0IV_TA0IFG:
      /* overflow of timer A0: increment its software extension */
      ta0_sw_ext++;
      /* update the LF timer if necessary */
      RTIMER_EXT_LF_HANDLE_OVF();
      break;
    default:
      break;
  }

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
/* Timer A1, CCR0 interrupt service routine (higher priority than TIMER1_A1) */
ISR(TIMER1_A0, timer1_a0_interrupt)
{
  DCSTAT_CPU_ON;
#if ENERGEST_CONF_ON
  ENERGEST_ON(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */

  /* can be used by application */
  RTIMER_EXT_LF_CALLBACK(RTIMER_EXT_LF_0);

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
/* Timer A1, CCR1-2 interrupt service routine */
ISR(TIMER1_A1, timer1_a1_interrupt)
{
  DCSTAT_CPU_ON;
#if ENERGEST_CONF_ON
  ENERGEST_ON(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */

  switch (TA1IV) {
    /* reserved for Contiki rtimer, but can also be used as rtimer-ext */
    case TA1IV_TA1CCR1:
      RTIMER_EXT_LF_CALLBACK(RTIMER_EXT_LF_1);
      break;
    /* reserved for etimer */
    case TA1IV_TA1CCR2:
#if RTIMER_EXT_CONF_USE_ETIMER
      /* increment also the etimer count */
      clock_count++;
      /* increment seconds counter if necessary */
      if(clock_count % CLOCK_CONF_SECOND == 0) {
        clock_secs++;
      }
      TA1CCR2 += CLOCK_INTERVAL;
      /* check whether there are etimers ready to be served */
      if(etimer_pending() &&
        (etimer_next_expiration_time() - clock_count - 1) > MAX_TICKS) {
        etimer_request_poll();
        __bic_status_register_on_exit(LPM4_bits); /* LPM4_EXIT; */
      }
#else /* RTIMER_EXT_CONF_USE_ETIMER */
      RTIMER_EXT_LF_CALLBACK(RTIMER_EXT_LF_2);
#endif /* RTIMER_EXT_CONF_USE_ETIMER */
      break;
    case TA1IV_TA1IFG:
      /* overflow of timer A1: increment its software extension */
      ta1_sw_ext++;
#if WATCHDOG_CONF_ON && WATCHDOG_CONF_RESET_ON_TA1IFG
      watchdog_reset();
#endif /* WATCHDOG */
      break;
    default:
      break;
  }

#if ENERGEST_CONF_ON
  ENERGEST_OFF(ENERGEST_TYPE_CPU);
#endif /* ENERGEST_CONF_ON */
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
