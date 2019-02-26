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
 * TA is a low-frequency (LF) timer with 1 CCRs (ACLK = 32kHz)
 * TB is a high-frequency (HF) timer with 1 CCRs (DCO = F_CPU)
 *
 * TA uses CCR1
 * TB uses CCR0
 *
 * This file also implements etimer by using CCR2 and hence overrides the
 * default implementation in clock.c.
 * NOTE: CCR2 is also used by Glossy for the clock sync, but register values
 *       are restored after usage.
 */
/*---------------------------------------------------------------------------*/
#include <string.h>
#include "contiki.h"
#include "energest.h"
#include "watchdog.h"
#include "rtimer-ext.h"
#include "isr_compat.h"
#include "dc-stat.h"
/*---------------------------------------------------------------------------*/
#define ETIMER_INTERVAL (RTIMER_ARCH_SECOND / CLOCK_SECOND)

#define MAX_TICKS (~((clock_time_t)0) / 2)

#define CLOCK_LT(a, b) ((int16_t)((a)-(b)) < 0)
/*---------------------------------------------------------------------------*/
static rtimer_ext_t rt[NUM_OF_RTIMER_EXTS]; /* rtimer_ext structs */
volatile rtimer_ext_clock_t ta1_sw_ext;
volatile rtimer_ext_clock_t tb0_sw_ext;

static volatile unsigned long seconds;
static volatile clock_time_t count = 0;
/*---------------------------------------------------------------------------*/
/* functions from clock.c                                                    */
/*---------------------------------------------------------------------------*/
void
clock_init(void)
{
  /* make sure the timer is configured and running */
  rtimer_ext_init();

#if RTIMER_EXT_CONF_USE_ETIMER
  /* Initialize ccr1 to create the X ms interval. */
  /* CCR2 interrupt enabled, interrupt occurs when timer equals CCR. */
  TACCTL2 = CCIE;

  /* Interrupt after X ms. */
  TACCR2 = ETIMER_INTERVAL;
#endif /* RTIMER_EXT_CONF_USE_ETIMER */

  count = 0;
}
/*---------------------------------------------------------------------------*/
/*clock_time_t
clock_time(void)
{
  return (unsigned long)(rtimer_ext_now_lf() /
                         (RTIMER_EXT_SECOND_LF / CLOCK_CONF_SECOND));
}*/
/*---------------------------------------------------------------------------*/
/*unsigned long
clock_seconds(void)
{
  return (unsigned long)(rtimer_ext_now_lf() / RTIMER_EXT_SECOND_LF);
}*/
/*---------------------------------------------------------------------------*/
/**
 * Delay the CPU for a multiple of 2.83 us.
 */
void
clock_delay(unsigned int i)
{
  while(i--) {
    _NOP();
  }
}
/*---------------------------------------------------------------------------*/
clock_time_t
clock_time(void)
{
  clock_time_t t1, t2;
  do {
    t1 = count;
    t2 = count;
  } while(t1 != t2);
  return t1;
}
/*---------------------------------------------------------------------------*/
void
clock_set(clock_time_t clock, clock_time_t fclock)
{
  TAR = fclock;
  TACCR2 = fclock + ETIMER_INTERVAL;
  count = clock;
}
/*---------------------------------------------------------------------------*/
unsigned long
clock_seconds(void)
{
  return seconds;
}
/*---------------------------------------------------------------------------*/
rtimer_clock_t
clock_counter(void)
{
  rtimer_clock_t t1, t2;
  do {
    t1 = TAR;
    t2 = TAR;
  } while(t1 != t2);
  return t1;
}
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
        *(&TACCR1 + (timer - RTIMER_EXT_LF_0)) +=
            (uint16_t) (rt[timer].period);
      } else {
        *(&TBCCR0 + timer) += (uint16_t) (rt[timer].period);
      }
      rt[timer].state = RTIMER_EXT_SCHEDULED;
    } else {
      if(timer >= RTIMER_EXT_LF_0) {
        /* otherwise, just stop it */
        *(&TACCTL1 + (timer - RTIMER_EXT_LF_0)) = 0;
      } else {
        *(&TBCCTL0 + timer) = 0;
      }
      rt[timer].state = RTIMER_EXT_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
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
  /* initialise HF timer B */
  TBCCTL0 = 0;
  // stop Timer B
  TBCTL = 0;
  /* SMCLK (DCO) | cont mode | /1 | overflow interrupt enable */
  TBCTL |= TBSSEL_2 | MC_2 | ID_0;
#if RTIMER_EXT_CONF_HF_ENABLE
  TBCTL |= TBIE;
#endif /* RTIMER_EXT_CONF_HF_ENABLE */

  /* initialise LF timer A */
  TACTL = 0;
  TACTL |= TASSEL_1 | ID_0 | MC_2 | TAIE;   /* ACLK */

  TACCTL1 = 0;

  /* NOTE: TACCR0 is used for rtimer and CCR2 is used by Glossy for 
   *       clock sync! */

  memset(rt, 0, sizeof(rt));
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_schedule(rtimer_ext_id_t timer, rtimer_ext_clock_t start,
                    rtimer_ext_clock_t period, rtimer_ext_callback_t func)
{
  if((timer < NUM_OF_RTIMER_EXTS)
      && (rt[timer].state != RTIMER_EXT_SCHEDULED)) {
    rt[timer].func = func;
    rt[timer].period = period;
    rt[timer].time = start + period;
    rt[timer].state = RTIMER_EXT_SCHEDULED;
    if(timer >= RTIMER_EXT_LF_0) {
      *(&TACCR1 + (timer - RTIMER_EXT_LF_0)) = (uint16_t) (start + period);
      *(&TACCTL1 + (timer - RTIMER_EXT_LF_0)) = CCIE | OUTMOD_4;
      /* if the scheduled time is in the past, then trigger an interrupt now */
      if(rt[timer].time <= rtimer_ext_now_lf()) {
        *(&TACCTL1 + (timer - RTIMER_EXT_LF_0)) |= CCIFG;
      }
    }
#if RTIMER_EXT_CONF_HF_ENABLE
    else {
      *(&TBCCR0 + timer) = (uint16_t) (start + period);
      *(&TBCCTL0 + timer) = CCIE | OUTMOD_4; /* enable interrupt */
      /* if the scheduled time is in the past, then trigger an interrupt now */
      if(rt[timer].time <= rtimer_ext_now_hf()) {
        *(&TBCCTL0 + timer) |= CCIFG;
      }
    }
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_wait_for_event(rtimer_ext_id_t timer, rtimer_ext_callback_t func)
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
      *(&TACCTL1 + timer - RTIMER_EXT_LF_0) = CM_1 | SCS;
      /* only enable interrupts when a callback function is provided */
      if(func) {
        *(&TACCTL1 + timer - RTIMER_EXT_LF_0) |= (CAP | CCIE);
      }
    } else {
      /* set the timer to capture mode */
      *(&TBCCTL0 + timer) = CM_1 | SCS;
      /* only enable interrupts when a callback function is provided */
      if(func) {
        *(&TBCCTL0 + timer) |= (CAP | CCIE);
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
      *(&TACCTL1 + timer - RTIMER_EXT_LF_0) = 0;
      rt[timer].state = RTIMER_EXT_INACTIVE;
    } else {
      *(&TBCCTL0 + timer) = 0;
      rt[timer].state = RTIMER_EXT_INACTIVE;
    }
  }
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_reset(void)
{
  TAR = 0;
  TBR = 0;
  TACTL &= ~TAIFG;
  TBCTL &= ~TBIFG;
  ta1_sw_ext = 0;
  tb0_sw_ext = 0;
  DCSTAT_RESET;
}
/*---------------------------------------------------------------------------*/
inline void
rtimer_ext_update_enable(void)
{
  TACTL |= TAIE;
}
/*---------------------------------------------------------------------------*/
inline void
rtimer_ext_update_disable(void)
{
  TACTL &= ~TAIE;
}
/*---------------------------------------------------------------------------*/
inline uint8_t
rtimer_ext_update_enabled(void)
{
  return ((TACTL & TAIE) > 0);
}
/*---------------------------------------------------------------------------*/
#if RTIMER_EXT_CONF_HF_ENABLE
rtimer_ext_clock_t
rtimer_ext_now_hf(void)
{
  /* disable all interrupts */
  uint16_t interrupt_enabled = __get_interrupt_state() & GIE;
  /* or use: __get_SR_register()  or  (READ_SR & GIE) */
  __dint();
  __nop();

  /* take a snapshot of both the HW timer and the SW extension */
  rtimer_ext_clock_t sw = tb0_sw_ext;
  uint16_t hw = TBR;
  if(TBCTL & TBIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    tb0_sw_ext++;
    TBCTL &= ~TBIFG;
    /* and take a new snapshot of the HW timer */
    hw = TBR;
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
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
/*---------------------------------------------------------------------------*/
uint16_t
rtimer_ext_now_lf_hw(void)
{
  uint16_t hw1, hw2;
  do {
    /* majority vote: loop until both value are the same
     * (necessary because clock sources of the CPU and TA are different) */
    hw1 = TAR;
    hw2 = TAR;
  }
  while (hw1 != hw2);
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
  if(TACTL & TAIFG) {
    /* in the meantime there has been an overflow of the HW timer: */
    /* manually increment the SW extension */
    sw++;
    ta1_sw_ext++;
    TACTL &= ~TAIFG;
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
#if RTIMER_EXT_CONF_HF_ENABLE
void
rtimer_ext_now(rtimer_ext_clock_t* const hf_val,
               rtimer_ext_clock_t* const lf_val)
{
  /* NOTE: This function will only work properly if the CPU and timer TA0 (HF)
   * are running from the same clock source */
  if(hf_val && lf_val) {
    /* disable all interrupts */
    uint16_t interrupt_enabled = __get_interrupt_state() & GIE; //READ_SR & GIE;
    __dint();
    __nop();

    /* take a snapshot of the SW extension */
    rtimer_ext_clock_t sw_hf = tb0_sw_ext;
    rtimer_ext_clock_t sw_lf = ta1_sw_ext;
    uint16_t hw_hf;
    uint16_t hw_lf;
    uint16_t hw_lf2;
    while (1) {
      hw_hf = TBR;
      hw_lf = TAR;
      hw_lf2 = TAR;
      if(hw_lf != hw_lf2) {
        continue;
      }
      if((TBCTL & TBIFG) && (sw_hf == tb0_sw_ext)) {
        /* in the meantime there has been an overflow of the HW timer: */
        /* manually increment the SW extension and recapture all values */
        sw_hf++;
        continue;
      }
      if((TACTL & TAIFG) && (sw_lf == ta1_sw_ext)) {
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
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
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
ISR(TIMERA1, timera1)
{
  DCSTAT_CPU_ON;
  watchdog_start();
  /* LF */
  switch (TAIV) {
    case TAIV_TACCR1:
      RTIMER_EXT_LF_CALLBACK(RTIMER_EXT_LF_0);
      break;
    case TAIV_TACCR2:
      /* HW timer bug fix: Interrupt handler called before TR==CCR.
      * Occurs when timer state is toggled between STOP and CONT. */
      while(TACTL & MC1 && TACCR2 - clock_counter() == 1);

      uint16_t last_tar = clock_counter();
      /* Make sure interrupt time is future */
      while(!CLOCK_LT(last_tar, TACCR2)) {
        TACCR2 += ETIMER_INTERVAL;
        ++count;
        if(count % CLOCK_CONF_SECOND == 0) {
          ++seconds;
          energest_flush();
        }
        last_tar = clock_counter();
      }
      if(etimer_pending() &&
        (etimer_next_expiration_time() - count - 1) > MAX_TICKS) {
        etimer_request_poll();
        LPM4_EXIT;
      }
      break;
    case TAIV_TAIFG:
      ta1_sw_ext++;
      break;
    default:
      break;
  }
  watchdog_stop();
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
#if RTIMER_EXT_CONF_HF_ENABLE
ISR(TIMERB0, timerb0)
{
  DCSTAT_CPU_ON;
  watchdog_start();
  /* HF */
  RTIMER_EXT_HF_CALLBACK(RTIMER_EXT_HF_0);
  watchdog_stop();
  DCSTAT_CPU_OFF;
}
/*---------------------------------------------------------------------------*/
void
rtimer_ext_notify_hf_timer_overflow(void)
{
  tb0_sw_ext++;
}
#endif /* RTIMER_EXT_CONF_HF_ENABLE */
/*---------------------------------------------------------------------------*/
