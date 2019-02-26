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
 * Author:  Federico Ferrari
 *          Reto Da Forno
 *          Jonas Baechli
 */

/**
 * \file
 * This is a extension of Contikis rtimer interface.
 * It offers an easy way to access multiple hardware timers in a way that is compatible to the original rtimer interface.
 * rtimer-arch is implemented on top of rtimer-ext.
 */

#ifndef RTIMER_EXT_H_
#define RTIMER_EXT_H_

#include "contiki-conf.h"


#ifndef RTIMER_EXT_CONF_NUM_HF       /* number of (usable) high-frequency timers */
#error "RTIMER_EXT_CONF_NUM_HF not defined!"
#endif /* RTIMER_EXT_CONF_NUM_HF */

#ifndef RTIMER_EXT_CONF_NUM_LF        /* number of (usable) low-frequency timers */
#error "RTIMER_EXT_CONF_NUM_LF not defined!"
#endif /* RTIMER_EXT_CONF_NUM_LF */

#ifndef RTIMER_EXT_CONF_USE_ETIMER
#define RTIMER_EXT_CONF_USE_ETIMER      1
#endif /* RTIMER_EXT_CONF_USE_ETIMER */

#ifndef RTIMER_EXT_CONF_HF_CLKSPEED
#define RTIMER_EXT_CONF_HF_CLKSPEED     SMCLK_SPEED
#endif /* RTIMER_EXT_CONF_HF_CLKSRC */

#ifndef RTIMER_EXT_CONF_LF_CLKSPEED
#define RTIMER_EXT_CONF_LF_CLKSPEED     ACLK_SPEED
#endif /* RTIMER_EXT_CONF_HF_CLKSRC */

/* overflow/update interrupt enable (when a LF CCR interrupt is used which
 * occurs at least once every 2 seconds, the timer overflow can be handled
 * within that ISR) */
#ifndef RTIMER_EXT_CONF_LF_UPDATE_INT
#define RTIMER_EXT_CONF_LF_UPDATE_INT   1
#endif /* RTIMER_EXT_CONF_LF_UPDATE_INT */

/**
 * @brief the number of timer ticks that (approx.) correspond to 1s
 */
#define RTIMER_EXT_SECOND_HF            ((rtimer_ext_clock_t)RTIMER_EXT_CONF_HF_CLKSPEED)
#define RTIMER_EXT_SECOND_LF            (RTIMER_EXT_CONF_LF_CLKSPEED)
#define RTIMER_EXT_HF_LF_RATIO          (RTIMER_EXT_SECOND_HF / RTIMER_EXT_SECOND_LF)

/**
 * @brief convert rtimer_ext values from clock ticks to milliseconds and
 * nanoseconds to rtimer_ext ticks
 */
#define RTIMER_EXT_HF_TO_MS(t)          ((t) / (RTIMER_EXT_SECOND_HF / 1000))
#define RTIMER_EXT_LF_TO_MS(t)          ((t * 1000) / (RTIMER_EXT_SECOND_LF))
#define NS_TO_RTIMER_EXT_HF_32(ns)      ((uint32_t)(ns) / \
                                     (1000000000 / RTIMER_EXT_SECOND_HF))
#define NS_TO_RTIMER_EXT_HF(ns)         (((rtimer_ext_clock_t)(ns) * \
                                      (rtimer_ext_clock_t)RTIMER_EXT_SECOND_HF) / \
                                     (rtimer_ext_clock_t)1000000000)


typedef uint64_t rtimer_ext_clock_t;

/**
 * @brief the rtimer_ext IDs
 */
typedef enum {
  RTIMER_EXT_HF_0 = 0,
#if RTIMER_EXT_CONF_NUM_HF > 1
  RTIMER_EXT_HF_1,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 2
  RTIMER_EXT_HF_2,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 3
  RTIMER_EXT_HF_3,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 4
  RTIMER_EXT_HF_4,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 5
  RTIMER_EXT_HF_5,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 6
  RTIMER_EXT_HF_6,
#endif
#if RTIMER_EXT_CONF_NUM_HF > 7
  RTIMER_EXT_HF_7,
#endif
  RTIMER_EXT_LF_0,
#if RTIMER_EXT_CONF_NUM_LF > 1
  RTIMER_EXT_LF_1,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 2
  RTIMER_EXT_LF_2,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 3
  RTIMER_EXT_LF_3,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 4
  RTIMER_EXT_LF_4,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 5
  RTIMER_EXT_LF_5,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 6
  RTIMER_EXT_LF_6,
#endif
#if RTIMER_EXT_CONF_NUM_LF > 7
  RTIMER_EXT_LF_7,
#endif
  NUM_OF_RTIMER_EXTS
} rtimer_ext_id_t;

/* this is necessary for the rtimer_ext_callback_t prototype declaration */
struct rtimer_ext;

/* prototype of a rtimer_ext callback function */
typedef char (*rtimer_ext_callback_t)(struct rtimer_ext *rt);

typedef enum {
  RTIMER_EXT_INACTIVE = 0,
  RTIMER_EXT_SCHEDULED = 1,
  RTIMER_EXT_JUST_EXPIRED = 2,
  RTIMER_EXT_WFE = 3,      /* wait for event */
} rtimer_ext_state_t;

/**
 * @brief state and parameters of an rtimer_ext
 */
typedef struct rtimer_ext {
  rtimer_ext_clock_t time;    /* if state = RTIMER_EXT_SCHEDULED: next expiration time;
                             otherwise: last expiration time */
  uint32_t period;            /* if period = 0: one-shot timer; otherwise: timer
                             period in clock ticks */
  rtimer_ext_callback_t func; /* callback function to execute when the rtimer_ext
                             expires */
  rtimer_ext_state_t state;   /* internal state of the rtimer_ext */
} rtimer_ext_t;


/**
 * @brief initialize the timers TA0 and TA1
 */
void rtimer_ext_init(void);

/**
 * @brief schedule (start) an rtimer_ext
 * @param[in] timer the ID of the rtimer_ext, must by of type rtimer_ext_id_t
 * @param[in] start the expiration time (absolute counter value)
 * @param[in] period the period; set this to zero if you don't want this timer
 * trigger an interrupt periodically
 * @param[in] func the callback function; will be executed as soon as the timer
 * has expired
 */
void rtimer_ext_schedule(rtimer_ext_id_t timer,
                         rtimer_ext_clock_t start,
                         uint32_t period,
                         rtimer_ext_callback_t func);

/**
 * @brief set a CCR of a timer to event mode (capture input)
 */
void rtimer_ext_wait_for_event(rtimer_ext_id_t timer, rtimer_ext_callback_t func);

/**
 * @brief stop an rtimer_ext
 * @param[in] timer the ID of the rtimer_ext, must by of type rtimer_ext_id_t
 */
void rtimer_ext_stop(rtimer_ext_id_t timer);

/**
 * @brief resets the rtimer_ext values (both LF and HF) to zero
 */
void rtimer_ext_reset(void);

/**
 * @brief enable the overflow/update interrupts for both, the LF and HF timer
 */
inline void rtimer_ext_update_enable(void);

/**
 * @brief disable the overflow/update interrupts for both, the LF and HF timer
 */
inline void rtimer_ext_update_disable(void);

/**
 * @brief check whether the overflow/update interrupt (of the HF timer) is
 * enabled
 */
inline uint8_t rtimer_ext_update_enabled();

/**
 * @brief get the current timer value of TA0 (high frequency)
 * @return timer value in timer clock ticks (timestamp)
 */
rtimer_ext_clock_t rtimer_ext_now_hf(void);

/**
 * @brief get the current timer value of TA1 (low frequency) with SW extension
 * @return timestamp (64 bits)
 */
rtimer_ext_clock_t rtimer_ext_now_lf(void);

/**
 * @brief get the current timer value of TA1 (low frequency)
 * @return timer value (16 bits)
 */
uint16_t rtimer_ext_now_lf_hw(void);

/**
 * @brief get the current timer value of both, the high and low frequency
 * timer
 * @param[in] hf_val value in timer clock ticks (timestamp)
 * @param[in] lf_val value in timer clock ticks (timestamp)
 * @remark This function will only work properly if the CPU is running on the
 * same clock source as the timer TA0 (HF) and this function can be executed
 * within one TA0 period (i.e. ~20ms @ 3.25 MHz).
 */
void rtimer_ext_now(rtimer_ext_clock_t* const hf_val, rtimer_ext_clock_t* const lf_val);

/**
 * @brief get the address of the software extensions of the timer module
 * @param[in] timer the ID of an rtimer_ext
 * @return the address of the software extension
 */
uint16_t rtimer_ext_swext_addr(rtimer_ext_id_t timer);

/**
 * @brief get the timestamp of the next scheduled expiration
 * @param[in] timer the ID of an rtimer_ext
 * @param[out] exp_time expiration time
 * @return 1 if the timer is still active, 0 otherwise
 */
uint8_t rtimer_ext_next_expiration(rtimer_ext_id_t timer, rtimer_ext_clock_t* exp_time);

#endif /* RTIMER_EXT_H_ */
