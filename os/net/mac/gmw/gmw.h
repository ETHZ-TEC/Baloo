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
 * \author
 *         Jonas Baechli   jonas.baechli@bluewin.ch
 *         Reto Da Forno   rdaforno@ee.ethz.ch
 *         Romain Jacob    jacobr@ethz.ch
 */
/*---------------------------------------------------------------------------*/
/**
 * @addtogroup  Net
 * @{
 */
/*---------------------------------------------------------------------------*/
/**
 * \defgroup gmw Glossy Middleware (GMW)
 *
 * The Glossy Middleware (GMW) is the underlying software module for using
 * Baloo, a flexible low-power network stack based on Synchronous Transmissions.
 *
 * @{
 */

 /**
  * \file
  *         This file is the main header file for the Glossy Middleware (GMW)
  */
 
#ifndef __GMW_H__
#define __GMW_H__

/* the correct include order is important */
#include "gmw-platform-conf.h"
#include "gmw-platform.h"
#include "gmw-conf.h"
#include "gmw-types.h"
#include "gmw-control.h"
#if GMW_CONF_USE_NOISE_DETECTION
#include "gmw-noise-detect.h"
#endif /* GMW_CONF_USE_NOISE_DETECTION */

#ifndef HOST_ID
#warning "HOST_ID not defined, set to 0"
#define HOST_ID    0
#endif

#if DEPCOMP
#include "config.h"
#endif
/*---------------------------------------------------------------------------*/

/* important values, do not modify */

/*
 * Minimum duration in us of a Glossy flood according to "Energy-efficient
 * Real-time Communication in Multi-hop Low-power Wireless Networks"
 * (Zimmerling et al.) plus 250 us of slack time.
 * GMW_T_HOP(len) should return the duration in us the radio hardware needs to
 *  transmit a packet of length len.
 * @param len   total size of the packet sent by the hardware
 * @param n     number of retransmissions, a glossy parameter
 * @param hops  maximal number of hops in the network
 * */
#define GMW_T_SLOT_MIN(len, n, hops)    ((hops + (2 * n) - 1) * \
                                          GMW_T_HOP(len) + \
                                          250 )
/* same formula but adapted for the strobing mode */
#define GMW_T_SLOT_STROBE_MIN(len, n)    (n * (GMW_T_HOP(len) + \
                                          STROBING_CONF_TX_TO_TX_DELAY) + 250 )

/*---------------------------------------------------------------------------*/
/* some macros to calculate from us to the base time unit used in the config
 * and back into us and clock ticks
 */

/* converts from us into multiples of GMW_CONF_SLOT_TIME_BASE */
#define GMW_US_TO_SLOT_TIME(t)          (((t) / GMW_CONF_SLOT_TIME_BASE) + \
                                        (((t) % GMW_CONF_SLOT_TIME_BASE) != 0))

/* converts from GMW_CONF_SLOT_TIME_BASE to local clock ticks */
#define GMW_SLOT_TIME_TO_TICKS(t)       ((gmw_rtimer_clock_t)(t) * \
                                        GMW_CONF_SLOT_TIME_BASE_CLOCK)

#define GMW_SLOT_TIME_TO_US(t)          ((t) * GMW_CONF_SLOT_TIME_BASE)

/* converts from us into multiples of GMW_CONF_GAP_TIME_BASE */
#define GMW_US_TO_GAP_TIME(t)           (((t) / GMW_CONF_GAP_TIME_BASE) + \
                                        (((t) % GMW_CONF_GAP_TIME_BASE) != 0))

/* converts from GMW_CONF_GAP_TIME_BASE to local clock ticks */
#define GMW_GAP_TIME_TO_TICKS(t)        ((gmw_rtimer_clock_t)(t) * \
                                        GMW_CONF_GAP_TIME_BASE_CLOCK)

#define GMW_GAP_TIME_TO_US(t)           ((t) * GMW_CONF_GAP_TIME_BASE)

#define GMW_US_TO_TICKS(t)              ((t) * GMW_RTIMER_SECOND / 1000000UL)

#define GMW_TICKS_TO_MS(t)              ((t) * 1000UL / GMW_RTIMER_SECOND)

#define GMW_TICKS_TO_US(t)              ((t) * 1000000UL / GMW_RTIMER_SECOND)

/* converts from time and period into ms */
#define GMW_PERIOD_TO_MS(t)             ((t) * 1000UL / GMW_CONF_TIME_SCALE)

#define GMW_PERIOD_TO_TICKS(t)          ((t) * GMW_RTIMER_SECOND / \
                                        GMW_CONF_TIME_SCALE)

#define GMW_MS_TO_TICKS(t)              ((t) * GMW_RTIMER_SECOND / 1000UL)

/* suitable to convert us/ppm (signed integer) to clock ticks */
#define GMW_PPM_TO_TICKS(t)             ((int32_t)(t) * \
                                        (int32_t)GMW_RTIMER_SECOND / 1000000)
#define GMW_TICKS_TO_PPM(t)             ((int32_t)(t) * 1000000 / \
                                        (int32_t)GMW_RTIMER_SECOND)

/*---------------------------------------------------------------------------*/

#ifndef MAX
#define MAX(x, y)                       ((x) > (y) ? (x) : (y))
#endif /* MAX */

#ifndef MIN
#define MIN(x, y)                       ((x) < (y) ? (x) : (y))
#endif /* MIN */

/*---------------------------------------------------------------------------*/

#if GMW_CONF_USE_MULTI_PRIMITIVES
uint8_t gmw_primitive;                              /* current primitive */
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */

/*---------------------------------------------------------------------------*/

/**
 * @brief                       Prepare the host and source implementation
 *                              structures required to start the middleware and
 *                              call the control structure initialization
 *                              functions from the middleware
 *                              (load default parameters) and the application.
 * @param host_protocol_impl    Hold the host implementation called by GMW
 * @param src_protocol_impl     Hold the source implementation called by GMW
 * @param control               Application side copy of the control structure
 */
void
gmw_init(gmw_protocol_impl_t* host_protocol_impl,
         gmw_protocol_impl_t* src_protocol_impl,
         gmw_control_t* control);

/**
 * @brief                       Start the Glossy Middleware
 * @param pre_gmw_proc          Pointer to the application task process control
 *                              block (struct process) that needs to be executed
 *                              before an GMW round. Set GMW_CONF_T_PREPROCESS
 *                              to the worst-case execution time (in ms) of
 *                              this function.
 * @param post_gmw_proc         Pointer to the application task process control
 *                              block (struct process), this process will be
 *                              called (polled) at the end of an GMW round
 * @param host_protocol_impl    Hold the host implementation called by GMW
 * @param src_protocol_impl     Hold the source implementation called by GMW
 */
void
gmw_start(struct process* pre_gmw_proc,
          struct process* post_gmw_proc,
          gmw_protocol_impl_t* host_protocol_impl,
          gmw_protocol_impl_t* src_protocol_impl);

/**
 * @brief                       set the pointer for the new schedule (host only)
 *                              It is memcpyed at the beginning of each round
 *                              and then used as the active control structure.
 * @param control               pointer to the new control to use in the next
 *                              round
 */
void
gmw_set_new_control(gmw_control_t* control);

/**
 * @brief                       query the sync status of the GMW
 * @return                      GMW state of type gmw_sync_state_t
 */
gmw_sync_state_t
gmw_get_state(void);

/**
 * @brief                       get the GMW statistics
 */
const gmw_statistics_t *
const gmw_get_stats(void);

/**
 * @brief                       reset the GMW statistics
 */
void
gmw_stats_reset(void);

/** @} */

/** @} */

#endif /* __GMW_H__ */
