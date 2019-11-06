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
 * \addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/

/**
 * \file
 *          This file contains the default macro to start and stop the execution
 *          of synchronous transmission primitives (Glossy by default),
 *          and interact with the rtimer and the radio.
 *          All these defaults can be overwritten to adapt to the
 *          platform-specific implementations of Glossy and rtimer.
 */

//TODO: Does it actually make sense to have such default defines here?

#ifndef GMW_PLATFORM_CONF_H_
#define GMW_PLATFORM_CONF_H_

/* override the default config with the platform specific config, if given */
#ifdef GMW_PLATFORM_CONF_PATH
#include GMW_PLATFORM_CONF_PATH
#else /* GMW_PLATFORM_CONF_PATH */
#warning "GMW_PLATFORM_CONF_PATH not defined"
#endif /* GMW_PLATFORM_CONF_PATH */

/*---------------------------------------------------------------------------*/
/*--- GLOSSY MACROS ---*/
/*---------------------------------------------------------------------------*/

#ifndef GMW_START
#define GMW_START(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal) \
      glossy_start(initiator_id, payload, payload_len, n_tx_max, sync, rf_cal)
#endif /* GMW_START */

#ifndef GMW_STOP
#define GMW_STOP()                      glossy_stop();  \
                                        stats.relay_cnt \
                                        = GMW_GET_RELAY_CNT_FIRST_RX();
#endif /* GMW_STOP */

#ifndef GMW_GET_T_REF
/* return HF timestamp of the start of the first reception */
#define GMW_GET_T_REF()                 glossy_get_t_ref()
#endif /* GMW_GET_T_REF */

#ifndef GMW_IS_T_REF_UPDATED
#define GMW_IS_T_REF_UPDATED()          glossy_is_t_ref_updated()
#endif /* GMW_IS_T_REF_UPDATED */

#ifndef GMW_GET_RELAY_CNT_FIRST_RX
#define GMW_GET_RELAY_CNT_FIRST_RX()    glossy_get_relay_cnt()
#endif /* GMW_GET_RELAY_CNT_FIRST_RX */

#ifndef GMW_HIGH_NOISE_DETECTED
  #define GMW_HIGH_NOISE_DETECTED()     gmw_high_noise_detected()
#endif /* GMW_HIGH_NOISE_DETECTED */

/* Glossy defines (mainly parameters for glossy start) */

#ifndef GMW_UNKNOWN_INITIATOR
#define GMW_UNKNOWN_INITIATOR    0
#endif /* GMW_UNKNOWN_INITIATOR */

#ifndef GMW_WITHOUT_RF_CAL
#define GMW_WITHOUT_RF_CAL       0
#endif /* GMW_WITHOUT_RF_CAL */

#ifndef GMW_WITH_RF_CAL
#define GMW_WITH_RF_CAL          1
#endif /* GMW_WITH_RF_CAL */

#ifndef GMW_WITH_SYNC
#define GMW_WITH_SYNC            1
#endif /* GMW_WITH_SYNC */

#ifndef GMW_WITHOUT_SYNC
#define GMW_WITHOUT_SYNC         0
#endif /* GMW_WITHOUT_SYNC */

/*---------------------------------------------------------------------------*/
/*--- RTIMER MACROS ---*/
/*---------------------------------------------------------------------------*/

#ifndef GMW_RTIMER_SCHEDULE
#define GMW_RTIMER_SCHEDULE(time, callback_func)     \
                 rtimer_ext_schedule(GMW_CONF_RTIMER_ID, time, 0, callback_func)
#endif /* GMW_RTIMER_SCHEDULE */

#ifndef GMW_RTIMER_NOW
#define GMW_RTIMER_NOW()                rtimer_ext_now_lf()
#endif /* GMW_RTIMER_NOW */

#ifndef GMW_RTIMER_STOP
#define GMW_RTIMER_STOP()               rtimer_ext_stop(GMW_CONF_RTIMER_ID)
#endif /* GMW_RTIMER_STOP */

#ifndef GMW_RTIMER_RESET
#define GMW_RTIMER_RESET()              rtimer_ext_reset()
#endif /* GMW_RTIMER_RESET */

#ifndef GMW_RTIMER_SECOND
#define GMW_RTIMER_SECOND               RTIMER_EXT_SECOND_LF
#endif /* GMW_RTIMER_SECOND */

/*---------------------------------------------------------------------------*/
/*--- RADIO MACROS ---*/
/*---------------------------------------------------------------------------*/

#ifndef GMW_SET_RF_CHANNEL
#define GMW_SET_RF_CHANNEL(channel)     gmw_set_rf_channel(channel)
#endif /* GMW_SET_RF_CHANNEL */

#endif /* GMW_PLATFORM_CONF_H_ */
