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
 * @addtogroup  gmw
 * @{
 */
/*---------------------------------------------------------------------------*/


/**
 * \file
 *            This file contains most of the defines used by GMW
 *            (and their default value)
 */

#ifndef GMW_CONF_H_
#define GMW_CONF_H_

/*---------------------------------------------------------------------------*/
/*--- Packet sizes and slot configurations ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macro holding the maximal possible size of the _payload_ of a
 *            control packet.
 *            Used to determine the slot time of a control slot.
 *
 * @note      This is different from the size of the control struct, as the
 *            schedule and configuration information are not send when the
 *            static feature is used.
 */
// TODO: optimize (same style as the tests in gmw-control.c)
#ifndef GMW_CONF_MAX_CONTROL_PKT_LEN
  #if (GMW_CONF_USE_STATIC_SCHED && GMW_CONF_USE_STATIC_CONFIG)
    #define GMW_CONF_MAX_CONTROL_PKT_LEN      (GMW_CONF_CONTROL_USER_BYTES + GMW_CONF_USE_MAGIC_NUMBER)
  #else
    #define GMW_CONF_MAX_CONTROL_PKT_LEN      ((GMW_CONF_MAX_SLOTS * (2 + GMW_CONF_USE_CONTROL_SLOT_CONFIG)) + \
                                               (GMW_CONF_USE_CONTROL_SLOT_CONFIG * GMW_CONTROL_SLOT_CONFIG_TIMELIST_SIZE) + \
                                               GMW_CONF_CONTROL_USER_BYTES + \
                                               GMW_CONF_USE_MAGIC_NUMBER + \
                                               GMW_SCHED_SECTION_HEADER_LEN + \
                                               GMW_CONFIG_SECTION_HEADER_LEN)
  #endif
#endif /* GMW_CONF_MAX_CONTROL_PKT_LEN */

/**
 * @brief     Macro holding the maximal possible size of the _payload_ of a
 *            control or data packet.
 *
 * @note      Not directly configurable; max of
 *            GMW_CONF_MAX_CONTROL_PKT_LEN and
 *            GMW_CONF_MAX_DATA_PKT_LEN
 */
#define GMW_MAX_PKT_LEN                   MAX(GMW_CONF_MAX_CONTROL_PKT_LEN, \
                                              GMW_CONF_MAX_DATA_PKT_LEN)

/**
 * @brief     Max number of slots per round.
 *            CONF value is 10 by default.
 *
 * @note      Used to allocate memory for the control structure.
 */
//TODO: Is there a check whether I set n_slots > GMW_CONF_MAX_SLOTS? Would be nice to add...
#ifndef GMW_CONF_MAX_SLOTS
#define GMW_CONF_MAX_SLOTS                10
#endif /* GMW_CONF_MAX_SLOTS */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- Time scale configurations ---*/
/*---------------------------------------------------------------------------*/
/* time base defines for GMW_CONF_PERIOD_TIME_BASE */
#define GMW_CONF_PERIOD_TIME_BASE_1s      0
#define GMW_CONF_PERIOD_TIME_BASE_100ms   1
#define GMW_CONF_PERIOD_TIME_BASE_10ms    2
#define GMW_CONF_PERIOD_TIME_BASE_1ms     3

/**
 * @brief     Macro holding  the time base used for round time and period
 *
 *            Default value is GMW_CONF_PERIOD_TIME_BASE_1s
 *            (i.e., in the control struct, period and time are in second).
 */
#ifndef GMW_CONF_PERIOD_TIME_BASE
#define GMW_CONF_PERIOD_TIME_BASE         GMW_CONF_PERIOD_TIME_BASE_1s
#endif

#ifndef GMW_CONF_TIME_SCALE
  #if GMW_CONF_PERIOD_TIME_BASE == GMW_CONF_PERIOD_TIME_BASE_1s
   #define GMW_CONF_TIME_SCALE  1UL   /* use UL to prevent overflows */
  #elif GMW_CONF_PERIOD_TIME_BASE == GMW_CONF_PERIOD_TIME_BASE_100ms
   #define GMW_CONF_TIME_SCALE  10UL
  #elif GMW_CONF_PERIOD_TIME_BASE == GMW_CONF_PERIOD_TIME_BASE_10ms
   #define GMW_CONF_TIME_SCALE  100UL 
  #elif GMW_CONF_PERIOD_TIME_BASE == GMW_CONF_PERIOD_TIME_BASE_1ms
   #define GMW_CONF_TIME_SCALE  1000UL
  #else
   #error "invalid value for GMW_CONF_PERIOD_TIME_BASE"
  #endif
#else
  #warning "Using a custom GMW_CONF_TIME_SCALE value (you probably don't want to do that)"
#endif /* GMW_CONF_TIME_SCALE */

#if !GMW_CONF_TIME_SCALE
#error "invalid value for GMW_CONF_TIME_SCALE"
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- Slot, gap, guard, processing times ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief     Macro holding the maximal slot time of a control slot,
 *            in micro-seconds (us).
 *
 *            Default value is computed based on GMW_CONF_MAX_CONTROL_PKT_LEN.
 */
#ifndef GMW_CONF_T_CONTROL
#define GMW_CONF_T_CONTROL       GMW_T_SLOT_MIN(GMW_CONF_MAX_CONTROL_PKT_LEN + \
                                          GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_CONTROL, \
                                          GMW_CONF_MAX_HOPS)
#endif /* GMW_CONF_T_CONTROL */

/**
 * @brief     Macro holding the maximal slot time of a data slot,
 *            in micro-seconds (us).
 *
 *            Default value is computed based on GMW_CONF_MAX_DATA_PKT_LEN.
 */
#ifndef GMW_CONF_T_DATA
#define GMW_CONF_T_DATA          GMW_T_SLOT_MIN(GMW_CONF_MAX_DATA_PKT_LEN + \
                                          GMW_CONF_RF_OVERHEAD, \
                                          GMW_CONF_TX_CNT_DATA, \
                                          GMW_CONF_MAX_HOPS)
#endif /* GMW_CONF_T_DATA */

/**
 * @brief     Macro holding the maximal slot time of a contention slot,
 *            in micro-seconds (us).
 *
 *            Default value is set to 8ms.
 */
#ifndef GMW_CONF_T_CONT
#define GMW_CONF_T_CONT                   8000LU /* in us */
#endif /* GMW_CONF_T_CONT */

/**
 * @brief     Macros holding slot times available when using per-slot config,
 *            in micro-seconds (us).
 *
 *            Default value is set equal to GMW_CONF_T_DATA for all.
 */
#if GMW_CONF_USE_CONTROL_SLOT_CONFIG
  #ifndef   GMW_SLOT_TIME_0
    #define GMW_SLOT_TIME_0               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_0 */
  #ifndef   GMW_SLOT_TIME_1
    #define GMW_SLOT_TIME_1               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_1 */
  #ifndef   GMW_SLOT_TIME_2
    #define GMW_SLOT_TIME_2               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_2 */
  #ifndef   GMW_SLOT_TIME_3
    #define GMW_SLOT_TIME_3               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_3 */
  #ifndef   GMW_SLOT_TIME_4
    #define GMW_SLOT_TIME_4               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_4 */
  #ifndef   GMW_SLOT_TIME_5
    #define GMW_SLOT_TIME_5               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_5 */
  #ifndef   GMW_SLOT_TIME_6
    #define GMW_SLOT_TIME_6               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_6 */
  #ifndef   GMW_SLOT_TIME_7
    #define GMW_SLOT_TIME_7               GMW_CONF_T_DATA
  #endif /* GMW_SLOT_TIME_7 */
#endif

/**
 * @brief     Macro holding the gap between two consecutive slots,
 *            in micro-seconds (us).
 *
 *            Default value is set to 4ms.
 *
 * @note      It must be large enough to finish all computations between slots
 *            (i.e., post-slot and pre-slot callback) plus the guard time.
 */
#ifndef GMW_CONF_T_GAP
#define GMW_CONF_T_GAP                    4000UL /* in us */
#endif /* GMW_CONF_T_GAP */

/**
 * @brief     Macro holding the gap between the control and the first data slot,
 *            in micro-seconds (us).
 *
 *            Default value is set equal to GMW_CONF_T_GAP.
 *
 * @note      The control-post callback might require more processing time,
 *            thus we let this gap time being customizable if necessary.
 */
#ifndef GMW_CONF_T_GAP_CONTROL
#define  GMW_CONF_T_GAP_CONTROL           GMW_CONF_T_GAP
#endif /* GMW_CONF_T_GAP_CONTROL */

/**
 * @brief     Macro holding the guard time, in micro-seconds (us).
 *
 *            Default value is set to 0.5ms.
 *
 * @note      The guard defines how long in advance a node start listening
 *            _before_ the expected start of a packet. Used to compensate for
 *            small clock drifts.
 */
#ifndef GMW_CONF_T_GUARD_SLOT
#define GMW_CONF_T_GUARD_SLOT             500LU /* in us */
#endif /* GMW_CONF_T_GUARD_SLOT */

/**
 * @brief     Macro holding the guard time for the control packet,
 *            in micro-seconds (us).
 *
 *            Default value is set equal to GMW_CONF_T_GUARD_SLOT.
 *
 * @note      If the time between rounds is large, the clock drift has larger
 *            impact and one may want a larger guard to compensate for it.
 */
#ifndef GMW_CONF_T_GUARD_ROUND
#define GMW_CONF_T_GUARD_ROUND            GMW_CONF_T_GUARD_SLOT
#endif /* GMW_CONF_T_GUARD_ROUND */

/**
 * @brief     The time reserved for the execution of the pre-process running
 *            before a GMW round. Time in milliseconds (ms).
 *
 *            Default value is set to 0.
 *
 * @note      If set to 0, the pre-processing is disabled.
 */
#ifndef GMW_CONF_T_PREPROCESS
#define GMW_CONF_T_PREPROCESS             0 /* in ms */
#endif /* GMW_CONF_T_PREPROCESS */

/**
 * @brief     Minimal time that should be available in between two GMW rounds
 *            for post-processing. Time in milliseconds (ms).
 *
 *            Default value is set to 0.
 *
 * @note      When the length of a GMW rounds vary (e.g., due to the use of the
 *            slot/round repeat feature), GMW_CONF_T_POSTPROCESS_MIN is used
 *            interrupt a round that would otherwise overruns into the next
 *            round.
 */
//TODO: document this better, not sure about where the 'warning' actually is...
#ifndef GMW_CONF_T_POSTPROCESS_MIN
#define GMW_CONF_T_POSTPROCESS_MIN        0 /* in ms */
#endif /* GMW_CONF_T_POSTPROCESS_MIN */


/**
 * @brief     Slack time required by the GMW host to put the control information
 *            into the packet buffer before the control slot starts,
 *            in microseconds (us).
 *
 *            Set to 2000 us by default.
 *
 * @note      This value is large enough to memcpy data as large as conventional
 *            radio buffers (i.e., less than 255 bytes). Leave this parameter
 *            to the default value unless trying to minimize latency.
 */
#ifndef GMW_CONF_PREROUND_SETUP_TIME
#define GMW_CONF_PREROUND_SETUP_TIME      2000UL /* in us */
#endif /* GMW_CONF_PREROUND_SETUP_TIME */

/**
 * @brief     Constant time offset that is subtracted from the time reference
 *            when nodes are re-synchronized after receiving a control packet.
 *
 * @note      It accounts for the time difference the start time of initiators
 *            and receivers in a Glossy flood.
 */
#ifndef GMW_CONF_T_REF_OFS
#warning "GMW_CONF_T_REF_OFS is platform dependent!"
#define GMW_CONF_T_REF_OFS                (GMW_RTIMER_SECOND / 1000) /* ticks */
#endif /* GMW_CONF_T_REF_OFS */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- Generic primitive configuration ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief     Number of retransmissions used for sending the control packet.
 *
 *            Default value is set to 4.
 *
 * @note      The control packet is always sent using Glossy. The number of
 *            retransmissions in Glossy provides a direct tradeoff between
 *            reliability and energy consumption.
 *
 * @note      This macro is used for the default setting of GMW_CONF_T_CONTROL.
 *
 * @note      As Baloo relies quite strongly on the successful reception of the
 *            control packet, a high value for GMW_CONF_TX_CNT_CONTROL (>= 4)
 *            is recommended.
 */
#ifndef GMW_CONF_TX_CNT_CONTROL
#define GMW_CONF_TX_CNT_CONTROL           4
#endif /* GMW_CONF_TX_CNT_CONTROL */

/**
 * @brief     Number of retransmissions used for sending the data packet.
 *
 *            Default value is set to 3.
 *
 * @note      This macro is used during the initialization of the control
 *            (see config_init() in gmw-control.c) and for the default
 *            setting of GMW_CONF_T_DATA.
 */
#ifndef GMW_CONF_TX_CNT_DATA
#define GMW_CONF_TX_CNT_DATA              3
#endif

/**
 * @brief     Maximal (expected) number of hops in the network between
 *            any two nodes.
 *
 *            Default value is set to 5.
 *
 * @note      This macro is typically used to compute GMW_T_SLOT_MIN,
 *            i.e., the required slot time for a synchronous transmission
 *            primitive to complete its execution (when an analytic formula
 *            is available, for example when using Glossy).
 */
#ifndef GMW_CONF_MAX_HOPS
#define GMW_CONF_MAX_HOPS                 5
#endif /* GMW_CONF_MAX_HOPS */

/**
 * @brief     Value corresponding to undefined relay counter.
 */
#define GMW_RELAY_COUNT_UNDEF             255
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*--- Additional features configuration ---*/
/*---------------------------------------------------------------------------*/
/**
 * @brief     Number of user-defined bytes in a control packet. User-bytes are
 *            shared (i.e., independent of the number of slots).
 *
 *            CONF value is 0 by default.
 */
#ifndef GMW_CONF_CONTROL_USER_BYTES
#define GMW_CONF_CONTROL_USER_BYTES        0
#endif

/**
 * @brief     Enable/disable the use of per-slot configuration.
 *
 * @note      CONF disabled by default.
 * @note      Per-slot configuration size (in bytes) is controlled by
 *            GMW_CONTROL_SLOT_CONFIG_TYPE_SIZE, with default value of 1
 *            (see gme-types.h).
 */
#ifndef GMW_CONF_USE_CONTROL_SLOT_CONFIG
#define GMW_CONF_USE_CONTROL_SLOT_CONFIG  0
#endif /* GMW_CONF_USE_CONTROL_SLOT_CONFIG */

/**
 * @brief     A magic number can be used by the middleware to identify a control
 *            packet, to avoid to mistakenly update t_ref based on a wrong
 *            packet. This is problem particularly sensitive when the static
 *            sched and config feature of the middleware is in use.
 *            Then, a node only requires to receive a sync Glossy packet to
 *            bootstrap (... possibly synchronizing with a node which is not
 *            the GMW host).
 *
 * @note      In the Glossy header, there is already a 2-bit magic number
 *            (GLOSSY_COMMON_HEADER). In most cases, this is sufficient to
 *            differentiate between different Glossy networks...
 *
 * @note      CONF disabled by default to save radio-on time.
 */
#ifndef GMW_CONF_USE_MAGIC_NUMBER
#define GMW_CONF_USE_MAGIC_NUMBER         0
#endif /*GMW_CONF_USE_MAGIC_NUMBER*/

#ifndef GMW_CONF_CONTROL_MAGIC_NUMBER
#define GMW_CONF_CONTROL_MAGIC_NUMBER     33
#endif /*GMW_CONF_CONTROL_MAGIC_NUMBER*/

/**
 * @brief     Enable/disable the use of the static schedule.
 *
 *            CONF disabled by default.
 *
 * @note      GMW_CONF_USE_STATIC_SCHED and GMW_CONF_USE_STATIC_CONFIG should
 *            be used together (both enabled or disabled). Should work with
 *            only one enabled, but not tested extensively.
 */
#ifndef GMW_CONF_USE_STATIC_SCHED
#define GMW_CONF_USE_STATIC_SCHED         0
#endif /*GMW_CONF_USE_STATIC_SCHED*/

/**
 * @brief     Enable/disable the use of the static configuration.
 *
 *            CONF disabled by default.
 *
 * @note      GMW_CONF_USE_STATIC_SCHED and GMW_CONF_USE_STATIC_CONFIG should
 *            be used together (both enabled or disabled). Should work with
 *            only one enabled, but not tested extensively.
 *
 * @note      If GMW_CONF_USE_STATIC_CONFIG is enabled, the slot-config
 *            (if any) is also considered static.
 */
#ifndef GMW_CONF_USE_STATIC_CONFIG
#define GMW_CONF_USE_STATIC_CONFIG        0
#endif /*GMW_CONF_USE_STATIC_CONFIG*/

/**
 * @brief     Enable/disable the use of multiple synchronous transmission
 *            primitives.
 *
 * @note      CONF disabled by default. Only Glossy is used.
 */
#ifndef GMW_CONF_USE_MULTI_PRIMITIVES
#define GMW_CONF_USE_MULTI_PRIMITIVES     0
#endif /* GMW_CONF_USE_MULTI_PRIMITIVES */

/**
 * @brief     Enable/disable the use of clock drift compensation.
 *
 * @note      CONF enabled by default.
 * @note      Implemented drift compensation is quite basic, but already
 *            works quite well.
 */
//TODO: document the clock-drift compensation and add info about reasonable max time between rounds (not to get off sync)
#ifndef GMW_CONF_USE_DRIFT_COMPENSATION
#define GMW_CONF_USE_DRIFT_COMPENSATION   1
#endif /* GMW_CONF_USE_DRIFT_COMPENSATION */

/**
 * @brief     Maximal clock deviation considered by the clock drift
 *            compensation algorithm, in part per million (ppm).
 *
 *            Default value is set to 150.
 */
#ifndef GMW_CONF_MAX_CLOCK_DEV
#define GMW_CONF_MAX_CLOCK_DEV            150
#endif /* GMW_CONF_MAX_CLOCK_DEV */

/**
 * @brief     Enable/disable the autoclean feature.
 *            When enabled, the packet buffer (gmw_payload) is memset to 0 after
 *            after each slot.
 *
 * @note      CONF disabled by default.
 */
#ifndef GMW_CONF_USE_AUTOCLEAN
#define GMW_CONF_USE_AUTOCLEAN            0
#endif /* GMW_CONF_USE_AUTOCLEAN */

/**
 * @brief     Enable/disable the use of the noise detection feature.
 *
 *            CONF disabled by default.
 *
 * @note      When the noise detection is enabled, the middleware runs an
 *            independent process during the execution of the synchronous
 *            transmission primitives. This process periodically samples the
 *            power on the wireless channel, and outputs a 'high noise' signal
 *            based on the GMW_CONF_HIGH_NOISE_THRESHOLD and the
 *            GMW_CONF_HIGH_NOISE_MIN_COUNT configuration parameters.
 */
#ifndef GMW_CONF_USE_NOISE_DETECTION
#define GMW_CONF_USE_NOISE_DETECTION      0
#endif /* GMW_CONF_USE_NOISE_DETECTION */

#if GMW_CONF_USE_NOISE_DETECTION

/**
 * @brief     Threshold for the noise detection feature,
 *            in dBm.
 *
 *            Must be user-defined (e.g., in project-conf.h)
 *
 * @note      A reasonable threshold is 3dB above the expected sensitivity
 *            of the radio. To reduce false positives, set to a higher value.
 */
#ifndef GMW_CONF_HIGH_NOISE_THRESHOLD
#warning "If GMW is configured to detect high noise, the threshold must be defined in the 'project-conf' file. Set to 0 dBm"
#define GMW_CONF_HIGH_NOISE_THRESHOLD     0
#endif /* GMW_CONF_HIGH_NOISE_THRESHOLD */

/**
 * @brief     Number of times the threshold must be exceeded to consider
 *            there is 'high noise' on the wireless channel.
 *
 *            Default value is set to 10.
 */
#ifndef GMW_CONF_HIGH_NOISE_MIN_COUNT
  #define GMW_CONF_HIGH_NOISE_MIN_COUNT   10
#endif /* GMW_CONF_HIGH_NOISE_MIN_COUNT */

#endif /* GMW_CONF_USE_NOISE_DETECTION */

/*---------------------------------------------------------------------------*/
/* the overhead (in Bytes) introduced by the primitives and the radio */
#ifndef GMW_CONF_RF_OVERHEAD
#error "GMW_CONF_RF_OVERHEAD is required!"
#endif
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/* some helpers and static defines that should not be changed! */

/* the time base for the slot time field in the config is 500 us */
#ifndef GMW_CONF_SLOT_TIME_BASE
#define GMW_CONF_SLOT_TIME_BASE           500UL /* in ms */
#else
#error "GMW_CONF_SLOT_TIME_BASE already defined!"
#endif

/* the time base for the slot time field in clock ticks */
#ifndef GMW_CONF_SLOT_TIME_BASE_CLOCK
#define GMW_CONF_SLOT_TIME_BASE_CLOCK     GMW_CONF_SLOT_TIME_BASE * \
                                          GMW_RTIMER_SECOND / (1000000UL)
#else
#error "GMW_CONF_SLOT_TIME_BASE_CLOCK already defined!"
#endif

/* the time base for the gap time field in the config is 100 us */
#ifndef GMW_CONF_GAP_TIME_BASE
#define GMW_CONF_GAP_TIME_BASE            100UL /* in us */
#else
#error "GMW_CONF_GAP_TIME_BASE already defined!"
#endif

/* the time base for the gap time field in clock ticks */
#ifndef GMW_CONF_GAP_TIME_BASE_CLOCK
#define GMW_CONF_GAP_TIME_BASE_CLOCK      GMW_CONF_GAP_TIME_BASE * \
                                          GMW_RTIMER_SECOND / (1000000UL)
#else
#error "GMW_CONF_GAP_TIME_BASE_CLOCK already defined!"
#endif
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

/** @} */

#endif /* GMW_CONF_H_ */
