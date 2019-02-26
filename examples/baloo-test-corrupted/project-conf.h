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
 * Author:  Jonas Bächli
 * 			    Reto Da Forno
 *          Romain Jacob
 */
#ifndef PROJECT_CONFIG_H_
#define PROJECT_CONFIG_H_


/*
 * application specific config file to overwrite default settings
 */

/* --- definitions for FLOCKLAB --- */

/* to compile for flocklab, pass FLOCKLAB=1 to the make command */
#ifdef FLOCKLAB

  #include "../../tools/flocklab/flocklab.h"

  #define GLOSSY_START_PIN              FLOCKLAB_LED1
  #define GLOSSY_TX_PIN                 FLOCKLAB_INT1
  #define GLOSSY_RX_PIN                 FLOCKLAB_INT2
#endif /* FLOCKLAB */


/* --- PLATFORM dependent definitions --- */

#ifdef PLATFORM_SKY

/* cc2420 config, don't change! */
  #define CC2420_CONF_AUTOACK           0
  #define CC2420_CONF_ADDRDECODE        0
  #define CC2420_CONF_SFD_TIMESTAMPS    0
  /* CPU frequency, don't change! */
  #define F_CPU                         4194304UL

  /* radio configuration */
  #define RF_CONF_TX_CH                 26
  #define RF_CONF_TX_POWER              (int8_t) 0 // dBm

  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            ADC0
    #define GLOSSY_TX_PIN               ADC1
    #define GLOSSY_RX_PIN               ADC2
    #define GMW_NOISE_DETECT_PIN        ADC6
    #define GMW_GLOSSY_DETECT_PIN       ADC7
  #endif /* FLOCKLAB */

#elif defined PLATFORM_DPP_CC430

  /* radio configuration */
  #define RF_CONF_TX_CH                 3 /* approx. 868.6 MHz */
  #define RF_CONF_TX_POWER              RF1A_TX_POWER_0_dBm

  /* GPIO config */
  #ifndef FLOCKLAB
    #define GLOSSY_START_PIN            COM_GPIO1
    //#define GLOSSY_RX_PIN               COM_GPIO2
    //#define GLOSSY_TX_PIN               COM_GPIO3
    #define GMW_NOISE_DETECT_PIN        COM_GPIO2
    #define GMW_GLOSSY_DETECT_PIN       COM_GPIO3
  #endif /* FLOCKLAB */

#else
  #error "unknown target platform"
#endif


/* --- GENERAL definitions --- */

#define HOST_ID                         1

/* RF configuration */
#define RF_CONF_MAX_PKT_LEN				      128

/* GMW configuration */
#define GMW_CONF_TX_CNT_DATA            2
#define GMW_CONF_MAX_DATA_PKT_LEN       2
#define GMW_CONF_MAX_SLOTS              20
#define GMW_CONF_USE_STATIC_SCHED       0
#define GMW_CONF_USE_STATIC_CONFIG      0

/* Set the length of the contention slot to the min value required to send 2 bytes of payload */
#define GMW_CONF_T_CONT                 GMW_T_SLOT_MIN( 2 \
                                        + GMW_CONF_RF_OVERHEAD, \
                                        GMW_CONF_TX_CNT_DATA, \
                                        GMW_CONF_MAX_HOPS)

#define GMW_CONF_PERIOD_TIME_BASE		GMW_CONF_PERIOD_TIME_BASE_1ms
#define GMW_CONF_T_GAP_CONTROL          15000
#define GMW_CONF_T_GAP                  12000

#define GMW_CONF_USE_AUTOCLEAN          1
#define GMW_CONF_USE_NOISE_DETECTION    1
/* A reasonable threshold is 3dB above the expected
 * sensitivity of the radio... */
#define GMW_CONF_HIGH_NOISE_THRESHOLD   -60
#define GMW_CONF_HIGH_NOISE_MIN_COUNT   80

/* debug config */
#define DEBUG_PRINT_CONF_LEVEL          DEBUG_PRINT_LVL_INFO

#endif /* __CONFIG_H__ */
